#include <plan_manage/ego_replan_fsm.h>

#include <chrono>
#include <functional>
#include <iomanip>
#include <sstream>
namespace ego_planner {
void EGOReplanFSM::init(rclcpp::Node::SharedPtr node) {
  node_ = node;
  vis_ = std::make_shared<PlanningVisualization>(node_);
  manager_ = std::make_unique<EGOPlannerManager>();
  manager_->initPlanModules(node_, vis_);
  execution_limits_ = {
      node_->declare_parameter("execution/max_vel", 1.0),
      node_->declare_parameter("execution/max_acc", 1.0),
      node_->declare_parameter("execution/max_jer", 3.0)};
  for (double limit : execution_limits_)
    if (!std::isfinite(limit) || limit <= 0)
      throw std::invalid_argument("Execution limits must be finite and positive");
  use_route_seed_ = node_->declare_parameter("fsm/use_route_seed", false);
  adjust_local_target_ = node_->declare_parameter("fsm/adjust_local_target", use_route_seed_);
  executor_timeout_ = node_->declare_parameter("fsm/executor_timeout", 0.3);
  if (!std::isfinite(executor_timeout_) || executor_timeout_ < .1 || executor_timeout_ > 1.)
    throw std::invalid_argument("Invalid executor acknowledgement timeout");
  allow_unknown_ = node_->declare_parameter("fsm/allow_unknown", false);
  RCLCPP_INFO(node_->get_logger(), "Unknown-space policy: %s", allow_unknown_ ? "optimistic" : "known-free-only");
  replan_interval_ = node_->declare_parameter("fsm/replan_interval", 1.0);
  planning_budget_ = node_->declare_parameter("fsm/planning_budget", 0.18);
  planning_lead_time_ = node_->declare_parameter("fsm/planning_lead_time", 0.25);
  if (!std::isfinite(replan_interval_) || !std::isfinite(planning_budget_) ||
      !std::isfinite(planning_lead_time_) || replan_interval_ <= 0 || planning_budget_ <= 0 ||
      planning_budget_ > .20 || planning_lead_time_ < planning_budget_ + .025 ||
      planning_lead_time_ > 1.)
    throw std::invalid_argument("Invalid planning timing");
  runtime_pub_ = node_->create_publisher<std_msgs::msg::String>("/planning/runtime_diagnostics", 100);
  worker_manager_ = manager_->forkForPlanning();
  diagnostics_pub_ = node_->create_publisher<std_msgs::msg::String>("/planning/diagnostics", rclcpp::QoS(100).transient_local());
  traj_pub_ = node_->create_publisher<traj_utils::msg::PolyTraj>("/planning/trajectory", 10);
  status_pub_ = node_->create_publisher<uav_interfaces::msg::NavigationStatus>(
      "/navigation/status", rclcpp::QoS(1).transient_local());
  safety_pub_ = node_->create_publisher<uav_interfaces::msg::PlanSafety>("/planning/safety", 10);
  enable_client_ = node_->create_client<std_srvs::srv::SetBool>("/navigation/control_enable");
  staged_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/navigation/staged_goals",
                                                             rclcpp::QoS(1).transient_local());
  goal_service_ = node_->create_service<uav_interfaces::srv::SetGoal>(
      "/navigation/set_goal", [this](const std::shared_ptr<uav_interfaces::srv::SetGoal::Request> q,
                                     std::shared_ptr<uav_interfaces::srv::SetGoal::Response> r) {
        Eigen::Vector3d goal(q->goal.x, q->goal.y, q->goal.z);
        if (!goal.allFinite()) {
          r->message = "目标坐标无效：含 NaN 或 Inf";
          return;
        }
        if (!healthy()) {
          r->message = "定位或局部地图无效：" + healthReport();
          RCLCPP_WARN(node_->get_logger(), "%s", r->message.c_str());
          return;
        }
        if (phase_ == "EXECUTING" || phase_ == "BRAKING" || phase_ == "WAIT_CONTROL") {
          r->message = "请先暂停或取消正在执行的任务";
          return;
        }
        stageGoals({goal});
        if (!q->preview) {
          r->success = true;
          r->message = "目标已暂存，点击开始导航执行";
          return;
        }
        if ((goal - position_).norm() < 0.1) {
          r->message = "目标距离当前位置过近";
          return;
        }
        if (velocity_.norm() > 0.15) {
          r->message = "请先悬停再预览";
          return;
        }
        r->success = plan(true, &goal);
        r->message = r->success ? "局部规划预览通过；未执行飞行，点击开始导航"
                                : "局部预览未通过；检查占据/未知空间，必要时先起飞悬停";
        if (!r->success) r->message += std::string(" (") + failureName(last_failure_) + ")";
        reason_ = r->message;
        status();
      });
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/navigation/odometry", 1, [this](nav_msgs::msg::Odometry::SharedPtr m) {
        const auto& p = m->pose.pose.position;
        const auto& q = m->pose.pose.orientation;
        const auto& v = m->twist.twist.linear;
        position_ = {p.x, p.y, p.z};
        Eigen::Quaterniond rot(q.w, q.x, q.y, q.z);
        if (rot.norm() < 1e-6) return;
        velocity_ = rot.normalized() * Eigen::Vector3d(v.x, v.y, v.z);
        odom_stamp_ = rclcpp::Time(m->header.stamp).seconds();
      });
  goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/navigation/goal", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
        Eigen::Vector3d p(m->pose.position.x, m->pose.position.y, m->pose.position.z);
        if (m->header.frame_id != "odom" || !p.allFinite()) {
          reason_ = "Goal must be finite and in odom";
          return;
        }
        stageGoals({p});
        reason_ = "Goal staged; start explicitly";
        status();
      });
  rviz_goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/navigation/rviz_goal", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
        Eigen::Vector3d p(m->pose.position.x, m->pose.position.y, position_.z());
        if (m->header.frame_id != "odom" || !p.allFinite() || !healthy()) return;
        stageGoals({p});
        reason_ = "RViz XY goal staged at current altitude";
        status();
      });
  waypoints_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
      "/navigation/waypoints", 10, [this](nav_msgs::msg::Path::SharedPtr m) {
        if (m->header.frame_id != "odom" || m->poses.empty()) return;
        std::vector<Eigen::Vector3d> temp;
        for (const auto& x : m->poses) {
          Eigen::Vector3d p(x.pose.position.x, x.pose.position.y, x.pose.position.z);
          if (!p.allFinite() || (!x.header.frame_id.empty() && x.header.frame_id != "odom")) return;
          temp.push_back(p);
        }
        stageGoals(temp);
        reason_ = "Waypoints staged";
        status();
      });
  health_sub_ = node_->create_subscription<uav_interfaces::msg::NavigationHealth>(
      "/navigation/health", 10, [this](uav_interfaces::msg::NavigationHealth::SharedPtr h) {
        if (epoch_ && epoch_ != h->epoch) stop("Localization epoch changed");
        epoch_ = h->epoch;
        odom_ok_ = h->odometry_valid;
        cloud_ok_ = h->cloud_valid;
      });
  control_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/navigation/control_status", 10,
      [this](std_msgs::msg::String::SharedPtr s) { control_ = s->data; });
  execution_sub_ = node_->create_subscription<uav_interfaces::msg::TrajectoryExecution>(
      "/planning/execution_status", 1,
      [this](uav_interfaces::msg::TrajectoryExecution::SharedPtr s) {
        if (s->session != session_) return;
        if (execution_ && rclcpp::Time(s->header.stamp) < rclcpp::Time(execution_->header.stamp))
          return;
        execution_ = s;
      });
  auto service = [this](const std::string& name, auto callback) {
    services_.push_back(node_->create_service<Trigger>(name, callback));
  };
  service("/navigation/check_planner_health", [this](Trigger::Request::SharedPtr,
                                                    Trigger::Response::SharedPtr r) {
    r->success = healthy();
    r->message = healthReport();
  });
  service("/navigation/start", [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
    if (!healthy() || staged_.empty() || !enable_client_->service_is_ready() ||
        (control_ != "AVAILABLE" && control_ != "ACTIVE")) {
      r->success = false;
      r->message =
          "Need fresh sensors, staged goals and armed airborne PX4 in "
          "hold/position mode";
      return;
    }
    // No implicit replacement of an executing task; the operator pauses/cancels
    // first.
    if (phase_ == "EXECUTING" || phase_ == "BRAKING" || phase_ == "WAIT_CONTROL") {
      r->success = false;
      r->message = "Pause or cancel before replacing the task";
      return;
    }
    for (const auto& goal : staged_)
      if (!manager_->grid_map_->isCollisionFree(goal)) {
        r->success = false;
        r->message = "Staged waypoint is occupied or outside vertical flight bounds";
        return;
      }
    retry_after_brake_ = false;
    brake_retries_ = 0;
    obstacle_retry_at_=0;
    session_++;
    session_started_at_ = node_->now().seconds();
    execution_.reset();
    handover_pending_ = cancel_pending_ = false;
    active_ = false;
    previous_id_ = 0;
    reference_valid_ = false;
    local_target_final_ = false;
    goals_ = staged_;
    waypoint_ = 0;
    settled_ = 0;
    pause_requested_ = false;
    phase_ = "WAIT_CONTROL";
    reason_ = "Waiting for explicit PX4 Offboard acknowledgement";
    enable(true);
    status();
    r->success = true;
    r->message = reason_;
  });
  service("/navigation/pause", [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
    r->success = phase_ == "EXECUTING" || phase_ == "BRAKING";
    if (r->success) {
      retry_after_brake_ = false;
      pause_requested_ = phase_ == "EXECUTING";
      reason_ = "Braking requested";
    }
    r->message = reason_;
  });
  service("/navigation/resume",
          [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
            r->success = phase_ == "PAUSED" && waypoint_ < goals_.size() && healthy() &&
                         control_ == "ACTIVE";
            if (r->success) {
              phase_ = "EXECUTING";
              reason_ = "Navigation resumed";
              retry_after_brake_ = false;
              brake_retries_ = 0;
              pause_requested_ = false;
              // 保留已停稳轨迹作为连续拼接起点，重新规划剩余航段。
              reference_valid_ = false;
              local_target_final_ = false;
              settled_ = 0;
              obstacle_retry_at_ = 0;
              last_plan_ = 0;
            }
            r->message = phase_;
          });
  service("/navigation/cancel",
          [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
            stop("Cancelled by operator");
            phase_ = "CANCELLED";
            status();
            r->success = true;
            r->message = "Navigation released to PX4 position mode";
          });
  timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), [this] { tick(); });
  duration_pub_ = node_->create_publisher<std_msgs::msg::Float64>("/planning/duration_ms", 10);
  planning_poll_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10), [this] { pollPlanning(); });
  status();
}
void EGOReplanFSM::stageGoals(const std::vector<Eigen::Vector3d>& points) {
  staged_ = points;
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  path.header.stamp = node_->now();
  for (const auto& point : points) {
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    p.pose.orientation.w = 1;
    p.pose.position.x = point.x();
    p.pose.position.y = point.y();
    p.pose.position.z = point.z();
    path.poses.push_back(p);
  }
  staged_pub_->publish(path);
  if (!points.empty())
    vis_->displayGoalPoint(points.back(), Eigen::Vector4d(0.85, 0.4, 1, 1), 0.25, 0);
}

bool EGOReplanFSM::healthy() const {
  double age = node_->now().seconds() - odom_stamp_;
  return odom_ok_ && cloud_ok_ && position_.allFinite() && velocity_.allFinite() && age >= -0.05 &&
         age < 0.3 && manager_->grid_map_->freshCloud();
}
std::string EGOReplanFSM::healthReport() const {
  std::ostringstream out;
  out << "odom_valid=" << odom_ok_ << ", cloud_valid=" << cloud_ok_
      << ", odom_age=" << node_->now().seconds() - odom_stamp_ << "s"
      << ", position_finite=" << position_.allFinite()
      << ", velocity_finite=" << velocity_.allFinite()
      << ", " << manager_->grid_map_->streamHealth();
  return out.str();
}
void EGOReplanFSM::enable(bool on) {
  if (!enable_client_->service_is_ready()) return;
  auto q = std::make_shared<std_srvs::srv::SetBool::Request>();
  q->data = on;
  enable_client_->async_send_request(
      q, [this, on](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture f) {
        if (on && !f.get()->success) {
          phase_ = "FAULT";
          reason_ = f.get()->message;
          active_ = false;
          status();
        }
      });
}
void EGOReplanFSM::stop(const std::string& why) {
  ++planning_generation_;
  RCLCPP_WARN(node_->get_logger(), "Navigation stopped: %s", why.c_str());
  active_ = false;
  handover_pending_ = cancel_pending_ = false;
  vis_->displayAcceptedPath({});
  vis_->clearActiveGoal();
  phase_ = "FAULT";
  reason_ = why;
  enable(false);
  status();
}
void EGOReplanFSM::status() {
  uav_interfaces::msg::NavigationStatus s;
  s.header.stamp = node_->now();
  s.header.frame_id = "odom";
  s.phase = phase_;
  s.reason = reason_;
  s.session = session_;
  s.waypoint_index = waypoint_;
  s.waypoint_count = goals_.size();
  s.control_requested = phase_ == "WAIT_CONTROL" || phase_ == "EXECUTING" || phase_ == "BRAKING" ||
                        phase_ == "PAUSED" || phase_ == "COMPLETE";
  status_pub_->publish(s);
}
PlanFailure EGOReplanFSM::segmentFailure(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const {
  if (!manager_->grid_map_->isCollisionFreeSegment(a, b)) return PlanFailure::COLLISION;
  if (!allow_unknown_ && !manager_->grid_map_->isKnownFreeSegment(a, b)) return PlanFailure::UNKNOWN;
  return PlanFailure::NONE;
}
bool EGOReplanFSM::traversable(const Eigen::Vector3d& p) const {
  return segmentFailure(p, p) == PlanFailure::NONE;
}
bool EGOReplanFSM::traversableSegment(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const {
  return segmentFailure(a, b) == PlanFailure::NONE;
}
PlanFailure EGOReplanFSM::validate(const poly_traj::Trajectory& t) const {
  if (t.getPieceNum() < 1 || !std::isfinite(t.getTotalDuration()) || t.getTotalDuration() <= 0 ||
      t.getTotalDuration() > 120)
    return PlanFailure::INVALID;
  for (double x = 0; x <= t.getTotalDuration(); x += .025) {
    const auto p = t.getPos(x);
    if (!p.allFinite()) return PlanFailure::INVALID;
    const auto failure = segmentFailure(t.getPos(std::max(0., x - .025)), p);
    if (failure != PlanFailure::NONE) {
      RCLCPP_INFO(node_->get_logger(), "Trajectory blocked at %.3f/%.3f: (%.4f %.4f %.4f) -> (%.4f %.4f %.4f)",
          x,t.getTotalDuration(),t.getPos(std::max(0.,x-.025)).x(),t.getPos(std::max(0.,x-.025)).y(),t.getPos(std::max(0.,x-.025)).z(),p.x(),p.y(),p.z());
      RCLCPP_INFO(node_->get_logger(), "Blocked map cell (%s): %s",
                  planning_worker_ ? "snapshot" : "live",
                  manager_->grid_map_->describeObstruction(t.getPos(std::max(0., x - .025)), p, allow_unknown_).c_str());
      return failure;
    }
  }
  const auto tail_failure = segmentFailure(t.getPos(std::max(0., t.getTotalDuration() - .025)),
                                          t.getPos(t.getTotalDuration()));
  if (tail_failure != PlanFailure::NONE) return tail_failure;
  std::array<double, 3> rates;
  try {
    rates = peakRates(t);
  } catch (const std::exception&) {
    return PlanFailure::INVALID;
  }
  if (!std::isfinite(rates[0] + rates[1] + rates[2])) return PlanFailure::INVALID;
  for (int i = 0; i < 3; ++i)
    if (rates[i] > execution_limits_[i] * 1.1 - .001) return PlanFailure::DYNAMIC;
  return PlanFailure::NONE;
}
void EGOReplanFSM::publish(const poly_traj::Trajectory& t, double start,
                          const GlobalTrajData* rollback_global, bool rollback_valid) {
  if (planning_worker_) {
    active_traj_ = t;
    active_start_ = start;
    return;
  }
  if (handover_pending_) throw std::logic_error("Unresolved trajectory handover");
  previous_id_ = 0;
  if (active_) {
    previous_traj_ = active_traj_;
    previous_start_ = active_start_;
    previous_id_ = trajectory_id_;
    previous_global_ = rollback_global ? *rollback_global : manager_->traj_.global_traj;
    previous_reference_valid_ = rollback_global ? rollback_valid : reference_valid_;
  }
  active_traj_ = t;
  active_start_ = start;
  active_ = true;
  trajectory_id_ = ++issued_id_;
  handover_pending_ = true;
  cancel_pending_ = false;
  traj_utils::msg::PolyTraj m;
  m.drone_id = 0;
  m.traj_id = trajectory_id_;
  m.order = 5;
  m.session = session_;
  m.start_time =
      rclcpp::Time(static_cast<int64_t>(start * 1e9), node_->get_clock()->get_clock_type());
  m.valid_duration = t.getTotalDuration();
  for (int i = 0; i < t.getPieceNum(); i++) {
    m.duration.push_back(t[i].getDuration());
    const auto c = t[i].getCoeffMat();
    for (int j = 0; j < 6; j++) {
      m.coef_x.push_back(c(0, j));
      m.coef_y.push_back(c(1, j));
      m.coef_z.push_back(c(2, j));
    }
  }
  status();
  traj_pub_->publish(m);
  std::vector<Eigen::Vector3d> accepted;
  for (int i = 0; i <= 100; ++i) accepted.push_back(t.getPos(t.getTotalDuration() * i / 100.0));
  vis_->displayAcceptedPath(accepted);
}

bool EGOReplanFSM::reconcileExecution(double now) {
  const double stamp = execution_ ? rclcpp::Time(execution_->header.stamp).seconds() : 0.;
  if (!execution_ || execution_->session != session_ || now - stamp < -.05 ||
      now - stamp > executor_timeout_ ||
      node_->count_publishers("/planning/execution_status") != 1) {
    if (!active_ && !handover_pending_ && now - session_started_at_ <= executor_timeout_) {
      reason_ = "Waiting for executor to acknowledge the new session";
      return false;
    }
    stop("Trajectory executor acknowledgement stale/missing");
    return false;
  }
  if (handover_pending_) {
    if ((execution_->current_traj_id != trajectory_id_ &&
         execution_->current_traj_id != previous_id_) ||
        (execution_->pending_traj_id != 0 && execution_->pending_traj_id != trajectory_id_)) {
      stop("Trajectory executor reference disagrees during handover");
      return false;
    }
    if (execution_->current_traj_id == trajectory_id_) {
      // 执行器已实际接管：即使撤销请求同时在途，也只能从新轨迹继续制动。
      handover_pending_ = cancel_pending_ = false;
      previous_id_ = 0;
    } else if (execution_->retired_traj_id >= trajectory_id_ &&
               execution_->pending_traj_id != trajectory_id_ &&
               execution_->current_traj_id == previous_id_) {
      // 未接管候选被明确撤销/拒收，才可恢复仍在执行的旧参考。
      active_ = previous_id_ != 0;
      if (active_) {
        active_traj_ = previous_traj_;
        active_start_ = previous_start_;
        trajectory_id_ = previous_id_;
        manager_->traj_.global_traj = previous_global_;
      }
      previous_id_ = 0;
      handover_pending_ = cancel_pending_ = false;
      // 连同全局参考及其进度回滚，避免用撤销候选的游标或重新生成运动中的参考。
      reference_valid_ = active_ && previous_reference_valid_;
      local_target_final_ = false;
      last_plan_ = obstacle_retry_at_ = settled_ = 0.;
      if (phase_ == "BRAKING") pause_requested_ = !retry_after_brake_;
      phase_ = waypoint_ < goals_.size() ? "EXECUTING" : "COMPLETE";
      reason_ = "Pending trajectory retired; returning to confirmed reference";
    } else if (now > active_start_ + executor_timeout_) {
      stop("Trajectory activation acknowledgement timed out");
      return false;
    }
  } else if (active_ && execution_->current_traj_id != trajectory_id_) {
    stop("Trajectory executor reference disagrees with planner");
    return false;
  }
  return true;
}

double EGOReplanFSM::firstCollision(const poly_traj::Trajectory& traj, double start,
                                    double until) const {
  const double begin = std::clamp(node_->now().seconds() - start, 0., traj.getTotalDuration());
  const double end = std::clamp(until - start, begin, traj.getTotalDuration());
  if (!traversable(traj.getPos(begin))) return start + begin;
  for (double t = begin; t < end;) {
    const double next = std::min(t + .025, end);
    if (!traversableSegment(traj.getPos(t), traj.getPos(next))) return start + t;
    t = next;
  }
  return std::numeric_limits<double>::infinity();
}

void EGOReplanFSM::safetyLease(int id, const poly_traj::Trajectory& traj, double start,
                               bool revoked) {
  const double now = node_->now().seconds();
  double until = now + .25;
  const bool moving_end = traj.getVel(traj.getTotalDuration()).norm() > .05;
  if (moving_end)
    until = std::min(until, start + traj.getTotalDuration());
  const double collision = firstCollision(traj, start, until);
  until = std::min(until, collision);
  uav_interfaces::msg::PlanSafety s;
  s.header.stamp = node_->now();
  s.session = session_;
  s.traj_id = id;
  s.valid = !revoked && until > now;
  s.valid_until = rclcpp::Time(static_cast<int64_t>(std::max(now, until) * 1e9),
                               node_->get_clock()->get_clock_type());
  if (revoked) s.reason = "Cancel pending trajectory; wait for executor acknowledgement";
  else if (!s.valid) s.reason = moving_end && start + traj.getTotalDuration() <= now
                                   ? "Moving trajectory duration elapsed"
                                   : "Trajectory blocked by occupied/unknown space";
  else if (std::isfinite(collision)) s.reason = "Safety horizon limited by occupied/unknown space";
  safety_pub_->publish(s);
  std_msgs::msg::String diagnostic;
  std::ostringstream text;
  text << std::setprecision(12) << "{\"kind\":\"safety_lease\",\"stamp\":" << now
       << ",\"session\":" << session_ << ",\"traj_id\":" << id
       << ",\"valid\":" << (s.valid ? "true" : "false")
       << ",\"until\":" << std::max(now, until)
       << ",\"gap_s\":" << (last_lease_stamp_ ? now - last_lease_stamp_ : 0.)
       << ",\"reason\":\"" << s.reason << "\",\"source_fresh\":"
       << (manager_->grid_map_->freshCloud() ? "true" : "false");
  if (std::isfinite(collision))
    text << ",\"blocked_cell\":" << manager_->grid_map_->describeObstruction(
        traj.getPos(std::clamp(collision - start, 0., traj.getTotalDuration())),
        traj.getPos(std::clamp(collision - start + .025, 0., traj.getTotalDuration())), allow_unknown_);
  text << "}";
  diagnostic.data = text.str();
  runtime_pub_->publish(diagnostic);
  last_lease_stamp_ = now;
}
EGOReplanFSM::~EGOReplanFSM() {
  if (planning_job_.valid()) planning_job_.wait();
}

bool EGOReplanFSM::plan(bool preview_only, const Eigen::Vector3d* preview_goal) {
  if (preview_only) {
    if (planning_job_.valid()) return false;
    return computePlan(true, preview_goal);
  }
  if (planning_job_.valid() || handover_pending_) return false;
  // Renew the confirmed reference before copying the map. All live state stays
  // on the executor thread; the worker owns its optimizer, A* pool and buffers.
  if (active_) safetyLease(trajectory_id_, active_traj_, active_start_);
  auto worker = std::make_unique<EGOReplanFSM>();
  worker->node_ = node_;
  worker->vis_ = vis_;
  worker->position_ = position_;
  worker->velocity_ = velocity_;
  worker->goals_ = goals_;
  worker->waypoint_ = waypoint_;
  worker->session_ = session_;
  worker->epoch_ = epoch_;
  worker->phase_ = phase_;
  worker->active_ = active_;
  worker->active_traj_ = active_traj_;
  worker->active_start_ = active_start_;
  worker->previous_id_ = previous_id_;
  worker->previous_traj_ = previous_traj_;
  worker->previous_start_ = previous_start_;
  worker->reference_valid_ = reference_valid_;
  worker->local_target_final_ = local_target_final_;
  worker->execution_limits_ = execution_limits_;
  worker->planning_budget_ = planning_budget_;
  worker->planning_lead_time_ = planning_lead_time_;
  worker->allow_unknown_ = allow_unknown_;
  worker->use_route_seed_ = use_route_seed_;
  worker->adjust_local_target_ = adjust_local_target_;
  worker->diagnostics_pub_ = diagnostics_pub_;
  worker->duration_pub_ = duration_pub_;
  if (!worker_manager_) worker_manager_ = manager_->forkForPlanning();
  worker->manager_ = std::move(worker_manager_);
  worker->manager_->traj_ = manager_->traj_;
  worker->manager_->useSnapshot(manager_->grid_map_->snapshot());
  worker->planning_worker_ = true;
  job_generation_ = planning_generation_;
  job_session_ = session_;
  job_epoch_ = epoch_;
  job_reference_id_ = active_ ? trajectory_id_ : 0;
  planning_job_ = std::async(std::launch::async, [job = std::move(worker)]() mutable {
    job->planning_success_ = job->computePlan(false, nullptr);
    return std::move(job);
  });
  return true;
}

void EGOReplanFSM::pollPlanning() {
  if (!planning_job_.valid() ||
      planning_job_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;
  std::unique_ptr<EGOReplanFSM> result;
  try {
    result = planning_job_.get();
  } catch (const std::exception& e) {
    stop(std::string("Planning worker failed: ") + e.what());
    return;
  }
  worker_manager_ = std::move(result->manager_);
  // Cancel, reset, braking and waypoint changes invalidate in-flight work.
  if (job_generation_ != planning_generation_ || job_session_ != session_ || job_epoch_ != epoch_ ||
      job_reference_id_ != (active_ ? trajectory_id_ : 0) || phase_ != "EXECUTING" ||
      handover_pending_ || pause_requested_ || control_ != "ACTIVE" || !healthy()) {
    std_msgs::msg::String discarded;
    discarded.data = "{\"kind\":\"plan_discard\",\"reason\":\"task_or_reference_changed\"}";
    runtime_pub_->publish(discarded);
    return;
  }
  auto failure = result->last_failure_;
  if (result->planning_success_) {
    if (node_->now().seconds() > result->active_start_ - .025)
      failure = PlanFailure::TIMEOUT;
    else
      failure = validate(result->active_traj_);  // Latest map, not the worker's snapshot.
  }
  std_msgs::msg::String diagnostic;
  std::ostringstream text;
  text << std::setprecision(12) << "{\"kind\":\"plan_commit\",\"stamp\":" << node_->now().seconds()
       << ",\"session\":" << session_ << ",\"result\":\"" << failureName(failure) << "\"}";
  diagnostic.data = text.str();
  runtime_pub_->publish(diagnostic);
  last_failure_ = failure;
  if (result->planning_success_ && failure == PlanFailure::NONE) {
    const auto saved_global = manager_->traj_.global_traj;
    const bool saved_valid = reference_valid_;
    manager_->traj_ = worker_manager_->traj_;
    reference_valid_ = result->reference_valid_;
    local_target_final_ = result->local_target_final_;
    publish(result->active_traj_, result->active_start_, &saved_global, saved_valid);
    reason_ = "Validated asynchronous plan; waiting for executor acknowledgement";
    status();
    return;
  }
  reason_ = std::string("Planning failed: ") + failureName(failure) + "; retaining validated trajectory";
  // A moving local trajectory cannot be held beyond its duration. Start a
  // validated stop while the old reference still has sufficient time.
  const double remaining = active_ ? active_start_ + active_traj_.getTotalDuration() - node_->now().seconds() : 0.;
  const double speed = active_ ? std::max(velocity_.norm(), active_traj_.getVel(std::clamp(
      node_->now().seconds() - active_start_, 0., active_traj_.getTotalDuration())).norm()) : velocity_.norm();
  const double stop_lead = .6 + speed / (execution_limits_[1] * .8) + 2 * execution_limits_[1] / execution_limits_[2];
  if (!active_ || remaining <= stop_lead + planning_lead_time_) {
    if (brake()) {
      phase_ = "BRAKING";
      retry_after_brake_ = true;
      reason_ = "Asynchronous planning failed; braking before reference expires";
    } else {
      stop("No valid initial/replacement trajectory or early braking trajectory");
      return;
    }
  }
  status();
}

bool EGOReplanFSM::computePlan(bool preview_only, const Eigen::Vector3d* preview_goal) {
  vis_->clearProcess();
  const auto wall_start = std::chrono::steady_clock::now();
  const auto deadline =
      wall_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                       std::chrono::duration<double>(planning_budget_));
  const double start = node_->now().seconds() + planning_lead_time_;
  struct Timing {
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub;
    std::chrono::steady_clock::time_point start;
    ~Timing() {
      std_msgs::msg::Float64 m;
      m.data = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
                   .count();
      pub->publish(m);
    }
  } timing{duration_pub_, wall_start};
  const auto saved_global = manager_->traj_.global_traj;
  const auto saved_local = manager_->traj_.local_traj;
  const bool saved_valid = reference_valid_;
  bool committed = false;
  struct Restore {
    std::function<void()> f;
    ~Restore() {
      f();
    }
  } restore{[&] {
    if (preview_only || !committed) {
      manager_->traj_.global_traj = saved_global;
      manager_->traj_.local_traj = saved_local;
      reference_valid_ = saved_valid;
    }
  }};
  Eigen::Vector3d p = position_, v = velocity_, a = Eigen::Vector3d::Zero();
  const poly_traj::Trajectory* reference = nullptr;
  double reference_start = 0;
  if (!preview_only && active_) {
    const bool pending = node_->now().seconds() < active_start_;
    if (!pending || previous_id_) {
      reference = pending ? &previous_traj_ : &active_traj_;
      reference_start = pending ? previous_start_ : active_start_;
      const double t = std::clamp(start - reference_start, 0., reference->getTotalDuration());
      p = reference->getPos(t);
      v = reference->getVel(t);
      a = reference->getAcc(t);
    }
  }
  const Eigen::Vector3d goal = preview_only ? *preview_goal : goals_[waypoint_];
  if ((goal - p).norm() < .03 && v.norm() < .1) {
    if (preview_only) {
      last_failure_ = PlanFailure::INVALID;
      return false;
    }
    local_target_final_ = true;
    if (active_) return true;
    return brake();
  }
  if (preview_only || !reference_valid_) {
    if (!manager_->planGlobalTrajWaypoints(p, v, a, {goal}, Eigen::Vector3d::Zero(),
                                           Eigen::Vector3d::Zero())) {
      last_failure_ = PlanFailure::INVALID;
      return false;
    }
    reference_valid_ = true;
  }
  std::vector<Eigen::Vector3d> guide;
  const auto& global = manager_->traj_.global_traj.traj;
  for (int i = 0; i <= 100; ++i)
    guide.push_back(global.getPos(global.getTotalDuration() * i / 100.));
  vis_->displayGlobalPathList(guide, .055, 0);
  if (!preview_only) vis_->displayGoalPoint(goal, {1, .8, .1, 1}, .3, 1);
  Eigen::Vector3d target, tv;
  bool touch = false;
  manager_->getLocalTarget(manager_->pp_.planning_horizen_, p, goal, target, tv, touch);
  const auto original_target=target;
  // Never move a user's final goal. A blocked rolling-horizon endpoint may
  // advance along the SAME global reference to a free continuation point.
  auto targetClear=[&](const Eigen::Vector3d& point, const Eigen::Vector3d& velocity) {
    if(!traversable(point))return false;
    for(int axis=0;axis<3;++axis)for(int sign : {-1,1}) {
      Eigen::Vector3d q=point;q[axis]+=sign*manager_->route_clearance_;
      if(!traversable(q))return false;
    }
    // The fixed terminal velocity also needs a free approach segment.
    // A free point just beyond an obstacle is not a feasible moving endpoint.
    if(velocity.norm()>.1 && !traversableSegment(point-velocity*.4,point))return false;
    return true;
  };
  if (adjust_local_target_ && !touch && !targetClear(target,tv)) {
    auto& data=manager_->traj_.global_traj;
    const double selected=data.glb_t_of_lc_tgt-data.global_start_time;
    const double step=manager_->grid_map_->getResolution()/manager_->pp_.max_vel_;
    bool found=false;
    for(int sign : {1,-1}) {
      for(double dt=step;dt<=2.5/manager_->pp_.max_vel_;dt+=step) {
        const double t=std::clamp(selected+sign*dt,0.,data.duration);
        const auto candidate=data.traj.getPos(t);
        if((candidate-p).norm()<manager_->pp_.planning_horizen_*.5 ||
           (candidate-p).norm()>manager_->pp_.planning_horizen_+1.5) continue;
        if(targetClear(candidate,data.traj.getVel(t))) {
          target=candidate;tv=manager_->continuationVelocity(candidate,goal,data.traj.getVel(t));touch=t>=data.duration-1e-5;
          if(touch)tv.setZero();
          data.glb_t_of_lc_tgt=data.global_start_time+t;found=true;break;
        }
      }
      if(found)break;
    }
  }
  if (!allow_unknown_) {
    // 已观测自由空间是本项目额外执行边界。仅缩短同一参考上的视距，
    // 不把近场盲区解释成障碍，也不引入侧向终点或零终速度。
    bool unknown_corridor = false;
    const int checks =
        std::max(1, static_cast<int>(std::ceil((target - p).norm() /
                                               (manager_->grid_map_->getResolution() * .5))));
    for (int i = 0; i <= checks; ++i) {
      const Eigen::Vector3d point = p + (target - p) * (static_cast<double>(i) / checks);
      if (manager_->grid_map_->getInflateOccupancy(point) != 0) break;  // 交给原版 A* 处理碰撞
      if (!manager_->grid_map_->isKnownFree(point)) {
        unknown_corridor = true;
        break;
      }
    }
    if (unknown_corridor && manager_->grid_map_->getInflateOccupancy(target) == 0) {
      auto& data = manager_->traj_.global_traj;
      const double selected = data.glb_t_of_lc_tgt - data.global_start_time;
      const double lower = std::max(0., data.last_glb_t_of_lc_tgt - data.global_start_time);
      const double step = manager_->grid_map_->getResolution() / manager_->pp_.max_vel_;
      for (double t = std::max(lower, selected - step);; t = std::max(lower, t - step)) {
        const auto candidate = data.traj.getPos(t);
        if ((candidate - p).norm() < .1) break;
        if (manager_->grid_map_->isKnownFreeSegment(p, candidate)) {
          target = candidate;
          tv = manager_->continuationVelocity(candidate, goal, data.traj.getVel(t));
          touch = false;
          data.glb_t_of_lc_tgt = data.global_start_time + t;
          break;
        }
        if (t <= lower) break;
      }
    }
  }
  manager_->planning_start_time_ = start;
  manager_->setStartJerkReference(reference ? reference->getJer(std::clamp(
      start - reference_start, 0., reference->getTotalDuration())) : Eigen::Vector3d::Zero(),
      reference != nullptr);
  manager_->setDeadline(deadline);
  const bool warm =
      reference && saved_valid && start - reference_start < reference->getTotalDuration() - .05;
  auto modes = retrySequence(warm);
  if (use_route_seed_) {
    const size_t at=warm ? 1 : (!manager_->grid_map_->isCollisionFreeSegment(p,target) ? 0 : 1);
    modes.insert(modes.begin()+at,InitMode{true,false,"route"});
  }
  const bool unusable_endpoint = !traversable(target);
  int trial = 0;
  for (const auto& mode : modes) {
    ++trial;
    manager_->route_seed_fallback_=false;
    if (reference) {
      manager_->traj_.local_traj.traj = *reference;
      manager_->traj_.local_traj.start_time = reference_start;
      manager_->traj_.local_traj.duration = reference->getTotalDuration();
    } else
      manager_->traj_.local_traj = saved_local;
    auto failure = PlanFailure::NONE;
    double time_scale = 1;
    std::array<double, 3> rates{0, 0, 0}, splice{0, 0, 0};
    poly_traj::Trajectory candidate;
    if (std::chrono::steady_clock::now() >= deadline)
      failure = PlanFailure::TIMEOUT;
    else if (!manager_->grid_map_->isCollisionFree(target))
      failure = PlanFailure::COLLISION;
    else if (!allow_unknown_ && !manager_->grid_map_->isKnownFree(target))
      failure = PlanFailure::UNKNOWN;
    else if (!manager_->reboundReplan(p, v, a, target, tv, mode.polynomial, mode.random, touch, std::string(mode.name)=="route"))
      failure = std::chrono::steady_clock::now() >= deadline ? PlanFailure::TIMEOUT
                                                             : PlanFailure::OPTIMIZER;
    else {
      candidate = manager_->traj_.local_traj.traj;
      failure = validate(candidate);
      // 动态超限只修正同一目标的时间分配，禁止借此改变几何目标或强制末速度为零。
      for (int i = 0;
           failure == PlanFailure::DYNAMIC && i < 6 && std::chrono::steady_clock::now() < deadline;
           ++i) {
        rates = peakRates(candidate);
        const double scale =
            std::max({1.05, rates[0] / execution_limits_[0],
                      std::sqrt(rates[1] / execution_limits_[1]),
                      std::cbrt(rates[2] / execution_limits_[2])}) * 1.03;
        candidate = stretchTime(candidate, scale);
        time_scale *= scale;
        failure = validate(candidate);
      }
      if (failure != PlanFailure::INVALID) {
        rates = peakRates(candidate);
        splice = {(candidate.getPos(0) - p).norm(), (candidate.getVel(0) - v).norm(),
                  (candidate.getAcc(0) - a).norm()};
      }
      if (failure == PlanFailure::NONE && (splice[0] > .01 || splice[1] > .01 || splice[2] > .02))
        failure = PlanFailure::CONTINUITY;
    }
    if (std::chrono::steady_clock::now() >= deadline || node_->now().seconds() > start - .025)
      failure = PlanFailure::TIMEOUT;
    last_failure_ = failure;
    const Eigen::Vector3d actual_tv = candidate.getPieceNum()>0 ? candidate.getVel(candidate.getTotalDuration()) : tv;
    std::ostringstream text;
    text << std::setprecision(12) << "{\"stamp\":" << node_->now().seconds()
         << ",\"route_seed_fallback\":" << (manager_->route_seed_fallback_ ? "true" : "false")
         << ",\"target_shifted\":" << ((target-original_target).norm()>.01 ? "true" : "false")
         << ",\"allow_unknown\":" << (allow_unknown_ ? "true" : "false")
         << ",\"session\":" << session_ << ",\"trial\":" << trial << ",\"initialization\":\""
         << mode.name << "\",\"result\":\"" << failureName(failure)
         << "\",\"preview\":" << (preview_only ? "true" : "false") << ",\"target\":[" << target.x()
         << "," << target.y() << "," << target.z() << "],\"target_velocity\":[" << actual_tv.x() << ","
         << actual_tv.y() << "," << actual_tv.z() << "],\"time_scale\":" << time_scale << ",\"splice_error\":["
         << splice[0] << "," << splice[1] << "," << splice[2] << "],\"peak_vaj\":[" << rates[0]
         << "," << rates[1] << "," << rates[2] << "]}";
    std_msgs::msg::String diagnostic;
    diagnostic.data = text.str();
    diagnostics_pub_->publish(diagnostic);
    if (failure != PlanFailure::NONE)
      RCLCPP_INFO(node_->get_logger(), "Plan attempt: %s", diagnostic.data.c_str());
    if (failure == PlanFailure::NONE) {
      if (!preview_only) {
        local_target_final_ = touch;
        manager_->traj_.local_traj.traj = candidate;
        manager_->traj_.local_traj.duration = candidate.getTotalDuration();
        manager_->traj_.local_traj.start_time = start;
        publish(candidate, start, &saved_global, saved_valid);
        committed = true;
      }
      return true;
    }
    if (!canRetry(failure, unusable_endpoint)) break;
  }
  reason_ = std::string("Planning failed: ") + failureName(last_failure_);
  return false;
}

bool EGOReplanFSM::brake(bool publish_result, double earliest_start) {
  double start = std::max(node_->now().seconds() + 0.1, earliest_start);
  Eigen::Vector3d p = position_, v = velocity_, a = Eigen::Vector3d::Zero();
  // A future stop probe uses the trajectory that owns its requested start
  // time, including a committed candidate after its handover.
  const bool pending = active_ && start < active_start_;
  if (active_ && (!pending || previous_id_)) {
    const auto& reference = pending ? previous_traj_ : active_traj_;
    const double reference_start = pending ? previous_start_ : active_start_;
    double t = std::clamp(start - reference_start, 0., reference.getTotalDuration());
    p = reference.getPos(t);
    v = reference.getVel(t);
    a = reference.getAcc(t);
  }
  for (double T = 1.; T <= 4.; T += 0.5) {
    Eigen::Matrix3d head, tail;
    head << p, v, a;
    tail << p + v * T * 0.5, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
    poly_traj::MinJerkOpt opt;
    opt.reset(head, tail, 1);
    opt.generate(Eigen::MatrixXd(3, 0), Eigen::VectorXd::Constant(1, T));
    auto traj = opt.getTraj();
    if (validate(traj) == PlanFailure::NONE) {
      if(publish_result) publish(traj, start);
      return true;
    }
  }
  return false;
}
void EGOReplanFSM::tick() {
  double now = node_->now().seconds();
  if (last_tick_ && (now < last_tick_ - 1e-6 || now - last_tick_ > 2.)) {
    stop("Clock discontinuity");
  }
  last_tick_ = now;
  bool running = phase_ == "EXECUTING" || phase_ == "BRAKING" || phase_ == "PAUSED" ||
                 phase_ == "COMPLETE" || phase_ == "WAIT_CONTROL";
  if (!running) {
    status();
    return;
  }
  if (!healthy()) {
    stop("Selected odometry or local map stale/invalid: " + healthReport());
    return;
  }
  if (phase_ == "WAIT_CONTROL") {
    if (control_ == "ACTIVE") {
      phase_ = "EXECUTING";
      reason_ = "Navigating";
      last_plan_ = 0;
    } else {
      status();
      return;
    }
  }
  if (control_ != "ACTIVE") {
    stop("Control handed back to pilot/PX4");
    return;
  }
  if (!reconcileExecution(now)) {
    status();
    return;
  }
  if (pause_requested_ && phase_ == "EXECUTING" && !handover_pending_) {
    retry_after_brake_ = false;
    if (brake()) {
      phase_ = "BRAKING";
      reason_ = "Executing collision-checked braking trajectory";
      pause_requested_ = false;
    } else {
      stop("No valid braking trajectory; PX4 takeover");
      return;
    }
  }
  if (active_) {
    const double begin = std::clamp(now - active_start_, 0., active_traj_.getTotalDuration());
    const double collision_time = firstCollision(active_traj_, active_start_,
                                                 active_start_ + active_traj_.getTotalDuration());
    if (std::isfinite(collision_time) && now - last_obstruction_stamp_ > .5) {
      // Capture evidence before a successful brake replaces the obstructed reference.
      std::ostringstream text;
      text << std::setprecision(12) << "{\"kind\":\"obstruction\",\"stamp\":" << now
           << ",\"session\":" << session_ << ",\"traj_id\":" << trajectory_id_
           << ",\"collision_time\":" << collision_time << ",\"blocked_cell\":"
           << manager_->grid_map_->describeObstruction(
                  active_traj_.getPos(std::clamp(collision_time - active_start_, 0., active_traj_.getTotalDuration())),
                  active_traj_.getPos(std::clamp(collision_time - active_start_ + .025, 0., active_traj_.getTotalDuration())), allow_unknown_) << "}";
      std_msgs::msg::String diagnostic;
      diagnostic.data = text.str();
      runtime_pub_->publish(diagnostic);
      RCLCPP_INFO(node_->get_logger(), "Trajectory obstruction: %s", diagnostic.data.c_str());
      last_obstruction_stamp_ = now;
    }
    if (handover_pending_) {
      const bool previous_blocked =
          previous_id_ &&
          std::isfinite(firstCollision(previous_traj_, previous_start_, active_start_));
      if (std::isfinite(collision_time) || previous_blocked || cancel_pending_) {
        cancel_pending_ = true;
        safetyLease(trajectory_id_, active_traj_, active_start_, true);
        reason_ = "Revoking pending trajectory before replanning or braking";
      }
    } else if (std::isfinite(collision_time)) {
      const double speed = std::max(velocity_.norm(), active_traj_.getVel(begin).norm());
      const double stop_lead = .6 + speed / (execution_limits_[1] * .8) +
                               2 * execution_limits_[1] / execution_limits_[2];
      // Do not spend the remaining braking horizon on another optimization.
      // A pending worker result is invalidated by the new reference id.
      if (collision_time - now <= stop_lead + planning_lead_time_ + planning_budget_) {
        if (brake()) {
          ++planning_generation_;
          phase_ = "BRAKING";
          retry_after_brake_ = waypoint_ < goals_.size();
          reason_ = "Obstacle approaching braking horizon; stopping before replanning";
        } else {
          stop("Current trajectory obstructed; no validated early braking trajectory");
          return;
        }
      } else if (phase_ == "EXECUTING" && !planning_job_.valid() && now >= obstacle_retry_at_) {
        obstacle_retry_at_ = now + .12;
        last_plan_ = now;
        plan();
        reason_ = "Obstacle ahead; asynchronous replanning with safety renewal";
      }
    }
  }

  if (phase_ == "BRAKING" && active_ && !handover_pending_ &&
      now >= active_start_ + active_traj_.getTotalDuration() && velocity_.norm() < 0.2) {
    if (retry_after_brake_ && ++brake_retries_ <= 3) {
      phase_ = "EXECUTING";
      reference_valid_ = false;
      local_target_final_ = false;
      last_plan_ = 0;
      reason_ = "Stopped for obstacle; retrying from rest";
    } else {
      phase_ = waypoint_ < goals_.size() ? "PAUSED" : "COMPLETE";
      reason_ = "Stopped; resume explicitly if waypoints remain";
    }
    retry_after_brake_ = false;
  }
  if (phase_ == "EXECUTING") {
    // 原版预设航点在接近时提前切换，避免每个中间航点都等待零速度。
    if (waypoint_ + 1 < goals_.size() &&
        (position_ - goals_[waypoint_]).norm() < 0.5 * manager_->pp_.max_vel_) {
      ++planning_generation_;
      ++waypoint_;
      reference_valid_ = false;
      local_target_final_ = false;
      settled_ = 0;
      last_plan_ = 0;
      reason_ = "Advance intermediate waypoint";
    }
    if ((position_ - goals_[waypoint_]).norm() < 0.3 && velocity_.norm() < 0.2) {
      if (!settled_) settled_ = now;
      if (!active_ && !brake()) {
        stop("Cannot validate hold at current goal");
        return;
      }
      if (now - settled_ > 0.6) {
        settled_ = 0;
        ++planning_generation_;
        waypoint_++;
        reference_valid_ = false;
        local_target_final_ = false;
        if (waypoint_ == goals_.size()) {
          phase_ = "COMPLETE";
          reason_ = "Waypoints completed; holding";
        } else {
          reference_valid_ = false;
          last_plan_ = 0;
        }
      }
    } else
      settled_ = 0;
    const bool near_local_end = active_ && !local_target_final_ &&
                                active_start_ + active_traj_.getTotalDuration() - now < 0.8;
    if (phase_ == "EXECUTING" && !handover_pending_ && !planning_job_.valid() && !settled_ &&
        (now - last_plan_ > replan_interval_ || (near_local_end && now - last_plan_ > .3)) &&
        (!active_ || !local_target_final_ ||
         now < active_start_ + active_traj_.getTotalDuration() - .5)) {
      last_plan_ = now;
      plan();

    }
  }
  // 只有执行器确认实际接管后才停止续发旧轨迹授权；等待期间仍逐段检查地图。
  if (handover_pending_ && previous_id_) safetyLease(previous_id_, previous_traj_, previous_start_);
  if (active_) safetyLease(trajectory_id_, active_traj_, active_start_, cancel_pending_);
  status();
}
}  // namespace ego_planner
