#include <plan_manage/planner_manager.h>

#include <thread>

#include "visualization_msgs/msg/marker.hpp"  // zx-todo

namespace ego_planner {

// SECTION interfaces for setup and query

EGOPlannerManager::EGOPlannerManager() {}

EGOPlannerManager::~EGOPlannerManager() {
  std::cout << "des manager" << std::endl;
}

void EGOPlannerManager::initPlanModules(rclcpp::Node::SharedPtr& node,
                                        PlanningVisualization::Ptr vis) {
  /* read algorithm parameters */
  node_ = node;

  node_->declare_parameter("manager/max_vel", -1.0);
  node_->declare_parameter("manager/max_acc", -1.0);
  node_->declare_parameter("manager/feasibility_tolerance", 0.0);
  node_->declare_parameter("manager/polyTraj_piece_length", -1.0);
  node_->declare_parameter("manager/planning_horizon", 5.0);
  node_->declare_parameter("manager/use_multitopology_trajs", false);

  node_->get_parameter("manager/max_vel", pp_.max_vel_);
  node_->get_parameter("manager/max_acc", pp_.max_acc_);
  node_->get_parameter("manager/feasibility_tolerance", pp_.feasibility_tolerance_);
  node_->get_parameter("manager/polyTraj_piece_length", pp_.polyTraj_piece_length);
  node_->get_parameter("manager/planning_horizon", pp_.planning_horizen_);
  node_->get_parameter("manager/use_multitopology_trajs", pp_.use_multitopology_trajs);

  RCLCPP_INFO(node_->get_logger(), "feasibility_tolerance: %f", pp_.feasibility_tolerance_);
  RCLCPP_INFO(node_->get_logger(), "polyTraj_piece_length: %f", pp_.polyTraj_piece_length);
  RCLCPP_INFO(node_->get_logger(), "planning_horizon: %f", pp_.planning_horizen_);
  RCLCPP_INFO(node_->get_logger(), "use_multitopology_trajs: %d", pp_.use_multitopology_trajs);
  RCLCPP_INFO(node_->get_logger(), "max_vel: %f", pp_.max_vel_);
  RCLCPP_INFO(node_->get_logger(), "max_acc: %f", pp_.max_acc_);

  for (double value :
       {pp_.max_vel_, pp_.max_acc_, pp_.polyTraj_piece_length, pp_.planning_horizen_})
    if (!std::isfinite(value) || value <= 0)
      throw std::invalid_argument("Invalid planner geometry/timing parameter");
  grid_map_.reset(new GridMap);
  grid_map_->initMap(node_);
  route_clearance_ =
      node_->declare_parameter("manager/route_clearance", 2 * grid_map_->getResolution());
  if (!std::isfinite(route_clearance_) || route_clearance_ < 0)
    throw std::invalid_argument("Invalid route clearance");
  adaptive_timing_ = node_->declare_parameter("manager/adaptive_timing", false);
  allow_route_seed_fallback_ = node_->declare_parameter("manager/route_seed_fallback", true);
  cruise_target_ = node_->declare_parameter("manager/cruise_target", false);
  route_cruise_ratio_ = node_->declare_parameter("manager/route_cruise_ratio", 0.85);
  route_terminal_ratio_ = node_->declare_parameter("manager/route_terminal_ratio", 0.7);
  for (double ratio : {route_cruise_ratio_, route_terminal_ratio_})
    if (!std::isfinite(ratio) || ratio <= 0 || ratio > 1)
      throw std::invalid_argument("Invalid route velocity ratio");

  ploy_traj_opt_.reset(new PolyTrajOptimizer);
  ploy_traj_opt_->setParam(node_);
  ploy_traj_opt_->setEnvironment(grid_map_);

  visualization_ = vis;
}

std::unique_ptr<EGOPlannerManager> EGOPlannerManager::forkForPlanning() const {
  auto worker = std::make_unique<EGOPlannerManager>();
  worker->node_ = node_;
  worker->visualization_ = visualization_;
  worker->pp_ = pp_;
  worker->traj_ = traj_;
  worker->route_clearance_ = route_clearance_;
  worker->adaptive_timing_ = adaptive_timing_;
  worker->allow_route_seed_fallback_ = allow_route_seed_fallback_;
  worker->cruise_target_ = cruise_target_;
  worker->route_cruise_ratio_ = route_cruise_ratio_;
  worker->route_terminal_ratio_ = route_terminal_ratio_;
  worker->grid_map_ = grid_map_->snapshot();
  worker->ploy_traj_opt_ = ploy_traj_opt_->fork(worker->grid_map_);
  return worker;
}

bool EGOPlannerManager::reboundReplan(const Eigen::Vector3d& start_pt,
                                      const Eigen::Vector3d& start_vel,
                                      const Eigen::Vector3d& start_acc,
                                      const Eigen::Vector3d& local_target_pt,
                                      const Eigen::Vector3d& local_target_vel,
                                      const bool flag_polyInit, const bool flag_randomPolyTraj,
                                      const bool touch_goal, const bool route_init) {
  rclcpp::Time t_start = node_->now();
  rclcpp::Duration t_opt = rclcpp::Duration::from_seconds(0.0);
  static thread_local int count = 0;
  cout << "\033[47;30m\n[" << t_start.seconds() << "] Replan " << count++ << "\033[0m" << endl;

  /*** STEP 1: INIT ***/
  route_seed_fallback_ = false;
  ploy_traj_opt_->setIfTouchGoal(touch_goal);
  double ts = pp_.polyTraj_piece_length / pp_.max_vel_;

  poly_traj::MinJerkOpt initMJO;
  ploy_traj_opt_->debug_astar_paths_.clear();
  if (route_init) {
    if (!computeRouteInit(start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
                          initMJO))
      return false;
  } else if (!computeInitState(start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
                               flag_polyInit, flag_randomPolyTraj, ts, initMJO)) {
    return false;
  }

  Eigen::MatrixXd cstr_pts =
      initMJO.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
  ploy_traj_opt_->setRouteGuide(route_init ? cstr_pts : Eigen::MatrixXd(3, 0));
  visualization_->displayAStarList({}, 0);
  std::vector<Eigen::Vector3d> initial_curve;
  const auto initial_traj = initMJO.getTraj();
  for (int i = 0; i <= 100; ++i)
    initial_curve.push_back(initial_traj.getPos(initial_traj.getTotalDuration() * i / 100.0));
  visualization_->displayInitPathList(initial_curve, 0.08, 0);
  vector<std::pair<int, int>> segments;
  const auto check = ploy_traj_opt_->finelyCheckAndSetConstraintPoints(segments, initMJO, true);
  visualization_->displayAStarList(ploy_traj_opt_->debug_astar_paths_, 0);
  if (check == PolyTrajOptimizer::CHK_RET::ERR) return false;
  auto t_init = node_->now() - t_start;

  t_start = node_->now();

  /*** STEP 2: OPTIMIZE ***/
  bool flag_success = false;
  vector<vector<Eigen::Vector3d>> vis_trajs;
  poly_traj::MinJerkOpt best_MJO;

  if (pp_.use_multitopology_trajs) {
    std::vector<ConstraintPoints> trajs = ploy_traj_opt_->distinctiveTrajs(segments);
    Eigen::VectorXi success = Eigen::VectorXi::Zero(trajs.size());
    poly_traj::Trajectory initTraj = initMJO.getTraj();
    int PN = initTraj.getPieceNum();
    Eigen::MatrixXd all_pos = initTraj.getPositions();
    Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
    tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
    double final_cost, min_cost = 999999.0;

    for (int i = trajs.size() - 1; i >= 0; i--) {
      ploy_traj_opt_->setConstraintPoints(trajs[i]);
      ploy_traj_opt_->setUseMultitopologyTrajs(true);
      if (ploy_traj_opt_->optimizeTrajectory(headState, tailState, innerPts,
                                             initTraj.getDurations(), final_cost)) {
        success[i] = true;

        if (final_cost < min_cost) {
          min_cost = final_cost;
          best_MJO = ploy_traj_opt_->getMinJerkOpt();
          flag_success = true;
        }

        // visualization
        Eigen::MatrixXd ctrl_pts_temp = ploy_traj_opt_->getMinJerkOpt().getInitConstraintPoints(
            ploy_traj_opt_->get_cps_num_prePiece_());
        std::vector<Eigen::Vector3d> point_set;
        for (int j = 0; j < ctrl_pts_temp.cols(); j++) {
          point_set.push_back(ctrl_pts_temp.col(j));
        }
        vis_trajs.push_back(point_set);
      }
    }

    t_opt = node_->now() - t_start;

    if (trajs.size() > 1) {
      // cout << "\033[1;33m"
      //      << "multi-trajs=" << trajs.size() << ",\033[1;0m"
      //      << " Success:fail=" << success.sum() << ":" << success.size() - success.sum() << endl;
    }

    visualization_->displayMultiOptimalPathList(
        vis_trajs, 0.1);  // This visuallization will take up several milliseconds.
  } else {
    poly_traj::Trajectory initTraj = initMJO.getTraj();
    int PN = initTraj.getPieceNum();
    Eigen::MatrixXd all_pos = initTraj.getPositions();
    Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
    tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
    double final_cost;
    flag_success = ploy_traj_opt_->optimizeTrajectory(headState, tailState, innerPts,
                                                      initTraj.getDurations(), final_cost);
    best_MJO = ploy_traj_opt_->getMinJerkOpt();

    t_opt = node_->now() - t_start;
  }

  visualization_->displayAStarList(ploy_traj_opt_->debug_astar_paths_, 0);
  // The FSM still performs full geometry/dynamics/PVA validation (and retiming).
  // A failed optimizer must not discard an independently feasible route seed.
  if (!flag_success && route_init && allow_route_seed_fallback_) {
    best_MJO = initMJO;
    flag_success = true;
    route_seed_fallback_ = true;
  }
  /*** STEP 3: Store and display results ***/
  cout << "Success=" << (flag_success ? "yes" : "no") << endl;
  if (flag_success) {
    static thread_local double sum_time = 0;
    static thread_local int count_success = 0;
    sum_time += (t_init + t_opt).seconds();
    count_success++;
    printf("Time:\033[42m%.3fms,\033[0m init:%.3fms, optimize:%.3fms, avg=%.3fms\n",
           (t_init + t_opt).seconds() * 1000, t_init.seconds() * 1000, t_opt.seconds() * 1000,
           sum_time / count_success * 1000);

    if (!setLocalTrajFromOpt(best_MJO, touch_goal)) return false;
    cstr_pts = best_MJO.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
    Eigen::MatrixXd curve(3, 101);
    const auto optimized = best_MJO.getTraj();
    for (int i = 0; i <= 100; ++i)
      curve.col(i) = optimized.getPos(optimized.getTotalDuration() * i / 100.0);
    visualization_->displayOptimalList(route_seed_fallback_ ? Eigen::MatrixXd(3, 0) : curve, 0);

    continous_failures_count_ = 0;
  } else {
    cstr_pts = ploy_traj_opt_->getMinJerkOpt().getInitConstraintPoints(
        ploy_traj_opt_->get_cps_num_prePiece_());
    visualization_->displayFailedList(cstr_pts, 0);

    continous_failures_count_++;
  }

  return flag_success;
}

bool EGOPlannerManager::computeRouteInit(const Eigen::Vector3d& p, const Eigen::Vector3d& v,
                                         const Eigen::Vector3d& a, const Eigen::Vector3d& target,
                                         const Eigen::Vector3d& tv, poly_traj::MinJerkOpt& seed) {
  std::vector<Eigen::Vector3d> route;
  const double margin = route_clearance_;
  if (!ploy_traj_opt_->searchRoute(p, target, route, margin) &&
      !ploy_traj_opt_->searchRoute(p, target, route, 0))
    return false;
  route.insert(route.begin(), p);
  route.push_back(target);
  ploy_traj_opt_->debug_astar_paths_.push_back(route);
  auto clear = [&](const Eigen::Vector3d& x, const Eigen::Vector3d& y) {
    if (!grid_map_->isCollisionFreeSegment(x, y)) return false;
    for (int axis = 0; axis < 3; ++axis)
      for (int sign : {-1, 1}) {
        Eigen::Vector3d d = Eigen::Vector3d::Zero();
        d[axis] = sign * margin;
        if (!grid_map_->isCollisionFreeSegment(x + d, y + d)) return false;
      }
    return true;
  };
  // Shortcut only line-of-sight-free edges; retain the search topology.
  std::vector<Eigen::Vector3d> corners{p};
  for (size_t i = 0; i + 1 < route.size();) {
    size_t j = route.size() - 1;
    while (j > i + 1 && !clear(route[i], route[j])) --j;
    if (!grid_map_->isCollisionFreeSegment(route[i], route[j])) return false;
    if ((route[j] - corners.back()).norm() > 1e-6) corners.push_back(route[j]);
    i = j;
  }
  if ((corners.back() - target).norm() > 1e-6)
    corners.push_back(target);
  else
    corners.back() = target;
  std::vector<double> length(corners.size(), 0);
  for (size_t i = 1; i < corners.size(); ++i)
    length[i] = length[i - 1] + (corners[i] - corners[i - 1]).norm();
  const int count = std::max(2, int(std::ceil(length.back() / pp_.polyTraj_piece_length)));
  std::vector<Eigen::Vector3d> knots{p};
  size_t edge = 1;
  for (int i = 1; i < count; ++i) {
    const double distance = length.back() * i / count;
    while (edge + 1 < length.size() && length[edge] < distance) ++edge;
    const double fraction = (distance - length[edge - 1]) / (length[edge] - length[edge - 1]);
    knots.push_back(corners[edge - 1] + fraction * (corners[edge] - corners[edge - 1]));
  }
  knots.push_back(target);
  // Arc-length sampling avoids tiny end pieces with large fixed end velocity.
  // Forward/backward acceleration passes provide physically plausible timing.
  std::vector<double> speed(count + 1, pp_.max_vel_ * route_cruise_ratio_), distance(count);
  for (int i = 0; i < count; ++i) distance[i] = (knots[i + 1] - knots[i]).norm();
  for (int i = 1; i < count; ++i) {
    const auto u = (knots[i] - knots[i - 1]).normalized(),
               w = (knots[i + 1] - knots[i]).normalized();
    const double angle = std::acos(std::clamp(u.dot(w), -1., 1.));
    if (angle > .05)
      speed[i] = std::min(
          speed[i], std::sqrt(pp_.max_acc_ * .7 * std::min(distance[i - 1], distance[i]) / angle));
  }
  Eigen::Vector3d terminal_velocity = Eigen::Vector3d::Zero();
  if (tv.norm() > .01) {
    const double terminal_speed =
        std::min({tv.norm(), speed[count - 1], pp_.max_vel_ * route_terminal_ratio_});
    terminal_velocity = (knots.back() - knots[count - 1]).normalized() * terminal_speed;
  }
  speed.front() = v.norm();
  speed.back() = terminal_velocity.norm();
  for (int i = 1; i < count; ++i)
    speed[i] = std::min(
        speed[i], std::sqrt(speed[i - 1] * speed[i - 1] + 2 * pp_.max_acc_ * .7 * distance[i - 1]));
  for (int i = count - 1; i > 0; --i)
    speed[i] = std::min(
        speed[i], std::sqrt(speed[i + 1] * speed[i + 1] + 2 * pp_.max_acc_ * .7 * distance[i]));
  Eigen::Matrix3d head, tail;
  head << p, v, a;
  tail << target, terminal_velocity, Eigen::Vector3d::Zero();
  Eigen::MatrixXd inner(3, count - 1);
  Eigen::VectorXd durations(count);
  for (int i = 0; i < count; ++i) {
    durations[i] = std::max(.15, 2 * distance[i] / std::max(.2, speed[i] + speed[i + 1]));
    if (i < count - 1) inner.col(i) = knots[i + 1];
  }
  seed.reset(head, tail, count);
  seed.generate(inner, durations);
  return true;
}

bool EGOPlannerManager::computeInitState(const Eigen::Vector3d& start_pt,
                                         const Eigen::Vector3d& start_vel,
                                         const Eigen::Vector3d& start_acc,
                                         const Eigen::Vector3d& local_target_pt,
                                         const Eigen::Vector3d& local_target_vel,
                                         const bool flag_polyInit, const bool flag_randomPolyTraj,
                                         const double& ts, poly_traj::MinJerkOpt& initMJO) {

  if (first_init_ || flag_polyInit) /*** case 1: polynomial initialization ***/
  {
    first_init_ = false;

    /* basic params */
    Eigen::Matrix3d headState, tailState;
    Eigen::MatrixXd innerPs;
    Eigen::VectorXd piece_dur_vec;
    int piece_nums;
    constexpr double init_of_init_totaldur = 2.0;
    headState << start_pt, start_vel, start_acc;
    tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

    /* determined or random inner point */
    if (!flag_randomPolyTraj) {
      if (innerPs.cols() != 0) {
        RCLCPP_ERROR(node_->get_logger(), "innerPs.cols() != 0");
      }

      piece_nums = 1;
      piece_dur_vec.resize(1);
      piece_dur_vec(0) = init_of_init_totaldur;
    } else {
      Eigen::Vector3d horizen_dir =
          ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
      Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
      innerPs.resize(3, 1);
      innerPs = (start_pt + local_target_pt) / 2 +
                (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() *
                    horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() *
                    vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);

      piece_nums = 2;
      piece_dur_vec.resize(2);
      piece_dur_vec = Eigen::Vector2d(init_of_init_totaldur / 2, init_of_init_totaldur / 2);
    }

    /* generate the init of init trajectory */
    initMJO.reset(headState, tailState, piece_nums);
    initMJO.generate(innerPs, piece_dur_vec);
    poly_traj::Trajectory initTraj = initMJO.getTraj();

    /* generate the real init trajectory */
    piece_nums = round((headState.col(0) - tailState.col(0)).norm() / pp_.polyTraj_piece_length);
    if (piece_nums < 2) piece_nums = 2;
    double piece_dur = init_of_init_totaldur / (double)piece_nums;
    piece_dur_vec.resize(piece_nums);
    piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, ts);
    innerPs.resize(3, piece_nums - 1);
    int id = 0;
    double t_s = piece_dur, t_e = init_of_init_totaldur - piece_dur / 2;
    for (double t = t_s; t < t_e; t += piece_dur) {
      innerPs.col(id++) = initTraj.getPos(t);
    }
    if (id != piece_nums - 1) {
      RCLCPP_ERROR(node_->get_logger(), "Should not happen! x_x");
      return false;
    }
    initMJO.reset(headState, tailState, piece_nums);
    initMJO.generate(innerPs, piece_dur_vec);
  } else /*** case 2: initialize from previous optimal trajectory ***/
  {
    if (traj_.global_traj.last_glb_t_of_lc_tgt < 0.0) {
      RCLCPP_ERROR(node_->get_logger(),
                   "You are initialzing a trajectory from a previous optimal trajectory, but no "
                   "previous trajectories up to now.");
      return false;
    }

    /* the trajectory time system is a little bit complicated... */
    double passed_t_on_lctraj =
        (planning_start_time_ >= 0 ? planning_start_time_ : node_->now().seconds()) -
        traj_.local_traj.start_time;
    double t_to_lc_end = traj_.local_traj.duration - passed_t_on_lctraj;
    if (t_to_lc_end < 0) {
      RCLCPP_INFO(node_->get_logger(), "t_to_lc_end < 0, exit and wait for another call.");
      return false;
    }
    double t_to_lc_tgt =
        t_to_lc_end + (traj_.global_traj.glb_t_of_lc_tgt - traj_.global_traj.last_glb_t_of_lc_tgt);
    int piece_nums = ceil((start_pt - local_target_pt).norm() / pp_.polyTraj_piece_length);
    if (piece_nums < 2) piece_nums = 2;

    Eigen::Matrix3d headState, tailState;
    Eigen::MatrixXd innerPs(3, piece_nums - 1);
    Eigen::VectorXd piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, t_to_lc_tgt / piece_nums);
    headState << start_pt, start_vel, start_acc;
    tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

    double t = piece_dur_vec(0);
    for (int i = 0; i < piece_nums - 1; ++i) {
      if (t < t_to_lc_end) {
        innerPs.col(i) = traj_.local_traj.traj.getPos(t + passed_t_on_lctraj);
      } else if (t <= t_to_lc_tgt) {
        double glb_t = t - t_to_lc_end + traj_.global_traj.last_glb_t_of_lc_tgt -
                       traj_.global_traj.global_start_time;
        innerPs.col(i) = traj_.global_traj.traj.getPos(glb_t);
      } else {
        RCLCPP_ERROR(node_->get_logger(),
                     "Should not happen! x_x 0x88 t=%.2f, t_to_lc_end=%.2f, t_to_lc_tgt=%.2f", t,
                     t_to_lc_end, t_to_lc_tgt);
      }

      t += piece_dur_vec(i + 1);
    }

    initMJO.reset(headState, tailState, piece_nums);
    initMJO.generate(innerPs, piece_dur_vec);
  }

  return true;
}

void EGOPlannerManager::getLocalTarget(const double planning_horizen,
                                       const Eigen::Vector3d& start_pt,
                                       const Eigen::Vector3d& global_end_pt,
                                       Eigen::Vector3d& local_target_pos,
                                       Eigen::Vector3d& local_target_vel, bool& touch_goal) {
  double t;
  touch_goal = false;

  traj_.global_traj.last_glb_t_of_lc_tgt = traj_.global_traj.glb_t_of_lc_tgt;

  double t_step = planning_horizen / 20 / pp_.max_vel_;
  for (t = traj_.global_traj.glb_t_of_lc_tgt;
       t < (traj_.global_traj.global_start_time + traj_.global_traj.duration); t += t_step) {
    Eigen::Vector3d pos_t = traj_.global_traj.traj.getPos(t - traj_.global_traj.global_start_time);
    double dist = (pos_t - start_pt).norm();

    if (dist >= planning_horizen) {
      local_target_pos = pos_t;
      traj_.global_traj.glb_t_of_lc_tgt = t;
      break;
    }
  }

  if ((t - traj_.global_traj.global_start_time) >=
      traj_.global_traj.duration - 1e-5)  // Last global point
  {
    local_target_pos = global_end_pt;
    traj_.global_traj.glb_t_of_lc_tgt =
        traj_.global_traj.global_start_time + traj_.global_traj.duration;
    touch_goal = true;
  }

  if (cruise_target_) {
    local_target_vel = continuationVelocity(
        local_target_pos, global_end_pt,
        traj_.global_traj.traj.getVel(t - traj_.global_traj.global_start_time));
  } else if ((global_end_pt - local_target_pos).norm() <
             (pp_.max_vel_ * pp_.max_vel_) / (2 * pp_.max_acc_)) {
    local_target_vel = Eigen::Vector3d::Zero();
  } else {
    local_target_vel = traj_.global_traj.traj.getVel(t - traj_.global_traj.global_start_time);
  }
}

Eigen::Vector3d EGOPlannerManager::continuationVelocity(
    const Eigen::Vector3d& target, const Eigen::Vector3d& goal,
    const Eigen::Vector3d& reference_velocity) const {
  if (!cruise_target_) return reference_velocity;
  const double remaining = (goal - target).norm();
  if (remaining < .03) return Eigen::Vector3d::Zero();
  Eigen::Vector3d direction = reference_velocity;
  if (direction.norm() < .01) direction = goal - target;
  const double speed =
      std::min(pp_.max_vel_ * route_cruise_ratio_, std::sqrt(2 * pp_.max_acc_ * .7 * remaining));
  return direction.normalized() * speed;
}

bool EGOPlannerManager::setLocalTrajFromOpt(const poly_traj::MinJerkOpt& opt,
                                            const bool touch_goal) {
  poly_traj::Trajectory traj = opt.getTraj();
  Eigen::MatrixXd cps = opt.getInitConstraintPoints(getCpsNumPrePiece());
  PtsChk_t pts_to_check;
  bool ret = ploy_traj_opt_->computePointsToCheck(
      traj, ConstraintPoints::two_thirds_id(cps, touch_goal), pts_to_check);
  if (ret && pts_to_check.size() >= 1 && pts_to_check.back().size() >= 1) {
    auto time_now = node_->now().seconds();
    traj_.setLocalTraj(traj, pts_to_check, time_now);
  }

  return ret;
}

bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d& start_pos,
                                                const Eigen::Vector3d& start_vel,
                                                const Eigen::Vector3d& start_acc,
                                                const std::vector<Eigen::Vector3d>& waypoints,
                                                const Eigen::Vector3d& end_vel,
                                                const Eigen::Vector3d& end_acc) {
  poly_traj::MinJerkOpt globalMJO;
  Eigen::Matrix<double, 3, 3> headState, tailState;
  headState << start_pos, start_vel, start_acc;
  tailState << waypoints.back(), end_vel, end_acc;
  Eigen::MatrixXd innerPts;

  if (waypoints.size() > 1) {
    innerPts.resize(3, waypoints.size() - 1);
    for (int i = 0; i < (int)waypoints.size() - 1; ++i) {
      innerPts.col(i) = waypoints[i];
    }
  } else {
    if (innerPts.size() != 0) {
      RCLCPP_ERROR(node_->get_logger(), "innerPts.size() != 0");
    }
  }

  globalMJO.reset(headState, tailState, waypoints.size());

  if (adaptive_timing_) {
    // 参考曲线仍以固定 P/V/A 求解，按实际极值连续校正时间，避免 1.5 倍跳变。
    Eigen::VectorXd durations(waypoints.size());
    for (size_t i = 0; i < waypoints.size(); ++i)
      durations(i) =
          std::max(.2, (waypoints[i] - (i ? waypoints[i - 1] : start_pos)).norm() / pp_.max_vel_);
    const double velocity_limit = std::max({pp_.max_vel_ * .98, start_vel.norm(), end_vel.norm()});
    const double acceleration_limit =
        std::max({pp_.max_acc_ * .9, start_acc.norm(), end_acc.norm()});
    for (int iteration = 0; iteration < 16; ++iteration) {
      if (!durations.allFinite() || durations.minCoeff() <= 0) return false;
      globalMJO.generate(innerPts, durations);
      const auto trajectory = globalMJO.getTraj();
      double scale;
      try {
        const double peak_velocity = trajectory.getMaxVelRate();
        const double peak_acceleration = trajectory.getMaxAccRate();
        if (!std::isfinite(peak_velocity) || !std::isfinite(peak_acceleration)) return false;
        scale = std::max(peak_velocity / velocity_limit,
                         std::sqrt(peak_acceleration / acceleration_limit));
      } catch (const std::exception&) {
        return false;
      }
      if (scale <= 1.0001) break;
      if (iteration == 15) return false;  // 时间分配未收敛时不发布失真的参考。
      durations *= scale * 1.005;
    }
    traj_.setGlobalTraj(globalMJO.getTraj(), node_->now().seconds());
    return true;
  }

  double des_vel = pp_.max_vel_ / 1.5;
  Eigen::VectorXd time_vec(waypoints.size());

  // cout << "Initial des_vel: " << des_vel << endl;
  // cout << "max_vel_: " << pp_.max_vel_ << endl;
  // cout << "Waypoints size: " << waypoints.size() << endl;
  // cout << "Start pos: " << start_pos.transpose() << endl;
  // cout << "End pos: " << waypoints.back().transpose() << endl;

  for (int j = 0; j < 2; ++j) {
    cout << "=== Iteration " << j << " ===" << endl;
    // cout << "des_vel: " << des_vel << endl;

    for (size_t i = 0; i < waypoints.size(); ++i) {
      double segment_time = (i == 0) ? (waypoints[0] - start_pos).norm() / des_vel
                                     : (waypoints[i] - waypoints[i - 1]).norm() / des_vel;

      // cout << "Segment " << i << " time: " << segment_time << endl;
      time_vec(i) = segment_time;
    }

    // cout << "Time vector: " << time_vec.transpose() << endl;
    // cout << "Time vector sum: " << time_vec.sum() << endl;

    globalMJO.generate(innerPts, time_vec);

    auto generated_traj = globalMJO.getTraj();
    // cout << "Generated traj total duration: " << generated_traj.getTotalDuration() << endl;

    if (globalMJO.getTraj().getMaxVelRate() < pp_.max_vel_ || start_vel.norm() > pp_.max_vel_ ||
        end_vel.norm() > pp_.max_vel_) {
      // cout << "Breaking early - velocity constraints satisfied" << endl;
      break;
    }

    des_vel /= 1.5;
    // cout << "Reducing des_vel to: " << des_vel << endl;
  }

  auto time_now = node_->now();
  // cout << "Final generated trajectory duration: " << globalMJO.getTraj().getTotalDuration() <<
  // endl; cout << "Current time: " << time_now.seconds() << endl;

  traj_.setGlobalTraj(globalMJO.getTraj(), time_now.seconds());

  return true;
}

}  // namespace ego_planner
