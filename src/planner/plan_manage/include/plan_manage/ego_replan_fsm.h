#pragma once
#include <plan_manage/planner_manager.h>
#include <plan_manage/replan_policy.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <traj_utils/msg/poly_traj.hpp>
#include <future>
#include <uav_interfaces/msg/navigation_health.hpp>
#include <uav_interfaces/msg/navigation_status.hpp>
#include <uav_interfaces/msg/plan_safety.hpp>
#include <uav_interfaces/msg/trajectory_execution.hpp>
#include <uav_interfaces/srv/set_goal.hpp>
namespace ego_planner {
class EGOReplanFSM {
 public:
  void init(rclcpp::Node::SharedPtr node);
  ~EGOReplanFSM();

 private:
  using Trigger = std_srvs::srv::Trigger;
  rclcpp::Node::SharedPtr node_;
  EGOPlannerManager::Ptr manager_;
  PlanningVisualization::Ptr vis_;
  std::vector<Eigen::Vector3d> staged_, goals_;
  Eigen::Vector3d position_{0, 0, 0}, velocity_{0, 0, 0};
  std::string phase_{"IDLE"}, reason_{"Stage a goal, then start after manual takeoff"},
      control_{"WAITING"};
  uint64_t session_{0}, epoch_{0};
  size_t waypoint_{0};
  double odom_stamp_{0}, last_plan_{0}, last_tick_{0}, last_safety_{0}, settled_{0},
      active_start_{0};
  bool reference_valid_{false}, local_target_final_{false};
  bool allow_unknown_{false}, use_route_seed_{false};
  bool adjust_local_target_{false};
  double executor_timeout_{0.3};
  double session_started_at_{0.};
  bool handover_pending_{false}, cancel_pending_{false};
  int issued_id_{0};
  uav_interfaces::msg::TrajectoryExecution::SharedPtr execution_;
  double obstacle_retry_at_{0};
  double replan_interval_{1.0}, planning_budget_{0.18}, planning_lead_time_{0.25};
  std::array<double, 3> execution_limits_{1.0, 1.0, 3.0};
  PlanFailure last_failure_{PlanFailure::NONE};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;
  bool retry_after_brake_{false};
  int brake_retries_{0};
  bool odom_ok_{false}, cloud_ok_{false}, active_{false}, pause_requested_{false};
  poly_traj::Trajectory active_traj_, previous_traj_;
  GlobalTrajData previous_global_;
  bool previous_reference_valid_{false};
  double previous_start_{0};
  int previous_id_{0};
  int trajectory_id_{0};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_, rviz_goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoints_sub_;
  rclcpp::Subscription<uav_interfaces::msg::NavigationHealth>::SharedPtr health_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_sub_;
  rclcpp::Subscription<uav_interfaces::msg::TrajectoryExecution>::SharedPtr execution_sub_;
  rclcpp::Publisher<traj_utils::msg::PolyTraj>::SharedPtr traj_pub_;
  rclcpp::Publisher<uav_interfaces::msg::NavigationStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<uav_interfaces::msg::PlanSafety>::SharedPtr safety_pub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_client_;
  std::vector<rclcpp::Service<Trigger>::SharedPtr> services_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr duration_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr planning_poll_timer_;
  std::future<std::unique_ptr<EGOReplanFSM>> planning_job_;
  EGOPlannerManager::Ptr worker_manager_;
  bool planning_worker_{false}, planning_success_{false};
  uint64_t planning_generation_{0}, job_generation_{0}, job_session_{0}, job_epoch_{0};
  int job_reference_id_{0};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr runtime_pub_;
  double last_lease_stamp_{0};
  double last_obstruction_stamp_{0};
  rclcpp::Service<uav_interfaces::srv::SetGoal>::SharedPtr goal_service_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr staged_pub_;
  void stageGoals(const std::vector<Eigen::Vector3d>& points);
  void tick();
  void status();
  void stop(const std::string& why);
  void enable(bool on);
  bool healthy() const;
  std::string healthReport() const;
  PlanFailure segmentFailure(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const;
  bool traversable(const Eigen::Vector3d& p) const;
  bool traversableSegment(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const;
  PlanFailure validate(const poly_traj::Trajectory& traj) const;
  void publish(const poly_traj::Trajectory& traj, double start,
               const GlobalTrajData* rollback_global = nullptr, bool rollback_valid = false);
  bool reconcileExecution(double now);
  void safetyLease(int id, const poly_traj::Trajectory& traj, double start, bool revoked = false);
  double firstCollision(const poly_traj::Trajectory& traj, double start, double until) const;
  bool plan(bool preview_only = false, const Eigen::Vector3d* preview_goal = nullptr);
  bool computePlan(bool preview_only, const Eigen::Vector3d* preview_goal);
  void pollPlanning();
  bool brake(bool publish_result = true, double earliest_start = -1.0);
};
}  // namespace ego_planner
