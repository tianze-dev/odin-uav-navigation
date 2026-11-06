#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <optimizer/poly_traj_optimizer.h>
#include <plan_env/grid_map.h>
#include <stdlib.h>
#include <traj_utils/planning_visualization.h>

#include <memory>
#include <optimizer/poly_traj_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traj_utils/plan_container.hpp>

namespace ego_planner {

// Fast Planner Manager
// Key algorithms of mapping and planning are called

class EGOPlannerManager {
  // SECTION stable
 public:
  EGOPlannerManager();
  ~EGOPlannerManager();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* main planning interface */
  void initPlanModules(rclcpp::Node::SharedPtr& node, PlanningVisualization::Ptr vis);
  std::unique_ptr<EGOPlannerManager> forkForPlanning() const;
  void useSnapshot(const GridMap::Ptr& map) {
    grid_map_ = map;
    ploy_traj_opt_->useSnapshot(map);
  }
  bool computeInitState(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& start_vel,
                        const Eigen::Vector3d& start_acc, const Eigen::Vector3d& local_target_pt,
                        const Eigen::Vector3d& local_target_vel, const bool flag_polyInit,
                        const bool flag_randomPolyTraj, const double& ts,
                        poly_traj::MinJerkOpt& initMJO);
  bool reboundReplan(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& start_vel,
                     const Eigen::Vector3d& start_acc, const Eigen::Vector3d& end_pt,
                     const Eigen::Vector3d& end_vel, const bool flag_polyInit,
                     const bool flag_randomPolyTraj, const bool touch_goal,
                     const bool route_init = false);
  bool computeRouteInit(const Eigen::Vector3d& p, const Eigen::Vector3d& v,
                        const Eigen::Vector3d& a, const Eigen::Vector3d& target,
                        const Eigen::Vector3d& tv, poly_traj::MinJerkOpt& seed);
  bool planGlobalTrajWaypoints(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel,
                               const Eigen::Vector3d& start_acc,
                               const std::vector<Eigen::Vector3d>& waypoints,
                               const Eigen::Vector3d& end_vel, const Eigen::Vector3d& end_acc);
  void getLocalTarget(const double planning_horizen, const Eigen::Vector3d& start_pt,
                      const Eigen::Vector3d& global_end_pt, Eigen::Vector3d& local_target_pos,
                      Eigen::Vector3d& local_target_vel, bool& touch_goal);
  Eigen::Vector3d continuationVelocity(const Eigen::Vector3d& target, const Eigen::Vector3d& goal,
                                       const Eigen::Vector3d& reference_velocity) const;
  void setStartJerkReference(const Eigen::Vector3d& jerk, bool valid) {
    ploy_traj_opt_->setStartJerkReference(jerk, valid);
  }
  bool setLocalTrajFromOpt(const poly_traj::MinJerkOpt& opt, const bool touch_goal);
  inline int getCpsNumPrePiece(void) {
    return ploy_traj_opt_->get_cps_num_prePiece_();
  }
  // inline PtsChk_t getPtsCheck(void) { return ploy_traj_opt_->get_pts_check_(); }

  void setDeadline(std::chrono::steady_clock::time_point t) {
    ploy_traj_opt_->setDeadline(t);
  }
  double planning_start_time_{-1.0};
  bool route_seed_fallback_{false};
  bool allow_route_seed_fallback_{true};
  double route_clearance_{0.3};
  bool adaptive_timing_{false}, cruise_target_{false};
  double route_cruise_ratio_{0.85}, route_terminal_ratio_{0.7};
  PlanParameters pp_;
  GridMap::Ptr grid_map_;
  TrajContainer traj_;

 private:
  PlanningVisualization::Ptr visualization_;

  PolyTrajOptimizer::Ptr ploy_traj_opt_;

  int continous_failures_count_{0};
  bool first_init_{true};

  rclcpp::Node::SharedPtr node_;

 public:
  using Ptr = std::unique_ptr<EGOPlannerManager>;

  // !SECTION
};
}  // namespace ego_planner

#endif
