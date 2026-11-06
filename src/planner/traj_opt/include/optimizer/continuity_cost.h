#pragma once
#include <Eigen/Core>

namespace ego_planner {
// 起点 P/V/A 是硬边界；jerk 只作软衔接，不阻止必要的避障改道。
inline double startJerkCost(const Eigen::Vector3d& jerk, const Eigen::Vector3d& previous,
                            double weight, Eigen::Vector3d& gradient) {
  const Eigen::Vector3d delta = jerk - previous;
  gradient = 2 * weight * delta;
  return weight * delta.squaredNorm();
}
}  // namespace ego_planner
