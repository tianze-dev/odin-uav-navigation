#pragma once
#include <array>
#include <optimizer/poly_traj_utils.hpp>
#include <string>
#include <vector>
namespace ego_planner {
enum class PlanFailure {
  NONE,
  INVALID,
  COLLISION,
  UNKNOWN,
  DYNAMIC,
  OPTIMIZER,
  TIMEOUT,
  CONTINUITY
};
inline const char* failureName(PlanFailure f) {
  static const char* names[]{"NONE",    "INVALID",   "COLLISION", "UNKNOWN",
                             "DYNAMIC", "OPTIMIZER", "TIMEOUT",   "CONTINUITY"};
  return names[static_cast<int>(f)];
}
struct InitMode {
  bool polynomial;
  bool random;
  const char* name;
};
inline std::vector<InitMode> retrySequence(bool warm) {
  std::vector<InitMode> modes;
  if (warm) modes.push_back({false, false, "warm"});
  modes.push_back({true, false, "polynomial"});
  modes.push_back({true, true, "random"});
  return modes;
}
inline bool canRetry(PlanFailure failure, bool unusable_endpoint) {
  return !unusable_endpoint && failure != PlanFailure::TIMEOUT && failure != PlanFailure::INVALID;
}
inline std::array<double, 3> peakRates(const poly_traj::Trajectory& t) {
  double jerk = 0;
  for (double s = 0; s < t.getTotalDuration(); s += .01) jerk = std::max(jerk, t.getJer(s).norm());
  jerk = std::max(jerk, t.getJer(t.getTotalDuration()).norm());
  double velocity = t.getMaxVelRate(), acceleration = t.getMaxAccRate();
  // 上游极值函数对常值导数返回 0；执行校验还必须包含每段端点。
  for (int i = 0; i < t.getPieceNum(); ++i) {
    for (double x : {0., t[i].getDuration()}) {
      velocity = std::max(velocity, t[i].getVel(x).norm());
      acceleration = std::max(acceleration, t[i].getAcc(x).norm());
      jerk = std::max(jerk, t[i].getJer(x).norm());
    }
  }
  return {velocity, acceleration, jerk};
}
// 拉长分段时间后重新解最小 jerk，保持起终端 P/V/A；不能直接缩放已有系数而改变衔接速度。
inline poly_traj::Trajectory stretchTime(const poly_traj::Trajectory& t, double scale) {
  const int n = t.getPieceNum();
  Eigen::Matrix3d head, tail;
  head << t.getPos(0), t.getVel(0), t.getAcc(0);
  tail << t.getPos(t.getTotalDuration()), t.getVel(t.getTotalDuration()),
      t.getAcc(t.getTotalDuration());
  poly_traj::MinJerkOpt opt;
  opt.reset(head, tail, n);
  opt.generate(t.getPositions().block(0, 1, 3, n - 1), t.getDurations() * scale);
  return opt.getTraj();
}
}  // namespace ego_planner
