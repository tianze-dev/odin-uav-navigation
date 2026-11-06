#pragma once
#include <array>
#include <cmath>
#include <optional>
namespace uav_rviz_plugins {
inline std::optional<std::array<double,3>> goalOnHeightPlane(
    const std::array<double,3>& origin, const std::array<double,3>& direction, double height) {
  if (!std::isfinite(height)) return std::nullopt;
  for (int i=0;i<3;++i)
    if (!std::isfinite(origin[i]) || !std::isfinite(direction[i])) return std::nullopt;
  if (std::abs(direction[2])<1e-6) return std::nullopt;
  const double distance=(height-origin[2])/direction[2];
  if (distance<0 || distance>10000) return std::nullopt;
  std::array<double,3> point{origin[0]+distance*direction[0],origin[1]+distance*direction[1],height};
  if (std::abs(point[0])>10000 || std::abs(point[1])>10000) return std::nullopt;
  return point;
}
}
