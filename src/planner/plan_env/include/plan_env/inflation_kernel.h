#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace scut {
// Offsets run from an occupied cell to forbidden vehicle-centre cells.
// For the cylinder, test the minimum distance between the TWO voxel boxes,
// not just their centres. This covers quantization of both hit and query.
inline std::vector<Eigen::Vector3i> inflationKernel(
    double resolution, double radius, const std::string& shape,
    double above, double below) {
  if (!std::isfinite(resolution) || resolution <= 0 ||
      !std::isfinite(radius) || radius < 0 ||
      !std::isfinite(above) || above < 0 ||
      !std::isfinite(below) || below < 0 ||
      (shape != "box" && shape != "cylinder"))
    throw std::invalid_argument("Invalid inflation geometry");
  const double extra = shape == "cylinder" ? resolution : 0;
  const int n = std::ceil((std::max({radius, above, below}) + extra) / resolution);
  if (n > 24) throw std::invalid_argument("Inflation kernel exceeds supported extent");
  std::vector<Eigen::Vector3i> offsets;
  for (int x = -n; x <= n; ++x)
    for (int y = -n; y <= n; ++y)
      for (int z = -n; z <= n; ++z) {
        if (shape == "cylinder") {
          const double dx = std::max(0, std::abs(x) - 1) * resolution;
          const double dy = std::max(0, std::abs(y) - 1) * resolution;
          const double dz = std::max(0, std::abs(z) - 1) * resolution;
          // A centre ABOVE an obstacle needs space for the body BELOW it.
          if (std::hypot(dx, dy) > radius + 1e-9 ||
              dz > (z >= 0 ? below : above) + 1e-9) continue;
        }
        offsets.emplace_back(x, y, z);
      }
  if (offsets.size() >= 32767)
    throw std::invalid_argument("Inflation kernel exceeds reference-count capacity");
  return offsets;
}
}
