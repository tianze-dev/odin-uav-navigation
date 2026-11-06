#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
namespace odin_sim {
constexpr unsigned kPointStep = 19;
inline std::array<double, 3> ray(double u, double v, unsigned w, unsigned h,
                                 double hfov, double vfov) {
  const double fx = w / (2 * std::tan(hfov / 2)),
               fy = h / (2 * std::tan(vfov / 2));
  double y = -(double(u) - (w - 1) * .5) / fx,
         z = -(double(v) - (h - 1) * .5) / fy;
  double l = std::sqrt(1 + y * y + z * z);
  return {1 / l, y / l, z / l};
}
inline void packPoint(uint8_t *out, float x, float y, float z, bool hit) {
  std::memset(out, 0, kPointStep);
  if (!hit || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    return;
  std::memcpy(out, &x, 4);
  std::memcpy(out + 4, &y, 4);
  std::memcpy(out + 8, &z, 4);
  out[12] = 128;
  uint16_t confidence = 255;
  std::memcpy(out + 13, &confidence, 2);
  // Offset time remains zero: ideal whole-frame snapshot, not row-group timing.
}
} // namespace odin_sim
