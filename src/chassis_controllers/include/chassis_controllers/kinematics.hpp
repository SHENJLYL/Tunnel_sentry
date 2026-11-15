#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace sentry {
constexpr double pi = 3.14159265358979323846;
inline double wrap(double angle) { return std::remainder(angle, 2.0 * pi); }
struct WheelTarget { double steering; double velocity; };

// Rolling direction is axle cross +Z. Axles point along local +X in the supplied URDF.
inline WheelTarget solve_wheel(double vx, double vy, double wz, double x, double y,
                              double mount, double current, double radius) {
  const double ux = vx - wz * y;
  const double uy = vy + wz * x;
  if (std::hypot(ux, uy) < 1e-6) return {current, 0.0};
  double target = std::atan2(uy, ux) + pi / 2.0 - mount;
  double speed = std::hypot(ux, uy) / radius;
  if (std::abs(wrap(target - current)) > pi / 2.0) {
    target += pi;
    speed = -speed;
  }
  const double error = wrap(target - current);
  return {current + error, speed * std::max(0.0, std::cos(error))};
}
}
