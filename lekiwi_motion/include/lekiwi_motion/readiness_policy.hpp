#ifndef LEKIWI_CONTROL__READINESS_POLICY_HPP_
#define LEKIWI_CONTROL__READINESS_POLICY_HPP_

#include <cmath>

namespace lekiwi_control::policy
{
// Reject missing, stale and future samples against the same ROS clock.
inline bool fresh(double now, double stamp, double max_age)
{
  return std::isfinite(now) && std::isfinite(stamp) && stamp > 0.0 &&
    now >= stamp && now - stamp <= max_age;
}

// Check measured planar velocities only after the caller verifies freshness.
inline bool stationary(double vx, double vy, double wz, double linear, double angular)
{
  return std::isfinite(vx) && std::isfinite(vy) && std::isfinite(wz) &&
    std::hypot(vx, vy) <= linear && std::abs(wz) <= angular;
}

// Validate each variance before summing so negative terms cannot cancel.
inline bool converged(double x, double y, double yaw, double max_pos, double max_yaw)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(yaw) &&
    x >= 0.0 && y >= 0.0 && yaw >= 0.0 && x + y <= max_pos && yaw <= max_yaw;
}
}  // namespace lekiwi_control::policy
#endif
