//
// Helper functions for using splines
//
// David Butterworth
//

#include <planning_through_pointclouds/spline_utils.h>

const float euclideanDistance(const spline_library::Vector3& p0, const spline_library::Vector3& p1)
{
  const float dx = static_cast<float>(p1[0] - p0[0]);
  const float dy = static_cast<float>(p1[1] - p0[1]);
  const float dz = static_cast<float>(p1[2] - p0[2]);

  return std::sqrt(dx*dx + dy*dy + dz*dz);
}
