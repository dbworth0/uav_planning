//
// Helper functions for using splines
//
// David Butterworth
//

#ifndef SPLINE_UTILS_H
#define SPLINE_UTILS_H

#include <memory> // std::shared_ptr
#include <vector>

#include <spline_library/vector.h>
#include <spline_library/spline.h>
#include <spline_library/hermite/cubic/cubic_hermite_spline.h>

// Get Euclidean distance between 2 Spline points
const float euclideanDistance(const spline_library::Vector3& p0, const spline_library::Vector3& p1);

// Fit a Centripetal Catmull-Rom spline between waypoints.
//
// The overall curve is tighter than Uniform or Chordal types.
//
// By default, the tangent of the first and last waypoint is set by
// extending the previous point in the same direction.
template <typename PointT>
std::shared_ptr<spline_library::Spline<spline_library::Vector3> > getCentripetalCatmullRomSpline(const std::vector<PointT>& path);

// Fit a Centripetal Catmull-Rom spline between waypoints.
//
// extend_direction = Distance beyond start and end points, used to
//                    calculate the tangent at these end-points.
template <typename PointT>
std::shared_ptr<spline_library::Spline<spline_library::Vector3> > getCentripetalCatmullRomSpline(const std::vector<PointT>& path,
                                                                                                 const spline_library::Vector3& start_point_extend_direction,
                                                                                                 const spline_library::Vector3& end_point_extend_direction,
                                                                                                 const float extend_length = 0.05);

// Fit a Centripetal Catmull-Rom spline between waypoints.
template <typename PointT>
std::shared_ptr<spline_library::Spline<spline_library::Vector3> > getCentripetalCatmullRomSpline(const std::vector<PointT>& path,
                                                                                                 const PointT& start_point_extend_direction,
                                                                                                 const PointT& end_point_extend_direction,
                                                                                                 const float extend_length = 0.05);


// Chordal

template <typename PointT>
std::shared_ptr<spline_library::Spline<spline_library::Vector3> > getChordalCatmullRomSpline(const std::vector<PointT>& path,
                                                                                                 const spline_library::Vector3& start_point_extend_direction,
                                                                                                 const spline_library::Vector3& end_point_extend_direction,
                                                                                                 const float extend_length = 0.05);

// Fit a Centripetal Catmull-Rom spline between waypoints.
template <typename PointT>
std::shared_ptr<spline_library::Spline<spline_library::Vector3> > getChordalCatmullRomSpline(const std::vector<PointT>& path,
                                                                                                 const PointT& start_point_extend_direction,
                                                                                                 const PointT& end_point_extend_direction,
                                                                                                 const float extend_length = 0.05);

                                                                                                 

// Sample discrete waypoints from a spline, between
// start time and end time.
//
// Each waypoint is 'dist_step' distance along the path, with the
// exact end point being returned.
// Output is a vector of points, and the spline's total length.
template <typename PointT>
void getPointsFromSpline(const std::shared_ptr<spline_library::Spline<spline_library::Vector3> >& spline,
                         const float t_start,
                         const float t_end,
                         const float dist_step,
                         std::vector<PointT>& splined_path_points,
                         double& spline_length);

// Sample discrete waypoints from the entire length of a spline.
//
// Each waypoint is 'dist_step' distance along the path, with the
// exact end point being returned.
// Returns a vector of points.
template <typename PointT>
const std::vector<PointT> getPointsFromSpline(const std::shared_ptr<spline_library::Spline<spline_library::Vector3> >& spline,
                                              const float dist_step);

// Sample discrete waypoints from the entire length of a spline.
//
// Each waypoint is 'dist_step' distance along the path, with the
// exact end point being returned.
// Output is a vector of points, and the spline's total length.
template <typename PointT>
void getPointsFromSpline(const std::shared_ptr<spline_library::Spline<spline_library::Vector3> >& spline,
                         const float dist_step,
                         std::vector<PointT>& splined_path_points,
                         double& spline_length);

// Implementations:
#include <planning_through_pointclouds/spline_utils.hpp>

#endif // SPLINE_UTILS_H
