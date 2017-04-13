//
// Helper functions for PCL (PointCloud Library)
//
// David Butterworth
//

#ifndef PCL_UTILS_H
#define PCL_UTILS_H

#include <string>

#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>





// Load a PCL PointCloud from a file.
// Supported file types: stl, obj, ply, pcd
//
// Works with .ply files that are x,y,z or x,y,z,intensity.
// It crashes if there are arbitary other scalar fields.
//
// It also crashes when loading .ply files saved with CloudCompare.
// Need to re-save them using Meshlab, which adds these 2 lines:
//   element face 0
//   property list uchar int vertex_indices
template <typename PointT>
const bool loadPointCloud(const std::string& file_path,
                          typename pcl::PointCloud<PointT>& cloud_out);

// Down-sample a PointCloud using a voxel grid approach
template <typename PointT>
void downsamplePointCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                          const float voxel_size,
                          typename pcl::PointCloud<PointT>::Ptr& cloud_out);

// Get the x,y,z extents of a cloud, relative to its origin.
//
// Returns floats because that's what PCL uses.
// If the PointCloud is empty, it returns bounds of min = +MaxFloat, max = -MinFloat.
template <typename PointT>
const std::vector<float> getPointCloudBounds(const typename pcl::PointCloud<PointT>::Ptr& cloud);

// Convert a vector of Points into a PointCloud.
// The points are colored red by default, for visualization in CloudCompare.
//
// PointT1 can be xyz or xyzrgb or xyzrgba
// PointT2 must be a type with fields R,G,B.
// This is used to save the path as .ply file.
template <typename PointT1, typename PointT2>
const pcl::PointCloud<PointT2> getPathPointCloudFromPoints(const std::vector<PointT1>& points,
                                                           const unsigned int color_r = 255,
                                                           const unsigned int color_g = 0,
                                                           const unsigned int color_b = 0);

// Crop PointCloud using a cuboid shape
template <typename PointT>
void cropPointCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
				    const typename pcl::PointCloud<PointT>::Ptr& cropped_cloud,
				    const Eigen::Vector3f& min_extents,
				    const Eigen::Vector3f& max_extents,
				    const Eigen::Affine3f& transform = Eigen::Affine3f::Identity());

// Add a linear segment at the start of a path, and another
// linear segment at the end of a path.
// resolution = max distance between the points.
template <typename PointT>
const std::vector<PointT> addTakeoffAndLanding(const std::vector<PointT>& input_path,
                                               const PointT start_point,
                                               const PointT goal_point,
                                               const double min_height, // not used
                                               const double resolution = 0.10);

// Draw a circle in a PointCloud
template <typename PointT>
void addCircleToPointCloud(const PointT center_point,
                           const float radius,
                           const float point_separation,
                           typename pcl::PointCloud<PointT>::Ptr& cloud);

// Draw an open-ended cylinder in a PointCloud
template <typename PointT>
void addCylinderToPointCloud(const PointT base_center_point,
                             const float radius,
                             const float z_height, // +/- value
                             const float point_separation,
                             typename pcl::PointCloud<PointT>::Ptr& cloud);

// Draw an open-ended cone in a PointCloud.
//
// end_radius can be bigger or smaller than base_radius.
template <typename PointT>
void addConeToPointCloud(const PointT base_center_point,
                         const float base_radius,
                         const float end_radius,
                         const float z_height, // +/- value
                         const float point_separation,
                         typename pcl::PointCloud<PointT>::Ptr& cloud);

// Draw a funnel in a PointCloud, consisting of a cylinder
// below the origin point, and a cone above the origin point.
template <typename PointT>
void addVerticalFunnelToPointCloud(const PointT origin_point,
                                   const float offset,
                                   const float middle_radius,
                                   const float top_radius,
                                   const float cylinder_height, // +/- value
                                   const float cone_height, // +/- value
                                   const float point_separation,
                                   typename pcl::PointCloud<PointT>::Ptr& cloud);


// Draw a funnel in a PointCloud, consisting of a cone above the origin point.
template <typename PointT>
void addFunnelToPointCloud(const PointT origin_point,
                                   const float offset,
                                   const float middle_radius,
                                   const float top_radius,
                                   const float cone_height, // +/- value
                                   const float point_separation,
                                   typename pcl::PointCloud<PointT>::Ptr& cloud);


// Draw an x-y plane in a PointCloud
template <typename PointT>
void addPlaneToPointCloud(const float min_x,
                          const float max_x,
                          const float min_y,
                          const float max_y,
                          const float z_height,
                          const float point_separation,
                          typename pcl::PointCloud<PointT>::Ptr& cloud);



// Draw a filled polygon of points in the x-y plane of a PointCloud.
template <typename PointT>
void addPolygonToPointCloud(const std::vector<std::vector<PointT> >& waypoints,
                            const float point_separation,
                            typename pcl::PointCloud<PointT>::Ptr& cloud,
                            const double z_height,
                            const double alpha);




// Add a wall between two x,y points which has a height of min_z to max_z.
template <typename PointT>
void addWallToPointCloud(const PointT p0,
                         const PointT p1,
                         const float point_separation,
                         typename pcl::PointCloud<PointT>::Ptr& cloud,
                         const double default_min_z = -10.0,
                         const double default_max_z = 10.0);

// Add a wall between each set of x,y waypoints in a list.
// Each row of waypoints is a separate wall.
//
// connect_endpoints True = Connect first and last waypoints to make an enclosed wall.
template <typename PointT>
void addWallsToPointCloud(const std::vector<std::vector<PointT> >& waypoints,
                          const float point_separation,
                          typename pcl::PointCloud<PointT>::Ptr& cloud,
                          const bool connect_endpoints = false,
                          const double default_min_z = -10.0,
                          const double default_max_z = 10.0);





template <typename PointT1, typename PointT2>
const pcl::PointCloud<PointT2> getColoredPointCloud(const unsigned int color_r,
                                                    const unsigned int color_g,
                                                    const unsigned int color_b,
                                                    typename pcl::PointCloud<PointT1>::Ptr& cloud_in);


// Save a PointCloud as a .ply file.
// This function writes each point (x,y,z) with a color (r,g,b,a)
template<typename PointT>
inline int savePLYFile(const std::string& file_name, const pcl::PointCloud<PointT>& cloud);


// Calculate the 2D euclidean distance on the x-y plane between two points.
template<typename PointT>
const float euclideanDistanceXY(const PointT& p0, const PointT& p1);



// Implementations:
#include <planning_through_pointclouds/pcl_utils.hpp>



#endif // PCL_UTILS_H
