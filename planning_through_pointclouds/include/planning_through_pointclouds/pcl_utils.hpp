//
// Helper functions for PCL (PointCloud Library)
//
// David Butterworth
//

#ifndef PCL_UTILS_IMPL_HPP
#define PCL_UTILS_IMPL_HPP

#include <vector>
#include <iostream> // cout
#include <limits> // numeric_limits

#include <type_traits> // C++0x
//#include <tr1/type_traits> // C++03, use std::tr1



#include <boost/filesystem.hpp> // exists()

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>  // loadPolygonFile()
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h> // removeNaNFromPointCloud()
#include <pcl/filters/voxel_grid.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h> // PLYWriter

#include <planning_through_pointclouds/utils.h> // lerp

#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h> // isPointIn2DPolygon


template <typename PointT>
const bool loadPointCloud(const std::string& file_path,
                          typename pcl::PointCloud<PointT>& cloud_out)
{
  if (!boost::filesystem::exists(file_path))
  {
    PCL_ERROR("Failed to load file in loadPointCloud(), file does not exist! \n");
    return false;
  }

  pcl::PolygonMesh mesh;

  // The file extension is used to decide which loader to use,
  // and each behaves differently:
  //   .pcd file: returns the number of points on success, or prints error and returns zero on failure.
  //   .ply file: returns the number of points on success, but throws error on failure
  //               e.g. vtkPLYReader (0xb16510): Could not open PLY file
  // because PCL loadPolygonFilePLY() doesn't catch these errors.

  int result = pcl::io::loadPolygonFile(file_path.c_str(), mesh); // this is in PCL vtk_lib_io.cpp
  if (result <= 0)
  {
    PCL_ERROR("Failed to load file in loadPointCloud(). \n");
    return false;
  }

  pcl::fromPCLPointCloud2<PointT>(mesh.cloud, cloud_out);
  return true;
  
  //std::vector<int> index;
  //pcl::removeNaNFromPointCloud(cloud_out, cloud_out, index);
}

template <typename PointT>
void downsamplePointCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                          const float voxel_size,
                          typename pcl::PointCloud<PointT>::Ptr& cloud_out)
{

  
  pcl::VoxelGrid<PointT> filter;
  filter.setInputCloud(cloud_in);
  filter.setLeafSize(voxel_size, voxel_size, voxel_size);
  filter.filter(*cloud_out);
}

template <typename PointT>
const std::vector<float> getPointCloudBounds(const typename pcl::PointCloud<PointT>::Ptr& cloud)
{
  PointT min_point;
  PointT max_point;
  pcl::getMinMax3D(*cloud, min_point, max_point);
  const std::vector<float> bounds = {min_point.x, max_point.x,
                                     min_point.y, max_point.y,
                                     min_point.z, max_point.z};
  return bounds;
}

template <typename PointT1, typename PointT2>
const pcl::PointCloud<PointT2> getPathPointCloudFromPoints(const std::vector<PointT1>& points,
                                                           const unsigned int color_r,
                                                           const unsigned int color_g,
                                                           const unsigned int color_b)
{
  pcl::PointCloud<PointT2> cloud;

  for (size_t i = 0; i < points.size(); ++i)
  {
    PointT2 point;
    point.x = points.at(i).x;
    point.y = points.at(i).y;
    point.z = points.at(i).z;

    // doesnt seem to work
    // If template types are different, assume the PointCloud
    // contains color points.
    // Uses template type_traits
    //if (!std::is_same<PointT1,PointT2>::value)
    //if ((std::is_same<PointT2,pcl::PointXYZRGB>::value) || (std::is_same<PointT2,pcl::PointXYZRGBA>::value))
    {
      // red
      point.r = static_cast<pcl::uint8_t>(color_r); // uint_8
      point.g = static_cast<pcl::uint8_t>(color_g);;
      point.b = static_cast<pcl::uint8_t>(color_b);;
    }

    cloud.push_back(point);
  }

  return cloud;
}

template <typename PointT>
void cropPointCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                    const typename pcl::PointCloud<PointT>::Ptr& cropped_cloud,
                    const Eigen::Vector3f& min_extents,
                    const Eigen::Vector3f& max_extents,
                    const Eigen::Affine3f& transform = Eigen::Affine3f::Identity())
{
  typename pcl::CropBox<PointT> crop_box;

  crop_box.setInputCloud(cloud_in);
  crop_box.setTransform(transform);

  crop_box.setMin(Eigen::Vector4f(min_extents.x(), min_extents.y(), min_extents.z(), 0.0));
  crop_box.setMax(Eigen::Vector4f(max_extents.x(), max_extents.y(), max_extents.z(), 0.0));

  crop_box.filter(*cropped_cloud);
}

template <typename PointT>
const std::vector<PointT> addTakeoffAndLanding(const std::vector<PointT>& input_path,
                                               const PointT start_point,
                                               const PointT goal_point,
                                               const double min_height, // not used
                                               const double resolution)
{
  std::vector<PointT> path;
  const float eps = 0.00001;
  //const double resolution = 0.10;

  // Add linear path segment between desired start position and
  // the start of the input path.
  if (std::fabs(start_point.z - input_path.at(0).z) > eps)
  {
    std::cout << "adding start seg" << std::endl;
    const std::vector<double> p0 = {start_point.x, start_point.y, start_point.z};
    const std::vector<double> p1 = {input_path.at(0).x, input_path.at(0).y, input_path.at(0).z};
    const std::vector<std::vector<double> > linear_segment = lerp(p0, p1, resolution);
    //if (linear_segment.size() == 0) throw
    for (size_t i = 0; i < linear_segment.size(); ++i)
    {
      path.push_back(PointT(linear_segment.at(i).at(0),
                            linear_segment.at(i).at(1),
                            linear_segment.at(i).at(2)));
    }
  }

  // Copy the input path
  for (size_t i = 0; i < input_path.size(); ++i)
  {
    path.push_back(input_path.at(i));
  }

  // Add linear path segment between the end of the input path
  // and the desired goal position.

  //if (std::fabs(goal_point.z - input_path.at(0).z) > eps)
  const int last_wp_idx = input_path.size() - 1;
  if (std::fabs(goal_point.z - input_path.at(last_wp_idx).z) > eps)
  {
    std::cout << "adding end seg" << std::endl;
    const std::vector<double> p0 = {input_path.at(last_wp_idx).x, input_path.at(last_wp_idx).y, input_path.at(last_wp_idx).z};
    const std::vector<double> p1 = {goal_point.x, goal_point.y, goal_point.z};
    const std::vector<std::vector<double> > linear_segment = lerp(p0, p1, resolution);
    //if (linear_segment.size() == 0) throw
    for (size_t i = 0; i < linear_segment.size(); ++i)
    {
      path.push_back(PointT(linear_segment.at(i).at(0),
                            linear_segment.at(i).at(1),
                            linear_segment.at(i).at(2)));
    }
  }

  return path;
}

template <typename PointT>
void addCircleToPointCloud(const PointT center_point,
                           const float radius,
                           const float point_separation,
                           typename pcl::PointCloud<PointT>::Ptr& cloud)
{
  const float circumference = 2.0 * M_PI * radius;

  // The number of points to draw on the circumference
  const unsigned int n = static_cast<unsigned int>(std::ceil(circumference / point_separation));

  for (int i = 0; i < n; ++i)
  {
    const PointT point(center_point.x + radius * std::cos(2.0*M_PI / n*i),
                       center_point.y + radius * std::sin(2.0*M_PI / n*i),
                       center_point.z);
    cloud->points.push_back(point);
  }

  // loadPointCloud() sets is_dense=True, so we need to change that
  cloud->is_dense = false; // May contain NaN or Inf values
  cloud->height = 1;
  cloud->width = cloud->points.size();
}

template <typename PointT>
void addCylinderToPointCloud(const PointT base_center_point,
                             const float radius,
                             const float z_height, // +/- value
                             const float point_separation,
                             typename pcl::PointCloud<PointT>::Ptr& cloud)
{
  const std::vector<double> z_values = range2(static_cast<double>(base_center_point.z),
                                              static_cast<double>(base_center_point.z + z_height),
                                              static_cast<double>(point_separation),
                                              true); // include end-points

  for (size_t i = 0; i < z_values.size(); ++i)
  {
    const PointT center_point(base_center_point.x,
                              base_center_point.y,
                              static_cast<float>(z_values.at(i)));
    addCircleToPointCloud<PointT>(center_point, radius, point_separation, cloud);
  }
}

template <typename PointT>
void addConeToPointCloud(const PointT base_center_point,
                         const float base_radius,
                         const float end_radius,
                         const float z_height, // +/- value
                         const float point_separation,
                         typename pcl::PointCloud<PointT>::Ptr& cloud)
{
  // Calculate the distance between slices (z_step) so the point separation
  // is correct as the cone radius increases
  const float slant_height = std::sqrt(z_height*z_height + (base_radius - end_radius)*(base_radius - end_radius));
  const unsigned int num_slant_steps = static_cast<unsigned int>(std::ceil(slant_height / point_separation));
  const float z_step = z_height / num_slant_steps;
  const std::vector<double> z_values = range2(static_cast<double>(base_center_point.z),
                                              static_cast<double>(base_center_point.z + z_height),
                                              static_cast<double>(z_step),
                                              true); // include end-points
  for (size_t i = 0; i < z_values.size(); ++i)
  {
    const PointT center_point(base_center_point.x,
                              base_center_point.y,
                              static_cast<float>(z_values.at(i)));
    const float radius = base_radius + (end_radius - base_radius) / num_slant_steps*i;
    addCircleToPointCloud<PointT>(center_point, radius, point_separation, cloud);
  }
}
// CHECK MORE UAV PATHS, with min height constraint
template <typename PointT>
void addVerticalFunnelToPointCloud(const PointT origin_point,
                                   const float offset,
                                   const float middle_radius,
                                   const float top_radius,
                                   const float cylinder_height, // +/- value
                                   const float cone_height, // +/- value
                                   const float point_separation,
                                   typename pcl::PointCloud<PointT>::Ptr& cloud)
{
  // Get dimensions of PointCloud.
  // If empty, bounds where min_z = +MaxFloat.
  const std::vector<float> bounds = getPointCloudBounds<PointT>(cloud);
  std::cout << "PointCloud bounds: " << std::endl;
  printStdVector(bounds);

  float min_z = bounds.at(4);
  std::cout << "min_z = " << min_z << std::endl;

  const PointT offset_origin_point(origin_point.x,
                                   origin_point.y,
                                   origin_point.z + offset);

  // Make sure the min z point of the cylinder is not below the
  // minimum z point of the entire PointCloud.
  const float cylinder_end_z = origin_point.z + offset + cylinder_height;

/*
  if (min_z < std::numeric_limits<float>::min() + 0.1)
  {
    std::cout << "min_z is Inf, will ignore" << std::endl;
    min_z = cylinder_end_z;
  }
*/
  std::cout << "cylinder_end_z = " << cylinder_end_z << std::endl;


  float cyl_height;
  if ((cylinder_end_z < min_z) && (min_z < std::numeric_limits<float>::max() - 0.1))
  {
    const float length_to_remove = cylinder_end_z - min_z; // can be negative value
    cyl_height = cylinder_height - length_to_remove;

    //std::cout << "a" << std::endl;
  }
  else
  {
    cyl_height = cylinder_height;

    //std::cout << "b" << std::endl;
  }

  addCylinderToPointCloud<PointT>(offset_origin_point,
                                  middle_radius,
                                  cyl_height, // length & direction
                                  point_separation,
                                  cloud);
  addConeToPointCloud<PointT>(offset_origin_point,
                              middle_radius,
                              top_radius,
                              cone_height, // length & direction
                              point_separation,
                              cloud);
}


template <typename PointT>
void addFunnelToPointCloud(const PointT origin_point,
                                   const float offset,
                                   const float middle_radius,
                                   const float top_radius,
                                   const float cone_height, // +/- value
                                   const float point_separation,
                                   typename pcl::PointCloud<PointT>::Ptr& cloud)
{
  const PointT offset_origin_point(origin_point.x,
                                  origin_point.y,
                                  origin_point.z + offset);

  addConeToPointCloud<PointT>(offset_origin_point,
                              middle_radius,
                              top_radius,
                              cone_height, // length & direction
                              point_separation,
                              cloud);
}

template <typename PointT>
void addPlaneToPointCloud(const float min_x,
                          const float max_x,
                          const float min_y,
                          const float max_y,
                          const float z_height,
                          const float point_separation,
                          typename pcl::PointCloud<PointT>::Ptr& cloud)
{
  /*
  const std::vector<float> bounds = getPointCloudBounds<PointT>(cloud);
  const float min_x = bounds.at(0);
  const float max_x = bounds.at(1);
  const float min_y = bounds.at(2);
  const float max_y = bounds.at(3);
  */
  const std::vector<double> x_values = range2(static_cast<double>(min_x),
                                              static_cast<double>(max_x),
                                              static_cast<double>(point_separation),
                                              true); // include end-points
  const std::vector<double> y_values = range2(static_cast<double>(min_y),
                                              static_cast<double>(max_y),
                                              static_cast<double>(point_separation),
                                              true); // include end-points
  for (size_t i = 0; i < x_values.size(); ++i)
  {
    for (size_t j = 0; j < y_values.size(); ++j)
    {
      const PointT point(static_cast<float>(x_values.at(i)),
                         static_cast<float>(y_values.at(j)),
                         z_height);
      cloud->points.push_back(point);
    }
  }
}



// Draw a filled polygon of points in the x-y plane of a PointCloud.
template <typename PointT>
void addPolygonToPointCloud(const std::vector<PointT>& points,
                            const float point_separation,
                            typename pcl::PointCloud<PointT>::Ptr& cloud,
                            const double z_height,
                            const double alpha) // for hull
{
  float min_x = std::numeric_limits<float>::max();
  float max_x = -1.0 * std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float max_y = -1.0 * std::numeric_limits<float>::max();
  for (std::size_t i = 0; i < points.size(); ++i)
  {
    if (points.at(i).x < min_x)
    {
      min_x = points.at(i).x;
    }
    else if (points.at(i).x > max_x)
    {
      max_x = points.at(i).x;
    }

    if (points.at(i).y < min_y)
    {
      min_y = points.at(i).y;
    }
    else if (points.at(i).y > max_y)
    {
      max_y = points.at(i).y;
    }
  }

  // Make an x-y planar region filled with points
  typename pcl::PointCloud<PointT>::Ptr temp_cloud(new pcl::PointCloud<PointT>());
  addPlaneToPointCloud<PointT>(min_x, max_x,
                               min_y, max_y,
                               z_height,
                               point_separation,
                               temp_cloud);

  std::cout << "temp_cloud points: " << temp_cloud->points.size() << std::endl;


  // Create a PointCloud representing the boundary of the polygon
  typename pcl::PointCloud<PointT>::Ptr polygon_cloud(new pcl::PointCloud<PointT>());
  for (std::size_t i = 0; i < points.size(); ++i)
  {
    PointT point;
    point.x = points.at(i).x;
    point.y = points.at(i).y;
    point.z = z_height;
    polygon_cloud->points.push_back(point);
  }

  std::cout << "polygon_cloud points: " << polygon_cloud->points.size() << std::endl;
  /*
  std::vector<pcl::Vertices> polygons;
  typename pcl::PointCloud<PointT>::Ptr hull(new pcl::PointCloud<PointT>());
  */
  /*
  //polygons.push_back(pcl::Vertices());
  polygons.resize(1);

  for (std::size_t i = 0; i < polygon_cloud->points.size(); ++i)
  {
    polygons.at(0).vertices.push_back(static_cast<uint32_t>(i));
  }
  */

  
  // TODO: Implement this
  // http://stackoverflow.com/questions/217578/how-can-i-determine-whether-a-2d-point-is-within-a-polygon

  
  /*
  // Get the indices of the points representing the concave hull of the polygon
  pcl::ConcaveHull<PointT> hull_calculator;
  hull_calculator.setInputCloud(polygon_cloud);
  //const double alpha = 0.1; //1000.0; // 1e3f
  hull_calculator.setAlpha(alpha); // smaller value = more detailed hull
  hull_calculator.reconstruct(*hull, polygons);
  std::cout << "hull points: " << hull->points.size() << std::endl;
  std::cout << "hull vertices: " << polygons.size() << std::endl;
  std::cout << "concave hull dim = " << hull_calculator.getDim() << std::endl;
  */

  /*
  // convex
  pcl::ConvexHull<PointT> hull_calculator;
  hull_calculator.setInputCloud(polygon_cloud);
  hull_calculator.reconstruct(*hull, polygons);
  std::cout << "hull points: " << hull->points.size() << std::endl;
  std::cout << "hull vertices: " << polygons.size() << std::endl;
  std::cout << "convex hull dim = " << hull_calculator.getDimension() << std::endl;

  // TODO: Make this a function

  // Remove points outside the polygon boundary
  pcl::CropHull<PointT> crop_filter;
  crop_filter.setInputCloud(temp_cloud); // The cloud to be cropped

  crop_filter.setHullCloud(hull);
  //crop_filter.setHullCloud(polygon_cloud);

  crop_filter.setHullIndices(polygons);
  crop_filter.setDim(2);
  //crop_filter.setCropOutside(true); // Remove points outside the hull

  
  typename pcl::PointCloud<PointT>::Ptr points_in_hull(new pcl::PointCloud<PointT>());
  crop_filter.filter(*points_in_hull);

  std::cout << "points_in_hull points: " << points_in_hull->points.size() << std::endl;

  // Copy the points to the output cloud
  //for (std::size_t i = 0; i < points_in_hull->points.size(); ++i)
  for (std::size_t i = 0; i < temp_cloud->points.size(); ++i)
  {
    cloud->points.push_back(points_in_hull->points.at(i));
  }
  */


  for (std::size_t i = 0; i < temp_cloud->points.size(); ++i)
  {
    if (pcl::isPointIn2DPolygon(temp_cloud->points.at(i), *polygon_cloud))
    {
      cloud->points.push_back(temp_cloud->points.at(i));
    }
  }

  //bool  pcl::isPointIn2DPolygon (const PointT &point, const pcl::PointCloud< PointT > &polygon)
}





template <typename PointT>
void addWallToPointCloud(const PointT p0,
                         const PointT p1,
                         const float point_separation,
                         typename pcl::PointCloud<PointT>::Ptr& cloud,
                         const double default_min_z = -10.0,
                         const double default_max_z = 10.0)
{
  const std::vector<float> bounds = getPointCloudBounds<PointT>(cloud);
  float min_z = bounds.at(4);
  float max_z = bounds.at(5);
  // Note: when input PointCloud is empty, getPointCloudBounds() returns limits of max Inf
  //   min_z: 3.40282e+38
  //   max_z: -3.40282e+38

  //std::cout << "In addWallToPointCloud()" << std::endl;
  //std::cout << "  min_z: " << min_z << std::endl;
  //std::cout << "  max_z: " << max_z << std::endl;

  if (min_z > 10000.0)

  {
    //std::cout << "min_z is Inf, will ignore" << std::endl;
    min_z = default_min_z;
  }

  if (max_z < -10000.0)
  {
    //std::cout << "1 max_z is Inf, will ignore" << std::endl;
    max_z = default_max_z;
  }

  //std::cout << "  min_z: " << min_z << std::endl;
  //std::cout << "  max_z: " << max_z << std::endl;


/*
  const std::vector<double> x_values = range2(static_cast<double>(p0.x),
                                              static_cast<double>(p1.x),
                                              static_cast<double>(point_separation),
                                              true); // include end-points
  const std::vector<double> y_values = range2(static_cast<double>(p0.y),
                                              static_cast<double>(p1.y),
                                              static_cast<double>(point_separation),
                                              true); // include end-points
                                              */
  const std::vector<double> z_values = range2(static_cast<double>(min_z),
                                              static_cast<double>(max_z),
                                              static_cast<double>(point_separation),
                                              false); // Don't include end-points

  //std::cout << "  x_values: " << x_values.size() << std::endl;
  //std::cout << "  y_values: " << y_values.size() << std::endl;


 // if (x_values.size() != y_values.size())
  //{
  //  throw std::runtime_error("x_values and y_values are different sizes in addWallToPointCloud()");
  //}

  const std::vector<double> p0_xy = {p0.x, p0.y,  0.0};
  const std::vector<double> p1_xy = {p1.x, p1.y,  0.0};
  const std::vector<std::vector<double> > xy_points = lerp(p0_xy, p1_xy, point_separation);

  for (size_t i = 0; i < xy_points.size(); ++i)
  {
    for (size_t j = 0; j < z_values.size(); ++j)
    {
      const PointT point(static_cast<float>(xy_points.at(i).at(0)),
                         static_cast<float>(xy_points.at(i).at(1)),
                         static_cast<float>(z_values.at(j)));
      cloud->points.push_back(point);
    }
  }
/*
  // Lay points in order of z value
  for (size_t k = 0; k < z_values.size(); ++k)
  {
    for (size_t i = 0; i < x_values.size(); ++i)
    {
      for (size_t j = 0; j < y_values.size(); ++j)
      {
        const PointT point(static_cast<float>(x_values.at(i)),
                           static_cast<float>(y_values.at(j)),
                           static_cast<float>(z_values.at(k)));
        cloud->points.push_back(point);
      }
    }
  }
  */
}



template <typename PointT>
void addWallsToPointCloud(const std::vector<std::vector<PointT> >& waypoints,
                          const float point_separation,
                          typename pcl::PointCloud<PointT>::Ptr& cloud,
                          const bool connect_endpoints,
                          const double default_min_z = -10.0,
                          const double default_max_z = 10.0)
{
  for (std::size_t i = 0; i < waypoints.size(); ++i)
  {
    for (std::size_t j = 0; j < (waypoints.at(i).size() - 1); ++j)
    {
      //std::cout << "Adding wall from " << waypoints.at(i).at(j).x << ", " << waypoints.at(i).at(j).y << " to "
      //                                 << waypoints.at(i).at(j+1).x << ", " << waypoints.at(i).at(j+1).y << std::endl;
      addWallToPointCloud<PointT>(PointT(waypoints.at(i).at(j)),
                                  PointT(waypoints.at(i).at(j+1)),
                                  point_separation,
                                  cloud,
                                  default_min_z, default_max_z); // z values
    }

    if (connect_endpoints)
    {
      // Join first and last points to make an enclosed loop of walls
      const unsigned int num_pts = waypoints.at(i).size();
      addWallToPointCloud<PointT>(PointT(waypoints.at(i).at(num_pts-1)),
                                  PointT(waypoints.at(i).at(0)),
                                  point_separation,
                                  cloud,
                                  default_min_z, default_max_z); // z values
    }
  }
}


template <typename PointT1, typename PointT2>
const pcl::PointCloud<PointT2> getColoredPointCloud(const unsigned int color_r,
                                                    const unsigned int color_g,
                          const unsigned int color_b,
                          typename pcl::PointCloud<PointT1>::Ptr& cloud_in)
{
  pcl::PointCloud<PointT2> cloud_out;

  for (std::size_t i = 0; i < cloud_in->points.size(); ++i)
  {
    PointT2 point;
    point.x = cloud_in->points.at(i).x;
    point.y = cloud_in->points.at(i).y;
    point.z = cloud_in->points.at(i).z;

    point.r = static_cast<pcl::uint8_t>(color_r); // uint_8
    point.g = static_cast<pcl::uint8_t>(color_g);;
    point.b = static_cast<pcl::uint8_t>(color_b);;

    cloud_out.push_back(point);
  }

  return cloud_out;
}


template<typename PointT>
inline int savePLYFile(const std::string& file_name, const pcl::PointCloud<PointT>& cloud)
{
  pcl::PLYWriter w;
  const bool binary = false;
  const bool use_camera = false; // don't write extra scalar fields
  return (w.write<PointT>(file_name, cloud, binary, use_camera));
}

template<typename PointT>
const float euclideanDistanceXY(const PointT& p0, const PointT& p1)
{
  const float dx = p1.x - p0.x;
  const float dy = p1.y - p0.y;

  return std::sqrt(dx*dx + dy*dy);
}

#endif // PCL_UTILS_IMPL_HPP
