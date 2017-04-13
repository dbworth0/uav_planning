/*
Example 31:

Test creating splines with different curvatures.


David Butterworth
*/

#include <iostream> // cout
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp> // to_upper

//#include <planning_through_pointclouds/utils.h> // processCommandLine, Timer
//#include <planning_through_pointclouds/ompl_utils.h> // getPlannerGraphMarker
#include <planning_through_pointclouds/pcl_utils.h> // getPathPointCloudFromPoints
//#include <planning_through_pointclouds/octree_map_3d_v2_ompl.h>

#include <pcl/io/ply_io.h> // savePLYFileASCII

#include <spline_library/vector.h>
#include <spline_library/hermite/cubic/cubic_hermite_spline.h>
#include <spline_library/hermite/cubic/uniform_cr_spline.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointXYZRGBA ColorPoint;
typedef pcl::PointCloud<ColorPoint> ColorPointCloud;

// Check which side of line that a point is on:
// http://math.stackexchange.com/questions/274712/calculate-on-which-side-of-a-straight-line-is-a-given-point-located

// For a line between two points, make a smooth curve that
// passes through a point offset from the line.
//
// This assumes the path is mainly x,y motion with no large changes in z altitude.
template <typename PointT>
std::shared_ptr<spline_library::Spline<spline_library::Vector3> > 
makeCurveOffsetFromLine(const PointT& start, const PointT& end, const double offset, const unsigned int type)
{

// horizon
  const PointT midpoint = PointT(start.x + ((end.x - start.x) / 2.0),
                                 start.y + ((end.y - start.y) / 2.0),
                                 start.z + ((end.z - start.z) / 2.0));

  std::cout << "Making curve: " << std::endl;
  std::cout << "  start: " << start.x << ", " << start.y << ", " << start.z << std::endl;
  std::cout << "  end: " << end.x << ", " << end.y << ", " << end.z << std::endl;
  std::cout << "  mid-point: " << midpoint.x << ", " << midpoint.y << ", " << midpoint.z << std::endl;

  const float dx = end.x - start.x;
  const float dy = end.y - start.y;
  const float dist = std::sqrt(dx*dx + dy*dy);
  const float dx_perp = dx / dist;
  const float dy_perp = dy / dist;

  // Find two offset points on horizontal x,y plane:
  //
  // Change this to (+ - - +) to flip the direction
  const Point offset_point_1 = PointT(midpoint.x - offset*dy_perp,
                                      midpoint.y + offset*dx_perp,
                                      midpoint.z);
  const Point offset_point_5 = PointT(midpoint.x + offset*dy_perp,
                                      midpoint.y - offset*dx_perp,
                                      midpoint.z);

  // Offset point in vertical direction
  const Point offset_point_3 = PointT(midpoint.x,
                                      midpoint.y,
                                      midpoint.z + offset);

  const Point offset_2_dir_midpoint = Point(offset_point_1.x + ((offset_point_3.x - offset_point_1.x) / 2.0),
                                            offset_point_1.y + ((offset_point_3.y - offset_point_1.y) / 2.0),
                                            offset_point_1.z + ((offset_point_3.z - offset_point_1.z) / 2.0));
  const Point offset_4_dir_midpoint = Point(offset_point_5.x + ((offset_point_3.x - offset_point_5.x) / 2.0),
                                            offset_point_5.y + ((offset_point_3.y - offset_point_5.y) / 2.0),
                                            offset_point_5.z + ((offset_point_3.z - offset_point_5.z) / 2.0));
  const float offset_2_dir_midpoint_distance = std::sqrt((offset_2_dir_midpoint.x - midpoint.x)*(offset_2_dir_midpoint.x - midpoint.x)
                                                         + (offset_2_dir_midpoint.y - midpoint.y)*(offset_2_dir_midpoint.y - midpoint.y)
                                                         + (offset_2_dir_midpoint.z - midpoint.z)*(offset_2_dir_midpoint.z - midpoint.z));
  const float offset_4_dir_midpoint_distance = std::sqrt((offset_4_dir_midpoint.x - midpoint.x)*(offset_4_dir_midpoint.x - midpoint.x)
                                                         + (offset_4_dir_midpoint.y - midpoint.y)*(offset_4_dir_midpoint.y - midpoint.y)
                                                         + (offset_4_dir_midpoint.z - midpoint.z)*(offset_4_dir_midpoint.z - midpoint.z));

  // Make unit vectors in desired direction
  const Point offset_2_dir_vector = Point((offset_2_dir_midpoint.x - midpoint.x) / offset_2_dir_midpoint_distance,
                                          (offset_2_dir_midpoint.y - midpoint.y) / offset_2_dir_midpoint_distance,
                                          (offset_2_dir_midpoint.z - midpoint.z) / offset_2_dir_midpoint_distance);
  const Point offset_4_dir_vector = Point((offset_4_dir_midpoint.x - midpoint.x) / offset_4_dir_midpoint_distance,
                                          (offset_4_dir_midpoint.y - midpoint.y) / offset_4_dir_midpoint_distance,
                                          (offset_4_dir_midpoint.z - midpoint.z) / offset_4_dir_midpoint_distance);

  // Create two offset points at 45 degree angles from vertical
  const Point offset_point_2 = Point(midpoint.x + offset*offset_2_dir_vector.x,
                                     midpoint.y + offset*offset_2_dir_vector.y,
                                     midpoint.z + offset*offset_2_dir_vector.z);
  const Point offset_point_4 = Point(midpoint.x + offset*offset_4_dir_vector.x,
                                     midpoint.y + offset*offset_4_dir_vector.y,
                                     midpoint.z + offset*offset_4_dir_vector.z);

  std::cout << "  offset point 1: " << offset_point_1.x << ", " << offset_point_1.y << ", " << offset_point_1.z << std::endl;
  std::cout << "  offset point 2: " << offset_point_2.x << ", " << offset_point_2.y << ", " << offset_point_2.z << std::endl;
  std::cout << "  offset point 3: " << offset_point_3.x << ", " << offset_point_3.y << ", " << offset_point_3.z << std::endl;
  std::cout << "  offset point 4: " << offset_point_4.x << ", " << offset_point_4.y << ", " << offset_point_4.z << std::endl;
  std::cout << "  offset point 5: " << offset_point_5.x << ", " << offset_point_5.y << ", " << offset_point_5.z << std::endl;
  
  std::vector<spline_library::Vector3> points;
  points.push_back(spline_library::Vector3({start.x, start.y, start.z}));
  if (type == 1)
  {
    points.push_back(spline_library::Vector3({offset_point_1.x, offset_point_1.y, offset_point_1.z}));
  }
  else if (type == 2)
  {
    points.push_back(spline_library::Vector3({offset_point_2.x, offset_point_2.y, offset_point_2.z}));
  }
  else if (type == 3)
  {
    points.push_back(spline_library::Vector3({offset_point_3.x, offset_point_3.y, offset_point_3.z}));
  }
  else if (type == 4)
  {
    points.push_back(spline_library::Vector3({offset_point_4.x, offset_point_4.y, offset_point_4.z}));
  }
  else if (type == 5)
  {
    points.push_back(spline_library::Vector3({offset_point_5.x, offset_point_5.y, offset_point_5.z}));
  }
  points.push_back(spline_library::Vector3({end.x, end.y, end.z}));

  // Add the first and last waypoints twice, with a small position change,
  // so the spline is generated correctly.
  // A smaller value creates a smoother curve.
  const float extend_length = 0.05; // large enough step to calculate tangent at the endpoints

  spline_library::Vector3 new_first_point = points.at(0) - points.at(2);
  new_first_point  = points.at(0) + new_first_point.normalized() * extend_length;

  spline_library::Vector3 new_last_point = points.at(2) - points.at(0);
  new_last_point  = points.at(points.size()-1) + new_last_point.normalized() * extend_length;

  points.insert(points.begin(), new_first_point);
  points.push_back(new_last_point);

  const float alpha = 1.0; // Chordal Catmull-Rom Spline
  std::shared_ptr<spline_library::Spline<spline_library::Vector3> > spline = std::make_shared<spline_library::CubicHermiteSpline<spline_library::Vector3> >(points, alpha);

  return spline;
}


//----------------------------------------------------------------------------//

int main(int argc, char **argv)
{
  const std::string path_to_save_files("/home/dbworth/");

  for (unsigned int type = 1; type <= 5; ++type)
  {
    std::shared_ptr<spline_library::Spline<spline_library::Vector3> > spline = makeCurveOffsetFromLine(Point(74.017, 3.295, 5.020),
                                                                                                       Point(122.811, 3.506, 6.173),
                                                                                                       8.0,
                                                                                                       type);
    const unsigned int num_points = 2000;
    std::vector<Point> splined_path;
    for (int i = 0; i < num_points; ++i)
    {
      const float t = static_cast<float>(i) * spline->getMaxT() / (num_points - 1);
      const spline_library::Vector3 point = spline->getPosition(t);
      splined_path.push_back(Point(point[0], point[1], point[2]));
    }
    std::cout << "Splined path now has " << splined_path.size() << " waypoints" << std::endl;
    const ColorPointCloud path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(splined_path);
    std::stringstream ss;
    ss << type;
    const std::string type_str = ss.str();
    const std::string outfile = path_to_save_files + std::string("test_ChordalCR_") + type_str + std::string(".ply");
    std::cout << "Writing to " << outfile << std::endl;
    pcl::io::savePLYFileASCII(outfile, path_pointcloud);
  }

  /*

  std::vector<spline_library::Vector3> points;
  points.push_back(spline_library::Vector3({-0.10, 0, 5.389}));
  points.push_back(spline_library::Vector3({0, 0, 5.389}));
  points.push_back(spline_library::Vector3({25, 8, 5.389}));
  points.push_back(spline_library::Vector3({50, 0, 5.389}));
  points.push_back(spline_library::Vector3({50.1, 0, 5.389}));
    
  const std::string path_to_save_files("/home/dbworth/");

  // Uniform Catmull-Rom spline
  {
    std::shared_ptr<spline_library::Spline<spline_library::Vector3> > spline = std::make_shared<spline_library::UniformCRSpline<spline_library::Vector3> >(points);
    const unsigned int num_points = 2000;
    std::vector<Point> splined_path;
    for (int i = 0; i < num_points; ++i)
    {
      const float t = static_cast<float>(i) * spline->getMaxT() / (num_points - 1);
      const spline_library::Vector3 point = spline->getPosition(t);
      splined_path.push_back(Point(point[0], point[1], point[2]));
    }
    std::cout << "Splined path now has " << splined_path.size() << " waypoints" << std::endl;
    const ColorPointCloud path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(splined_path);
    const std::string outfile = path_to_save_files + std::string("test_UniformCR.ply");
    std::cout << "Writing to " << outfile << std::endl;
    pcl::io::savePLYFileASCII(outfile, path_pointcloud);
  }

  // Centripetal Catmull-Rom spline
  {
    //std::shared_ptr<spline_library::Spline<spline_library::Vector3> > spline = std::make_shared<spline_library::UniformCRSpline<spline_library::Vector3> >(points);

    const float alpha = 0.5; // Centripetal Catmull-Rom Spline
    std::shared_ptr<spline_library::Spline<spline_library::Vector3> > spline = std::make_shared<spline_library::CubicHermiteSpline<spline_library::Vector3> >(points, alpha);


    const unsigned int num_points = 2000;
    std::vector<Point> splined_path;
    for (int i = 0; i < num_points; ++i)
    {
      const float t = static_cast<float>(i) * spline->getMaxT() / (num_points - 1);
      const spline_library::Vector3 point = spline->getPosition(t);
      splined_path.push_back(Point(point[0], point[1], point[2]));
    }
    std::cout << "Splined path now has " << splined_path.size() << " waypoints" << std::endl;
    const ColorPointCloud path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(splined_path);
    const std::string outfile = path_to_save_files + std::string("test_CentripetalCR.ply");
    std::cout << "Writing to " << outfile << std::endl;
    pcl::io::savePLYFileASCII(outfile, path_pointcloud);
  }

  // Chordal Catmull-Rom spline
  {
    //std::shared_ptr<spline_library::Spline<spline_library::Vector3> > spline = std::make_shared<spline_library::UniformCRSpline<spline_library::Vector3> >(points);

    const float alpha = 1.0; // Chordal Catmull-Rom Spline
    std::shared_ptr<spline_library::Spline<spline_library::Vector3> > spline = std::make_shared<spline_library::CubicHermiteSpline<spline_library::Vector3> >(points, alpha);


    const unsigned int num_points = 2000;
    std::vector<Point> splined_path;
    for (int i = 0; i < num_points; ++i)
    {
      const float t = static_cast<float>(i) * spline->getMaxT() / (num_points - 1);
      const spline_library::Vector3 point = spline->getPosition(t);
      splined_path.push_back(Point(point[0], point[1], point[2]));
    }
    std::cout << "Splined path now has " << splined_path.size() << " waypoints" << std::endl;
    const ColorPointCloud path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(splined_path);
    const std::string outfile = path_to_save_files + std::string("test_ChordalCR.ply");
    std::cout << "Writing to " << outfile << std::endl;
    pcl::io::savePLYFileASCII(outfile, path_pointcloud);
  }
  */


  return 0;
}
