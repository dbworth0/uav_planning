/*
Example 11:

Find path through a PointCloud using RRT algorithms
from OMPL (Open Motion Planning Libray).
The PointCloud is converted into a PCL Octree, and collision-checking is
performed by checking for points within a sphere (the robot's radius).
The paths are smoothed using various methods:
 - Parabolas
 - Catmull-Rom Splines (uniform, chordal, centripetal)
A .ply file is written for each type of smoothed path.

Usage:
export PATH=$PATH:~/path/to/your/catkin_workspace/devel/lib/planning_through_pointclouds/
cd ~/path/to/your/catkin_workspace/planning_through_pointclouds/data/
ex11_plan_through_pointcloud_ompl_octree --input ./cropped_cloud_1.ply --start 67.0 2.0 -2.0 --goal 65.0 39.0 -1.0 \
--planner RRT --max_planning_time 5.0 --output ~/path_out_11_1_rrt.ply

Or to let BIT* find the first solution:
ex11_plan_through_pointcloud_ompl_octree --input ./cropped_cloud_1.ply --start 67.0 2.0 -2.0 --goal 65.0 39.0 -1.0 \
--planner BIT_STAR --max_planning_time -1 --output ~/path_out_11_2_bitstar.ply

Or a larger PointCloud:
ex11_plan_through_pointcloud_ompl_octree --input ./cropped_cloud_2.ply --start 13.0 20.0 -0.8 --goal 100.0 60.0 0.0 \
--planner BIT_STAR --max_planning_time -1 --output ~/path_out_11_3_bitstar.ply

Or complete PointCloud:
ex11_plan_through_pointcloud_ompl_octree --input ./complete_cloud_1.pcd --start -12.0 -2.0 2.0 --goal 133.0 97.0 2.5 \
--planner BIT_STAR --max_planning_time -1 --output ~/path_out_11_4_bitstar.ply


Churchill Club:

ex11_plan_through_pointcloud_ompl_octree --input ./pointcloud4-Churchill_Country_Club.ply --start 0.0 0.0 2.0 --goal 90.0 -25.0 1.0 \
--planner BIT_STAR --max_planning_time 10.0 --output ~/path_out_14_churchill_2.ply

ex11_plan_through_pointcloud_ompl_octree --input ./pointcloud4-Churchill_Country_Club.ply --start 0.0 0.0 5.0 --goal 90.0 -25.0 5.0 \
--planner BIT_STAR --max_planning_time 10.0 --output ~/path_out_14_churchill_1.ply


Indoor:

ex11_plan_through_pointcloud_ompl_octree --input ./pointcloud5_indoor_highbay_no_floor.pcd --start -2.0 4.2 0.2 --goal 23.9 -1.8 0.2 \
--planner BIT_STAR --max_planning_time 5.0 --output ~/path_out_11_indoor_bitstar.ply


David Butterworth
*/

#include <iostream> // cout
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp> // to_upper

#include <planning_through_pointclouds/utils.h> // processCommandLine, Timer
//#include <planning_through_pointclouds/ompl_utils.h> // getPlannerGraphMarker
#include <planning_through_pointclouds/pcl_utils.h> // loadPointCloud, getPathPointCloudFromPoints
#include <planning_through_pointclouds/octree_map_3d_v2_ompl.h>

#include <pcl/io/ply_io.h> // savePLYFileASCII

#include <spline_library/vector.h>
#include <spline_library/hermite/cubic/cubic_hermite_spline.h>
#include <spline_library/hermite/cubic/uniform_cr_spline.h>

//----------------------------------------------------------------------------//

int main(int argc, char **argv)
{
  std::string pointcloud_file;
  std::vector<float> start;
  std::vector<float> goal;
  std::string planner;
  double max_planning_time;
  std::string output_filename;

  const bool result = processCommandLine(argc, argv, pointcloud_file, start, goal, planner, max_planning_time, output_filename);
  if (!result)
  {
    return EXIT_FAILURE;
  }
  std::cout << "pointcloud_file =  "<< pointcloud_file << std::endl;
  std::cout << "output_filename =  "<< output_filename << std::endl;
  const std::vector<std::string> output_filename_tokens = splitString(output_filename);

  PointCloud::Ptr input_pointcloud(new PointCloud());
  if (!loadPointCloud(pointcloud_file, *input_pointcloud))
  {
    return EXIT_FAILURE;
  }
  std::cout << "Loaded PointCloud with " << input_pointcloud->points.size() << " points" << std::endl;

  Timer timer;

  std::cout << "Initializing OctreeMap3D..." << std::endl;
  timer.reset();

  // Half the diameter of DJI S1000.
  const double robot_radius = 1.1 / 2.0;

  const double voxel_size = 0.10;
  OctreeMap3D map(input_pointcloud, voxel_size, robot_radius);
  std::cout << "  took " << timer.read() << " ms." << std::endl;

  const Point start_point(start.at(0), start.at(1), start.at(2));
  const Point goal_point(goal.at(0), goal.at(1), goal.at(2));

  std::cout << "Finding path to goal..." << std::endl;

  boost::to_upper(planner);
  const PLANNER_TYPE planner_type = stringToEnum(planner);
  const bool interpolate_path = true;
  const double waypoint_separation = 0.0; // 0 = output waypoints at the collision-checking resolution, which is 10cm
  timer.reset();
  const std::vector<Point> path = map.findPath(start_point, goal_point, planner_type, max_planning_time, interpolate_path, waypoint_separation);
  const double time_taken = timer.read();
  std::cout << "  took " << time_taken << " ms." << std::endl;
  std::cout << "    Path length: " << map.getPathLength() << std::endl;
  std::cout << "    No. waypoints: " << path.size() << std::endl;

  const uint num_checks = map.getNumCollisionChecks();
  std::cout << "    No. of collision checks: " << num_checks << " (" << time_taken/num_checks << " ms per check)" << std::endl;

  ColorPointCloud path_pointcloud;
  std::string outfile;

  // Save trajectory
  path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(path);
  outfile = output_filename_tokens[0] + std::string("_orig.ply");
  std::cout << "Writing to " << outfile << std::endl;
  pcl::io::savePLYFileASCII(outfile, path_pointcloud);


  // Fit parabola to waypoints
  // This uses Hauser Parabolic Path Smoother
  {
    std::cout << "Short-cutting path with parabolic curves..." << std::endl;
    timer.reset();
    const std::vector<Point> parabolic_path = map.getParabolicPath();
    std::cout << "  took " << timer.read() << " ms." << std::endl;
    std::cout << "Parabolic path now has " << parabolic_path.size() << " waypoints" << std::endl;
    // Save trajectory
    path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(parabolic_path);
    outfile = output_filename_tokens[0] + std::string("_parabolic.ply");
    std::cout << "Writing to " << outfile << std::endl;
    pcl::io::savePLYFileASCII(outfile, path_pointcloud);
  }


  // Reduce the number of vertices in the path:
  std::cout << "Simplifying path..." << std::endl;
  timer.reset();
  const std::vector<Point> simplified_path = map.getReducedPath();
  std::cout << "  took " << timer.read() << " ms." << std::endl;
  std::cout << "Simplified path now has " << simplified_path.size() << " waypoints" << std::endl;
  // Save trajectory
  path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(simplified_path);
  outfile = output_filename_tokens[0] + std::string("_simplified.ply");
  std::cout << "Writing to " << outfile << std::endl;
  pcl::io::savePLYFileASCII(outfile, path_pointcloud);


  // Fit splines (Uniform Catmull-Rom) between waypoints
  {
    std::vector<spline_library::Vector3> points;
    for (size_t i = 0; i < simplified_path.size(); ++i)
    {
      points.push_back(spline_library::Vector3({simplified_path.at(i).x, simplified_path.at(i).y, simplified_path.at(i).z}));
    }
    // Add the first and last waypoints twice, with a small position change, so
    // the spline is generated correctly.
    const float extend_length = 0.05; // large enough step to calculate tangent at the endpoints
    spline_library::Vector3 first_point = points.at(0) - points.at(1);
    first_point  = points.at(0) + first_point.normalized() * extend_length;
    spline_library::Vector3 last_point = points.at(points.size()-1) - points.at(points.size()-2);
    last_point  = points.at(points.size()-1) + last_point.normalized() * extend_length;

    //std::cout << "  " << first_point[0] << "," << first_point[1] << "," << first_point[2] << std::endl;
    //std::cout << "  " << last_point[0] << "," << last_point[1] << "," << last_point[2] << std::endl;

    //std::cout << "  " << points.at(0)[0] << "," << points.at(0)[1] << "," << points.at(0)[2] << std::endl;
    //std::cout << "  " << points.back()[0] << "," << points.back()[1] << "," << points.back()[2] << std::endl;

    points.insert(points.begin(), first_point);
    points.push_back(last_point);

    //std::cout << "  " << points.at(0)[0] << "," << points.at(0)[1] << "," << points.at(0)[2] << std::endl;
    //std::cout << "  " << points.back()[0] << "," << points.back()[1] << "," << points.back()[2] << std::endl;

    std::shared_ptr<spline_library::Spline<spline_library::Vector3> > spline = std::make_shared<spline_library::UniformCRSpline<spline_library::Vector3> >(points);
    const unsigned int num_points = 2000;
    //std::cout << "spline duration = " << spline->getMaxT() << std::endl;
    std::vector<Point> splined_path;
    for (int i = 0; i < num_points; ++i)
    {
      const float t = static_cast<float>(i) * spline->getMaxT() / (num_points - 1);
      const spline_library::Vector3 point = spline->getPosition(t);
      //std::cout << "  t = " << t << "  " << point[0] << "," << point[1] << "," << point[2] << std::endl;
      splined_path.push_back(Point(point[0], point[1], point[2]));
    }
    std::cout << "Splined path now has " << splined_path.size() << " waypoints" << std::endl;

    // Save trajectory
    path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(splined_path);
    outfile = output_filename_tokens[0] + std::string("_splined_UniformCR.ply");
    std::cout << "Writing to " << outfile << std::endl;
    pcl::io::savePLYFileASCII(outfile, path_pointcloud);
  }


  // Fit splines (Centripetal Catmull-Rom) between waypoints
  // Use this one, curve is tighter than Uniform or Chordal types.
  {
    std::vector<spline_library::Vector3> points;
    for (size_t i = 0; i < simplified_path.size(); ++i)
    {
      points.push_back(spline_library::Vector3({simplified_path.at(i).x, simplified_path.at(i).y, simplified_path.at(i).z}));
    }
    // Add the first and last waypoints twice, with a small position change, so
    // the spline is generated correctly.
    const float extend_length = 0.05; // large enough step to calculate tangent at the endpoints
    spline_library::Vector3 first_point = points.at(0) - points.at(1);
    first_point  = points.at(0) + first_point.normalized() * extend_length;
    spline_library::Vector3 last_point = points.at(points.size()-1) - points.at(points.size()-2);
    last_point  = points.at(points.size()-1) + last_point.normalized() * extend_length;

    //std::cout << "  " << first_point[0] << "," << first_point[1] << "," << first_point[2] << std::endl;
    //std::cout << "  " << last_point[0] << "," << last_point[1] << "," << last_point[2] << std::endl;

    //std::cout << "  " << points.at(0)[0] << "," << points.at(0)[1] << "," << points.at(0)[2] << std::endl;
    //std::cout << "  " << points.back()[0] << "," << points.back()[1] << "," << points.back()[2] << std::endl;

    points.insert(points.begin(), first_point);
    points.push_back(last_point);

    //std::cout << "  " << points.at(0)[0] << "," << points.at(0)[1] << "," << points.at(0)[2] << std::endl;
    //std::cout << "  " << points.back()[0] << "," << points.back()[1] << "," << points.back()[2] << std::endl;

    const float alpha = 0.5; // Centripetal Catmull-Rom Spline
    std::shared_ptr<spline_library::Spline<spline_library::Vector3> > spline = std::make_shared<spline_library::CubicHermiteSpline<spline_library::Vector3> >(points, alpha);

    const unsigned int num_points = 2000;

    //std::cout << "spline duration = " << spline->getMaxT() << std::endl;
    std::vector<Point> splined_path;
    for (int i = 0; i < num_points; ++i)
    {
      const float t = static_cast<float>(i) * spline->getMaxT() / (num_points - 1);
      const spline_library::Vector3 point = spline->getPosition(t);
      //std::cout << "  t = " << t << "  " << point[0] << "," << point[1] << "," << point[2] << std::endl;
      splined_path.push_back(Point(point[0], point[1], point[2]));
    }
    std::cout << "Splined path now has " << splined_path.size() << " waypoints" << std::endl;

    // Save trajectory
    path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(splined_path);
    outfile = output_filename_tokens[0] + std::string("_splined_CentripetalCR.ply");
    std::cout << "Writing to " << outfile << std::endl;
    pcl::io::savePLYFileASCII(outfile, path_pointcloud);
  }


  // Fit splines (Chordal Catmull-Rom) between waypoints
  {
    std::vector<spline_library::Vector3> points;
    for (size_t i = 0; i < simplified_path.size(); ++i)
    {
      points.push_back(spline_library::Vector3({simplified_path.at(i).x, simplified_path.at(i).y, simplified_path.at(i).z}));
    }
    // Add the first and last waypoints twice, with a small position change, so
    // the spline is generated correctly.
    const float extend_length = 0.05; // large enough step to calculate tangent at the endpoints
    spline_library::Vector3 first_point = points.at(0) - points.at(1);
    first_point  = points.at(0) + first_point.normalized() * extend_length;
    spline_library::Vector3 last_point = points.at(points.size()-1) - points.at(points.size()-2);
    last_point  = points.at(points.size()-1) + last_point.normalized() * extend_length;

    //std::cout << "  " << first_point[0] << "," << first_point[1] << "," << first_point[2] << std::endl;
    //std::cout << "  " << last_point[0] << "," << last_point[1] << "," << last_point[2] << std::endl;

    //std::cout << "  " << points.at(0)[0] << "," << points.at(0)[1] << "," << points.at(0)[2] << std::endl;
    //std::cout << "  " << points.back()[0] << "," << points.back()[1] << "," << points.back()[2] << std::endl;

    points.insert(points.begin(), first_point);
    points.push_back(last_point);

    //std::cout << "  " << points.at(0)[0] << "," << points.at(0)[1] << "," << points.at(0)[2] << std::endl;
    //std::cout << "  " << points.back()[0] << "," << points.back()[1] << "," << points.back()[2] << std::endl;

    const float alpha = 1.0; // Chordal Catmull-Rom Spline
    std::shared_ptr<spline_library::Spline<spline_library::Vector3> > spline = std::make_shared<spline_library::CubicHermiteSpline<spline_library::Vector3> >(points, alpha);

    const unsigned int num_points = 2000;
    //std::cout << "spline duration = " << spline->getMaxT() << std::endl;
    std::vector<Point> splined_path;
    for (int i = 0; i < num_points; ++i)
    {
      const float t = static_cast<float>(i) * spline->getMaxT() / (num_points - 1);
      const spline_library::Vector3 point = spline->getPosition(t);
      //std::cout << "  t = " << t << "  " << point[0] << "," << point[1] << "," << point[2] << std::endl;
      splined_path.push_back(Point(point[0], point[1], point[2]));
    }
    std::cout << "Splined path now has " << splined_path.size() << " waypoints" << std::endl;

    // Save trajectory
    path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(splined_path);
    outfile = output_filename_tokens[0] + std::string("_splined_ChordalCR.ply");
    std::cout << "Writing to " << outfile << std::endl;
    pcl::io::savePLYFileASCII(outfile, path_pointcloud);
  }

  return 0;
}
