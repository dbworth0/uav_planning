/*
Example 12:

Find path through a PointCloud using RRT algorithms
from OMPL (Open Motion Planning Libray).
The PointCloud is converted into an OctoMap, and collision-checking is performed
between a sphere and the OcTree using FCL (Flexible Collision Library).
The final path is written out as a .ply file.

Usage:
export PATH=$PATH:~/path/to/your/catkin_workspace/devel/lib/planning_through_pointclouds/
cd ~/path/to/your/catkin_workspace/planning_through_pointclouds/data/

ex12_plan_through_pointcloud_ompl_octomap --input ./cropped_cloud_1.ply --start 67.0 2.0 -2.0 --goal 65.0 39.0 -1.0 \
--planner BIT_STAR --max_planning_time -1 --output ~/path_out_15_2_bitstar.ply

ex12_plan_through_pointcloud_ompl_octomap --input ./cropped_cloud_2.ply --start 13.0 20.0 -0.8 --goal 100.0 60.0 0.0 \
--planner BIT_STAR --max_planning_time -1 --output ~/path_out_16_2_bitstar.ply

ex12_plan_through_pointcloud_ompl_octomap --input ./complete_cloud_1.pcd --start -12.0 -2.0 2.0 --goal 133.0 97.0 2.5 \
--planner BIT_STAR --max_planning_time -1 --output ~/path_out_17_2_bitstar.ply


David Butterworth
*/

#include <iostream> // cout
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp> // to_upper

#include <planning_through_pointclouds/utils.h> // processCommandLine, Timer
//#include <planning_through_pointclouds/ompl_utils.h> // getPlannerGraphMarker
#include <planning_through_pointclouds/pcl_utils.h> // loadPointCloud, getPathPointCloudFromPoints
#include <planning_through_pointclouds/octomap_map_3d_v1_ompl_fcl.h>

#include <pcl/io/ply_io.h> // savePLYFileASCII

//----------------------------------------------------------------------------//

int main(int argc, char **argv)
{
  std::string pointcloud_file;
  std::vector<float> start;
  std::vector<float> goal;
  std::string planner;
  double max_planning_time;
  std::string output_path;

  const bool result = processCommandLine(argc, argv, pointcloud_file, start, goal, planner, max_planning_time, output_path);
  if (!result)
  {
    return EXIT_FAILURE;
  }
  std::cout << "pointcloud_file =  "<< pointcloud_file << std::endl;
  std::cout << "output_path =  "<< output_path << std::endl;

  PointCloud::Ptr input_pointcloud(new PointCloud());
  if (!loadPointCloud(pointcloud_file, *input_pointcloud))
  {
    return EXIT_FAILURE;
  }
  std::cout << "Loaded PointCloud with " << input_pointcloud->points.size() << " points" << std::endl;

  Timer timer;

  std::cout << "Initializing OctomapMap3D..." << std::endl;
  timer.reset();

  // Half the diameter of DJI S1000.
  const double robot_radius = 1.1 / 2.0;
  const double voxel_size = 0.10;
  OctomapMap3D map(input_pointcloud, voxel_size, robot_radius);
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
  std::cout << "    Path length " << map.getPathLength() << std::endl;
  const uint num_checks = map.getNumCollisionChecks();
  std::cout << "    No. of collision checks: " << num_checks << " (" << time_taken/num_checks << " ms per check)" << std::endl;

  std::cout << "Simplifying path..." << std::endl;
  timer.reset();
  const std::vector<Point> path_simplified = map.getSimplifiedPath();
  std::cout << "  took " << timer.read() << " ms." << std::endl;

  // Save the planned trajectory
  const ColorPointCloud path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(path);
  pcl::io::savePLYFileASCII(output_path, path_pointcloud); 

  return 0;
}
