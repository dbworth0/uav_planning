/*
Example 13:

Find path through a PointCloud using RRT algorithms
from OMPL (Open Motion Planning Libray).

The PointCloud is converted into a PCL Octree, and collision-checking is
performed by checking for points within a sphere (the robot's radius).
The final path is written out as a .ply file.

This uses a custom State space (x,y,elevation) similar to Dubins Airplane,
with variable turning radius and slope constraint.

Planners include RRT*, BIT*, or a custom Task-space RRT*.
The Task-space RRT* treats the angle separately to reduce computation,
instead of planning in the full state-space, but the resulting paths
are longer than normal RRT*.

Has optional min height (elevation) constraint.
If set, the path will have a vertical linear "take off" segment until
it reaches the min height, planning occurs within the height constraint,
then the path finishes with a vertical linear "landing" segment.


Usage:
export PATH=$PATH:~/path/to/your/catkin_workspace/devel/lib/planning_through_pointclouds/
cd ~/path/to/your/catkin_workspace/planning_through_pointclouds/data/

5m min height:
ex13_plan_through_pointcloud_ompl_octree_dubins --input ./pointcloud4-Churchill_Country_Club.ply --start 0.0 0.0 2.0 --goal 90.0 -25.0 1.0 \
--planner BIT_STAR --max_planning_time 10.0 --min_height 5.0 --turning_radius 1.0 --max_z_slope 0.15 --output ~/path_out_13_3_churchill.ply


David Butterworth
*/

#include <iostream> // cout
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp> // to_upper

#include <planning_through_pointclouds/utils.h> // processCommandLine, Timer, splitString
#include <planning_through_pointclouds/pcl_utils.h> // loadPointCloud, getPathPointCloudFromPoints, addTakeoffAndLanding
#include <planning_through_pointclouds/octree_map_3d_v3_ompl_dubins.h>

#include <pcl/io/ply_io.h> // savePLYFileASCII

//----------------------------------------------------------------------------//

int main(int argc, char **argv)
{
  std::string pointcloud_file;
  std::vector<float> start;
  std::vector<float> goal;
  std::string planner;
  double max_planning_time;
  double min_height;
  double turning_radius;
  double max_z_slope;
  std::string output_filename;

  const bool result = processCommandLine(argc, argv,
                                         pointcloud_file, start, goal,
                                         planner, max_planning_time,
                                         min_height, turning_radius, max_z_slope,
                                         output_filename);
  if (!result)
  {
    return EXIT_FAILURE;
  }
  std::cout << "pointcloud_file =  "<< pointcloud_file << std::endl;
  std::cout << "output_filename =  "<< output_filename << std::endl;
  const std::vector<std::string> output_filename_tokens = splitString(output_filename);

  if ((min_height > start.at(2)) || (min_height > goal.at(2)))
  {
    std::cout << "Using min height constraint: " << min_height << std::endl;
  }

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
  double waypoint_separation = 0.0; // 0 = output waypoints at the collision-checking resolution, which is 10cm
  timer.reset();
  const std::vector<Point> path = map.findPath(start_point, goal_point,
                                               planner_type, max_planning_time,
                                               interpolate_path, waypoint_separation,
                                               min_height,
                                               turning_radius,
                                               max_z_slope);
  const double time_taken = timer.read();
  std::cout << "  took " << time_taken << " ms." << std::endl;
  std::cout << "    Path length " << map.getPathLength() << std::endl;
  const uint num_checks = map.getNumCollisionChecks();
  std::cout << "    No. of collision checks: " << num_checks << " (" << time_taken/num_checks << " ms per check)" << std::endl;

  ColorPointCloud path_pointcloud;
  std::string outfile;

  const std::vector<Point> path2 = addTakeoffAndLanding<Point>(path, start_point, goal_point, min_height);

  // Save trajectory
  path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(path2);
  outfile = output_filename_tokens[0] + std::string("_orig.ply");
  std::cout << "Writing to " << outfile << std::endl;
  pcl::io::savePLYFileASCII(outfile, path_pointcloud);


  std::cout << "Simplifying path..." << std::endl;
  timer.reset();
  const double max_shortcutting_time = 60.0; //30.0;
  const bool interpolate_simplified_path = true;
  waypoint_separation = 0.10; // interpolate
  const std::vector<Point> simplified_path = map.getSimplifiedPath(max_shortcutting_time, interpolate_simplified_path, waypoint_separation);
  std::cout << "  took " << timer.read() << " ms." << std::endl;
  const std::vector<Point> simplified_path2 = addTakeoffAndLanding<Point>(simplified_path, start_point, goal_point, min_height);
  // Save trajectory
  path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(simplified_path2);
  outfile = output_filename_tokens[0] + std::string("_simplified.ply");
  std::cout << "Writing to " << outfile << std::endl;
  pcl::io::savePLYFileASCII(outfile, path_pointcloud);


  // Simplify path using different method:
  std::cout << "Reducing path..." << std::endl;
  timer.reset();
  const std::vector<Point> reduced_path = map.getReducedPath();
  std::cout << "  took " << timer.read() << " ms." << std::endl;
  std::cout << "Reduced path now has " << reduced_path.size() << " waypoints" << std::endl;
  const std::vector<Point> reduced_path2 = addTakeoffAndLanding<Point>(reduced_path, start_point, goal_point, min_height);
  // Save trajectory
  path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(reduced_path2);
  outfile = output_filename_tokens[0] + std::string("_reduced.ply");
  std::cout << "Writing to " << outfile << std::endl;
  pcl::io::savePLYFileASCII(outfile, path_pointcloud);

  return 0;
}
