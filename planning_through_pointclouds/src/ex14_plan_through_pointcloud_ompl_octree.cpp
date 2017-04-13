/*
Example 14:

This adds takeof and landing linear section, with cylinder and funnel constraint.





Find path through a PointCloud using RRT algorithms
from OMPL (Open Motion Planning Libray).
The PointCloud is converted into a PCL Octree, and collision-checking is
performed by checking for points within a sphere (the robot's radius).

The path is smoothed using a centripetal Catmull-Rom Splines
The paths are written as .ply files.

Has optional min height (elevation) constraint.
If set, the path will have a vertical linear "take off" segment until
it reaches the min height, planning occurs within the height constraint,
then the path finishes with a vertical linear "landing" segment.


Usage:
export PATH=$PATH:~/path/to/your/catkin_workspace/devel/lib/planning_through_pointclouds/
cd ~/path/to/your/catkin_workspace/planning_through_pointclouds/data/




Use a minimum height constraint. This means the take-off and landing have a linear segment:

ex14_plan_through_pointcloud_ompl_octree --input ./pointcloud4-Churchill_Country_Club.ply --start 0.0 0.0 2.0 --goal 90.0 -25.0 1.0 \
--min_height 5.0 --max_height 100 --takeoff_funnel +0.0 1.0 10.0 -5.0 +5.0 --landing_funnel +0.0 1.0 10.0 -5.0 +5.0 \
--planner BIT_STAR --max_planning_time 10.0 --output ~/path_out_14_1_churchill.ply


No min height constraint. Use take-off and landing constraint funnels:

ex14_plan_through_pointcloud_ompl_octree --input ./pointcloud4-Churchill_Country_Club.ply --start 0.0 0.0 2.0 --goal 90.0 -25.0 1.0 \
--min_height -100 --max_height 100 --takeoff_funnel +4.0 1.0 1.5 -20.0 +3.0 --landing_funnel +4.0 1.0 5.0 -20.0 +3.0 \
--planner BIT_STAR --max_planning_time 40.0 --output ~/path_out_14_1_churchill.ply


But it can be hard to find a solution through the cylinder, so use min height constraint instead
so a linear segment will be added at the ends:

ex14_plan_through_pointcloud_ompl_octree --input ./pointcloud4-Churchill_Country_Club.ply --start 0.0 0.0 2.0 --goal 90.0 -25.0 1.0 \
--min_height +6.0 --max_height 100 --takeoff_funnel +4.0 0.8 1.5 -20.0 +3.0 --landing_funnel +4.0 0.8 5.0 -20.0 +3.0 \
--planner BIT_STAR --max_planning_time 60.0 --output ~/path_out_14_1_churchill.ply




New map:

z = 6 meters, top funnel radius = 1.5
z = 6 meters, top funnel radius = 5.0

ex14_plan_through_pointcloud_ompl_octree --input ./pointcloud8-Churchill_Country_Club-Dec2016.ply --start 0.0 0.0 3.0 --goal 102.0 -4.0 2.0 \
--min_height +6.0 --max_height 100 --takeoff_funnel +4.0 0.8 1.5 -20.0 +3.0 --landing_funnel +4.0 0.8 5.0 -20.0 +3.0 \
--planner BIT_STAR --max_planning_time 60.0 --output ~/path_out_14_1_churchill.ply


z = 8 meters, top funnel radius = 1.5
z = 8 meters, top funnel radius = 5.0

ex14_plan_through_pointcloud_ompl_octree --input ./pointcloud8-Churchill_Country_Club-Dec2016.ply --start 0.0 0.0 3.0 --goal 102.0 -4.0 2.0 \
--min_height +8.0 --max_height 100 --takeoff_funnel +4.0 0.8 1.5 -20.0 +3.0 --landing_funnel +4.0 0.8 5.0 -20.0 +3.0 \
--planner BIT_STAR --max_planning_time 60.0 --output ~/path_out_14_1_churchill.ply


z = 10 meters, top funnel radius = 1.5
z = 10 meters, top funnel radius = 5.0

ex14_plan_through_pointcloud_ompl_octree --input ./pointcloud8-Churchill_Country_Club-Dec2016.ply --start 0.0 0.0 3.0 --goal 102.0 -4.0 2.0 \
--min_height +10.0 --max_height 100 --takeoff_funnel +4.0 0.8 1.5 -20.0 +3.0 --landing_funnel +4.0 0.8 5.0 -20.0 +3.0 \
--planner BIT_STAR --max_planning_time 60.0 --output ~/path_out_14_1_churchill.ply



 (0, 0, 3) and ending at (102 -4 2).


ex14_plan_through_pointcloud_ompl_octree --input ./pointcloud4-Churchill_Country_Club.ply --start 0.0 0.0 2.0 --goal 90.0 -25.0 1.0 \
--min_height 5.0 --max_height 100 --takeoff_funnel +0.0 1.0 10.0 -5.0 +5.0 --landing_funnel +0.0 1.0 10.0 -5.0 +5.0 \
--planner BIT_STAR --max_planning_time 10.0 --output ~/path_out_14_1_churchill.ply


z = -1.68 + 1.5 = -0.18
z = -5.4 + 1.5 = -3.9



Schenley Park

ex14_plan_through_pointcloud_ompl_octree --input ./schenley_park.ply --start 0.0 0.0 -0.18 --goal 90.0 10.0 -3.9 \
--min_height -100.0 --max_height 100 --takeoff_funnel 0.0 1.0 1.0 1.0 1.0 --landing_funnel 0.0 1.0 1.0 1.0 1.0 \
--planner BIT_STAR --max_planning_time 20.0 --output ~/path_out_schenley_1.ply




David Butterworth
*/

#include <iostream> // cout
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp> // to_upper

#include <planning_through_pointclouds/utils.h> // processCommandLine, Timer
#include <planning_through_pointclouds/pcl_utils.h> // loadPointCloud, getPathPointCloudFromPoints, addTakeoffAndLanding, addVerticalFunnelToPointCloud
#include <planning_through_pointclouds/spline_utils.h> // getCentripetalCatmullRomSpline, getPointsFromSpline
#include <planning_through_pointclouds/octree_map_3d_v4_ompl.h>


#include <spline_library/vector.h>
#include <spline_library/hermite/cubic/cubic_hermite_spline.h>

//#include <pcl/io/ply_io.h> // savePLYFileASCII

/*
template<typename PointT>
inline int savePLYFile(const std::string& file_name, const pcl::PointCloud<PointT>& cloud)
{
  pcl::PLYWriter w;
  const bool binary = false;
  const bool use_camera = false; // don't write extra scalar fields
  return (w.write<PointT>(file_name, cloud, binary, use_camera));
}
*/

//----------------------------------------------------------------------------//

int main(int argc, char **argv)
{
  std::string pointcloud_file;
  std::vector<float> start;
  std::vector<float> goal;
  double min_height;
  double max_height;
  std::vector<float> takeoff_funnel;
  std::vector<float> landing_funnel;
  std::string planner;
  double max_planning_time;
  std::string output_filename;

  const bool result = processCommandLine5(argc, argv,
                                          pointcloud_file,
                                          start, goal,
                                          min_height, max_height,
                                          takeoff_funnel,
                                          landing_funnel,
                                          planner, max_planning_time, output_filename);
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

  const Point start_point(start.at(0), start.at(1), start.at(2));
  const Point goal_point(goal.at(0), goal.at(1), goal.at(2));

  const float point_separation = 0.02;
  addVerticalFunnelToPointCloud<Point>(start_point,
                                       takeoff_funnel.at(0), // offset
                                       takeoff_funnel.at(1), // middle_radius
                                       takeoff_funnel.at(2), // top_radius
                                       takeoff_funnel.at(3), // cylinder_height
                                       takeoff_funnel.at(4), // cone_height
                                       point_separation,
                                       input_pointcloud);
  addVerticalFunnelToPointCloud<Point>(goal_point, 
                                       landing_funnel.at(0), // offset
                                       landing_funnel.at(1), // middle_radius
                                       landing_funnel.at(2), // top_radius
                                       landing_funnel.at(3), // cylinder_height
                                       landing_funnel.at(4), // cone_height
                                       point_separation,
                                       input_pointcloud);

  Timer timer;

  std::cout << "Initializing OctreeMap3D..." << std::endl;
  timer.reset();

  // Half the diameter of DJI S1000.
  const double robot_radius = 1.1 / 2.0;

  const double voxel_size = 0.10;
  OctreeMap3D map(input_pointcloud, voxel_size, robot_radius);
  std::cout << "  took " << timer.read() << " ms." << std::endl;

  std::cout << "Finding path to goal..." << std::endl;

  boost::to_upper(planner);
  const PLANNER_TYPE planner_type = stringToEnum(planner);
  const bool interpolate_path = true;
  const double waypoint_separation = 0.0; // 0 = output waypoints at the collision-checking resolution, which is 10cm
  timer.reset();
  const std::vector<Point> path = map.findPath(start_point, goal_point, min_height, planner_type, max_planning_time, interpolate_path, waypoint_separation);
  const double time_taken = timer.read();
  std::cout << "  took " << time_taken << " ms." << std::endl;
  std::cout << "    Path length: " << map.getPathLength() << std::endl;
  std::cout << "    No. waypoints: " << path.size() << std::endl;

  const uint num_checks = map.getNumCollisionChecks();
  std::cout << "    No. of collision checks: " << num_checks << " (" << time_taken/num_checks << " ms per check)" << std::endl;

  ColorPointCloud path_pointcloud;
  std::string outfile;

  const std::vector<Point> path2 = addTakeoffAndLanding<Point>(path, start_point, goal_point, min_height);

  // Save trajectory
  //path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(path2);
  path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(path);
  outfile = output_filename_tokens[0] + std::string("_orig.ply");
  std::cout << "Writing to " << outfile << std::endl;
  //pcl::io::savePLYFileASCII(outfile, path_pointcloud);
  savePLYFile(outfile, path_pointcloud);

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
  //pcl::io::savePLYFileASCII(outfile, path_pointcloud);
  savePLYFile(outfile, path_pointcloud);

  // Fit splines (Centripetal Catmull-Rom) between waypoints
  // Use this one, curve is tighter than Uniform or Chordal types.
  std::shared_ptr<spline_library::Spline<spline_library::Vector3> > spline = getCentripetalCatmullRomSpline<Point>(simplified_path);
  const float dist_step = 0.01; // interpolation step = 1cm
  std::vector<Point> splined_path = getPointsFromSpline<Point>(spline, dist_step);
  std::cout << "Splined path now has " << splined_path.size() << " waypoints" << std::endl;

  const std::vector<Point> splined_path2 = addTakeoffAndLanding<Point>(splined_path, start_point, goal_point, min_height, dist_step);

  // Save trajectory
  //path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(splined_path2);
  path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(splined_path);
  outfile = output_filename_tokens[0] + std::string("_splined_CentripetalCR.ply");
  std::cout << "Writing to " << outfile << std::endl;
  //pcl::io::savePLYFileASCII(outfile, path_pointcloud);
  savePLYFile(outfile, path_pointcloud);

  // Save PointCloud with constraint funnels
  //pcl::io::savePCDFileASCII("/home/dbworth/cloud.pcd", *input_pointcloud);



  /*
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
  
  */


  return 0;
}
