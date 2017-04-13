/*
Example 15:

Same as example 14, but visualized using Rviz.


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
roslaunch planning_through_pointclouds ex15_3D_map_with_planning_constraints.launch


--takeoff_funnel +1.0 1.0 3.0 -2.0 +1.3 --landing_funnel +1.0 1.0 3.0 -2.0 +1.3




David Butterworth
*/

#include <iostream> // cout
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp> // to_upper

#include <planning_through_pointclouds/utils.h> // processCommandLine, Timer, range2()
#include <planning_through_pointclouds/pcl_utils.h> // loadPointCloud, getPathPointCloudFromPoints, addTakeoffAndLanding, addVerticalFunnelToPointCloud
#include <planning_through_pointclouds/octree_map_3d_v4_ompl.h>
#include <planning_through_pointclouds/ros_utils.h> // makePointCloudMsg

#include <pcl/io/ply_io.h> // savePLYFileASCII

#include <spline_library/vector.h>
#include <spline_library/hermite/cubic/cubic_hermite_spline.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> // PointCloud2ConstPtr
#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>

//----------------------------------------------------------------------------//

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;

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
  //std::cout << "  width:" << input_pointcloud->width << std::endl;
  //std::cout << "  height:" << input_pointcloud->height << std::endl;
  //std::cout << "  is_dense:" << input_pointcloud->is_dense << std::endl;

  // Keep copy of input PointCloud
  PointCloud::Ptr pointcloud(new PointCloud(*input_pointcloud));

  const Point start_point(start.at(0), start.at(1), start.at(2));
  const Point goal_point(goal.at(0), goal.at(1), goal.at(2));





  //const float point_separation = 0.02;
  const float point_separation = 0.10;

  addVerticalFunnelToPointCloud<Point>(start_point,
                                       takeoff_funnel.at(0), // offset
                                       takeoff_funnel.at(1), // middle_radius
                                       takeoff_funnel.at(2), // top_radius
                                       takeoff_funnel.at(3), // cylinder_height
                                       takeoff_funnel.at(4), // cone_height
                                       point_separation,
                                       pointcloud);
  addVerticalFunnelToPointCloud<Point>(goal_point, 
                                       landing_funnel.at(0), // offset
                                       landing_funnel.at(1), // middle_radius
                                       landing_funnel.at(2), // top_radius
                                       landing_funnel.at(3), // cylinder_height
                                       landing_funnel.at(4), // cone_height
                                       point_separation,
                                       pointcloud);

  // For debug purposes, make a PointCloud containing just the 2 constraint funnels
  PointCloud::Ptr funnels_pointcloud(new PointCloud());
  addVerticalFunnelToPointCloud<Point>(start_point,
                                       takeoff_funnel.at(0), takeoff_funnel.at(1), takeoff_funnel.at(2), takeoff_funnel.at(3), takeoff_funnel.at(4),
                                       point_separation, funnels_pointcloud);
  addVerticalFunnelToPointCloud<Point>(goal_point, 
                                       landing_funnel.at(0), landing_funnel.at(1), landing_funnel.at(2), landing_funnel.at(3), landing_funnel.at(4),
                                       point_separation, funnels_pointcloud);

  Timer timer;

  std::cout << "Initializing OctreeMap3D..." << std::endl;
  timer.reset();

  // Half the diameter of DJI S1000.
  const double robot_radius = 1.1 / 2.0;

  const double voxel_size = 0.10;
  OctreeMap3D map(pointcloud, voxel_size, robot_radius);
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


  // Reduce the number of vertices in the path:
  std::cout << "Simplifying path..." << std::endl;
  timer.reset();
  const std::vector<Point> simplified_path = map.getReducedPath();
  std::cout << "  took " << timer.read() << " ms." << std::endl;
  std::cout << "Simplified path now has " << simplified_path.size() << " waypoints" << std::endl;


  // Fit splines (Centripetal Catmull-Rom) between waypoints
  // Use this one, curve is tighter than Uniform or Chordal types.
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
  points.insert(points.begin(), first_point);
  points.push_back(last_point);
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

  const std::vector<Point> splined_path2 = addTakeoffAndLanding<Point>(splined_path, start_point, goal_point, min_height);


  const sensor_msgs::PointCloud2ConstPtr input_cloud_msg = makePointCloudMsg(*input_pointcloud);
  const sensor_msgs::PointCloud2ConstPtr funnels_cloud_msg = makePointCloudMsg(*funnels_pointcloud);
  const sensor_msgs::PointCloud2ConstPtr cloud_msg = makePointCloudMsg(*pointcloud);
  const double marker_size = 0.4;
  visualization_msgs::Marker start_marker = getPointMarker("start", start_point, marker_size);
  visualization_msgs::Marker goal_marker = getPointMarker("goal", goal_point, marker_size);
  const nav_msgs::PathConstPtr path_msg = makePathMsg(path2);
  const nav_msgs::PathConstPtr path_splined_msg = makePathMsg(splined_path2);

  ros::Publisher input_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("input_cloud", 1);
  ros::Publisher funnels_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("funnels_cloud", 1);
  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1);
  ros::Publisher path_splined_pub = nh.advertise<nav_msgs::Path>("path_splined", 1);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    input_cloud_pub.publish(input_cloud_msg);
    funnels_cloud_pub.publish(funnels_cloud_msg);
    cloud_pub.publish(cloud_msg);
    marker_pub.publish(start_marker);
    marker_pub.publish(goal_marker);
    path_pub.publish(path_msg);
    path_splined_pub.publish(path_splined_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
