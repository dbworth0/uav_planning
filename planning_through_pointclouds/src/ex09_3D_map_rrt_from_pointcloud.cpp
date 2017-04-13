//
// Example 9:
//
// Find path through a PointCloud using RRT algorithms
// from OMPL (Open Motion Planning Libray).
// The PointCloud is converted into a 3D occupancy (voxel) grid
// which is then expanded by the robot's radius.
// The planned path and expanded states are shown in ROS Rviz.
//
// Usage:
// roscore
// roslaunch planning_through_pointclouds ex09_3D_map_RRT_from_pointcloud.launch 
// (It takes a few seconds for the padded occupancy grid to be generated)
//
// David Butterworth
//

#include <iostream> // cout
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> // PointCloud2ConstPtr
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <planning_through_pointclouds/ompl_utils.h> // getPlannerGraphMarker
#include <planning_through_pointclouds/pcl_utils.h> // loadPointCloud
#include <planning_through_pointclouds/ros_utils.h> // makePointCloudMsg, makePathMsg, addPointToPath
#include <planning_through_pointclouds/voxel_map_3d_v4_ompl_occgrid.h>

//----------------------------------------------------------------------------//

// Forward declarations:


//----------------------------------------------------------------------------//

int main(int argc, char **argv)
{
  std::string pointcloud_file("");
  if (argc < 2)
  {
    std::cout << "ERROR, wrong number of arguments!" << std::endl;
    std::cout << "Usage: " << argv[0] <<" </path/to/pointcloud.pcd> " << std::endl;
    return EXIT_FAILURE;
  }
  else
  {
    pointcloud_file = std::string(argv[1]);
  }
  std::cout << "pointcloud_file =  "<< pointcloud_file << std::endl;

  ros::init(argc, argv, "test");
  ros::NodeHandle nh;

  PointCloud::Ptr input_pointcloud(new PointCloud());
  loadPointCloud(pointcloud_file, *input_pointcloud);
  std::cout << "Loaded PointCloud with " << input_pointcloud->points.size() << " points" << std::endl;

  std::cout << "Initializing VoxelMap3D..." << std::endl;

  // Pad voxel grid to half the diameter of DJI S1000
  const double padding_radius = 1.1 / 2.0;
  const double voxel_size = 0.10;
  VoxelMap3D map(input_pointcloud, voxel_size, padding_radius);

  //const PointCloud::Ptr downsampled_pointcloud = map.getDownsampledPointCloud();

  const Point start_point(67.0, 2.0, -2.0);
  const Point goal_point(65.0, 39.0, -1.0);

  std::cout << "Finding path to goal..." << std::endl;

  // RRT
  const std::vector<Point> path_rrt = map.findPath(start_point, goal_point, RRT, 120.0);
  std::cout << "    RRT path length " << map.getPathLength() << std::endl;
  const uint num_checks = map.getNumCollisionChecks();
  std::cout << "    No. of collision checks: " << num_checks << std::endl;
  const std::vector<Point> path_rrt_simplified = map.getSimplifiedPath();
  const ompl::base::PlannerDataPtr rrt_planner_data = map.getPlannerData();
  const visualization_msgs::MarkerArray rrt_graph_msg = getPlannerGraphMarker("rrt_expanded_states", rrt_planner_data, 0.02, 0.04);
  
  std::cout << "Converting Occupancy Grid to PointCloud..." << std::endl;
  const ColorPointCloud occupancy_grid_3d_cloud = map.getOccupancyGrid3D();

  const sensor_msgs::PointCloud2ConstPtr input_cloud_msg = makePointCloudMsg(*input_pointcloud);
  //const sensor_msgs::PointCloud2ConstPtr downsampled_cloud_msg = makePointCloudMsg(*downsampled_pointcloud);
  const sensor_msgs::PointCloud2ConstPtr occupancy_grid_cloud_msg = makePointCloudMsg(occupancy_grid_3d_cloud);
  const nav_msgs::PathConstPtr path_rrt_msg = makePathMsg(path_rrt);
  const nav_msgs::PathConstPtr path_rrt_simplified_msg = makePathMsg(path_rrt_simplified);

  ros::Publisher input_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("input_cloud", 1);
  //ros::Publisher downsampled_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("downsampled_cloud", 1);
  ros::Publisher occupancy_grid_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("occupancy_grid_cloud", 1);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  const double marker_size = 0.4;
  visualization_msgs::Marker start_marker = getPointMarker("start", start_point, marker_size);
  visualization_msgs::Marker goal_marker = getPointMarker("goal", goal_point, marker_size);
  ros::Publisher path_rrt_pub = nh.advertise<nav_msgs::Path>("path_rrt", 1);
  ros::Publisher path_rrt_simplified_pub = nh.advertise<nav_msgs::Path>("path_rrt_simplified", 1);
  ros::Publisher expanded_states_rrt_pub = nh.advertise<visualization_msgs::MarkerArray>("rrt_expanded_states", 1);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    input_cloud_pub.publish(input_cloud_msg);
    //downsampled_cloud_pub.publish(downsampled_cloud_msg);
    occupancy_grid_cloud_pub.publish(occupancy_grid_cloud_msg);
    marker_pub.publish(start_marker);
    marker_pub.publish(goal_marker);
    path_rrt_pub.publish(path_rrt_msg);
    path_rrt_simplified_pub.publish(path_rrt_simplified_msg);
    expanded_states_rrt_pub.publish(rrt_graph_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
