//
// Example 8:
//
// Find path through a 3D occupancy (voxel) grid using
// RRT algorithms from OMPL (Open Motion Planning Libray).
// The planned path and expanded states are shown in ROS Rviz.
//
// Usage:
// roscore
// roslaunch planning_through_pointclouds ex08_3D_map_RRT.launch
//
// David Butterworth
//

#include <iostream> // cout
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> // PointCloud2ConstPtr
#include <nav_msgs/Path.h>
//#include <visualization_msgs/Marker.h>
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
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;

  std::cout << "Initializing VoxelMap3D..." << std::endl;

  const uint width = 50;
  const uint length = 50;
  const uint depth = 50;

  const double voxel_size = 0.01;
  VoxelMap3D map(width, length, depth, voxel_size);

  map.addCuboidObstacle({7,7,3}, {13,13,10});
  map.addCuboidObstacle({36,7,3}, {47,13,10});
  map.addCuboidObstacle({7,36,3}, {13,47,10});
  map.addCuboidObstacle({36,36,3}, {47,47,10});
  map.addCuboidObstacle({7,7,33}, {13,13,40});
  map.addCuboidObstacle({36,7,33}, {47,13,40});
  map.addCuboidObstacle({7,36,33}, {13,47,40});
  map.addCuboidObstacle({36,36,33}, {47,47,40});
  map.addCuboidObstacle({17,22,15}, {25,35,23});
  map.addCuboidObstacle({25,22,28}, {43,33,33});

  // RRT
  const std::vector<Point> path_rrt = map.findPath({0,0,0}, {49,49,49}, RRT, 30.0);
  std::cout << "    RRT path length " << map.getPathLength() << std::endl;
  std::cout << "      No. of collision checks: " << map.getNumCollisionChecks() << std::endl;
  const std::vector<Point> path_rrt_simplified = map.getSimplifiedPath();
  const ompl::base::PlannerDataPtr planner_data_rrt = map.getPlannerData();
  const visualization_msgs::MarkerArray graph_msg_rrt = getPlannerGraphMarker("rrt_expanded_states", planner_data_rrt, 0.001, 0.002);
  
  // RRT-Connect
  const std::vector<Point> path_rrt_connect = map.findPath({0,0,0}, {49,49,49}, RRT_CONNECT, 30.0);
  std::cout << "    RRT-Connect path length " << map.getPathLength() << std::endl;
  std::cout << "      No. of collision checks: " << map.getNumCollisionChecks() << std::endl;
  const std::vector<Point> path_rrt_connect_simplified = map.getSimplifiedPath();
  const ompl::base::PlannerDataPtr planner_data_rrt_connect = map.getPlannerData();
  const visualization_msgs::MarkerArray graph_msg_rrt_connect = getPlannerGraphMarker("rrt_connect_expanded_states", planner_data_rrt_connect, 0.001, 0.002);

  // RRT* - takes the full 30 seconds before it finishes!
  const std::vector<Point> path_rrt_star = map.findPath({0,0,0}, {49,49,49}, RRT_STAR, 30.0);
  std::cout << "    RRT* path length " << map.getPathLength() << std::endl;
  std::cout << "      No. of collision checks: " << map.getNumCollisionChecks() << std::endl;
  const std::vector<Point> path_rrt_star_simplified = map.getSimplifiedPath();
  const ompl::base::PlannerDataPtr planner_data_rrt_star = map.getPlannerData();
  //const visualization_msgs::MarkerArray graph_msg_rrt_star = getPlannerGraphMarker("rrt_star_expanded_states", planner_data_rrt_star, 0.001, 0.002);

  // BIT*
  const std::vector<Point> path_bit_star = map.findPath({0,0,0}, {49,49,49}, BIT_STAR, 30.0);
  std::cout << "    BIT* path length " << map.getPathLength() << std::endl;
  std::cout << "      No. of collision checks: " << map.getNumCollisionChecks() << std::endl;
  const std::vector<Point> path_bit_star_simplified = map.getSimplifiedPath();
  const ompl::base::PlannerDataPtr planner_data_bit_star = map.getPlannerData();
  const visualization_msgs::MarkerArray graph_msg_bit_star = getPlannerGraphMarker("bit_star_expanded_states", planner_data_bit_star, 0.001, 0.002);


// MOVE THESE UP NEXT TO EACH PLANNER

  const ColorPointCloud occupancy_grid_3d_cloud = map.getOccupancyGrid3D();
  const sensor_msgs::PointCloud2ConstPtr occupancy_grid_cloud_msg = makePointCloudMsg(occupancy_grid_3d_cloud);

  const nav_msgs::PathConstPtr path_rrt_msg = makePathMsg(path_rrt);
  const nav_msgs::PathConstPtr path_rrt_simplified_msg = makePathMsg(path_rrt_simplified);

  const nav_msgs::PathConstPtr path_rrt_connect_msg = makePathMsg(path_rrt_connect);
  const nav_msgs::PathConstPtr path_rrt_connect_simplified_msg = makePathMsg(path_rrt_connect_simplified);

  const nav_msgs::PathConstPtr path_rrt_star_msg = makePathMsg(path_rrt_star);
  const nav_msgs::PathConstPtr path_rrt_star_simplified_msg = makePathMsg(path_rrt_star_simplified);

  const nav_msgs::PathConstPtr path_bit_star_msg = makePathMsg(path_bit_star);
  const nav_msgs::PathConstPtr path_bit_star_simplified_msg = makePathMsg(path_bit_star_simplified);



  ros::Publisher occupancy_grid_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("occupancy_grid_cloud", 1);

  ros::Publisher path_rrt_pub = nh.advertise<nav_msgs::Path>("rrt_path", 1);
  ros::Publisher path_rrt_simplified_pub = nh.advertise<nav_msgs::Path>("rrt_path_simplified", 1);
  ros::Publisher expanded_states_rrt_pub = nh.advertise<visualization_msgs::MarkerArray>("rrt_expanded_states", 1);

  ros::Publisher path_rrt_connect_pub = nh.advertise<nav_msgs::Path>("rrt_connect_path", 1);
  ros::Publisher path_rrt_connect_simplified_pub = nh.advertise<nav_msgs::Path>("rrt_connect_path_simplified", 1);
  ros::Publisher expanded_states_rrt_connect_pub = nh.advertise<visualization_msgs::MarkerArray>("rrt_connect_expanded_states", 1);

  ros::Publisher path_rrt_star_pub = nh.advertise<nav_msgs::Path>("rrt_star_path", 1);
  ros::Publisher path_rrt_star_simplified_pub = nh.advertise<nav_msgs::Path>("rrt_star_path_simplified", 1);
  //ros::Publisher expanded_states_rrt_star_pub = nh.advertise<visualization_msgs::MarkerArray>("rrt_star_expanded_states", 1);

  ros::Publisher path_bit_star_pub = nh.advertise<nav_msgs::Path>("bit_star_path", 1);
  ros::Publisher path_bit_star_simplified_pub = nh.advertise<nav_msgs::Path>("bit_star_path_simplified", 1);
  ros::Publisher expanded_states_bit_star_pub = nh.advertise<visualization_msgs::MarkerArray>("bit_star_expanded_states", 1);


  // TODO:
  // filter the number of BIT* and RRT* expanded states to less, so we can display them.


  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    occupancy_grid_cloud_pub.publish(occupancy_grid_cloud_msg);

    path_rrt_pub.publish(path_rrt_msg);
    path_rrt_simplified_pub.publish(path_rrt_simplified_msg);
    expanded_states_rrt_pub.publish(graph_msg_rrt);

    path_rrt_connect_pub.publish(path_rrt_connect_msg);
    path_rrt_connect_simplified_pub.publish(path_rrt_connect_simplified_msg);
    expanded_states_rrt_connect_pub.publish(graph_msg_rrt_connect);

    path_rrt_star_pub.publish(path_rrt_star_msg);
    path_rrt_star_simplified_pub.publish(path_rrt_star_simplified_msg);
    //expanded_states_rrt_star_pub.publish(graph_msg_rrt_star); // Crashes computer, too many line segments!

    path_bit_star_pub.publish(path_bit_star_msg);
    path_bit_star_simplified_pub.publish(path_bit_star_simplified_msg);
    //expanded_states_bit_star_pub.publish(graph_msg_bit_star); // too many


    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}

//----------------------------------------------------------------------------//
