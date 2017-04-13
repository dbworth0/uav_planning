//
// Example 1:
// Find path through a 2D occupancy grid using Dijkstra or A*.
//
// Planning is done over an explicit graph.
// Uses BGL (Boost Graph Libray).
// Visualized using ROS Rviz.
//
// Usage:
// roscore
// roslaunch planning_through_pointclouds ex01_2D_map_dijkstra_astar.launch
//
// David Butterworth
//

#include <iostream> // cout
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> // PointCloud2ConstPtr
#include <nav_msgs/Path.h>

#include <planning_through_pointclouds/ros_utils.h> // makePointCloudMsg, addPointToPath
#include <planning_through_pointclouds/grid_map_2d.h>

//----------------------------------------------------------------------------//

// Forward declarations:
nav_msgs::PathConstPtr make2DPathMsg(const PathCoordinates& path,
                                     const float point_separation,
                                     const std::string& frame_id = "map");

//----------------------------------------------------------------------------//

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");

  ros::NodeHandle nh;

  std::cout << "Initializing GridMap2D..." << std::endl;

  //const int width = 7;
  //const int length = 5;
  const int width = 60;
  const int length = 40;

  //const int connectivity = 4;
  const int connectivity = 8;
  GridMap2D map(width, length, connectivity);

  //map.addRectangleObstacle({2,2}, {5,2});
  //const PathCoordinates path_astar = map.getShortestPathDijkstra({4,0}, {3,4});
  //const PathCoordinates path_dijkstra = map.getShortestPathDijkstra({4,0}, {3,4});

  map.addRectangleObstacle({30,20}, {50,25});
  map.addLineObstacle({8,20}, {20,8});
  map.addLineObstacle({7,20}, {20,7});
  map.addLineObstacle({4,17}, {7,20});
  map.addLineObstacle({4,16}, {8,20});
  map.addLineObstacle({17,4}, {20,7});
  map.addLineObstacle({16,4}, {20,8});
  
  const PathCoordinates path_dijkstra = map.getShortestPathDijkstra({0,0}, {59,39});
  const PathCoordinates path_astar = map.getShortestPathAstar({0,0}, {59,39});
  const PathCoordinates smoothed_path = map.smooth2DPath(path_astar);
  //printPath(smoothed_path);

  // Will freeze here if graph too big
  //map.viewGraph();

  const float point_separation = 0.01;
  //const float point_separation = 0.005;
  const ColorPointCloud occupancy_grid_2d_cloud = map.getOccupancyGrid2D(point_separation);

  const ColorPointCloud astar_expanded_states_cloud = map.getAstarExpandedStates(point_separation);

  sensor_msgs::PointCloud2ConstPtr occupancy_grid_cloud_msg = makePointCloudMsg(occupancy_grid_2d_cloud);
  const nav_msgs::PathConstPtr path_dijkstra_msg = make2DPathMsg(path_dijkstra, point_separation);
  const nav_msgs::PathConstPtr path_astar_msg = make2DPathMsg(path_astar, point_separation);
  const sensor_msgs::PointCloud2ConstPtr astar_expansions_cloud_msg = makePointCloudMsg(astar_expanded_states_cloud);
  const nav_msgs::PathConstPtr smoothed_path_msg = make2DPathMsg(smoothed_path, point_separation);

  ros::Publisher occupancy_grid_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("occupancy_grid_cloud", 1);
  ros::Publisher path_dijkstra_pub = nh.advertise<nav_msgs::Path>("path_dijkstra", 1);
  ros::Publisher path_astar_pub = nh.advertise<nav_msgs::Path>("path_astar", 1);
  ros::Publisher astar_expansions_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("astar_expanded_states_cloud", 1);
  ros::Publisher smoothed_path_pub = nh.advertise<nav_msgs::Path>("smoothed_path", 1);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    occupancy_grid_cloud_pub.publish(occupancy_grid_cloud_msg);
    path_dijkstra_pub.publish(path_dijkstra_msg);
    path_astar_pub.publish(path_astar_msg);
    astar_expansions_cloud_pub.publish(astar_expansions_cloud_msg);
    smoothed_path_pub.publish(smoothed_path_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

//----------------------------------------------------------------------------//

// Convert the path co-ordinates (std::vector of Cells) to a ROS Path Msg
nav_msgs::PathConstPtr make2DPathMsg(const PathCoordinates& path,
                                     const float point_separation,
                                     const std::string& frame_id)
{
  nav_msgs::PathPtr path_msg(new nav_msgs::Path());
  path_msg->header.stamp = ros::Time::now();
  path_msg->header.frame_id = frame_id;

  // Position the path on top of the cell
  const double z_height = point_separation/2.0 + 0.001;

  for (size_t i = 0; i < path.size(); ++i)
  {
    const Point position(static_cast<float>(path.at(i).x) * point_separation,
                         static_cast<float>(path.at(i).y) * point_separation,
                         z_height);
    addPointToPath(path_msg, position);
  }

  return path_msg;
}
