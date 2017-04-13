//
// Example 2:
// Find path through a 3D occupancy (voxel) grid using Dijkstra or A*.
//
// Planning is done over an explicit graph.
// Uses BGL (Boost Graph Libray).
// Visualized using ROS Rviz.
//
// Usage:
// roscore
// roslaunch planning_through_pointclouds ex02_3D_map_dijkstra_astar.launch
//
// David Butterworth
//

#include <iostream> // cout
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> // PointCloud2ConstPtr
#include <nav_msgs/Path.h>

#include <planning_through_pointclouds/pcl_utils.h> // loadPointCloud
#include <planning_through_pointclouds/ros_utils.h> // makePointCloudMsg, addPointToPath
#include <planning_through_pointclouds/voxel_map_3d_v1.h>

//----------------------------------------------------------------------------//

// Forward declarations:
nav_msgs::PathConstPtr make3DPathMsg(const PathCoordinates& path,
                                     const float point_separation,
                                     const std::string& frame_id = "map");

//----------------------------------------------------------------------------//

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");

  ros::NodeHandle nh;

  std::cout << "Initializing VoxelMap3D..." << std::endl;

  //const uint width = 7;
  //const uint length = 5;
  //const uint depth = 3;
  const uint width = 50;
  const uint length = 50;
  const uint depth = 50;

  const double voxel_size = 0.01;
  //const uint connectivity = 6;
  const uint connectivity = 26;
  VoxelMap3D map(width, length, depth, voxel_size, connectivity);

  //map.addCuboidObstacle({2,2,1}, {5,2,3});
  //const PathCoordinates path_dijkstra = map.getShortestPathDijkstra({0,0,0}, {6,4,2});

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

  const PathCoordinates path_dijkstra = map.getShortestPathDijkstra({0,0,0}, {49,49,49});
  //printPath(path_dijkstra);

  const PathCoordinates path_astar = map.getShortestPathAstar({0,0,0}, {49,49,49});
  //printPath(path_astar);

  // Will freeze here if graph too big
  //map.viewGraph();

  const float point_separation = 0.01; // ToDo: remove this param
  const ColorPointCloud occupancy_grid_3d_cloud = map.getOccupancyGrid3D(point_separation);
  const ColorPointCloud astar_expanded_states_cloud = map.getAstarExpandedStates(point_separation);

  const sensor_msgs::PointCloud2ConstPtr occupancy_grid_cloud_msg = makePointCloudMsg(occupancy_grid_3d_cloud);
  const nav_msgs::PathConstPtr path_dijkstra_msg = make3DPathMsg(path_dijkstra, point_separation);
  const nav_msgs::PathConstPtr path_astar_msg = make3DPathMsg(path_astar, point_separation);
  const sensor_msgs::PointCloud2ConstPtr astar_expansions_cloud_msg = makePointCloudMsg(astar_expanded_states_cloud);

  ros::Publisher occupancy_grid_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("occupancy_grid_cloud", 1);
  ros::Publisher path_dijkstra_pub = nh.advertise<nav_msgs::Path>("path_dijkstra", 1);
  ros::Publisher path_astar_pub = nh.advertise<nav_msgs::Path>("path_astar", 1);
  ros::Publisher astar_expansions_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("astar_expanded_states_cloud", 1);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    occupancy_grid_cloud_pub.publish(occupancy_grid_cloud_msg);
    path_dijkstra_pub.publish(path_dijkstra_msg);
    path_astar_pub.publish(path_astar_msg);
    astar_expansions_cloud_pub.publish(astar_expansions_cloud_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

//----------------------------------------------------------------------------//

// Convert the path co-ordinates to a ROS Path Msg
nav_msgs::PathConstPtr make3DPathMsg(const PathCoordinates& path,
                                     const float point_separation,
                                     const std::string& frame_id)
{
  nav_msgs::PathPtr path_msg(new nav_msgs::Path());
  path_msg->header.stamp = ros::Time::now();
  path_msg->header.frame_id = frame_id;

  for (size_t i = 0; i < path.size(); ++i)
  {
    const Point position(path.at(i).x*point_separation,
                            path.at(i).y*point_separation,
                            path.at(i).z*point_separation);
    addPointToPath(path_msg, position);
  }

  return path_msg;
}
