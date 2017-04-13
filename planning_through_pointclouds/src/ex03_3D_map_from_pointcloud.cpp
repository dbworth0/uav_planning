//
// Example 3:
// Find path through a PointCloud using Dijkstra or A*.
//
// The PointCloud is converted into 3D occupancy (voxel) grid.
// Planning is done over an explicit graph.
// Uses BGL (Boost Graph Libray).
// Visualized using ROS Rviz.
//
// Usage:
// roscore
// roslaunch planning_through_pointclouds ex03_3D_map_from_pointcloud.launch
// (It takes about 3mins for path to be calculated and displayed.)
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

#include <planning_through_pointclouds/pcl_utils.h> // loadPointCloud
#include <planning_through_pointclouds/ros_utils.h> // makePointCloudMsg, addPointToPath
#include <planning_through_pointclouds/voxel_map_3d_v1.h>

//----------------------------------------------------------------------------//

// Forward declarations:
nav_msgs::PathConstPtr make3DPathMsg(const std::vector<ColorPoint>& path,
                                     const std::string& frame_id = "map");

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

  PointCloud::Ptr input_pointcloud(new PointCloud());
  loadPointCloud(pointcloud_file, *input_pointcloud);
  std::cout << "Loaded PointCloud with " << input_pointcloud->points.size() << " points" << std::endl;

  //const bool publish_ros_msg = false;
  const bool publish_ros_msg = true;

  std::cout << "Initializing VoxelMap3D..." << std::endl;

  // Pad voxel grid to half the diameter of DJI S1000
  const double padding_radius = 1.1 / 2.0;
  const double voxel_size = 0.10;
  const uint connectivity = 6;
  //const uint connectivity = 26; // uses almost 16gb RAM
  VoxelMap3D map(input_pointcloud, voxel_size, padding_radius, connectivity);

  const PointCloud::Ptr downsampled_pointcloud = map.getDownsampledPointCloud();

  const Point start_point(67.0, 2.0, -2.0);
  const Point goal_point(65.0, 39.0, -1.0);

  std::cout << "Finding shortest path to goal..." << std::endl;
  const PathCoordinates path_dijkstra_coords = map.getShortestPathDijkstra(start_point, goal_point);
  //printPath(path_dijkstra_coords);

  const PathCoordinates path_astar_coords = map.getShortestPathAstar(start_point, goal_point);
  //printPath(path_astar);

  // Will freeze here if graph too big
  //map.viewGraph();

  if (publish_ros_msg)
  {
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;

    std::cout << "Publishing data as ROS msgs..." << std::endl;

    std::cout << "Converting Occupancy Grid to PointCloud..." << std::endl;
    const ColorPointCloud occupancy_grid_3d_cloud = map.getOccupancyGrid3D();

    const sensor_msgs::PointCloud2ConstPtr input_cloud_msg = makePointCloudMsg(*input_pointcloud);
    const sensor_msgs::PointCloud2ConstPtr downsampled_cloud_msg = makePointCloudMsg(*downsampled_pointcloud);
    const sensor_msgs::PointCloud2ConstPtr occupancy_grid_cloud_msg = makePointCloudMsg(occupancy_grid_3d_cloud);

    const std::vector<ColorPoint> path_dijkstra = map.getPathPoints(path_dijkstra_coords);
    const nav_msgs::PathConstPtr path_dijkstra_msg = make3DPathMsg(path_dijkstra);

    const std::vector<ColorPoint> path_astar = map.getPathPoints(path_astar_coords);
    const nav_msgs::PathConstPtr path_astar_msg = make3DPathMsg(path_astar);

    ros::Publisher input_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("input_cloud", 1);
    ros::Publisher downsampled_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("downsampled_cloud", 1);
    ros::Publisher occupancy_grid_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("occupancy_grid_cloud", 1);
    ros::Publisher path_dijkstra_pub = nh.advertise<nav_msgs::Path>("path_dijkstra", 1);
    ros::Publisher path_astar_pub = nh.advertise<nav_msgs::Path>("path_astar", 1);

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    const double marker_size = 0.4;
    visualization_msgs::Marker start_marker = getPointMarker("start", start_point, marker_size);
    visualization_msgs::Marker goal_marker = getPointMarker("goal", goal_point, marker_size);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
      occupancy_grid_cloud_pub.publish(occupancy_grid_cloud_msg);
      path_dijkstra_pub.publish(path_dijkstra_msg);
      path_astar_pub.publish(path_astar_msg);
      input_cloud_pub.publish(input_cloud_msg);
      downsampled_cloud_pub.publish(downsampled_cloud_msg);

      marker_pub.publish(start_marker);
      marker_pub.publish(goal_marker);

      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  return 0;
}

//----------------------------------------------------------------------------//

// Convert the path (being a set of x,y,z points) to a ROS Path Msg
nav_msgs::PathConstPtr make3DPathMsg(const std::vector<ColorPoint>& path,
                                    const std::string& frame_id)
{
  nav_msgs::PathPtr path_msg(new nav_msgs::Path());
  path_msg->header.stamp = ros::Time::now();
  path_msg->header.frame_id = frame_id;

  for (size_t i = 0; i < path.size(); ++i)
  {
    const Point position(path.at(i).x, path.at(i).y, path.at(i).z);
    addPointToPath(path_msg, position);
  }

  return path_msg;
}
