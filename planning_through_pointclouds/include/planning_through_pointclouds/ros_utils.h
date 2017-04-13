//
// Helper functions for ROS
//
// David Butterworth
//

#ifndef ROS_UTILS_H
#define ROS_UTILS_H

#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
typedef pcl::PointXYZ Point;

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <planning_through_pointclouds/ros_utils.hpp>

// Convert a PCL PointCloud to a ROS PointCLoud2 Msg
template <typename PointCloudT>
sensor_msgs::PointCloud2Ptr makePointCloudMsg(const PointCloudT& pointcloud,
                                              const std::string& frame_id = "map");

// Make a ROS Path Msg from a vector of PCL Points.
// This wraps vectorOfPointToPathMsg()
nav_msgs::PathConstPtr makePathMsg(const std::vector<Point>& path,
                                   const std::string& frame_id = "map");

// Convert a vector of PCL Point to a ROS Path Msg
nav_msgs::PathConstPtr vectorOfPointToPathMsg(const std::vector<Point>& path,
                                              const std::string& frame_id);

// Convert the position of each pose in ROS Path Msg to a vector of PCL Point
const std::vector<Point> pathMsgToVectorOfPoint(nav_msgs::PathPtr& path_msg);
const std::vector<Point> pathMsgToVectorOfPoint(nav_msgs::PathConstPtr& path_msg);


// Add a point (x,y,z) to a ROS Path Msg
void addPointToPath(nav_msgs::PathPtr path, const Point& position);

// Make an Rviz Marker to represent a point.
// (a small red sphere)
const visualization_msgs::Marker getPointMarker(const std::string& name,
                                                const pcl::PointXYZ& point,
                                                const double size,
                                                const std::string& frame_id = "map");

#endif // ROS_UTILS_H
