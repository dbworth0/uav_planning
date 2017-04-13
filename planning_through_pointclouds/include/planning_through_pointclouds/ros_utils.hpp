//
// Helper functions for ROS
//
// David Butterworth
//

#ifndef ROS_UTILS_IMPL_HPP
#define ROS_UTILS_IMPL_HPP

#include <pcl_conversions/pcl_conversions.h>  // toROSMsg()

template <typename PointCloudT>
sensor_msgs::PointCloud2Ptr makePointCloudMsg(const PointCloudT& pointcloud,
                                              const std::string& frame_id = "map")
{
  sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(pointcloud, *cloud_msg);
  cloud_msg->header.stamp = ros::Time::now();
  cloud_msg->header.frame_id = frame_id;

  return cloud_msg;
}

#endif // ROS_UTILS_IMPL_HPP
