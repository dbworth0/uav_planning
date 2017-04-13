//
// Helper functions for ROS
//
// David Butterworth
//

#include <planning_through_pointclouds/ros_utils.h>

nav_msgs::PathConstPtr vectorOfPointToPathMsg(const std::vector<Point>& waypoints,
                                              const std::string& frame_id)
{
  nav_msgs::PathPtr path_msg(new nav_msgs::Path());
  path_msg->header.stamp = ros::Time::now();
  path_msg->header.frame_id = frame_id;

  for (size_t i = 0; i < waypoints.size(); ++i)
  {
    addPointToPath(path_msg, waypoints.at(i));
  }

  return path_msg;
}

const std::vector<Point> pathMsgToVectorOfPoint(nav_msgs::PathPtr& path_msg)
{
  std::cout << "path msg contains " << path_msg->poses.size() << " points" << std::endl;

  std::vector<Point> points;
  for (size_t i = 0; i < path_msg->poses.size(); ++i)
  {
    points.push_back(Point(static_cast<float>(path_msg->poses.at(i).pose.position.x),
                           static_cast<float>(path_msg->poses.at(i).pose.position.y),
                           static_cast<float>(path_msg->poses.at(i).pose.position.z)));
  }

  return points;
}

const std::vector<Point> pathMsgToVectorOfPoint(nav_msgs::PathConstPtr& path_msg)
{
  return pathMsgToVectorOfPoint(path_msg);
}

nav_msgs::PathConstPtr makePathMsg(const std::vector<Point>& path,
                                   const std::string& frame_id)
{
  return vectorOfPointToPathMsg(path, frame_id);
}

void addPointToPath(nav_msgs::PathPtr path, const Point& position)
{
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = static_cast<double>(position.x);
  pose.pose.position.y = static_cast<double>(position.y);
  pose.pose.position.z = static_cast<double>(position.z);
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;

  path->poses.push_back(pose);
}

const visualization_msgs::Marker getPointMarker(const std::string& name,
                                                const pcl::PointXYZ& point,
                                                const double size,
                                                const std::string& frame_id)
{
  static int counter = 0;

  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;

  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();

  marker.ns = name;
  marker.id = counter;
  counter++;

  //marker.type = visualization_msgs::Marker::CUBE;
  marker.type = visualization_msgs::Marker::SPHERE;

  marker.pose.position.x = point.x;
  marker.pose.position.y = point.y;
  marker.pose.position.z = point.z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;

  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0; // solid color

  marker.lifetime = ros::Duration();

  return marker;
}
