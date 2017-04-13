/*
Example 30
Visualize a UAV model in Rviz and animate it's motion


Usage:
roslaunch planning_through_pointclouds ex30_animate_uav_in_rviz.launch


David Butterworth
*/

#include <iostream> // cout
//#include <string>
//#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> // PointCloud2ConstPtr

//#include <planning_through_pointclouds/utils.h> // processCommandLine, Timer
//#include <planning_through_pointclouds/ompl_utils.h> // getPlannerGraphMarker
#include <planning_through_pointclouds/pcl_utils.h> // loadPointCloud, getPointCloudBounds
#include <planning_through_pointclouds/ros_utils.h> // makePointCloudMsg
//#include <planning_through_pointclouds/opencv_utils.h> // printCvMatType


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointXYZRGBA ColorPoint;
typedef pcl::PointCloud<ColorPoint> ColorPointCloud;

//#include <nav_msgs/OccupancyGrid.h>

//#include <dynamic_voronoi/dynamicvoronoi.h>



#include <tf/transform_broadcaster.h>



// move to ROS utils
void publishTfFrame(const std::string& frame_id,
                    const double x, const double y, const double z,
                    const double roll, const double pitch, const double yaw,
                    const std::string& base_frame_id)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(x, y, z));
  tf::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), base_frame_id, frame_id));
}


#include <visualization_msgs/Marker.h>

// Create an Rviz Marker for a mesh file
const visualization_msgs::Marker getMeshMarker(const std::string& name,
                                               const std::string& mesh_file,
                                               const pcl::PointXYZ& position,
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

  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_resource = mesh_file; // pr2_description/meshes/base_v0/base.dae
  marker.mesh_use_embedded_materials = true;

  marker.pose.position.x = position.x;
  marker.pose.position.y = position.y;
  marker.pose.position.z = position.z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  const double scale = 1.0;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;

  marker.color.r = 87.0 / 255.0;
  marker.color.g = 65.0 / 255.0;
  marker.color.b = 47.0 / 255.0;
  marker.color.a = 1.0; // solid color

  marker.lifetime = ros::Duration();

  return marker;
}

// move to pcl utils
template <typename PointT>
void printPointCloudRangeBounds(typename pcl::PointCloud<PointT>::Ptr& cloud)
{
  const std::vector<float> bounds = getPointCloudBounds<Point>(cloud);
  const double x_range = bounds.at(1) - bounds.at(0);
  const double y_range = bounds.at(3) - bounds.at(2);
  const double z_range = bounds.at(5) - bounds.at(4);
  std::cout << "  range: " << bounds.at(0) << " : " << bounds.at(1) << std::endl;
  std::cout << "         " << bounds.at(2) << " : " << bounds.at(3) << std::endl;
  std::cout << "         " << bounds.at(4) << " : " << bounds.at(5) << std::endl;
  std::cout << "  dimensions: " << x_range << ", " << y_range << ", " << z_range << " meters" << std::endl;
}



//----------------------------------------------------------------------------//

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;

  std::string path_to_data("");
  if (argc < 2)
  {
    std::cout << "ERROR, wrong number of arguments!" << std::endl;
    std::cout << "Usage: " << argv[0] <<" </path/to/data/> " << std::endl;
    return EXIT_FAILURE;
  }
  else
  {
    path_to_data = std::string(argv[1]);
  }
  std::cout << "path_to_data =  "<< path_to_data << std::endl;

  PointCloud::Ptr ground_points_cloud(new PointCloud());
  loadPointCloud(path_to_data+"demo1_ground_points.ply", *ground_points_cloud);
  std::cout << "Loaded ground PointCloud with " << ground_points_cloud->points.size() << " points" << std::endl;
  printPointCloudRangeBounds<Point>(ground_points_cloud);



  PointCloud::Ptr other_points_cloud(new PointCloud());
  loadPointCloud(path_to_data+"demo1_other_points.ply", *other_points_cloud);
  std::cout << "Loaded PointCloud with " << other_points_cloud->points.size() << " points" << std::endl;



// 
// default = map

/*
  range: 57.5262 : 87.479
         11.1363 : 44.3791
         -4.14929 : -2.40888

*/


  const sensor_msgs::PointCloud2ConstPtr ground_points_cloud_msg = makePointCloudMsg(*ground_points_cloud);
  const sensor_msgs::PointCloud2ConstPtr other_points_cloud_msg = makePointCloudMsg(*other_points_cloud);

  const visualization_msgs::Marker ground_mesh_marker_msg = getMeshMarker("ground_mesh",
                                                                      "package://planning_through_pointclouds/data/demo1_ground_mesh.ply",
                                                                      pcl::PointXYZ(0,0,0),
                                                                      "/map");


  ros::Publisher ground_points_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("ground_points", 1);
  ros::Publisher other_points_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("other_points", 1);


  ros::Publisher ground_mesh_pub = nh.advertise<visualization_msgs::Marker>("ground_mesh_marker", 1);


  ros::Rate loop_rate(1.0);
  while (ros::ok())
  {
    ground_points_cloud_pub.publish(ground_points_cloud_msg);
    other_points_cloud_pub.publish(other_points_cloud_msg);

    ground_mesh_pub.publish(ground_mesh_marker_msg);

    publishTfFrame("/mesh_frame", 57.5262, 11.1363, -4.14929, 0,0,0, "/map");
    publishTfFrame("/base_link", 60, 12, 0, 0,0,0, "/map");





    ros::spinOnce();
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}

