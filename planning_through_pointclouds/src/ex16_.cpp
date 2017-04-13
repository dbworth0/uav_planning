/*
Example ? - tbd:

This makes a voronoi plan and displays in Rviz?




Find path through a PointCloud using RRT algorithms
from OMPL (Open Motion Planning Libray).
The PointCloud is converted into a PCL Octree, and collision-checking is
performed by checking for points within a sphere (the robot's radius).
The final path is written out as a .ply file.

Usage:
export PATH=$PATH:~/path/to/your/catkin_workspace/devel/lib/planning_through_pointclouds/
cd ~/path/to/your/catkin_workspace/planning_through_pointclouds/data/


cropped_cloud_1.ply

roslaunch planning_through_pointclouds ex14.launch

67.0 2.0 -2.0 --goal 65.0 39.0 -1.0

David Butterworth
*/

#include <iostream> // cout
//#include <string>
//#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> // PointCloud2ConstPtr

#include <Eigen/Geometry>

//#include <boost/algorithm/string.hpp> // to_upper

//#include <planning_through_pointclouds/utils.h> // processCommandLine, Timer
//#include <planning_through_pointclouds/ompl_utils.h> // getPlannerGraphMarker
#include <planning_through_pointclouds/pcl_utils.h> // loadPointCloud, getPointCloudBounds
#include <planning_through_pointclouds/ros_utils.h> // makePointCloudMsg
#include <planning_through_pointclouds/opencv_utils.h> // printCvMatType


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointXYZRGBA ColorPoint;
typedef pcl::PointCloud<ColorPoint> ColorPointCloud;


//#include <planning_through_pointclouds/octree_map_3d_v3_ompl_dubins.h>

//#include <pcl/io/ply_io.h> // savePLYFileASCII
#include <pcl/io/png_io.h> // savePNGFile

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp> // imwrite
#include <opencv2/imgproc/imgproc.hpp> // cvtColor


//#include <SplineLibrary/spline_library/hermite/cubic/cubic_hermite_spline.h>

////#include <voronoi_2d/voronoi.hpp> // WHY DOESNT THIS WORK ??
//#include <voronoi_2d/voronoi/src/voronoi.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h> // LETHAL_OBSTACLE
#include <voronoi_planner/planner_base.h>
#include <nav_msgs/OccupancyGrid.h>

//#include <dynamic_voronoi/dynamicvoronoi.h>



//#include <iostream>
//#include <fstream>
//#include <string.h>



// Set the z value of all points in a PointCloud to a specific value
template <typename PointT>
void flattenPointCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud, const float z_height)
{
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    cloud->points.at(i).z = z_height;
  }
}


template <typename PointT>
void addPointCloudToCostMap(const typename pcl::PointCloud<PointT>::Ptr& cloud,
                            costmap_2d::Costmap2D& costmap)
{
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    unsigned int mx;
    unsigned int my;
    costmap.worldToMap(static_cast<double>(cloud->points.at(i).x),
                       static_cast<double>(cloud->points.at(i).y),
                       mx, my);
    costmap.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE); // lethal = 254
  }
}


//----------------------------------------------------------------------------//

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;

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


  const std::vector<float> bounds = getPointCloudBounds<Point>(input_pointcloud);
  std::cout << "Cloud dimensions: " << std::endl
            << "  x_min: " << bounds.at(0) << " "
            << "  x_max: " << bounds.at(1) << std::endl
            << "  y_min: " << bounds.at(2) << " "
            << "  y_max: " << bounds.at(3) << std::endl
            << "  z_min: " << bounds.at(4) << " "
            << "  z_max: " << bounds.at(5) << std::endl;

  // Crop a z-slice of the PointCloud
  const float z_height = -1.0;
  const float slice_height = 0.50;
  const float z_min = z_height - slice_height;
  const float z_max = z_height + slice_height;
  PointCloud::Ptr cropped_cloud(new PointCloud());
  const Eigen::Vector3f min_extents(bounds.at(0), bounds.at(2), z_min);
  const Eigen::Vector3f max_extents(bounds.at(1), bounds.at(3), z_max);

  //cropPointCloud<Point>(input_pointcloud, cropped_cloud, min_extents, max_extents);

  //PointCloud::Ptr cloud_slice(new PointCloud(*cropped_cloud));
  //flattenPointCloud<Point>(cloud_slice, z_height);

  const float cell_size = 0.01;

  const float x_range = bounds.at(1) - bounds.at(0);
  const float y_range = bounds.at(3) - bounds.at(2);
  const unsigned int cells_size_x = static_cast<uint>(std::ceil(x_range / cell_size));
  const unsigned int cells_size_y = static_cast<uint>(std::ceil(y_range / cell_size));
  const float origin_x = bounds.at(0); // x_min
  const float origin_y = bounds.at(2); // y_min
  std::cout << "CostMap resolution: " << cell_size << std::endl;

  costmap_2d::Costmap2D* costmap = new costmap_2d::Costmap2D(cells_size_x,
                                                             cells_size_y,
                                                             static_cast<double>(cell_size),
                                                             static_cast<double>(origin_x),
                                                             static_cast<double>(origin_y));

  addPointCloudToCostMap<Point>(input_pointcloud, *costmap);




  //std::cout << "Publishing data as ROS msgs..." << std::endl;


  voronoi_planner::VoronoiPlannerBase planner;

  //planner.init("VoronoiPlanner", *costmap, "/map");


  // Tolerance is not actually used inside Voronoi Planner
  //const double tolerance = 0.0;

  const std::string frame_id = "map";

  geometry_msgs::Pose start;
  //start.header.stamp = ros::Time::now();
  //start.header.frame_id = frame_id;
  start.position.x = 0.0;
  start.position.y = 0.0;
  start.position.z = 0.0;
  start.orientation.w = 1.0;

  geometry_msgs::Pose goal;
  //goal.header.stamp = ros::Time::now();
  //goal.header.frame_id = frame_id;
  goal.position.x = 0.5;
  goal.position.y = 0.0;
  goal.position.z = 0.0;
  goal.orientation.w = 1.0;

  std::vector<std::pair<float, float> > path;
  bool do_pruning = false;
  //std::string method = "prune";
  bool res = planner.plan(costmap, start, goal, path, do_pruning);
  if (!res)
  {
    std::cout << "Failed to plan()" << std::endl;
    return -1;
  }
  std::cout << "Planning succeeded" << std::endl;
  const nav_msgs::OccupancyGrid voronoi_map_msg_no_pruning = planner.getVoronoiMapMsg(frame_id);

  /*
  // Create ROS Msg for path
  nav_msgs::Path path_msg;
  //path_msg.poses.resize(path.size());
  if (!path.empty())
  {
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();
  }
  for (size_t i = 0; i < path.size(); ++i)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = std::get<0>(path.at(i));
    pose.pose.position.y = std::get<1>(path.at(i));
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose);
  }
  */

  path.clear();

  do_pruning = true;
  std::string method = "prune";
  res = planner.plan(costmap, start, goal, path, do_pruning, method);
  if (!res)
  {
    std::cout << "Failed to plan()" << std::endl;
    return -1;
  }
  std::cout << "Planning succeeded" << std::endl;
  const nav_msgs::OccupancyGrid voronoi_map_msg_pruned = planner.getVoronoiMapMsg(frame_id);

  path.clear();

  do_pruning = true;
  method = "alternative";
  res = planner.plan(costmap, start, goal, path, do_pruning, method);
  if (!res)
  {
    std::cout << "Failed to plan()" << std::endl;
    return -1;
  }
  std::cout << "Planning succeeded" << std::endl;
  const nav_msgs::OccupancyGrid voronoi_map_msg_pruned_alternative = planner.getVoronoiMapMsg(frame_id);






/*
  const std::string png_output_path("/home/dbworth/slice.png");

// Load image into 8UC3 matrix
cv::Mat image;
image = cv::imread("/home/dbworth/binary_image1.png");
if (!image.data)
{
    std::cout <<  "Could not open or find the image" << std::endl;
    //return -1;
}
printCvMatType(image);
//cv::namedWindow("Display window", CV_WINDOW_AUTOSIZE);
//cv::imshow("Display window", image);
//cv::waitKey(0); 

// Convert to 8UC1
cv::Mat binary_image(image.size(), CV_8UC1);
cv::cvtColor(image, binary_image, CV_BGR2GRAY);
printCvMatType(binary_image);


  // Choose both false, or one true
  //bool doPrune = false;
  bool doPrune = true;
  //bool doPruneAlternative = false;
  // LOAD PGM MAP AND INITIALIZE THE VORONOI
  //std::ifstream is("/home/dbworth/testmap.pgm");
  //std::ifstream is("/home/dbworth/binary_image1_ASCII.pgm"); // header has P2
  //std::ifstream is("/home/dbworth/binary_image1_raw.pgm"); // header has P5, this works
  std::ifstream is("/home/dbworth/binary_image3_raw.pgm");
  if (!is) {
    std::cerr << "Could not open map file for reading.\n";
    exit(-1);
  }
  bool **map=NULL;
  int sizeX, sizeY;
  loadPGM( is, &sizeX, &sizeY, &map );
  is.close();
  fprintf(stderr, "Map loaded (%dx%d).\n", sizeX, sizeY);

  // create the voronoi object and initialize it with the map
  DynamicVoronoi voronoi;
  voronoi.initializeMap(sizeX, sizeY, map);
  voronoi.update(); // update distance map and Voronoi diagram
  if (doPrune) voronoi.prune();  // prune the Voronoi
  //if (doPruneAlternative) voronoi.updateAlternativePrunedDiagram();  // prune the Voronoi, not supposed in this version
  // Save image file
  voronoi.visualize("/home/dbworth/output.ppm");
  std::cerr << "Generated initial frame.\n";
*/







  //std::cout << "Publishing data as ROS msgs..." << std::endl;



  const sensor_msgs::PointCloud2ConstPtr input_cloud_msg = makePointCloudMsg(*input_pointcloud);
  //const sensor_msgs::PointCloud2ConstPtr cropped_cloud_msg = makePointCloudMsg(*cropped_cloud);

  ros::Publisher input_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("input_cloud", 1);
  //ros::Publisher cropped_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1);

  ros::Publisher voronoi_map_pub1 = nh.advertise<nav_msgs::OccupancyGrid>("voronoi_map_no_pruning", 1);
  ros::Publisher voronoi_map_pub2 = nh.advertise<nav_msgs::OccupancyGrid>("voronoi_map_pruned", 1);
  ros::Publisher voronoi_map_pub3 = nh.advertise<nav_msgs::OccupancyGrid>("voronoi_map_pruned_alternative", 1);


  //ros::Publisher occupancy_grid_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("occupancy_grid_cloud", 1);


  ros::Rate loop_rate(1.0);
  while (ros::ok())
  {
    //occupancy_grid_cloud_pub.publish(occupancy_grid_cloud_msg);
    input_cloud_pub.publish(input_cloud_msg);
    //cropped_cloud_pub.publish(cropped_cloud_msg);

    voronoi_map_pub1.publish(voronoi_map_msg_no_pruning);
    voronoi_map_pub2.publish(voronoi_map_msg_pruned);
    voronoi_map_pub3.publish(voronoi_map_msg_pruned_alternative);


    ros::spinOnce();
    loop_rate.sleep();
  }



  delete costmap;



  return 0;
}
