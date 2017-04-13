/*
Example 16:


Simple loops - CAN DELETE, BUT FIRST TEST USING A SMOOTH END POINT DERIVATIVE



Alt routes



Same as example , but visualized using Rviz.

Find path through a PointCloud using RRT algorithms
from OMPL (Open Motion Planning Libray).
The PointCloud is converted into a PCL Octree, and collision-checking is
performed by checking for points within a sphere (the robot's radius).

The path is smoothed using a centripetal Catmull-Rom Splines
The paths are written as .ply files.

Has optional height constraint. If set, the path will
have a vertical linear "take off" segment until it reaches the min height,
planning occurs within the height constraint, and the
path finishes with a vertical linear "landing" segment.



Usage:

roslaunch planning_through_pointclouds ex16_plan_alternate_routes_ground_robot.launch 




export PATH=$PATH:~/path/to/your/catkin_workspace/devel/lib/planning_through_pointclouds/
cd ~/path/to/your/catkin_workspace/planning_through_pointclouds/data/

ex16_plan_through_pointcloud_ompl_octree

 --input ./pointcloud4-Churchill_Country_Club.ply --start 0.0 0.0 2.0 --goal 90.0 -25.0 1.0 \
--min_height 5.0 --max_height 100 --planner BIT_STAR --max_planning_time 10.0 --output ~/path_out_14_1_churchill.ply


David Butterworth
*/

#include <iostream> // cout
#include <string>
#include <vector>

//#include <boost/algorithm/string.hpp> // to_upper

#include <planning_through_pointclouds/utils.h> // processCommandLine, Timer, range2()
//#include <planning_through_pointclouds/ompl_utils.h> // getPlannerGraphMarker
#include <planning_through_pointclouds/pcl_utils.h> // loadPointCloud, getPathPointCloudFromPoints, addTakeoffAndLanding,
// addVerticalFunnelToPointCloud
#include <planning_through_pointclouds/spline_utils.h> // getCentripetalCatmullRomSpline, getPointsFromSpline
#include <planning_through_pointclouds/octree_map_3d_v4_ompl.h>



#include <pcl/io/ply_io.h> // savePLYFileASCII

#include <spline_library/vector.h>
#include <spline_library/hermite/cubic/cubic_hermite_spline.h>



#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> // PointCloud2ConstPtr


#include <planning_through_pointclouds/ros_utils.h> // makePointCloudMsg


#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>

#include <sstream> //boost/lexical_cast.hpp>

//----------------------------------------------------------------------------//

// Forward declarations:





// Get the point which is a specific distance along a Spline
const Point getPointAtDistanceAlongSpline(const std::shared_ptr<spline_library::Spline<spline_library::Vector3> >& spline, const float distance)
{

  Point point_at_distance;

  const float t_start = 0.0;
  const float t_end = spline->getMaxT();
  //const float dist_step = 0.10; // 10cm

  //float accum_distance = 0.0;
  float delta_dist_accum = 0.0;
  float t = t_start;
  const float t_step = 0.00001; // this must be very small
  spline_library::Vector3 prev_point = spline->getPosition(0.0);

  bool done = false;
  while (!done)
  {
    const spline_library::Vector3 point = spline->getPosition(t);
    const float delta_dist = euclideanDistance(point, prev_point);
    delta_dist_accum += delta_dist;
    //std::cout << "delta_dist_accum: " << delta_dist_accum << std::endl;

    //if (delta_dist > dist_step)
    //if (delta_dist_accum > dist_step)
    if (delta_dist_accum > distance)
    {
      std::cout << "delta_dist_accum: " << delta_dist_accum << std::endl;

      // Return the Point where the distance along the spline is slightly LONGER than desired
      //point_at_distance = Point(point[0], point[1], point[2]);
      //return point_at_distance;

      /*
      t -= t_step;
      const spline_library::Vector3 point_to_add = spline->getPosition(t);
      const float delta_dist_actual = euclideanDistance(point_to_add, prev_point);
      std::cout << "delta_dist_accum: " << delta_dist_accum << std::endl;
      delta_dist_accum -= delta_dist;
      std::cout << "delta_dist_accum: " << delta_dist_accum << std::endl;
      std::cout << "       delta_dist = " << delta_dist << std::endl;
      delta_dist_accum += delta_dist_actual; // MAY NOT NED TO DO THIS
      std::cout << "       delta_dist_actual = " << delta_dist_actual << std::endl;

      std::cout << "delta_dist_accum: " << delta_dist_accum << std::endl;
      */
      // Return the Point where the distance along the spline is slightly SHORTER than desired
      point_at_distance = Point(prev_point[0], prev_point[1], prev_point[2]);
      return point_at_distance;

      done = true;
    }
    else
    {
      //std::cout << "  t = " << t << "  " << point[0] << "," << point[1] << "," << point[2] << std::endl;
      prev_point = point;
    }

    t += t_step;
    
    if (t > t_end)
    {
      done = true;
      std::cout << "t > t_end " << std::endl;

      // TODO: THROW error, point not found
    }
  }

  point_at_distance = Point(0,0,0);
  return point_at_distance;
}



//std::vector<Point> getPath(OctreeMap3D& octree_map_3d, const Point& p0, const Point& p1, const std::string& path_name)
void makePath(OctreeMap3D& octree_map_3d, const Point& p0, const Point& p1, const std::string& path_name,
              std::shared_ptr<spline_library::Spline<spline_library::Vector3> >& spline)
{
  // TEMP
  const PLANNER_TYPE planner_type = BIT_STAR;
  const double min_height = -100; // disable min_height constraint
  const double max_planning_time = 3.0;



  const bool interpolate_path = true;
  const double waypoint_separation = 0.0; // 0 = output waypoints at the collision-checking resolution, which is 10cm

  //timer.reset();
  const std::vector<Point> path = octree_map_3d.findPath(p0, // start
                                                         p1, // goal
                                                         min_height,
                                                         planner_type,
                                                         max_planning_time,
                                                         interpolate_path,
                                                         waypoint_separation);
  //const double time_taken = timer.read();
  //std::cout << "  took " << time_taken << " ms." << std::endl;
  std::cout << "    Path length: " << octree_map_3d.getPathLength() << std::endl;
  std::cout << "    No. waypoints: " << path.size() << std::endl;

  const uint num_checks = octree_map_3d.getNumCollisionChecks();
  std::cout << "    No. of collision checks: " << num_checks << std::endl;
  //std::cout << "    No. of collision checks: " << num_checks << " (" << time_taken/num_checks << " ms per check)" << std::endl;

  ColorPointCloud path_pointcloud;
  std::string outfile;

  // Reduce the number of vertices in the path:
  std::cout << "    Simplifying path..." << std::endl;
  //timer.reset();
  const std::vector<Point> simplified_path = octree_map_3d.getReducedPath();
  //std::cout << "  took " << timer.read() << " ms." << std::endl;
  std::cout << "    Simplified path now has " << simplified_path.size() << " waypoints" << std::endl;

  //std::shared_ptr<spline_library::Spline<spline_library::Vector3> > spline = getCentripetalCatmullRomSpline(simplified_path);
  spline = getCentripetalCatmullRomSpline<Point>(simplified_path);

  const float dist_step = 0.01; // interpolation step = 1cm
  std::vector<Point> splined_path = getPointsFromSpline<Point>(spline, dist_step);

  // Save trajectory
  path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(splined_path);
  //outfile = std::string("/home/dbworth/test.ply");

  std::stringstream ss; //output string stream
  //ss << i;
  ss << path_name;
  outfile = std::string("/home/dbworth/alt_route_") + ss.str() + std::string(".ply");

  std::cout << "    Writing to " << outfile << std::endl;
  pcl::io::savePLYFileASCII(outfile, path_pointcloud);

  //return splined_path;
}



//----------------------------------------------------------------------------//

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;


// TODO: and min AND max height constraint


/*
  std::string pointcloud_file;
  std::vector<float> start;
  std::vector<float> goal;
  double min_height;
  double max_height;
  std::string planner;
  double max_planning_time;
  std::string output_filename;

  const bool result = processCommandLine(argc, argv,
                                         pointcloud_file, start, goal, min_height, max_height,
                                         planner, max_planning_time, output_filename);
  if (!result)
  {
    return EXIT_FAILURE;
  }
  std::cout << "pointcloud_file =  "<< pointcloud_file << std::endl;
  std::cout << "output_filename =  "<< output_filename << std::endl;
  const std::vector<std::string> output_filename_tokens = splitString(output_filename);

  if ((min_height > start.at(2)) || (min_height > goal.at(2)))
  {
    std::cout << "Using min height constraint: " << min_height << std::endl;
  }
*/

  //std::cout << "  width:" << input_pointcloud->width << std::endl;
  //std::cout << "  height:" << input_pointcloud->height << std::endl;
  //std::cout << "  is_dense:" << input_pointcloud->is_dense << std::endl;


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

/*
// 9 Collision points for NSH map
std::vector<Point> col_points;
col_points.push_back(Point(0.3,3.2, 0.2+2.0));
col_points.push_back(Point(2.33,2.41, 0.2+2.0));
col_points.push_back(Point(4.42,1.64, 0.2+2.0));
col_points.push_back(Point(6.32,0.98, 0.2+2.0));
col_points.push_back(Point(9.08,0.14, 0.2+2.0));
col_points.push_back(Point(11.47,-0.41, 0.2+2.0));
col_points.push_back(Point(14.26,-0.92, 0.2+2.0));
col_points.push_back(Point(17.17,-1.37, 0.2+2.0));
col_points.push_back(Point(20.87,-1.74, 0.2+2.0));
*/
//for (int i = 0; i < 10; ++i) {



  PointCloud::Ptr input_pointcloud(new PointCloud());
  if (!loadPointCloud(pointcloud_file, *input_pointcloud))
  {
    return EXIT_FAILURE;
  }
  std::cout << "Loaded PointCloud with " << input_pointcloud->points.size() << " points" << std::endl;

/*
  int i = 3;
  //if (i > 0)
  {
    addCylinderToPointCloud<Point>(col_points.at(i-1),
                             1.0, // radius
                             -2.0, // length & direction
                             0.02, // point_sep
                             input_pointcloud);
  }
*/
  //const Point start_point(start.at(0), start.at(1), start.at(2));
  //const Point goal_point(goal.at(0), goal.at(1), goal.at(2));

  const Point start_point(-2.0, 4.2, 0.2);
  const Point goal_point(23.9, -1.8, 0.2);
  //const PLANNER_TYPE planner_type = BIT_STAR;
  //const double min_height = -100; // disable min_height constraint
  //const double max_planning_time = 3.0;






  //Timer timer;
  std::cout << "Initializing OctreeMap3D..." << std::endl;
  //timer.reset();


  // Half the diameter of DJI S1000.
  const double robot_radius = 1.1 / 2.0;

  const double voxel_size = 0.10;



  OctreeMap3D main_map(input_pointcloud, voxel_size, robot_radius);
  //std::cout << "  took " << timer.read() << " ms." << std::endl;




  std::cout << "Finding main path from start to goal..." << std::endl;

  //std::vector<Point> path = getPath(map, start_point, goal_point, "mainpath",
  std::shared_ptr<spline_library::Spline<spline_library::Vector3> > spline;
  makePath(main_map, start_point, goal_point, "mainpath", spline);



  // TODO: Make   getSplineLength()
  const std::vector<double> d_values = range2(0.0, 27.0, 1.0,
                                              true); // include end-points

printStdVector(d_values);
uint j = 0;
//for (size_t i = 1; i < d_values.size()-1; ++i)
for (size_t i = 1; i < 4; ++i)
{
  std::cout << "Start: " << d_values.at(i-1) << "  Put obs: " << d_values.at(i) << "   end: " << d_values.at(i+1) << std::endl;

  const Point p0 = getPointAtDistanceAlongSpline(spline, d_values.at(i - 1));
  const Point p1 = getPointAtDistanceAlongSpline(spline, d_values.at(i + 1));
  std::cout << "  Sub-path " << j << "  from " << p0 << " to " << p1 << std::endl;
  

  // Copy the original PointCloud and add an obstacle
  PointCloud::Ptr cloud(new PointCloud(*input_pointcloud));
  const Point obstacle_position = getPointAtDistanceAlongSpline(spline, d_values.at(i));
  addCylinderToPointCloud<Point>(obstacle_position,
                             0.3, // radius
                             //1.0, // radius
                             -2.0, // length & direction
                             0.02, // point_sep
                             cloud);

  std::cout << "Initializing OctreeMap3D..." << std::endl;
  OctreeMap3D map(cloud, voxel_size, robot_radius);
  //std::cout << "  took " << timer.read() << " ms." << std::endl;
  std::cout << "Finding sub-path..." << std::endl;
  std::shared_ptr<spline_library::Spline<spline_library::Vector3> > spline2;
  std::stringstream ss;
  ss << j; // path name
  makePath(map, p0, p1, ss.str(), spline2);

  j++; // sub-path counter
}


//} // end for loop

/*
  std::cout << "splined_path2 size = " << splined_path2.size() << std::endl;

  const sensor_msgs::PointCloud2ConstPtr input_cloud_msg = makePointCloudMsg(*input_pointcloud);

  const nav_msgs::PathConstPtr path_msg = makePathMsg(path2);
  const nav_msgs::PathConstPtr path_splined_msg = makePathMsg(splined_path2);

  ros::Publisher input_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("input_cloud", 1);

  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  const double marker_size = 0.4;
  visualization_msgs::Marker start_marker = getPointMarker("start", start_point, marker_size);
  visualization_msgs::Marker goal_marker = getPointMarker("goal", goal_point, marker_size);

  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1);
  ros::Publisher path_splined_pub = nh.advertise<nav_msgs::Path>("path_splined", 1);


  int pub_count = 0;
  int max_pub = 1;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    input_cloud_pub.publish(input_cloud_msg);

    if (pub_count < max_pub)
    {

      pub_count++;
    }

      marker_pub.publish(start_marker);
      marker_pub.publish(goal_marker);
      path_pub.publish(path_msg);
      path_splined_pub.publish(path_splined_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  */

  return 0;
}
