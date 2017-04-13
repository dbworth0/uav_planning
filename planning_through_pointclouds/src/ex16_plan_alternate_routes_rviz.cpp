/*
Example 16:

Alt routes

You must delete all .ind files before running this.
TODO: Check directory for .ind or .ply files matching x-x-x and throw error.


Only input is the .ply file

Indoor #7
pointcloud7_indoor_highbay_no_floor_small.pcd
const Point start_point(-2.0, 4.2, 0.2);
const Point goal_point(23.9, -1.8, 0.2);

Indoor #10
const Point start_point(0.0, 0.0, -0.7);
const Point goal_point(18.0, -5.0, -0.7);



Schenley Park

ex16_plan_alternate_routes_rviz schenley_park.ply

const Point start_point(0.0, 0.0, -0.18);
const Point goal_point(90.0, 10.0, -3.9);



Country Club

ex16_plan_alternate_routes_rviz pointcloud4-Churchill_Country_Club.ply



Carrie Furnace

Short section:

ex16_plan_alternate_routes_rviz pointcloud9_carrie_furnace_1_main.ply


Long section:

ex16_plan_alternate_routes_rviz pointcloud10_carrie_furnace_2_main.ply





NOTE:
the planner creates .ply .length .ind files as it goes, and these are used during planning
and for the robot.

// TODO:
On Churchill map, with min_height = -100
the main path goes under the ground. WHY ??

Also, throws when start point is in collision.
Should save the PointCloud with obstacles.




Usage:

ex16_plan_alternate_routes_rviz pointcloud10_indoor_highbay_8Jan2017_small_no_floor.ply

if you enable ROS in code:
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

#include <fstream>
//#include <fstream>
//#include <iostream>
//#include <iomanip> // setprecision

#include <limits> // max double

#include <stdio.h> // FILE


//#include <boost/algorithm/string.hpp> // to_upper

#include <planning_through_pointclouds/utils.h> // processCommandLine, Timer, range, range2
//#include <planning_through_pointclouds/ompl_utils.h> // getPlannerGraphMarker
#include <planning_through_pointclouds/pcl_utils.h> // loadPointCloud, getPathPointCloudFromPoints, addTakeoffAndLanding,
// addVerticalFunnelToPointCloud
#include <planning_through_pointclouds/spline_utils.h> // getCentripetalCatmullRomSpline, getPointsFromSpline
#include <planning_through_pointclouds/octree_map_3d_v4_ompl.h>
#include <planning_through_pointclouds/ply_io.h> // writePlyFile


#include <pcl/io/ply_io.h> // loadPLYFile
//savePLYFileASCII

#include <pcl/io/pcd_io.h> // savePCDFileASCII


#include <spline_library/vector.h>
#include <spline_library/hermite/cubic/cubic_hermite_spline.h>



//#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h> // PointCloud2ConstPtr

//#include <planning_through_pointclouds/ros_utils.h> // makePointCloudMsg


#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>

#include <sstream> //boost/lexical_cast.hpp>



//#define BOOST_FILESYSTEM_VERSION 3
//#define BOOST_FILESYSTEM_NO_DEPRECATED 
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>

#include <algorithm> // std::reverse

//----------------------------------------------------------------------------//

// Forward declarations:





// Load a path from .ply file and return the path
// as a vector of Point.
template<typename PointT>
const std::vector<PointT> getPathFromPLYFile(const std::string& file_name)
{
  typename pcl::PointCloud<PointT>::Ptr cloud(new PointCloud());
  pcl::io::loadPLYFile(file_name, *cloud);

  std::vector<PointT> path_points;
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    path_points.push_back(cloud->points.at(i));
  }
  //std::cout << "Loaded " << path_points.size() << " points from .ply file" << std::endl;

  return path_points;
}

// Load a path from a .ply file
// where the file is named like '1-0-2.ply'
// with a full path of 'path_prefix/1-0-2.ply'
template<typename PointT>
const std::vector<PointT> loadPathFile(const std::string& path_prefix,
                                       const int path_level, const int path_parent_index, const int path_index)
{
  std::stringstream ss;
  ss << path_level << "-" << path_parent_index << "-" << path_index << ".ply";
  const std::string file_name = path_prefix + ss.str();

  /*
  typename pcl::PointCloud<PointT>::Ptr cloud(new PointCloud());
  pcl::io::loadPLYFile(file_name, *cloud);

  std::vector<PointT> path_points;
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    path_points.push_back(cloud->points.at(i));
  }
  */
  std::vector<PointT> path_points = getPathFromPLYFile<PointT>(file_name);

  //std::cout << "Loaded " << path_points.size() << " points from .ply file" << std::endl;
  std::cout << "Loaded path file " << file_name << " with " << path_points.size() << " points" << std::endl;

  return path_points;
}

// Get the length (distance) along a path
template <typename PointT>
const double getPathLength(const std::vector<PointT>& path)
{

  double accum_distance = 0.0;
  for (size_t i = 0; i < (path.size() - 1); ++i)
  {
    const double delta_dist = static_cast<double>(euclideanDistance(path.at(i), path.at(i+1)));
    accum_distance += delta_dist;
  }

  return accum_distance;
}

// Get the distance along a path, from the start to a point.
template <typename PointT>
const double getDistanceFromStartOfPath(const std::vector<PointT>& path,
                                        const unsigned int point_index)
{

  double accum_distance = 0.0;
  for (unsigned int i = 0; i <= point_index; ++i)
  {
    const double delta_dist = static_cast<double>(euclideanDistance(path.at(i), path.at(i+1)));
    accum_distance += delta_dist;
  }

  return accum_distance;
}

// Get the distance along a path, from a point to the end.
template <typename PointT>
const double getDistanceToEndOfPath(const std::vector<PointT>& path,
                                    const unsigned int point_index)
{

  double accum_distance = 0.0;
  for (unsigned int i = point_index; i < path.size()-1; ++i)
  {
    const double delta_dist = static_cast<double>(euclideanDistance(path.at(i), path.at(i+1)));
    accum_distance += delta_dist;
  }

  return accum_distance;
}

// Get a point on a path, some distance from the start.
// This is similar to getPointAtDistanceAlongSpline(), except it works with a vector of Point.
// 
template <typename PointT>
//const PointT getPointAtDistanceAlongPath(const std::vector<PointT>& path, const float distance)
void getPointAtDistanceAlongPath(const std::vector<PointT>& path,
                                 const float distance,
                                 PointT& point_at_distance,
                                 unsigned int& point_index)
{
  //Point point_at_distance;

  float delta_dist_accum = 0.0;
  unsigned int index = 0;
  PointT prev_point = path.at(index);

  bool done = false;
  while (!done)
  {
    const PointT point = path.at(index);
    const float delta_dist = euclideanDistance(point, prev_point);
    delta_dist_accum += delta_dist;
    //std::cout << "delta_dist_accum: " << delta_dist_accum << std::endl;

    if (delta_dist_accum > distance)
    {
      std::cout << "Path delta_dist_accum: " << delta_dist_accum << std::endl;

      // Return the Point where the distance along the spline is slightly SHORTER than desired
      point_at_distance = PointT(prev_point.x, prev_point.y, prev_point.z);
      point_index = index - 1;
      //return point_at_distance;

      done = true;
      return;
    }
    else
    {
      //std::cout << "  t = " << t << "  " << point[0] << "," << point[1] << "," << point[2] << std::endl;
      prev_point = point;
    }

    index += 1;
    
    if (index > (path.size() - 1))
    {
      //done = true;
      //std::cout << "index > No. points in path" << std::endl;

      std::stringstream error;
      error << "Failed to find point at distance " << distance
            << " along path in getPointAtDistanceAlongPath(),"
            << " delta_dist_accum = " << delta_dist_accum << ","
            << " index > Number of discrete points in path.";
      throw std::runtime_error(error.str());
    }
  }

  // Default, something went wrong
  //point_at_distance = PointT(0,0,0);
  //point_index = 0;
  //return PointT(0,0,0);
}


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
      // throw std::runtime_error(error.str());
    }
  }

  // Default, something went wrong
  point_at_distance = Point(0,0,0);
  return point_at_distance;
}

// Save the length of a spline to file.
// e.g. "0-0-0.length"
void saveSplineLengthFile(const std::string& output_file_prefix,
                          const int path_level, const int path_parent_index, const int path_index,
                          const double length)
{
  std::stringstream ss;
  ss << path_level << "-" << path_parent_index << "-" << path_index << ".length";
  const std::string output_file = output_file_prefix + ss.str();

  //const std::string output_file(argv[binvox_file_indices[0]]);
  std::ofstream* output = new std::ofstream(output_file, std::ios::out); // | std::ios::binary);
  if (!output->good())
  {
    std::cerr << "Error: Could not open output file " << output << "!" << std::endl;
    exit(1);
  }

  *output << length;

  //*output << "translate " << formatFloat(translate.x) << " " << formatFloat(translate.y) << " " << formatFloat(translate.z) << "\n";
  //  *output << static_cast<char>(run_value);
  //  *output << static_cast<char>(run_length);
  
  output->close();

  //std::cout << "done" << std::endl << std::endl;
}

// Read the spline length from file.
// This assumes the file contains only one number.
const double loadSplineLengthFile(const std::string& file_prefix,
                                  const int path_level, const int path_parent_index, const int path_index
                          )
{
  std::stringstream ss;
  ss << path_level << "-" << path_parent_index << "-" << path_index << ".length";
  const std::string input_file = file_prefix + ss.str();

  std::cout << "reading " << input_file << std::endl;

  double length = 0.0;

  std::ifstream* input = new std::ifstream(input_file, std::ios::in);
  if (!input->good())
  {
    std::cerr << "Error: Could not open input file " << input << "!" << std::endl;
    exit(1);
  }

  *input >> length;
  input->close();

  return length;
}




// Convenience function to save path as .ply file
/*
template <typename PointT>
void savePathFile(const std::string& output_file_prefix,
                  const int path_level, const int path_parent_index, const int path_index,
                  const typename pcl::PointCloud<PointT>& path_pointcloud)
{
  std::stringstream ss;
  ss << path_level << "-" << path_parent_index << "-" << path_index << ".ply";
  const std::string outfile = output_file_prefix + ss.str();
  //std::cout << "    Writing to " << outfile << std::endl;
  savePLYFile(outfile, path_pointcloud);
}
*/




// Save a red-colored path to .ply file in ASCII format
// which is used for visualization purposes.
//
// Fields are:
// x, y, z, red, green, blue, alpha
template <typename PointT>
void savePathToPlyFile(//const std::string& output_file_prefix,
                       const std::string& filename,
                       const std::vector<PointT>& path_points,
                       const unsigned int color[3])
{
  const unsigned int color_r = color[0];
  const unsigned int color_g = color[1];
  const unsigned int color_b = color[2];
  typename pcl::PointCloud<ColorPoint> path_pointcloud = getPathPointCloudFromPoints<PointT, ColorPoint>(path_points, color_r, color_g, color_b);

  //const std::string outfile = output_file_prefix + filename + std::string(".ply");
  std::cout << "    in savePathFile(), Writing to " << filename << std::endl;
  savePLYFile(filename, path_pointcloud);
}

// Save a path to .ply file in ASCII format
// with the fields required by the robot:
// x, y, z, speed, wait_time, ind
//
// TODO: maybe ColorPoint should be a parameter
template <typename PointT>
void savePathToPlyFile(//const std::string& output_file_prefix,
                       const std::string& filename,
                       const std::vector<PointT>& path_points,
                       const bool set_z_values_to_zero,
                       const float speed, // Not yet used by path-following controller
                       const float wait_time) // Not yet used by path-following controller
{
  //std::stringstream ss;
  //ss << path_level << "-" << path_parent_index << "-" << path_index << ".ply";
  //const std::string outfile = output_file_prefix + ss.str();
  //const std::string outfile = output_file_prefix + filename + std::string(".ply");

  // We don't need the ColorPoint, we only use x,y,z but this function only supports
  // returning the ColorPoint
  typename pcl::PointCloud<ColorPoint> path_pointcloud = getPathPointCloudFromPoints<PointT, ColorPoint>(path_points);

  // Copy x,y,z points into a linear vector of vertices:
  // x0, y0, z0, x1, y1, z1, x2, y2, z2, ...
  std::vector<float> vertices;
  for (size_t i = 0; i < path_pointcloud.points.size(); ++i)
  {
    vertices.push_back(path_pointcloud.points.at(i).x);
    vertices.push_back(path_pointcloud.points.at(i).y);

    if (set_z_values_to_zero)
    {
      vertices.push_back(0.0);
    }
    else
    {
      vertices.push_back(path_pointcloud.points.at(i).z);
    }
  }

  const size_t num_vertices = vertices.size();
  std::vector<float> speed_values(num_vertices, speed);
  std::vector<float> wait_time_values(num_vertices, wait_time);
  std::vector<int> ind_values = range<int>(0, (num_vertices - 1));

  const bool binary_format = false;
  writePlyFile(filename,
               vertices,
               "speed", speed_values, 
               "wait_time", wait_time_values,
               "ind", ind_values,
               binary_format);
}





// Convenience function to save a path to a file
// e.g. "0-0-0.ply"
// with extra fields required by the robot.
template <typename PointT>
void savePathToFile(const std::string& output_file_prefix,
                    const int path_level, const int path_parent_index, const int path_index,
                    const std::vector<PointT>& path_points,
                    const bool set_z_values_to_zero,
                    const float speed,
                    const float wait_time)
{
  std::stringstream ss;
  ss << path_level << "-" << path_parent_index << "-" << path_index << ".ply";
  const std::string filename = ss.str();

  //savePathToPlyFile<PointT>(output_file_prefix, filename, path_points, set_z_values_to_zero, speed, wait_time);
  savePathToPlyFile<PointT>(output_file_prefix+filename, path_points, set_z_values_to_zero, speed, wait_time);
}

// Convenience function to save a red-colored path to a file
// e.g. "0-0-0_colored.ply"
// which is used for visualization purposes.
template <typename PointT>
void savePathToFile(const std::string& output_file_prefix,
                    const int path_level, const int path_parent_index, const int path_index,
                    const std::vector<PointT>& path_points,
                    const std::string& filename_suffix = "") // 0-0-0_colored.ply
{
  /*
  //typename pcl::PointCloud<PointT> path_pointcloud = getPathPointCloudFromPoints<PointT, PointT>(path_points);
  typename pcl::PointCloud<ColorPoint> path_pointcloud = getPathPointCloudFromPoints<PointT, ColorPoint>(path_points);

  std::stringstream ss;
  ss << path_level << "-" << path_parent_index << "-" << path_index << filename_suffix << ".ply";
  const std::string outfile = output_file_prefix + ss.str();
  std::cout << "    in savePathFile(), Writing to " << outfile << std::endl;
  savePLYFile(outfile, path_pointcloud);
  */

  std::stringstream ss;
  ss << path_level << "-" << path_parent_index << "-" << path_index << filename_suffix << ".ply";
  const std::string filename = ss.str();

  const unsigned int red_color[3] = {255, 0, 0};
  //savePathToPlyFile<PointT>(output_file_prefix, filename, path_points, red_color);
  savePathToPlyFile<PointT>(output_file_prefix+filename, path_points, red_color);
}






void writePlyFile(const std::string& filename,
                  const std::vector<float>& vertices,

                  const std::string& property_4_name,
                  const std::vector<float>& property_4_values,

                  const std::string& property_5_name,
                  const std::vector<float>& property_5_values,

                  const std::string& property_6_name,
                  const std::vector<int>& property_6_values,

                  const bool binary_format);





// Convenience function for debugging.
// Save a PointCloud as .pcd file.
template <typename PointT>
void savePointCloudFile(const std::string& output_file_prefix,
                        const int path_level, const int path_parent_index, const int path_index,
                        typename pcl::PointCloud<PointT>& cloud
                     )
{
  //typename pcl::PointCloud<PointT> path_pointcloud = getPathPointCloudFromPoints<PointT, PointT>(path_points);
  //typename pcl::PointCloud<ColorPoint> path_pointcloud = getPathPointCloudFromPoints<PointT, ColorPoint>(path_points);

  std::stringstream ss;
  ss << path_level << "-" << path_parent_index << "-" << path_index << ".pcd";
  const std::string outfile = output_file_prefix + ss.str();
  std::cout << "    in savePointCloudFile(), Writing to " << outfile << std::endl;
  //savePLYFile(outfile, cloud);

  pcl::io::savePCDFileASCII(outfile, cloud);
}


// TODO: use boost filesystem?
//
// move to utils
// #include <stdio.h> // FILE
const bool checkFileExists(const std::string& name)
{
  if (FILE* file = fopen(name.c_str(), "r"))
  {
    fclose(file);
    return true;
  }
  else
  {
    return false;
  }   
}


// Read the .ind file and return information about the child paths.
//
// The first line of the file is a single integer, being the number
// of subsequent paths.
// Each subsequent line contains information for one child path.
void getPathsFromIndexFile(const std::string& filename,
                           std::vector<std::vector<unsigned int> >& paths)
{
  unsigned int num_child_paths = 0;

  std::ifstream input(filename, std::ios::in);
  if (!input.good())
  {
    std::cerr << "Error: Could not open input file '" << filename << "' in getPathsFromIndexFile()" << std::endl;
    exit(1);
  }
  bool first_line = true;
  std::string line;
  for (std::string line; std::getline(input, line); )
  {
    std::stringstream iss(line);

    if (first_line)
    {
      iss >> num_child_paths;
      first_line = false;
      //std::cout << "Read num_child_paths = " << num_child_paths << " from file" << std::endl;
    }
    else
    {
      // Read the values from this line into a vector
      int value;
      std::vector<unsigned int> line_values;
      while (iss >> value)
      {
        line_values.push_back(value);
      }
      //std::cout << "Read line:" << std::endl;
      printStdVector(line_values);
      paths.push_back(line_values);
    }
  }

  // Sanity check
  if (paths.size() != num_child_paths)
  {
    throw std::runtime_error("Number of child paths read from file does not match the number of lines read.");
  }
}



  
  



/*
void saveIndexFile(const std::string& output_file_prefix,

                   unsigned int index_file_level,
                   unsigned int index_file_parent_index,
                   unsigned int index_file_index,

                   unsigned int child_path_level,

                   unsigned int loop_start_level, unsigned int loop_start_file_index, unsigned int loop_start_point_index,
                   unsigned int loop_end_level, unsigned int loop_end_file_index, unsigned int loop_end_point_index,

                   unsigned int child_path_index,

                   unsigned int obstacle_point_index // custom field
                   )
                   */

// PUT THIS IN UTILS

void removeSubstring(std::string& s,
                     const std::string& p)
{
  std::string::size_type n = p.length();

  for (std::string::size_type i = s.find(p); i != std::string::npos; i = s.find(p))
  {
    s.erase(i, n);
  }
}

/*
void removeSubstring(std::string& s,
                     char* p)
{
  std::string substring = p;
  removeSubstring(s, substring);
}
*/


const std::vector<std::string> getIndexFiles(const std::string& output_file_prefix,
                                             const std::string& file_level,
                                             const std::string& file_parent_index,
                                             const std::string& file_index,
                                             const std::string& file_extension = ".ind")
                                             //const int path_level, const int path_parent_index, const int path_index)
{

  std::vector<std::string> matching_files;

  // Regular Expression to match (PERL format)
  // [0-9]* matches any number of digits e.g. 0 or 10 or 150
  // \. matches the dot character
  std::stringstream pattern;
  //pattern << parent_level <<"-[0-9]*-" << path_parent_index << ".ind";
  pattern << file_level << "-" << file_parent_index << "-" << file_index << file_extension;
  const boost::regex my_filter(pattern.str());
  std::cout << "  RegEx: " << pattern.str() << std::endl;

  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator it(output_file_prefix); it != end_itr; ++it)
  {
    // Skip if not a file
    if (!boost::filesystem::is_regular_file(it->status()))
    {
      continue;
    }

    boost::smatch what;
    if (boost::regex_match(it->path().filename().string(), what, my_filter))
    {
      std::cout << "  Matched filename: " << it->path().filename().string() << std::endl;
      matching_files.push_back(it->path().filename().string());
    }  
  }

  return matching_files;
}

const std::string makeString(const int value)
{
  std::stringstream ss;
  ss << value;
  return ss.str();
}

const std::string makeString(const unsigned int value)
{
  std::stringstream ss;
  ss << value;
  return ss.str();
}

// Get the index of the end point of a child path.
// The location of this point is assumed to be on the main path.
//const unsigned int getEndPositionOfPath(

// Get information about a path from its parent's .ind file
//const bool 
void getPathInfo(
//void readIndexFile(
                 const std::string& output_file_prefix,
                 const unsigned int path_level,
                 const unsigned int path_parent_index,
                 const unsigned int path_index,

                 unsigned int& start_level,
                 unsigned int& start_path_index,
                 unsigned int& start_point_index,

                 unsigned int& end_level,
                 unsigned int& end_path_index,
                 unsigned int& end_point_index,

                 unsigned int& child_path_index,

                 unsigned int& obstacle_point_index,

                 //std::string& path_info_filename // 1-0-0.ind
                 std::vector<unsigned int>& path_info_file_indices
                     )

{
  //namespace fs = ::boost::filesystem;
  if (!boost::filesystem::exists(output_file_prefix) || !boost::filesystem::is_directory(output_file_prefix))
  {
    throw std::runtime_error("Directory " + output_file_prefix + " does not exist in getPathInfo()");
  }

  std::cout << "\n  in getPathInfo(" << path_level << "-" << path_parent_index << "-" << path_index << ")" << std::endl;

  

  //if (path_level > 0)
  if (path_level < 1)
  {
    throw std::runtime_error("Called getPathInfo() with path_level = 0");
    //return false;
  }
  
  // The path's info is stored in its parent's file
  const unsigned int parent_level = path_level - 1;


  //std::vector<std::string> matching_files;

  std::vector<std::string> matching_files = getIndexFiles(output_file_prefix,
                                                          makeString(parent_level),
                                                          std::string("[0-9]*"),
                                                          makeString(path_parent_index)
                                                          );




  /*
    

  // Regular Expression to match (PERL format)
  // [0-9]* matches any number of digits e.g. 0 or 10 or 150
  // \. matches the dot character
  std::stringstream pattern;
  pattern << parent_level <<"-[0-9]*-" << path_parent_index << ".ind";
  const boost::regex my_filter(pattern.str());
  std::cout << "  RegEx: " << pattern.str() << std::endl;

  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator it(output_file_prefix); it != end_itr; ++it)
  {
    // Skip if not a file
    if (!boost::filesystem::is_regular_file(it->status()))
    {
      continue;
    }

    boost::smatch what;
    if (boost::regex_match(it->path().filename().string(), what, my_filter))
    {
      std::cout << "  Matched filename: " << it->path().filename().string() << std::endl;
      matching_files.push_back(it->path().filename().string());
    }  
  }
  */

  // Debug:
  //std::cout << "\n  getPathInfo():" << std::endl;
  //std::cout << "  matching files:" << std::endl;
  //printStdVector(matching_files);

  // There should be exactly 1 file with a matching filename
  if (matching_files.size() == 0)
  {
    std::stringstream error;
    error << "No files found matching '" << parent_level <<"-*-" << path_parent_index << ".ind' in getPathInfo()";
    throw std::runtime_error(error.str());
  }
  if (matching_files.size() > 1)
  {
    std::cout << "  Matched filenames:" << std::endl;
    printStdVector(matching_files);
    std::stringstream error;
    error << "Found more than one file matching '" << parent_level <<"-*-" << path_parent_index << ".ind' in getPathInfo()";
    throw std::runtime_error(error.str());
  }



  const std::string filename = output_file_prefix + matching_files.at(0);
  //const unsigned int parent_index = 0; // HOW TO GET THIS VALUE ?

  // Load the .ind file that contains this path's information
  /*
  const unsigned int parent_level = path_level - 1;
  std::stringstream ss;
  ss << parent_level << "-" << 0 << "-" << path_parent_index << ".ind"; // HOW TO GET THIS VALUE ?
  const std::string filename = output_file_prefix + ss.str();
  bool file_exists = checkFileExists(filename);
  if (!file_exists)
  {
    throw std::runtime_error("File '" + filename + "' does not exist in getEndPositionOfPath()");
  }
  */

  //const std::string output_file = output_file_prefix + ss.str();

  //const unsigned int parent_level = level - 1;
  //onst unsigned int index

  //unsigned int num_child_paths = 0;
  std::vector<std::vector<unsigned int> > child_paths;
  getPathsFromIndexFile(filename, child_paths);

  bool found_path_info = false;
  unsigned int path_info_index = 0;
  for (size_t i = 0; i < child_paths.size(); ++i)
  {
    // Debug:
    //std::cout << "  checking " << child_paths.at(i).at(0) << "-" << child_paths.at(i).at(2) << "-" << child_paths.at(i).at(7) << std::endl;

    if ((child_paths.at(i).at(0) == path_level) && (child_paths.at(i).at(2) == path_parent_index) && (child_paths.at(i).at(7) == path_index))
    {
      std::cout << "    Found match" << std::endl;
      path_info_index = i;
      found_path_info = true;
      break;
    }
  }
  
  if (!found_path_info)
  {
    std::stringstream error;
    error << "Failed to find info for path '" << path_level << "-" << path_parent_index << "-" << path_index
          << "' inside file '" << filename << " in getPathInfo()";
    throw std::runtime_error(error.str());
  }

  if (child_paths.at(path_info_index).size() != 9)
  {
    std::stringstream error;
    error << "Child path info contains " << child_paths.size() << " values in getPathInfo()";
    throw std::runtime_error(error.str());
  }

  if (child_paths.at(path_info_index).at(4) != 0)
  {
    // Currently this function expects that the end point is on the main path
    throw std::runtime_error("End point is not on the main path (level = 0) in getPathInfo()");
  }

  //const unsigned int end_point_index = child_paths.at(path_info_index).at(6);
  //std::cout << "  end point index: " << end_point_index << std::endl;

  start_level          = child_paths.at(path_info_index).at(1);
  start_path_index     = child_paths.at(path_info_index).at(2);
  start_point_index    = child_paths.at(path_info_index).at(3);

  end_level            = child_paths.at(path_info_index).at(4);
  end_path_index       = child_paths.at(path_info_index).at(5);
  end_point_index      = child_paths.at(path_info_index).at(6);

  child_path_index     = child_paths.at(path_info_index).at(7);

  obstacle_point_index = child_paths.at(path_info_index).at(8); // custom 9th field

  std::string temp_string = matching_files.at(0);
  removeSubstring(temp_string, ".ind");
  path_info_file_indices = splitStringAsType<unsigned int>(temp_string, '-');


  if (path_info_file_indices.size() != 3)
  {
    throw std::runtime_error("path_info_file_indices should have 3 numbers in getPathInfo()");
  }



  // Debug:
  //std::cout << "  getPathInfo: " << std::endl;
  //std::cout << "  " << child_paths.at(path_info_index).at(0)
  //          << " "
  //          << child_paths.at(path_info_index).at(1) << "," << child_paths.at(path_info_index).at(2) << "," << child_paths.at(path_info_index).at(3)
  //          << " "
  //          << child_paths.at(path_info_index).at(4) << "," << child_paths.at(path_info_index).at(5) << "," << child_paths.at(path_info_index).at(6)
  //          << " "
  //          << child_paths.at(path_info_index).at(7)
  //          << " "
  //          << child_paths.at(path_info_index).at(8) << std::endl;


  //const double distance_to_end_point = getDistanceFromStartOfPath<Point>(parent_path, end_point_index);
  //std::cout << "  distance from start: " << distance_to_end_point << std::endl;




  //} // endif path_level > 0
  //return true;
}



// FILE UTILS ?  io utils?

// File all .ind files in the output directory,
// Remove obstacle point indices from .ind files, because that information was
// only required during planning and is not part of the standard file format.
void cleanIndexFiles(const std::string& output_file_prefix)
{
  const std::vector<std::string> index_files = getIndexFiles(output_file_prefix, "[0-9]*", "[0-9]*", "[0-9]*");
  for (size_t i = 0; i < index_files.size(); ++i)
  {
    std::cout << "Cleaning file " << index_files.at(i) << std::endl;

    const std::string filename = output_file_prefix + index_files.at(i);

    std::vector<std::vector<unsigned int> > child_paths;
    getPathsFromIndexFile(filename, child_paths);

    for (size_t j = 0; j < child_paths.size(); ++j)
    {
      if (child_paths.at(j).size() != 9)
      {
        // There should be 9 values on each row
        std::stringstream error;
        error << "Child path info contains " << child_paths.size() << " values in getPathInfo()";
        throw std::runtime_error(error.str());
      }
    }

    // Remove the 9th value
    std::vector<std::vector<unsigned int> > updated_child_paths;
    for (size_t j = 0; j < child_paths.size(); ++j)
    {
      std::vector<unsigned int> path_info;
      for (size_t k = 0; k < 8; ++k)
      {
        path_info.push_back(child_paths.at(j).at(k));
      }
      updated_child_paths.push_back(path_info);
    }

    // Debug:
    //std::cout << "  updated child paths:" << std::endl;
    //for (size_t j = 0; j < updated_child_paths.size(); ++j)
    //{
    //  printStdVector(updated_child_paths.at(j));
    //}

    // Over-write the file
    std::ofstream* output = new std::ofstream(filename, std::ios::out);
    if (!output->good())
    {
      std::stringstream error;
      error << "Error: Could not open output file '" << filename << "' in cleanIndexFiles()";
      throw std::runtime_error(error.str());
    }
    // Write the first line
    const unsigned int num_child_paths = updated_child_paths.size();
    *output << num_child_paths << "\n";
    // Write 1 line for each child path
    for (int j = 0; j < updated_child_paths.size(); ++j)
    {
      for (int k = 0; k < updated_child_paths.at(j).size(); ++k)
      {
        *output << updated_child_paths.at(j).at(k);

        if (k < (updated_child_paths.at(j).size() - 1))
        {
          *output << " ";
        }
      }
      *output << "\n";
    }
    output->close();
  }
}


void deleteIndexFiles(const std::string& output_file_prefix)
{
  const std::vector<std::string> index_files = getIndexFiles(output_file_prefix, "[0-9]*", "[0-9]*", "[0-9]*", ".ind");
  for (size_t i = 0; i < index_files.size(); ++i)
  {
    std::cout << "Deleting file " << index_files.at(i) << std::endl;

    const std::string filename = output_file_prefix + index_files.at(i);

    boost::filesystem::remove_all(filename);
  }
}


// Remove non-colored path files
// e.g. delete 0-0-0.ply
// but keep 0-0-0_colored.ply
void deleteNonColoredPathFiles(const std::string& output_file_prefix)
{
  const std::vector<std::string> index_files = getIndexFiles(output_file_prefix, "[0-9]*", "[0-9]*", "[0-9]*", ".ply");
  for (size_t i = 0; i < index_files.size(); ++i)
  {
    std::cout << "Deleting file " << index_files.at(i) << std::endl;

    const std::string filename = output_file_prefix + index_files.at(i);

    boost::filesystem::remove_all(filename);
  }
}

void deleteLengthFiles(const std::string& output_file_prefix)
{
  const std::vector<std::string> index_files = getIndexFiles(output_file_prefix, "[0-9]*", "[0-9]*", "[0-9]*", ".length");
  for (size_t i = 0; i < index_files.size(); ++i)
  {
    std::cout << "Deleting file " << index_files.at(i) << std::endl;

    const std::string filename = output_file_prefix + index_files.at(i);

    boost::filesystem::remove_all(filename);
  }
}







// Get the end position of a path, which is the distance
// along its parent path.
// Also get the obstacle point around which this path was created,
// and the obstacle points for all upper-level paths.
//void getParentEndPositionAndObstaclePoints(const std::string& output_file_prefix,
void getParentEndPointAndObstaclePoints(const std::string& output_file_prefix,
                                           const unsigned int path_level,
                                           const unsigned int path_parent_index,
                                           const unsigned int path_index,

                                           //parent_path_level, parent_path_parent_index, parent_path_index,
                                           //double& parent_end_position,
                                           unsigned int& parent_end_point_index,
                                           std::vector<Point>& parent_obstacle_points
                                           )
{
  std::cout << "\n in getParentEndPositionAndObstaclePoints() " << path_level << "-" << path_parent_index << "-" << path_index << std::endl;

  if (path_level == 0) // && (path_parent_index == 0) && (path_index ++ 0)
  {
    // Should only call this function for level 1, level 2, etc.
    throw std::runtime_error("Tried to call getParentEndPositionAndObstaclePoints() with path_level = 0");
  }

  unsigned int path_level_        = path_level;
  unsigned int path_parent_index_ = path_parent_index;
  unsigned int path_index_        = path_index;

  bool got_parent_end_point = false;
  bool done = false;
  while (!done)
  {

    unsigned int parent_path_start_level;
    unsigned int parent_path_start_path_index;
    unsigned int parent_path_start_point_index;
    
    unsigned int parent_path_end_level;
    unsigned int parent_path_end_path_index;
    unsigned int parent_path_end_point_index;

    unsigned int child_path_index;

    unsigned int parent_path_obstacle_point_index;

    std::vector<unsigned int> path_info_file_indices;

    getPathInfo(output_file_prefix,
                //parent_path_level, parent_path_parent_index, parent_path_index,
                path_level_,
                path_parent_index_,
                path_index_,

                parent_path_start_level,
                parent_path_start_path_index,
                parent_path_start_point_index,

                parent_path_end_level,
                parent_path_end_path_index,
                parent_path_end_point_index,

                child_path_index,

                parent_path_obstacle_point_index,

                path_info_file_indices
                );

    //std::cout << "  getPathInfo() returned " << path_info_file_indices.at(0) << "," << path_info_file_indices.at(1) << "," << path_info_file_indices.at(2) << std::endl;

    if (!got_parent_end_point)
    {
      // Get the end point index of the parent path
      got_parent_end_point = true;
      parent_end_point_index = parent_path_end_point_index;
    }

    // Get the obstacle point for this parent path
    // (and all upper level paths).
    const std::vector<Point> parent_path = loadPathFile<Point>(output_file_prefix,
                                                               path_info_file_indices.at(0), // filename level
                                                               path_info_file_indices.at(1), // filename parent index
                                                               path_info_file_indices.at(2)); // filename index
    const Point parent_path_obstacle_point = parent_path.at(parent_path_obstacle_point_index);
    parent_obstacle_points.push_back(parent_path_obstacle_point);

    if ((path_info_file_indices.at(0) == 0) && (path_info_file_indices.at(1) == 0) && (path_info_file_indices.at(2) == 0))
    {
      //std::cout << "getParentEndPositionAndObstaclePoints() is done, reached main path" << std::endl;
      done = true;
    }
    else
    {
      // Get info for the parent's parent
      path_level_        = path_info_file_indices.at(0); // level         parent_path_start_level;
      path_parent_index_ = path_info_file_indices.at(1); // parent index  parent_path_start_path_index;
      path_index_        = path_info_file_indices.at(2); // index         child_path_index;
    }
  } // while !done

  if (parent_obstacle_points.size() == 0)
  {
    throw std::runtime_error("Found no parent path obstacle points in getParentEndPositionAndObstaclePoints()");
  }
}


                    



// Create or update a .ind file.
//
// This is designed to be called by child paths, starting from level 1.
// It will generate the .ind file if it doesn't yet exist.
//
// obstacle_point_index // custom field 9th field
//
// rename updateIndexFile
void saveIndexFile(const std::string& output_file_prefix,

                   unsigned int index_file_level,
                   unsigned int index_file_parent_index,
                   unsigned int index_file_index,

                   unsigned int child_path_level,

                   unsigned int loop_start_level, unsigned int loop_start_file_index, unsigned int loop_start_point_index,
                   unsigned int loop_end_level, unsigned int loop_end_file_index, unsigned int loop_end_point_index,

                   unsigned int child_path_index,

                   unsigned int obstacle_point_index // custom field
                   )
{
  if (child_path_level < 1)
  {
    //std::cout << "ERROR: don't need to call saveIndexFile() for level 0" << std::endl;
    throw std::runtime_error("don't need to call saveIndexFile() for level 0");
  }

  std::stringstream ss;
  ss << index_file_level << "-" << index_file_parent_index << "-" << index_file_index << ".ind";
  const std::string output_file = output_file_prefix + ss.str();

  std::vector<std::vector<unsigned int> > child_paths;

  bool file_exists = checkFileExists(output_file);
  if (file_exists)
  {
    // File exists, read in the existing data
    std::cout << "File exists " << std::endl;

    getPathsFromIndexFile(output_file, child_paths);
    //getPathsFromIndexFile(const std::string& filename, std::vector<std::vector<unsigned int> >& paths);
  }
  else
  {
    std::cout << "File does not exist" << std::endl;
  }

  // Append the new child path
  const std::vector<unsigned int> new_child_path = {child_path_level,
                                                    loop_start_level, loop_start_file_index, loop_start_point_index,
                                                    loop_end_level, loop_end_file_index, loop_end_point_index,
                                                    child_path_index,
                                                    obstacle_point_index // custom 9th field
                                                   };
  child_paths.push_back(new_child_path);

  std::ofstream* output = new std::ofstream(output_file, std::ios::out);
  if (!output->good())
  {
    std::cerr << "Error: Could not open output file " << output << "!" << std::endl;
    exit(1);
  }

  // Update the first line
  const unsigned int num_child_paths = child_paths.size();
  *output << num_child_paths << "\n";

  // Write 1 line for each child path
  for (int i = 0; i < child_paths.size(); ++i)
  {
    for (int j = 0; j < child_paths.at(i).size(); ++j)
    {
      *output << child_paths.at(i).at(j);

      if (j < (child_paths.at(i).size() - 1))
      {
        *output << " ";
      }
    }
    *output << "\n";
  }

  output->close();
}
    /*
    unsigned int num_child_paths = 0;

    std::ifstream input(output_file, std::ios::in);
    if (!input.good())
    {
      std::cerr << "Error: Could not open input file " << input << "!" << std::endl;
      exit(1);
    }
    bool first_line = true;
    std::string line;
    for (std::string line; std::getline(input, line); )
    {
      std::stringstream iss(line);

      if (first_line)
      {
        iss >> num_child_paths;
        first_line = false;
        //std::cout << "Read num_child_paths = " << num_child_paths << " from file" << std::endl;
      }
      else
      {
        // Read the values from this line into a vector
        int value;
        std::vector<unsigned int> line_values;
        while (iss >> value)
        {
          line_values.push_back(value);
        }
        //std::cout << "Read line:" << std::endl;
        printStdVector(line_values);
        child_paths.push_back(line_values);
      }
    }
    */


enum VIRTUAL_OBSTACLE_TYPE
{
  VERTICAL_CYLINDER = 1
};

// Add virtual obstacles to a PointCloud.
//
// This adds a new obstacle, as well as recursively adding obstacles from parent paths.
//
// obstacle_position = the position (along the parent path) to add the new virtual obstacle.
//
// Obstacles are rendered as 3D cylinders, with a height the same as the PointCloud itself.
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr addVirtualObstacles(const typename pcl::PointCloud<PointT>::Ptr input_cloud,
                                                          const int level,
                                                          const std::vector<Point>& parent_obstacle_points,
                                                          const PointT obstacle_position,
                                                          const int obstacle_type, // = VERTICAL_CYLINDER
                                                          const float obstacle_radius, // = 4.0;
                                                          const float obstacle_min_z,
                                                          const float obstacle_max_z
                                                 )
                                                 //const double obstacle_position,
                         //typename pcl::PointCloud<PointT>::Ptr& pointcloud)
{
  if (level < 1)
  {
    throw std::runtime_error("level must be > 0 in addVirtualObstacles()");
  }

  //const float obstacle_radius = 0.5;
  //const float obstacle_radius = 1.0;
  //const float obstacle_radius = 2.0; // Note robot radius + obstacle radius MUST BE <  min length parameter 3 meters
  //const float obstacle_radius = 6.0;
  //const float obstacle_radius = 4.0;

  // Copy the original PointCloud
  typename pcl::PointCloud<PointT>::Ptr output_cloud(new PointCloud(*input_cloud));

  // Add previous obstacles
  for (size_t i = 0; i < parent_obstacle_points.size(); ++i)
  {
    addCylinderToPointCloud<Point>(PointT(parent_obstacle_points.at(i).x, parent_obstacle_points.at(i).y, obstacle_min_z),
                                   obstacle_radius, // radius               TODO: THIS SHOULD BE PARAMETER     // all these params are Floats
                                   (obstacle_max_z - obstacle_min_z), // length & direction
                                   0.02, // point_separation
                                   output_cloud);
  }

  // Add a new obstacle
  addCylinderToPointCloud<Point>(PointT(obstacle_position.x, obstacle_position.y, obstacle_min_z),
                                 obstacle_radius, // radius               TODO: THIS SHOULD BE PARAMETER     // all these params are Floats
                                 (obstacle_max_z - obstacle_min_z), // length & direction
                                 0.02, // point_separation
                                 output_cloud);

  return output_cloud;
}





// Plan a path and return the path 
bool planPath(
              //const int path_level, const int path_parent_index, const int path_index,
              OctreeMap3D& octree_map_3d,
              const Point& p0, const Point& p1,
              //std::vector<Point>& splined_path,
              //double& path_length
              //const std::string& path_name,
              //std::shared_ptr<spline_library::Spline<spline_library::Vector3> >& spline
              std::vector<Point>& simplified_path
              )
{
  // TEMP
  const PLANNER_TYPE planner_type = BIT_STAR;

  //const double min_height = -100; // disable min_height constraint
  const double min_height = 5.0;


  //const double max_planning_time = 3.0;
  //const double max_planning_time = 4.0;
  const double max_planning_time = 10.0;



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

  if (path.size() == 0)
  {
    std::cout << "  Planner failed to find path, path size = 0" << std::endl;
    return false;
  }

  const uint num_checks = octree_map_3d.getNumCollisionChecks();
  std::cout << "    No. of collision checks: " << num_checks << std::endl;
  //std::cout << "    No. of collision checks: " << num_checks << " (" << time_taken/num_checks << " ms per check)" << std::endl;

  //ColorPointCloud path_pointcloud;
  std::string outfile;

  // Reduce the number of vertices in the path:
  std::cout << "    Simplifying path..." << std::endl;
  //timer.reset();
  //const std::vector<Point> 
  simplified_path = octree_map_3d.getReducedPath();
  //std::cout << "  took " << timer.read() << " ms." << std::endl;
  std::cout << "    Simplified path now has " << simplified_path.size() << " waypoints" << std::endl;

  //std::shared_ptr<spline_library::Spline<spline_library::Vector3> > spline = getCentripetalCatmullRomSpline(simplified_path);
  //spline = getCentripetalCatmullRomSpline<Point>(simplified_path);

  //const float dist_step = 0.01; // interpolation step = 1cm
  //std::vector<Point> splined_path = getPointsFromSpline<Point>(spline, dist_step);
  //std::vector<Point> splined_path;
  //double spline_length;
  //getPointsFromSpline<Point>(spline, dist_step, splined_path, path_length); //spline_length);
  //std::cout << "    Discretized path length: " << spline_length << std::endl;

  //return splined_path;

  // Save trajectory
  //ColorPointCloud path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(splined_path);
  //outfile = std::string("/home/dbworth/test.ply");

  
  /*
  std::string outfile_prefix("/home/dbworth/");


  // Save path as .ply file
  // e.g. "0-0-0.ply"
  std::stringstream ss;
  ss << path_level << "-" << path_parent_index << "-" << path_index << ".ply";
  outfile = outfile_prefix + ss.str();
  std::cout << "    Writing to " << outfile << std::endl;
  savePLYFile(outfile, path_pointcloud);

  // Save length of spline to file
  // e.g. "0-0-0.length"
  saveSplineLengthFile(outfile_prefix, path_level, path_parent_index, path_index, spline_length);
  */

  return true;
}




// Plan a path and return the path and it's length.
//
// This allows you to specify the points beyond the path end-points,
// which are used to define the tanget.
const bool makePath(OctreeMap3D& octree_map_3d,
                    const Point& p0, const Point& p1,
                    const Point& point_before_start_point,
                    const Point& point_after_end_point,
                    const float endpoint_smoothing_extend_distance,
                    const float interpolation_step,
                    std::vector<Point>& splined_path,
                    double& path_length)
{
  //std::cout << "\n in makePath() \n" << std::endl;

  std::vector<Point> simplified_path;
  bool res = planPath(octree_map_3d, p0, p1, simplified_path);
  if (res == false)
  {
    return false;
  }

  std::shared_ptr<spline_library::Spline<spline_library::Vector3> > spline = getCentripetalCatmullRomSpline(simplified_path,
                                                                                                            point_before_start_point,
                                                                                                            point_after_end_point,
                                                                                                            endpoint_smoothing_extend_distance);
  getPointsFromSpline<Point>(spline, interpolation_step, splined_path, path_length); //spline_length);
  //std::cout << "    Discretized path length: " << path_length << std::endl;

  return true;
}

// Plan a path and return the path and it's length.
const bool makePath(OctreeMap3D& octree_map_3d,
                    const Point& p0, const Point& p1,
                    const float interpolation_step,
                    std::vector<Point>& splined_path,
                    double& path_length)
{
  //std::cout << "\n in makePath() 2 \n" << std::endl;

  std::vector<Point> simplified_path;
  bool res = planPath(octree_map_3d, p0, p1, simplified_path);
  if (res == false)
  {
    return false;
  }

  std::shared_ptr<spline_library::Spline<spline_library::Vector3> > spline = getCentripetalCatmullRomSpline(simplified_path);

  getPointsFromSpline<Point>(spline, interpolation_step, splined_path, path_length); //spline_length);
  //std::cout << "    Discretized path length: " << path_length << std::endl;

  return true;
}



// Plan a path and save it to file
//
// DONT USE THIS BECAUSE ONLY CALL THIS FUNCTION ONCE
/*
bool makePath(const int path_level, const int path_parent_index, const int path_index,
              OctreeMap3D& octree_map_3d,
              const Point& p0, const Point& p1
              )
{
  std::vector<Point> splined_path;
  double path_length;
  //bool res = makePath(path_level, path_parent_index, path_index, octree_map_3d, p0, p1, splined_path, path_length);
  bool res = makePath(octree_map_3d, p0, p1, splined_path, path_length);
  if (res == false)
  {
    return false;
  }

  // Save trajectory
  //ColorPointCloud path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(splined_path);
  //outfile = std::string("/home/dbworth/test.ply");

  std::string outfile_prefix("/home/dbworth/");

  // Save path as .ply file
  // e.g. "0-0-0.ply"
  //savePathFile(outfile_prefix, path_level, path_parent_index, path_index, path_pointcloud);
  savePathFile(outfile_prefix, path_level, path_parent_index, path_index, splined_path);


  // Save length of spline to file
  // e.g. "0-0-0.length"
  saveSplineLengthFile(outfile_prefix, path_level, path_parent_index, path_index, path_length);

  return true;
}
*/


/*
// OLD VERSION, maybe dont need
//std::vector<Point> getPath(OctreeMap3D& octree_map_3d, const Point& p0, const Point& p1, const std::string& path_name)
void makePath(OctreeMap3D& octree_map_3d,
              const Point& p0, const Point& p1,
              const std::string& path_name,
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

  if (path.size() == 0)
  {
    std::cout << "  Planner failed to find path, path size = 0" << std::endl;
    return;
  }

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
  //std::vector<Point> splined_path = getPointsFromSpline<Point>(spline, dist_step);
  std::vector<Point> splined_path;
  double spline_length;
  getPointsFromSpline<Point>(spline, dist_step, splined_path, spline_length);
  std::cout << "    Discretized path length: " << spline_length << std::endl;

  // Save trajectory
  path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(splined_path);
  //outfile = std::string("/home/dbworth/test.ply");

  std::stringstream ss; //output string stream
  //ss << i;
  ss << path_name;
  outfile = std::string("/home/dbworth/alt_route_") + ss.str() + std::string(".ply");

  std::cout << "    Writing to " << outfile << std::endl;
  //pcl::io::savePLYFileASCII(outfile, path_pointcloud);
  savePLYFile(outfile, path_pointcloud);

  //return splined_path;
}
*/

/*
// Get the indices of higher level paths
const std::vector<unsigned int> getParentPaths(const unsigned int level, const unsigned int num_levels)
{
  //for (unsigned int level = 0; level < num_levels; ++level)
  for (unsigned int i = 0; i < level; ++i)
  {
    for (unsigned int j = 0; j < 1000; ++j)
    {
      for (unsigned int j = 0; j < 1000; ++j)
      {

      }
    }

  }

  std::vector<unsigned int> parent_path_indices = 
}
*/
// Get the indices of higher level paths
const std::vector<std::vector<unsigned int> > getLevelPathIndices(const unsigned int level,
                                                                  const std::vector<std::vector<unsigned int> >& path_indices)
{
  std::vector<std::vector<unsigned int> > indices;

  std::cout << "  Level " << level << " has these path files:" << std::endl;
  //for (unsigned int level = 0; level < num_levels; ++level)
  for (unsigned int i = 0; i < path_indices.size(); ++i)
  {
    if (path_indices.at(i).at(0) == level)
    {
      std::cout << "    " << path_indices.at(i).at(0) << "-" << path_indices.at(i).at(1) << "-" << path_indices.at(i).at(2) << ".ply" << std::endl;
      indices.push_back(path_indices.at(i));
    }
  }

  return indices;
}




// Store the indices for the paths that were saved, so we
// can look them up when creating new child paths.
void storePathIndices(const unsigned int level,
                     const unsigned int parent_index,
                     const unsigned int index,
                     std::vector<std::vector<unsigned int> >& path_indices)
{
  std::vector<unsigned int> indices = {level, parent_index, index};
  path_indices.push_back(indices);
}


/*
// Check if positions along a path are longer than the length of the path.
void checkPositions(const double path_length,
                    const double start_position,
                    const double obstacle_position)
{
  if (start_position > path_length)
  {
    std::stringstream error;
    error << "start_position " << start_position << " is longer than path_length " << path_length;
    throw std::runtime_error(error.str());
  }

  if (obstacle_position > path_length)
  {
    std::stringstream error;
    error << "obstacle_position " << obstacle_position << " is longer than path_length " << path_length;
    throw std::runtime_error(error.str());
  }
}

void checkPositions(const double path_length,
                    const double end_position)
{
  if (end_position > path_length)
  {
    std::stringstream error;
    error << "end_position " << end_position << " is longer than path_length " << path_length;
    throw std::runtime_error(error.str());
  }
}
*/
const bool checkPositions(const double length_of_path_containing_start_point,
                          const double start_position,
                          const double obstacle_position,
                          const double length_of_path_containing_end_point,
                          const double end_position
                         )
{
  std::cout << "in checkPositions()" << std::endl;

  std::cout << "start_position :" << start_position << std::endl;
  std::cout << "length_of_path_containing_start_point :" << length_of_path_containing_start_point << std::endl;

  std::cout << "obstacle_position :" << obstacle_position << std::endl;
  std::cout << "length_of_path_containing_obstacle_point :" << length_of_path_containing_start_point << std::endl;

  std::cout << "end_position :" << end_position << std::endl;
  std::cout << "length_of_path_containing_end_point :" << length_of_path_containing_end_point << std::endl;



  if (start_position > length_of_path_containing_start_point)
  {
    std::stringstream error;
    error << "start_position " << start_position << " is longer than path length " << length_of_path_containing_start_point;
    //throw std::runtime_error(error.str());
    std::cout << error.str() << std::endl;
    return false;
  }

  if (obstacle_position > length_of_path_containing_start_point)
  {
    std::stringstream error;
    error << "obstacle_position " << obstacle_position << " is longer than path length " << length_of_path_containing_start_point;
    //throw std::runtime_error(error.str());
    std::cout << error.str() << std::endl;
    return false;
  }

  if (end_position > length_of_path_containing_end_point)
  {
    std::stringstream error;
    error << "end_position " << end_position << " is longer than path length " << length_of_path_containing_end_point;
    //throw std::runtime_error(error.str());
    std::cout << error.str() << std::endl;
    return false;
  }

  return true;
}


// Make an alternate path using a random-sampling based planner.
const bool makeShortestAlternatePath(OctreeMap3D& octree_map_3d,
                                     //const Point& start_point,
                                     const unsigned int start_point_index,
                                     const std::vector<Point>& path_containing_start_point,

                                     const double initial_end_position,
                                     const std::vector<Point>& path_containing_end_point,
                                     //const double virtual_obstacle_step_size, // NOT USED
                                     const double length_of_path_containing_end_point,
                                     const bool try_multiple_end_points, // = true;

                                     const bool smooth_ends_of_path,
                                     const float endpoint_smoothing_extend_distance,
                                     const float path_interpolation_step,

                                     std::vector<Point>& path,
                                     double& path_length,
                                     unsigned int& end_point_index
                               )
{
  //const bool try_multiple_end_points = false;
  //const bool try_multiple_end_points = true;
  //const unsigned int max_extra_attempts = 2; // plan upto 5 paths with different end-points

  // not using   virtual_obstacle_step_size
  const double end_point_step_size = 1.10; // make this 10cm


  const Point start_point = path_containing_start_point.at(start_point_index);
  double end_position_along_path = initial_end_position;
  bool found_a_path = false;

  std::vector<Point> loop_path_points;

  double loop_length;
  std::vector<double> loop_lengths; // The length of each alternate path loop
  std::vector<double> total_path_lengths; // The length of the loop + the remaining length of the main path
  unsigned int loop_end_point_index;

  std::vector<Point> shortest_loop_path;
  double shortest_loop_length = std::numeric_limits<double>::max();
  double shortest_loop_total_length = std::numeric_limits<double>::max();
  unsigned int shortest_loop_end_point_index;
  //bool found_shortest_loop = false; // flag, because there may be no valid alternate paths for a virtual obstacle

  // Try generating an alternate route around the obstacle.
  // The start point is fixed, but the end point is
  // in the range {start_position+min_loop_length, parent_path_length}
  bool planning_complete = false;
  unsigned int num_attempts_suceeded = 0;
  unsigned int num_attempts_failed = 0;
  while (!planning_complete)
  {
    if (end_position_along_path > length_of_path_containing_end_point)
    {
      std::cout << "makeShortestAlternatePath(): end_position_along_path > length_of_path_containing_end_point" << std::endl;
      planning_complete = true;
    }
    else if ((try_multiple_end_points == false) && (num_attempts_suceeded >= 1))
    {
      std::cout << "makeShortestAlternatePath(): try_multiple_end_points == false) && (num_attempts_suceeded >= 1)" << std::endl;
      // Found 1 alternate path
      planning_complete = true;
    }
    else if (num_attempts_failed >= 1)
    {
      std::cout << "makeShortestAlternatePath(): num_attempts_failed >= 1" << std::endl;
      // Stop if planning fails on first attempt, or when trying
      // different end-points
      planning_complete = true;
    }
    else
    {
      //std::cout << "  try Loop from " << start_position << " to " << end_position << ", obstacle at " << obstacle_position_along_path << std::endl;

      // Get the point for the end position

      // Get the x,y,z end point and its index,
      // located at some position along the main path:
      Point loop_end_point;
      getPointAtDistanceAlongPath<Point>(path_containing_end_point, end_position_along_path,
                                         loop_end_point, loop_end_point_index);

      bool planning_suceeded = false;

      loop_path_points.clear();

      // Try to plan a path:
      // (this does NOT save the .ply file yet)
      if (smooth_ends_of_path == false)
      {
        planning_suceeded = makePath(octree_map_3d,
                                     start_point,
                                     loop_end_point,
                                     path_interpolation_step,
                                     loop_path_points, // the path that was found
                                     loop_length); // its length
      }
      else
      {
        if ((loop_end_point_index + 1) > (path_containing_end_point.size() - 1))
        {
          throw std::runtime_error("point_after_end_point is beyond the end of path in makeShortestAlternatePath()");
        }

        // The start and end of this path will be a smooth blend with the parent path
        //const Point point_before_start_point = path_containing_start_point.at(start_point_index - 1);
        //const Point point_after_end_point = path_containing_end_point.at(loop_end_point_index + 1);

        //const Point point_before_start_point = path_containing_start_point.at(start_point_index - 1) - path_containing_start_point.at(start_point_index);
        const Point point_before_start_point = Point((path_containing_start_point.at(start_point_index - 1).x - path_containing_start_point.at(start_point_index).x),
                                                     (path_containing_start_point.at(start_point_index - 1).y - path_containing_start_point.at(start_point_index).y),
                                                     (path_containing_start_point.at(start_point_index - 1).z - path_containing_start_point.at(start_point_index).z));

        //const Point point_after_end_point = path_containing_end_point.at(loop_end_point_index + 1) - path_containing_end_point.at(loop_end_point_index);
        const Point point_after_end_point = Point((path_containing_end_point.at(loop_end_point_index + 1).x - path_containing_end_point.at(loop_end_point_index).x),
                                                  (path_containing_end_point.at(loop_end_point_index + 1).y - path_containing_end_point.at(loop_end_point_index).y),
                                                  (path_containing_end_point.at(loop_end_point_index + 1).z - path_containing_end_point.at(loop_end_point_index).z));

        planning_suceeded = makePath(octree_map_3d,
                                     start_point,
                                     loop_end_point,
                                     point_before_start_point,
                                     point_after_end_point,
                                     endpoint_smoothing_extend_distance,
                                     path_interpolation_step,
                                     loop_path_points, // the path that was found
                                     loop_length); // its length
      }


      // Get the distance from where this loop joins its parent path (which is currently hard-coded to the main path)
      // to the goal position (which is the end of the main path).
      //
      // This is the distance from 'loop_end_point' to the goal, which is
      // the same as subtracting result from getPointAtDistanceAlongPath() above from length of main path.
      const double remaining_distance = getDistanceToEndOfPath<Point>(path_containing_end_point, loop_end_point_index);
      const double total_length_with_loop = loop_length + remaining_distance;

      std::cout << "Returned from makePath(), loop_length: " << loop_length << std::endl;
      std::cout << "                   remaining_distance: " << remaining_distance << std::endl;
      std::cout << "                       total distance: " << total_length_with_loop << std::endl;


      if (planning_suceeded)
      {
        num_attempts_suceeded++;
        found_a_path = true;

        // Get the total length of an alternate route, from the global start position
        // to the goal, via this potential loop path:
        // getTotalLengthOfPath() 
        //const double length_to_start_of_loop = start_position;
        //const double length_after_end_of_loop = getDistanceToEndOfPath<Point>(parent_path, end_point_index);
        //const double total_length_with_loop = length_to_start_of_loop + loop_length + length_after_end_of_loop;
        //std::cout << "    distances: " << length_to_start_of_loop << ", " << loop_length << ", " << length_after_end_of_loop;

        // For debug
        loop_lengths.push_back(loop_length);
        total_path_lengths.push_back(total_length_with_loop);

        if (total_length_with_loop < shortest_loop_total_length)
        //if (loop_length < shortest_loop_length)
        {
          // Copy vector of Point
          shortest_loop_path = loop_path_points;

          shortest_loop_length = loop_length; // Length of just the loop (this value is returned and saved to .length file)
          shortest_loop_total_length = total_length_with_loop; // Length of loop + remaining length of main path

          shortest_loop_end_point_index = loop_end_point_index;

          //found_shortest_loop = true;
        }
        else
        {
          // If loops aren't getting shorter, there's no need to keep planning.
          // (A shorter loop is only found in the special case that a plan
          //  results in a loop that curves back towards the start, due to
          //  the location of an obstacle.)
          planning_complete = true;
        }
      

        // Increment the end position
        end_position_along_path += end_point_step_size;

      } // endif planning succeeded
      else
      {
        // Planner failed to find a path
        num_attempts_failed++;
      }
    }
  } // while planning

  // Debug:
  std::cout << "  makeShortestAlternatePath(): " << num_attempts_suceeded << " attempts succeeded \n" << std::endl;
  std::cout << "  Loop lengths:" << std::endl;
  printStdVector(loop_lengths);
  std::cout << "  Total path lengths:" << std::endl;
  printStdVector(total_path_lengths);


  if (found_a_path == true)
  {
    // Found a path.
    // If more than one end-points were tested, the shortest
    // loop is returned.
    path = shortest_loop_path;
    path_length = shortest_loop_length;
    end_point_index = shortest_loop_end_point_index;
    return true;
  }

  return false;
}





const bool makeCurvedPath(OctreeMap3D& octree_map_3d,
                          const Point& p0,
                          const Point& obstacle_point, // We assume this to be mid-point of start and end
                          const Point& p1,
                          const Point& point_before_start_point,
                          const Point& point_after_end_point,
                          const float endpoint_smoothing_extend_distance,
                          const float interpolation_step,

                          const unsigned int angle_type,
                          const float offset, // offset distance of the curve

                          std::vector<Point>& splined_path,
                          double& path_length)
{
  //std::cout << "\n in makeCurvedPath() \n" << std::endl;


  
  const Point linear_midpoint = Point(p0.x + ((p1.x - p0.x) / 2.0),
                               p0.y + ((p1.y - p0.y) / 2.0),
                               p0.z + ((p1.z - p0.z) / 2.0));

  std::cout << "Making curve: " << std::endl;
  std::cout << "  start: " << p0.x << ", " << p0.y << ", " << p0.z << std::endl;
  std::cout << "  end: " << p1.x << ", " << p1.y << ", " << p1.z << std::endl;
  std::cout << "  mid-point: " << linear_midpoint.x << ", " << linear_midpoint.y << ", " << linear_midpoint.z << std::endl;

  const float dx = p1.x - p0.x;
  const float dy = p1.y - p0.y;
  const float dist = std::sqrt(dx*dx + dy*dy);
  const float dx_perp = dx / dist;
  const float dy_perp = dy / dist;

  // Find two offset points on horizontal x,y plane:
  //
  // Change this to (+ - - +) to flip the direction
  const Point offset_point_1 = Point(linear_midpoint.x - offset*dy_perp,
                                     linear_midpoint.y + offset*dx_perp,
                                     linear_midpoint.z);
  const Point offset_point_5 = Point(linear_midpoint.x + offset*dy_perp,
                                     linear_midpoint.y - offset*dx_perp,
                                     linear_midpoint.z);

  // Offset point in vertical direction
  const Point offset_point_3 = Point(obstacle_point.x,
                                     obstacle_point.y,
                                     obstacle_point.z + offset);

  const Point offset_2_dir_midpoint = Point(offset_point_1.x + ((offset_point_3.x - offset_point_1.x) / 2.0),
                                            offset_point_1.y + ((offset_point_3.y - offset_point_1.y) / 2.0),
                                            offset_point_1.z + ((offset_point_3.z - offset_point_1.z) / 2.0));
  const Point offset_4_dir_midpoint = Point(offset_point_5.x + ((offset_point_3.x - offset_point_5.x) / 2.0),
                                            offset_point_5.y + ((offset_point_3.y - offset_point_5.y) / 2.0),
                                            offset_point_5.z + ((offset_point_3.z - offset_point_5.z) / 2.0));
  const float offset_2_dir_midpoint_distance = std::sqrt((offset_2_dir_midpoint.x - obstacle_point.x)*(offset_2_dir_midpoint.x - obstacle_point.x)
                                                       + (offset_2_dir_midpoint.y - obstacle_point.y)*(offset_2_dir_midpoint.y - obstacle_point.y)
                                                       + (offset_2_dir_midpoint.z - obstacle_point.z)*(offset_2_dir_midpoint.z - obstacle_point.z));
  const float offset_4_dir_midpoint_distance = std::sqrt((offset_4_dir_midpoint.x - obstacle_point.x)*(offset_4_dir_midpoint.x - obstacle_point.x)
                                                       + (offset_4_dir_midpoint.y - obstacle_point.y)*(offset_4_dir_midpoint.y - obstacle_point.y)
                                                       + (offset_4_dir_midpoint.z - obstacle_point.z)*(offset_4_dir_midpoint.z - obstacle_point.z));

  // Make unit vectors in desired direction
  const Point offset_2_dir_vector = Point((offset_2_dir_midpoint.x - obstacle_point.x) / offset_2_dir_midpoint_distance,
                                          (offset_2_dir_midpoint.y - obstacle_point.y) / offset_2_dir_midpoint_distance,
                                          (offset_2_dir_midpoint.z - obstacle_point.z) / offset_2_dir_midpoint_distance);
  const Point offset_4_dir_vector = Point((offset_4_dir_midpoint.x - obstacle_point.x) / offset_4_dir_midpoint_distance,
                                          (offset_4_dir_midpoint.y - obstacle_point.y) / offset_4_dir_midpoint_distance,
                                          (offset_4_dir_midpoint.z - obstacle_point.z) / offset_4_dir_midpoint_distance);

  // Create two offset points at 45 degree angles from vertical
  const Point offset_point_2 = Point(linear_midpoint.x + offset*offset_2_dir_vector.x,
                                     linear_midpoint.y + offset*offset_2_dir_vector.y,
                                     linear_midpoint.z + offset*offset_2_dir_vector.z);
  const Point offset_point_4 = Point(linear_midpoint.x + offset*offset_4_dir_vector.x,
                                     linear_midpoint.y + offset*offset_4_dir_vector.y,
                                     linear_midpoint.z + offset*offset_4_dir_vector.z);

  /*
  std::cout << "  offset point 1: " << offset_point_1.x << ", " << offset_point_1.y << ", " << offset_point_1.z << std::endl;
  std::cout << "  offset point 2: " << offset_point_2.x << ", " << offset_point_2.y << ", " << offset_point_2.z << std::endl;
  std::cout << "  offset point 3: " << offset_point_3.x << ", " << offset_point_3.y << ", " << offset_point_3.z << std::endl;
  std::cout << "  offset point 4: " << offset_point_4.x << ", " << offset_point_4.y << ", " << offset_point_4.z << std::endl;
  std::cout << "  offset point 5: " << offset_point_5.x << ", " << offset_point_5.y << ", " << offset_point_5.z << std::endl;
  */

  std::vector<Point> points;
  //points.push_back(point_before_start_point);
  points.push_back(p0);
  if (angle_type == 1)
  {
    points.push_back(offset_point_1);
  }
  else if (angle_type == 2)
  {
    points.push_back(offset_point_2);
  }
  else if (angle_type == 3)
  {
    points.push_back(offset_point_3);
  }
  else if (angle_type == 4)
  {
    points.push_back(offset_point_4);
  }
  else if (angle_type == 5)
  {
    points.push_back(offset_point_5);
  }
  else
  {
    // throw runtime
  }
  points.push_back(p1);
  //points.push_back(point_after_end_point);

  std::shared_ptr<spline_library::Spline<spline_library::Vector3> > spline = getChordalCatmullRomSpline(points,
                                                                                                        point_before_start_point,
                                                                                                        point_after_end_point,
                                                                                                        endpoint_smoothing_extend_distance);
  getPointsFromSpline<Point>(spline, interpolation_step, splined_path, path_length);
  //std::cout << "    Discretized path length: " << path_length << std::endl;

  // Collision-check the interpolated spline path
  bool path_in_collision = octree_map_3d.isPathInCollision(splined_path);
  if (path_in_collision)
  {
    return false;
  }

  return true;
}



// Make an alternate path by fitting a spline to 3 control points,
// where the mid-point is offset from the main path by the robot's radius.
//
// angle_type: 1 = horizontal, left
//             3 = vertical
//             5 = horizontal, right
//
// TODO: Try planning with multiple offset distances or positions,
//       then choose the 'best' loop path.
const bool makeGeometricAlternatePath(OctreeMap3D& octree_map_3d,

                                     const unsigned int start_point_index, // TODO: Why did makeShortest pass in this index?
                                     const std::vector<Point>& path_containing_start_point,

                                     const Point& obstacle_point, // We assume this to be mid-point of start and end

                                     const double initial_end_position, // Is the only end point    // TODO: Why did makeShortest pass in this index?
                                     const std::vector<Point>& path_containing_end_point,

                                     const double length_of_path_containing_end_point,
                                     //const bool try_multiple_end_points, // No

                                     const bool smooth_ends_of_path,
                                     const float endpoint_smoothing_extend_distance,
                                     const float interpolation_step,

                                     const unsigned int angle_type,
                                     const float offset_distance,

                                     std::vector<Point>& path,
                                     double& path_length,
                                     unsigned int& end_point_index
                               )
{


  const Point start_point = path_containing_start_point.at(start_point_index);
  const double end_position_along_path = initial_end_position;
  bool found_a_path = false;

  std::vector<Point> loop_path_points;

  double loop_length;
  //std::vector<double> loop_lengths; // The length of each alternate path loop
  //std::vector<double> total_path_lengths; // The length of the loop + the remaining length of the main path
  unsigned int loop_end_point_index;

  std::vector<Point> best_loop_path;
  double best_loop_length;
  unsigned int best_loop_end_point_index;

  //double shortest_loop_length = std::numeric_limits<double>::max();
  //double shortest_loop_total_length = std::numeric_limits<double>::max();
  //unsigned int shortest_loop_end_point_index;
  //bool found_shortest_loop = false; // flag, because there may be no valid alternate paths for a virtual obstacle

  // Try generating an alternate route around the obstacle.
  // The start point is fixed, but the end point is
  // in the range {start_position+min_loop_length, parent_path_length}
  bool planning_complete = false;
  unsigned int num_attempts_suceeded = 0;
  unsigned int num_attempts_failed = 0;
  while (!planning_complete)
  {
    if (end_position_along_path > length_of_path_containing_end_point)
    {
      std::cout << "makeGeometricAlternatePath(): end_position_along_path > length_of_path_containing_end_point" << std::endl;
      planning_complete = true;
    }
    else if (num_attempts_failed >= 1)
    {
      std::cout << "makeGeometricAlternatePath(): num_attempts_failed >= 1" << std::endl;
      // Stop if planning fails on first attempt, or when trying
      // different end-points
      planning_complete = true;
    }
    else
    {
      //std::cout << "  try Loop from " << start_position << " to " << end_position << ", obstacle at " << obstacle_position_along_path << std::endl;

      // Get the point for the end position

      // Get the x,y,z end point and its index,
      // located at some position along the main path:
      Point loop_end_point;
      getPointAtDistanceAlongPath<Point>(path_containing_end_point, end_position_along_path,
                                         loop_end_point, loop_end_point_index);

      bool planning_suceeded = false;

      loop_path_points.clear();

      // Try to plan a path:
      // (this does NOT save the .ply file yet)
      if (smooth_ends_of_path == false)
      {
        throw std::runtime_error("smooth_ends_of_path == false not implemented in makeGeometricAlternatePath()");
      }
      else
      {
        if ((loop_end_point_index + 1) > (path_containing_end_point.size() - 1))
        {
          throw std::runtime_error("point_after_end_point is beyond the end of path in makeGeometricAlternatePath()");
        }

        // The start and end of this path will be a smooth blend with the parent path
        const Point point_before_start_point = Point((path_containing_start_point.at(start_point_index - 1).x - path_containing_start_point.at(start_point_index).x),
                                                     (path_containing_start_point.at(start_point_index - 1).y - path_containing_start_point.at(start_point_index).y),
                                                     (path_containing_start_point.at(start_point_index - 1).z - path_containing_start_point.at(start_point_index).z));
        const Point point_after_end_point = Point((path_containing_end_point.at(loop_end_point_index + 1).x - path_containing_end_point.at(loop_end_point_index).x),
                                                  (path_containing_end_point.at(loop_end_point_index + 1).y - path_containing_end_point.at(loop_end_point_index).y),
                                                  (path_containing_end_point.at(loop_end_point_index + 1).z - path_containing_end_point.at(loop_end_point_index).z));

        planning_suceeded = makeCurvedPath(octree_map_3d,
                                           start_point,
                                           obstacle_point,
                                           loop_end_point,
                                           point_before_start_point,
                                           point_after_end_point,
                                           endpoint_smoothing_extend_distance,
                                           interpolation_step,

                                           angle_type,
                                           offset_distance,

                                           loop_path_points, // the path that was found
                                           loop_length); // its length
      }

      if (planning_suceeded)
      {
        num_attempts_suceeded++;

        found_a_path = true; // We have a valid path

        // TODO: Try multiple offsets, choose 'best' path
        {
          best_loop_path = loop_path_points;
          best_loop_length = loop_length; // Length of just the loop (this value is returned and saved to .length file)
          best_loop_end_point_index = loop_end_point_index;
        }

        planning_complete = true; // Break out of while loop

        /*
        // For debug
        loop_lengths.push_back(loop_length);
        total_path_lengths.push_back(total_length_with_loop);

        if (total_length_with_loop < shortest_loop_total_length)
        //if (loop_length < shortest_loop_length)
        {
          // Copy vector of Point
          shortest_loop_path = loop_path_points;

          shortest_loop_length = loop_length; // Length of just the loop (this value is returned and saved to .length file)
          shortest_loop_total_length = total_length_with_loop; // Length of loop + remaining length of main path

          shortest_loop_end_point_index = loop_end_point_index;

          //found_shortest_loop = true;
        }
        else
        {
          // If loops aren't getting shorter, there's no need to keep planning.
          // (A shorter loop is only found in the special case that a plan
          //  results in a loop that curves back towards the start, due to
          //  the location of an obstacle.)
          planning_complete = true;
        }
        */

        // Increment the end position
        //end_position_along_path += end_point_step_size;

      } // endif planning succeeded
      else
      {
        // Planner failed to find a path
        num_attempts_failed++;
      }
    }
  } // while planning

  // Debug:
  std::cout << "  makeGeometricAlternatePath(): " << num_attempts_suceeded << " attempts succeeded \n" << std::endl;

  if (found_a_path == true)
  {
    // Found a path
    path = best_loop_path;
    path_length = best_loop_length;
    end_point_index = best_loop_end_point_index;
    return true;
  }

  return false;
}


// Make a circular path starting at (0,0)
//
// #include <algorithm> // std::reverse
template <typename PointT>
const std::vector<PointT> makeCircularPath(const PointT center_point,
                                           const float point_separation,
                                           const bool reverse_order_of_points
                           )
{
  std::vector<Point> path;

  const float radius = std::sqrt(center_point.x*center_point.x + center_point.y*center_point.y);
  const float circumference = 2.0 * M_PI * radius;

  // The number of points to draw on the circumference
  const unsigned int n = static_cast<unsigned int>(std::ceil(circumference / point_separation));

  for (int i = 0; i < n; ++i)
  {
    const PointT point(center_point.x + radius * std::cos(2.0*M_PI / n*i),
                       center_point.y + radius * std::sin(2.0*M_PI / n*i),
                       center_point.z);
    path.push_back(point);
  }

  float smallest_dist = std::numeric_limits<float>::max();
  int closest_point_index = 0;
  for (size_t i = 0; i < path.size(); ++i)
  {
    const float dist = std::fabs(0.0 - path.at(i).x) + std::fabs(0.0 - path.at(i).y);
    if (dist < smallest_dist)
    {
      smallest_dist = dist;
      closest_point_index = i;
    }
  }
  std::cout << "Point closest to zero = " << closest_point_index << std::endl;

  // Re-order the points
  std::vector<Point> reordered_path;
  // Copy points from origin (0,0) to end of file
  // Copy points from start of file to before origin (0,0)





  if (reverse_order_of_points == false)
  {
    for (int i = closest_point_index; i < path.size(); ++i)
    {
      reordered_path.push_back(path.at(i));
    }
    for (int i = 0; i < closest_point_index; ++i)
    {
      reordered_path.push_back(path.at(i));
    }
  }
  else
  {
    for (int i = closest_point_index+1; i < path.size(); ++i)
    {
      reordered_path.push_back(path.at(i));
    }
    for (int i = 0; i < closest_point_index+1; ++i)
    {
      reordered_path.push_back(path.at(i));
    }

    std::reverse(reordered_path.begin(), reordered_path.end());
  }

  if (reordered_path.size() != path.size())
  {
    throw std::runtime_error("reordered_path size != path size");
  }

  return reordered_path;
}




//----------------------------------------------------------------------------//

int main(int argc, char **argv)
{
  //ros::init(argc, argv, "test");
  //ros::NodeHandle nh;


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


  // Generate manual paths
  // (for Schenley Park)

  /*
  {
    // Red, main path
    const double height = 3.0;
    const Point p0(0.0, 0.0, height);
    const Point p1(7.97, 0.64, height);
    const Point p2(27.31, 0.81, height);
    const Point p3(45.21, 2.49, height);
    const Point p4(56.91, 4.60, height);
    const Point p5(71.76, 5.13, height);
    const Point p6(90.0, 10.0, height);
    std::vector<Point> path1;
    path1.push_back(p0);
    path1.push_back(p1);
    path1.push_back(p2);
    path1.push_back(p3);
    path1.push_back(p4);
    path1.push_back(p5);
    path1.push_back(p6);

    std::shared_ptr<spline_library::Spline<spline_library::Vector3> > spline = getCentripetalCatmullRomSpline(path1);
    const float dist_step = 0.01; // interpolation step = 1cm
    std::vector<Point> splined_path;
    double path_length;
    getPointsFromSpline<Point>(spline, dist_step, splined_path, path_length);
    std::cout << "    Discretized path length: " << path_length << std::endl;

    std::string outfile_prefix("/home/dbworth/");
    const bool set_z_values_to_zero = false; // make it a 2D path
    const float speed = 0.5; // extra field, for the mobile robot
    const float wait_time = 0.0; // extra field, for the mobile robot
    savePathFile(outfile_prefix, "0", splined_path, set_z_values_to_zero, speed, wait_time);

    const unsigned int red[3] = {255, 0, 0};
    savePathFile(outfile_prefix, "0_colored", splined_path, red);
  }
  {
    // yellow branch
    const double height = 3.0;
    //const Point p0(34.36, 1.29, height);
    //const Point p1(44.99, -0.96, height);
    //const Point p2(53.64, -5.19, height);

    const Point p0(31.78, 1.09, height);
    //const Point p1(43.20, -0.57, height);
    const Point p1(42.40, -0.57, height);
    const Point p2(60.91, -4.42, height);

    const Point p3(81.58, 7.47, height);

    std::vector<Point> path1;
    path1.push_back(p0);
    path1.push_back(p1);
    path1.push_back(p2);
    path1.push_back(p3);

    std::shared_ptr<spline_library::Spline<spline_library::Vector3> > spline = getCentripetalCatmullRomSpline(path1);
    const float dist_step = 0.01; // interpolation step = 1cm
    std::vector<Point> splined_path;
    double path_length;
    getPointsFromSpline<Point>(spline, dist_step, splined_path, path_length);
    std::cout << "    Discretized path length: " << path_length << std::endl;

    std::string outfile_prefix("/home/dbworth/");
    const bool set_z_values_to_zero = false; // make it a 2D path
    const float speed = 0.5; // extra field, for the mobile robot
    const float wait_time = 0.0; // extra field, for the mobile robot
    savePathFile(outfile_prefix, "1", splined_path, set_z_values_to_zero, speed, wait_time);

    const unsigned int yellow[3] = {232, 232, 0};
    savePathFile(outfile_prefix, "1_colored", splined_path, yellow);
  }
  {
    // green branch
    const double height = 3.0;

    //const Point p0(28.30, 0.86, height);
    const Point p0(26.79, 0.78, height);
    const Point p1(30.5, 2.0, height);

    const Point p2(42.59, 8.0, height);

    const Point p3(53.3, 5.4, height);
    const Point p4(57.77, 4.67, height);

    std::vector<Point> path1;
    path1.push_back(p0);
    path1.push_back(p1);
    path1.push_back(p2);
    path1.push_back(p3);
    path1.push_back(p4);

    std::shared_ptr<spline_library::Spline<spline_library::Vector3> > spline = getCentripetalCatmullRomSpline(path1);
    const float dist_step = 0.01; // interpolation step = 1cm
    std::vector<Point> splined_path;
    double path_length;
    getPointsFromSpline<Point>(spline, dist_step, splined_path, path_length);
    std::cout << "    Discretized path length: " << path_length << std::endl;

    std::string outfile_prefix("/home/dbworth/");
    const bool set_z_values_to_zero = false; // make it a 2D path
    const float speed = 0.5; // extra field, for the mobile robot
    const float wait_time = 0.0; // extra field, for the mobile robot
    savePathFile(outfile_prefix, "2", splined_path, set_z_values_to_zero, speed, wait_time);

    const unsigned int green[3] = {29, 119, 40};
    savePathFile(outfile_prefix, "2_colored", splined_path, green);
  }
  {
    // blue branch
    const double height = 3.0;

    //const Point p0(46.02, 7.71, height);
    const Point p0(44.13, 8.01, height);
    const Point p1(49.12, 7.9, height);

    const Point p2(62.34, 11.10, height);
    const Point p3(72.69, 7.05, height);
    const Point p4(78.45, 6.61, height);

    std::vector<Point> path1;
    path1.push_back(p0);
    path1.push_back(p1);
    path1.push_back(p2);
    path1.push_back(p3);
    path1.push_back(p4);

    std::shared_ptr<spline_library::Spline<spline_library::Vector3> > spline = getCentripetalCatmullRomSpline(path1);
    const float dist_step = 0.01; // interpolation step = 1cm
    std::vector<Point> splined_path;
    double path_length;
    getPointsFromSpline<Point>(spline, dist_step, splined_path, path_length);
    std::cout << "    Discretized path length: " << path_length << std::endl;

    std::string outfile_prefix("/home/dbworth/");
    const bool set_z_values_to_zero = false; // make it a 2D path
    const float speed = 0.5; // extra field, for the mobile robot
    const float wait_time = 0.0; // extra field, for the mobile robot
    savePathFile(outfile_prefix, "3", splined_path, set_z_values_to_zero, speed, wait_time);

    const unsigned int blue[3] = {42, 106, 255};
    savePathFile(outfile_prefix, "3_colored", splined_path, blue);
  }

  exit(1);
  */
  

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


  // Config params:
  //
  // Directory to save paths (.ply) and index (.ind) files
  std::string outfile_prefix("/home/dbworth/");


  // Make sure the output directory does not already contain any .ind files
  // e.g. 0-0-0.ind
  // because this code will append data if a file already exists.
  const std::vector<std::string> existing_index_files = getIndexFiles(outfile_prefix, "[0-9]*", "[0-9]*", "[0-9]*");
  if (existing_index_files.size() > 0)
  {
    printStdVector(existing_index_files);
    std::stringstream error;
    error << "Directory '" << outfile_prefix << "' already contains .ind files, "
          << "you must delete them before running this program." << std::endl;
    throw std::runtime_error(error.str());
  }

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


/*
saveIndexFile("/home/dbworth/",
              1, // level
              0, // path index
              0, 0, 20,
              0, 0, 50
            );

return 0;
*/




  PointCloud::Ptr input_pointcloud(new PointCloud());
  if (!loadPointCloud(pointcloud_file, *input_pointcloud))
  {
    return EXIT_FAILURE;
  }
  std::cout << "Loaded PointCloud with " << input_pointcloud->points.size() << " points" << std::endl;



  // SET THE START & GOAL FOR THE MAIN PATH:

  // pointcloud7_indoor_highbay_no_floor_small.pcd
  //const Point start_point(-2.0, 4.2, 0.2);
  //const Point goal_point(23.9, -1.8, 0.2);

  // #10
  //const Point start_point(0.0, 0.0, -0.7);
  ////const Point goal_point(3.5, -3.5, 0.0);
  //const Point goal_point(18.0, -5.0, -0.7);

  // Schenley Park
  //const Point start_point(0.0, 0.0, -0.18);
  //const Point goal_point(90.0, 10.0, -3.9);

  // pointcloud4-Churchill_Country_Club.ply
  //const Point start_point(0.0, 0.0, 5.0);
  //const Point goal_point(90.0, -25.0, 5.0);

  // None




  // Special options:

  // Use a circle as the main path
  const bool plan_circle = false;

  // Use a path provided by the User as the main path
  //const bool use_provided_main_path = false;
  const bool use_provided_main_path = true;
  //const std::string provided_main_path_file("/home/dbworth/carrie_furnace_short_path_v3.ply");
  const std::string provided_main_path_file("/home/dbworth/carrie_furnace_long_path_v3.ply");
  const Point start_point(0.0, 0.0, 0.0);
  const Point goal_point(0.0, 0.0, 0.0);

  // Carrie, short section
  // robot radius = 4
  // obstacle radius = 4
  // dist to obstacle = 25
  // min loop length = 50



  //const PLANNER_TYPE planner_type = BIT_STAR;
  //const double min_height = -100; // disable min_height constraint
  //const double max_planning_time = 3.0;






  //Timer timer;
  std::cout << "Initializing OctreeMap3D..." << std::endl;
  //timer.reset();


  // Half the diameter of DJI S1000.
  //const double robot_radius = 1.1 / 2.0; // 0.55
  //const double robot_radius = 1.0;
  const double robot_radius = 1.5;
  //const double robot_radius = 2.0;
  //const double robot_radius = 3.5;
  //const double robot_radius = 4.0; // Carrie, short section

  // Note robot radius + obstacle radius MUST BE <  min length parameter 3 meters
  const float obstacle_radius = 4.0;


  const double voxel_size = 0.10;


  const unsigned int num_levels = 1;
  //const unsigned int num_levels = 3;
  //const unsigned int num_levels = 10;

  // Maximum number of alternate paths per level:
  //const unsigned int max_loops = 1;
  //const unsigned int max_loops = 2;
  //const unsigned int max_loops = 10;
  //const unsigned int max_loops = 100;
  const unsigned int max_loops = 30;

  const unsigned int max_attempts = std::numeric_limits<unsigned int>::max();
  //const unsigned int max_attempts = 3;


  //const double robot_to_obstacle_distance = 3.0; // virtual obstacle appears 2m away from robot
  //const double robot_to_obstacle_distance = 4.0;
  //const double robot_to_obstacle_distance = 13.0;
  const double robot_to_obstacle_distance = 25.0;
  //const double robot_to_obstacle_distance = 35.0;

  const double initial_start_position_along_path = 6.0;
  // try 160m to test doorway


  // Try making alternate paths every X meters along the parent path:
  //
  // NOTE: If step size is longer than an alternate loop's length (e.g. 10m), it will never plan an alternate path!
  // need to add check for that.
  //
  //const double virtual_obstacle_step_size = 0.7; // try in steps of 0.5m
  //const double virtual_obstacle_step_size = 1.5;
  //const double virtual_obstacle_step_size = 1.0;
  //const double virtual_obstacle_step_size = 2.0; // was using
  //const double virtual_obstacle_step_size = 4.0;
  //const double virtual_obstacle_step_size = 8.0;
  const double virtual_obstacle_step_size = 10.0;
  
  //const double min_loop_length = 6.0; // start & end-point are 6m apart
  //const double min_loop_length = 10.0;
  //const double min_loop_length = 22.0;
  //const double min_loop_length = 26.0;
  const double min_loop_length = 50.0;
  //const double min_loop_length = 70.0;

  // If true, the planner will make alternate paths of 'min_loop_length' or longer,
  // such that the total distance of the alternate path and the remainder of the main path
  // length is shortest.
  //const bool try_multiple_end_points_for_alternate_paths = true;
  const bool try_multiple_end_points_for_alternate_paths = false;

  // if true, The start and end of this path will be a smooth blend with the parent path
  const bool smooth_path_ends = true;
  //const bool smooth_path_ends = false;

  // If 'smooth_path_ends' is enabled, this is the distance beyond the end-points
  // used to calculate the tangent.
  const float endpoint_smoothing_extend_distance = 0.05; // Smaller value gives smoother curve with geometric alternate paths
  //const float endpoint_smoothing_extend_distance = 0.10;
  //const float endpoint_smoothing_extend_distance = 13.0;
  //const float endpoint_smoothing_extend_distance = 25.0;

  const float path_interpolation_step = 0.10; // 10cm

  // For savePathFile()
  // the 2D robot should have the z values being 0, but should do it in post-processing because the .ply files are read back in during planning
  //const bool set_z_values_to_zero = true;
  const bool set_z_values_to_zero = false;

  // Debug params:

  // Save a .pcd containing only the virtual obstacle cylinders
  const bool save_obstacles_as_pcd = false;
  //const bool save_obstacles_as_pcd = true;





  // TODO: pass obstacle_radius into addCylinder()
  // obstacle_radius
  //
  // Pass  max_planning_time into findPath
  //
  // Add switch: plan in 2D (so BIT* uses x,y)
  // Option to squish pointcloud to 2D


  // Also change  min_height  value


  // Store the indices of all the .ply files that are saved
  // so we can look-up the parent of a path.
  // e.g. store (0,0,0) for 0-0-0.ply
  //      store (1,0,0) for 1-0-0.ply
  //      store (1,0,1) for 1-0-1.ply
  std::vector<std::vector<unsigned int> > path_indices;




  std::vector<Point> main_path;
  double main_path_length;

  if (plan_circle)
  {
    // Make a circular path starting at (0,0), 
    // with points 2cm apart.
    const Point center_point(0.5, -2.4, 0.0);
    const bool reverse_order_of_points = true;
    main_path = makeCircularPath<Point>(center_point, 0.01, reverse_order_of_points); // 2, 4, 6

    const float radius = std::sqrt(center_point.x*center_point.x + center_point.y*center_point.y);
    const float circumference = 2.0 * M_PI * radius;
    main_path_length = static_cast<double>(circumference);
  }

  else if (use_provided_main_path)
  {
    main_path = getPathFromPLYFile<Point>(provided_main_path_file);
    main_path_length = getPathLength<Point>(main_path);
    std::cout << "Loaded user's main path file " << provided_main_path_file << std::endl;
    std::cout << "with " << main_path.size() << " points, distance = " << main_path_length << std::endl;
  }

  else
  {
    // Plan the main path:
    OctreeMap3D main_map(input_pointcloud, voxel_size, robot_radius);
    std::cout << "Finding main path from start to goal..." << std::endl;
    //std::vector<Point> main_path;
    //double main_path_length;
    const bool planning_suceeded = makePath(main_map, start_point, goal_point, path_interpolation_step, main_path, main_path_length);
    if (!planning_suceeded)
    {
      throw std::runtime_error("Failed to plan main path");
    }
  }






  storePathIndices(0, 0, 0, path_indices);


  // Save main path as file "0-0-0.ply"
  //const bool set_z_values_to_zero = true; // make it a 2D path
  const float speed = 0.5; // extra field, for the mobile robot
  const float wait_time = 0.0; // extra field, for the mobile robot
  savePathToFile(outfile_prefix, 0, 0, 0, main_path, set_z_values_to_zero, speed, wait_time);

  // Debug:
  // Save red-colored version of main path as file "0-0-0_colored.ply", to view in CloudCompare
  const std::string filename_suffix("_colored");
  savePathToFile(outfile_prefix, 0, 0, 0, main_path, filename_suffix);


  // Save length of main path to file "0-0-0.length"
  saveSplineLengthFile(outfile_prefix, 0, 0, 0, main_path_length);
  //const std::vector<Point> main_path = loadPathFile<Point>(outfile_prefix, 0, 0, 0);
  //const double main_path_length = loadSplineLengthFile(outfile_prefix, 0, 0, 0);
  std::cout << "  main_path_length = " << main_path_length << std::endl;







  
  for (unsigned int level = 1; level <= num_levels; ++level)
  {
    const unsigned int parent_level = level - 1;

    // Get the indices of higher level paths
    // e.g.
    // for level 1, higher level indices = 0-0-0
    // for level 2, higher level indices = 1-0-0
    //                                     1-0-1
    //                                     1-0-2 etc.
    const std::vector<std::vector<unsigned int> > parent_path_indices = getLevelPathIndices(parent_level, path_indices);

    //unsigned int path_index = 0;
    unsigned int current_path_index = 0;

    for (size_t i = 0; i < parent_path_indices.size(); ++i)
    {
    //int i = 0;

      const unsigned int parent_path_level        = parent_path_indices.at(i).at(0);
      const unsigned int parent_path_parent_index = parent_path_indices.at(i).at(1);
      const unsigned int parent_path_index        = parent_path_indices.at(i).at(2);
      std::cout << std::endl;
      std::cout << "  ******************************" << std::endl;
      std::cout << "  Planning loop for parent " << parent_path_level << "-" << parent_path_parent_index << "-" << parent_path_index << std::endl;
      std::cout << "  ******************************" << std::endl;
      std::cout << std::endl;






// todo: obstacle point is 0

        // ToDo: Should get the level & index of the start and index points,
        // and use that to dynamically load those paths.

/*
            unsigned int loop_start_level = parent_path_level;
            unsigned int loop_start_file_index = parent_path_parent_index;
            unsigned int loop_start_point_index = start_point_index; // index of point on the parent path
            // Assuming all paths end on the main path (level 0)
            unsigned int loop_end_level = 0;
            unsigned int loop_end_file_index = 0;
            unsigned int loop_end_point_index = end_point_index; // shortest_loop_end_point_index

*/


      // Load the parent path
      // e.g. 1-0-0.ply
      // This is where the start point is located.
      // Note: the end point is located on the main path.
      const std::vector<Point> parent_path = loadPathFile<Point>(outfile_prefix,
                                                                 parent_path_level, parent_path_parent_index, parent_path_index);
      if (parent_path.size() == 0)
      {
        throw std::runtime_error("parent_path has 0 points!");
      }

      //loadIndexFile(parent_path_indices.at(i).at(0), parent_path_indices.at(i).at(1), parent_path_indices.at(i).at(2))
      //getEndPositionOfPath(parent_path_indices.at(i).at(0), parent_path_indices.at(i).at(1), parent_path_indices.at(i).at(2));

      // Load the parent path:
      //const std::vector<Point> parent_path = getPathFromPLYFile<Point>("/home/dbworth/0-0-0.ply");
      //const std::vector<Point> parent_path = loadPathFile<Point>(outfile_prefix, 0,0,0);

      //const double parent_spline_length = loadSplineLengthFile(outfile_prefix, 0,0,0); // path_level, path_parent_index, path_index, spline_length);
      const double parent_path_length = loadSplineLengthFile(outfile_prefix,
                                                             parent_path_level, parent_path_parent_index, parent_path_index);
      //std::cout << "Level: " << level << "  parent_spline_length = " << parent_spline_length << std::endl;
      std::cout << "  parent_path_length = " << parent_path_length << std::endl;



      // For each parent path, plan one or more loops
      // by moving the start position and obstacle
      // along the length of the parent path:

      // Start and end positions are defined by their distance along the path
      double start_position_along_path = initial_start_position_along_path; // start a loop at X meters along the spline

      unsigned int num_loops_attempted = 0;
      unsigned int num_loops_planned = 0;
      bool planning_loops = true;
      while ((planning_loops) && (num_loops_planned < max_loops) && (num_loops_attempted < max_attempts)) // TODO: This is max_attempts per level, not total
      {
        //start_position = virtual_obstacle_step_size; // start a loop at 0.5m along the spline
        //end_position = start_position + min_loop_length; // end of loop is at least 6.5m along spline





        // TEMP
        // TRY ONLY ONE END POINT


        // TODO: fix end position

        // Start and end positions are defined by their distance along the path
        //double start_position_along_path = virtual_obstacle_step_size; // start a loop at 0.5m along the spline

        double obstacle_position_along_path = start_position_along_path + robot_to_obstacle_distance;

        //double end_position_along_path = start_position_along_path + min_loop_length; // end of loop is at least 6.5m along spline
        double end_position_along_path; // = start_position_along_path + min_loop_length; // end of loop is at least 6.5m along spline


        // getParentEndPositionAndObstaclePoints

        // if 0-0-0, then return defaults
        // if 1-0-0 


        unsigned int parent_path_end_point_index;
        std::vector<Point> parent_obstacle_points;


        std::cout << std::endl;
        std::cout << "  ******************************" << std::endl;
        std::cout << "  Planning for level " << level << "  current_path_index " << current_path_index << std::endl;
        std::cout << "  ******************************" << std::endl;
        std::cout << std::endl;


        if (level == 1)
        {
          end_position_along_path = start_position_along_path + min_loop_length; // end of loop is at least X meters along spline
        }
        else
        {
          // Get the end position of the parent path, which is the distance
          // along the main path.
          // Also get the obstacle point of the parent path, and
          // the parent's parent path, and so on...
          //double parent_end_position;
          //std::vector<Point> parent_obstacle_points;
          //getParentEndPositionAndObstaclePoints(
          getParentEndPointAndObstaclePoints(outfile_prefix,
                                                parent_path_level, parent_path_parent_index, parent_path_index,
                                                //parent_end_position,
                                                parent_path_end_point_index,
                                                parent_obstacle_points);

          // currently all paths end on the main path
          end_position_along_path = getDistanceFromStartOfPath<Point>(main_path, parent_path_end_point_index);
          const double add_to_end = 0.10;
          end_position_along_path += add_to_end;
        }

        /*
        if (end_position_along_path > main_path_length) // end point is always on main path
        {
          std::cout << "  end_position_along_path " << end_position_along_path << " > parent_path_length " << parent_path_length << std::endl;
          std::cout << "  No more room" << std::endl;

          // There's not enough length on the parent path
          // to fit any more alternate route loops.
          planning_loops = false;
          break;
        }
        */

        //bool found_path;


        // Check the positions are not longer than the path
        if (!checkPositions(parent_path_length, start_position_along_path, obstacle_position_along_path,
                            main_path_length, end_position_along_path))
        {
          // There's not enough length on the parent path
          // to fit any more alternate route loops.
          planning_loops = false;

          std::cout << "\n *** WARNING: Not enough room on parent parent." << std::endl;
          std::cout << "main_path_length:" << main_path_length << std::endl;
          std::cout << "parent_path_length:" << parent_path_length << std::endl;
          std::cout << "start_position_along_path:" << start_position_along_path << std::endl;
          std::cout << "obstacle_position_along_path:" << obstacle_position_along_path << std::endl;
          std::cout << "end_position_along_path:" << end_position_along_path << std::endl;

          break;
        }




        // Get the x,y,z point for the start and obstacle
        // located at some position along the parent path:
        std::cout << "About to get start point at distance " << start_position_along_path << std::endl;
        Point start_point;
        unsigned int start_point_index;
        getPointAtDistanceAlongPath<Point>(parent_path, start_position_along_path, start_point, start_point_index);
        std::cout << "start point: " << start_point.x << ", " << start_point.y << "   index: " << start_point_index << std::endl;

        std::cout << "About to get obstacle point at distance " << obstacle_position_along_path << std::endl;
        Point obstacle_point;
        unsigned int obstacle_point_index;
        getPointAtDistanceAlongPath<Point>(parent_path, obstacle_position_along_path, obstacle_point, obstacle_point_index);
        std::cout << "obstacle point: " << obstacle_point.x << ", " << obstacle_point.y << "   index: " << obstacle_point_index << std::endl;

        // Similarly, the x,y,z point for the end is
        // located at some position along the main path:
        //Point end_point;
        //unsigned int end_point_index;
        //getPointAtDistanceAlongPath<Point>(main_path, end_position_along_path, end_point, end_point_index);



        // MOVE THESE ABOE

        const bool use_geometric_alternate_paths = true;
        //const  = 3;
        const float geometric_offset_distance = 8.0;


        std::vector<Point> shortest_alternate_path;
        double alternate_path_length;
        unsigned int end_point_index;
        bool found_path = false;

        const std::vector<unsigned int> angle_types = {1, 5};
        //const std::vector<unsigned int> angle_types = {1, 2, 3, 4, 5};


        //for (unsigned int angle_type = 1; angle_type <= 5; ++angle_type)
        //for (unsigned int angle_type = 1; angle_type <= 1; ++angle_type)
        //for (unsigned int angle_type = 3; angle_type <= 3; ++angle_type)
        for (size_t ati = 0; ati < angle_types.size(); ++ati)
        {
          const unsigned int angle_type = angle_types.at(ati);


          if (use_geometric_alternate_paths)
          {
            // Generate alternate path using a spline

            //PointCloud::Ptr cloud_with_obstacles(new PointCloud(*input_pointcloud));

            // TODO: initialize this only once?
            OctreeMap3D map(input_pointcloud, voxel_size, robot_radius);

            found_path = makeGeometricAlternatePath(map,
                                                    start_point_index, parent_path,

                                                    obstacle_point, // mid-point position along path

                                                    end_position_along_path, main_path, // TODO: Don't need end position for this method

                                                    main_path_length, // length_of_path_containing_end_point

                                                    smooth_path_ends,
                                                    endpoint_smoothing_extend_distance,
                                                    path_interpolation_step,

                                                    angle_type, // horizontal or vertical loop
                                                    geometric_offset_distance,

                                                    shortest_alternate_path,
                                                    alternate_path_length,
                                                    end_point_index
                                                     );


          }

          // TODO: HOW TO SWITCH THIS ON AND OFF WITH THE FOR LOOP ABOVE  ??
          else
          {
            // Generate alternate path using RRT and virtual obstacles


            // Add virtual obstacles (includes those on parent paths)
            const std::vector<float> bounds = getPointCloudBounds<Point>(input_pointcloud);
            std::cout << "in addVirtualObstacles(), PointCloud bounds: " << std::endl;
            printStdVector(bounds);
            const float obstacle_min_z = bounds.at(4);
            const float obstacle_max_z = bounds.at(5);
            PointCloud::Ptr cloud_with_obstacles = addVirtualObstacles<Point>(input_pointcloud,
                                                                              level,
                                                                              parent_obstacle_points, // upper-level obstacles
                                                                              obstacle_point, // new obstacle
                                                                              VERTICAL_CYLINDER,
                                                                              obstacle_radius,
                                                                              obstacle_min_z,
                                                                              obstacle_max_z);
            //PointCloud::Ptr cloud_with_obstacles(new PointCloud());

            // Debug
            // Create a PointCloud containing only the obstacles and save it as .pcd file
            if (save_obstacles_as_pcd)
            {
              PointCloud::Ptr empty_pointcloud(new PointCloud());
              PointCloud::Ptr obstacles_cloud = addVirtualObstacles<Point>(empty_pointcloud,
                                                                           level,
                                                                           parent_obstacle_points,
                                                                           obstacle_point,
                                                                           VERTICAL_CYLINDER,
                                                                           obstacle_radius,
                                                                           obstacle_min_z,
                                                                           obstacle_max_z);
              savePointCloudFile(outfile_prefix,
                                 level, parent_path_index, current_path_index,
                                 *obstacles_cloud);
            }

            //std::cout << "Initializing OctreeMap3D..." << std::endl;
            OctreeMap3D map(cloud_with_obstacles, voxel_size, robot_radius);

            //std::vector<Point> shortest_alternate_path;
            //double alternate_path_length;
            //unsigned int end_point_index;
            //bool found_path = false;
            try
            {
              found_path = makeShortestAlternatePath(map,
                                                     start_point_index, parent_path,

                                                     end_position_along_path, main_path,
                                                     //virtual_obstacle_step_size, // not used
                                                     main_path_length,
                                                     try_multiple_end_points_for_alternate_paths, // = true;

                                                     smooth_path_ends,
                                                     endpoint_smoothing_extend_distance,
                                                     path_interpolation_step,

                                                     shortest_alternate_path,
                                                     alternate_path_length,
                                                     end_point_index
                                                     );
            }
            catch (std::exception& ex)
            {
              // Catch exceptions inside Planner
              // e.g. if the start point is in collision
              std::cout << "  Error planning for level " << level << ", current_path_index " << current_path_index << std::endl;
              std::cout << "  Exception:" << ex.what() << std::endl;

              /*
              // For debug:
              // Save the PointCloud with virtual obstacles
              std::stringstream ss;
              ss << "cloud_with_obstacles-level" << level << "-index" << current_path_index << ".pcd";
              std::string outfile = outfile_prefix + ss.str();
              std::cout << "  Saving " << outfile << std::endl;
              pcl::io::savePCDFileASCII(outfile, *cloud_with_obstacles);

              // For debug:
              // Save a .pcd file containing a circle of the robot's radius
              // around the start point of the alternate path
              PointCloud::Ptr start_point_radius_cloud(new PointCloud());
              addCircleToPointCloud<Point>(start_point, robot_radius, 0.01, start_point_radius_cloud);
              std::stringstream ss2;
              ss2 << "start_point_radius_cloud-level" << level << "-index" << current_path_index << ".pcd";
              outfile = outfile_prefix + ss2.str();
              std::cout << "  Saving " << outfile << std::endl;
              pcl::io::savePCDFileASCII(outfile, *start_point_radius_cloud);
              */

              //// Don't plan any more paths:
              //planning_loops = false;
              //break;
            }
          }





          num_loops_attempted++; 

          if (found_path)
          {


            // Debug
            std::cout << "Returned from makeShortestAlternatePath()" << std::endl;
            std::cout << "    alternate_path_length = " << alternate_path_length << std::endl;
            const double d = getDistanceToEndOfPath<Point>(shortest_alternate_path, 0);
            std::cout << "    Actual length of 'shortest_alternate_path' = " << d << std::endl;
            // THROW EXCEPTION BELOW, after saving file



            std::cout << "result = " << found_path << std::endl;
            // Found a path:


            storePathIndices(level,
                             parent_path_index, current_path_index,
                             path_indices);

            // Save path as .ply file
            // e.g. "1-0-0.ply"
            //const bool set_z_values_to_zero = true; // make it a 2D path
            const float speed = 0.5; // extra field, for the mobile robot
            const float wait_time = 0.0; // extra field, for the mobile robot
            savePathToFile(outfile_prefix,
                           level, parent_path_index, current_path_index,
                           shortest_alternate_path,
                           set_z_values_to_zero, speed, wait_time);

            // Debug:
            // Save a red-colored .ply file, to view in CloudCompare
            // e.g. "1-0-0_colored.ply"
            const std::string filename_suffix("_colored");
            savePathToFile(outfile_prefix,
                           level, parent_path_index, current_path_index,
                           shortest_alternate_path,
                           filename_suffix);

            // saveIndexFile

            unsigned int index_file_level = parent_path_level;
            unsigned int index_file_parent_index = parent_path_parent_index;
            unsigned int index_file_index = parent_path_index;

            unsigned int loop_start_level = parent_path_level;
            //unsigned int loop_start_file_index = parent_path_parent_index;
            unsigned int loop_start_file_index = parent_path_index;
            unsigned int loop_start_point_index = start_point_index; // index of point on the parent path
            // Assuming all paths end on the main path (level 0)
            unsigned int loop_end_level = 0;
            unsigned int loop_end_file_index = 0;
            unsigned int loop_end_point_index = end_point_index; // shortest_loop_end_point_index

            saveIndexFile(outfile_prefix,
                          index_file_level,
                          index_file_parent_index,
                          index_file_index,
                          level,

                          loop_start_level, loop_start_file_index, loop_start_point_index,

                          loop_end_level, loop_end_file_index, loop_end_point_index,
                          current_path_index,

                          obstacle_point_index
                         );

            // Save length of spline to file
            // e.g. "0-0-0.length"
            saveSplineLengthFile(outfile_prefix,
                                 //level, parent_path_parent_index, current_path_index,
                                 level, parent_path_index, current_path_index,
                                 alternate_path_length); // shortest_loop_length

            // Paths on each level have a unique index number, starting from zero
            current_path_index++;

            num_loops_planned++;

            if (std::fabs(d - alternate_path_length) > 0.1)
            {
              throw std::runtime_error("LENGTHS DONT MATCH");
            }

          } // endif found a path


        } // end foreach angle_type






        // Try a virtual obstacle in a new position:
        start_position_along_path += virtual_obstacle_step_size;

        //num_loops_planned++; // Moved to 'if plan succeeded'

      } // endwhile planning_loops

      //std::cout << "\nDone, planned " << num_loops_planned << " for parent path" << std::endl;
      std::cout << "\nDone, attempted to plan " << num_loops_attempted << " loops," << std::endl;
      std::cout << "         " << num_loops_planned << " plans were successful." << std::endl;

        //while ((planning_loops) && (num_loops_planned < max_loops
      

      //unsigned int path_index = 0;

      /*
      bool level_complete = false;
      while (!level_complete) // Should be Planning loop for parent path
      {
        //start_position = virtual_obstacle_step_size; // start a loop at 0.5m along the spline
        //end_position = start_position + min_loop_length; // end of loop is at least 6.5m along spline

        if (end_position > parent_spline_length)
        {
          level_complete = true;
        }
        else
        {
          //const Point obstacle_position1 = getPointAtDistanceAlongSpline(spline, obstacle_position_along_path);

          // Get the points for the start and obstacle, which are
          // located at some position along the discretized path:
          Point start_point;
          unsigned int start_point_index;
          getPointAtDistanceAlongPath<Point>(parent_path, start_position, start_point, start_point_index);
          //const double obstacle_position_along_path = start_position + robot_to_obstacle_distance;
          obstacle_position_along_path = start_position + robot_to_obstacle_distance;
          Point obstacle_point;
          unsigned int obstacle_point_index;
          getPointAtDistanceAlongPath<Point>(parent_path, obstacle_position_along_path, obstacle_point, obstacle_point_index);

          //std::cout << "  obstacle_position on Spline: " << obstacle_position1 << "   on path: " << obstacle_position << std::endl;
          //std::cout << "  point at pos_idx = " << parent_path.at(obstacle_position_index) << std::endl;
          
          // Add virtual obstacles (includes those on parent paths)
          PointCloud::Ptr cloud_with_obstacles = addVirtualObstacles<Point>(input_pointcloud, level, obstacle_point);

          //std::cout << "Initializing OctreeMap3D..." << std::endl;
          OctreeMap3D map(cloud_with_obstacles, voxel_size, robot_radius);

          // For debug
          std::vector<double> loop_lengths;

          std::vector<Point> shortest_loop_path;
          double shortest_loop_length = 0; // = std::numeric_limits<double>::max();
          double shortest_loop_total_length = std::numeric_limits<double>::max();
          unsigned int shortest_loop_end_point_index;
          bool found_shortest_loop = false; // flag, because there may be no valid alternate paths for a virtual obstacle

          


          // Try generating an alternate route around the obstacle.
          // The start point is fixed, but the end point is
          // in the range {+6m, parent_path_length}

          bool loop_complete = false;
          while (!loop_complete)
          {
            if (end_position > parent_spline_length)
            {
              loop_complete = true;
            }
            else
            {
              
              std::cout << "  try Loop from " << start_position << " to " << end_position << ", obstacle at " << obstacle_position_along_path << std::endl;

              // Get the point for the end position
              Point end_point;
              unsigned int end_point_index;
              getPointAtDistanceAlongPath<Point>(parent_path, end_position, end_point, end_point_index);

              std::vector<Point> loop_path_points;
              double loop_length;
              //makePath(path_level, path_parent_index, path_index, octree_map_3d, p0, p1, splined_path, path_length);
              bool res = makePath(0, 0, 1, map, start_point, end_point, loop_path_points, loop_length);

              if (res)
              {
                // Found a path:

                 // Get the total length of an alternate route, from the global start position
                // to the goal, via this potential loop path:
                // getTotalLengthOfPath() 
                const double length_to_start_of_loop = start_position;
                const double length_after_end_of_loop = getDistanceToEndOfPath<Point>(parent_path, end_point_index);
                const double total_length_with_loop = length_to_start_of_loop + loop_length + length_after_end_of_loop;
                std::cout << "    distances: " << length_to_start_of_loop << ", " << loop_length << ", " << length_after_end_of_loop;

                // Debug
                loop_lengths.push_back(total_length_with_loop);

                if (total_length_with_loop < shortest_loop_total_length)
                {
                  shortest_loop_path = loop_path_points;
                  shortest_loop_length = loop_length; // length of just the loop
                  shortest_loop_total_length = total_length_with_loop; // length of parent segments AND loop
                  shortest_loop_end_point_index = end_point_index;
                  found_shortest_loop = true;
                }
              }

              // Increment the end position
              end_position += virtual_obstacle_step_size;
            }
          }

          //std::cout << "  try Loop from " << start_position << " to " << end_position << std::endl;

          std::cout << "  Loop lengths:" << std::endl;
          printStdVector(loop_lengths);
          std::cout << "" << std::endl;

          if (found_shortest_loop)
          {
            //unsigned int path_level = level;
            unsigned int parent_level = level - 1;
            unsigned int path_parent_index = 0; //getParentPathIndex()

            
            // Save path as .ply file
            // e.g. "0-0-0.ply"
            //savePathFile(outfile_prefix, path_level, path_parent_index, path_index, path_pointcloud);
            savePathFile(outfile_prefix, level, path_parent_index, current_path_index, shortest_loop_path);
            storePathIndices(level, path_parent_index, current_path_index, path_indices);

            // saveIndexFile

            unsigned int loop_start_level = parent_level;
            unsigned int loop_start_file_index = path_parent_index;
            unsigned int loop_start_point_index = start_point_index;

            unsigned int loop_end_level = 0;
            unsigned int loop_end_file_index = 0;
            unsigned int loop_end_point_index = shortest_loop_end_point_index;

            saveIndexFile(outfile_prefix,
                          level,
                          0, // ?
                          current_path_index,
                          loop_start_level, loop_start_file_index, loop_start_point_index,
                          loop_end_level, loop_end_file_index, loop_end_point_index
              );

            // Save length of spline to file
            // e.g. "0-0-0.length"
            saveSplineLengthFile(outfile_prefix, level, path_parent_index, current_path_index, shortest_loop_length);

            // Paths on each level have a unique index number, starting from zero
            current_path_index++;

          } // planner found a valid loop


          // Try a virtual obstacle in a new position:
          start_position += virtual_obstacle_step_size;
          // Reset end position:
          end_position = start_position + min_loop_length;
        }

      } // end while level complete
      */


    } // foreach parent path

    //} // endif level 2



  } // foreach level



  // Remove obstacle point indices from .ind files, because that information was
  // only required during planning and is not part of the standard file format.
  cleanIndexFiles(outfile_prefix);

  // Debug:
  // Only keep red colored .ply files
  // e.g. 0-0-0_colored.ply
  //deleteIndexFiles(outfile_prefix);
  //deleteNonColoredPathFiles(outfile_prefix);
  deleteLengthFiles(outfile_prefix);



/*
printStdVector(d_values);
uint j = 0;
for (size_t i = 1; i < d_values.size()-1; ++i)
//for (size_t i = 1; i < 4; ++i)
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
*/



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

  return EXIT_SUCCESS;
}
