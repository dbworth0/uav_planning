/*
Example 18:

Based on 17, but adds ability to plan using waypoints, (actually altitude segments)
All arguments are hard-coded.








Usage:
export PATH=$PATH:~/path/to/your/catkin_workspace/devel/lib/planning_through_pointclouds/
cd ~/path/to/your/catkin_workspace/planning_through_pointclouds/data/

ex18_plan_through_pointcloud_ompl_octree_dtm_waypoints


TODO:
 - add takeoff
  - do a final collision-check.



David Butterworth
*/

#include <iostream> // cout
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp> // to_upper

#include <planning_through_pointclouds/utils.h> // processCommandLine, Timer, lerp
#include <planning_through_pointclouds/pcl_utils.h> // loadPointCloud, getPathPointCloudFromPoints, addTakeoffAndLanding, addVerticalFunnelToPointCloud
#include <planning_through_pointclouds/spline_utils.h> // getCentripetalCatmullRomSpline, getPointsFromSpline
#include <planning_through_pointclouds/octree_map_3d_v5_ompl.h>

#include <spline_library/vector.h>
#include <spline_library/hermite/cubic/cubic_hermite_spline.h>

#include <limits>

#include <cstdlib> // getenv

//----------------------------------------------------------------------------//

// this calculates the required z value of each waypoint, to match the altitude

// Define a path segment with altitude constraints
/*
struct Segment
{
  Point start;
  Point end;
  double min_altitude;
  double max_altitude;
  Segment()
   :  {};
};
*/

struct Waypoint2
{
  Point point;
  double min_altitude;
  double max_altitude;
  Waypoint2(const Point& _point, const double _min_altitude, const double _max_altitude)
   : point(_point)
   , min_altitude(_min_altitude)
   , max_altitude(_max_altitude)
   {};
};

struct Waypoint
{
  Point point;
  double altitude;
  double collision_radius;
  Waypoint(const Point& _point, const double _altitude, const double _collision_radius)
   : point(_point)
   , altitude(_altitude)
   , collision_radius(_collision_radius)
   {};
};


int main(int argc, char **argv)
{
  // Hack-ish way to get paths
  const std::string path_to_data = boost::filesystem::system_complete(argv[0]).remove_filename().string() + std::string("/../data/");
  const std::string path_to_home = std::string(std::getenv("HOME")) +  std::string("/");
  std::cout << "path_to_data: " << path_to_data << std::endl;
  std::cout << "path_to_home: " << path_to_home << std::endl;

  // For storing Planner parameters
  std::string pointcloud_file; // = path_to_data + "pointcloud10_carrie_furnace_2_main.ply";
  Point start_point;
  Point goal_point;
  //double user_specified_robot_radius; // = 4.0;
  double min_height;
  double max_height;
  bool use_height_constraints = false; // = false;
  std::vector<float> takeoff_funnel;
  std::vector<float> landing_funnel;
  bool use_funnel_constraints = false;
  std::string elevation_map_pointcloud_file; // = path_to_data + "pointcloud10_carrie_furnace_2_elevation_100ppm.ply";
  double start_min_altitude;
  double start_max_altitude;
  double goal_min_altitude;
  double goal_max_altitude;
  bool use_altitude_constraint = false; // = false;
  bool use_fixed_altitude_at_endpoints = false; // = false;
  double endpoints_z_offset;
  bool use_offset_endpoints_z_value = false; // = false;
  double path_interpolation_step; // = 0.10; // 10cm
  std::string planner; // = "BIT_STAR";
  double max_planning_time; // = 5.0;
  std::string output_filename; // = path_to_home + "path_out_18_1.ply";


  // -----

  /*
  // Carrie Furnace, map #2
  pointcloud_file = path_to_data + "pointcloud10_carrie_furnace_2_main.ply";
  elevation_map_pointcloud_file = path_to_data + "pointcloud10_carrie_furnace_2_elevation_100ppm.ply";
  //user_specified_robot_radius = 4.0; // The collision-checking radius
  //endpoints_z_offset = 4.5; // Offset, because the end-points are closer to the ground than the collision-checking radius
  //use_offset_endpoints_z_value = true; // Don't need when using fixed altitude
  path_interpolation_step = 0.10; // 10cm
  planner = "BIT_STAR";
  max_planning_time = 15.0;
  output_filename = path_to_home + "path_out_18_1.ply";

  use_altitude_constraint = true;
  use_fixed_altitude_at_endpoints = true; // Doesn't work

  std::vector<Waypoint> waypoints;

  waypoints.push_back(Waypoint(Point(  0.00, 0.00, -0.99), 2.0, 1.7)); // Start
  waypoints.push_back(Waypoint(Point( 69.11, 3.72, -1.65), 6.0, 4.0)); // Increase altitude after car obstacle
  waypoints.push_back(Waypoint(Point(141.48, 2.40, -0.08), 6.0, 4.0)); // End of straight
  waypoints.push_back(Waypoint(Point(148.59, 16.62, 0.38), 6.0, 4.0)); // Turn left
  waypoints.push_back(Waypoint(Point(143.22, 35.64, 1.5), 5.5, 4.0));
  //waypoints.push_back(Waypoint(Point(115.16, 75.57, 0.63), 2.0, 1.7)); // Lower altitude, before doorway
  waypoints.push_back(Waypoint(Point(115.16, 75.57, 0.63), 2.4, 1.7));
  waypoints.push_back(Waypoint(Point(104.74, 98.31, 0.80), 2.0, 1.7)); // inside doorway
  //waypoints.push_back(Waypoint(Point(100.01, 107.61, 2.71), 2.0, 1.5)); // Far side of doorway
  //waypoints.push_back(Waypoint(Point(100.78, 107.66, 0.83), 2.0, 1.5));
  //waypoints.push_back(Waypoint(Point(100.30, 107.62, 0.83), 2.0, 1.5));
  waypoints.push_back(Waypoint(Point(100.50, 107.62, 0.83), 2.0, 1.5));
  waypoints.push_back(Waypoint(Point(96.76, 109.98, 0.91), 2.0, 1.5));
  waypoints.push_back(Waypoint(Point(77.32, 110.91, 0.61), 3.6, 2.0));
  //waypoints.push_back(Waypoint(Point(72.25, 114.82, 0.66), 4.3, 2.5)); // Increase altitude
  waypoints.push_back(Waypoint(Point(70.87, 115.58, 0.62), 4.0, 2.0));
  waypoints.push_back(Waypoint(Point(65.88, 122.61, 0.87), 4.3, 2.0));
  waypoints.push_back(Waypoint(Point(45.57, 178.45, 1.88), 5.0, 2.5)); // Goal
  */

  // Long Path v1
  /*
  waypoints.push_back(Waypoint(Point(  0.00, 0.00, -0.99), 2.0, 1.7)); // Start
  waypoints.push_back(Waypoint(Point( 69.11, 3.72, -1.65), 6.0, 4.0)); // Increase altitude after car obstacle
  waypoints.push_back(Waypoint(Point(141.48, 2.40, -0.08), 6.0, 4.0)); // End of straight
  waypoints.push_back(Waypoint(Point(148.59, 16.62, 0.38), 6.0, 4.0)); // Turn left
  waypoints.push_back(Waypoint(Point(143.22, 35.64, 1.5), 6.0, 4.0));
  waypoints.push_back(Waypoint(Point(108.83, 89.28, 0.607), 2.0, 1.7)); // Lower altitude, before doorway
  //waypoints.push_back(Waypoint(Point(100.11, 109.10, 0.85), 2.0, 1.5)); // Far side of doorway
  waypoints.push_back(Waypoint(Point(99.2, 109.30, 0.85), 2.0, 1.5));
  waypoints.push_back(Waypoint(Point(73.4, 110.12, 0.53), 4.0, 4.0)); // Increase altitude
  waypoints.push_back(Waypoint(Point(45.57, 178.45, 1.88), 4.0, 2.5)); // Goal
  */

  // -----

  // Orchard, site #2

  pointcloud_file = path_to_data + "pointcloud11_orchard_2_main.ply";
  elevation_map_pointcloud_file = path_to_data + "pointcloud11_orchard_2_elevation_100ppm.ply";

  path_interpolation_step = 0.10; // 10cm
  planner = "BIT_STAR";
  max_planning_time = 15.0;
  output_filename = path_to_home + "path_out_18_2.ply";

  use_altitude_constraint = true;
  use_fixed_altitude_at_endpoints = true; // Doesn't work

  std::vector<Waypoint> waypoints;

  //const double default_radius = 6.0;
  const double default_radius = 2.0;
  waypoints.push_back(Waypoint(Point(0.0, 0.0, 1.727), 15.0, default_radius)); // Start

  waypoints.push_back(Waypoint(Point(41.18, 0.21, 0.0), 15.0, default_radius));
  waypoints.push_back(Waypoint(Point(82.44, 21.31, 0.0), 15.0, default_radius));

  //waypoints.push_back(Waypoint(Point(123.49, 19.17, 0.0), 15.0, default_radius));
  //waypoints.push_back(Waypoint(Point(303.49, -24.97, 0.0), 15.0, default_radius));

  waypoints.push_back(Waypoint(Point(212.76, 3.91, 0.0), 15.0, default_radius));



  waypoints.push_back(Waypoint(Point(429.19, -76.84, 0.0), 15.0, default_radius));
 

  
  waypoints.push_back(Waypoint(Point(441.0, -96.0, 9.856), 15.0, default_radius)); // Goal


//ex17_plan_through_pointcloud_ompl_octree_dtm --input ./pointcloud11_orchard_2_main.ply --start 0.0 0.0 1.727 --goal 441.0 -96.0 9.856 \\
//--elevation_map ./pointcloud11_orchard_2_elevation_100ppm.ply --min_altitude 15.0 --max_altitude 24.0 --robot_radius 6.0  \
//--path_interpolation_step 0.10 --planner BIT_STAR --max_planning_time 60.0 --output ~/path_out_17_12.ply



  // -----




  std::cout << "output_filename =  "<< output_filename << std::endl;
  const std::vector<std::string> output_filename_tokens = splitString(output_filename);

  std::cout << "Loading input PointCloud " << pointcloud_file << std::endl;
  PointCloud::Ptr input_pointcloud(new PointCloud());
  if (!loadPointCloud(pointcloud_file, *input_pointcloud))
  {
    return EXIT_FAILURE;
  }
  std::cout << "Loaded " << input_pointcloud->points.size() << " points" << std::endl;

  PointCloud::Ptr elevation_map_pointcloud(new PointCloud());
  std::cout << "Loading elevation map "<< elevation_map_pointcloud_file << std::endl;
  if (!loadPointCloud(elevation_map_pointcloud_file, *elevation_map_pointcloud))
  {
    return EXIT_FAILURE;
  }
  std::cout << "Loaded " << elevation_map_pointcloud->points.size() << " points" << std::endl;

  /*
  // The radius of a sphere used for collision-checking
  double robot_radius = 1.1 / 2.0; // r = 0.55 = Half the diameter of DJI S1000.
  if (user_specified_robot_radius > 0.000001)
  {
    robot_radius = user_specified_robot_radius;
  }
  std::cout << "Robot radius: " << robot_radius << std::endl;
  */

  if (path_interpolation_step < 0.000001)
  {
    // Set default value
    path_interpolation_step = 0.01; // 1cm between points
  }

  std::cout << "Initializing OctreeMap3D..." << std::endl;
  const double voxel_size = 0.10;
  double robot_radius = 1.1 / 2.0; // Dummy value, we set this later for each segment
  OctreeMap3D map(input_pointcloud, voxel_size, robot_radius);

  if (use_altitude_constraint)
  {
    // Store ptr to the PointCloud that represents the terrain elevation
    // and create a kd-tree from the x,y values.
    map.setElevationMap(elevation_map_pointcloud);
  }



  // -----

  std::vector<std::vector<Point> > path_segments;

  const int num_path_segments = waypoints.size() - 1;
  for (int i = 0; i < num_path_segments; ++i)
  {

    std::cout << "\nPlanning path segment " << i << " of " << num_path_segments << std::endl;

    start_point = waypoints.at(i).point;
    goal_point = waypoints.at(i+1).point;

    //start_min_altitude = waypoints.at(i).min_altitude;
    //start_max_altitude = waypoints.at(i).max_altitude;
    //goal_min_altitude = waypoints.at(i+1).min_altitude;
    //goal_max_altitude = waypoints.at(i+1).max_altitude;

    // Set the z values

    //start_point.z = map.getGroundHeight(start_point) + 4.5;
    //goal_point.z = map.getGroundHeight(goal_point) + 4.5;

    start_point.z = map.getGroundHeight(start_point) + waypoints.at(i).altitude;
    goal_point.z = map.getGroundHeight(goal_point) + waypoints.at(i+1).altitude;

    /*
    // Altitude +tol/-tol
    const double tol = 1.0;
    start_min_altitude = waypoints.at(i).altitude - tol;
    start_max_altitude = waypoints.at(i).altitude + tol;
    goal_min_altitude = waypoints.at(i+1).altitude - tol;
    goal_max_altitude = waypoints.at(i+1).altitude + tol;
    */
    // Altitude +tol/0
    const double tol = 10.0;
    start_min_altitude = waypoints.at(i).altitude;
    start_max_altitude = waypoints.at(i).altitude + tol;
    goal_min_altitude = waypoints.at(i+1).altitude;
    goal_max_altitude = waypoints.at(i+1).altitude + tol;



    // The radius of a sphere used for collision-checking
    const double collision_radius = std::min(waypoints.at(i).collision_radius, waypoints.at(i+1).collision_radius);
    std::cout << "Collision-check radius: " << collision_radius << std::endl;
    map.setRobotRadius(collision_radius);




    if (use_height_constraints)
    {
      std::cout << "Using height constraints: " << std::endl;
      std::cout << "    min z value: " << min_height << std::endl;
      std::cout << "    max z value: " << max_height << std::endl;

      if ((min_height < start_point.z) || (min_height < goal_point.z))
      {
        throw std::runtime_error("If specified, 'min_height' should be greater than the z value of the start and goal");
      }
      if ((max_height < start_point.z) || (max_height < goal_point.z))
      {
        throw std::runtime_error("If specified, 'max_height' should be greater than the z value of the start and goal");
      }
    }

    double takeoff_funnel_min_z;
    double landing_funnel_min_z;
    if (use_funnel_constraints)
    {
      throw std::runtime_error("funnel constraints not implemented in this example!");
    }

    if (use_altitude_constraint)
    {
      std::cout << "Using altitude constraints: " << std::endl;
      std::cout << "    start min z value: " << start_min_altitude << std::endl;
      std::cout << "    start max z value: " << start_max_altitude << std::endl;
      std::cout << "    goal min z value: " << goal_min_altitude << std::endl;
      std::cout << "    goal max z value: " << goal_max_altitude << std::endl;
    }

    //Timer timer;
    //timer.reset();
    //std::cout << "  took " << timer.read() << " ms." << std::endl;




    std::cout << "Finding path to goal..." << std::endl;

    boost::to_upper(planner);
    const PLANNER_TYPE planner_type = stringToEnum(planner);
    const bool interpolate_path = true;
    const double waypoint_separation = 0.0; // 0 = output waypoints at the collision-checking resolution, which is 10cm
    Timer timer;
    timer.reset();
    std::vector<Point> path = map.findPath3D(start_point, goal_point,

                                             min_height, max_height, use_height_constraints,
                                             takeoff_funnel_min_z, landing_funnel_min_z, use_funnel_constraints,

                                             start_min_altitude, start_max_altitude,
                                             goal_min_altitude, goal_max_altitude,

                                             use_altitude_constraint, use_fixed_altitude_at_endpoints,
                                             endpoints_z_offset, use_offset_endpoints_z_value,
                                             planner_type, max_planning_time, interpolate_path, waypoint_separation);
    const double time_taken = timer.read();
    std::cout << "  took " << time_taken << " ms." << std::endl;
    std::cout << "    Path length: " << map.getPathLength() << std::endl;
    std::cout << "    No. waypoints: " << path.size() << std::endl;
    const uint num_checks = map.getNumCollisionChecks();
    std::cout << "    No. of collision checks: " << num_checks << " (" << time_taken/num_checks << " ms per check)" << std::endl;
    if (path.size() == 0)
    {
      return EXIT_FAILURE;

      // TODO: throw exception if length < 2
    }

    //ColorPointCloud path_pointcloud;
    //std::string outfile;

    std::stringstream seg_ss;
    seg_ss << i;
    const std::string segment_id = seg_ss.str();

    /*
    {
      std::cout << "a" << std::endl;

      if (use_height_constraints || use_funnel_constraints || (use_altitude_constraint && !use_fixed_altitude_at_endpoints))
      {
        // Interpolate between the path and the true start & end points, with 1cm resolution
        path = addTakeoffAndLanding<Point>(path, start_point, goal_point, 0, path_interpolation_step);
      }

      std::cout << "b" << std::endl;

      ColorPointCloud path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(path);
      std::string outfile = output_filename_tokens[0] + std::string("_orig.ply");
      std::cout << "Writing to " << outfile << std::endl;
      savePLYFile(outfile, path_pointcloud);
    }
    */

    // Reduce the number of vertices in the path:
    std::cout << "Simplifying path..." << std::endl;
    timer.reset();
    const std::vector<Point> simplified_path = map.getReducedPath();
    std::cout << "  took " << timer.read() << " ms." << std::endl;
    std::cout << "Simplified path now has " << simplified_path.size() << " waypoints" << std::endl;

    // Save this segment
    path_segments.push_back(simplified_path);

    {
      // Debug: save this segment (not smoothed)
      std::vector<Point> piecewise_linear_path;
      for (size_t j = 0; j < (simplified_path.size() - 1); ++j)
      {
        std::vector<double> p0 = {simplified_path.at(j).x, simplified_path.at(j).y, simplified_path.at(j).z};
        std::vector<double> p1 = {simplified_path.at(j+1).x, simplified_path.at(j+1).y, simplified_path.at(j+1).z};
        const std::vector<std::vector<double> > linear_segment = lerp(p0, p1, path_interpolation_step);
        // TODO: if (linear_segment.size() == 0) throw
        for (size_t k = 0; k < linear_segment.size(); ++k)
        {
          piecewise_linear_path.push_back(Point(linear_segment.at(k).at(0),
                                                linear_segment.at(k).at(1),
                                                linear_segment.at(k).at(2)));
        }
      }
      std::cout << "Piecewise linear path has " << piecewise_linear_path.size() << " waypoints" << std::endl;
      ColorPointCloud path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(piecewise_linear_path);
      std::string outfile = output_filename_tokens[0] + std::string("_segment_") + segment_id + std::string(".ply");
      std::cout << "Writing to " << outfile << std::endl;
      savePLYFile(outfile, path_pointcloud);
    }

  } // end foreach waypoint



  // Combine the segments
  std::vector<Point> final_path;
  for (size_t i = 0; i < path_segments.size(); ++i)
  {
    for (size_t j = 0; j < path_segments.at(i).size(); ++j)
    {
      if ((i > 0) && (j == 0))
      {
        // Don't add duplicate points
        continue;
      }
      std::cout << " push back Point " << path_segments.at(i).at(j).x << ", " << path_segments.at(i).at(j).y << ", " << path_segments.at(i).at(j).z << std::endl;
      final_path.push_back(path_segments.at(i).at(j));
    }
  }
  std::cout << "Final path (not smoothed) has " << final_path.size() << " waypoints: " << std::endl;
  //std::cout << "\nFinal path: " << std::endl;
  for (size_t j = 0; j < final_path.size(); ++j)
  {
    //std::cout << "  Segment " << j << ": " << std::endl;
    //const int end_point_idx = final_path.at(j).size() = 1;
    //std::cout << "    altitude: " << map.getAltitude(final_path.at(j).at(0)) << " to " << map.getAltitude(final_path.at(j).at(end_point_idx)) << std::endl;

    std::cout << "  Point " << j << "  ("
              << final_path.at(j).x << ", " << final_path.at(j).y << ", " << final_path.at(j).z
              << ")  altitude: " << map.getAltitude(final_path.at(j))
              << std::endl;
  }

  {
    
  }



  {
    // Save final path (not smoothed)
    std::vector<Point> piecewise_linear_path;
    for (size_t j = 0; j < (final_path.size() - 1); ++j)
    {
      //std::cout << "adding end seg" << std::endl;
      std::vector<double> p0 = {final_path.at(j).x, final_path.at(j).y, final_path.at(j).z};
      std::vector<double> p1 = {final_path.at(j+1).x, final_path.at(j+1).y, final_path.at(j+1).z};
      const std::vector<std::vector<double> > linear_segment = lerp(p0, p1, path_interpolation_step);
      // TODO: if (linear_segment.size() == 0) throw
      for (size_t k = 0; k < linear_segment.size(); ++k)
      {
        piecewise_linear_path.push_back(Point(linear_segment.at(k).at(0),
                                              linear_segment.at(k).at(1),
                                              linear_segment.at(k).at(2)));
      }
    }

    // Interpolate between the path end-points and the true start & end points
    piecewise_linear_path = addTakeoffAndLanding<Point>(piecewise_linear_path, waypoints.at(0).point, waypoints.at(num_path_segments).point, 0, path_interpolation_step);

    ColorPointCloud path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(piecewise_linear_path);
    std::string outfile = output_filename_tokens[0] + std::string("_final.ply");
    std::cout << "Writing to " << outfile << std::endl;
    savePLYFile(outfile, path_pointcloud);
  }



  {
    // Smooth the final path by fitting splines (Centripetal Catmull-Rom) between waypoints.
    // Use this one, curve is tighter than Uniform or Chordal types.
    std::shared_ptr<spline_library::Spline<spline_library::Vector3> > spline = getCentripetalCatmullRomSpline<Point>(final_path);
    std::vector<Point> splined_path = getPointsFromSpline<Point>(spline, path_interpolation_step);
    std::cout << "Splined path has " << splined_path.size() << " waypoints" << std::endl;

    // Interpolate between the path end-points and the true start & end points
    splined_path = addTakeoffAndLanding<Point>(splined_path, waypoints.at(0).point, waypoints.at(num_path_segments).point, path_interpolation_step);

    ColorPointCloud path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(splined_path);
    std::string outfile = output_filename_tokens[0] + std::string("_final_smoothed.ply");
    std::cout << "Writing to " << outfile << std::endl;
    savePLYFile(outfile, path_pointcloud);
  }




  return 0;
}
