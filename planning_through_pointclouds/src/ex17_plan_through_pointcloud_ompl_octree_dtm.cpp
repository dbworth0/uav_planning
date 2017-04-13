/*
Example 17:

Based on 14, but makes min/max height constraint optional, take-off & landing funnel optional,
and adds ability to specify a terrain map.
A robot radius that can colliede with start point ground,
and vary the interpolation spacing.








Usage:
export PATH=$PATH:~/path/to/your/catkin_workspace/devel/lib/planning_through_pointclouds/
cd ~/path/to/your/catkin_workspace/planning_through_pointclouds/data/


Plan a simple path from start to goal:

ex17_plan_through_pointcloud_ompl_octree_dtm --input ./cropped_cloud_1.ply --start 67.0 2.0 -3.0 --goal 65.0 39.0 -2.0 \
--planner BIT_STAR --max_planning_time 5.0 --output ~/path_out_17_1.ply



Plan path which satisfies height constraints:
The start & end of the path will be vertical, linear segments to reach the desired minimum height.
The 'max_height' parameter is optional.

ex17_plan_through_pointcloud_ompl_octree_dtm --input ./cropped_cloud_1.ply --start 67.0 2.0 -3.0 --goal 65.0 39.0 -2.0 \
--min_height 0.0 --max_height 100 \
--planner BIT_STAR --max_planning_time 5.0 --output ~/path_out_17_2.ply



Plan a path where the take-off and landing phases are constrained by funnels:
The funnels are defined by: <offset> <middle_radius> <top_radius> <funnel_height>
so in this example the funnels begin +1 meter above the start and goal positions,
and have a height of 1.3 meters.
The landing funnel has a larger top radius which creates a larger, smoother path.

ex17_plan_through_pointcloud_ompl_octree_dtm --input ./cropped_cloud_1.ply --start 67.0 2.0 -3.0 --goal 65.0 39.0 -2.0 \
--takeoff_funnel +1.0 0.8 1.2 +1.3 --landing_funnel +1.0 0.8 3.0 +1.3 \
--planner BIT_STAR --max_planning_time 5.0 --output ~/path_out_17_3.ply



Plan a path using a ground elevation map:
The desired altitude is higher than the start & goal positions, so linear segments
will be added to reached the desired height.

ex17_plan_through_pointcloud_ompl_octree_dtm --input ./schenley_park_1.ply --start 0.0 0.0 -0.4 --goal 90.0 10.0 -4.1 \
--elevation_map ./schenley_park_1_elevation_100ppm.ply --min_altitude 2.2 --max_altitude 2.3 \
--planner BIT_STAR --max_planning_time 60.0 --output ~/path_out_17_4.ply


Plan a path using a ground elevation map:
Fix the altitude at the start & end points to be within the constraints.

ex17_plan_through_pointcloud_ompl_octree_dtm --input ./schenley_park_1.ply --start 0.0 0.0 -0.4 --goal 90.0 10.0 -4.1 \
--elevation_map ./schenley_park_1_elevation_100ppm.ply --min_altitude 2.2 --max_altitude 2.3 --fix_altitude_at_endpoints \
--planner BIT_STAR --max_planning_time 60.0 --output ~/path_out_17_5.ply


ELEVATION 1.5m high, larger robot radius:

ex17_plan_through_pointcloud_ompl_octree_dtm --input ./schenley_park_1.ply --start 0.0 0.0 -0.18 --goal 90.0 10.0 -3.89 \
--elevation_map ./schenley_park_1_elevation_100ppm.ply --min_altitude 1.5 --max_altitude 1.7 --fix_altitude_at_endpoints \
--robot_radius 1.4 \
--planner BIT_STAR --max_planning_time 60.0 --output ~/path_out_17_6.ply



Carrie Furnace #1




Funnels:

ex17_plan_through_pointcloud_ompl_octree_dtm --input ./pointcloud9_carrie_furnace_1_main.ply --start 0.0 0.0 0.0 --goal 174.7 -0.11 1.3 \
--takeoff_funnel +1.0 0.8 1.2 +1.3 --landing_funnel +1.0 0.8 3.0 +1.3 \
--planner BIT_STAR --max_planning_time 5.0 --output ~/path_out_17_7.ply


Elevation:

ex17_plan_through_pointcloud_ompl_octree_dtm --input ./pointcloud9_carrie_furnace_1_main.ply --start 0.0 0.0 0.0 --goal 174.7 -0.11 1.3 \
--elevation_map ./pointcloud9_carrie_furnace_1_elevation_100ppm.ply --min_altitude 6.0 --max_altitude 7.0 \
--path_interpolation_step 0.10 \
--planner BIT_STAR --max_planning_time 5.0 --output ~/path_out_17_8.ply


Elevation, larger robot radius:

ex17_plan_through_pointcloud_ompl_octree_dtm --input ./pointcloud9_carrie_furnace_1_main.ply --start 0.0 0.0 0.0 --goal 174.7 -0.11 1.3 \
--elevation_map ./pointcloud9_carrie_furnace_1_elevation_100ppm.ply --min_altitude 6.0 --max_altitude 7.0 \
--robot_radius 4.0 --endpoints_z_offset 4.0 \
--path_interpolation_step 0.10 \
--planner BIT_STAR --max_planning_time 5.0 --output ~/path_out_17_9.ply





Different goal position, like Viveks:

(0, 0, 5) and ends at (160, 4.5, 5).

ex17_plan_through_pointcloud_ompl_octree_dtm --input ./pointcloud9_carrie_furnace_1_main.ply --start 0.0 0.0 0.0 --goal 160.0 4.5 1.4 \
--elevation_map ./pointcloud9_carrie_furnace_1_elevation_100ppm.ply --min_altitude 6.0 --max_altitude 7.0 \
--robot_radius 4.0 --endpoints_z_offset 4.0 \
--path_interpolation_step 0.10 \
--planner BIT_STAR --max_planning_time 5.0 --output ~/path_out_17_10.ply



--takeoff_funnel +2.0 2.1 2.1 +1.5 --landing_funnel +2.0 2.1 2.1 +1.5 \



Carrie Furance #2

TEST: This path goes through building wall.

ex17_plan_through_pointcloud_ompl_octree_dtm --input ./pointcloud10_carrie_furnace_2_main.ply \
--start 141.48 2.4 3.9 --goal 73.18 110.12 4.53 \
--planner BIT_STAR --max_planning_time 5.0 --output ~/path_out_17_11.ply

TEST:

ex17_plan_through_pointcloud_ompl_octree_dtm --input ./pointcloud10_carrie_furnace_2_main.ply \
--start 141.48 2.4 3.9 --goal 73.18 110.12 4.53 \
--elevation_map ./pointcloud10_carrie_furnace_2_elevation_100ppm.ply --min_altitude 3.9 --max_altitude 6.0 \
--planner BIT_STAR --max_planning_time 5.0 --output ~/path_out_17_11.ply


TEST: Ground outside of bounds!

ex17_plan_through_pointcloud_ompl_octree_dtm --input ./pointcloud10_carrie_furnace_2_main.ply \
--start 141.48 2.4 3.9 --goal 73.18 110.12 4.53 \
--robot_radius 1.5 \
--planner BIT_STAR --max_planning_time 5.0 --output ~/path_out_17_11.ply

TEST: Nice smooth through doorway, but wrong doorway.

ex17_plan_through_pointcloud_ompl_octree_dtm --input ./pointcloud10_carrie_furnace_2_main.ply \
--start 141.48 2.4 3.9 --goal 73.18 110.12 4.53 \
--elevation_map ./pointcloud10_carrie_furnace_2_elevation_100ppm.ply --min_altitude 3.9 --max_altitude 6.0 \
--robot_radius 1.5 \
--planner BIT_STAR --max_planning_time 20.0 --output ~/path_out_17_11.ply



./pointcloud10_carrie_furnace_2_main_Simplified_for_Planning.ply
pointcloud10_carrie_furnace_2_main.ply




Orchard - site #2

Start and stop path at 3m altitude,
fly at 15m altitude.

Funnels +2 offset, radius = 45, height = 5m above start point (it doesn't look nice if funnel is height of the altitude constraint)

tried robot = 2m, 3m

alt 15 to 17
alt 15 to 30

ex17_plan_through_pointcloud_ompl_octree_dtm --input ./pointcloud11_orchard_2_main.ply --start 0.0 0.0 1.727 --goal 441.0 -96.0 9.856 \
--elevation_map ./pointcloud11_orchard_2_elevation_100ppm.ply --min_altitude 15.0 --max_altitude 17.0 \
--takeoff_funnel +2.0 3.0 45.0 +5.0 --landing_funnel +2.0 3.0 45.0 +5.0 \
--robot_radius 6.0  \
--path_interpolation_step 0.10 \
--planner BIT_STAR --max_planning_time 20.0 --output ~/path_out_17_12.ply


OVER THE TOP:
ex17_plan_through_pointcloud_ompl_octree_dtm --input ./pointcloud11_orchard_2_main.ply --start 0.0 0.0 1.727 --goal 441.0 -96.0 9.856 \
--elevation_map ./pointcloud11_orchard_2_elevation_100ppm.ply --min_altitude 15.0 --max_altitude 24.0 --robot_radius 4.0  \
--path_interpolation_step 0.10 --planner BIT_STAR --max_planning_time 60.0 --output ~/path_out_17_12.ply



ex17_plan_through_pointcloud_ompl_octree_dtm --input ./pointcloud11_orchard_2_main.ply --start 0.0 0.0 1.727 --goal 441.0 -96.0 9.856 \\
--elevation_map ./pointcloud11_orchard_2_elevation_100ppm.ply --min_altitude 15.0 --max_altitude 24.0 --robot_radius 6.0  \
--path_interpolation_step 0.10 --planner BIT_STAR --max_planning_time 60.0 --output ~/path_out_17_12.ply




The flag
--fix_altitude_at_endpoints
is if you want constant altitude, without a take-off and landing section.



r = 3, alt = 15 to 17
r = 6, alt = 15 to 30



--endpoints_z_offset 4.0






Add:
 - add funnels with altitude



David Butterworth
*/

#include <iostream> // cout
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp> // to_upper

#include <planning_through_pointclouds/utils.h> // processCommandLine, Timer
#include <planning_through_pointclouds/pcl_utils.h> // loadPointCloud, getPathPointCloudFromPoints, addTakeoffAndLanding, addVerticalFunnelToPointCloud
#include <planning_through_pointclouds/spline_utils.h> // getCentripetalCatmullRomSpline, getPointsFromSpline
#include <planning_through_pointclouds/octree_map_3d_v5_ompl.h>


#include <spline_library/vector.h>
#include <spline_library/hermite/cubic/cubic_hermite_spline.h>

#include <limits>

//----------------------------------------------------------------------------//

int main(int argc, char **argv)
{
  // For storing command-line arguments
  std::string pointcloud_file;
  std::vector<float> start;
  std::vector<float> goal;
  double user_specified_robot_radius;
  double min_height;
  double max_height;
  bool use_height_constraints;
  std::vector<float> takeoff_funnel;
  std::vector<float> landing_funnel;
  bool use_funnel_constraints;
  std::string elevation_map_pointcloud_file;
  double min_altitude;
  double max_altitude;
  bool use_altitude_constraint;
  bool use_fixed_altitude_at_endpoints;
  double endpoints_z_offset;
  bool use_offset_endpoints_z_value;
  double path_interpolation_step;
  std::string planner;
  double max_planning_time;
  std::string output_filename;

  const bool result = processCommandLine6(argc, argv,
                                          pointcloud_file,
                                          start, goal,
                                          user_specified_robot_radius,
                                          min_height, max_height, use_height_constraints,
                                          takeoff_funnel, landing_funnel, use_funnel_constraints,
                                          elevation_map_pointcloud_file,
                                          min_altitude, max_altitude, use_altitude_constraint,
                                          use_fixed_altitude_at_endpoints,
                                          endpoints_z_offset, use_offset_endpoints_z_value,
                                          planner, max_planning_time, output_filename, path_interpolation_step);
  if (!result)
  {
    return EXIT_FAILURE;
  }
  std::cout << "output_filename =  "<< output_filename << std::endl;
  const std::vector<std::string> output_filename_tokens = splitString(output_filename);

  Point start_point(start.at(0), start.at(1), start.at(2));
  Point goal_point(goal.at(0), goal.at(1), goal.at(2));

  if (use_height_constraints)
  {
    std::cout << "Using height constraints: " << std::endl;
    std::cout << "    min z value: " << min_height << std::endl;
    std::cout << "    max z value: " << max_height << std::endl;

    if ((min_height < start.at(2)) || (min_height < goal.at(2)))
    {
      throw std::runtime_error("If specified, 'min_height' should be greater than the z value of the start and goal");
    }
    if ((max_height < start.at(2)) || (max_height < goal.at(2)))
    {
      throw std::runtime_error("If specified, 'max_height' should be greater than the z value of the start and goal");
    }
  }

  std::cout << "Loading input PointCloud " << pointcloud_file << std::endl;
  PointCloud::Ptr input_pointcloud(new PointCloud());
  if (!loadPointCloud(pointcloud_file, *input_pointcloud))
  {
    return EXIT_FAILURE;
  }
  std::cout << "Loaded " << input_pointcloud->points.size() << " points" << std::endl;

  double takeoff_funnel_min_z;
  double landing_funnel_min_z;
  if (use_funnel_constraints)
  {
    std::cout << "Using funnel constraints: " << std::endl;

    const float point_separation = 0.20;
    addFunnelToPointCloud<Point>(start_point,
                                 takeoff_funnel.at(0), // offset
                                 takeoff_funnel.at(1), // middle_radius
                                 takeoff_funnel.at(2), // top_radius
                                 takeoff_funnel.at(3), // cone_height
                                 point_separation,
                                 input_pointcloud);
    addFunnelToPointCloud<Point>(goal_point, 
                                 landing_funnel.at(0), // offset
                                 landing_funnel.at(1), // middle_radius
                                 landing_funnel.at(2), // top_radius
                                 landing_funnel.at(3), // cone_height
                                 point_separation,
                                 input_pointcloud);

    takeoff_funnel_min_z = start_point.z + takeoff_funnel.at(0);
    landing_funnel_min_z = goal_point.z + landing_funnel.at(0);
    min_height = std::min(takeoff_funnel_min_z, landing_funnel_min_z);
    std::cout << "    Take-off funnel min z: " << takeoff_funnel_min_z << std::endl;
    std::cout << "    Landing funnel min z: " << landing_funnel_min_z << std::endl;
    std::cout << "    global min z value: " << min_height << std::endl;

    {
      // For debugging, save a PointCloud containing just the funnel shapes
      PointCloud::Ptr funnels_pointcloud(new PointCloud());
      addFunnelToPointCloud<Point>(start_point,
                                   takeoff_funnel.at(0),
                                   takeoff_funnel.at(1),
                                   takeoff_funnel.at(2),
                                   takeoff_funnel.at(3),
                                   point_separation,
                                   funnels_pointcloud);
      addFunnelToPointCloud<Point>(goal_point, 
                                   landing_funnel.at(0),
                                   landing_funnel.at(1),
                                   landing_funnel.at(2),
                                   landing_funnel.at(3),
                                   point_separation,
                                   funnels_pointcloud);
      savePLYFile(output_filename_tokens[0] + std::string("_constraint_funnels.ply"), *funnels_pointcloud);
    }
  }

  std::vector<std::vector<Point> > vector_constraints;
  //const bool use_vector_constraints = false;
  const bool use_vector_constraints = true;

  // Big shed near start
  vector_constraints.push_back(std::vector<Point>{Point(-18.79, 19.24, 0), Point(-6.01, 18.78, 0),
                                                  Point(-6.87, -18.19, 0), Point(-19.83, -17.64, 0)
                                                  });




  // Lake
  vector_constraints.push_back(std::vector<Point>{Point(62.68, -30.02, 0), Point(87.06, -29.76, 0),
                                                  Point(116.13, -31.39, 0), Point(119.25, -37.91, 0),
                                                  Point(117.61, -43.39, 0), Point(112.49, -47.35, 0),
                                                  Point(76.80, -58.71, 0), Point(63.93, -57.63, 0),
                                                  Point(58.44, -51.62, 0), Point(57.34, -36.19, 0)
                                                  });


  // Small building
  vector_constraints.push_back(std::vector<Point>{Point(59.36, -1.28, 0), Point(63.37, -2.42, 0),
                                                  Point(60.16, -12.81, 0), Point(56.35, -11.59, 0)
                                                  });


  // Larger building
  vector_constraints.push_back(std::vector<Point>{Point(79.98, 14.81, 0), Point(88.41, 11.05, 0),
                                                  Point(83.63, -0.08, 0), Point(75.05, 3.60, 0)
                                                  });

  // Large building near goal
  vector_constraints.push_back(std::vector<Point>{Point(411.48, -83.17, 0), Point(426.77, -86.60, 0),
                                                  Point(422.47, -105.02, 0), Point(407.36, -101.58, 0)});


  // L-shape
  /*
  vector_constraints.push_back(std::vector<Point>{Point(100.0, 4.0, 0), Point(110.0, 2.15, 0),
                                                  Point(107.07, -76.0, 0), Point(-5.0, -71.7, 0),
                                                  Point(0.07, -62.76, 0), Point(93.67, -64.15, 0)});
  */
  // U-shape
  vector_constraints.push_back(std::vector<Point>{Point(100.0, 4.0, 0), Point(110.0, 2.15, 0),
                                                  //Point(99.31, 22.61, 0), Point(108.79, 23.09, 0),
                                                  Point(107.07, -76.0, 0), Point(-5.0, -71.7, 0),
                                                  Point(-18.83, -18.19, 0), Point(-7.0, -17.9, 0),
                                                  Point(0.07, -62.76, 0), Point(93.67, -64.15, 0)});


  // Along powerlines
  vector_constraints.push_back(std::vector<Point>{Point(141.05, 15.22, 0), Point(301.19, -32.20, 0),
                                                  //Point(296.30, -44.69, 0), Point(137.98, 2.55, 0)
                                                  Point(286.39, -78.44, 0), Point(108.42, -39.82, 0)
                                                  });








  if (use_vector_constraints)
  {
    std::cout << "Using vector constraints: " << std::endl;

    //const float point_separation = 0.02;
    const float point_separation = 0.20;
    const std::vector<float> bounds = getPointCloudBounds<Point>(input_pointcloud);
    const float min_z = bounds.at(4);
    const float max_z = bounds.at(5);
    std::cout << "  min_z: " << min_z << std::endl;
    std::cout << "  max_z: " << max_z << std::endl;
    addWallsToPointCloud<Point>(vector_constraints,
                                point_separation,
                                input_pointcloud,
                                true, // connect end-points to make loop
                                min_z, max_z);

    {
      // For debugging, save a red-colored PointCloud containing just the no-fly zone shapes
      PointCloud::Ptr no_fly_zones_pointcloud(new PointCloud());
      addWallsToPointCloud<Point>(vector_constraints,
                                 point_separation,
                                 no_fly_zones_pointcloud,
                                 true, // connect end-points to make loop
                                 //-10.0, 20.0); // z values
                                 min_z, max_z);

      // Fill-in the regions (doesn't work yet)
      /*
      //for (std::size_t i = 0; i < vector_constraints.size(); ++i)
      //{
        addPolygonToPointCloud<Point>(vector_constraints.at(0),
                            point_separation,
                            no_fly_zones_pointcloud,
                            20.0,
                            10000.0); // alpha, for Concave Hull
      //}
      */

      ColorPointCloud no_fly_zones_pointcloud_colored = getColoredPointCloud<Point, ColorPoint>(255, 0, 0, // red
                                                                                                no_fly_zones_pointcloud);
      savePLYFile(output_filename_tokens[0] + std::string("_vector_constraints.ply"), no_fly_zones_pointcloud_colored);
    }

  }

  PointCloud::Ptr elevation_map_pointcloud(new PointCloud());
  if (use_altitude_constraint)
  {
    std::cout << "Using altitude constraints: " << std::endl;
    std::cout << "    min alt: " << min_altitude << std::endl;
    std::cout << "    max alt: " << max_altitude << std::endl;

    std::cout << "Loading elevation map "<< elevation_map_pointcloud_file << std::endl;

    if (!loadPointCloud(elevation_map_pointcloud_file, *elevation_map_pointcloud))
    {
      return EXIT_FAILURE;
    }
    std::cout << "Loaded " << elevation_map_pointcloud->points.size() << " points" << std::endl;
  }

  Timer timer;

  std::cout << "Initializing OctreeMap3D..." << std::endl;
  timer.reset();

  // The radius of a sphere used for collision-checking
  double robot_radius = 1.1 / 2.0; // r = 0.55 = Half the diameter of DJI S1000.
  if (user_specified_robot_radius > 0.000001)
  {
    robot_radius = user_specified_robot_radius;
  }
  std::cout << "Robot radius: " << robot_radius << std::endl;

  if (path_interpolation_step < 0.000001)
  {
    // Set default value
    path_interpolation_step = 0.01; // 1cm between points
  }

  const double voxel_size = 0.10;
  OctreeMap3D map(input_pointcloud, voxel_size, robot_radius);
  std::cout << "  took " << timer.read() << " ms." << std::endl;

  if (use_altitude_constraint)
  {
    // Store ptr to the PointCloud that represents the terrain elevation
    // and create a kd-tree from the x,y values.
    map.setElevationMap(elevation_map_pointcloud);
  }



  if (use_funnel_constraints)
  {
    // For debug, print altitude of funnel constraints:

    std::cout << "Funnel constraints: " << std::endl;


    std::cout << "    Take-off funnel min z: " << takeoff_funnel_min_z << "  altitude = "
              <<  map.getAltitude(Point(start_point.x, start_point.y, takeoff_funnel_min_z)) << std::endl;

    std::cout << "                    max z: " << (takeoff_funnel_min_z + takeoff_funnel.at(3)) << "  altitude = "
              <<  map.getAltitude(Point(start_point.x, start_point.y, (takeoff_funnel_min_z + takeoff_funnel.at(3)))) << std::endl;



    std::cout << "    Landing funnel min z: " << landing_funnel_min_z << "  altitude = "
              <<  map.getAltitude(Point(goal_point.x, goal_point.y, landing_funnel_min_z)) << std::endl;

    std::cout << "                    max z: " << (landing_funnel_min_z + landing_funnel.at(3)) << "  altitude = "
              <<  map.getAltitude(Point(goal_point.x, goal_point.y, (landing_funnel_min_z + landing_funnel.at(3)))) << std::endl;


    std::cout << "    global min z value: " << min_height << std::endl;
  }



  std::cout << "Finding path to goal..." << std::endl;

  boost::to_upper(planner);
  const PLANNER_TYPE planner_type = stringToEnum(planner);
  const bool interpolate_path = true;
  const double waypoint_separation = 0.0; // 0 = output waypoints at the collision-checking resolution, which is 10cm
  timer.reset();
  std::vector<Point> path = map.findPath3D(start_point, goal_point,
                                           min_height, max_height, use_height_constraints,
                                           takeoff_funnel_min_z, landing_funnel_min_z, use_funnel_constraints,
                                           min_altitude, max_altitude, // for start
                                           min_altitude, max_altitude, // for goal
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
  }

  ColorPointCloud path_pointcloud;
  std::string outfile;

  {
    if (use_height_constraints || use_funnel_constraints || (use_altitude_constraint && !use_fixed_altitude_at_endpoints))
    {
      // Interpolate between the path and the true start & end points, with 1cm resolution
      path = addTakeoffAndLanding<Point>(path, start_point, goal_point, 0, path_interpolation_step);
    }

    path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(path);
    outfile = output_filename_tokens[0] + std::string("_orig.ply");
    std::cout << "Writing to " << outfile << std::endl;
    savePLYFile(outfile, path_pointcloud);
  }

  // Reduce the number of vertices in the path:
  std::cout << "Simplifying path..." << std::endl;
  timer.reset();
  const std::vector<Point> simplified_path = map.getReducedPath();
  std::cout << "  took " << timer.read() << " ms." << std::endl;
  std::cout << "Simplified path now has " << simplified_path.size() << " waypoints" << std::endl;
  {
    // Save trajectory
    path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(simplified_path);
    outfile = output_filename_tokens[0] + std::string("_simplified.ply");
    std::cout << "Writing to " << outfile << std::endl;
    savePLYFile(outfile, path_pointcloud);
  }

  // Fit splines (Centripetal Catmull-Rom) between waypoints
  // Use this one, curve is tighter than Uniform or Chordal types.
  std::shared_ptr<spline_library::Spline<spline_library::Vector3> > spline = getCentripetalCatmullRomSpline<Point>(simplified_path);
  std::vector<Point> splined_path = getPointsFromSpline<Point>(spline, path_interpolation_step);
  std::cout << "Splined path now has " << splined_path.size() << " waypoints" << std::endl;
  if (use_height_constraints || use_funnel_constraints || (use_altitude_constraint && !use_fixed_altitude_at_endpoints))
  {
    // Interpolate between the path and the true start & end points, with 1cm resolution
    splined_path = addTakeoffAndLanding<Point>(splined_path, start_point, goal_point, 0, path_interpolation_step);
  }
  path_pointcloud = getPathPointCloudFromPoints<Point, ColorPoint>(splined_path);
  outfile = output_filename_tokens[0] + std::string("_splined_CentripetalCR.ply");
  std::cout << "Writing to " << outfile << std::endl;
  savePLYFile(outfile, path_pointcloud);

  return 0;
}
