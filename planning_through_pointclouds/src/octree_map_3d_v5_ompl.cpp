//
// Octree Map 3D v5


#include <planning_through_pointclouds/octree_map_3d_v5_ompl.h>

#include <iostream> // cout
//#include <queue> // constant-time lookup of largest element
//#include <algorithm> // std::reverse

#include <pcl/common/distances.h> // euclideanDistance()

//#include <planning_through_pointclouds/utils.h> // range, printStdVector
#include <planning_through_pointclouds/pcl_utils.h> // loadPointCloud, getPointCloudBounds, downsamplePointCloud, euclideanDistanceXY

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
//#include <ompl/geometric/planners/prm/PRM.h>

// BIT* is not available in OMPL distributed with Ubuntu, you must compile from trunk
#ifdef USE_BITSTAR
#include <ompl/geometric/planners/bitstar/BITstar.h>
#endif

OctreeMap3D::OctreeMap3D(PointCloud::Ptr& input_pointcloud, const double voxel_size, const double robot_radius)
  : octree_voxel_size_(voxel_size)
  , robot_radius_(robot_radius)
  , ground_pointcloud_(NULL)
{
  pointcloud_dims_ = Bounds(getPointCloudBounds<Point>(input_pointcloud));

  std::cout << "Cloud dimensions: " << std::endl
            << "  x_min: " << pointcloud_dims_.min_x << " "
            << "  x_max: " << pointcloud_dims_.max_x << std::endl
            << "  y_min: " << pointcloud_dims_.min_y << " "
            << "  y_max: " << pointcloud_dims_.max_y << std::endl
            << "  z_min: " << pointcloud_dims_.min_z << " "
            << "  z_max: " << pointcloud_dims_.max_z << std::endl;
  // ToDo: Verify max is greater than min

  std::cout << "Generating Octree from PointCloud... " << std::endl;
  octree_ = pcl::octree::OctreePointCloudSearch<Point>::Ptr(new pcl::octree::OctreePointCloudSearch<Point>(octree_voxel_size_));
  octree_->setInputCloud(input_pointcloud); // set ptr to input data
  octree_->addPointsFromInputCloud();

  std::cout << "Done." << std::endl;
}



void OctreeMap3D::setElevationMap(PointCloud::Ptr& pointcloud)
{
  ground_pointcloud_ = pointcloud;

  std::cout << "Generating kd-tree from Elevation Map... " << std::endl;

  // Copy elevation cloud and make the z-values the same.
  // (This is so the z-value doesn't affect the FLANN nearest neighbour search)
  PointCloud::Ptr elevation_pointcloud_2d(new PointCloud(*ground_pointcloud_));
  for (size_t i = 0; i < elevation_pointcloud_2d->points.size(); ++i)
  {
    elevation_pointcloud_2d->points.at(i).z = 0.0; 
  }

  ground_xy_kdtree_.setInputCloud(elevation_pointcloud_2d);
}


void OctreeMap3D::setRobotRadius(const double radius)
{
  robot_radius_ = radius;
}



const std::vector<Point> OctreeMap3D::findPath3D(const Point& start_point,
                                                 const Point& goal_point,

                                                 const double min_height,
                                                 const double max_height,
                                                 const bool use_height_constraints,

                                                 const double takeoff_funnel_min_z,
                                                 const double landing_funnel_min_z,
                                                 const bool use_funnel_constraints,

                                                 const double start_min_altitude, const double start_max_altitude,
                                                 const double goal_min_altitude, const double goal_max_altitude,
                                                 const bool use_altitude_constraint,
                                                 const bool use_fixed_altitude_at_endpoints,

                                                 const double endpoints_z_offset,
                                                 const bool use_offset_endpoints_z_value,

                                                 const int planner_type,
                                                 const double max_planning_time,
                                                 const bool interpolate_path,
                                                 const double interpolation_step)
{
  std::cout << "  Start point: " << start_point.x << "," << start_point.y << "," << start_point.z << std::endl;
  std::cout << "  Goal point: " << goal_point.x << "," << goal_point.y << "," << goal_point.z << std::endl;


  // For  use_offset_endpoints_z_value
  // TODO: add checks, if min_height < (z + offset), or if altitude < (z + offset)


  if (use_offset_endpoints_z_value)
  {
    if (use_altitude_constraint)
    {

    }

      /*
  {
    start[2] = min_height;
  }
  if (use_height_constraints && (min_height > pointcloud_dims_.min_z))
  {
    goal[2] = min_height;
  }


  if (use_offset_endpoints_z_value)
  {
    start[2] = start_point.z + endpoints_z_offset;

    goal[2] = goal_point.z + endpoints_z_offset;
  }
  */



  }











  // Check if start and goal are within the PointCloud and not in collision.
  // Note: Depending on the arguments enabled, the planner may actually use a different
  //       start and goal point. See code below.
  if (use_offset_endpoints_z_value)
  {
    checkStartAndGoalValidity(Point(start_point.x,
                                    start_point.y,
                                    start_point.z + endpoints_z_offset),
                              Point(goal_point.x,
                                    goal_point.y,
                                    goal_point.z + endpoints_z_offset));
  }
  
  else if (use_fixed_altitude_at_endpoints)
  {
    checkStartAndGoalValidity(Point(start_point.x,
                                    start_point.y,
                                    getGroundHeight(start_point) + start_min_altitude),
                              Point(goal_point.x,
                                    goal_point.y,
                                    getGroundHeight(goal_point) + goal_min_altitude));
  }
  
  else
  {
    checkStartAndGoalValidity(start_point, goal_point);
  }

  // Clear results from previous plan
  num_collision_checks_ = 0; // reset counter
  path_length_ = 0.0;
  // TODO: zero out these
  // ompl::base::PlannerDataPtr getPlannerData();
  // const std::vector<Point> getSimplifiedPath(
  // const std::vector<Point> getReducedPath();

  if (use_altitude_constraint && !ground_pointcloud_)
  {
    throw std::runtime_error("You need to call setElevationMap()");
  }





  if (use_altitude_constraint)
  {
    //min_altitude_ = min_altitude;
    //max_altitude_ = max_altitude;
    min_altitude_ = std::min(start_min_altitude, goal_min_altitude);
    max_altitude_ = std::max(start_max_altitude, goal_max_altitude);
    std::cout << "min_altitude_: " << min_altitude_ << std::endl;
    std::cout << "max_altitude_: " << max_altitude_ << std::endl;

    if (max_altitude_ < min_altitude_)
    {
      throw std::runtime_error("'max_altitude' must be greater than 'min_altitude' in findPath3D()");
    }

    std::cout << "    The ground height at start x,y = " << getGroundHeight(start_point) << std::endl;
    std::cout << "                         goal x,y = " << getGroundHeight(goal_point) << std::endl;

    std::cout << "    The altitude of the start point = " << getAltitude(start_point) << std::endl;
    std::cout << "                        goal point = " << getAltitude(goal_point) << std::endl;
  }

  // Initialize Vector state space (x,y,z linear motion)
  const int num_dimensions = 3;
  state_space_.reset(new ompl::base::RealVectorStateSpace(num_dimensions));
  ompl::base::RealVectorBounds bounds(num_dimensions);
  bounds.setLow(0,  pointcloud_dims_.min_x);
  bounds.setHigh(0, pointcloud_dims_.max_x);
  bounds.setLow(1,  pointcloud_dims_.min_y);
  bounds.setHigh(1, pointcloud_dims_.max_y);
  bounds.setLow(2,  pointcloud_dims_.min_z);
  bounds.setHigh(2, pointcloud_dims_.max_z);


  if (use_height_constraints && (min_height > pointcloud_dims_.min_z))
  {
    bounds.setLow(2, min_height - 0.001); // Just below the z value of the start & end points
  }
  if (use_height_constraints && (max_height < pointcloud_dims_.max_z))
  {
    bounds.setHigh(2, max_height);
  }
  
  if (use_funnel_constraints)
  {
    // Use the lowest point of both funnels
    const double min_z_height = std::min(takeoff_funnel_min_z, landing_funnel_min_z);
    bounds.setLow(2, min_z_height);
  }


  /*
  if (use_altitude_constraint)
  {
    // Bound the z values for the sample-based planner, to reduce the space from which we sample (x,y,z) values.
    // This doesn't work for maps with a valley between the start and goal, because the minimum altitude of that area is lower.
    const double min_z_height = std::min(getGroundHeight(start_point) + start_min_altitude, getGroundHeight(goal_point) + goal_min_altitude);
    const double max_z_height = std::max(getGroundHeight(start_point) + start_max_altitude, getGroundHeight(goal_point) + goal_max_altitude);
    const double epsilon = 0.001;
    bounds.setLow(2,  min_z_height - epsilon); // Tolerance, to allow for fixed-altitude start and goal
    bounds.setHigh(2, max_z_height + epsilon);
    std::cout << "Bounding min_z_height: " << min_z_height - epsilon << std::endl;
    std::cout << "         max_z_height: " << max_z_height + epsilon << std::endl;
  }
  */



  // For testing
  //
  // pointcloud7_indoor_highbay_no_floor_small.pcd
  //bounds.setLow(2,  0.2-0.005); // .005 is min threshold, .002 doesnt work as well
  //bounds.setHigh(2, 0.2+0.005);
  // #10
  //bounds.setLow(2,  -0.7-0.005); // .005 is min threshold, .002 doesnt work as well
  //bounds.setHigh(2, -0.7+0.005);


  state_space_->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
  state_space_->setup();

  space_information_ = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(state_space_));

  // Collision-check a sphere of the robot's radius
  //space_information_->setStateValidityChecker(boost::bind(&OctreeMap3D::isStateValid_CheckSphere, this, _1));
  if (!use_altitude_constraint)
  {
    space_information_->setStateValidityChecker(boost::bind(&OctreeMap3D::isStateValid_CheckSphere, this, _1));
  }
  else
  {
    space_information_->setStateValidityChecker(boost::bind(&OctreeMap3D::isStateValid_CheckSphere_CheckAltitude, this, _1));
  }

  


  
  // Set collision-checking resolution (one or half voxel size).
  // In OMPL, collision-checking defaults to 0.01 (1%) of the state's maximum extent.
  const double max_extent = std::max({pointcloud_dims_.x_range, pointcloud_dims_.y_range, pointcloud_dims_.z_range}); // C++11
  const double resolution_meters = octree_voxel_size_; // / 2.0;
  const double resolution_percent = 1.0 / (max_extent / resolution_meters);
  space_information_->setStateValidityCheckingResolution(resolution_percent); 
  std::cout << "OMPL collision-checking resolution: "
            << resolution_percent << "% = " << resolution_percent*max_extent << "m"
            << "  (default: 1% = " << 0.01*max_extent << "m)" << std::endl;

  // Use a custom State Sampler:
  //
  // This example uses a uniform distribution, which is the same that OMPL uses by default.
  // Note: RRT* and BIT* use a StateSampler but not ValidStateSampler
  //state_space_->setStateSamplerAllocator(&allocateStateSamplerUniform);

  space_information_->setup();

  ompl::base::ScopedState<> start(state_space_);
  start[0] = start_point.x;
  start[1] = start_point.y;
  start[2] = start_point.z;
  //start.print(cout);
  ompl::base::ScopedState<> goal(state_space_);
  goal[0] = goal_point.x;
  goal[1] = goal_point.y;
  goal[2] = goal_point.z;
  //goal.print(cout);

  // If using a minimum height constraint, we set the z value
  // of the start & goal position to be the minimum height.
  // Later, a linear segment will be added between the true
  // start & goal positions.
  if (use_height_constraints && (min_height > pointcloud_dims_.min_z))
  {
    start[2] = min_height;
  }
  if (use_height_constraints && (min_height > pointcloud_dims_.min_z))
  {
    goal[2] = min_height;
  }

  if (use_offset_endpoints_z_value)
  {
    start[2] = start_point.z + endpoints_z_offset;
    goal[2] = goal_point.z + endpoints_z_offset;
  }

  if (use_funnel_constraints)
  {
    // Set the z values of the start & goal positions to be inside
    // the funnel constraint, being just above the base of the funnel.
    // Later, a linear segment will be added between the true start & goal positions.
    const double epsilon = 0.05;
    start[2] = takeoff_funnel_min_z + epsilon;
    goal[2] = landing_funnel_min_z + epsilon;

    std::cout << "  use_funnel_constraints == true" << std::endl;
    std::cout << "    start[2]: " << start[2] << std::endl;
    std::cout << "    goal[2]: " << goal[2] << std::endl;
  }

  //if (use_altitude_constraint)
  if (use_altitude_constraint && !use_funnel_constraints)
  {
    if (!use_fixed_altitude_at_endpoints)
    {
      std::cout << "\n not fixed: " << std::endl;

      // Set the z values of the start & goal positions to be
      // within the altitude constraints.
      // Later, a linear segment will be added between the true start & goal positions.

      if ((getAltitude(start_point) < start_min_altitude) || (getAltitude(start_point) > start_max_altitude))
      {
        start[2] = getGroundHeight(start_point) + ((start_min_altitude + start_max_altitude) / 2.0);
        std::cout << "\n start[2]: " << start[2] << std::endl;
      }

      if ((getAltitude(goal_point) < goal_min_altitude) || (getAltitude(goal_point) > goal_max_altitude))
      {
        goal[2] = getGroundHeight(goal_point) + ((goal_min_altitude + goal_max_altitude) / 2.0);
        std::cout << "\n goal[2]: " << goal[2] << std::endl;
      }
    }
    else
    {
      // Fix the altitude of the start & goal positions to be exactly
      // the middle of the altitude constraints.
      //start[2] = getGroundHeight(start_point) + ((min_altitude_ + max_altitude_) / 2.0);
      //goal[2] = getGroundHeight(goal_point) + ((min_altitude_ + max_altitude_) / 2.0);

      //const float epsilon = 0.15;
      const float epsilon = 0.01;
      start[2] = getGroundHeight(start_point) + start_min_altitude + epsilon;
      goal[2] = getGroundHeight(goal_point) + goal_min_altitude + epsilon;

      //start[2] = start_point.z;
      //goal[2] = goal_point.z;

      std::cout << "  start[2]: " << start[2] << std::endl;
      std::cout << "  goal[2]: " << goal[2] << std::endl;
    }
  }



  problem_definition_ = ompl::base::ProblemDefinitionPtr(new ompl::base::ProblemDefinition(space_information_));
  problem_definition_->setStartAndGoalStates(start, goal);

  // Optional
  // This is the default objective anyway
  problem_definition_->setOptimizationObjective(ompl::base::OptimizationObjectivePtr(new ompl::base::PathLengthOptimizationObjective(space_information_)));

  double planning_time_limit = max_planning_time;

  if (planner_type == RRT)
  {
    ompl_planner_ = ompl::base::PlannerPtr(new ompl::geometric::RRT(space_information_));
  }
  else if (planner_type == RRT_CONNECT)
  {
    ompl_planner_ = ompl::base::PlannerPtr(new ompl::geometric::RRTConnect(space_information_));
  }
  else if (planner_type == RRT_STAR)
  {
    // RRT* will keep planning for max_planning_time
    ompl_planner_ = ompl::base::PlannerPtr(new ompl::geometric::RRTstar(space_information_));
  }
  else if (planner_type == BIT_STAR)
  {
    // Check that we have OMPL version with BIT*
#ifdef USE_BITSTAR
    if (max_planning_time < 0.0)
    {
      // Force the planner to stop after it finds any solution
      problem_definition_->setOptimizationObjective(this->getPathLengthObjectiveWithCostThreshold(space_information_, std::numeric_limits<double>::max()));
      // Force the solver to run by setting max planning time to 10 minutes.
      // Note: if set to the numeric limit, the planner will exit immediately!
      //planning_time_limit = std::numeric_limits<double>::max();
      planning_time_limit = 60.0*10;
    }
    // BIT* will keep planning for max_planning_time, unless set to negative value
    boost::shared_ptr<ompl::geometric::BITstar> bitstar(new ompl::geometric::BITstar(space_information_));
    std::cout << "BIT* default re-wire factor: " << bitstar->getRewireFactor() << std::endl; // 1.1
    std::cout << "BIT* default samples per batch: " << bitstar->getSamplesPerBatch() << std::endl; // 100
    std::cout << "BIT* default pruning: " << bitstar->getPruning() << std::endl; // True
    //// Set the rewiring scale factor, s, such that r_rrg = s r_rrg*.
    //bitstar->setRewireFactor(1.2);
    //bitstar->setSamplesPerBatch(100);
    //// Enable pruning of vertices/samples that CANNOT improve the current solution
    //bitstar->setPruning(true);
    ompl_planner_ = bitstar;
#else
    std::cout << "ERROR: BIT* is not available" << std::endl;
    return std::vector<Point>();
#endif
  }
  else
  {
    throw std::runtime_error("Unknown planner_type in findPath()");
  }

  ompl_planner_->setProblemDefinition(problem_definition_);
  ompl_planner_->setup();

  std::cout << "OMPL planning time limit = " << planning_time_limit << std::endl;

  ompl::base::PlannerStatus solved = ompl_planner_->solve(planning_time_limit);
  if (!solved)
  {
    std::cout << "ERROR: Failed to find path!" << std::endl;
    return std::vector<Point>();
  }

  path_length_ = problem_definition_->getSolutionPath()->length();
  boost::shared_ptr<ompl::geometric::PathGeometric> ompl_path = boost::static_pointer_cast<ompl::geometric::PathGeometric>(problem_definition_->getSolutionPath());

  if (interpolate_path)
  {
    if (interpolation_step > 0.00001)
    {
      const int num_waypoints = std::ceil(path_length_ / interpolation_step);
      // OMPL will evenly distribute the waypoints along the path length
      ompl_path->interpolate(num_waypoints);
    }
    else
    {
      // One waypoint per valid state.
      // The distance between waypoints thus depends on
      // OMPL's collision-checking resolution.
      ompl_path->interpolate();
    }
  }

  return getPathPoints(ompl_path);
}

ompl::base::OptimizationObjectivePtr OctreeMap3D::getPathLengthObjectiveWithCostThreshold(const ompl::base::SpaceInformationPtr& space_information,
                                                                                         const double cost_threshold)
{
  ompl::base::OptimizationObjectivePtr objective(new ompl::base::PathLengthOptimizationObjective(space_information));
  objective->setCostThreshold(ompl::base::Cost(cost_threshold));
  return objective;
}

// ToDo: Should check if state isvalid.
//       Also, OMPL expects this function to be thread save but isInCollision() is not.
const bool OctreeMap3D::isStateValid_CheckSphere(const ompl::base::State* state)
{
  num_collision_checks_++;

  const ompl::base::RealVectorStateSpace::StateType* state_3d = state->as<ompl::base::RealVectorStateSpace::StateType>();
  const Point center_point = Point(static_cast<float>(state_3d->values[0]),
                                   static_cast<float>(state_3d->values[1]),
                                   static_cast<float>(state_3d->values[2]));
  if (isInCollisionSphere(center_point, robot_radius_))
  {
    return false;
  }

  return true;
}

const bool OctreeMap3D::isStateValid_CheckSphere_CheckAltitude(const ompl::base::State* state)
{
  num_collision_checks_++;

  const ompl::base::RealVectorStateSpace::StateType* state_3d = state->as<ompl::base::RealVectorStateSpace::StateType>();
  const Point center_point = Point(static_cast<float>(state_3d->values[0]),
                                   static_cast<float>(state_3d->values[1]),
                                   static_cast<float>(state_3d->values[2]));

  const float altitude = getAltitude(center_point);
  //std::cout << "altitude: " << altitude << "   " << min_altitude_ << " : " << max_altitude_ << std::endl;

Point start_point(0,0,0);
Point goal_point(441,-96,0);
const float r = 45.0;


  // Option
  const bool use_funnel_constraints = true;


  bool check_altitude = true;

  if (use_funnel_constraints)
  {
    // Using constraint funnels for take-off and landing,
    // therefore don't constrain the altitude within the funnel radius
    // so the path is a smooth curve.
    if ((euclideanDistanceXY(center_point, start_point) < r) || (euclideanDistanceXY(center_point, goal_point) < r))
    {
      check_altitude = false;
    }
  }



  if (check_altitude)
  {
    if (altitude < min_altitude_)
    {
      return false;
    }
    else if (altitude > max_altitude_)
    {
      return false;
    }
  }



  if (isInCollisionSphere(center_point, robot_radius_))
  {
    return false;
  }

  return true;
}

const bool OctreeMap3D::isInCollisionSphere(const Point& center_point, const double radius)
{
  std::vector<int> nn_indices;
  std::vector<float> nn_dists;


Point start_point(0,0,0);
Point goal_point(441,-96,0);
// robot_radius_near_endpoints


  double radius_to_check = radius;

// 0.10 should be voxel resolution

// TODO: add if using funnels?

  // This prevents collision with ground, near start point, when using large radius
  if ((pcl::euclideanDistance(center_point, start_point) < (radius + 0.11)) || (pcl::euclideanDistance(center_point, goal_point) < (radius + 0.11)))

  // This prevents collision error with funnel
  //if ((pcl::euclideanDistance(center_point, start_point) < 45.0) || (pcl::euclideanDistance(center_point, goal_point) < 45.0))
  {
    // We are very close to the end-points, so do collision checking using the
    // minimum required robot radius. This prevents false positive, when using
    // an inflated radius where the path is close to the ground.
    radius_to_check =  0.8;
  }
  

  const uint max_nn = 1; // 1 or more neighbours = collision
  if (octree_->radiusSearch(center_point, radius_to_check, nn_indices, nn_dists, max_nn))
  {
    return true;
  }

  return false;
}

const ompl::base::PlannerDataPtr OctreeMap3D::getPlannerData()
{
  const ompl::base::PlannerDataPtr planner_data(new ompl::base::PlannerData(space_information_));
  ompl_planner_->getPlannerData(*planner_data);
  return planner_data;
}

const std::vector<Point> OctreeMap3D::getPathPoints(const boost::shared_ptr<ompl::geometric::PathGeometric> path_geometric_ptr)
{
  if (path_geometric_ptr->getStateCount() <= 0)
  {
    throw std::runtime_error("OMPL path has zero states in getPathPoints()");
  }

  // Convert path coordinates
  std::vector<Point> path_points;
  for (std::size_t i = 0; i < path_geometric_ptr->getStateCount(); ++i)
  {
    if (!path_geometric_ptr->getState(i))
    {
      throw std::runtime_error("getState() returned null ptr in getPathPoints()");
    }

    const ompl::base::RealVectorStateSpace::StateType* real_state = 
        static_cast<const ompl::base::RealVectorStateSpace::StateType*>(path_geometric_ptr->getState(i));
    const float x = real_state->values[0];
    const float y = real_state->values[1];
    const float z = real_state->values[2];

    path_points.push_back(Point(x, y, z));
  }

  return path_points;
}

const std::vector<Point> OctreeMap3D::getSimplifiedPath(const double max_time,
                                                        const bool interpolate_path,
                                                        const double interpolation_step)
{
  boost::shared_ptr<ompl::geometric::PathGeometric> ompl_path = boost::static_pointer_cast<ompl::geometric::PathGeometric>(problem_definition_->getSolutionPath());
  if (!ompl_path)
  {
    throw std::runtime_error("Path is empty in getSimplifiedPath()");
  }

  ompl::geometric::PathSimplifier simplifier(space_information_);
  simplifier.simplify(*ompl_path, max_time);

  // This never finishes:
  //simplifier.smoothBSpline(*ompl_path, max_time);

  if (interpolate_path)
  {
    if (interpolation_step > 0.00001)
    {
      const int num_waypoints = std::ceil(path_length_ / interpolation_step);
      // OMPL will evenly distribute the waypoints along the path length
      ompl_path->interpolate(num_waypoints);
    }
    else
    {
      // One waypoint per valid state.
      // The distance between waypoints thus depends on
      // OMPL's collision-checking resolution.
      ompl_path->interpolate();
    }
  }

  // short-cut
  //path->interpolate(100);
  //simplifier.reduceVertices(*ompl_path);

  return getPathPoints(ompl_path);
}

const std::vector<Point> OctreeMap3D::getReducedPath()
{
  boost::shared_ptr<ompl::geometric::PathGeometric> ompl_path = boost::static_pointer_cast<ompl::geometric::PathGeometric>(problem_definition_->getSolutionPath());
  if (!ompl_path)
  {
    throw std::runtime_error("Path is empty in getReducedPath()");
  }

  ompl::geometric::PathSimplifier simplifier(space_information_);
  simplifier.reduceVertices(*ompl_path);

  return getPathPoints(ompl_path);
}

void OctreeMap3D::checkStartAndGoalValidity(const Point& start_point, const Point& goal_point)
{
  if ((start_point.x < pointcloud_dims_.min_x) || (start_point.x > pointcloud_dims_.max_x)
       && (start_point.y < pointcloud_dims_.min_y) || (start_point.y > pointcloud_dims_.max_y)
       && (start_point.z < pointcloud_dims_.min_z) || (start_point.z > pointcloud_dims_.max_z))
  {
    throw std::runtime_error("start point is outside extents of input PointCloud");
  }

  if ((goal_point.x < pointcloud_dims_.min_x) || (goal_point.x > pointcloud_dims_.max_x)
       && (goal_point.y < pointcloud_dims_.min_y) || (goal_point.y > pointcloud_dims_.max_y)
       && (goal_point.z < pointcloud_dims_.min_z) || (goal_point.z > pointcloud_dims_.max_z))
  {
    throw std::runtime_error("goal point is outside extents of input PointCloud");
  }

  if (isInCollisionSphere(start_point, robot_radius_))
  {
    std::stringstream ss;
    ss << "Start point (" << start_point.x << ", " << start_point.y << ", " << start_point.z << ") is in collision!";
    throw std::runtime_error(ss.str());
  }

  if (isInCollisionSphere(start_point, robot_radius_))
  {
    std::stringstream ss;
    ss << "Goal point (" << goal_point.x << ", " << goal_point.y << ", " << goal_point.z << ") is in collision!";
    throw std::runtime_error(ss.str());
  }
}

const float OctreeMap3D::getGroundHeight(const Point& point)
{
  const int K = 1;
  std::vector<int> result_points_indices(K);
  std::vector<float> result_distances(K);
  if (ground_xy_kdtree_.nearestKSearch(point, K, result_points_indices, result_distances) > 0)
  {
    if (result_points_indices.size() != 1)
    {
      throw std::runtime_error("nearestKSearch() should only find 1 point");
    }
    const int closest_point_index = result_points_indices.at(0);

    return ground_pointcloud_->points.at(closest_point_index).z;
  }

  throw std::runtime_error("nearestKSearch() failed to find the nearest point");
}

const float OctreeMap3D::getAltitude(const Point& point)
{
  const float ground_height = getGroundHeight(point);

  return point.z - ground_height;
}