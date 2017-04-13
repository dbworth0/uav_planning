//
// Octree Map 3D v3
//
// Stores a PointCloud in a PCL Octree.
// Planning using RRT variants from OMPL (Open Motion Planning Libray)
// with a custom State space (x,y,elevation) similar to Dubins Airplane.
// Can use RRT* or BIT*, as well as a custom Task-space RRT*.
//
// The Task-space RRT* treats the angle separately to reduce computation,
// instead of planning in the full state-space, but the resulting paths
// are longer than normal RRT*.
//
// During planning, a state is considered "valid" if all points within
// the robot's radius are free. The planner interpolates between
// states and makes sure the space in-between is also free.
// 
// David Butterworth
//

// Ocassionally throws error
//
// terminate called after throwing an instance of 'ompl::Exception'
// what():  Trying to prune goal vertex. Something went wrong.
//
// terminate called after throwing an instance of 'ompl::Exception'
// what():  Attempting to access a pruned vertex.

#include <planning_through_pointclouds/octree_map_3d_v3_ompl_dubins.h>

#include <iostream> // cout
#include <queue> // constant-time lookup of largest element
#include <algorithm> // std::reverse

//#include <pcl/common/distances.h> // euclideanDistance()

//#include <planning_through_pointclouds/utils.h> // range, printStdVector
#include <planning_through_pointclouds/pcl_utils.h> // loadPointCloud, downsamplePointCloud

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
//#include <ompl/geometric/planners/prm/PRM.h>

#include <ompl_state_spaces/dubins_z_state_space.h> // DubinsZStateSpace, DubinsZMotionValidator, TaskSpaceDubinsZMotionValidator
#include <boost/function.hpp> // for task_space_rrtstar.h>
#include <planning_through_pointclouds/task_space_rrtstar.h> // Custom RRT*

#include <Eigen/Geometry>

// BIT* is not available in OMPL distributed with Ubuntu, you must compile from trunk
#ifdef USE_BITSTAR
#include <ompl/geometric/planners/bitstar/BITstar.h>
#endif

// Get the angle between two vectors in radians
const double angleBetweenVectors(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
{
  const Eigen::Vector3d v1n(v1.normalized());
  const Eigen::Vector3d v2n(v2.normalized());

  const double a = v1n.dot(v2n);
  double angle = std::acos(a);

  const Eigen::Vector3d cross_product = v1n.cross(v2n);
  const Eigen::Vector3d normal(0.0, 0.0, 1.0);
  if (normal.dot(cross_product) < 0.0)
  {
    angle = -1.0 * angle;
  }
  return angle;
}

// Get the angle (in radians) between the start and goal positions.
// This is used to set the start angle to point towards the goal.
const double angleBetweenStartAndGoal(const Point& start, const Point& goal)
{
  const Eigen::Vector3d start_vec(1.0, 0.0, 0.0);
  const Eigen::Vector3d start_to_goal_vec((goal.x - start.x), (goal.y - start.y), (goal.z - start.z));
  //std::cout << "Vector start to goal: " << start_to_goal_vec << std::endl;

  return angleBetweenVectors(start_vec, start_to_goal_vec);  
}

OctreeMap3D::OctreeMap3D(PointCloud::Ptr& input_pointcloud, const double voxel_size, const double robot_radius)
  : octree_voxel_size_(voxel_size)
  , robot_radius_(robot_radius)
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

const std::vector<Point> OctreeMap3D::findPath(const Point& start_point,
                                               const Point& goal_point,
                                               const int planner_type,
                                               const double max_planning_time,
                                               const bool interpolate_path,
                                               const double interpolation_step,
                                               const double min_height,
                                               const double turning_radius,
                                               const double max_z_slope
                                               )
{
  std::cout << "  Start point: " << start_point.x << "," << start_point.y << "," << start_point.z << std::endl;
  std::cout << "  Goal point: " << goal_point.x << "," << goal_point.y << "," << goal_point.z << std::endl;

  // Check if start and goal are within the PointCloud and not in collision
  checkStartAndGoalValidity(start_point, goal_point);

  //const double turning_radius = 1.0; // 158.0; // is this rho?
  //const double max_z_slope = 0.15;

  // Custom DubinsZ State Space has 3 sub-spaces, however OMPL sees it as
  // an SE2 space (R^2 and SO2):
  //   Compound state [
  //     RealVectorState [0 0 5]
  //     SO2State [-0.271153]
  //   ]
  state_space_.reset(new ompl_state_spaces::DubinsZStateSpace(turning_radius, max_z_slope));

  const int num_dimensions = 3;
  ompl::base::RealVectorBounds bounds(num_dimensions);
  bounds.setLow(0,  pointcloud_dims_.min_x);
  bounds.setHigh(0, pointcloud_dims_.max_x);
  bounds.setLow(1,  pointcloud_dims_.min_y);
  bounds.setHigh(1, pointcloud_dims_.max_y);
  bounds.setLow(2,  pointcloud_dims_.min_z);
  if (min_height > start_point.z)
  {
    bounds.setLow(2, min_height);
  }
  bounds.setHigh(2, pointcloud_dims_.max_z);
  state_space_->as<ompl_state_spaces::DubinsZStateSpace>()->SetBounds(bounds);
  state_space_->setup();

  space_information_ = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(state_space_));

  ompl::base::MotionValidatorPtr mvp(new ompl_state_spaces::DubinsZMotionValidator(space_information_));
  space_information_->setMotionValidator(mvp);

  // Collision-check a sphere of the robot's radius
  space_information_->setStateValidityChecker(boost::bind(&OctreeMap3D::isStateValid_CheckSphere, this, _1));

  // Set collision-checking resolution (one or half voxel size).
  // In OMPL, collision-checking defaults to 0.01 (1%) of the state's maximum extent.
  const double max_extent = std::max({pointcloud_dims_.x_range, pointcloud_dims_.y_range, pointcloud_dims_.z_range}); // C++11
  const double resolution_meters = octree_voxel_size_; // / 2.0;
  const double resolution_percent = 1.0 / (max_extent / resolution_meters);
  space_information_->setStateValidityCheckingResolution(resolution_percent); 
  std::cout << "OMPL collision-checking resolution: "
            << resolution_percent << "% = " << resolution_percent*max_extent << "m"
            << "  (default: 1% = " << 0.01*max_extent << "m)" << std::endl;

  space_information_->setup();

  const double angle_start_to_goal = angleBetweenStartAndGoal(start_point, goal_point);
  std::cout << "angle to goal: " << angle_start_to_goal << std::endl;

  ompl::base::ScopedState<ompl_state_spaces::DubinsZStateSpace> startc(state_space_);
  ompl::base::ScopedState<ompl_state_spaces::DubinsZStateSpace> goalc(state_space_);

  startc->GetTranslation().values[0] = start_point.x;
  startc->GetTranslation().values[1] = start_point.y;
  startc->GetTranslation().values[2] = start_point.z;
  if (min_height > start_point.z)
  {
    startc->GetTranslation().values[2] = min_height;
  }
  startc->GetRotation().value = angle_start_to_goal;

  goalc->GetTranslation().values[0] = goal_point.x;
  goalc->GetTranslation().values[1] = goal_point.y;
  goalc->GetTranslation().values[2] = goal_point.z;
  if (min_height > goal_point.z)
  {
    goalc->GetTranslation().values[2] = min_height;
  }
  goalc->GetRotation().value = angle_start_to_goal;

  ompl::base::ScopedState<> start(startc);
  ompl::base::ScopedState<> goal(goalc);

  std::cout << "start = " << start << std::endl;
  std::cout << "goal = " << goal << std::endl;

  problem_definition_ = ompl::base::ProblemDefinitionPtr(new ompl::base::ProblemDefinition(space_information_));
  problem_definition_->setStartAndGoalStates(start, goal);

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
  else if (planner_type == TASK_SPACE_RRT_STAR)
  {
    boost::shared_ptr<task_space_rrtstar::RRTstar> rrtstar = boost::shared_ptr<task_space_rrtstar::RRTstar>(new task_space_rrtstar::RRTstar(space_information_));
    rrtstar->setRange(std::numeric_limits<double>::max());
    ompl_state_spaces::TaskSpaceDubinsZMotionValidatorPtr ts_dubins(new ompl_state_spaces::TaskSpaceDubinsZMotionValidator(space_information_));
    rrtstar->set_task_space_sampling(true);
    rrtstar->set_task_space_mv(ts_dubins);

    ompl_planner_ = ompl::base::PlannerPtr(rrtstar);
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

    // ToDo: try these:
    //bitstar->setRewireFactor(3); //1.2
    //bitstar->setSamplesPerBatch(100); // was 30
    //bitstar->setPruning(true);
    //bitstar->setKNearest(true);
    //// bitstar->setStrictQueueOrdering(true);

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

  num_collision_checks_ = 0; // reset counter

  std::cout << "OMPL planning time limit = " << planning_time_limit << std::endl;

  ompl::base::PlannerStatus solved = ompl_planner_->solve(planning_time_limit);
  if (!solved)
  {
    std::cout << "ERROR: Failed to find path!" << std::endl;
    return std::vector<Point>();
  }

  path_length_ = problem_definition_->getSolutionPath()->length();
  boost::shared_ptr<ompl::geometric::PathGeometric> ompl_path = boost::static_pointer_cast<ompl::geometric::PathGeometric>(problem_definition_->getSolutionPath());

  // Print waypoints:
  //std::cout << "Solution: " << std::endl;
  //ompl_path->printAsMatrix(std::cout);

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

  // Print interpolated path waypoints:
  //std::cout << "Interpolated solution: " << std::endl;
  //ompl_path->printAsMatrix(std::cout);

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

  const ompl_state_spaces::DubinsZStateSpace::StateType* state_3d = state->as<ompl_state_spaces::DubinsZStateSpace::StateType>();
  const Point center_point(static_cast<float>(state_3d->GetTranslation().values[0]),
                           static_cast<float>(state_3d->GetTranslation().values[1]),
                           static_cast<float>(state_3d->GetTranslation().values[2]));

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

  const uint max_nn = 1; // 1 or more neighbours = collision

  if (octree_->radiusSearch(center_point, radius, nn_indices, nn_dists, max_nn))
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

    ompl_state_spaces::DubinsZStateSpace::StateType* state = 
        static_cast<ompl_state_spaces::DubinsZStateSpace::StateType*>(path_geometric_ptr->getState(i));
    const float x = state->GetTranslation().values[0];
    const float y = state->GetTranslation().values[1];
    const float z = state->GetTranslation().values[2];

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
      //const int num_waypoints = std::ceil(path_length_ / interpolation_step); // length has changed
      const int num_waypoints = std::ceil(ompl_path->length() / interpolation_step);
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

  //  maxSteps  The maximum number of attempts to "short-cut" the path. If this value is set to 0 (the default), 
  //            the number of attempts made is equal to the number of states in path.
  //  maxEmptySteps  Not all iterations of this function produce a simplification. If an iteration does not produce
  //                 a simplification, it is called an empty step. maxEmptySteps denotes the maximum number of consecutive empty
  //                 steps before the simplification process terminates. If this value is set to 0 (the default), the number of
  //                 attempts made is equal to the number of states in path.
  //  rangeRatio  The maximum distance between states a connection is attempted, as a fraction relative to the total
  //              number of states (between 0 and 1).
  ompl::geometric::PathSimplifier simplifier(space_information_);
  const unsigned int max_steps = 0;
  const unsigned int max_empty_steps = 0;
  const double range_ratio = 0.9;
  simplifier.reduceVertices(*ompl_path, max_steps, max_empty_steps, range_ratio);

  // Must interpolate the Dubins-Z path:
  // One waypoint per valid state.
  // The distance between waypoints thus depends on
  // OMPL's collision-checking resolution.
  ompl_path->interpolate();

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
    throw std::runtime_error("start point is in collision!");
  }

  if (isInCollisionSphere(start_point, robot_radius_))
  {
    throw std::runtime_error("goal point is in collision!");
  }
}
