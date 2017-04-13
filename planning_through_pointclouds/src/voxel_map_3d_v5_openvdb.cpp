//
// Voxel Map 3D v5
//
// A 3D occupancy grid, stored in an OpenVDB data structure.








// Planning using RRT variants from OMPL (Open Motion Planning Libray)
// in a RealVectorStateSpace (x,y,z position).
//
// There are 2 options for collision-checking:
//    1. The input PointCloud is expanded by the radius of the robot, and
//       a state is considered "valid" if the voxel at that position is free.
//    2. No obstacle expansion. A state is considered "valid" if
//       all voxels within the robot's radius are free.
// The planner interpolates between states and makes sure all voxels
// in-between are also free.
// 
// David Butterworth
//

#include <planning_through_pointclouds/voxel_map_3d_v5_openvdb.h>

#include <iostream> // cout
#include <queue> // constant-time lookup of largest element
#include <algorithm> // std::reverse

#include <pcl/common/distances.h> // euclideanDistance()

#include <planning_through_pointclouds/utils.h> // range, printStdVector
#include <planning_through_pointclouds/pcl_utils.h> // loadPointCloud, downsamplePointCloud

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
//#include <ompl/geometric/planners/prm/PRM.h>

#include "ompl/base/objectives/PathLengthOptimizationObjective.h"

// BIT* is not available in OMPL distributed with Ubuntu, you must compile from trunk
#ifdef USE_BITSTAR
#include <ompl/geometric/planners/bitstar/BITstar.h>
#endif

VoxelMap3D::VoxelMap3D(const uint width, const uint length, const uint depth, const double voxel_size)
  : occupancy_grid_voxel_size_(voxel_size)
  , padded_occupancy_grid_(true) // To use the correct collision-checker, we assume a synthetic occupancy grid is already "padded"
  , robot_radius_(0.0)
{
  if ((width > std::numeric_limits<IndexType>::max()) || (length > std::numeric_limits<IndexType>::max()) || (depth > std::numeric_limits<IndexType>::max()))
  {
    throw std::runtime_error("Number of required voxels in one dimension exceeds size of IndexType");
  }

  occupancy_grid_width_ = static_cast<IndexType>(width);
  occupancy_grid_length_ = static_cast<IndexType>(length);
  occupancy_grid_depth_ = static_cast<IndexType>(depth);

  // Initialize the OpenVDB library.  This must be called at least
  // once per program and may safely be called multiple times.
  openvdb::initialize();

  // Initialize the floating-point grid with background value = 0
  occupancy_grid_3d_ = openvdb::FloatGrid::create();

  // Note: The grid uses zero memory until voxels are explicitly added

  // Get an accessor for coordinate-based access to voxels
  openvdb::FloatGrid::Accessor grid_accessor = occupancy_grid_3d_->getAccessor();

  const bool use_sparse_grid = false;

  if (use_sparse_grid == false)
  {
    // Initialize all voxels in the Grid to FREE_SPACE
    for (IndexType i = 0; i < occupancy_grid_width_; ++i)
    {
      for (IndexType j = 0; j < occupancy_grid_length_; ++j)
      {
        for (IndexType k = 0; k < occupancy_grid_depth_; ++k)
        {
          // Note: OpenVDB co-ordinates have signed indices of type int32_t
          openvdb::Coord xyz(static_cast<int32_t>(i),
                             static_cast<int32_t>(j),
                             static_cast<int32_t>(k));

          grid_accessor.setValue(xyz, static_cast<float>(FREE_SPACE));
        }
      }
    }
  }

  // These dimensions are used to set the range
  // of the OMPL planning state space
  pointcloud_dims_ = Bounds(0, occupancy_grid_width_ * occupancy_grid_voxel_size_,
                            0, occupancy_grid_length_ * occupancy_grid_voxel_size_,
                            0, occupancy_grid_depth_ * occupancy_grid_voxel_size_);
}

VoxelMap3D::VoxelMap3D(PointCloud::Ptr& input_pointcloud, const double voxel_size, const double robot_radius, const bool padded_occupancy_grid)
  : occupancy_grid_voxel_size_(voxel_size)
  , padded_occupancy_grid_(padded_occupancy_grid)
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

  const uint num_x_voxels = static_cast<uint>(std::ceil(pointcloud_dims_.x_range / occupancy_grid_voxel_size_));
  const uint num_y_voxels = static_cast<uint>(std::ceil(pointcloud_dims_.y_range / occupancy_grid_voxel_size_));
  const uint num_z_voxels = static_cast<uint>(std::ceil(pointcloud_dims_.z_range / occupancy_grid_voxel_size_));
  //std::cout << "  x range: " << pointcloud_dims_.x_range << std::endl;
  //std::cout << "  y range: " << pointcloud_dims_.y_range << std::endl;
  //std::cout << "  z range: " << pointcloud_dims_.z_range << std::endl;

  std::cout << "num_x_voxels: " << num_x_voxels << std::endl;
  std::cout << "num_y_voxels: " << num_y_voxels << std::endl;
  std::cout << "num_z_voxels: " << num_z_voxels << std::endl;

  if ((num_x_voxels+1 > std::numeric_limits<IndexType>::max()) || (num_y_voxels+1 > std::numeric_limits<IndexType>::max()) || (num_z_voxels+1 > std::numeric_limits<IndexType>::max()))
  {
    throw std::runtime_error("Number of required voxels in one dimension exceeds size of IndexType");
  }

  occupancy_grid_width_ = static_cast<IndexType>(num_x_voxels + 1); // +1 to account for rounding errors in range
  occupancy_grid_length_ = static_cast<IndexType>(num_y_voxels + 1);
  occupancy_grid_depth_ = static_cast<IndexType>(num_z_voxels + 1);

  // Initialize the OpenVDB library.  This must be called at least
  // once per program and may safely be called multiple times.
  openvdb::initialize();

  // Initialize the floating-point grid with background value = 0
  occupancy_grid_3d_ = openvdb::FloatGrid::create();

  // Note: The grid uses zero memory until voxels are explicitly added

  // Get an accessor for coordinate-based access to voxels
  openvdb::FloatGrid::Accessor grid_accessor = occupancy_grid_3d_->getAccessor();

  const bool use_sparse_grid = false;

  if (use_sparse_grid == false)
  {
    // Initialize all voxels in the Grid to FREE_SPACE
    for (IndexType i = 0; i < occupancy_grid_width_; ++i)
    {
      for (IndexType j = 0; j < occupancy_grid_length_; ++j)
      {
        for (IndexType k = 0; k < occupancy_grid_depth_; ++k)
        {
          // Note: OpenVDB co-ordinates have signed indices of type int32_t
          openvdb::Coord xyz(static_cast<int32_t>(i),
                             static_cast<int32_t>(j),
                             static_cast<int32_t>(k));

          grid_accessor.setValue(xyz, static_cast<float>(FREE_SPACE));
        }
      }
    }
  }



  // Why down-sample?  I think it was faster than iterating over all points.


  std::cout << "Downsampling PointCloud... " << std::endl;
  // ToDo: Write my own filter that makes points evenly distributed.
  // Note: The resulting points are not within the center of the voxel grid
  PointCloud::Ptr downsampled_pointcloud(new PointCloud());
  downsamplePointCloud<Point>(input_pointcloud, occupancy_grid_voxel_size_, downsampled_pointcloud);
  std::cout << "down-sampled cloud has " << downsampled_pointcloud->points.size() << " points" << std::endl;


  // TODO:
  // Iterate over the VDB grid and check if num points == num voxels


  std::cout << "Generating Occupancy Grid from PointCloud... " << std::endl;
  for (PointCloud::iterator it = downsampled_pointcloud->begin(); it != downsampled_pointcloud->end(); ++it)
  //for (PointCloud::iterator it = input_pointcloud->begin(); it != input_pointcloud->end(); ++it)
  {
    const Voxel voxel_idx = getVoxelIndex(*it);

    // ToDo: Check voxel index is within grid before trying to add it

    // Note: OpenVDB co-ordinates have signed indices of type int32_t
    openvdb::Coord xyz(static_cast<int32_t>(voxel_idx.x),
                       static_cast<int32_t>(voxel_idx.y),
                       static_cast<int32_t>(voxel_idx.z));

    grid_accessor.setValue(xyz, static_cast<float>(OBSTACLE));

    //std::cout << "Setting voxel "
    //          << static_cast<int32_t>(voxel_idx.x) << ", " << static_cast<int32_t>(voxel_idx.y) << ", " << static_cast<int32_t>(voxel_idx.z)
    //          << "  to  " << static_cast<float>(OBSTACLE) << std::endl;


    /*
    // Apply padding to this point, by turning on
    // all voxels enclosed within some radius
    if (padded_occupancy_grid_)
    {
      // Get the voxel indices that make up a cube.
      // (this only returns the valid indices inside the voxel grid)
      const PathCoordinates cube_indices = getVoxelCubeIndices(voxel_idx, robot_radius_);

      for (size_t i = 0; i < cube_indices.size(); ++i)
      {
        const Point point = getPoint(cube_indices.at(i), occupancy_grid_voxel_size_);
        const float dist = pcl::euclideanDistance(*it, point);
        if (dist <= robot_radius_)
        {
          //occupancy_grid_3d_.at(cube_indices.at(i).x).at(cube_indices.at(i).y).at(cube_indices.at(i).z).at(0) = OBSTACLE;
        }
      }
    }
    */
  }
  

  downsampled_pointcloud.reset();


  std::cout << "Done." << std::endl;
}

void VoxelMap3D::addCuboidObstacle(const std::initializer_list<IndexType>& min_corner, const std::initializer_list<IndexType>& max_corner)
{
  /*
  const std::vector<IndexType> min_corner_vec(min_corner);
  const std::vector<IndexType> max_corner_vec(max_corner);

  if ((max_corner_vec.at(0) < min_corner_vec.at(0))
      || (max_corner_vec.at(1) < min_corner_vec.at(1))
      || (max_corner_vec.at(2) < min_corner_vec.at(2)))
  {
    throw std::runtime_error("max indices should be larger than min indices");
  }

  for (IndexType i = min_corner_vec.at(0); i <= max_corner_vec.at(0); ++i)
  {
    for (IndexType j = min_corner_vec.at(1); j <= max_corner_vec.at(1); ++j)
    {
      for (IndexType k = min_corner_vec.at(2); k <= max_corner_vec.at(2); ++k)
      {
        if ((i < 0) || (i > occupancy_grid_width_-1)
             || (j < 0) || (j > occupancy_grid_length_-1)
             || (k < 0) || (k > occupancy_grid_depth_-1))
        {
          // Ignore voxels outside the Occupancy Grid
          continue;
        }

        occupancy_grid_3d_.at(i).at(j).at(k).at(0) = OBSTACLE;
      }
    }
  }
  */
}

const std::vector<Point> VoxelMap3D::findPath(const std::initializer_list<IndexType>& start_cell,
                                              const std::initializer_list<IndexType>& goal_cell,
                                              const int planner_type,
                                              const double max_planning_time,
                                              const bool interpolate_path,
                                              const double interpolation_step)
{
  const std::vector<IndexType> start_cell_vec(start_cell);
  const std::vector<IndexType> goal_cell_vec(goal_cell);

  const Point start_point(start_cell_vec.at(0) * occupancy_grid_voxel_size_,
                          start_cell_vec.at(1) * occupancy_grid_voxel_size_,
                          start_cell_vec.at(2) * occupancy_grid_voxel_size_);
  const Point goal_point(goal_cell_vec.at(0) * occupancy_grid_voxel_size_,
                         goal_cell_vec.at(1) * occupancy_grid_voxel_size_,
                         goal_cell_vec.at(2) * occupancy_grid_voxel_size_);

  return findPath(start_point, goal_point, planner_type, max_planning_time, interpolate_path);
}

const std::vector<Point> VoxelMap3D::findPath(const Point& start_point,
                                              const Point& goal_point,
                                              const int planner_type,
                                              const double max_planning_time,
                                              const bool interpolate_path,
                                              const double interpolation_step)
{
  /*
  const Voxel start_cell = getVoxelIndex(start_point);
  const Voxel goal_cell = getVoxelIndex(goal_point);

  std::cout << "  Start point: " << start_point.x << "," << start_point.y << "," << start_point.z
            << "  Voxel: (" << start_cell.x << "," << start_cell.y << "," << start_cell.z << ")" << std::endl;
  std::cout << "  Goal point: " << goal_point.x << "," << goal_point.y << "," << goal_point.z
            << "  Voxel: (" << goal_cell.x << "," << goal_cell.y<< "," << goal_cell.z << ")" << std::endl;

  // Check if start and goal are within the occupancy grid
  // and not inside an obstacle.
  // ToDo: change this function's input type to Voxel
  const std::vector<IndexType> start_cell_vec = {start_cell.x, start_cell.y, start_cell.z};
  const std::vector<IndexType> goal_cell_vec = {goal_cell.x, goal_cell.y, goal_cell.z};
  checkStartAndGoalValidity(start_cell_vec, goal_cell_vec);

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
  state_space_->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
  state_space_->setup();

  space_information_ = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(state_space_));

  if (padded_occupancy_grid_)
  {
    // By default we use a padded occupancy grid, so the State Validity Checker
    // only needs to check if a single voxel is not in collision
    space_information_->setStateValidityChecker(boost::bind(&VoxelMap3D::isStateValid, this, _1));
  }
  else
  {
    // Alternatively, collision-check a sphere of the robot's radius
    space_information_->setStateValidityChecker(boost::bind(&VoxelMap3D::isStateValid_CheckSphere, this, _1));
  }
  
  // Set collision-checking resolution (one or half voxel size).
  // In OMPL, collision-checking defaults to 0.01 (1%) of the state's maximum extent.
  const double max_extent = std::max({pointcloud_dims_.x_range, pointcloud_dims_.y_range, pointcloud_dims_.z_range}); // C++11
  const double resolution_meters = occupancy_grid_voxel_size_; // / 2.0;
  const double resolution_percent = 1.0 / (max_extent / resolution_meters);
  space_information_->setStateValidityCheckingResolution(resolution_percent); 
  std::cout << "OMPL collision-checking resolution: "
            << resolution_percent << "% = " << resolution_percent*max_extent << "m"
            << "  (default: 1% = " << 0.01*max_extent << "m)" << std::endl;

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

  problem_definition_ = ompl::base::ProblemDefinitionPtr(new ompl::base::ProblemDefinition(space_information_));
  problem_definition_->setStartAndGoalStates(start, goal);

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

  if (interpolate_path)
  {
    if (interpolation_step > 0.00001)
    {
      const int num_waypoints = std::ceil(path_length_ / interpolation_step);
      // OMPL will evenly distributed the waypoints along the path length
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

  */

  return std::vector<Point>();


}

ompl::base::OptimizationObjectivePtr VoxelMap3D::getPathLengthObjectiveWithCostThreshold(const ompl::base::SpaceInformationPtr& space_information,
                                                                                         const double cost_threshold)
{
  ompl::base::OptimizationObjectivePtr objective(new ompl::base::PathLengthOptimizationObjective(space_information));
  objective->setCostThreshold(ompl::base::Cost(cost_threshold));
  return objective;
}

// ToDo: Should check if state isvalid.
//       Also, OMPL expects this function to be thread save but isInCollision() is not.
const bool VoxelMap3D::isStateValid(const ompl::base::State* state)
{
  num_collision_checks_++;

  const ompl::base::RealVectorStateSpace::StateType* state_3d = state->as<ompl::base::RealVectorStateSpace::StateType>();
  const Voxel voxel = getVoxelIndex(Point(static_cast<float>(state_3d->values[0]),
                                          static_cast<float>(state_3d->values[1]),
                                          static_cast<float>(state_3d->values[2])));
  if (isInCollision(voxel))
  {
    // Voxel is obstacle, therefore state is not valid
    return false;
  }

  return true;
}

const bool VoxelMap3D::isInCollision(const Voxel& voxel)
{
  /*
  if ((voxel.x < 0) || (voxel.x > occupancy_grid_width_-1)
       || (voxel.y < 0) || (voxel.y > occupancy_grid_length_-1)
       || (voxel.z < 0) || (voxel.z > occupancy_grid_depth_-1))
  {
    // Outside range of occupancy grid
    return true;
  }

  if (occupancy_grid_3d_.at(voxel.x).at(voxel.y).at(voxel.z).at(0) == OBSTACLE)
  {
    return true;
  }

  return false;
  */

  return false;
}

const bool VoxelMap3D::isStateValid_CheckSphere(const ompl::base::State* state)
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

const bool VoxelMap3D::isInCollisionSphere(const Point& center_point, const double radius)
{
  
  // Get the voxel index of the center point
  const Voxel center_point_idx = getVoxelIndex(center_point);

  // Calculate the radius in voxels
  // e.g. for r = 0.32, with voxel size = 0.1
  //      ==> +/- 4 voxels from the center
  const IndexType radius_voxels = static_cast<IndexType>(std::ceil(radius / occupancy_grid_voxel_size_));

  // Get the x,y,z voxel indices for a bounding cube.
  // We need the indices to be signed integer because for index 0,
  // the range() function should be a range of negative to positive values.
  // The invalid indices (outside the Occupancy Grid) are ignored later.
  const std::vector<int> x_indices = range<int>(center_point_idx.x - radius_voxels, center_point_idx.x + radius_voxels);
  const std::vector<int> y_indices = range<int>(center_point_idx.y - radius_voxels, center_point_idx.y + radius_voxels);
  const std::vector<int> z_indices = range<int>(center_point_idx.z - radius_voxels, center_point_idx.z + radius_voxels);

  if ((x_indices.size() != y_indices.size()) || (x_indices.size() != z_indices.size()))
  {
    throw std::runtime_error("x, y and z indices are not same length in isInCollisionSphere()");
  }


  // Get an accessor for coordinate-based access to voxels
  openvdb::FloatGrid::Accessor grid_accessor = occupancy_grid_3d_->getAccessor();





  PathCoordinates voxel_indices;
  for (size_t i = 0; i < x_indices.size(); ++i)
  {
    for (size_t j = 0; j < y_indices.size(); ++j)
    {
      for (size_t k = 0; k < z_indices.size(); ++k)
      {
        const IndexType x = static_cast<IndexType>(x_indices.at(i));
        const IndexType y = static_cast<IndexType>(y_indices.at(j));
        const IndexType z = static_cast<IndexType>(z_indices.at(k));

        // Make sure voxel is within the Occupancy Grid
        if ((x >= 0) && (x < occupancy_grid_width_)
             && (y >= 0) && (y < occupancy_grid_length_)
             && (z >= 0) && (z < occupancy_grid_depth_))
        {
          const Point point = getPoint(Voxel(x,y,z), occupancy_grid_voxel_size_);
          if (pcl::euclideanDistance(point, center_point) < radius)
          {


            // Note: OpenVDB co-ordinates have signed indices of type int32_t
            openvdb::Coord xyz(static_cast<int32_t>(x),
                               static_cast<int32_t>(y),
                               static_cast<int32_t>(z));

            //if (occupancy_grid_3d_.at(x).at(y).at(z).at(0) == OBSTACLE)

            //std::cout << "  value of voxel idx " << x << ", " << y << ", " << z
            //          << "  =  " << grid_accessor.getValue(xyz) << std::endl;

            // TODO: Change this so we're not comparing a float!
            if (grid_accessor.getValue(xyz) == static_cast<float>(OBSTACLE))
            {
              // Collision detected, there's at least
              // one point contained with the sphere
              return true;
            }
          }
        }
      }
    }
  }

  return false;

}

const ompl::base::PlannerDataPtr VoxelMap3D::getPlannerData()
{
  const ompl::base::PlannerDataPtr planner_data(new ompl::base::PlannerData(space_information_));
  ompl_planner_->getPlannerData(*planner_data);
  return planner_data;
}

const std::vector<Point> VoxelMap3D::getPathPoints(const boost::shared_ptr<ompl::geometric::PathGeometric> path_geometric_ptr)
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

const std::vector<Point> VoxelMap3D::getSimplifiedPath(const double max_time)
{
  boost::shared_ptr<ompl::geometric::PathGeometric> ompl_path = boost::static_pointer_cast<ompl::geometric::PathGeometric>(problem_definition_->getSolutionPath());
  if (!ompl_path)
  {
    throw std::runtime_error("Path is empty in getSimplifiedPath()");
  }

  ompl::geometric::PathSimplifier simplifier(space_information_);
  simplifier.simplify(*ompl_path, max_time);

  // short-cut
  //path->interpolate(100);
  //simplifier.reduceVertices(*ompl_path);

  return getPathPoints(ompl_path);
}

void VoxelMap3D::checkStartAndGoalValidity(const std::vector<IndexType>& start_cell, const std::vector<IndexType>& goal_cell)
{
  /*
  if ((start_cell.at(0) < 0) || (start_cell.at(0) > occupancy_grid_width_-1)
       && (start_cell.at(1) < 0) || (start_cell.at(1) > occupancy_grid_length_-1)
       && (start_cell.at(2) < 0) || (start_cell.at(2) > occupancy_grid_depth_-1))
  {
    throw std::runtime_error("start cell is outside Occupancy Grid");
  }

  if ((goal_cell.at(0) < 0) || (goal_cell.at(0) > occupancy_grid_width_-1)
       && (goal_cell.at(1) < 0) || (goal_cell.at(1) > occupancy_grid_length_-1)
       && (goal_cell.at(2) < 0) || (goal_cell.at(2) > occupancy_grid_depth_-1))
  {
    throw std::runtime_error("goal cell is outside Occupancy Grid");
  }

  if (padded_occupancy_grid_)
  {
    if (occupancy_grid_3d_.at(start_cell.at(0)).at(start_cell.at(1)).at(start_cell.at(2)).at(0) == OBSTACLE)
    {
      throw std::runtime_error("start cell is in obstacle!");
    }

    if (occupancy_grid_3d_.at(goal_cell.at(0)).at(goal_cell.at(1)).at(goal_cell.at(2)).at(0) == OBSTACLE)
    {
      throw std::runtime_error("goal cell is in obstacle!");
    }
  }
  else
  {
    const Point start_point = getPoint(Voxel(start_cell.at(0), start_cell.at(1), start_cell.at(2)), occupancy_grid_voxel_size_);
    if (isInCollisionSphere(start_point, robot_radius_))
    {
      throw std::runtime_error("start is in collision!");
    }
    const Point goal_point = getPoint(Voxel(goal_cell.at(0), goal_cell.at(1), goal_cell.at(2)), occupancy_grid_voxel_size_);
    if (isInCollisionSphere(start_point, robot_radius_))
    {
      throw std::runtime_error("goal is in collision!");
    }
  }
  */
}

const ColorPointCloud VoxelMap3D::getOccupancyGrid3D()
{
  return getOccupancyGrid3D(occupancy_grid_voxel_size_);
}

const ColorPointCloud VoxelMap3D::getOccupancyGrid3D(const float point_separation)
{
  ColorPointCloud cloud;

/*
  for (IndexType i = 0; i < occupancy_grid_width_; ++i)
  {
    for (IndexType j = 0; j < occupancy_grid_length_; ++j)
    {
      for (IndexType k = 0; k < occupancy_grid_depth_; ++k)
      {
        if (occupancy_grid_3d_.at(i).at(j).at(k).at(0) == OBSTACLE)
        {
          const Point point = getPoint(Voxel(i, j, k), point_separation);

          ColorPoint color_point;
          color_point.x = point.x;
          color_point.y = point.y;
          color_point.z = point.z;
          // brown = 90,39,41
          color_point.r = 90; // uint_8
          color_point.g = 39;
          color_point.b = 41;
          color_point.a = 255;

          cloud.points.push_back(color_point);
        }
      }
    }
  }
*/

  return cloud;
}

const Voxel VoxelMap3D::getVoxelIndex(const Point& point)
{
  const IndexType i = static_cast<IndexType>(std::round((point.x - pointcloud_dims_.min_x) / occupancy_grid_voxel_size_));
  const IndexType j = static_cast<IndexType>(std::round((point.y - pointcloud_dims_.min_y) / occupancy_grid_voxel_size_));
  const IndexType k = static_cast<IndexType>(std::round((point.z - pointcloud_dims_.min_z) / occupancy_grid_voxel_size_));
  return Voxel(i, j, k);
}

const Point VoxelMap3D::getPoint(const Voxel& index, const float point_separation)
{
  Point point;
  point.x = pointcloud_dims_.min_x + static_cast<float>(index.x) * point_separation;
  point.y = pointcloud_dims_.min_y + static_cast<float>(index.y) * point_separation;
  point.z = pointcloud_dims_.min_z + static_cast<float>(index.z) * point_separation;
  return point;
}

const uint VoxelMap3D::getLinearIndex(const IndexType i, const IndexType j, const IndexType k)
{
  const uint i_step = static_cast<uint>(occupancy_grid_width_);
  const uint j_step = static_cast<uint>(occupancy_grid_length_);

  return static_cast<uint>(i) + i_step*(static_cast<uint>(j) + j_step*(static_cast<uint>(k)));
}

const Voxel VoxelMap3D::getCoordinates(const uint index)
{
  const uint i_step = static_cast<uint>(occupancy_grid_width_);
  const uint j_step = static_cast<uint>(occupancy_grid_length_);

  const IndexType x = static_cast<IndexType>(index % i_step); // modulus (remainder)
  const IndexType y = static_cast<IndexType>((index - x) / i_step % j_step);
  const IndexType z = static_cast<IndexType>(((index - x) / i_step - y) / j_step);

  return Voxel(x, y, z);
}

const PathCoordinates VoxelMap3D::getVoxelCubeIndices(const Voxel& center_point_idx, const double radius)
{
  // Calculate the radius in voxels
  // e.g. for r = 0.32, with voxel size = 0.1
  //      ==> +/- 4 voxels from the center
  const IndexType radius_voxels = static_cast<IndexType>(std::ceil(radius / occupancy_grid_voxel_size_));

  // Get the x,y,z voxel indices for a bounding cube.
  // We need the indices to be signed integer because for index 0,
  // the range() function should be a range of negative to positive values.
  // The invalid indices (outside the Occupancy Grid) are ignored later.
  const std::vector<int> x_indices = range<int>(center_point_idx.x - radius_voxels, center_point_idx.x + radius_voxels);
  const std::vector<int> y_indices = range<int>(center_point_idx.y - radius_voxels, center_point_idx.y + radius_voxels);
  const std::vector<int> z_indices = range<int>(center_point_idx.z - radius_voxels, center_point_idx.z + radius_voxels);

  if ((x_indices.size() != y_indices.size()) || (x_indices.size() != z_indices.size()))
  {
    throw std::runtime_error("x, y and z indices are not same length in getVoxelCubeIndices()");
  }

  PathCoordinates voxel_indices;
  for (size_t i = 0; i < x_indices.size(); ++i)
  {
    for (size_t j = 0; j < y_indices.size(); ++j)
    {
      for (size_t k = 0; k < z_indices.size(); ++k)
      {
        const IndexType x = static_cast<IndexType>(x_indices.at(i));
        const IndexType y = static_cast<IndexType>(y_indices.at(j));
        const IndexType z = static_cast<IndexType>(z_indices.at(k));

        // Only return valid indices, not the entire cube
        if ((x >= 0) && (x < occupancy_grid_width_)
             && (y >= 0) && (y < occupancy_grid_length_)
             && (z >= 0) && (z < occupancy_grid_depth_))
        {
          voxel_indices.push_back(Voxel(x, y, z));
        }
      }
    }
  }

  return voxel_indices;
}

const PathCoordinates VoxelMap3D::getVoxelCubeIndices(const Point& center_point, const double radius)
{
  // Get the voxel index of the center point
  const Voxel center_point_idx = getVoxelIndex(center_point);

  return getVoxelCubeIndices(center_point_idx, radius);
}
