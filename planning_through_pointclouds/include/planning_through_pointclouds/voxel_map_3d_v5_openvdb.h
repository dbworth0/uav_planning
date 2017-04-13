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

#ifndef VOXEL_MAP_3D_H
#define VOXEL_MAP_3D_H

#include <initializer_list>
#include <string>
#include <vector>

//#include <boost/function.hpp>

#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointXYZRGBA ColorPoint;
typedef pcl::PointCloud<ColorPoint> ColorPointCloud;


#include <openvdb/openvdb.h>



#define FREE_SPACE 1
#define OBSTACLE 2

// Index type for the Occupancy Grid.
// On a 64-bit system,
// size of int: 4 bytes, max value = 2147483647
// size of uint32_t: 4 bytes, max value = 4294967295
// size of uint16_t: 2 bytes, max value = 65535
// so 65,535 should be large enough to index the Occupancy Grid.
typedef uint16_t IndexType;

// Type for storing an index into the Occupancy Grid
struct Voxel
{
  IndexType x;
  IndexType y;
  IndexType z;
  Voxel() : x(0), y(0), z(0) {};
  Voxel(const IndexType _x, const IndexType _y, const IndexType _z) : x(_x), y(_y), z(_z) {};
};

typedef std::vector<Voxel> PathCoordinates;

enum PLANNER_TYPE
{
  RRT = 1,
  RRT_CONNECT,
  RRT_STAR,
  BIT_STAR
};
const PLANNER_TYPE stringToEnum(const std::string& name)
{
  if (name == "RRT") return RRT;
  if (name == "RRT_CONNECT") return RRT_CONNECT;
  if (name == "RRT_STAR") return RRT_STAR;
  if (name == "BIT_STAR") return BIT_STAR;
  throw std::runtime_error("Unknown name in stringToEnum()");
}

class VoxelMap3D
{
  public:
    // Initialize a discrete, 3D voxel map.
    // connectivity is 6 or 26.
    VoxelMap3D(const uint width, const uint length, const uint depth, const double voxel_size);
    // padded_occupancy_grid = If true, this will dilate the occupancy grid obstacles by robot_radius.
    VoxelMap3D(PointCloud::Ptr& input_pointcloud, const double voxel_size, const double robot_radius, const bool padded_occupancy_grid = true);

    ~VoxelMap3D() {};

    void addCuboidObstacle(const std::initializer_list<IndexType>& min_corner,
                           const std::initializer_list<IndexType>& max_corner);

    // Find a path using sampling-based planner.
    //
    // planner_type The planner to use: RRT
    //                                  RRT_CONNECT
    //                                  RRT_STAR
    //                                  BIT_STAR
    // interpolate_path If false, the output path contains only
    //                            the critical waypoints.
    //                  If true and step = 0, OMPL will interpolate the path so it contains
    //                                        one waypoint for each collision check.
    //                  If true and step > 0, OMPL will interpolate the path so it contains
    //                                        one waypoint every step distance.
    const std::vector<Point> findPath(const Point& start_point,
                                      const Point& goal_point,
                                      const int planner_type,
                                      const double max_planning_time,
                                      const bool interpolate_path = false,
                                      const double interpolation_step = 0.0);
    const std::vector<Point> findPath(const std::initializer_list<IndexType>& start_cell,
                                      const std::initializer_list<IndexType>& goal_cell,
                                      const int planner_type,
                                      const double max_planning_time,
                                      const bool interpolate_path = false,
                                      const double interpolation_step = 0.0);

    const uint getNumCollisionChecks() { return num_collision_checks_; };

    const double getPathLength() { return path_length_; };

    // Get the OMPL Planner data.
    // Can be used to extract the graph of states expanded by the planner.
    const ompl::base::PlannerDataPtr getPlannerData();

    // Simplify the OMPL path.
    const std::vector<Point> getSimplifiedPath(const double max_time = 30.0);

    // Get the 3D Occupancy Grid as a PointCloud.
    // point_separation is how far apart to separate each grid
    // position, for visualization purposes.
    const ColorPointCloud getOccupancyGrid3D();
    const ColorPointCloud getOccupancyGrid3D(const float point_separation);

    // These two are public methods used for benchmarking purposes:

    // Check if a single voxel in the occupancy grid is in collision (an obstacle).
    const bool isInCollision(const Voxel& voxel);

    // Check if any voxels contained within a sphere in the occupancy grid are obstacles.
    const bool isInCollisionSphere(const Point& center_point, const double radius);

  private:

    struct Bounds
    {
      double min_x;
      double max_x;
      double min_y;
      double max_y;
      double min_z;
      double max_z;
      double x_range;
      double y_range;
      double z_range;
      Bounds()
       : min_x(0.0), max_x(0.0)
       , min_y(0.0), max_y(0.0)
       , min_z(0.0), max_z(0.0)
       , x_range(0.0)
       , y_range(0.0)
       , z_range(0.0) {};
      Bounds(const double x0, const double x1,
             const double y0, const double y1,
             const double z0, const double z1)
       : min_x(x0), max_x(x1)
       , min_y(y0), max_y(y1)
       , min_z(z0), max_z(z1)
       , x_range(x1 - x0)
       , y_range(y1 - y0)
       , z_range(z1 - z0) {};
      Bounds(const double (&bounds)[6])
       : min_x(bounds[0]), max_x(bounds[1])
       , min_y(bounds[2]), max_y(bounds[3])
       , min_z(bounds[4]), max_z(bounds[5])
       , x_range(bounds[1] - bounds[0])
       , y_range(bounds[3] - bounds[2])
       , z_range(bounds[5] - bounds[4]) {};
      Bounds(const std::vector<float>& bounds)
       : min_x(static_cast<double>(bounds.at(0))), max_x(static_cast<double>(bounds.at(1)))
       , min_y(static_cast<double>(bounds.at(2))), max_y(static_cast<double>(bounds.at(3)))
       , min_z(static_cast<double>(bounds.at(4))), max_z(static_cast<double>(bounds.at(5)))
       , x_range(static_cast<double>(bounds.at(1)) - static_cast<double>(bounds.at(0)))
       , y_range(static_cast<double>(bounds.at(3)) - static_cast<double>(bounds.at(2)))
       , z_range(static_cast<double>(bounds.at(5)) - static_cast<double>(bounds.at(4))) {};
    };

    // If map is initialized from a PointCloud, we store
    // the x,y,z bounds of the cloud.
    Bounds pointcloud_dims_;

    // Occupancy Grid:
    // x,y,z,status
    // status 0 = un-initialized
    //        1 = free
    //        2 = obstacle
    // Vector (header info) is allocated on the stack but the elements on the heap
//    std::vector<std::vector<std::vector<std::vector<uint8_t> > > > occupancy_grid_3d_;


    // Create an empty OpenVDB Grid.
    // floating-point grid with background value = 0
    //
    // Using signed co-ordinates, the maximum size is
    // For 32-bit signed coordinates these are (-2,147,483,648, -2147483648, -2147483648)
    // and (2147483647, 2147483647, 2147483647)
    //
    // TODO: - Implement an integer or bitset grid.
    //       - Use un-signed co-ordinates.
    openvdb::FloatGrid::Ptr occupancy_grid_3d_;



    IndexType occupancy_grid_width_;
    IndexType occupancy_grid_length_;
    IndexType occupancy_grid_depth_;
    double occupancy_grid_voxel_size_;
    bool padded_occupancy_grid_;
    double robot_radius_;

    // Get the voxel index (i,j,k) in the Occupancy Grid
    // for a Point (x,y,z float values)
    const Voxel getVoxelIndex(const Point& point);

    // Get the x,y,z Point corresponding to a voxel in the Occupancy Grid
    const Point getPoint(const Voxel& index, const float point_separation);

    // Get the linear index (i) corresponding to a voxel in the Occupancy Grid.
    // The linear index is also the Vertex ID in the Boost graph.
    const uint getLinearIndex(const IndexType i, const IndexType j, const IndexType k);

    // Get the voxel index (i,j,k) in the Occupancy Grid for a linear index (i).
    // The linear index is also the Vertex ID in the Boost graph.
    const Voxel getCoordinates(const uint index);

    // For an x,y,z point in a PointCloud, get the indices of a voxelized 
    // bounding cube of some radius from the center point.
    // This only returns valid voxel indices within the Occupancy Grid.
    const PathCoordinates getVoxelCubeIndices(const Point& center_point, const double radius);

    // For a Voxel in the Occupancy Grid, get the indices of a voxelized 
    // bounding cube of some radius from the center point.
    // This only returns valid voxel indices within the Occupancy Grid.
    const PathCoordinates getVoxelCubeIndices(const Voxel& center_point_idx, const double radius);
    
    // Make sure the start and goal positions are contained within the
    // Occupancy Grid and not inside an obstacle.
    void checkStartAndGoalValidity(const std::vector<IndexType>& start_cell,
                                   const std::vector<IndexType>& goal_cell);

    // Create a optimization objective for OMPL planners like RRT* and BIT*.
    // if cost_threshold = 0.0, the planner will keep trying to minimize the path length until max_planning_time is reached.
    // if cost_threshold = max_double, the planner will return the first solution.
    ompl::base::OptimizationObjectivePtr getPathLengthObjectiveWithCostThreshold(const ompl::base::SpaceInformationPtr& space_information,
                                                                                 const double cost_threshold);

    // Check if an OMPL Planning State is valid.
    // This collision-checks a single voxel in the occupancy grid.
    const bool isStateValid(const ompl::base::State* state);

    // Check if an OMPL Planning State is valid.
    // This checks that a sphere is collision free.
    const bool isStateValid_CheckSphere(const ompl::base::State* state);

    // Extract the path from the OMPL planner solution.
    const std::vector<Point> getPathPoints(const boost::shared_ptr<ompl::geometric::PathGeometric> path_geometric_ptr);

    ompl::base::StateSpacePtr state_space_;
    ompl::base::SpaceInformationPtr space_information_;
    ompl::base::ProblemDefinitionPtr problem_definition_;
    ompl::base::PlannerPtr ompl_planner_;

    uint num_collision_checks_;
    double path_length_;

}; // end class VoxelMap3D

#endif // VOXEL_MAP_3D_H
