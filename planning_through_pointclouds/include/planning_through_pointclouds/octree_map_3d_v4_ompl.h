//
// Octree Map 3D v4
//
// Similar to v2, but removes parabolic smoother.
// Path smoothing is done with an external module.
// This adds an optional height constraint, so the z values
// of the path will be between a min and max range.
//
// Stores a PointCloud in a PCL Octree.
// Planning using RRT variants from OMPL (Open Motion Planning Libray)
// in a RealVectorStateSpace (x,y,z position).
//
// During planning, a state is considered "valid" if all points within
// the robot's radius are free. The planner interpolates between
// states and makes sure the space in-between is also free.
// 
// David Butterworth
//

#ifndef OCTREE_MAP_3D_H
#define OCTREE_MAP_3D_H

#include <initializer_list>
#include <string>
#include <vector>

//#include <boost/function.hpp>

#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/Planner.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>

//#include <ompl/base/ValidStateSampler.h>
#include <ompl/base/StateSampler.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointXYZRGBA ColorPoint;
typedef pcl::PointCloud<ColorPoint> ColorPointCloud;
#include <pcl/octree/octree.h>

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

class OctreeMap3D
{
  public:

    // Initialize Octree Map
    OctreeMap3D(PointCloud::Ptr& input_pointcloud, const double voxel_size, const double robot_radius);

    ~OctreeMap3D() {};

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
                                      const double min_height,
                                      const int planner_type,
                                      const double max_planning_time,
                                      const bool interpolate_path = false,
                                      const double interpolation_step = 0.0);

    const uint getNumCollisionChecks() { return num_collision_checks_; };

    const double getPathLength() { return path_length_; };

    // Get the OMPL Planner data.
    // Can be used to extract the graph of states expanded by the planner.
    const ompl::base::PlannerDataPtr getPlannerData();

    // Short-cut the path using OMPL Simplifier.
    const std::vector<Point> getSimplifiedPath(const double max_time = 30.0,
                                               const bool interpolate_path = false,
                                               const double interpolation_step = 0.10);

    // Simplify path by removing vertices
    const std::vector<Point> getReducedPath();

    // Check if any voxels contained within a sphere in the occupancy grid are obstacles.
    const bool isInCollisionSphere(const Point& center_point, const double radius);

    // Check if a path is in collision
    const bool isPathInCollision(std::vector<Point>& path);

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

    pcl::octree::OctreePointCloudSearch<Point>::Ptr octree_;
    float octree_voxel_size_;

    double robot_radius_;

    ompl::base::StateSpacePtr state_space_;
    ompl::base::SpaceInformationPtr space_information_;
    ompl::base::ProblemDefinitionPtr problem_definition_;
    ompl::base::PlannerPtr ompl_planner_;

    uint num_collision_checks_;
    double path_length_;

    // Make sure the start and goal positions are contained within the
    // extents of the input PointCloud and are not in collision.
    void checkStartAndGoalValidity(const Point& start_point, const Point& goal_point);

    // Create a optimization objective for OMPL planners like RRT* and BIT*.
    // if cost_threshold = 0.0, the planner will keep trying to minimize the path length until max_planning_time is reached.
    // if cost_threshold = max_double, the planner will return the first solution.
    ompl::base::OptimizationObjectivePtr getPathLengthObjectiveWithCostThreshold(const ompl::base::SpaceInformationPtr& space_information,
                                                                                 const double cost_threshold);

    // Check if an OMPL Planning State is valid.
    // This checks that a sphere is collision free.
    const bool isStateValid_CheckSphere(const ompl::base::State* state);

    // Extract the path from the OMPL planner solution.
    const std::vector<Point> getPathPoints(const boost::shared_ptr<ompl::geometric::PathGeometric> path_geometric_ptr);

}; // end class OctreeMap3D

//----------------------------------------------------------------------------//

// Experimenting with a custom State Space Sampler
class StateSamplerUniform : public ompl::base::StateSampler
{
  public:
    StateSamplerUniform(const ompl::base::StateSpace* space)
     : ompl::base::StateSampler(space)
     , space_(space)
     , attempts_(100) // MAX_VALID_SAMPLE_ATTEMPTS
    {
      std::cout << "Using custom StateSamplerUniform()" << std::endl;
    }

    virtual void sampleUniform(ompl::base::State* state)
    {
      const unsigned int dim = space_->getDimension();

      const ompl::base::RealVectorBounds& bounds = space_->as<ompl::base::RealVectorStateSpace>()->getBounds();

      ompl::base::RealVectorStateSpace::StateType* rstate = static_cast<ompl::base::RealVectorStateSpace::StateType *>(state);
      for (unsigned int i = 0; i < dim; ++i)
      {
        rstate->values[i] = rng_.uniformReal(bounds.low[i], bounds.high[i]);
      }
      //std::cout << "Sampled state: " << rstate->values[0] << ", " << rstate->values[1] << ", " << rstate->values[2] << std::endl;
    }

    virtual void sampleUniformNear(ompl::base::State* state, const ompl::base::State* near, const double distance)
    {
      throw ompl::Exception("StateSamplerUniform::sampleUniformNear", "not implemented");

      /*
      unsigned int attempts = 0;
      bool valid = false;
      do
      {
        //const ompl::base::StateSpacePtr state_space = si_->getStateSpace();

        const unsigned int dim = space_->getDimension();
        //const ompl::base::RealVectorBounds& bounds = static_cast<const ompl::base::RealVectorStateSpace *>(state_space)->getBounds();
        const ompl::base::RealVectorBounds& bounds = space_->as<ompl::base::RealVectorStateSpace>()->getBounds();

        ompl::base::RealVectorStateSpace::StateType* rstate = static_cast<ompl::base::RealVectorStateSpace::StateType *>(state);
        const ompl::base::RealVectorStateSpace::StateType* rnear = static_cast<const ompl::base::RealVectorStateSpace::StateType *>(near);
        for (unsigned int i = 0; i < dim; ++i)
        {
          rstate->values[i] = rng_.uniformReal(std::max(bounds.low[i], rnear->values[i] - distance),
                                               std::min(bounds.high[i], rnear->values[i] + distance));
        }
        
        valid = si_->isValid(state);

        ++attempts;
      }
      while (!valid && attempts < attempts_);

      return valid;
      */
    }

    virtual void sampleGaussian(ompl::base::State* state, const ompl::base::State* mean, const double stdDev)
    {
      throw ompl::Exception("StateSamplerUniform::sampleGaussian", "not implemented");
    }

  protected:
    ompl::base::StateSamplerPtr sampler_;
    const ompl::base::StateSpace* space_;
    const unsigned int attempts_;
    ompl::RNG rng_;
};

ompl::base::StateSamplerPtr allocateStateSamplerUniform(const ompl::base::StateSpace* space)
{
  return ompl::base::StateSamplerPtr(new StateSamplerUniform(space));
}

#endif // OCTREE_MAP_3D_H
