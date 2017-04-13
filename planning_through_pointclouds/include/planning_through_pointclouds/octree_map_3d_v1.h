//
// Octree Map 3D v1
//
// Stores a PointCloud in an Octree.
// Planning using Dijkstra or A*.
// 
// Uses discrete search on an implicit graph.
// The only information about the vertices is a set of "distances", "predecessors", "visited" and a queue.
// The edges are found online by collision-checking the occupancy grid and motion is restricted to a lattice.
// Search speed is increased by using a fixed-size "visited" array, which reduces the number of collision-checks.
// 
// David Butterworth
//

// ToDo: Make cost relative to voxel size

#ifndef OCTREE_MAP_3D_H
#define OCTREE_MAP_3D_H

#include <initializer_list>
#include <string>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointXYZRGBA ColorPoint;
typedef pcl::PointCloud<ColorPoint> ColorPointCloud;
#include <pcl/octree/octree.h>

// On a 64-bit system,
// size of int: 4 bytes, max value = 2147483647
// size of uint32_t: 4 bytes, max value = 4294967295
// size of uint16_t: 2 bytes, max value = 65535
// so 65,535 should be large enough to index the Occupancy Grid.
typedef uint16_t IndexType;

// ToDo: rename this
// This is type for storing an index in the implicit lattice
struct Voxel
{
  IndexType x;
  IndexType y;
  IndexType z;
  Voxel() : x(0), y(0), z(0) {};
  Voxel(const IndexType _x, const IndexType _y, const IndexType _z) : x(_x), y(_y), z(_z) {};
};

typedef std::vector<Voxel> PathCoordinates; 

class OctreeMap3D
{
  public:
    // Initialize a 3D Octree Map from a PointCloud.
    // connectivity is 6 or 26.
    OctreeMap3D(PointCloud::Ptr& input_pointcloud, const double voxel_size, const double collision_radius, const uint connectivity);

    ~OctreeMap3D() {};

    // Calculate shortest path from [x,y] to [x,y]
    // Uses Dijkstra or A*.
    // x,y are cell indices, indexed from zero.
    const PathCoordinates getShortestPath(const std::initializer_list<IndexType>& start_cell,
                                          const std::initializer_list<IndexType>& goal_cell,
                                          const bool use_astar = false);
    const PathCoordinates getShortestPath(const Point& start_point,
                                          const Point& goal_point,
                                          const bool use_astar = false);

    // Make sure the start and goal positions are contained within the
    // Occupancy Grid and not inside an obstacle.
    void checkStartAndGoalValidity(const std::vector<IndexType>& start_cell,
                                   const std::vector<IndexType>& goal_cell);

    // Get the path as a set of x,y,z points, to the same scale
    // as the input PointCloud.
    const std::vector<ColorPoint> getPathPoints(const PathCoordinates& positions);

    // Get the path as a PointCloud.
    const ColorPointCloud getPathPointCloud(const PathCoordinates& positions);

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

    // For collision-checking, this radius
    // is half the diameter of our robot
    double collision_radius_;


    // ToDo: rename these, they are the dimensions of the implicit lattice
    IndexType occupancy_grid_width_;
    IndexType occupancy_grid_length_;
    IndexType occupancy_grid_depth_;
    double occupancy_grid_voxel_size_;


    // Get the voxel index (i,j,k) in the Occupancy Grid
    // for a Point (x,y,z float values)
    const Voxel getVoxelIndex(const Point& point);

    // Get the x,y,z Point corresponding to a voxel in the Occupancy Grid
    //const Point getPoint(const Voxel& index, const float point_separation);
    const Point getPoint(const Voxel& index);

    // Get the linear index (i) corresponding to a voxel in the Occupancy Grid.
    // The linear index is also the Vertex ID in the Boost graph.
    const uint getLinearIndex(const IndexType i, const IndexType j, const IndexType k);

    // Get the voxel index (i,j,k) in the Occupancy Grid for a linear index (i).
    // The linear index is also the Vertex ID in the Boost graph.
    const Voxel getCoordinates(const uint index);

    // Using a "visited" (closed) list is slower
    // if we have already created all the edges.
    // However, with implicit edges where we check
    // edge validity by collision-checking as we go,
    // using a "visited" list is faster overall.
    std::vector<bool> visited_; // the "closed" set

    std::vector<uint> distances_;
    std::vector<uint> predecessors_;

    typedef float Weight;

    struct Edge
    {
      uint vertex;
      Weight weight;
      Edge(const uint _vertex, const float _weight) : vertex(_vertex), weight(_weight) {}
    };

    struct Vertex
    {
      uint vertex;
      float weight;
      Vertex(const uint _vertex, const float _weight) : vertex(_vertex), weight(_weight) {}
    };

    class prioritize
    {
      public: bool operator ()(const Vertex& v0, const Vertex& v1)
      {
        return v0.weight > v1.weight;
      }
    };

    uint graph_connectivity_;

    // Get the neighbouring vertices for a specific vertex.
    //
    // Returns a list of valid edges, meaning those
    // which are not in collision and each comprising
    // the destination vertex and weight.
    const std::vector<Edge> getNeighbours(const uint vertex_ID);

    // Calculate the Euclidean distance between two vertices
    const float euclideanDistance(const uint v0, const uint v1);


    // ToDo: isInCollision()

    // Check for collision within a sphere of the robot's diameter
    const bool checkCollision(const Point& point);

}; // end class OctreeMap3D

//----------------------------------------------------------------------------//

// Print a std::vector<Voxel>
void printPath(const PathCoordinates& path);

//----------------------------------------------------------------------------//

#endif // OCTREE_MAP_3D_H
