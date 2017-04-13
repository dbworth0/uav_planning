//
// Voxel Map 3D v1
//
// A 3D occupancy grid.
// Planning with Dijkstra or A* from BGL (Boost Graph Libray).
// 
// This version uses an occupancy grid stored as a multi-dimensional std::vector
// and planning is done on a Boost graph adjacency list, with vertex properties and edge coloring.
//
// David Butterworth
//

// ToDo: Make cost relative to voxel size

#ifndef VOXEL_MAP_3D_H
#define VOXEL_MAP_3D_H

#include <initializer_list>
#include <string>
#include <vector>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointXYZRGBA ColorPoint; // alpha used for visualizing A* states
typedef pcl::PointCloud<ColorPoint> ColorPointCloud;

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

class VoxelMap3D
{
  public:
    // Initialize a discrete, 3D voxel map.
    // connectivity is 6 or 26.
    VoxelMap3D(const uint width, const uint length, const uint depth, const double voxel_size, const uint connectivity);
    VoxelMap3D(PointCloud::Ptr& input_pointcloud, const double voxel_size, const double padding_radius, const uint connectivity);

    ~VoxelMap3D() {};

    void addGraphVertices();

    // Build the Boost graph from the Occupancy Grid, where
    // cells being FREE_SPACE are connected with edges.
    void buildGraphFromOccupancyGrid();

    void addCuboidObstacle(const std::initializer_list<IndexType>& min_corner,
                           const std::initializer_list<IndexType>& max_corner);

    // Calculate shortest path from [x,y] to [x,y]
    // using Dijkstra's algorithm.
    // x,y are cell indices, indexed from zero.
    const PathCoordinates getShortestPathDijkstra(const std::initializer_list<IndexType>& start_cell,
                                                  const std::initializer_list<IndexType>& goal_cell);
    const PathCoordinates getShortestPathDijkstra(const Point& start_point,
                                                  const Point& goal_point);

    // Calculate shortest path from [x,y] to [x,y]
    // using A* algorithm.
    // x,y are cell indices, indexed from zero.
    const PathCoordinates getShortestPathAstar(const std::initializer_list<IndexType>& start_cell,
                                               const std::initializer_list<IndexType>& goal_cell);
    const PathCoordinates getShortestPathAstar(const Point& start_point,
                                               const Point& goal_point);

    // Make sure the start and goal positions are contained within the
    // Occupancy Grid and not inside an obstacle.
    void checkStartAndGoalValidity(const std::vector<IndexType>& start_cell,
                                   const std::vector<IndexType>& goal_cell);

    // Get the path as a set of x,y,z points, to the same scale
    // as the input PointCloud.
    const std::vector<ColorPoint> getPathPoints(const PathCoordinates& positions);

    // Get the path as a PointCloud.
    const ColorPointCloud getPathPointCloud(const PathCoordinates& positions);

    // Visualize the graph
    void viewGraph();

    // Get the down-sampled PointCloud.
    // (for debug purposes)
    const PointCloud::Ptr getDownsampledPointCloud() { return downsampled_pointcloud_; };

    // Get the 3D Occupancy Grid as a PointCloud.
    // point_separation is how far apart to separate each grid
    // position, for visualization purposes.
    const ColorPointCloud getOccupancyGrid3D();
    const ColorPointCloud getOccupancyGrid3D(const float point_separation);

    // Get the vertices expanded by A* as a PointCloud.
    //   white = un-discovered
    //   black = closed (have been completely examined)
    //   gray = open (discovered but not expanded)
    const ColorPointCloud getAstarExpandedStates();
    const ColorPointCloud getAstarExpandedStates(const float point_separation);

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

    // For debug purposes
    PointCloud::Ptr downsampled_pointcloud_;

    // Occupancy Grid:
    // x,y,z,status
    // status 0 = un-initialized
    //        1 = free
    //        2 = obstacle
    // Vector (header info) is allocated on the stack but the elements on the heap
    std::vector<std::vector<std::vector<std::vector<IndexType> > > > occupancy_grid_3d_;
    IndexType occupancy_grid_width_;
    IndexType occupancy_grid_length_;
    IndexType occupancy_grid_depth_;
    double occupancy_grid_voxel_size_;

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
    
    // Boost graph typedefs
    typedef float Weight;
    typedef float Cost;
    std::vector<int> vertex_colors_;
    typedef boost::property<boost::edge_weight_t, Weight> EdgeWeightProperty;
    typedef boost::property<boost::vertex_name_t, std::string> NameProperty;

    // Directed graph, allows parallel (self-connected) edges
    //typedef boost::adjacency_list<boost::listS, boost::vecS, boost::directedS, NameProperty, EdgeWeightProperty> DirectedGraph;

    // Un-directed graph, no parallel edges allowed
    // edges stored in a set, vertices stored in a vector
    typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, NameProperty, EdgeWeightProperty> UndirectedGraph;

    typedef boost::graph_traits<UndirectedGraph>::vertex_descriptor Vertex;
    typedef boost::graph_traits<UndirectedGraph>::vertex_iterator VertexIterator;
    typedef boost::property_map<UndirectedGraph, boost::vertex_index_t>::type IndexMap;
    typedef boost::property_map<UndirectedGraph, boost::vertex_name_t>::type NameMap;
    typedef boost::iterator_property_map<Vertex*, IndexMap, Vertex, Vertex&> PredecessorMap;
    typedef boost::iterator_property_map<Weight*, IndexMap, Weight, Weight&> DistanceMap;

    UndirectedGraph graph_;

    uint graph_connectivity_;
 
    // Remove all edges (connections between vertices) in the Boost graph.
    void removeAllGraphEdges();

    // Return the range of vertex iterators in the Boost graph,
    // from start to end.
    std::pair<VertexIterator,VertexIterator> getVertices() const;

}; // end class VoxelMap3D

//----------------------------------------------------------------------------//

// Same as method getCoordinates() but this is used by the A* Visitor
const Voxel getVoxelCoordinates(const std::vector<IndexType>& occupancy_grid_size, const uint index);

// Print a std::vector<Voxel>
void printPath(const PathCoordinates& path);

//----------------------------------------------------------------------------//

// A* Heuristic (Euclidean distance)
template <class Graph, class CostType>
class euclidean_distance_heuristic : public boost::astar_heuristic<Graph, CostType>
{
public:
  typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;

  euclidean_distance_heuristic(const std::initializer_list<IndexType>& occupancy_grid_size,
                               Vertex goal)
    : occupancy_grid_size_(occupancy_grid_size),
      goal_(goal)
    {
      const Voxel goal_coordinates = getVoxelCoordinates(occupancy_grid_size_, goal);
      goal_x_ = static_cast<CostType>(goal_coordinates.x);
      goal_y_ = static_cast<CostType>(goal_coordinates.y);
      goal_z_ = static_cast<CostType>(goal_coordinates.z);
    }
  CostType operator()(Vertex v)
  {
    const Voxel coordinates = getVoxelCoordinates(occupancy_grid_size_, v);
    const CostType x = static_cast<CostType>(coordinates.x);
    const CostType y = static_cast<CostType>(coordinates.y);
    const CostType z = static_cast<CostType>(coordinates.z);
    const CostType dx = goal_x_ - x;
    const CostType dy = goal_y_ - y;
    const CostType dz = goal_z_ - z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
  }
private:
  std::vector<IndexType> occupancy_grid_size_;
  Vertex goal_;
  CostType goal_x_;
  CostType goal_y_;
  CostType goal_z_;
};

// A* Visitor that terminates when we find the goal
struct found_goal {}; // exception for termination
template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
  astar_goal_visitor(Vertex goal) : goal_(goal) {}
  template <class Graph>
  void examine_vertex(Vertex u, Graph& g)
  {
    if (u == goal_)
    {
      throw found_goal();
    }
  }
private:
  Vertex goal_;
};

#endif // VOXEL_MAP_3D_H
