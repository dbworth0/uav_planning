//
// Grid Map 2D
//
// David Butterworth
//

#ifndef GRID_MAP_2D_H
#define GRID_MAP_2D_H

#include <initializer_list>
#include <string>
#include <vector>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointXYZRGB ColorPoint;
typedef pcl::PointCloud<ColorPoint> ColorPointCloud;

// Type for storing an index into the Occupancy Grid
struct Cell
{
  int x;
  int y;
  Cell() : x(0), y(0) {};
  Cell(const int _x, const int _y) : x(_x), y(_y) {};
};

typedef std::vector<Cell> PathCoordinates; 

#define FREE_SPACE 1
#define OBSTACLE 2

class GridMap2D
{
  public:
    // Initialize a discrete, 2D grid map.
    // width is number of cells along x-axis
    // connectivity is 4 or 8
    GridMap2D(const int width, const int length, const int connectivity);
    ~GridMap2D() {};

    void addRectangleObstacle(const std::initializer_list<int>& min_corner, const std::initializer_list<int>& max_corner);
    void addLineObstacle(const std::initializer_list<int>& start, const std::initializer_list<int>& end);

    // Calculate shortest path from [x,y] to [x,y]
    // using Dijkstra's algorithm.
    // x,y are cell indices, indexed from zero.
    std::vector<Cell> getShortestPathDijkstra(const std::initializer_list<int>& start_cell,
                                              const std::initializer_list<int>& goal_cell);

    // Calculate shortest path from [x,y] to [x,y]
    // using A* algorithm.
    // x,y are cell indices, indexed from zero.
    std::vector<Cell> getShortestPathAstar(const std::initializer_list<int>& start_cell,
                                           const std::initializer_list<int>& goal_cell);

    // Smooth (short-cut) the path using linear interpolation and
    // by collision checking the Occupancy Grid.
    const PathCoordinates smooth2DPath(const PathCoordinates& path);

    // Visualize the graph
    void viewGraph();
   
    // Get the 2D Occupancy Grid as a PointCloud.
    // point_separation is how far apart to separate each grid
    // position, for visualization purposes.
    const ColorPointCloud getOccupancyGrid2D(const float point_separation);

    // Get the vertices expanded by A* as a PointCloud.
    //   white = un-discovered
    //   black = closed (have been completely examined)
    //   gray = open (discovered but not expanded)
    const ColorPointCloud getAstarExpandedStates(const float point_separation);

  private:
    // Occupancy Grid:
    // x,y,status
    // status 0 = un-initialized
    //        1 = free
    //        2 = obstacle
    // Vector (header info) is allocated on the stack but the elements on the heap
    std::vector<std::vector<std::vector<int> > > occupancy_grid_2d_;
    int occupancy_grid_width_;
    int occupancy_grid_length_;

    // Boost graph typedefs
    typedef float Weight;
    typedef float Cost;
    std::vector<int> vertex_colors_;
    typedef boost::property<boost::edge_weight_t, Weight> EdgeWeightProperty;
    typedef boost::property<boost::vertex_name_t, std::string> NameProperty;

    // Directed graph, allows parallel (self-connected) edges
    //typedef boost::adjacency_list<boost::listS, boost::vecS, boost::directedS, NameProperty, EdgeWeightProperty> DirectedGraph;

    // Un-directed graph, no parallel edges allowed
    // edges stored in a set, vertices stored in a vector.
    // Note: 'weight' becomes cost when using A* algorithm
    typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, NameProperty, EdgeWeightProperty> UndirectedGraph;

    UndirectedGraph graph_;

    typedef boost::graph_traits<UndirectedGraph>::vertex_descriptor Vertex;
    typedef boost::graph_traits<UndirectedGraph>::vertex_iterator VertexIterator;
    typedef boost::property_map<UndirectedGraph, boost::vertex_index_t>::type IndexMap;
    typedef boost::property_map<UndirectedGraph, boost::vertex_name_t>::type NameMap;
    typedef boost::iterator_property_map<Vertex*, IndexMap, Vertex, Vertex&> PredecessorMap;
    typedef boost::iterator_property_map<Weight*, IndexMap, Weight, Weight&> DistanceMap;

    int graph_connectivity_;
 
    // Build the Boost graph from the Occupancy Grid,
    // where cells being FREE_SPACE are connected
    // with edges.
    void buildGraphFromOccupancyGrid();

    // Remove all edges (connections between vertices) in the Boost graph.
    void removeAllGraphEdges();

    // Return the range of vertex iterators in the Boost graph,
    // from start to end.
    std::pair<VertexIterator,VertexIterator> getVertices() const;

    // Check if a set of cells are collision-free.
    // Returns true = no collision.
    const bool checkCellsAreFreeSpace(const PathCoordinates& cells);

    // Similar to above, this checks if cells in a set are obstacles.
    // Returns true = collision detected.
    // Also returns the last cell which is an obstacle.
    const bool checkCellsForCollision(const PathCoordinates& cells, Cell& last_collision_cell);

    // For a specific cell (which may be an obstacle), find
    // the closest cell(s) which is on the path
    const PathCoordinates getClosestCellsOnPath(const Cell& last_collision_cell, const PathCoordinates& path);

}; // end class GridMap2D

//----------------------------------------------------------------------------//

// Convert an x,y value into a linear index, where the
// cells of the grid are ordered in row-major order.
const int getLinearIndex(const std::initializer_list<int>& size,
                         const int i, const int j);

// Convert a linear index into x,y co-ordinates
// of a cell in the Occupancy Grid
const Cell getCoordinates(const std::initializer_list<int>& size,
                          const int index);
const Cell getCoordinates(const std::vector<int>& size,
                          const int index);

// Print a std::vector<Cell>
void printPath(const PathCoordinates& path);

// For an interpolating line between two points, return a
// list of the cells underneath the line.
// Uses a Supercover DDA algorithm, similar to Bresenham's line algorithm.
const PathCoordinates getCellsIntersectingLine(const int p0_x, const int p0_y,
                                               const int p1_x, const int p1_y);

// Calculate the Euclidean distance between two Cells in the Occupancy Grid.
const float euclideanDistance(const Cell& c0, const Cell& c1);

// Sort a std::vector of pair by the second element
template <class T1, class T2, class Pred = std::less<T2> >
struct sort_pair_second
{
  bool operator()(const std::pair<T1,T2>&left, const std::pair<T1,T2>&right)
  {
    Pred p;
    return p(left.second, right.second);
  }
};

//----------------------------------------------------------------------------//

// A* Heuristic (Euclidean distance)
template <class Graph, class CostType>
class euclidean_distance_heuristic : public boost::astar_heuristic<Graph, CostType>
{
public:
  typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;

  euclidean_distance_heuristic(const std::initializer_list<int>& occupancy_grid_size,
                               Vertex goal)
    : occupancy_grid_size_(occupancy_grid_size),
      goal_(goal)
    {
      const Cell goal_coordinates = getCoordinates(occupancy_grid_size_, goal);
      goal_x_ = static_cast<CostType>(goal_coordinates.x);
      goal_y_ = static_cast<CostType>(goal_coordinates.y);
    }
  CostType operator()(Vertex v)
  {
    const Cell coordinates = getCoordinates(occupancy_grid_size_, v);
    const CostType x = static_cast<CostType>(coordinates.x);
    const CostType y = static_cast<CostType>(coordinates.y);
    const CostType dx = goal_x_ - x;
    const CostType dy = goal_y_ - y;
    const CostType cost = std::sqrt(dx*dx + dy*dy);
    return cost;
  }
private:
  std::vector<int> occupancy_grid_size_;
  Vertex goal_;
  CostType goal_x_;
  CostType goal_y_;
};

// A* Heuristic (Manhattan distance)
template <class Graph, class CostType>
class manhattan_distance_heuristic : public boost::astar_heuristic<Graph, CostType>
{
public:
  typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;

  manhattan_distance_heuristic(const std::initializer_list<int>& occupancy_grid_size,
                               Vertex goal)
    : occupancy_grid_size_(occupancy_grid_size),
      goal_(goal)
    {
      const Cell goal_coordinates = getCoordinates(occupancy_grid_size_, goal);
      goal_x_ = static_cast<CostType>(goal_coordinates.x);
      goal_y_ = static_cast<CostType>(goal_coordinates.y);
    }
  CostType operator()(Vertex v)
  {
    const Cell coordinates = getCoordinates(occupancy_grid_size_, v);
    const CostType x = static_cast<CostType>(coordinates.x);
    const CostType y = static_cast<CostType>(coordinates.y);
    const CostType dx = goal_x_ - x;
    const CostType dy = goal_y_ - y;
    const CostType cost = std::fabs(dx) + std::fabs(dy);
    return cost;
  }
private:
  std::vector<int> occupancy_grid_size_;
  Vertex goal_;
  CostType goal_x_;
  CostType goal_y_;
};

// A* Heuristic (Octile distance)
template <class Graph, class CostType>
class octile_distance_heuristic : public boost::astar_heuristic<Graph, CostType>
{
public:
  typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;

  octile_distance_heuristic(const std::initializer_list<int>& occupancy_grid_size,
                               Vertex goal)
    : occupancy_grid_size_(occupancy_grid_size),
      goal_(goal)
    {
      const Cell goal_coordinates = getCoordinates(occupancy_grid_size_, goal);
      goal_x_ = static_cast<CostType>(goal_coordinates.x);
      goal_y_ = static_cast<CostType>(goal_coordinates.y);
    }
  CostType operator()(Vertex v)
  {
    const Cell coordinates = getCoordinates(occupancy_grid_size_, v);
    const CostType x = static_cast<CostType>(coordinates.x);
    const CostType y = static_cast<CostType>(coordinates.y);
    const CostType dx = std::fabs(goal_x_ - x);
    const CostType dy = std::fabs(goal_y_ - y);
    const CostType cost = (dx + dy) + (std::sqrt(2.0) - 2.0)*std::min(dx, dy);
    return cost;
  }
private:
  std::vector<int> occupancy_grid_size_;
  Vertex goal_;
  CostType goal_x_;
  CostType goal_y_;
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

#endif // GRID_MAP_2D_H
