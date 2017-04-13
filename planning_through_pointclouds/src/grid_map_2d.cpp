//
// Grid Map 2D
//
// David Butterworth
//

#include <planning_through_pointclouds/grid_map_2d.h>

#include <iostream> // cout
#include <list>

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/iteration_macros.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/graphviz.hpp>

#include <planning_through_pointclouds/utils.h> // range
#include <planning_through_pointclouds/boost_graph_utils.h> // getAdjacentVertices

GridMap2D::GridMap2D(const int width, const int length, const int connectivity)
  : occupancy_grid_width_(width)
  , occupancy_grid_length_(length)
  , graph_connectivity_(connectivity)
{
  if ((graph_connectivity_ != 4) && (graph_connectivity_ != 8))
  {
    throw std::runtime_error("connectivity must be '4' or '8'");
  }

  // Initialize the Occupancy Grid
  initializeVector(occupancy_grid_2d_, occupancy_grid_width_, occupancy_grid_length_, 1, FREE_SPACE);
  //printVector(occupancy_grid_2d_);

  // Add vertices to our Boost graph,
  // in row-major order, where the last
  // subscript 'y' of 'x,y' is incremented first.
  int vertex_count = 0;
  for (int i = 0; i < occupancy_grid_width_; ++i)
  {
    for (int j = 0; j < occupancy_grid_length_; ++j)
    {
      // Add a vertex.
      // The name string is only used for visualization.
      const std::string vertex_name = "v" + boost::lexical_cast<std::string>(vertex_count);
      boost::add_vertex(std::string(vertex_name), graph_);

      vertex_count++;
    }
  }

  //buildGraphFromOccupancyGrid();
}

void GridMap2D::buildGraphFromOccupancyGrid()
{
  // In case the graph was already built,
  // remove all edges
  //removeAllGraphEdges();

  // Convert the Occupancy Grid into a graph
  // starting along the length (y values): v0, v1, v3, ...
  for (int y = 0; y < occupancy_grid_length_; ++y)
  {
    for (int x = 0; x < occupancy_grid_width_; ++x)
    {
      if (occupancy_grid_2d_.at(x).at(y).at(0) == FREE_SPACE)
      {
        const int current_vertex = getLinearIndex({occupancy_grid_width_,occupancy_grid_length_}, x, y);

        // Add 4-connected edges.
        //
        // Some edges will be created twice, but Boost graph
        // ignores this for an un-directed graph.
        {
          const Weight weight = 1.0;

          const int neighbour_cells_x[4] = {x,   x,   x-1, x+1};
          const int neighbour_cells_y[4] = {y-1, y+1, y,   y  };          

          for (int i = 0; i < 4; ++i)
          {
            const int neighbour_x = neighbour_cells_x[i];
            const int neighbour_y = neighbour_cells_y[i];

            if ((neighbour_x >= 0) && (neighbour_x <= occupancy_grid_width_-1)
                 && (neighbour_y >= 0) && (neighbour_y <= occupancy_grid_length_-1))
            {

              if (occupancy_grid_2d_.at(neighbour_x).at(neighbour_y).at(0) == FREE_SPACE)
              {
                const int neighbour_vertex = getLinearIndex({occupancy_grid_width_,occupancy_grid_length_}, neighbour_x, neighbour_y);
                boost::add_edge(boost::vertex(current_vertex, graph_), boost::vertex(neighbour_vertex, graph_), weight, graph_);
              }
            }
          }
        }

        // Add 8-connected edges
        if (graph_connectivity_ == 8)
        {
          const Weight weight = std::sqrt(2.0);

          const int neighbour_cells_x[4] = {x-1, x-1, x+1, x+1};
          const int neighbour_cells_y[4] = {y-1, y+1, y-1, y+1};

          for (int i = 0; i < 4; ++i)
          {
            const int neighbour_x = neighbour_cells_x[i];
            const int neighbour_y = neighbour_cells_y[i];

            if ((neighbour_x >= 0) && (neighbour_x <= occupancy_grid_width_-1)
                 && (neighbour_y >= 0) && (neighbour_y <= occupancy_grid_length_-1))
            {
              if (occupancy_grid_2d_.at(neighbour_x).at(neighbour_y).at(0) == FREE_SPACE)
              {
                const int neighbour_vertex = getLinearIndex({occupancy_grid_width_,occupancy_grid_length_}, neighbour_x, neighbour_y);
                boost::add_edge(boost::vertex(current_vertex, graph_), boost::vertex(neighbour_vertex, graph_), weight, graph_);
              }
            }
          }
        }
      }
    }
  }
}

void GridMap2D::addRectangleObstacle(const std::initializer_list<int>& min_corner, const std::initializer_list<int>& max_corner)
{
  const std::vector<int> min_corner_vec(min_corner);
  const std::vector<int> max_corner_vec(max_corner);

  if ((max_corner_vec.at(0) < min_corner_vec.at(0)) || (max_corner_vec.at(1) < min_corner_vec.at(1)))
  {
    throw std::runtime_error("max indices should be larger than min indices");
  }

  for (int i = min_corner_vec.at(0); i <= max_corner_vec.at(0); ++i)
  {
    for (int j = min_corner_vec.at(1); j <= max_corner_vec.at(1); ++j)
    {
      if ((i < 0) || (i > occupancy_grid_width_-1)
           || (j < 0) || (j > occupancy_grid_length_-1))
      {
        // Ignore cells outside the Occupancy Grid
        continue;
      }

      occupancy_grid_2d_.at(i).at(j).at(0) = OBSTACLE;
    }
  }
}

void GridMap2D::addLineObstacle(const std::initializer_list<int>& start, const std::initializer_list<int>& end)
{
  const std::vector<int> start_vec(start);
  const std::vector<int> end_vec(end);

  const std::vector<int> x_values = range(start_vec.at(0), end_vec.at(0));
  const std::vector<int> y_values = range(start_vec.at(1), end_vec.at(1));

  if (x_values.size() != y_values.size())
  {
    throw std::runtime_error("size of 'x_values' neq to 'y_values'");
  }

  for (size_t i = 0; i < x_values.size(); ++i)
  {
    const int x = x_values.at(i);
    const int y = y_values.at(i);

    if ((x < 0) || (x > occupancy_grid_width_-1)
         || (y < 0) || (y > occupancy_grid_length_-1))
    {
      // Ignore cells outside the Occupancy Grid
      continue;
    }

    occupancy_grid_2d_.at(x).at(y).at(0) = OBSTACLE;
  }
}

std::vector<Cell> GridMap2D::getShortestPathDijkstra(const std::initializer_list<int>& start_cell,
                                                     const std::initializer_list<int>& goal_cell)
{
  // Build graph, accounting for any obstacles
  buildGraphFromOccupancyGrid();

  const std::vector<int> start_cell_vec(start_cell);
  const std::vector<int> goal_cell_vec(goal_cell);
  const int start_vertex_ID = getLinearIndex({occupancy_grid_width_,occupancy_grid_length_}, start_cell_vec.at(0), start_cell_vec.at(1));
  const int goal_vertex_ID = getLinearIndex({occupancy_grid_width_,occupancy_grid_length_}, goal_cell_vec.at(0), goal_cell_vec.at(1));

  std::cout << "Finding shortest path using Dijkstra..." << std::endl;
  std::cout << "Start: [" << start_cell_vec.at(0) << "," << start_cell_vec.at(1) << "], index = " << start_vertex_ID << std::endl;
  std::cout << "Goal: [" << goal_cell_vec.at(0) << "," << goal_cell_vec.at(1) << "], index = " << goal_vertex_ID << std::endl;

  std::vector<Vertex> predecessors(boost::num_vertices(graph_));
  std::vector<Weight> distances(boost::num_vertices(graph_));
  IndexMap index_map = boost::get(boost::vertex_index, graph_);
  PredecessorMap predecessor_map(&predecessors[0], index_map);
  DistanceMap distance_map(&distances[0], index_map);
  NameMap name_map = boost::get(boost::vertex_name, graph_);
  Vertex start_vertex = boost::vertex(start_vertex_ID, graph_);
  Vertex goal_vertex = boost::vertex(goal_vertex_ID, graph_);
 
  // Compute shortest paths from start_vertex to all vertices,
  // and store the output in predecessors and distances
  boost::dijkstra_shortest_paths(graph_, start_vertex,
    boost::distance_map(distance_map).predecessor_map(predecessor_map));

  /*
  // Print distance from start vertex to all other vertices:
  std::cout << "distances and parents:" << std::endl;
  BGL_FORALL_VERTICES(v, graph_, UndirectedGraph)
  {
    std::cout << "distance(" << name_map[start_vertex] << ", " << name_map[v] << ") = " << distance_map[v] << ", ";
    std::cout << "predecessor(" << name_map[v] << ") = " << name_map[predecessor_map[v]] << std::endl;
  }
  std::cout << std::endl;
  */
 
  // Extract the shortest path by stepping
  // from the goal back to the start:
  typedef std::vector<UndirectedGraph::edge_descriptor> PathType;
  PathType path;
  Vertex v = goal_vertex;
  for (Vertex u = predecessor_map[v]; u != v; v = u, u = predecessor_map[v])
  {
    std::pair<UndirectedGraph::edge_descriptor, bool> edgePair = boost::edge(u, v, graph_);
    UndirectedGraph::edge_descriptor edge = edgePair.first;
 
    path.push_back(edge);
  }
 
  /*
  // Print the shortest path
  std::cout << "Shortest path from start to goal:" << std::endl;
  float totalDistance = 0;
  for (PathType::reverse_iterator pathIterator = path.rbegin(); pathIterator != path.rend(); ++pathIterator)
  {
    std::cout << name_map[boost::source(*pathIterator, graph_)] << " -> " << name_map[boost::target(*pathIterator, graph_)]
              << " = " << boost::get( boost::edge_weight, graph_, *pathIterator ) << std::endl;
  }
  std::cout << std::endl;
  */
  
  std::cout << "Total distance (Dijkstra): " << distance_map[goal_vertex] << std::endl;

  std::vector<Cell> path_coordinates;
  for (PathType::reverse_iterator pathIterator = path.rbegin(); pathIterator != path.rend(); ++pathIterator)
  {
    const int vertex_ID = boost::source(*pathIterator, graph_);
    const Cell coordinates = getCoordinates({occupancy_grid_width_,occupancy_grid_length_}, vertex_ID);
    path_coordinates.push_back(coordinates);
    //std::cout << "vertex_ID: " << vertex_ID << "  " << coordinates.x << "," << coordinates.y << std::endl;
  }
  // Add the goal position, because the above path only
  // includes the first vertex of each edge.
  const Cell goal_coordinates = getCoordinates({occupancy_grid_width_,occupancy_grid_length_}, goal_vertex_ID);
  path_coordinates.push_back(goal_coordinates);

  return path_coordinates;
}

std::vector<Cell> GridMap2D::getShortestPathAstar(const std::initializer_list<int>& start_cell,
                                                  const std::initializer_list<int>& goal_cell)
{
  // Build graph, accounting for any obstacles
  buildGraphFromOccupancyGrid();

  const std::vector<int> start_cell_vec(start_cell);
  const std::vector<int> goal_cell_vec(goal_cell);
  const int start_vertex_ID = getLinearIndex({occupancy_grid_width_,occupancy_grid_length_}, start_cell_vec.at(0), start_cell_vec.at(1));
  const int goal_vertex_ID = getLinearIndex({occupancy_grid_width_,occupancy_grid_length_}, goal_cell_vec.at(0), goal_cell_vec.at(1));

  std::cout << "Finding shortest path using A*..." << std::endl;
  std::cout << "  Start: [" << start_cell_vec.at(0) << "," << start_cell_vec.at(1) << "], index = " << start_vertex_ID << std::endl;
  std::cout << "  Goal: [" << goal_cell_vec.at(0) << "," << goal_cell_vec.at(1) << "], index = " << goal_vertex_ID << std::endl;

  std::vector<Vertex> predecessors(boost::num_vertices(graph_));
  std::vector<Weight> distances(boost::num_vertices(graph_));
  IndexMap index_map = boost::get(boost::vertex_index, graph_);
  PredecessorMap predecessor_map(&predecessors[0], index_map);
  DistanceMap distance_map(&distances[0], index_map);
  NameMap name_map = boost::get(boost::vertex_name, graph_);
  Vertex start_vertex = boost::vertex(start_vertex_ID, graph_);
  Vertex goal_vertex = boost::vertex(goal_vertex_ID, graph_);

  typedef std::vector<boost::default_color_type> ColorMap;
  ColorMap colors_map(boost::num_vertices(graph_));
  // white = un-discovered
  // black = closed (have been completely examined)
  // gray = open (discovered but not expanded)
  const char* color_names[] = {"white", "gray", "green", "red", "black"};
  bool success = false;
  std::list<Vertex> shortest_path;
  try
  {
    // Use BGL A* named parameter interface
    astar_search(graph_,
                 start_vertex,
                 //euclidean_distance_heuristic<UndirectedGraph, Cost>(
                 //manhattan_distance_heuristic<UndirectedGraph, Cost>(
                 octile_distance_heuristic<UndirectedGraph, Cost>(
                   {occupancy_grid_width_,occupancy_grid_length_}, goal_vertex),
                 boost::distance_map(distance_map)
                   .predecessor_map(predecessor_map)
                   .color_map(&colors_map[0])
                   .visitor(astar_goal_visitor<Vertex>(goal_vertex)));  
  }
  catch (found_goal fg)
  {
    // Found a path to the goal.
    // (The A* Visitor throws when it visits the goal vertex)
    for (Vertex v = goal_vertex;; v = predecessor_map[v])
    {
      shortest_path.push_front(v);
      if (predecessor_map[v] == v)
      {
        break;
      }
    }
    /*
    // Print the shortest path
    std::cout << "Shortest path from " << name_map[start_vertex] << " to "
              << name_map[goal_vertex] << ": ";
    std::list<Vertex>::iterator spi = shortest_path.begin();
    std::cout << name_map[start_vertex];
    for (++spi; spi != shortest_path.end(); ++spi)
    {
      std::cout << " -> " << name_map[*spi];
    }
    std::cout << std::endl;
    */
    std::cout << "Total distance (A*): " << distance_map[goal_vertex] << std::endl;

    success = true;
  }

  if (!success)
  {
    std::cout << "ERROR: Didn't find a path from " << name_map[start_vertex] << "to"
              << name_map[goal_vertex] << "!" << std::endl;
  }
 
  std::vector<Cell> path_coordinates;

  // Add the start position, because the A* shortest path only
  // includes the second vertex of each edge.
  const Cell start_coordinates = getCoordinates({occupancy_grid_width_,occupancy_grid_length_}, start_vertex_ID);
  path_coordinates.push_back(start_coordinates);
  std::list<Vertex>::iterator spi = shortest_path.begin();
  for (++spi; spi != shortest_path.end(); ++spi)
  {
    const int vertex_ID = *spi;
    const Cell coordinates = getCoordinates({occupancy_grid_width_,occupancy_grid_length_}, vertex_ID);
    path_coordinates.push_back(coordinates);
    //std::cout << "vertex_ID: " << vertex_ID << "  " << coordinates.x << "," << coordinates.y << "  " << color_names[colors_map[*spi]] << std::endl;
  }

  // Copy the vertex "colors" into member variable, for visualization
  vertex_colors_.clear();
  BGL_FORALL_VERTICES(v, graph_, UndirectedGraph)
  {
    //std::cout << "Vertex " << v << " = " << color_names[colors_map[v]] << std::endl;
    vertex_colors_.push_back(static_cast<int>(colors_map[v]));
  }

  return path_coordinates;
}

const PathCoordinates GridMap2D::smooth2DPath(const PathCoordinates& path)
{
  PathCoordinates smoothed_path;

  int i = 0;

  int segment_start_x = path.at(0).x;
  int segment_start_y = path.at(0).y;
  smoothed_path.push_back(path.at(0));

  bool done = false;
  while (!done)
  {
    if (i+1 < path.size())
    {
      int next_x;
      int next_y;

      bool checking_next_point = true;
      while (checking_next_point)
      {
        i++;

        if (i == path.size())
        {
          // Reached the end of the path
          smoothed_path.push_back(path.at(i-1));
          done = true;
          break;
        }

        next_x = path.at(i).x;
        next_y = path.at(i).y;

        // Short-cut the path using linear interpolation
        PathCoordinates interp_line_cells = getCellsIntersectingLine(segment_start_x, segment_start_y, next_x, next_y);

        Cell last_collision_cell;
        if (checkCellsForCollision(interp_line_cells, last_collision_cell))
        {
          // The interpolating line collided with a cel.
          // Get the cell on the path which is closest to
          // the last colliding cell.
          PathCoordinates closest_cells = getClosestCellsOnPath(last_collision_cell, path);

          // For the closest cells, sort them so the cell closest
          // to our start position is first
          std::vector<std::pair<Cell, float> > sorted_closest_cells;
          for (size_t j = 0; j < closest_cells.size(); ++j)
          {
            const float dist_from_start = euclideanDistance(closest_cells.at(j), smoothed_path.at(0));
            sorted_closest_cells.push_back(std::make_pair(closest_cells.at(j), dist_from_start));
          }
          std::sort(sorted_closest_cells.begin(), sorted_closest_cells.end(), sort_pair_second<Cell, float>());
          closest_cells.clear();
          for (size_t j = 0; j < sorted_closest_cells.size(); ++j)
          {
            closest_cells.push_back(sorted_closest_cells.at(j).first);
          }

          // Add to the path the cell closest to the collision
          // which has not already been added to the path
          for (size_t j = 0; j < closest_cells.size(); ++j)
          {
            bool cell_already_added = false;
            for (size_t k = 0; k < smoothed_path.size(); ++k)
            {
              if ((closest_cells.at(j).x == smoothed_path.at(k).x) && (closest_cells.at(j).y == smoothed_path.at(k).y))
              {
                cell_already_added = true;
                break;
              }
            }

            if (!cell_already_added)
            {
              // Insert a waypoint at the point closest to the obstacle
              smoothed_path.push_back(closest_cells.at(j));

              segment_start_x = closest_cells.at(j).x;
              segment_start_y = closest_cells.at(j).y;
              break;
            }
          }

          // ToDo: Should reset i back
          // to the index of the point we added
          checking_next_point = false;
          break;
        }
      }
    }

    i++;

    if (i == path.size())
    {
      done = true;
    }
  }

  float length = 0.0;
  for (size_t i = 0; i < smoothed_path.size()-1; ++i)
  {
    length += euclideanDistance(smoothed_path.at(i),smoothed_path.at(i+1));
  }
  std::cout << "Total distance (smoothed): " << length << std::endl;

  return smoothed_path;
}

void GridMap2D::viewGraph()
{
  boost::dynamic_properties dp;
  dp.property("id", get(boost::vertex_name, graph_)); // vertex_index
  dp.property("weight", get(boost::edge_weight, graph_));
  
  // Write the graph to the terminal
  //write_graphviz_dp(std::cout, graph_, dp, std::string("id"));

  // Write graph to .png and view image
  std::ofstream fout("graph.dot");
  boost::write_graphviz_dp(fout, graph_, dp, std::string("id"));
  const int result1 = std::system("dot -Tpng graph.dot > ~/graph.png");
  const int result2 = std::system("display ~/graph.png");
}

const ColorPointCloud GridMap2D::getOccupancyGrid2D(const float point_separation)
{
  ColorPointCloud cloud;

  for (int i = 0; i < occupancy_grid_width_; ++i)
  {
    for (int j = 0; j < occupancy_grid_length_; ++j)
    {
      ColorPoint point;
      point.x = static_cast<float>(i) * point_separation;
      point.y = static_cast<float>(j) * point_separation;
      point.z = 0.0;

      if (occupancy_grid_2d_.at(i).at(j).at(0) == FREE_SPACE)
      {
        // light gray
        point.r = 220;
        point.g = 220;
        point.b = 220;
      }
      else if (occupancy_grid_2d_.at(i).at(j).at(0) == OBSTACLE)
      {
        // brown = 90,39,41
        point.r = 90; // uint_8
        point.g = 39;
        point.b = 41;
      }
      else
      {
        // Unknown cell type, color red
        point.r = 255;
        point.g = 0;
        point.b = 0;
      }

      cloud.points.push_back(point);
    }
  }

  return cloud;
}

const ColorPointCloud GridMap2D::getAstarExpandedStates(const float point_separation)
{
  // Color names:
  //   {"white", "gray", "green", "red", "black"};

  ColorPointCloud cloud;

  if (vertex_colors_.size() != (occupancy_grid_width_ * occupancy_grid_length_))
  {
    std::cout << "No vertex color data in getAstarExpandedStates()" << std::endl;
    return cloud;
  }

  for (int i = 0; i < occupancy_grid_width_; ++i)
  {
    for (int j = 0; j < occupancy_grid_length_; ++j)
    {
      ColorPoint point;
      point.x = static_cast<float>(i) * point_separation;
      point.y = static_cast<float>(j) * point_separation;
      point.z = 0.0;

      if (occupancy_grid_2d_.at(i).at(j).at(0) == OBSTACLE)
      {
        // Obstacle, color red
        point.r = 255;
        point.g = 0;
        point.b = 0;
      }
      else
      {
        const int vertex_ID = getLinearIndex({occupancy_grid_width_,occupancy_grid_length_}, i, j);
        const int color_idx = vertex_colors_.at(vertex_ID);

        if (color_idx == 0)
        {
          // white
          point.r = 255;
          point.g = 255;
          point.b = 255;
        }
        else if (color_idx == 1)
        {
          // gray
          point.r = 105;
          point.g = 105;
          point.b = 105;
        }
        else if (color_idx == 4)
        {
          // black
          point.r = 0;
          point.g = 0;
          point.b = 0;
        }
        else
        {
          // We don't expect 'green', 'red' or other labels
          std::cout << "ERROR: Un-expected color idx " << color_idx << " in getAstarExpandedStates()" << std::endl;
        }
      }

      cloud.points.push_back(point);
    }
  }

  return cloud;
}

void GridMap2D::removeAllGraphEdges()
{
  BGL_FORALL_VERTICES(v, graph_, UndirectedGraph)
  {
    boost::clear_vertex(v, graph_);
  }
}

/*
void GridMap2D::removeAllGraphEdges()
{
  const std::vector<int> adjacent_vertices = getAdjacentVertices(22, graph_);
  for (size_t i = 0; i < adjacent_vertices.size(); ++i)
  {
    boost::remove_edge(boost::vertex(22, graph_),boost::vertex(adjacent_vertices.at(i), graph_), graph_);
  }
}
*/

std::pair<GridMap2D::VertexIterator,GridMap2D::VertexIterator> GridMap2D::getVertices() const
{
  return boost::vertices(graph_);
}

const bool GridMap2D::checkCellsAreFreeSpace(const PathCoordinates& cells)
{
  for (size_t i = 0; i < cells.size(); ++i)
  {
    if ((cells.at(i).x < 0) || (cells.at(i).x > occupancy_grid_width_-1)
         || (cells.at(i).y < 0) || (cells.at(i).y > occupancy_grid_length_-1))
    {
      // This should never happen
      throw std::runtime_error("cell indices are outside range of Occupancy Grid in checkCellsAreFreeSpace()");
    }

    if (occupancy_grid_2d_.at(cells.at(i).x).at(cells.at(i).y).at(0) != FREE_SPACE)
    {
      // Detected collision
      return false;
    }
  }

  return true;
}

const bool GridMap2D::checkCellsForCollision(const PathCoordinates& cells, Cell& last_collision_cell)
{
  bool found_collision = false;

  for (size_t i = 0; i < cells.size(); ++i)
  {
    if ((cells.at(i).x < 0) || (cells.at(i).x > occupancy_grid_width_-1)
         || (cells.at(i).y < 0) || (cells.at(i).y > occupancy_grid_length_-1))
    {
      // This should never happen
      throw std::runtime_error("cell indices are outside range of Occupancy Grid in checkCellsAreFreeSpace()");
    }

    if (occupancy_grid_2d_.at(cells.at(i).x).at(cells.at(i).y).at(0) == OBSTACLE)
    {
      // Detected collision
      found_collision = true;
      last_collision_cell = Cell(cells.at(i).x, cells.at(i).y);
    }
  }

  return found_collision;
}

const PathCoordinates GridMap2D::getClosestCellsOnPath(const Cell& last_collision_cell, const PathCoordinates& path)
{
  // There could be more than one cell
  // which is equally close
  PathCoordinates closest_cells;

  const int x = last_collision_cell.x;
  const int y = last_collision_cell.y;

  const int neighbour_cells_x[4] = {x,   x,   x-1, x+1};
  const int neighbour_cells_y[4] = {y-1, y+1, y,   y  };          
  for (int i = 0; i < 4; ++i)
  {
    const int neighbour_x = neighbour_cells_x[i];
    const int neighbour_y = neighbour_cells_y[i];

    if ((neighbour_x >= 0) && (neighbour_x <= occupancy_grid_width_-1)
         && (neighbour_y >= 0) && (neighbour_y <= occupancy_grid_length_-1))
    {
      for (size_t j = 0; j < path.size(); ++j)
      {
        if ((neighbour_x == path.at(j).x) && (neighbour_y == path.at(j).y))
        {
          // A 4-connected neighbour is on the path
          closest_cells.push_back(Cell(neighbour_x, neighbour_y));
        }
      }
    }
  }

  // ToDo: If found nothing yet, iterate over the line because closest
  // cell is farther than 1 cell distance away.

  if (closest_cells.size() == 0)
  {
    // Should never happen
    std::cout << "ERROR: closest_cells.size() = 0 in getClosestCellOnPath()" << std::endl;
  }

  return closest_cells;
}

//----------------------------------------------------------------------------//

const int getLinearIndex(const std::initializer_list<int>& size,
                         const int i, const int j)
{
  const std::vector<int> size_vec(size);
  const int j_step = size_vec.at(1);

  return (i)*j_step + (j);
}

const Cell getCoordinates(const std::initializer_list<int>& size,
                          const int index)
{
  const std::vector<int> size_vec(size);

  return getCoordinates(size_vec, index);
}

const Cell getCoordinates(const std::vector<int>& size,
                          const int index)
{
  const int j_step = size.at(1);

  const int x = static_cast<int>(index / j_step);
  const int y = static_cast<int>(index % j_step); // modulus (remainder)

  return Cell(x, y);
}

void printPath(const PathCoordinates& path)
{
  std::cout << "Path co-ordinates: " << std::endl;
  for (size_t i = 0; i < path.size(); ++i)
  {
    std::cout << "  " << path.at(i).x << "," << path.at(i).y << std::endl;
  }
}

const PathCoordinates getCellsIntersectingLine(const int p0_x, const int p0_y,
                                               const int p1_x, const int p1_y)
{
  const int dx = p1_x - p0_x;
  const int dy = p1_y - p0_y;

  const int nx = std::abs(dx);
  const int ny = std::abs(dy);

  const bool sign_x = dx > 0 ? 1 : -1;
  const bool sign_y = dy > 0 ? 1 : -1;

  Cell p = Cell(p0_x, p0_y);

  PathCoordinates intersecting_cells;
  intersecting_cells.push_back(Cell(p.x, p.y));

  for (int ix = 0, iy = 0; ix < nx || iy < ny;)
  {
    if ((0.5+ix) / nx == (0.5+iy) / ny)
    {
      // Next step is diagonal
      p.x += sign_x;
      p.y += sign_y;
      ix++;
      iy++;
    }
    else if ((0.5+ix) / nx < (0.5+iy) / ny)
    {
      // Next step is horizontal
      p.x += sign_x;
      ix++;
    }
    else
    {
      // Next step is vertical
      p.y += sign_y;
      iy++;
    }

    intersecting_cells.push_back(Cell(p.x, p.y));
  }

  return intersecting_cells;
}

const float euclideanDistance(const Cell& c0, const Cell& c1)
{
  const float dx = c0.x - c1.x;
  const float dy = c0.y - c1.y;
  return std::sqrt(dx*dx + dy*dy);
}

//----------------------------------------------------------------------------//
