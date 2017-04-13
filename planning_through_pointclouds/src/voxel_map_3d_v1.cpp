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

#include <planning_through_pointclouds/voxel_map_3d_v1.h>

#include <iostream> // cout
#include <list>

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/iteration_macros.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/graphviz.hpp>

#include <pcl/common/distances.h> // euclideanDistance()

#include <planning_through_pointclouds/utils.h> // range, printStdVector
#include <planning_through_pointclouds/pcl_utils.h> // loadPointCloud, downsamplePointCloud
#include <planning_through_pointclouds/boost_graph_utils.h> // getAdjacentVertices

VoxelMap3D::VoxelMap3D(const uint width, const uint length, const uint depth, const double voxel_size, const uint connectivity)
  : occupancy_grid_voxel_size_(voxel_size)
  , graph_connectivity_(connectivity)
  , downsampled_pointcloud_(new PointCloud())
{
  if ((graph_connectivity_ != 6) && (graph_connectivity_ != 26))
  {
    throw std::runtime_error("connectivity must be '6' or '26'");
  }

  if ((width > std::numeric_limits<IndexType>::max()) || (length > std::numeric_limits<IndexType>::max()) || (depth > std::numeric_limits<IndexType>::max()))
  {
    throw std::runtime_error("Number of required voxels in one dimension exceeds size of IndexType");
  }

  occupancy_grid_width_ = static_cast<IndexType>(width);
  occupancy_grid_length_ = static_cast<IndexType>(length);
  occupancy_grid_depth_ = static_cast<IndexType>(depth);

  // Initialize the Occupancy Grid
  initializeVector<IndexType>(occupancy_grid_3d_, occupancy_grid_width_, occupancy_grid_length_, occupancy_grid_depth_, 1, FREE_SPACE);
  //printVector(occupancy_grid_3d_);

  addGraphVertices();

  //buildGraphFromOccupancyGrid();
}

VoxelMap3D::VoxelMap3D(PointCloud::Ptr& input_pointcloud, const double voxel_size, const double padding_radius, const uint connectivity)
  : occupancy_grid_voxel_size_(voxel_size)
  , graph_connectivity_(connectivity)
  , downsampled_pointcloud_(new PointCloud())
{
  if ((graph_connectivity_ != 6) && (graph_connectivity_ != 26))
  {
    throw std::runtime_error("connectivity must be '6' or '26'");
  }

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

  // Initialize the Occupancy Grid
  initializeVector<IndexType>(occupancy_grid_3d_, occupancy_grid_width_, occupancy_grid_length_, occupancy_grid_depth_, 1, FREE_SPACE);
  //printVector(occupancy_grid_3d_);

  std::cout << "Downsampling PointCloud... " << std::endl;
  // ToDo: Write my own filter that makes points evenly distributed.
  // Note: The resulting points are not within the center of the voxel grid
  downsamplePointCloud<Point>(input_pointcloud, occupancy_grid_voxel_size_, downsampled_pointcloud_);

  std::cout << "Generating Occupancy Grid from PointCloud... " << std::endl;
  for (PointCloud::iterator it = downsampled_pointcloud_->begin(); it != downsampled_pointcloud_->end(); ++it)
  {
    const Voxel voxel_idx = getVoxelIndex(*it);

    // ToDo: Check voxel index is within grid before trying to add it
    occupancy_grid_3d_.at(voxel_idx.x).at(voxel_idx.y).at(voxel_idx.z).at(0) = OBSTACLE;

    // Apply padding to this point, by turning on
    // all voxels enclosed within some radius
    if (padding_radius > 0.00001)
    {
      // Get the voxel indices that make up a cube.
      // (this only returns the valid indices inside the voxel grid)
      const PathCoordinates cube_indices = getVoxelCubeIndices(voxel_idx, padding_radius);

      for (size_t i = 0; i < cube_indices.size(); ++i)
      {
        const Point point = getPoint(cube_indices.at(i), occupancy_grid_voxel_size_);
        const float dist = pcl::euclideanDistance(*it, point);
        if (dist <= padding_radius)
        {
          occupancy_grid_3d_.at(cube_indices.at(i).x).at(cube_indices.at(i).y).at(cube_indices.at(i).z).at(0) = OBSTACLE;
        }
      }
    }
  }
  std::cout << "Done." << std::endl;

  //addGraphVertices();

  //buildGraphFromOccupancyGrid();
}

void VoxelMap3D::addGraphVertices()
{
  // Add vertices to our Boost graph,
  // in row-major order, where the last
  // subscript 'z' of 'x,y,z' is incremented first.
  uint vertex_count = 0;
  for (IndexType i = 0; i < occupancy_grid_width_; ++i)
  {
    for (IndexType j = 0; j < occupancy_grid_length_; ++j)
    {
      for (IndexType k = 0; k < occupancy_grid_depth_; ++k)
      {
        // Add a vertex.
        // The name string is only used for visualization.
        const std::string vertex_name = "v" + boost::lexical_cast<std::string>(vertex_count);
        boost::add_vertex(std::string(vertex_name), graph_);

        vertex_count++;
      }
    }
  }

  std::cout << "Added " << vertex_count << " vertices" << std::endl;
}

void VoxelMap3D::buildGraphFromOccupancyGrid()
{
  std::cout << "Building graph from Occupancy Grid..." << std::endl;

  // In case the graph was already built,
  // remove all edges
  //removeAllGraphEdges();

  // Convert the Occupancy Grid into a graph
  // starting along the width (x values): v0, v1, v3, ...
  for (IndexType z = 0; z < occupancy_grid_depth_; ++z)
  {
    for (IndexType y = 0; y < occupancy_grid_length_; ++y)
    {
      for (IndexType x = 0; x < occupancy_grid_width_; ++x)
      {
        if (occupancy_grid_3d_.at(x).at(y).at(z).at(0) == FREE_SPACE)
        {
          // Add graph edges for voxels that
          // are connected by their faces.
          //
          // Some edges will be created twice, but Boost graph
          // ignores this for an un-directed graph.
          {
            const Weight weight = 1.0;

            std::vector<Voxel> neighbour_voxels;
            
            neighbour_voxels.push_back(Voxel(x-1, y, z));
            neighbour_voxels.push_back(Voxel(x+1, y, z));
            neighbour_voxels.push_back(Voxel(x, y-1, z));
            neighbour_voxels.push_back(Voxel(x, y+1, z));
            neighbour_voxels.push_back(Voxel(x, y, z-1));
            neighbour_voxels.push_back(Voxel(x, y, z+1));
            
            /*
            // add only 3 forward edges
            neighbour_voxels.push_back(Voxel(x+1, y, z));
            neighbour_voxels.push_back(Voxel(x, y+1, z));
            neighbour_voxels.push_back(Voxel(x, y, z+1));
            */

            const uint vertex_ID = getLinearIndex(x, y, z);

            for (IndexType i = 0; i < 6; ++i)
            //for (IndexType i = 0; i < 3; ++i)
            {
              const IndexType neighbour_x = neighbour_voxels.at(i).x;
              const IndexType neighbour_y = neighbour_voxels.at(i).y;
              const IndexType neighbour_z = neighbour_voxels.at(i).z;

              if ((neighbour_x >= 0) && (neighbour_x <= occupancy_grid_width_-1)
                   && (neighbour_y >= 0) && (neighbour_y <= occupancy_grid_length_-1)
                   && (neighbour_z >= 0) && (neighbour_z <= occupancy_grid_depth_-1))
              {

                if (occupancy_grid_3d_.at(neighbour_x).at(neighbour_y).at(neighbour_z).at(0) == FREE_SPACE)
                {
                  const uint neighbour_vertex_ID = getLinearIndex(neighbour_x, neighbour_y, neighbour_z);
                  boost::add_edge(boost::vertex(static_cast<int>(vertex_ID), graph_),
                                                boost::vertex(static_cast<int>(neighbour_vertex_ID), graph_),
                                                weight, graph_);
                }
              }
            }
          }

          // Add 12 voxels connected by their edges
          if (graph_connectivity_ == 26)
          {
            const Weight weight = std::sqrt(2.0);

            std::vector<Voxel> neighbour_voxels;
            neighbour_voxels.push_back(Voxel(x-1, y-1, z));
            neighbour_voxels.push_back(Voxel(x+1, y-1, z));
            neighbour_voxels.push_back(Voxel(x-1, y-1, z));
            neighbour_voxels.push_back(Voxel(x-1, y+1, z));

            neighbour_voxels.push_back(Voxel(x-1, y, z-1));
            neighbour_voxels.push_back(Voxel(x+1, y, z-1));
            neighbour_voxels.push_back(Voxel(x-1, y, z+1));
            neighbour_voxels.push_back(Voxel(x+1, y, z+1));

            neighbour_voxels.push_back(Voxel(x, y-1, z-1));
            neighbour_voxels.push_back(Voxel(x, y+1, z-1));
            neighbour_voxels.push_back(Voxel(x, y-1, z+1));
            neighbour_voxels.push_back(Voxel(x, y+1, z+1));

            const uint vertex_ID = getLinearIndex(x, y, z);

            for (IndexType i = 0; i < 12; ++i)
            {
              const IndexType neighbour_x = neighbour_voxels.at(i).x;
              const IndexType neighbour_y = neighbour_voxels.at(i).y;
              const IndexType neighbour_z = neighbour_voxels.at(i).z;

              if ((neighbour_x >= 0) && (neighbour_x <= occupancy_grid_width_-1)
                   && (neighbour_y >= 0) && (neighbour_y <= occupancy_grid_length_-1)
                   && (neighbour_z >= 0) && (neighbour_z <= occupancy_grid_depth_-1))
              {
                if (occupancy_grid_3d_.at(neighbour_x).at(neighbour_y).at(neighbour_z).at(0) == FREE_SPACE)
                {
                  const uint neighbour_vertex_ID = getLinearIndex(neighbour_x, neighbour_y, neighbour_z);
                  boost::add_edge(boost::vertex(static_cast<int>(vertex_ID), graph_),
                                  boost::vertex(static_cast<int>(neighbour_vertex_ID), graph_),
                                  weight, graph_);
                }
              }
            }
          }

          // Add 8 voxels connected by their corners
          if (graph_connectivity_ == 26)
          {
            const Weight weight = std::sqrt(3.0);

            std::vector<Voxel> neighbour_voxels;
            neighbour_voxels.push_back(Voxel(x-1, y-1, z-1));
            neighbour_voxels.push_back(Voxel(x+1, y-1, z-1));
            neighbour_voxels.push_back(Voxel(x-1, y-1, z-1));
            neighbour_voxels.push_back(Voxel(x-1, y+1, z-1));

            neighbour_voxels.push_back(Voxel(x-1, y-1, z+1));
            neighbour_voxels.push_back(Voxel(x+1, y-1, z+1));
            neighbour_voxels.push_back(Voxel(x-1, y-1, z+1));
            neighbour_voxels.push_back(Voxel(x-1, y+1, z+1));

            const uint vertex_ID = getLinearIndex(x, y, z);

            for (IndexType i = 0; i < 8; ++i)
            {
              const IndexType neighbour_x = neighbour_voxels.at(i).x;
              const IndexType neighbour_y = neighbour_voxels.at(i).y;
              const IndexType neighbour_z = neighbour_voxels.at(i).z;

              if ((neighbour_x >= 0) && (neighbour_x <= occupancy_grid_width_-1)
                   && (neighbour_y >= 0) && (neighbour_y <= occupancy_grid_length_-1)
                   && (neighbour_z >= 0) && (neighbour_z <= occupancy_grid_depth_-1))
              {
                if (occupancy_grid_3d_.at(neighbour_x).at(neighbour_y).at(neighbour_z).at(0) == FREE_SPACE)
                {
                  const uint neighbour_vertex_ID = getLinearIndex(neighbour_x, neighbour_y, neighbour_z);
                  boost::add_edge(boost::vertex(static_cast<int>(vertex_ID), graph_),
                                  boost::vertex(static_cast<int>(neighbour_vertex_ID), graph_),
                                  weight, graph_);
                }
              }
            }
          }

        } // end if FREE_SPACE

      } // end for x
    } // end for y
  } // end for z
}

void VoxelMap3D::addCuboidObstacle(const std::initializer_list<IndexType>& min_corner, const std::initializer_list<IndexType>& max_corner)
{
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
}

const PathCoordinates VoxelMap3D::getShortestPathDijkstra(const Point& start_point,
                                                          const Point& goal_point)
{
  const Voxel start_cell = getVoxelIndex(start_point);
  const Voxel goal_cell = getVoxelIndex(goal_point);

  return getShortestPathDijkstra({start_cell.x, start_cell.y, start_cell.z},
                                 {goal_cell.x, goal_cell.y, goal_cell.z});
}

const PathCoordinates VoxelMap3D::getShortestPathDijkstra(const std::initializer_list<IndexType>& start_cell,
                                                          const std::initializer_list<IndexType>& goal_cell)
{
  // Build graph, accounting for any obstacles.
  buildGraphFromOccupancyGrid();

  const std::vector<IndexType> start_cell_vec(start_cell);
  const std::vector<IndexType> goal_cell_vec(goal_cell);
  const uint start_vertex_ID = getLinearIndex(start_cell_vec.at(0), start_cell_vec.at(1), start_cell_vec.at(2));
  const uint goal_vertex_ID = getLinearIndex(goal_cell_vec.at(0), goal_cell_vec.at(1), goal_cell_vec.at(2));

  std::cout << "Finding shortest path using Dijkstra..." << std::endl;
  std::cout << "  Start: [" << start_cell_vec.at(0) << "," << start_cell_vec.at(1) << "," << start_cell_vec.at(2) << "], index = " << start_vertex_ID << std::endl;
  std::cout << "  Goal: [" << goal_cell_vec.at(0) << "," << goal_cell_vec.at(1) << "," << goal_cell_vec.at(2) << "], index = " << goal_vertex_ID << std::endl;

  checkStartAndGoalValidity(start_cell, goal_cell);

  std::vector<Vertex> predecessors(boost::num_vertices(graph_));
  std::vector<Weight> distances(boost::num_vertices(graph_));
  IndexMap index_map = boost::get(boost::vertex_index, graph_);
  PredecessorMap predecessor_map(&predecessors[0], index_map);
  DistanceMap distance_map(&distances[0], index_map);
  NameMap name_map = boost::get(boost::vertex_name, graph_);
  Vertex start_vertex = boost::vertex(static_cast<int>(start_vertex_ID), graph_);
  Vertex goal_vertex = boost::vertex(static_cast<int>(goal_vertex_ID), graph_);

  // Compute shortest paths from start_vertex to all vertices,
  // and store the output in predecessors and distances
  boost::dijkstra_shortest_paths(graph_,
                                 start_vertex,
                                 boost::distance_map(distance_map)
                                   .predecessor_map(predecessor_map));
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
    //std::cout << name_map[boost::source(*pathIterator, graph_)] << " -> " << name_map[boost::target(*pathIterator, graph_)]
    //          << " = " << boost::get( boost::edge_weight, graph_, *pathIterator ) << std::endl;

    const Voxel p0 = getCoordinates(static_cast<uint>(boost::source(*pathIterator, graph_)));
    const Voxel p1 = getCoordinates(static_cast<uint>(boost::target(*pathIterator, graph_)));
    std::cout << name_map[boost::source(*pathIterator, graph_)]
              << " (" << p0.x << "," << p0.y << "," << p0.z << ") " 
              << " -> " << name_map[boost::target(*pathIterator, graph_)]
              << " = " << boost::get(boost::edge_weight, graph_, *pathIterator)
              << " (" << p1.x << "," << p1.y << "," << p1.z << ") "
              << std::endl;
  }
  std::cout << std::endl;
  */
  
  std::cout << "Total distance (Dijkstra): " << distance_map[goal_vertex] << std::endl;

  PathCoordinates path_coordinates;
  for (PathType::reverse_iterator pathIterator = path.rbegin(); pathIterator != path.rend(); ++pathIterator)
  {
    const uint vertex_ID = static_cast<uint>(boost::source(*pathIterator, graph_));
    const Voxel coordinates = getCoordinates(vertex_ID);
    path_coordinates.push_back(coordinates);
    //std::cout << "vertex_ID: " << vertex_ID << "  " << coordinates.x << "," << coordinates.y << "," << coordinates.z << std::endl;
  }
  // Add the goal position, because the above path only
  // includes the first vertex of each edge.
  const Voxel goal_coordinates = getCoordinates(goal_vertex_ID);
  path_coordinates.push_back(goal_coordinates);
  //std::cout << "vertex_ID: " << goal_vertex_ID << "  " << goal_coordinates.x << "," << goal_coordinates.y << "," << goal_coordinates.z << std::endl;

  return path_coordinates;
}

const PathCoordinates VoxelMap3D::getShortestPathAstar(const Point& start_point,
                                                      const Point& goal_point)
{
  const Voxel start_cell = getVoxelIndex(start_point);
  const Voxel goal_cell = getVoxelIndex(goal_point);

  return getShortestPathAstar({start_cell.x, start_cell.y, start_cell.z},
                              {goal_cell.x, goal_cell.y, goal_cell.z});
}

const PathCoordinates VoxelMap3D::getShortestPathAstar(const std::initializer_list<IndexType>& start_cell,
                                                       const std::initializer_list<IndexType>& goal_cell)
{
  // Build graph, accounting for any obstacles.
  buildGraphFromOccupancyGrid();

  const std::vector<IndexType> start_cell_vec(start_cell);
  const std::vector<IndexType> goal_cell_vec(goal_cell);
  const uint start_vertex_ID = getLinearIndex(start_cell_vec.at(0), start_cell_vec.at(1), start_cell_vec.at(2));
  const uint goal_vertex_ID = getLinearIndex(goal_cell_vec.at(0), goal_cell_vec.at(1), goal_cell_vec.at(2));

  std::cout << "Finding shortest path using A*..." << std::endl;
  std::cout << "  Start: [" << start_cell_vec.at(0) << "," << start_cell_vec.at(1) << "," << start_cell_vec.at(2) << "], index = " << start_vertex_ID << std::endl;
  std::cout << "  Goal: [" << goal_cell_vec.at(0) << "," << goal_cell_vec.at(1) << "," << goal_cell_vec.at(2) << "], index = " << goal_vertex_ID << std::endl;

  checkStartAndGoalValidity(start_cell, goal_cell);

  std::vector<Vertex> predecessors(boost::num_vertices(graph_));
  std::vector<Weight> distances(boost::num_vertices(graph_));
  IndexMap index_map = boost::get(boost::vertex_index, graph_);
  PredecessorMap predecessor_map(&predecessors[0], index_map);
  DistanceMap distance_map(&distances[0], index_map);
  NameMap name_map = boost::get(boost::vertex_name, graph_);
  Vertex start_vertex = boost::vertex(static_cast<int>(start_vertex_ID), graph_);
  Vertex goal_vertex = boost::vertex(static_cast<int>(goal_vertex_ID), graph_);

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
                 euclidean_distance_heuristic<UndirectedGraph, Cost>(
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
 
  PathCoordinates path_coordinates;

  // Add the start position, because the A* shortest path only
  // includes the second vertex of each edge.
  const Voxel start_coordinates = getCoordinates(start_vertex_ID);
  path_coordinates.push_back(start_coordinates);
  std::list<Vertex>::iterator spi = shortest_path.begin();
  for (++spi; spi != shortest_path.end(); ++spi)
  {
    const uint vertex_ID = static_cast<uint>(*spi);
    const Voxel coordinates = getCoordinates(vertex_ID);
    path_coordinates.push_back(coordinates);
    //std::cout << "vertex_ID: " << vertex_ID << "  " << coordinates.x << "," << coordinates.y << "," << coordinates.z << "  " << color_names[colors_map[*spi]] << std::endl;
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

void VoxelMap3D::checkStartAndGoalValidity(const std::vector<IndexType>& start_cell, const std::vector<IndexType>& goal_cell)
{
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

  if (occupancy_grid_3d_.at(start_cell.at(0)).at(start_cell.at(1)).at(start_cell.at(2)).at(0) == OBSTACLE)
  {
    throw std::runtime_error("start cell is within an obstacle!");
  }

  if (occupancy_grid_3d_.at(goal_cell.at(0)).at(goal_cell.at(1)).at(goal_cell.at(2)).at(0) == OBSTACLE)
  {
    throw std::runtime_error("goal cell is within an obstacle!");
  }
}

const std::vector<ColorPoint> VoxelMap3D::getPathPoints(const PathCoordinates& positions)
{
  std::vector<ColorPoint> path;
  for (size_t i = 0; i < positions.size(); ++i)
  {
    ColorPoint point;
    point.x = pointcloud_dims_.min_x + positions.at(i).x*occupancy_grid_voxel_size_;
    point.y = pointcloud_dims_.min_y + positions.at(i).y*occupancy_grid_voxel_size_;
    point.z = pointcloud_dims_.min_z + positions.at(i).z*occupancy_grid_voxel_size_;

    // red
    point.r = 255; // uint_8
    point.g = 0;
    point.b = 0;

    path.push_back(point);
  }

  return path;
}

const ColorPointCloud VoxelMap3D::getPathPointCloud(const PathCoordinates& positions)
{
  ColorPointCloud cloud;

  for (size_t i = 0; i < positions.size(); ++i)
  {
    ColorPoint point;
    point.x = pointcloud_dims_.min_x + positions.at(i).x*occupancy_grid_voxel_size_;
    point.y = pointcloud_dims_.min_y + positions.at(i).y*occupancy_grid_voxel_size_;
    point.z = pointcloud_dims_.min_z + positions.at(i).z*occupancy_grid_voxel_size_;

    // red
    point.r = 255; // uint_8
    point.g = 0;
    point.b = 0;

    cloud.push_back(point);
  }

  return cloud;
}

void VoxelMap3D::viewGraph()
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

const ColorPointCloud VoxelMap3D::getOccupancyGrid3D()
{
  return getOccupancyGrid3D(occupancy_grid_voxel_size_);
}

const ColorPointCloud VoxelMap3D::getOccupancyGrid3D(const float point_separation)
{
  ColorPointCloud cloud;

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

  return cloud;
}

const ColorPointCloud VoxelMap3D::getAstarExpandedStates()
{
  return getAstarExpandedStates(occupancy_grid_voxel_size_);
}

const ColorPointCloud VoxelMap3D::getAstarExpandedStates(const float point_separation)
{
  // Color names:
  //   {"white", "gray", "green", "red", "black"};

  ColorPointCloud cloud;

  if (vertex_colors_.size() != (occupancy_grid_width_ * occupancy_grid_length_ * occupancy_grid_depth_))
  {
    std::cout << "No vertex color data in getAstarExpandedStates()" << std::endl;
    return cloud;
  }

  for (IndexType i = 0; i < occupancy_grid_width_; ++i)
  {
    for (IndexType j = 0; j < occupancy_grid_length_; ++j)
    {
      for (IndexType k = 0; k < occupancy_grid_depth_; ++k)
      {
        ColorPoint point;
        point.x = static_cast<float>(i) * point_separation;
        point.y = static_cast<float>(j) * point_separation;
        point.z = static_cast<float>(k) * point_separation;

        if (occupancy_grid_3d_.at(i).at(j).at(k).at(0) == OBSTACLE)
        {
          // Obstacle, color red
          //point.r = 255;
          //point.g = 0;
          //point.b = 0;
        }
        else
        {
          const uint vertex_ID = getLinearIndex(i, j, k);
          const int color_idx = vertex_colors_.at(vertex_ID);

          if (color_idx == 0)
          {
            // white (un-discovered)
            //point.r = 255;
            //point.g = 255;
            //point.b = 255;
            //point.a = 10; // transparent
          }
          else if (color_idx == 1)
          {
            // gray
            point.r = 105;
            point.g = 105;
            point.b = 105;
            point.a = 180;
          }
          else if (color_idx == 4)
          {
            // black
            point.r = 0;
            point.g = 0;
            point.b = 0;
            point.a = 230;
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
  }

  return cloud;
}

const Voxel VoxelMap3D::getVoxelIndex(const Point& point)
{
  const IndexType i = static_cast<IndexType>(std::round((point.x - pointcloud_dims_.min_x) / occupancy_grid_voxel_size_));
  const IndexType j = static_cast<IndexType>(std::round((point.y - pointcloud_dims_.min_y) / occupancy_grid_voxel_size_));
  const IndexType k = static_cast<IndexType>(std::round((point.z - pointcloud_dims_.min_z) / occupancy_grid_voxel_size_));
  return Voxel(i, j, k);
}

// ToDo: Re-factor point sep?
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
  //std::cout << "radius = " << radius << std::endl;
  //std::cout << "radius_voxels = " << radius_voxels << std::endl;

  // Get the x,y,z voxel indices for a bounding cube.
  // We need the indices to be signed integer because for index 0,
  // the range() function should be a range of negative to positive values.
  // The invalid indices (outside the Occupancy Grid) are ignored later.
  const std::vector<int> x_indices = range<int>(center_point_idx.x - radius_voxels, center_point_idx.x + radius_voxels);
  const std::vector<int> y_indices = range<int>(center_point_idx.y - radius_voxels, center_point_idx.y + radius_voxels);
  const std::vector<int> z_indices = range<int>(center_point_idx.z - radius_voxels, center_point_idx.z + radius_voxels);
  //std::cout << "center_point_idx: " << center_point_idx.x << ", " << center_point_idx.y << ", " << center_point_idx.z << std::endl;
  //printStdVector(x_indices);
  //printStdVector(y_indices);
  //printStdVector(z_indices);

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

void VoxelMap3D::removeAllGraphEdges()
{
  BGL_FORALL_VERTICES(v, graph_, UndirectedGraph)
  {
    boost::clear_vertex(v, graph_);
  }
}

/*
void VoxelMap3D::removeAllGraphEdges()
{
  const std::vector<int> adjacent_vertices = getAdjacentVertices(22, graph_);
  for (size_t i = 0; i < adjacent_vertices.size(); ++i)
  {
    boost::remove_edge(boost::vertex(22, graph_),boost::vertex(adjacent_vertices.at(i), graph_), graph_);
  }
}
*/

std::pair<VoxelMap3D::VertexIterator,VoxelMap3D::VertexIterator> VoxelMap3D::getVertices() const
{
  return boost::vertices(graph_);
}

//----------------------------------------------------------------------------//

// This is used by the A* Visitor
const Voxel getVoxelCoordinates(const std::vector<IndexType>& occupancy_grid_size, const uint index)
{
  const uint i_step = static_cast<uint>(occupancy_grid_size.at(0));
  const uint j_step = static_cast<uint>(occupancy_grid_size.at(1));

  const IndexType x = static_cast<IndexType>(index % i_step); // modulus (remainder)
  const IndexType y = static_cast<IndexType>((index - x) / i_step % j_step);
  const IndexType z = static_cast<IndexType>(((index - x) / i_step - y) / j_step);

  return Voxel(x, y, z);
}

void printPath(const PathCoordinates& path)
{
  std::cout << "Path co-ordinates: " << std::endl;
  for (size_t i = 0; i < path.size(); ++i)
  {
    std::cout << "  " << path.at(i).x << "," << path.at(i).y << "," << path.at(i).z << std::endl;
  }
}

//----------------------------------------------------------------------------//
