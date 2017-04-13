//
// Voxel Map 3D v3
//
// A 3D occupancy grid.
// Planning with Dijkstra or A*.
// 
// Uses an implicit graph.
// The only information about the vertices is a set of "distances", "predecessors" and a queue.
// The edges are found online by collision-checking the occupancy grid and motion is restricted to a lattice.
// 
// David Butterworth
//

#include <planning_through_pointclouds/voxel_map_3d_v3.h>

#include <iostream> // cout
#include <queue> // constant-time lookup of largest element
#include <algorithm> // std::reverse

#include <pcl/common/distances.h> // euclideanDistance()

#include <planning_through_pointclouds/utils.h> // range, printStdVector
#include <planning_through_pointclouds/pcl_utils.h> // loadPointCloud, downsamplePointCloud

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
  initializeVector<uint8_t>(occupancy_grid_3d_, occupancy_grid_width_, occupancy_grid_length_, occupancy_grid_depth_, 1, FREE_SPACE);
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
  initializeVector<uint8_t>(occupancy_grid_3d_, occupancy_grid_width_, occupancy_grid_length_, occupancy_grid_depth_, 1, FREE_SPACE);

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

  const uint num_vertices = occupancy_grid_width_ * occupancy_grid_length_ * occupancy_grid_depth_;

  // We add 6 and duplicates aren't removed
  const uint estimated_num_edges = 6 * num_vertices;

  const uint num_bytes_each = 4; // 4 bytes for uint or float

  // distances + predecessors
  const uint vertices_memory_mb = (num_vertices * 2 * num_bytes_each)/ 1000000;

  // Estimate: add 6 edges for each vertice, being 3 numbers
  const uint edges_memory_mb = 0; //estimated_num_edges * 3 * num_bytes_each / 1000000;

  std::cout << "Estimated memory requirements: " << std::endl;
  std::cout << "  Occupancy Grid: " << num_vertices*14*num_bytes_each << " MB" << std::endl;
  std::cout << "  Graph vertices: " << vertices_memory_mb << " MB" << std::endl;
  std::cout << "  Graph edges: " << edges_memory_mb << " MB" << std::endl;
}

void VoxelMap3D::addGraphVertices()
{
  const uint vertex_count = occupancy_grid_width_ * occupancy_grid_length_ * occupancy_grid_depth_;

  //not_visited_.resize(vertex_count);
  distances_.resize(vertex_count);
  predecessors_.resize(vertex_count);

  for (uint i = 0; i < vertex_count; ++i)
  {
    // All vertices are not visited yet
    //not_visited_.at(i) = i;

    // Distance to each vertex is "INF"
    distances_.at(i) = std::numeric_limits<uint>::max();

    predecessors_.at(i) = 0; // un-defined
  }

  std::cout << "Added " << vertex_count << " vertices" << std::endl;
}

const std::vector<VoxelMap3D::Edge> VoxelMap3D::getNeighbours(const uint vertex_ID)
{
  std::vector<Edge> edges;

  const Voxel voxel_idx = getCoordinates(vertex_ID);

  const IndexType x = voxel_idx.x;
  const IndexType y = voxel_idx.y;
  const IndexType z = voxel_idx.z;

  // Add graph edges for voxels that
  // are connected by their faces.
  {
    const Weight weight = 1.0;

    std::vector<Voxel> neighbour_voxels;

    // Add edges in all directions.
    // The implementation of Dijkstra and A* requires an edge going
    // each way, for an un-directed graph.

    neighbour_voxels.push_back(Voxel(x-1, y, z));
    neighbour_voxels.push_back(Voxel(x+1, y, z));
    neighbour_voxels.push_back(Voxel(x, y-1, z));
    neighbour_voxels.push_back(Voxel(x, y+1, z));
    neighbour_voxels.push_back(Voxel(x, y, z-1));
    neighbour_voxels.push_back(Voxel(x, y, z+1));
    
    for (IndexType i = 0; i < 6; ++i)
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
          edges.push_back(Edge(neighbour_vertex_ID, weight));
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
          edges.push_back(Edge(neighbour_vertex_ID, weight));
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
          edges.push_back(Edge(neighbour_vertex_ID, weight));
        }
      }
    }
  }

  return edges;
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

const PathCoordinates VoxelMap3D::getShortestPath(const Point& start_point,
                                                  const Point& goal_point,
                                                  const bool use_astar)
{
  const Voxel start_cell = getVoxelIndex(start_point);
  const Voxel goal_cell = getVoxelIndex(goal_point);

  return getShortestPath({start_cell.x, start_cell.y, start_cell.z},
                         {goal_cell.x, goal_cell.y, goal_cell.z},
                         use_astar);
}

const PathCoordinates VoxelMap3D::getShortestPath(const std::initializer_list<IndexType>& start_cell,
                                                  const std::initializer_list<IndexType>& goal_cell,
                                                  const bool use_astar)
{
  // Reset variables
  {
    const uint vertex_count = occupancy_grid_width_ * occupancy_grid_length_ * occupancy_grid_depth_;
    for (uint i = 0; i < vertex_count; ++i)
    {
      // Distance to each vertex is "INF"
      distances_.at(i) = std::numeric_limits<uint>::max();

      predecessors_.at(i) = 0; // un-defined
    }
  }

  if (!use_astar)
  {
    std::cout << "Finding shortest path using Dijkstra..." << std::endl;
  }
  else
  {
    std::cout << "Finding shortest path using A*..." << std::endl;
  }

  const std::vector<IndexType> start_cell_vec(start_cell);
  const std::vector<IndexType> goal_cell_vec(goal_cell);
  const uint start_vertex_ID = getLinearIndex(start_cell_vec.at(0), start_cell_vec.at(1), start_cell_vec.at(2));
  const uint goal_vertex_ID = getLinearIndex(goal_cell_vec.at(0), goal_cell_vec.at(1), goal_cell_vec.at(2));

  std::cout << "  Start: [" << start_cell_vec.at(0) << "," << start_cell_vec.at(1) << "," << start_cell_vec.at(2) << "], index = " << start_vertex_ID << std::endl;
  std::cout << "  Goal: [" << goal_cell_vec.at(0) << "," << goal_cell_vec.at(1) << "," << goal_cell_vec.at(2) << "], index = " << goal_vertex_ID << std::endl;

  checkStartAndGoalValidity(start_cell, goal_cell);

  // The queue of un-visited vertices initially contains
  // only the starting vertex with a cost of zero
  std::priority_queue<Vertex, std::vector<Vertex>, prioritize> pq;
  pq.push(Vertex(start_vertex_ID, 0));

  distances_.at(start_vertex_ID) = 0;

  uint visited_vertices_count = 0;
  bool found_goal = false;
  while (!pq.empty() && !found_goal)
  {
    // Get the vertex with min distance
    // and remove this vertex from queue
    const Vertex current_vertex = pq.top(); // vertex index with cost
    pq.pop();
    const uint u = current_vertex.vertex;

    // Debug:
    //const Voxel coordinates = getCoordinates(u);
    //std::cout << " u = " << u;
    //std::cout << "  co-ords: " << coordinates.x << "," << coordinates.y << "," << coordinates.z << std::endl;

    if (u == goal_vertex_ID)
    {
      // Reached the goal, exit without expanding all vertices
      found_goal = true;
      break;
    }

    visited_vertices_count++;

    // Get neighbouring vertices and the cost to reach them
    const std::vector<Edge> neighbours = getNeighbours(u);

    if (neighbours.size() == 0)
    {
      std::cout << "  ERROR, no neighbours" << std::endl;
      const Voxel coordinates = getCoordinates(u);
      std::cout << "  co-ords: " << coordinates.x << "," << coordinates.y << "," << coordinates.z << std::endl;
    }
    // Check the distance to each neighbouring vertex
    for (size_t i = 0; i < neighbours.size(); ++i)
    {
      // Alternative distance = distance to current vertex (u) + distance to neighbour vertex (it's weight)
      const float alt_dist = distances_.at(u) + neighbours.at(i).weight;

      if (alt_dist < distances_.at(neighbours.at(i).vertex))
      {
        distances_.at(neighbours.at(i).vertex) = alt_dist;
        predecessors_.at(neighbours.at(i).vertex) = u;

        float heuristic = 0.0;
        if (use_astar)
        {
          // Cost to goal
          heuristic = euclideanDistance(neighbours.at(i).vertex, goal_vertex_ID);
          // For tie-breaking.
          // This provides big speed-up too, but breaks 'admissibility'.
          heuristic *= 1.1;
        }

        // Add this neighbouring vertex to the queue to be examined
        pq.push(Vertex(neighbours.at(i).vertex, alt_dist + heuristic));
      }
    }
  }
  std::cout << "  Visited " << visited_vertices_count << " vertices" << std::endl;

  // Extract the shortest path by iterating backwards
  // through the vertex predecessors
  std::vector<uint> path_vertices;
  path_vertices.push_back(goal_vertex_ID);
  uint u = goal_vertex_ID;
  while (1)
  {
    u = predecessors_[u];
    path_vertices.push_back(u);
    if (u == start_vertex_ID)
    {
      break;
    }
  }
  std::reverse(path_vertices.begin(), path_vertices.end());

  /*
  // Print the shortest path
  std::cout << "Shortest path (Dijkstra): " << std::endl;
  std::cout << path_vertices.at(0);
  for (uint i = 1; i < path_vertices.size(); ++i)
  {
    std::cout << " --> " << path_vertices.at(i);
  }
  std::cout << std::endl;
  */

  if (!use_astar)
  {
    std::cout << "  Total distance (Dijkstra): " << distances_.at(goal_vertex_ID) << std::endl;
  }
  else
  {
    std::cout << "  Total distance (A*): " << distances_.at(goal_vertex_ID) << std::endl;
  }

  PathCoordinates path_coordinates;
  for (size_t i = 0; i < path_vertices.size(); ++i)
  {
    const Voxel coordinates = getCoordinates(path_vertices.at(i));
    path_coordinates.push_back(coordinates);
    //std::cout << "vertex_ID: " << path_vertices.at(i) << "  " << coordinates.x << "," << coordinates.y << "," << coordinates.z << std::endl;
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

const float VoxelMap3D::euclideanDistance(const uint v0, const uint v1)
{
  const Voxel v0_coordinates = getCoordinates(v0);
  const Voxel v1_coordinates = getCoordinates(v1);

  const float v0_x = static_cast<float>(v0_coordinates.x);
  const float v0_y = static_cast<float>(v0_coordinates.y);
  const float v0_z = static_cast<float>(v0_coordinates.z);

  const float v1_x = static_cast<float>(v1_coordinates.x);
  const float v1_y = static_cast<float>(v1_coordinates.y);
  const float v1_z = static_cast<float>(v1_coordinates.z);

  const float dx = v1_x - v0_x;
  const float dy = v1_y - v0_y;
  const float dz = v1_z - v0_z;

  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

//----------------------------------------------------------------------------//

void printPath(const PathCoordinates& path)
{
  std::cout << "Path co-ordinates: " << std::endl;
  for (size_t i = 0; i < path.size(); ++i)
  {
    std::cout << "  " << path.at(i).x << "," << path.at(i).y << "," << path.at(i).z << std::endl;
  }
}

//----------------------------------------------------------------------------//
