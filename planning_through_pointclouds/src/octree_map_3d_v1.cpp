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

#include <planning_through_pointclouds/octree_map_3d_v1.h>

#include <iostream> // cout
#include <queue> // constant-time lookup of largest element
#include <algorithm> // std::reverse
#include <unordered_map>

#include <planning_through_pointclouds/utils.h> // range, printStdVector
#include <planning_through_pointclouds/pcl_utils.h> // getPointCloudBounds

OctreeMap3D::OctreeMap3D(PointCloud::Ptr& input_pointcloud, const double voxel_size, const double collision_radius, const uint connectivity)
  : occupancy_grid_voxel_size_(voxel_size)
  , collision_radius_(collision_radius)
  , graph_connectivity_(connectivity)
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

  std::cout << "Generating Octree from PointCloud... " << std::endl;
  octree_ = pcl::octree::OctreePointCloudSearch<Point>::Ptr(new pcl::octree::OctreePointCloudSearch<Point>(occupancy_grid_voxel_size_));
  octree_->setInputCloud(input_pointcloud); // set ptr to input data
  octree_->addPointsFromInputCloud();
}

const std::vector<OctreeMap3D::Edge> OctreeMap3D::getNeighbours(const uint vertex_ID)
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
        //if (checkCollision(getPoint(neighbour_voxels.at(i))) == false)
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
        //if (checkCollision(getPoint(neighbour_voxels.at(i))) == false)
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
        //if (checkCollision(getPoint(neighbour_voxels.at(i))) == false)
        {
          const uint neighbour_vertex_ID = getLinearIndex(neighbour_x, neighbour_y, neighbour_z);
          edges.push_back(Edge(neighbour_vertex_ID, weight));
        }
      }
    }
  }

  return edges;
}

const PathCoordinates OctreeMap3D::getShortestPath(const Point& start_point,
                                                   const Point& goal_point,
                                                   const bool use_astar)
{
  const Voxel start_cell = getVoxelIndex(start_point);
  const Voxel goal_cell = getVoxelIndex(goal_point);

  return getShortestPath({start_cell.x, start_cell.y, start_cell.z},
                         {goal_cell.x, goal_cell.y, goal_cell.z},
                         use_astar);
}

const PathCoordinates OctreeMap3D::getShortestPath(const std::initializer_list<IndexType>& start_cell,
                                                   const std::initializer_list<IndexType>& goal_cell,
                                                   const bool use_astar)
{
  // this is slower
  //std::unordered_map<uint, Weight> distances_map_; // hash table

  // Reset variables
  {
    const uint vertex_count = occupancy_grid_width_ * occupancy_grid_length_ * occupancy_grid_depth_;

    visited_.clear();
    distances_.clear();
    predecessors_.clear();

    visited_.resize(vertex_count);
    distances_.resize(vertex_count);
    predecessors_.resize(vertex_count);

    for (uint i = 0; i < vertex_count; ++i)
    {
      visited_.at(i) = false;

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
  //distances_map_.emplace(start_vertex_ID, 0);

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

    visited_.at(u) = true;

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
      if (visited_.at(neighbours.at(i).vertex))
      {
        continue;
      }

      if (checkCollision(getPoint(getCoordinates(neighbours.at(i).vertex))) == true)
      {
        // In-collision
        continue;
      }

      // Alternative distance = distance to current vertex (u) + distance to neighbour vertex (it's weight)
      const float alt_dist = distances_.at(u) + neighbours.at(i).weight;
      //const float alt_dist = distances_map_[u] + neighbours.at(i).weight;

      if (alt_dist < distances_.at(neighbours.at(i).vertex))
      //if (alt_dist < distances_[neighbours.at(i).vertex])
      {
        distances_.at(neighbours.at(i).vertex) = alt_dist;
        //distances_map_.emplace(neighbours.at(i).vertex, alt_dist);

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
    //std::cout << "  Total distance (Dijkstra): " << distances_map_[goal_vertex_ID] << std::endl;
  }
  else
  {
    std::cout << "  Total distance (A*): " << distances_.at(goal_vertex_ID) << std::endl;
    //std::cout << "  Total distance (A*): " << distances_map_[goal_vertex_ID] << std::endl;
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

void OctreeMap3D::checkStartAndGoalValidity(const std::vector<IndexType>& start_cell, const std::vector<IndexType>& goal_cell)
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

  if (checkCollision(getPoint(Voxel(start_cell.at(0), start_cell.at(1), start_cell.at(2)))) == true)
  {
    throw std::runtime_error("start cell is within an obstacle!");
  }

  if (checkCollision(getPoint(Voxel(goal_cell.at(0), goal_cell.at(1), goal_cell.at(2)))) == true)
  {
    throw std::runtime_error("goal cell is within an obstacle!");
  }
}

const std::vector<ColorPoint> OctreeMap3D::getPathPoints(const PathCoordinates& positions)
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

const ColorPointCloud OctreeMap3D::getPathPointCloud(const PathCoordinates& positions)
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

const Voxel OctreeMap3D::getVoxelIndex(const Point& point)
{
  const IndexType i = static_cast<IndexType>(std::round((point.x - pointcloud_dims_.min_x) / occupancy_grid_voxel_size_));
  const IndexType j = static_cast<IndexType>(std::round((point.y - pointcloud_dims_.min_y) / occupancy_grid_voxel_size_));
  const IndexType k = static_cast<IndexType>(std::round((point.z - pointcloud_dims_.min_z) / occupancy_grid_voxel_size_));
  return Voxel(i, j, k);
}

const Point OctreeMap3D::getPoint(const Voxel& index)
{
  Point point;
  point.x = pointcloud_dims_.min_x + static_cast<float>(index.x) * occupancy_grid_voxel_size_; // ToDo: implicit lattice voxel size ?
  point.y = pointcloud_dims_.min_y + static_cast<float>(index.y) * occupancy_grid_voxel_size_;
  point.z = pointcloud_dims_.min_z + static_cast<float>(index.z) * occupancy_grid_voxel_size_;
  return point;
}

const uint OctreeMap3D::getLinearIndex(const IndexType i, const IndexType j, const IndexType k)
{
  const uint i_step = static_cast<uint>(occupancy_grid_width_);
  const uint j_step = static_cast<uint>(occupancy_grid_length_);

  return static_cast<uint>(i) + i_step*(static_cast<uint>(j) + j_step*(static_cast<uint>(k)));
}

const Voxel OctreeMap3D::getCoordinates(const uint index)
{
  const uint i_step = static_cast<uint>(occupancy_grid_width_);
  const uint j_step = static_cast<uint>(occupancy_grid_length_);

  const IndexType x = static_cast<IndexType>(index % i_step); // modulus (remainder)
  const IndexType y = static_cast<IndexType>((index - x) / i_step % j_step);
  const IndexType z = static_cast<IndexType>(((index - x) / i_step - y) / j_step);

  return Voxel(x, y, z);
}

const float OctreeMap3D::euclideanDistance(const uint v0, const uint v1)
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

const bool OctreeMap3D::checkCollision(const Point& point)
{
  std::vector<int> nn_indices;
  std::vector<float> nn_dists;

  const uint max_nn = 1; // 1 or more neighbours = collision

  if (octree_->radiusSearch(point, collision_radius_, nn_indices, nn_dists, max_nn))
  {
    return true;
  }

  return false;
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
