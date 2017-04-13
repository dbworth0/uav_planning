//
// Test 3 iterations of Dijkstra algorithm
//
// David Butterworth
//

#include <iostream> // cout
#include <string>
#include <vector>

#include <limits>
#include <queue> // constant-time lookup of largest element
#include <list>
#include <algorithm>    // std::reverse
//#include <planning_through_pointclouds/voxel_map_3d_v2.h>

/*
#include <stdlib.h>
#include <stdio.h>
#include <string.h> // strncmp

int parseLine(char* line){
    // This assumes that a digit will be found and the line ends in " Kb".
    int i = strlen(line);
    const char* p = line;
    while (*p <'0' || *p > '9') p++;
    line[i-3] = '\0';
    i = atoi(p);
    return i;
}

int getVirtualMemoryValue(){ //Note: this value is in KB!
    FILE* file = fopen("/proc/self/status", "r");
    int result = -1;
    char line[128];

    while (fgets(line, 128, file) != NULL){
        if (strncmp(line, "VmSize:", 7) == 0){
            result = parseLine(line);
            break;
        }
    }
    fclose(file);
    return result;
}

int getPhysMemValue(){ //Note: this value is in KB!
    FILE* file = fopen("/proc/self/status", "r");
    int result = -1;
    char line[128];

    while (fgets(line, 128, file) != NULL){
        if (strncmp(line, "VmRSS:", 6) == 0){
            result = parseLine(line);
            break;
        }
    }
    fclose(file);
    return result;
}

#include <sys/types.h>
#include <sys/sysinfo.h>
*/


std::vector<uint> not_visited;
std::vector<uint> distances;
std::vector<uint> predecessors;

struct Edge
{
  uint v0;
  uint v1;
  uint weight;
  Edge(const uint _v0, const uint _v1, const int _weight) : v0(_v0), v1(_v1), weight(_weight) {}
};

std::vector<Edge> adjacency_list_;

// Orig implementation, slow on large graphs
int dijkstra(const uint start_vertex, const uint goal_vertex, const uint num_vertices)
{
  // Allocate memory
  not_visited.resize(num_vertices);
  distances.resize(num_vertices);
  predecessors.resize(num_vertices);

  for (uint i = 0; i < num_vertices; ++i)
  {
    // All vertices are not visited yet
    not_visited.at(i) = i;

    // Distance to each vertex is "INF"
    distances.at(i) = std::numeric_limits<uint>::max();

    predecessors.at(i) = 0; // un-defined
  }

  distances.at(start_vertex) = 0;

  uint count = 0; // for debug
  bool found_goal = false;
  while (!not_visited.empty() && !found_goal)
  {
    std::cout << "Iteration: " << count << std::endl;
    count++;

    // debug:
    std::cout << "  not_visited.size() = " << not_visited.size() << std::endl;
    //for (size_t i = 0; i < not_visited.size(); ++i)
    //{
    //  std::cout << not_visited.at(i) << ", ";
    //}
    //std::cout << std::endl;

    // Get the vertex with min distance
    // and remove this vertex from queue
    uint u = 0;
    uint min_distance = std::numeric_limits<uint>::max();
    uint min_distance_idx = 0;
    for (size_t i = 0; i < not_visited.size(); ++i)
    {
      if (distances.at(not_visited.at(i)) < min_distance)
      {
        u = not_visited.at(i);
        min_distance_idx = i;
      }
    }
    //if (min_distance_idx == goal_vertex)
    if (u == goal_vertex)
    {
      // Reached the goal, exit early without expanding all vertices
      found_goal = true;
      break;
    }
    not_visited.erase(not_visited.begin() + min_distance_idx);
    //std::cout << "  vertex in queue with min distance: " << u << std::endl;

    // Get neighbouring vertices and the cost to reach them
    std::vector<std::pair<uint,uint> > neighbours;
    //std::cout << " u = " << u << std::endl;
    for (size_t i = 0; i < adjacency_list_.size(); ++i)
    {
      //std::cout << "adjacency_list_.at(i).v0 = " << adjacency_list_.at(i).v0 << std::endl;
      //std::cout << "adjacency_list_.at(i).v1 = " << adjacency_list_.at(i).v1 << std::endl;
      if (adjacency_list_.at(i).v0 == u)
      {
        neighbours.push_back( std::make_pair(adjacency_list_.at(i).v1, adjacency_list_.at(i).weight) );
      }
      else if (adjacency_list_.at(i).v1 == u)
      {
        neighbours.push_back( std::make_pair(adjacency_list_.at(i).v0, adjacency_list_.at(i).weight) );
      }
    }
    // debug:
    //std::cout << "  neighbour vertices: ";
    //for (size_t i = 0; i < neighbours.size(); ++i)
    //{
    //  std::cout << neighbours.at(i).first << " (weight = " << neighbours.at(i).second << ") " << ", ";
    //}
    //std::cout << std::endl;
    if (neighbours.size() == 0) std::cout << "ERROR, no neighbours" << std::endl;

    // Check the distance to each neighbour
    for (size_t i = 0; i < neighbours.size(); ++i)
    {
      //std::cout << "  checking neighbour idx" << i << std::endl;
      // Alternative distance = distance to current vertex (u) + distance to neighbour vertex (it's weight)
      const uint alt_dist = distances.at(u) + neighbours.at(i).second;
      //std::cout << "  alt_dist = " << alt_dist << std::endl;
      if (alt_dist < distances.at(neighbours.at(i).first))
      {
        distances.at(neighbours.at(i).first) = alt_dist;
        predecessors.at(neighbours.at(i).first) = u;
        //std::cout << "  distances[" << neighbours.at(i).first << "] = " << alt_dist << std::endl;
      }
    }
  }
}



class prioritize
{
  public: bool operator ()(const std::pair<uint,uint>& v0, const std::pair<uint,uint>& v1)
  {
    return v0.second > v1.second;
  }
};



int dijkstraUsingQueue(const uint start_vertex, const uint goal_vertex, const uint num_vertices)
{
  // Allocate memory
  //not_visited.resize(num_vertices);
  distances.resize(num_vertices);
  predecessors.resize(num_vertices);

  for (uint i = 0; i < num_vertices; ++i)
  {
    // All vertices are not visited yet
    //not_visited.at(i) = i;

    // Distance to each vertex is "INF"
    distances.at(i) = std::numeric_limits<uint>::max();

    predecessors.at(i) = 0; // un-defined
  }

  // The queue of un-visited vertices initially
  // contains only the starting vertex
  std::priority_queue<std::pair<uint,uint>, std::vector<std::pair<uint,uint> >, prioritize> pq;
  pq.push(std::make_pair(start_vertex, 0));

  distances.at(start_vertex) = 0;

  uint count = 0; // for debug
  bool found_goal = false;
  //while (!not_visited.empty() && !found_goal)
  while (!pq.empty() && !found_goal)
  {
    std::cout << "Iteration: " << count << std::endl;
    count++;

    const std::pair<uint,uint> current_vertex = pq.top(); // vertex index with cost
    pq.pop();
    uint u = current_vertex.first;

    //if (min_distance_idx == goal_vertex)
    if (u == goal_vertex)
    {
      // Reached the goal, exit early without expanding all vertices
      found_goal = true;
      break;
    }


    // Get neighbouring vertices and the cost to reach them
    std::vector<std::pair<uint,uint> > neighbours;
    //std::cout << " u = " << u << std::endl;
    for (size_t i = 0; i < adjacency_list_.size(); ++i)
    {
      //std::cout << "adjacency_list_.at(i).v0 = " << adjacency_list_.at(i).v0 << std::endl;
      //std::cout << "adjacency_list_.at(i).v1 = " << adjacency_list_.at(i).v1 << std::endl;
      if (adjacency_list_.at(i).v0 == u)
      {
        neighbours.push_back( std::make_pair(adjacency_list_.at(i).v1, adjacency_list_.at(i).weight) );
      }
      else if (adjacency_list_.at(i).v1 == u)
      {
        neighbours.push_back( std::make_pair(adjacency_list_.at(i).v0, adjacency_list_.at(i).weight) );
      }
    }
    // debug:
    //std::cout << "  neighbour vertices: ";
    //for (size_t i = 0; i < neighbours.size(); ++i)
    //{
    //  std::cout << neighbours.at(i).first << " (weight = " << neighbours.at(i).second << ") " << ", ";
    //}
    //std::cout << std::endl;
    if (neighbours.size() == 0) std::cout << "ERROR, no neighbours" << std::endl;

    // Check the distance to each neighbour
    for (size_t i = 0; i < neighbours.size(); ++i)
    {
      //std::cout << "  checking neighbour idx" << i << std::endl;
      // Alternative distance = distance to current vertex (u) + distance to neighbour vertex (it's weight)
      const uint alt_dist = distances.at(u) + neighbours.at(i).second;
      //std::cout << "  alt_dist = " << alt_dist << std::endl;
      if (alt_dist < distances.at(neighbours.at(i).first))
      {
        distances.at(neighbours.at(i).first) = alt_dist;
        predecessors.at(neighbours.at(i).first) = u;
        //std::cout << "  distances[" << neighbours.at(i).first << "] = " << alt_dist << std::endl;

        // Add this neighbouring vertex to the queue to be examined
        pq.push(std::make_pair(neighbours.at(i).first, alt_dist));
      }
    }
  }
}

struct Vertex
{
  uint vertex;
  uint weight;
  Vertex(const uint _vertex, const int _weight) : vertex(_vertex), weight(_weight) {}
};

//std::vector< std::list<Vertex> > staggered_adjacency_list;
std::vector< std::vector<Vertex> > staggered_adjacency_list;

// Using adjacency list with only num_vertices number of entries.
int dijkstraUsingQueue2(const uint start_vertex, const uint goal_vertex, const uint num_vertices)
{
  // Allocate memory
  //not_visited.resize(num_vertices);
  distances.resize(num_vertices);
  predecessors.resize(num_vertices);

  for (uint i = 0; i < num_vertices; ++i)
  {
    // Distance to each vertex is "INF"
    distances.at(i) = std::numeric_limits<uint>::max();

    predecessors.at(i) = 0; // un-defined
  }

  // The queue of un-visited vertices initially
  // contains only the starting vertex
  std::priority_queue<std::pair<uint,uint>, std::vector<std::pair<uint,uint> >, prioritize> pq;
  pq.push(std::make_pair(start_vertex, 0));

  distances.at(start_vertex) = 0;

  uint count = 0; // for debug
  bool found_goal = false;
  //while (!not_visited.empty() && !found_goal)
  while (!pq.empty() && !found_goal)
  {
    std::cout << "Iteration: " << count << std::endl;
    count++;

    const std::pair<uint,uint> current_vertex = pq.top(); // vertex index with cost
    pq.pop();
    uint u = current_vertex.first;

    std::cout << " u = " << u << std::endl;
    std::cout << "staggered_adjacency_list.at(u): " << staggered_adjacency_list.at(u).size() << std::endl;

    //if (min_distance_idx == goal_vertex)
    if (u == goal_vertex)
    {
      // Reached the goal, exit early without expanding all vertices
      found_goal = true;
      break;
    }

    // Get neighbouring vertices and the cost to reach them

    if (staggered_adjacency_list.at(u).size() == 0) std::cout << "ERROR, no neighbours" << std::endl;

    // Check the distance to each neighbour
    for (size_t i = 0; i < staggered_adjacency_list.at(u).size(); ++i)
    {
      std::cout << "  checking neighbour idx" << i << std::endl;

      // Alternative distance = distance to current vertex (u) + distance to neighbour vertex (it's weight)
      //const uint alt_dist = distances.at(u) + neighbours.at(i).second;
      const uint alt_dist = distances.at(u) + staggered_adjacency_list.at(u).at(i).weight;

      //std::cout << "  alt_dist = " << alt_dist << std::endl;
      if (alt_dist < distances.at(staggered_adjacency_list.at(u).at(i).vertex))
      {
        distances.at(staggered_adjacency_list.at(u).at(i).vertex) = alt_dist;
        predecessors.at(staggered_adjacency_list.at(u).at(i).vertex) = u;
        //std::cout << "  distances[" << neighbours.at(i).first << "] = " << alt_dist << std::endl;

        // Add this neighbouring vertex to the queue to be examined
        pq.push(std::make_pair(staggered_adjacency_list.at(u).at(i).vertex, alt_dist));
      }
    }
  }
}

//----------------------------------------------------------------------------//

int main(int argc, char **argv)
{
    /*
  std::cout << "System info: " << std::endl;
#if __GNUC__
#if __x86_64__ || __ppc64__
  std::cout << "  64-bit" << std::endl;
#else
  std::cout << "  32-bit" << std::endl;
#endif
#endif
  // 8 bytes, 18446744073709551615
  std::cout << "Size of long unsigned int: " << sizeof(long unsigned int) << " bytes, max value = " << std::numeric_limits<long unsigned int>::max() << std::endl;

  std::cout << "Size of int: " << sizeof(int) << " bytes, max value = " << std::numeric_limits<int>::max() << std::endl;

  // 4 bytes, 4294967295
  std::cout << "Size of uint32_t: " << sizeof(uint32_t) << " bytes, max value = " << std::numeric_limits<uint32_t>::max() << std::endl;
  std::cout << "Size of uint16_t: " << sizeof(uint16_t) << " bytes, max value = " << std::numeric_limits<uint16_t>::max() << std::endl;
  std::cout << "Size of uint8_t: " << sizeof(uint8_t) << " bytes, max value = " << static_cast<int>(std::numeric_limits<uint8_t>::max()) << std::endl;
  */

  
  uint num_vertices = 12;

  const int w = 1;

  const uint start = 11;
  const uint goal = 5;

  /*
  adjacency_list_.push_back(Edge(0, 1, w));
  adjacency_list_.push_back(Edge(0, 3, w));

  adjacency_list_.push_back(Edge(1, 4, w));
  adjacency_list_.push_back(Edge(1, 2, w));

  adjacency_list_.push_back(Edge(2, 5, w));

  adjacency_list_.push_back(Edge(3, 4, w));
  adjacency_list_.push_back(Edge(3, 6, w));

  adjacency_list_.push_back(Edge(4, 5, w));
  adjacency_list_.push_back(Edge(4, 7, w));

  adjacency_list_.push_back(Edge(5, 8, w));

  adjacency_list_.push_back(Edge(6, 7, w));
  adjacency_list_.push_back(Edge(6, 9, w));

  adjacency_list_.push_back(Edge(7, 8, w));
  adjacency_list_.push_back(Edge(7, 10, w));

  adjacency_list_.push_back(Edge(8, 11, w));

  adjacency_list_.push_back(Edge(9, 10, w));

  adjacency_list_.push_back(Edge(10, 11, w));



  // The orig implementation with early cancel takes 7 iterations out of 12 vertices.
  // vs. the version using queue takes only 4 iterations!
  dijkstra(start, goal, num_vertices);
  //dijkstraUsingQueue(start, goal, num_vertices);
  */

  // This also takes 4 iterations.
  // For an un-directed graph, we need double the number of edges,
  // however it means the adjacency list has a fixed number
  // of entries which makes it faster to index into.
  staggered_adjacency_list.resize(num_vertices);

  staggered_adjacency_list.at(0).push_back(Vertex(1, w));
  staggered_adjacency_list.at(0).push_back(Vertex(3, w));
  staggered_adjacency_list.at(1).push_back(Vertex(0, w));
  staggered_adjacency_list.at(3).push_back(Vertex(0, w));

  staggered_adjacency_list.at(1).push_back(Vertex(4, w));
  staggered_adjacency_list.at(1).push_back(Vertex(2, w));
  staggered_adjacency_list.at(4).push_back(Vertex(1, w));
  staggered_adjacency_list.at(2).push_back(Vertex(1, w));

  staggered_adjacency_list.at(2).push_back(Vertex(5, w));
  staggered_adjacency_list.at(5).push_back(Vertex(2, w));

  staggered_adjacency_list.at(3).push_back(Vertex(4, w));
  staggered_adjacency_list.at(3).push_back(Vertex(6, w));
  staggered_adjacency_list.at(4).push_back(Vertex(3, w));
  staggered_adjacency_list.at(6).push_back(Vertex(3, w));

  staggered_adjacency_list.at(4).push_back(Vertex(5, w));
  staggered_adjacency_list.at(4).push_back(Vertex(7, w));
  staggered_adjacency_list.at(5).push_back(Vertex(4, w));
  staggered_adjacency_list.at(7).push_back(Vertex(4, w));

  staggered_adjacency_list.at(5).push_back(Vertex(8, w));
  staggered_adjacency_list.at(8).push_back(Vertex(5, w));

  staggered_adjacency_list.at(6).push_back(Vertex(7, w));
  staggered_adjacency_list.at(6).push_back(Vertex(9, w));
  staggered_adjacency_list.at(7).push_back(Vertex(6, w));
  staggered_adjacency_list.at(9).push_back(Vertex(6, w));

  staggered_adjacency_list.at(7).push_back(Vertex(8, w));
  staggered_adjacency_list.at(7).push_back(Vertex(10, w));
  staggered_adjacency_list.at(8).push_back(Vertex(7, w));
  staggered_adjacency_list.at(10).push_back(Vertex(7, w));

  staggered_adjacency_list.at(8).push_back(Vertex(11, w));
  staggered_adjacency_list.at(11).push_back(Vertex(8, w));

  staggered_adjacency_list.at(9).push_back(Vertex(10, w));
  staggered_adjacency_list.at(10).push_back(Vertex(9, w));

  staggered_adjacency_list.at(10).push_back(Vertex(11, w));
  staggered_adjacency_list.at(11).push_back(Vertex(10, w));

  //
  dijkstraUsingQueue2(start, goal, num_vertices);


  /*
  // Print distances to each vertex
  for (uint i = 1; i < num_vertices; i++)
  {
    if (distances[i]!= std::numeric_limits<uint>::max())
    {
      std::cout << distances[i] << " ";
    }
    else
    {
      std::cout<<"-1 ";
    }
  }
  std::cout << std::endl;
  */

  // Print the shortest path
  std::cout << "path: " << std::endl;
  std::vector<uint> path;
  path.push_back(goal);
  uint u = goal;
  while (1)
  {
    u = predecessors[u];
    path.push_back(u);
    if (u == start)
    {
      break;
    }
  }
  std::reverse(path.begin(), path.end());
  for (uint i = 0; i < path.size(); i++)
  {
    std::cout << path.at(i) << ", ";
  }
  std::cout << std::endl;

  



/*
//ros::init(argc, argv, "test");

  std::cout << "System info: " << std::endl;

  // Check GCC
#if __GNUC__
#if __x86_64__ || __ppc64__
  std::cout << "  64-bit" << std::endl;
#else
  std::cout << "  32-bit" << std::endl;
#endif
#endif
  std::cout << "Size of int: " << sizeof(int) << " bytes, max value = " << std::numeric_limits<int>::max() << std::endl;
  std::cout << "Size of uint32_t: " << sizeof(uint32_t) << " bytes, max value = " << std::numeric_limits<uint32_t>::max() << std::endl;
  std::cout << "Size of uint16_t: " << sizeof(uint16_t) << " bytes, max value = " << std::numeric_limits<uint16_t>::max() << std::endl;
  std::cout << "Size of uint8_t: " << sizeof(uint8_t) << " bytes, max value = " << static_cast<int>(std::numeric_limits<uint8_t>::max()) << std::endl;

  //std::cout << "  "<< argv[0]  << "  size" << argc  << std::endl;
  //std::cout << "  "<< argv[1]  << std::endl;
  //std::cout << "  "<< argv[2]  << std::endl;

  std::cout << "Virtual memory (current process): " << getVirtualMemoryValue() << " kb" << std::endl;
  std::cout << "Physical memory (current process): " << getPhysMemValue() << " kb" << std::endl;

struct sysinfo memInfo;
sysinfo(&memInfo);
long long totalVirtualMem = memInfo.totalram;
//Add other values in next statement to avoid int overflow on right hand side...
totalVirtualMem += memInfo.totalswap;
totalVirtualMem *= memInfo.mem_unit;

long long totalPhysMem = memInfo.totalram;
//Multiply in next statement to avoid int overflow on right hand side...
totalPhysMem *= memInfo.mem_unit;

  std::cout << "Total virtual memory available: " << totalVirtualMem << " bytes" <<  std::endl;
  std::cout << "Total physical memory available: " << totalPhysMem << " bytes" << std::endl;
  

  const uint num_vertices = 6539508;
  //std::vector<std::vector<uint> > g;
  //g.resize(num_vertices);
  std::vector<std::pair<int,int> > g;

  g.resize(num_vertices);

  std::cout << "Virtual memory (current process): " << getVirtualMemoryValue() << " kb" << std::endl;
  std::cout << "Physical memory (current process): " << getPhysMemValue() << " kb" << std::endl;

*/

  return 0;
}

//----------------------------------------------------------------------------//

