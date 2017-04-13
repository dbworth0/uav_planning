//
// Example 6:
// Find path through a PointCloud using A* (or alternatively Dijkstra).
//
// The PointCloud is converted into 3D occupancy (voxel) grid.
// Planning is done over an implicit graph.
// There is still an explicit list of vertices and distances, but the
// edges are implicit and there's no adjacency list stored in memory.
//
// This uses less memory than the previous implementation,
// however it takes longer to find the shortest path.
// 
// The planned path is written out as a .ply file.
//
// Usage:
// export PATH=$PATH:~/path/to/your/catkin_workspace/devel/lib/planning_through_pointclouds/
// cd ~/path/to/your/catkin_workspace/planning_through_pointclouds/data/
// ex06_plan_through_pointcloud_v3 --input ./cropped_cloud_1.ply --start 67.0 2.0 -2.0 --goal 65.0 39.0 -1.0 --output ~/path_out3_26conn.ply
//
// After the occupancy grid has been created, planning takes 50ms.
//
// Or a larger PointCloud:
// ex06_plan_through_pointcloud_v3 --input ./cropped_cloud_2.ply --start 13.0 20.0 -1.0 --goal 100.0 60.0 0.0 --output ~/path_out4_26conn.ply
//
// David Butterworth
//

#include <iostream> // cout
#include <string>
#include <vector>

#include <planning_through_pointclouds/utils.h> // processCommandLine, Timer, getProcessPhysicalMemory
#include <planning_through_pointclouds/pcl_utils.h> // loadPointCloud
//#include <planning_through_pointclouds/voxel_map_3d_v1.h>
//#include <planning_through_pointclouds/voxel_map_3d_v2.h>
#include <planning_through_pointclouds/voxel_map_3d_v3.h>

#include <pcl/io/ply_io.h> // savePLYFileASCII

//----------------------------------------------------------------------------//

int main(int argc, char **argv)
{
  std::string pointcloud_file;
  std::vector<float> start;
  std::vector<float> goal;
  std::string output_path;

  const bool result = processCommandLine(argc, argv, pointcloud_file, start, goal, output_path);
  if (!result)
  {
    return EXIT_FAILURE;
  }
  std::cout << "pointcloud_file =  "<< pointcloud_file << std::endl;
  std::cout << "output_path =  "<< output_path << std::endl;

  uint ram_used = getProcessPhysicalMemory();

  PointCloud::Ptr input_pointcloud(new PointCloud());
  loadPointCloud(pointcloud_file, *input_pointcloud);
  std::cout << "Loaded PointCloud with " << input_pointcloud->points.size() << " points" << std::endl;

  // Pad voxel grid to half the diameter of DJI S1000
  const double padding_radius = 1.1 / 2.0;
  const double voxel_size = 0.10;
  //const uint connectivity = 6;
  const uint connectivity = 26;
  VoxelMap3D map(input_pointcloud, voxel_size, padding_radius, connectivity);

  std::cout << "RAM used by Occupancy Grid: " << (getProcessPhysicalMemory() - ram_used)/1000 << " MB" << std::endl;
  ram_used = getProcessPhysicalMemory();

  map.addGraphVertices();
  std::cout << "RAM used by graph vertices: " << (getProcessPhysicalMemory() - ram_used)/1000 << " MB" << std::endl;
  ram_used = getProcessPhysicalMemory();

  const Point start_point(start.at(0), start.at(1), start.at(2));
  const Point goal_point(goal.at(0), goal.at(1), goal.at(2));

  Timer timer;

  // Dijkstra
  //timer.reset();
  //const PathCoordinates path_dijkstra_coords = map.getShortestPath(start_point, goal_point);
  ////printPath(path_dijkstra_coords);
  //std::cout << "  Took " << timer.read() << " ms." << std::endl;

  // A*
  timer.reset();
  const bool use_astar = true;
  const PathCoordinates path_astar_coords = map.getShortestPath(start_point, goal_point, use_astar);
  std::cout << "  Took " << timer.read() << " ms." << std::endl;

  // Save the planned trajectory
  //const ColorPointCloud path_pointcloud = map.getPathPointCloud(path_dijkstra_coords);
  const ColorPointCloud path_pointcloud = map.getPathPointCloud(path_astar_coords);
  pcl::io::savePLYFileASCII (output_path, path_pointcloud); 

  return 0;
}
