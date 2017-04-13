//
// Example 4:
// Find path through a PointCloud using Dijkstra (or optionally A*).
//
// The PointCloud is converted into 3D occupancy (voxel) grid.
// Planning is done over an explicit graph.
// Uses BGL (Boost Graph Libray), which uses a lot of RAM to store vertices!
//
// The planned path is written out as a .ply file.
//
// Usage:
// export PATH=$PATH:~/path/to/your/catkin_workspace/devel/lib/planning_through_pointclouds/
// cd ~/path/to/your/catkin_workspace/planning_through_pointclouds/data/
// ex04_plan_through_pointcloud_v1 --input ./cropped_cloud_1.ply --start 67.0 2.0 -2.0 --goal 65.0 39.0 -1.0 --output ~/path_out.ply
//
// David Butterworth
//

#include <iostream> // cout
#include <string>
#include <vector>

#include <planning_through_pointclouds/utils.h> // processCommandLine
#include <planning_through_pointclouds/pcl_utils.h> // loadPointCloud
#include <planning_through_pointclouds/voxel_map_3d_v1.h>

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

  PointCloud::Ptr input_pointcloud(new PointCloud());
  loadPointCloud(pointcloud_file, *input_pointcloud);
  std::cout << "Loaded PointCloud with " << input_pointcloud->points.size() << " points" << std::endl;

  std::cout << "Initializing VoxelMap3D..." << std::endl;

  // Pad voxel grid to half the diameter of DJI S1000
  const double padding_radius = 1.1 / 2.0;
  const double voxel_size = 0.10;
  const uint connectivity = 6;
  //const uint connectivity = 26; // Careful, make sure you have 13.5GB RAM available!
  VoxelMap3D map(input_pointcloud, voxel_size, padding_radius, connectivity);

  const PointCloud::Ptr downsampled_pointcloud = map.getDownsampledPointCloud();

  const Point start_point(start.at(0), start.at(1), start.at(2));
  const Point goal_point(goal.at(0), goal.at(1), goal.at(2));

  const PathCoordinates path_dijkstra_coords = map.getShortestPathDijkstra(start_point, goal_point);
  //printPath(path_dijkstra_coords);

  //const PathCoordinates path_astar_coords = map.getShortestPathAstar(start_point, goal_point);
  //printPath(path_astar);

  // Save the planned trajectory
  const ColorPointCloud path_pointcloud = map.getPathPointCloud(path_dijkstra_coords);
  pcl::io::savePLYFileASCII (output_path, path_pointcloud); 

  return 0;
}
