/*
For a path, measure its height relative to a DTM (terrain map).

David Butterworth



height_of_path schenley_path.ply schenley_elevation_100ppm.ply



Speed to find closest point:

kd-tree FLANN:
Took 0.036702 ms. 

PCL BruteForce method is about same speed as just iterating over points.

Iterate over points:
Took 13.2931 ms. 


*/

#include <iostream> // cout
#include <string>
#include <vector>

#include <fstream>

#include <planning_through_pointclouds/utils.h> // printStdVector
//#include <planning_through_pointclouds/pcl_utils.h> // 
#include <planning_through_pointclouds/pcl_utils.h> // loadPointCloud

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/brute_force.h>

#include <queue> // priority_queue
#include <limits>

/*
const float euclideanDistance2D(const Point& p0, const Point& p1)
{
  const float dx = p1.x - p0.x;
  const float dy = p1.y - p0.y;
  return std::sqrt(dx*dx + dy*dy);
}
*/



// For an x,y point, get the elevation of the ground terrain.
//
// The kd-tree is built from a PointCloud of x,y values
// and elevation_pointcloud contains the true z values.
const float getElevation(const pcl::KdTreeFLANN<Point>& kdtree, PointCloud::Ptr elevation_pointcloud, const Point& point)
{
  const int K = 1;
  std::vector<int> result_points_indices(K);
  std::vector<float> result_distances(K);
  if (kdtree.nearestKSearch(point, K, result_points_indices, result_distances) > 0)
  {
    if (result_points_indices.size() != 1)
    {
      // throw runtime
    }
    const int closest_point_index = result_points_indices.at(0);

    return elevation_pointcloud->points.at(closest_point_index).z;
  }

  // throw runtime
}

//----------------------------------------------------------------------------//

int main(int argc, char **argv)
{
  std::string path_file("");
  std::string elevation_file("");
  if (argc < 3)
  {
    std::cout << "ERROR, wrong number of arguments!" << std::endl;
    std::cout << "Usage: " << argv[0] <<" </path/to/robot_path.ply> </path/to/elevation_map.ply> " << std::endl;
    return EXIT_FAILURE;
  }
  else
  {
    path_file = std::string(argv[1]);
    elevation_file = std::string(argv[2]);
  }
  std::cout << "path_file =  "<< path_file << std::endl;
  std::cout << "elevation_file =  "<< elevation_file << std::endl;

  PointCloud::Ptr path_pointcloud(new PointCloud());
  loadPointCloud(path_file, *path_pointcloud);
  std::cout << "Loaded path with " << path_pointcloud->points.size() << " points" << std::endl;

  PointCloud::Ptr elevation_pointcloud(new PointCloud());
  loadPointCloud(elevation_file, *elevation_pointcloud);
  std::cout << "Loaded elevation map with " << elevation_pointcloud->points.size() << " points" << std::endl;

  // Copy elevation cloud and make the z-values the same.
  // (This is so the z-value doesn't affect the nearest neighbour search)
  PointCloud::Ptr elevation_pointcloud_2d(new PointCloud(*elevation_pointcloud));
  for (size_t i = 0; i < elevation_pointcloud_2d->points.size(); ++i)
  {
    elevation_pointcloud_2d->points.at(i).z = 0.0; 
  }

  pcl::KdTreeFLANN<Point> kdtree;
  //pcl::search::BruteForce<Point> kdtree;
  std::cout << "Adding PointCloud to kd-tree..." << std::endl;
  //kdtree.setInputCloud(elevation_pointcloud);
  kdtree.setInputCloud(elevation_pointcloud_2d);


  for (size_t i = 0; i < path_pointcloud->points.size(); ++i)
  {
    const float ground_height = getElevation(kdtree, elevation_pointcloud, path_pointcloud->points.at(i));
    const float path_z = path_pointcloud->points.at(i).z;
    const float path_height_from_ground = path_z - ground_height;

    if ((i < 50) || (i > (path_pointcloud->points.size() - 50)))
    {
      std::cout << "Point " << i << "  height = " << path_height_from_ground << std::endl;
    }
    
  }



 // K nearest neighbor search

  /*
  const int K = 1;

  pcl::PointXYZ search_point;
  search_point.x = 0.0;
  search_point.y = 0.0;
  search_point.z = 3.0;

  std::vector<int> result_points_indices(K);
  std::vector<float> result_distances(K);

  std::cout << "K nearest neighbor search at (" << search_point.x 
            << " " << search_point.y 
            << " " << search_point.z
            << ") with K=" << K << std::endl;

  Timer timer;

  timer.reset();
  if (kdtree.nearestKSearch(search_point, K, result_points_indices, result_distances) > 0)
  {
    for (size_t i = 0; i < result_points_indices.size (); ++i)
      std::cout << "    "  <<   elevation_pointcloud->points[ result_points_indices[i] ].x 
                << " " << elevation_pointcloud->points[ result_points_indices[i] ].y 
                << " " << elevation_pointcloud->points[ result_points_indices[i] ].z 
                << " (squared distance: " << result_distances[i] << ")" << std::endl;
  }
  std::cout << "Took " << timer.read() << " ms. \n" << std::endl;
  */


  /*
  timer.reset();
  int closest_point_idx = 0;
  float closest_point_dist = std::numeric_limits<float>::max();
  for (size_t i = 0; i < elevation_pointcloud->points.size(); ++i)
  {
    const float d = euclideanDistance2D(search_point, elevation_pointcloud->points.at(i));
    if (d < closest_point_dist)
    {
      closest_point_idx = i;
      closest_point_dist = d;
    }  
  }
  std::cout << "Closest point in 2D: " << elevation_pointcloud->points.at(closest_point_idx).x << ", " << elevation_pointcloud->points.at(closest_point_idx).y << std::endl;
  std::cout << "Took " << timer.read() << " ms. \n" << std::endl;
  */


  return EXIT_SUCCESS;
}
