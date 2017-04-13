//
// Compare the memory usage of OpenVDB


/*

Usage:

test_openvdb cropped_cloud_1.ply
test_openvdb cropped_cloud_2.ply


test_openvdb double_cropped_cloud_3_BINARY.ply


TODO: make voxel map  OpenVDB has Int32Grid
instead of float.


*/


//
// David Butterworth
//

#include <iostream> // cout
#include <string>
#include <vector>

#include <planning_through_pointclouds/utils.h> // Timer, getProcessPhysicalMemory
#include <planning_through_pointclouds/pcl_utils.h> // loadPointCloud

#include <planning_through_pointclouds/voxel_map_3d_v5_openvdb.h>


//#include <planning_through_pointclouds/ros_utils.h> // makePointCloudMsg, addPointToPath
//#include <planning_through_pointclouds/voxel_map_3d_v4_ompl_occgrid.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;

//#include <pcl/common/distances.h> // euclideanDistance()
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/octree/octree.h>

//#include <octomap/octomap.h>

#include <openvdb/openvdb.h>

#include <random>

//----------------------------------------------------------------------------//




int main(int argc, char **argv)
{

  const int num_samples = 10000;
  




  std::string pointcloud_file("");
  if (argc < 2)
  {
    std::cout << "ERROR, wrong number of arguments!" << std::endl;
    std::cout << "Usage: " << argv[0] <<" </path/to/pointcloud.pcd> " << std::endl;
    return EXIT_FAILURE;
  }
  else
  {
    pointcloud_file = std::string(argv[1]);
  }
  std::cout << "pointcloud_file =  "<< pointcloud_file << std::endl;

  PointCloud::Ptr input_pointcloud(new PointCloud());
  loadPointCloud(pointcloud_file, *input_pointcloud);
  std::cout << "Loaded PointCloud with " << input_pointcloud->points.size() << " points" << std::endl;
  if (input_pointcloud->points.size() == 0)
  {
    return EXIT_FAILURE;
  }

  const std::vector<float> bounds = getPointCloudBounds<Point>(input_pointcloud);
  const double x_range = bounds.at(1) - bounds.at(0);
  const double y_range = bounds.at(3) - bounds.at(2);
  const double z_range = bounds.at(5) - bounds.at(4);
  std::cout << "  range: " << bounds.at(0) << " : " << bounds.at(1) << std::endl;
  std::cout << "         " << bounds.at(2) << " : " << bounds.at(3) << std::endl;
  std::cout << "         " << bounds.at(4) << " : " << bounds.at(5) << std::endl;
  std::cout << "  dimensions: " << x_range << ", " << y_range << ", " << z_range << " meters" << std::endl;
  


  //const bool publish_ros_msg = false;
  //const bool publish_ros_msg = true;
  //std::cout << "Initializing VoxelMap3D..." << std::endl;


  const double voxel_size = 0.10; // This is used to calculate the number of voxels, but
                                  // currently the grid doesn't have any dimensions relative to the real world.

  const double padding_radius = 0.0; // Not implemented



  // Check a sphere of this radius
  const double radius = 0.75;


  std::cout << "\nCreating OpenVDB Grid from PointCloud..." << std::endl;

  uint ram_used = getProcessPhysicalMemory();

  //VoxelMap3D map(150, 400, 107, voxel_size); // 29 MB
  //VoxelMap3D map(1000, 600, 194, voxel_size); // 502 MB
  //VoxelMap3D map(3724, 3698, 492, voxel_size); // Do not used, too big for RAM

  VoxelMap3D occupancy_grid(input_pointcloud, voxel_size, padding_radius);
  
  std::cout << "  RAM used: " << (getProcessPhysicalMemory() - ram_used)/1000 << " MB" << std::endl;



  // Generate random sample points in the range [min, max)
  // using Mersenne Twister random-number generator
  // from C++ 11
  std::uniform_real_distribution<double> x_val_distr(bounds.at(0), bounds.at(1));
  std::uniform_real_distribution<double> y_val_distr(bounds.at(2), bounds.at(3));
  std::uniform_real_distribution<double> z_val_distr(bounds.at(4), bounds.at(5));

  std::mt19937 rng; 
  rng.seed(std::random_device{}()); // Initialize with non-deterministic seed

  std::vector<Point> points_to_check;
  for (int i = 0; i < num_samples; ++i)
  {
    const Point point(x_val_distr(rng),
                      y_val_distr(rng),
                      z_val_distr(rng));
    points_to_check.push_back(point);
  }
  //for (size_t i = 0; i < points_to_check.size(); ++i)
  //{
  //  std::cout << "  point " << i << " " << points_to_check.at(i).x << ", " << points_to_check.at(i).y << ", " << points_to_check.at(i).z << std::endl;
  //}



  //std::cout << "\nCreating Occupancy Grid from PointCloud..." << std::endl;

  //const double padding_radius = 0.0;
  //VoxelMap3D occupancy_grid(input_pointcloud, voxel_size, padding_radius);

  std::cout << "\nCollision-checking against Occupancy Grid:" << std::endl;

  Timer timer;



  double total_time_in_collision = 0;
  double total_time_collision_free = 0;
  int num_in_collision = 0;
  int num_collision_free = 0;
  for (size_t i = 0; i < points_to_check.size(); ++i)
  {
    timer.reset();

    const bool in_collision = occupancy_grid.isInCollisionSphere(points_to_check.at(i), radius);
    if (in_collision)
    {
      total_time_in_collision += timer.read();
      num_in_collision++;
    }
    else
    {
      total_time_collision_free += timer.read();
      num_collision_free++;
    }
  }
  total_time_in_collision /= num_in_collision;
  total_time_collision_free /= num_collision_free;
  std::cout << "  " << num_in_collision << " points in collision, average check time = " << total_time_in_collision << " ms" << std::endl;
  std::cout << "  " << num_collision_free << " points collision-free, average check time = " << total_time_collision_free << " ms" << std::endl;
  //std::cout << "    Avg. time " << (total_time_in_collision + total_time_collision_free) / 2.0 << " ms" << std::endl;
  std::cout << "     " << total_time_in_collision << " " << total_time_collision_free << " " << (total_time_in_collision + total_time_collision_free) / 2.0 << std::endl;




/*
    // Initialize the OpenVDB library.  This must be called at least
    // once per program and may safely be called multiple times.
    openvdb::initialize();
    // Create an empty floating-point grid with background value 0.
    openvdb::FloatGrid::Ptr grid = openvdb::FloatGrid::create();
    std::cout << "Testing random access:" << std::endl;
    // Get an accessor for coordinate-based access to voxels.
    openvdb::FloatGrid::Accessor accessor = grid->getAccessor();
    // Define a coordinate with large signed indices.
    openvdb::Coord xyz(1000, -200000000, 30000000);
    // Set the voxel value at (1000, -200000000, 30000000) to 1.
    accessor.setValue(xyz, 1.0);
    // Verify that the voxel value at (1000, -200000000, 30000000) is 1.
    std::cout << "Grid" << xyz << " = " << accessor.getValue(xyz) << std::endl;
    // Reset the coordinates to those of a different voxel.
    xyz.reset(1000, 200000000, -30000000);
    // Verify that the voxel value at (1000, 200000000, -30000000) is
    // the background value, 0.
    std::cout << "Grid" << xyz << " = " << accessor.getValue(xyz) << std::endl;
    // Set the voxel value at (1000, 200000000, -30000000) to 2.
    accessor.setValue(xyz, 2.0);
    // Set the voxels at the two extremes of the available coordinate space.
    // For 32-bit signed coordinates these are (-2147483648, -2147483648, -2147483648)
    // and (2147483647, 2147483647, 2147483647).
    accessor.setValue(openvdb::Coord::min(), 3.0f);
    accessor.setValue(openvdb::Coord::max(), 4.0f);
    std::cout << "Testing sequential access:" << std::endl;
    // Print all active ("on") voxels by means of an iterator.
    for (openvdb::FloatGrid::ValueOnCIter iter = grid->cbeginValueOn(); iter; ++iter) {
        std::cout << "Grid" << iter.getCoord() << " = " << *iter << std::endl;
    }
*/


  return 0;
}
