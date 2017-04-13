//
// Compare the memory usage of various volumetric structures:

//  - the input PointCloud
//  - voxel occupancy grid (STL vector)
//  - k-d tree (PCL)
//  - octree (PCL)
//  - OctoMap (brute force)
//  - FCL
//  - FCL
// - FCL



/*

Usage:


test_world_representations cropped_cloud_1.pcd cropped_cloud_1_leafsize_10cm.bt occupancy_grid

test_world_representations cropped_cloud_2.pcd cropped_cloud_2_leafsize_10cm.bt occupancy_grid

test_world_representations complete_cloud_1.pcd complete_cloud_1_leafsize_10cm.bt kdtree


To make the .bt file, us the converter tool from the pcl_octomap package.

pointcloud2octree cropped_cloud_1.pcd --resolution 0.1 -o cropped_cloud_1_leafsize_10cm.bt
pointcloud2octree complete_cloud_1.pcd --resolution 0.1 -o complete_cloud_1_leafsize_10cm.bt




*/


//
// David Butterworth
//

#include <iostream> // cout
#include <string>
#include <vector>

#include <planning_through_pointclouds/utils.h> // Timer, getProcessPhysicalMemory
#include <planning_through_pointclouds/pcl_utils.h> // loadPointCloud, getPointCloudBounds
//#include <planning_through_pointclouds/ros_utils.h> // makePointCloudMsg, addPointToPath
#include <planning_through_pointclouds/voxel_map_3d_v4_ompl_occgrid.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;

#include <pcl/common/distances.h> // euclideanDistance()
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree.h>

#include <octomap/octomap.h>

// Includes for FCL 0.3.3 in ROS Indigo
//  // This works with FCL 0.3.3 from ROS Indigo. Interface has been changed since then.

/*
#include <fcl/collision.h>
#include <fcl/shape/geometric_shapes.h> // Sphere
#include <fcl/octree.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/shape/geometric_shape_to_BVH_model.h>
#include <fcl/broadphase/broadphase.h>
*/


//----------------------------------------------------------------------------//




int main(int argc, char **argv)
{

  std::vector<Point> points_to_check;
  points_to_check.push_back(Point(68.25, 29.04, 0.25)); // cloud 1, point on edge of roof
  points_to_check.push_back(Point(68.25, 28.5, 0.25)); // cloud 1, near edge
  points_to_check.push_back(Point(68.25, 27.0, 0.25)); // cloud 1, towards start, free-space

  // Voxel resolution for the Occupancy Grid and Octree
  const double voxel_size = 0.10;

  // Check a sphere of this radius
  const double radius = 0.75;

  const int num_checks_to_avg = 1;
  

// TODO: change to input_file
  // and check if it's .pcd or .bt for appropriate tests.

  std::string pointcloud_file("");
  std::string octomap_file("");
  std::string test_type("");
  if (argc < 4)
  {
    std::cout << "ERROR, wrong number of arguments!" << std::endl;
    std::cout << "Usage: test_world_representations </path/to/pointcloud.pcd> </path/to/octomap.bt> <test_type> " << std::endl;
    return EXIT_FAILURE;
  }
  else
  {
    pointcloud_file = std::string(argv[1]);
    octomap_file = std::string(argv[2]);
    test_type = std::string(argv[3]);
  }
  std::cout << "pointcloud_file = " << pointcloud_file << std::endl;
  std::cout << "octomap_file = " << octomap_file << std::endl;
  std::cout << "test_type =  "<< test_type << std::endl;







  bool test_occupancy_grid = false;
  bool test_kdtree = false;
  bool test_octree = false;
  bool test_octomap = false;

  if (test_type == std::string("occupancy_grid"))
  {
    test_occupancy_grid = true;
  }
  else if (test_type == std::string("kdtree"))
  {
    test_kdtree = true;
  }
  else if (test_type == std::string("octree"))
  {
    test_octree = true;
  }
  else if (test_type == std::string("octomap"))
  {
    test_octomap = true;
  }
  else
  {
    std::cout << "ERROR: unknown test_type" << std::endl;
    return EXIT_FAILURE;
  }




  PointCloud::Ptr input_pointcloud(new PointCloud());
  {
    uint ram_used = getProcessPhysicalMemory();

    loadPointCloud(pointcloud_file, *input_pointcloud);
    std::cout << "\nLoaded PointCloud with " << input_pointcloud->points.size() << " points" << std::endl;
    const std::vector<float> bounds = getPointCloudBounds<Point>(input_pointcloud);
    const double x_range = bounds.at(1) - bounds.at(0);
    const double y_range = bounds.at(3) - bounds.at(2);
    const double z_range = bounds.at(5) - bounds.at(4);
    std::cout << "  dimensions: " << x_range << ", " << y_range << ", " << z_range << " meters" << std::endl;
    std::cout << "  RAM used: " << (getProcessPhysicalMemory() - ram_used)/1000 << " MB" << std::endl;
  }




  bool result;
  Timer timer;







  if (test_occupancy_grid)
  {
    std::cout << "\nCreating Occupancy Grid from PointCloud..." << std::endl;
    const double padding_radius = 0.0;
    uint ram_used = getProcessPhysicalMemory();
    VoxelMap3D occupancy_grid(input_pointcloud, voxel_size, padding_radius);
    std::cout << "  RAM used: " << (getProcessPhysicalMemory() - ram_used)/1000 << " MB" << std::endl;

  }


  else if (test_kdtree)
  {
    std::cout << "\nCreating k-d tree from PointCloud..." << std::endl;
    uint ram_used = getProcessPhysicalMemory();
    pcl::KdTreeFLANN<Point> kdtree;
    kdtree.setInputCloud(input_pointcloud);
    std::cout << "RAM used: " << (getProcessPhysicalMemory() - ram_used)/1000 << " MB" << std::endl;


  }


  else if (test_octree)
  {
    std::cout << "\nCreating Octree from PointCloud..." << std::endl;
    uint ram_used = getProcessPhysicalMemory();

    //pcl::search::Search<PointXYZ>* pcl_octree = new pcl::search::Octree<PointXYZ> (voxelResolution);
    //pcl::search::Search<Point> pcl_octree(voxelResolution);

    pcl::octree::OctreePointCloudSearch<Point> pcl_octree(voxel_size);
    pcl_octree.setInputCloud(input_pointcloud); // set ptr to input data
    pcl_octree.addPointsFromInputCloud();

    std::cout << "RAM used: " << (getProcessPhysicalMemory() - ram_used)/1000 << " MB" << std::endl;
  }



  else if (test_octomap) // Do this first, otherwise getProcessPhysicalMemory() seems to return wrong value
  {
    std::cout << "\nLoading OctoMap from BonsaiTree file..." << std::endl;
    uint ram_used = getProcessPhysicalMemory();

    //const std::string octomap_file(".bt");
    boost::shared_ptr<octomap::OcTree> octomap_octree = boost::shared_ptr<octomap::OcTree>(new octomap::OcTree(octomap_file));

    if (!octomap_octree)
    {
      std::cout << "null ptr" << std::endl;
    }
    std::cout << "RAM used: " << (getProcessPhysicalMemory() - ram_used)/1000 << " MB" << std::endl;


  }




  return 0;
}
