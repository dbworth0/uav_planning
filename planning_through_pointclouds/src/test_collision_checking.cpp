//
// Compare the speed of collision-checking a sphere volume against
// a PointCloud stored in various volumetric structures:
//  - the input PointCloud
//  - voxel occupancy grid (STL vector)
//  - k-d tree (PCL)
//  - octree (PCL)
//  - OctoMap (brute force)
//  - FCL
//  - FCL
// - FCL

// TODO: Make a 2nd version, where it samples a random point (x,y,z) in the cloud, 
// then keeps two lists of collision check times (in collision, or not).
// That would average out the data access time.

// ToDo: check time of accessing a single point or voxel
// Also count the number of points/voxels checked by each method, so know how many checks they're doing.



/*

Usage:


test_collision_checking

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


#include <fcl/collision.h>
//#include <fcl/collision_data.h>
#include <fcl/shape/geometric_shapes.h> // Sphere
#include <fcl/octree.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/shape/geometric_shape_to_BVH_model.h>

#include <fcl/broadphase/broadphase.h>
//#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
/*

#include <fcl/narrowphase/narrowphase.h>
#include <iostream>



#include <fcl/geometry/octree/octree.h>

#include "fcl/config.h"

#include "fcl/narrowphase/collision.h"
#include "fcl/broadphase/broadphase_bruteforce.h"
#include "fcl/broadphase/broadphase_spatialhash.h"
#include "fcl/broadphase/broadphase_SaP.h"
#include "fcl/broadphase/broadphase_SSaP.h"
#include "fcl/broadphase/broadphase_interval_tree.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree_array.h"
#include "fcl/geometry/geometric_shape_to_BVH_model.h"
*/

#include <random>
//#include <iostream>
//#include <ctime>


struct CollisionData
{
  CollisionData()
  {
    // Stop after 1 point found in collision
    request = fcl::CollisionRequest(1, false, 0, false);
    done = false;
  }
  fcl::CollisionRequest request;
  fcl::CollisionResult result;
  bool done;
};

bool defaultCollisionFunction(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata_)
{
  CollisionData* cdata = static_cast<CollisionData*>(cdata_);
  const fcl::CollisionRequest& request = cdata->request;
  fcl::CollisionResult& result = cdata->result;

  if (cdata->done)
  {
    return true;
  }

  fcl::collide(o1, o2, request, result);

  if (!request.enable_cost && (result.isCollision()) && (result.numContacts() >= request.num_max_contacts))
  {
    cdata->done = true;
  }

  return cdata->done;
}


//----------------------------------------------------------------------------//

const bool checkCollisionInPointCloud(const Point& point, const pcl::PointCloud<Point>::Ptr& pointcloud, const double radius)
{
  for (PointCloud::iterator it = pointcloud->begin(); it != pointcloud->end(); ++it)
  {
    if (pcl::euclideanDistance(*it, point) < radius)
    {
      return true;
    }
  }

  return false;
}

const bool checkCollisionInKdTree(const Point& point, pcl::KdTreeFLANN<Point>& kdtree, const double radius)
{
  std::vector<int> nn_indices;
  std::vector<float> nn_dists;

  const uint max_nn = 1; // 1 or more neighbours = collision

  if (kdtree.radiusSearch(point, radius, nn_indices, nn_dists, max_nn))
  {
    return true;
  }

  return false;
}

const bool checkCollisionInOctree(const Point& point, pcl::octree::OctreePointCloudSearch<Point>& octree, const double radius)
{
  std::vector<int> nn_indices;
  std::vector<float> nn_dists;

  const uint max_nn = 1; // 1 or more neighbours = collision

  if (octree.radiusSearch(point, radius, nn_indices, nn_dists, max_nn))
  {
    return true;
  }

  return false;
}


// Collision check a sphere in an OctoMap, using brute-force
// to iterate over all the leaves in the Octree
const bool checkCollisionInOctoMapOcTree(const Point& point, boost::shared_ptr<octomap::OcTree> octree, const double radius)
{
  for (octomap::OcTree::leaf_iterator it = octree->begin(), end = octree->end(); it != end; ++it)
  {
    if (octree->isNodeOccupied(*it))
    {
      const Point leaf_point(static_cast<float>(it.getX()),
                             static_cast<float>(it.getY()),
                             static_cast<float>(it.getZ()));
      if (pcl::euclideanDistance(leaf_point, point) < radius)
      {
        return true;
      }
    }
  }

  return false;
}




//----------------------------------------------------------------------------//




int main(int argc, char **argv)
{
  const std::string pointcloud_file("/home/dbworth/catkin_workspaces/uav_ws/src/planning_through_pointclouds/data/cropped_cloud_1.ply");
  const std::string octomap_file("/home/dbworth/catkin_workspaces/uav_ws/src/planning_through_pointclouds/data/cropped_cloud_1_leafsize_10cm.bt");


  //const std::string pointcloud_file("/home/dbworth/catkin_workspaces/uav_ws/src/planning_through_pointclouds/data/cropped_cloud_2.ply");
  //const std::string octomap_file("/home/dbworth/catkin_workspaces/uav_ws/src/planning_through_pointclouds/data/cropped_cloud_2_leafsize_10cm.bt");


  //const std::string pointcloud_file("/home/dbworth/catkin_workspaces/uav_ws/src/planning_through_pointclouds/data/complete_cloud_1.pcd");
  //const std::string octomap_file("/home/dbworth/catkin_workspaces/uav_ws/src/planning_through_pointclouds/data/complete_cloud_1_leafsize_10cm.bt");

  // double_cropped_cloud_3_BINARY.ply


  const int num_samples = 10000;
  


  const bool collision_check_pointcloud = true;

  //const bool collision_check_occupancy_grid = true;
  const bool collision_check_occupancy_grid = false;

  const bool collision_check_kdtree = true;
  //const bool collision_check_kdtree = false;

  const bool collision_check_octree = true;
  //const bool collision_check_octree = false;

  const bool collision_check_octomap = true;
  //const bool collision_check_octomap = false;




  
  //uint ram_used0 = getProcessPhysicalMemory();
  PointCloud::Ptr input_pointcloud(new PointCloud());
  loadPointCloud(pointcloud_file, *input_pointcloud);
  std::cout << "\nLoaded PointCloud with " << input_pointcloud->points.size() << " points" << std::endl;
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
  
  //std::cout << "  RAM used: " << (getProcessPhysicalMemory() - ram_used0)/1000 << " MB" << std::endl;
  

  /*
  std::vector<Point> points_to_check;
  points_to_check.push_back(Point(68.25, 29.04, 0.25)); // cloud 1, point on edge of roof
  points_to_check.push_back(Point(68.25, 28.5, 0.25)); // cloud 1, near edge
  points_to_check.push_back(Point(68.25, 27.0, 0.25)); // cloud 1, towards start, free-space
  */

  // ToDo: check time of accessing a single point or voxel
  // Also count the number of points/voxels checked by each method, so know how many checks they're doing.

  // Voxel resolution for the Occupancy Grid and Octree
  const double voxel_size = 0.10;

  // Check a sphere of this radius
  const double radius = 0.75;

  //bool result;
  bool in_collision; // MOVE THIS
  Timer timer;




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


  if (collision_check_pointcloud)
  {
    std::cout << "\nCollision-checking against PointCloud:" << std::endl;

    double total_time_in_collision = 0;
    double total_time_collision_free = 0;
    int num_in_collision = 0;
    int num_collision_free = 0;
    for (size_t i = 0; i < points_to_check.size(); ++i)
    {
      timer.reset();

      in_collision = checkCollisionInPointCloud(points_to_check.at(i), input_pointcloud, radius);
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
  }

  if (collision_check_occupancy_grid)
  {
    //std::cout << "\nCreating Occupancy Grid from PointCloud..." << std::endl;

    const double padding_radius = 0.0;
    VoxelMap3D occupancy_grid(input_pointcloud, voxel_size, padding_radius);

    std::cout << "\nCollision-checking against Occupancy Grid:" << std::endl;

    double total_time_in_collision = 0;
    double total_time_collision_free = 0;
    int num_in_collision = 0;
    int num_collision_free = 0;
    for (size_t i = 0; i < points_to_check.size(); ++i)
    {
      timer.reset();

      in_collision = occupancy_grid.isInCollisionSphere(points_to_check.at(i), radius);
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
    for (size_t i = 0; i < points_to_check.size(); ++i)
    {
      double total_time = 0;
      for (int j = 0; j < num_checks_to_avg; ++j)
      {
        timer.reset();
        result = occupancy_grid.isInCollisionSphere(points_to_check.at(i), radius);
        total_time += timer.read();
      }
      total_time /= num_checks_to_avg;
      std::cout << "  point " << i << ", collision check result: " << result << " in " << total_time << " ms. (averaged over " << num_checks_to_avg << " checks)" << std::endl;

    }
    */
  }

  if (collision_check_kdtree)
  {
    //std::cout << "\nCreating k-d tree from PointCloud..." << std::endl;

    pcl::KdTreeFLANN<Point> kdtree;
    kdtree.setInputCloud(input_pointcloud);

    std::cout << "\nCollision-checking against k-d tree:" << std::endl;

    double total_time_in_collision = 0;
    double total_time_collision_free = 0;
    int num_in_collision = 0;
    int num_collision_free = 0;
    for (size_t i = 0; i < points_to_check.size(); ++i)
    {
      timer.reset();

      in_collision = checkCollisionInKdTree(points_to_check.at(i), kdtree, radius);
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
    for (size_t i = 0; i < points_to_check.size(); ++i)
    {
      double total_time = 0;
      for (int j = 0; j < num_checks_to_avg; ++j)
      {
        timer.reset();
        result = checkCollisionInKdTree(points_to_check.at(i), kdtree, radius);
        total_time += timer.read();
      }
      total_time /= num_checks_to_avg;
      std::cout << "  point " << i << ", collision check result: " << result << " in " << total_time << " ms. (averaged over " << num_checks_to_avg << " checks)" << std::endl;

      //timer.reset();
      //result = checkCollisionInKdTree(points_to_check.at(i), kdtree, radius);
      //std::cout << "  point " << i << ", collision check result: " << result << " in " << timer.read() << " ms." << std::endl;
    }
    */
  }





  if (collision_check_octree)
  {
    //std::cout << "\nCreating Octree from PointCloud..." << std::endl;

    //pcl::search::Search<PointXYZ>* pcl_octree = new pcl::search::Octree<PointXYZ> (voxelResolution);
    //pcl::search::Search<Point> pcl_octree(voxelResolution);

    pcl::octree::OctreePointCloudSearch<Point> pcl_octree(voxel_size);
    pcl_octree.setInputCloud(input_pointcloud); // set ptr to input data
    pcl_octree.addPointsFromInputCloud();


    std::cout << "\nCollision-checking against octree:" << std::endl;


    double total_time_in_collision = 0;
    double total_time_collision_free = 0;
    int num_in_collision = 0;
    int num_collision_free = 0;
    for (size_t i = 0; i < points_to_check.size(); ++i)
    {
      timer.reset();

      in_collision = checkCollisionInOctree(points_to_check.at(i), pcl_octree, radius);
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
    for (size_t i = 0; i < points_to_check.size(); ++i)
    {
      double total_time = 0;
      for (int j = 0; j < num_checks_to_avg; ++j)
      {
        timer.reset();
        result = checkCollisionInOctree(points_to_check.at(i), pcl_octree, radius);
        total_time += timer.read();
      }
      total_time /= num_checks_to_avg;
      std::cout << "  point " << i << ", collision check result: " << result << " in " << total_time << " ms. (averaged over " << num_checks_to_avg << " checks)" << std::endl;

      //timer.reset();
      //result = checkCollisionInOctree(points_to_check.at(i), pcl_octree, radius);
      //std::cout << "  point " << i << ", collision check result: " << result << " in " << timer.read() << " ms." << std::endl;
    }
    */
  }




  if (collision_check_octomap)
  {
    //std::cout << "\nLoading OctoMap from BonsaiTree file..." << std::endl;

    //const std::string octomap_file(".bt");
    boost::shared_ptr<octomap::OcTree> octomap_octree = boost::shared_ptr<octomap::OcTree>(new octomap::OcTree(octomap_file));


    std::cout << "\nCollision-checking against OctoMap (brute force):" << std::endl;
    {

      double total_time_in_collision = 0;
      double total_time_collision_free = 0;
      int num_in_collision = 0;
      int num_collision_free = 0;
      for (size_t i = 0; i < points_to_check.size(); ++i)
      {
        timer.reset();

        in_collision = checkCollisionInOctoMapOcTree(points_to_check.at(i), octomap_octree, radius);
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
    }



    /*
    for (size_t i = 0; i < points_to_check.size(); ++i)
    {
      double total_time = 0;
      for (int j = 0; j < num_checks_to_avg; ++j)
      {
        timer.reset();
        result = checkCollisionInOctoMapOcTree(points_to_check.at(i), octomap_octree, radius);
        total_time += timer.read();
      }
      total_time /= num_checks_to_avg;
      std::cout << "  point " << i << ", collision check result: " << result << " in " << total_time << " ms. (averaged over " << num_checks_to_avg << " checks)" << std::endl;

      //timer.reset();
      //result = checkCollisionInOctoMapOcTree(points_to_check.at(i), octomap_octree, radius);
      //std::cout << "  point " << i << ", collision check result: " << result << " in " << timer.read() << " ms." << std::endl;
    }
    */



    std::cout << "\nCollision-checking against OctoMap (FCL Sphere and OcTree):" << std::endl;
    {
      boost::shared_ptr<fcl::OcTree> fcl_octree = boost::shared_ptr<fcl::OcTree>(new fcl::OcTree(boost::shared_ptr<const octomap::OcTree>(octomap_octree)));
      boost::shared_ptr<fcl::CollisionGeometry> fcl_octree_ptr(fcl_octree);

      boost::shared_ptr<fcl::Sphere> fcl_sphere = boost::shared_ptr<fcl::Sphere>(new fcl::Sphere(radius));




      fcl::CollisionObject sphere_co(fcl_sphere);
      fcl::CollisionObject octree_co(fcl_octree_ptr);


      double total_time_in_collision = 0;
      double total_time_collision_free = 0;
      int num_in_collision = 0;
      int num_collision_free = 0;
      for (size_t i = 0; i < points_to_check.size(); ++i)
      {

        timer.reset();

        CollisionData collision_data;
        sphere_co.setTranslation(fcl::Vec3f(points_to_check.at(i).x, points_to_check.at(i).y, points_to_check.at(i).z));

        fcl::collide(&sphere_co, &octree_co, collision_data.request, collision_data.result);
        in_collision = collision_data.result.isCollision();

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
      for (size_t i = 0; i < points_to_check.size(); ++i)
      {
        CollisionData collision_data;
        sphere_co.setTranslation(fcl::Vec3f(points_to_check.at(i).x, points_to_check.at(i).y, points_to_check.at(i).z));

        double total_time = 0;
        for (int j = 0; j < num_checks_to_avg; ++j)
        {
          timer.reset();
          fcl::collide(&sphere_co, &octree_co, collision_data.request, collision_data.result);
          result = collision_data.result.isCollision();
          total_time += timer.read();
        }
        total_time /= num_checks_to_avg;
        std::cout << "  point " << i << ", collision check result: " << result << " in " << total_time << " ms. (averaged over " << num_checks_to_avg << " checks)" << std::endl;

        //timer.reset();
        //fcl::collide(&sphere_co, &octree_co, collision_data.request, collision_data.result);
        //std::cout << "  point " << i << ", collision check result: " << collision_data.result.isCollision() << " in " << timer.read() << " ms." << std::endl;
      }
      */
    }

    std::cout << "\nCollision-checking against OctoMap (FCL Sphere BVHModel in DynamicAABBTree and OcTree):" << std::endl;
    {
      boost::shared_ptr<fcl::OcTree> fcl_octree = boost::shared_ptr<fcl::OcTree>(new fcl::OcTree(boost::shared_ptr<const octomap::OcTree>(octomap_octree)));
      boost::shared_ptr<fcl::CollisionGeometry> fcl_octree_ptr(fcl_octree);

      boost::shared_ptr<fcl::Sphere> fcl_sphere = boost::shared_ptr<fcl::Sphere>(new fcl::Sphere(radius));




      fcl::BVHModel<fcl::OBBRSS>* sphere_model = new fcl::BVHModel<fcl::OBBRSS>();
      fcl::generateBVHModel(*sphere_model, *fcl_sphere, fcl::Transform3f(), 16, 16); // 16 segments, 16 rings

      fcl::Transform3f sphere_transform;
      sphere_transform.setIdentity();
      fcl::CollisionObject sphere_co(boost::shared_ptr<fcl::CollisionGeometry>(sphere_model), sphere_transform);

      fcl::DynamicAABBTreeCollisionManager* sphere_manager = new fcl::DynamicAABBTreeCollisionManager();
      sphere_manager->registerObject(&sphere_co);
      sphere_manager->setup();

      fcl::CollisionObject octree_co(fcl_octree_ptr);

      double total_time_in_collision = 0;
      double total_time_collision_free = 0;
      int num_in_collision = 0;
      int num_collision_free = 0;
      for (size_t i = 0; i < points_to_check.size(); ++i)
      {
        timer.reset();

        CollisionData collision_data;
        sphere_co.setTranslation(fcl::Vec3f(points_to_check.at(i).x, points_to_check.at(i).y, points_to_check.at(i).z));

        sphere_manager->collide(&octree_co, &collision_data, defaultCollisionFunction);
        in_collision = collision_data.result.isCollision();

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
      for (size_t i = 0; i < points_to_check.size(); ++i)
      {
        CollisionData collision_data;
        sphere_co.setTranslation(fcl::Vec3f(points_to_check.at(i).x, points_to_check.at(i).y, points_to_check.at(i).z));

        double total_time = 0;
        for (int j = 0; j < num_checks_to_avg; ++j)
        {
          timer.reset();
          sphere_manager->collide(&octree_co, &collision_data, defaultCollisionFunction);
          result = collision_data.result.isCollision();
          total_time += timer.read();
        }
        total_time /= num_checks_to_avg;
        std::cout << "  point " << i << ", collision check result: " << result << " in " << total_time << " ms. (averaged over " << num_checks_to_avg << " checks)" << std::endl;

        //timer.reset();
        //sphere_manager->collide(&octree_co, &collision_data, defaultCollisionFunction);
        //std::cout << "  point " << i << ", collision check result: " << collision_data.result.isCollision() << " in " << timer.read() << " ms." << std::endl;
      }
      */
    }

    std::cout << "\nCollision-checking against OctoMap (FCL Sphere BVHModel in DynamicAABBTree and OcTree in DynamicAABBTree):" << std::endl;
    {
      boost::shared_ptr<fcl::OcTree> fcl_octree = boost::shared_ptr<fcl::OcTree>(new fcl::OcTree(boost::shared_ptr<const octomap::OcTree>(octomap_octree)));
      boost::shared_ptr<fcl::CollisionGeometry> fcl_octree_ptr(fcl_octree);

      boost::shared_ptr<fcl::Sphere> fcl_sphere = boost::shared_ptr<fcl::Sphere>(new fcl::Sphere(radius));




      fcl::BVHModel<fcl::OBBRSS>* sphere_model = new fcl::BVHModel<fcl::OBBRSS>();
      fcl::generateBVHModel(*sphere_model, *fcl_sphere, fcl::Transform3f(), 16, 16); // 16 segments, 16 rings

      fcl::Transform3f sphere_transform;
      sphere_transform.setIdentity();
      fcl::CollisionObject sphere_co(boost::shared_ptr<fcl::CollisionGeometry>(sphere_model), sphere_transform);

      fcl::DynamicAABBTreeCollisionManager* sphere_manager = new fcl::DynamicAABBTreeCollisionManager();
      sphere_manager->registerObject(&sphere_co);
      sphere_manager->setup();

      fcl::CollisionObject octree_co(fcl_octree_ptr);

      fcl::DynamicAABBTreeCollisionManager* octree_manager = new fcl::DynamicAABBTreeCollisionManager();
      octree_manager->registerObject(&octree_co);
      octree_manager->setup();


      double total_time_in_collision = 0;
      double total_time_collision_free = 0;
      int num_in_collision = 0;
      int num_collision_free = 0;
      for (size_t i = 0; i < points_to_check.size(); ++i)
      {
        timer.reset();

        CollisionData collision_data;
        sphere_co.setTranslation(fcl::Vec3f(points_to_check.at(i).x, points_to_check.at(i).y, points_to_check.at(i).z));

        sphere_manager->collide(octree_manager, &collision_data, defaultCollisionFunction);
        in_collision = collision_data.result.isCollision();

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
      for (size_t i = 0; i < points_to_check.size(); ++i)
      {
        CollisionData collision_data;
        sphere_co.setTranslation(fcl::Vec3f(points_to_check.at(i).x, points_to_check.at(i).y, points_to_check.at(i).z));


        double total_time = 0;
        for (int j = 0; j < num_checks_to_avg; ++j)
        {
          timer.reset();
          sphere_manager->collide(octree_manager, &collision_data, defaultCollisionFunction);
          result = collision_data.result.isCollision();
          total_time += timer.read();
        }
        total_time /= num_checks_to_avg;
        std::cout << "  point " << i << ", collision check result: " << result << " in " << total_time << " ms. (averaged over " << num_checks_to_avg << " checks)" << std::endl;

      }
      */
    }
  }




  return 0;
}
