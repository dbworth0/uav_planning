/*
Convert an elevation or terrain map  to a PointCloud .ply

The DTM is a comma-separated list of x,y,z values
with .xyz extension, created by LAS Tools.

This works, but we don't need this script any more. We create the DTM cloud using CloudCompare.

David Butterworth
*/

#include <iostream> // cout
#include <string>
#include <vector>

#include <fstream>

#include <planning_through_pointclouds/utils.h> // printStdVector
//#include <planning_through_pointclouds/pcl_utils.h> // 

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;

//#include <pcl/io/io.h>
#include <pcl/io/ply_io.h> //savePLYFileASCII

#define OUTPUT_PATH "/home/dbworth/"

//----------------------------------------------------------------------------//

int main(int argc, char **argv)
{
  std::string dem_file("");
  if (argc < 2)
  {
    std::cout << "ERROR, wrong number of arguments!" << std::endl;
    std::cout << "Usage: " << argv[0] <<" </path/to/dem.xyz> " << std::endl;
    return EXIT_FAILURE;
  }
  else
  {
    dem_file = std::string(argv[1]);
  }
  std::cout << "dem_file =  "<< dem_file << std::endl;

  // Read list of points from file
  std::vector< std::vector<float> > points;
  std::ifstream infile(dem_file, std::ios::in);
  if (!infile.good())
  {
    std::cerr << "Error: Could not open input file '" << dem_file << std::endl;
    exit(1);
  }
  while (infile)
  {
    // Copy each line of comma-separate values into a stream
    std::string line_string;
    if (!getline(infile, line_string))
    {
      break;
    }
    std::istringstream ss(line_string);
    //std::cout << "Line:" << std::endl;
    //std::cout << "  string: " << line_string << std::endl;

    // Separate into float values
    std::vector<float> values;
    while (ss)
    {
      std::string value_string;
      if (!getline(ss, value_string, ','))
      {
        break;
      }
      std::stringstream value_ss(value_string);
      float value;
      value_ss >> value;
      values.push_back(value);
    }
    //printStdVector(values);
    //std::cout << "  " << std::endl;

    points.push_back(values);
  }
  if (!infile.eof())
  {
    std::cerr << "error eof\n";
  }

  PointCloud cloud;
  for (size_t i = 0; i < points.size(); ++i)
  {
    Point point;
    point.x = points.at(i).at(0);
    point.y = points.at(i).at(1);
    point.z = points.at(i).at(2);
    cloud.push_back(point);
  }
  const std::string outfile = std::string(OUTPUT_PATH) + std::string("elevation.ply");
  pcl::io::savePLYFileASCII(outfile, cloud);


  return EXIT_SUCCESS;
}
