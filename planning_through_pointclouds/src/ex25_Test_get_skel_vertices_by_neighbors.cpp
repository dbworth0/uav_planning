/*
Input is a binary skeleton, white on black



ex25 4 /home/dbworth/skeleton/MyMap1_cropped_DynVoronoi.png /home/dbworth/skeleton/MyMap1_cropped_DynVoronoi_vertices_method1.png
ex25 4 /home/dbworth/skeleton/MyMap1_cropped_DynVoronoi_thin4.png /home/dbworth/skeleton/MyMap1_cropped_DynVoronoi_thin4_vertices_method1.png

ex25 8 /home/dbworth/skeleton/MyMap1_Zhang.png /home/dbworth/skeleton/MyMap1_Zhang_vertices_method1.png
ex25 8 /home/dbworth/skeleton/MyMap1_Zhang_thin8.png /home/dbworth/skeleton/MyMap1_Zhang_thin8_vertices_method1.png


ex25 4 /home/dbworth/skeleton/MyMap2_DynVoronoi_thin4.png /home/dbworth/skeleton/MyMap2_DynVoronoi_thin4_vertices_method1.png
ex25 8 /home/dbworth/skeleton/MyMap2_Zhang_thin8.png /home/dbworth/skeleton/MyMap2_Zhang_thin8_vertices_method1.png


ex25 4 /home/dbworth/skeleton/beeson1-DynVoronoi-thin4.png /home/dbworth/skeleton/beeson1-DynVoronoi-thin4_vertices_method1.png
ex25 8 /home/dbworth/skeleton/beeson1-Zhang-thin8.png /home/dbworth/skeleton/beeson1-Zhang-thin8_vertices_method1.png



ex25 4 /home/dbworth/skeleton/FR079_DynVoronoi_thin4.png /home/dbworth/skeleton/FR079_DynVoronoi_thin4_vertices_method1.png
ex25 8 /home/dbworth/skeleton/FR079_Zhang_thin8.png /home/dbworth/skeleton/FR079_Zhang_thin8_vertices_method1.png



ex25 4 /home/dbworth/skeleton/FR101_DynVoronoi_thin4.png /home/dbworth/skeleton/FR101_DynVoronoi_thin4_vertices_method1.png
ex25 8 /home/dbworth/skeleton/FR101_Zhang_thin8.png /home/dbworth/skeleton/FR101_Zhang_thin8_vertices_method1.png





*/

#include <iostream> // cout
//#include <string>
//#include <vector>
#include <cstdlib> // atoi

//#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h> // PointCloud2ConstPtr

//#include <Eigen/Geometry>

//#include <boost/algorithm/string.hpp> // to_upper

#include <planning_through_pointclouds/utils.h> // splitString
//#include <planning_through_pointclouds/ompl_utils.h> // getPlannerGraphMarker
//#include <planning_through_pointclouds/pcl_utils.h> // loadPointCloud, getPointCloudBounds
//#include <planning_through_pointclouds/ros_utils.h> // makePointCloudMsg
#include <planning_through_pointclouds/opencv_utils.h> // printCvMatType

#include <opencv2/core/core.hpp> // normalize
#include <opencv2/highgui/highgui.hpp> // imwrite
#include <opencv2/imgproc/imgproc.hpp> // cvtColor


/*
const bool is4Connected(const cv::Point& p0, const cv::Point& p1)
{
  const unsigned int x_diff = std::abs(p0.x - p1.x);
  const unsigned int y_diff = std::abs(p0.y - p1.y);
  if (((x_diff == 1) && (y_diff == 0)) || ((x_diff == 0) && (y_diff == 1)))
  {
    return true;
  }
  return false;
}
*/

// Find vertices of skeleton.
//
// Vertices are drawn as red pixels on top of the skeleton.
// This works on 8UC1 image, pixel values are 0 or 255.
// The outer border of pixels are not checked.
void findVerticesByCountingNeighbors(const cv::Mat& in_image, const uint connectivity, cv::Mat& out_image)
{
  if ((connectivity != 4) && (connectivity != 8))
  {
    //throw
  }

  out_image = cv::Mat(in_image.size(), CV_8UC3);
  cv::cvtColor(in_image, out_image, CV_GRAY2RGB);

  struct RGB
  {
    uchar blue;
    uchar green;
    uchar red;
  };

  unsigned int num_4_conn_px = 0;
  unsigned int num_8_conn_px = 0;
  for (int i = 1; i < in_image.rows-1; ++i)
  {
    const uchar* row_ptr = in_image.ptr<uchar>(i);
    for (int j = 1; j < in_image.cols-1; ++j)
    {
      //std::cout << "Pixel value: " << static_cast<int>(row_ptr[j]) << std::endl;

      if (row_ptr[j] == 0)
      {
        // 0 = not the skeleton
        continue;
      }

      num_4_conn_px = 0;
      num_8_conn_px = 0;


      if (connectivity == 4)
      {
        if (in_image.at<uchar>(i+0,j+1) != 0) num_4_conn_px++;
        if (in_image.at<uchar>(i+0,j-1) != 0) num_4_conn_px++;
        if (in_image.at<uchar>(i+1,j+0) != 0) num_4_conn_px++;
        if (in_image.at<uchar>(i-1,j+0) != 0) num_4_conn_px++;

        if (num_4_conn_px > 2)
        {
          RGB& pixel = out_image.ptr<RGB>(i)[j]; // scanline y, pixel x
          pixel.red = 255;
          pixel.green = 0;
          pixel.blue = 0;
        }
      }

      else if (connectivity == 8)
      {
        // Check 4-connected neighbours first
        if (in_image.at<uchar>(i+0,j+1) != 0) num_4_conn_px++;
        if (in_image.at<uchar>(i+0,j-1) != 0) num_4_conn_px++;
        if (in_image.at<uchar>(i+1,j+0) != 0) num_4_conn_px++;
        if (in_image.at<uchar>(i-1,j+0) != 0) num_4_conn_px++;

        if (in_image.at<uchar>(i+1,j+1) != 0) num_8_conn_px++;
        if (in_image.at<uchar>(i+1,j-1) != 0) num_8_conn_px++;
        if (in_image.at<uchar>(i-1,j+1) != 0) num_8_conn_px++;
        if (in_image.at<uchar>(i-1,j-1) != 0) num_8_conn_px++;

        if ((num_4_conn_px + num_8_conn_px) > 2)
        //if ((num_4_conn_px > 2) || ((num_4_conn_px == 1) && (num_8_conn_px >= 2)))
        {
          RGB& pixel = out_image.ptr<RGB>(i)[j]; // scanline y, pixel x
          pixel.red = 255;
          pixel.green = 0;
          pixel.blue = 0;
        }
      }

    }
  }
}

// Invert black and white colors, but leave other colors the same
const cv::Mat invertBlackAndWhite(const cv::Mat& in_image)
{
  cv::Mat inverted_image = cv::Mat(in_image.size(), CV_8UC3);

  struct RGB
  {
    uchar blue;
    uchar green;
    uchar red;
  };

  for (int i = 0; i < in_image.rows; ++i)
  {
    const uchar* row_ptr = in_image.ptr<uchar>(i);
    for (int j = 0; j < in_image.cols; ++j)
    {
      const RGB& in_pixel = in_image.ptr<RGB>(i)[j]; // scanline y, pixel x
      RGB& out_pixel = inverted_image.ptr<RGB>(i)[j];

      if ((in_pixel.red == 0) && (in_pixel.green == 0) && (in_pixel.blue == 0))
      {
        out_pixel.red = 255;
        out_pixel.green = 255;
        out_pixel.blue = 255;
      }
      else if ((in_pixel.red == 255) && (in_pixel.green == 255) && (in_pixel.blue == 255))
      {
        out_pixel.red = 0;
        out_pixel.green = 0;
        out_pixel.blue = 0;
      }
      else
      {
        out_pixel = in_pixel;
      }
    }
  }

  return inverted_image;
}

// THIS IS DUPLICATED IN MULTIPLE .cpp
//
// Print info about number of vertices detected
// - by counting red-colored pixels
// - by counting number of clusters
void printNumVertices(const cv::Mat& in_image)
{
  cv::Mat clusters_image = cv::Mat::zeros(in_image.size(), CV_8UC1);

  struct RGB
  {
    uchar blue;
    uchar green;
    uchar red;
  };

  uint num_red_pixels = 0;

  for (int i = 1; i < in_image.rows-1; ++i)
  {
    for (int j = 1; j < in_image.cols-1; ++j)
    {
      const RGB& in_pixel = in_image.ptr<RGB>(i)[j]; // scanline y, pixel x

      if ((in_pixel.red == 255) && (in_pixel.green == 0) && (in_pixel.blue == 0))
      {
        num_red_pixels++;
        clusters_image.at<uchar>(i,j) = 255;
      }
    }
  }

  // Find the connected-components:
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(clusters_image,
                   contours, hierarchy,
                   CV_RETR_LIST, // A flat list of contours, no hierachy
                   CV_CHAIN_APPROX_NONE);  // Use exact pixels

  std::cout << "Detected: " << contours.size() << " vertices (pixel clusters)" << std::endl;
  std::cout << "          " << num_red_pixels << " individual vertex pixels" << std::endl;
}

//----------------------------------------------------------------------------//

int main(int argc, char **argv)
{
  
  std::string input_filename("");
  std::string output_filename("");
  int connectivity = 0;

  //if (argc < 3)
  if (argc < 4)
  {
    std::cout << "ERROR, wrong number of arguments!" << std::endl;
    std::cout << "Usage: " << argv[0] <<" <4 or 8> </path/to/image.png> </output/file.png> " << std::endl;
    return EXIT_FAILURE;
  }
  else
  {
    input_filename = std::string(argv[2]);
    output_filename = std::string(argv[3]);
    connectivity = atoi(argv[1]);
    std::cout << "conn: " << connectivity << std::endl;
  }
  //std::cout << "pointcloud_file =  "<< pointcloud_file << std::endl;
  

  const std::vector<std::string> output_filename_tokens = splitString(output_filename);
  std::string outfile;


  // 2.4.8
  std::cout << "OpenCV version : " << CV_VERSION << std::endl;
  std::cout << "Major version : " << CV_MAJOR_VERSION << std::endl;
  std::cout << "Minor version : " << CV_MINOR_VERSION << std::endl;
  std::cout << "Subminor version : " << CV_SUBMINOR_VERSION << std::endl;


  // Parameters for writing .png image using libPNG
  const std::vector<int> png_params = {CV_IMWRITE_PNG_COMPRESSION, 0};

  // Load image into 8UC3 matrix
  cv::Mat image;
  image = cv::imread(input_filename);
  if (!image.data)
  {
    std::cout <<  "Could not open or find the image" << std::endl;
    return -1;
  }

  // Convert to 8UC1
  cv::Mat gray_image(image.size(), CV_8UC1);
  cv::cvtColor(image, gray_image, CV_BGR2GRAY);
  // Make sure it's a binary black-and-white image
  cv::Mat binary_image(image.size(), CV_8UC1);
  cv::threshold(gray_image, binary_image, 150, 255, CV_THRESH_BINARY);

  // Find all pixels with more than 2 neighbours:
  cv::Mat vertices_image;
  findVerticesByCountingNeighbors(binary_image, connectivity, vertices_image);
  printNumVertices(vertices_image);
  cv::imshow("vertices_image", vertices_image);
  cv::waitKey(0);
  outfile = output_filename_tokens[0] + std::string("_vertices.png");
  cv::imwrite(outfile, vertices_image, png_params);

  // Create another copy with white backward
  cv::Mat inverted_vertices_image = invertBlackAndWhite(vertices_image);
  outfile = output_filename_tokens[0] + std::string("_vertices_inv.png");
  cv::imwrite(outfile, inverted_vertices_image, png_params);



  return 0;
}
