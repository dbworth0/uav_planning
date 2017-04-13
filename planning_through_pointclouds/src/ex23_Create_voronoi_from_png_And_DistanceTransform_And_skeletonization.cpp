/*
Example 23:


ex23 cropped_cloud_1.ply
*/

#include <iostream> // cout
//#include <string>
//#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> // PointCloud2ConstPtr

#include <Eigen/Geometry>

//#include <boost/algorithm/string.hpp> // to_upper

//#include <planning_through_pointclouds/utils.h> // processCommandLine, Timer
//#include <planning_through_pointclouds/ompl_utils.h> // getPlannerGraphMarker
#include <planning_through_pointclouds/pcl_utils.h> // loadPointCloud, getPointCloudBounds
#include <planning_through_pointclouds/ros_utils.h> // makePointCloudMsg
#include <planning_through_pointclouds/opencv_utils.h> // printCvMatType


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointXYZRGBA ColorPoint;
typedef pcl::PointCloud<ColorPoint> ColorPointCloud;


//#include <planning_through_pointclouds/octree_map_3d_v3_ompl_dubins.h>

//#include <pcl/io/ply_io.h> // savePLYFileASCII
#include <pcl/io/png_io.h> // savePNGFile

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp> // imwrite
#include <opencv2/imgproc/imgproc.hpp> // cvtColor


//#include <SplineLibrary/spline_library/hermite/cubic/cubic_hermite_spline.h>

////#include <voronoi_2d/voronoi.hpp> // WHY DOESNT THIS WORK ??
#include <voronoi_2d/voronoi/src/voronoi.h> // VoronoiThinner - Zhang & Guo algorithms
#include <voronoi_2d/evg-thin/evg-thin.hh>


#include <dynamic_voronoi/dynamicvoronoi.h>



#include <iostream>
#include <fstream>
#include <string.h>

void loadPGM( std::istream &is, int *sizeX, int *sizeY, bool ***map ) {
  std::string tag;
  is >> tag;
  if (tag!="P5") {
    std::cerr << "Awaiting 'P5' in pgm header, found " << tag << std::endl;
    exit(-1);
  }
  
  while (is.peek()==' ' || is.peek()=='\n') is.ignore();
  while (is.peek()=='#') is.ignore(255, '\n');
  is >> *sizeX;
  while (is.peek()=='#') is.ignore(255, '\n');
  is >> *sizeY;
  while (is.peek()=='#') is.ignore(255, '\n');
  is >> tag;
  if (tag!="255") {
    std::cerr << "Awaiting '255' in pgm header, found " << tag << std::endl;
    exit(-1);
  }
  is.ignore(255, '\n');  
  
  *map = new bool*[*sizeX];

  for (int x=0; x<*sizeX; x++) {
    (*map)[x] = new bool[*sizeY];
  }
  for (int y=*sizeY-1; y>=0; y--) {
    for (int x=0; x<*sizeX; x++) {
      int c = is.get();
      if ((double)c<255-255*0.2) (*map)[x][y] = true; // cell is occupied
      else (*map)[x][y] = false; // cell is free
      if (!is.good()) {
        std::cerr << "Error reading pgm map.\n";
        exit(-1);
      }
    }
  }
}


// Set the z value of all points in a PointCloud to a specific value
template <typename PointT>
void flattenPointCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud, const float z_height)
{
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    cloud->points.at(i).z = z_height;
  }
}


/*
// Draw voronoi diagram
// based on tutorial at LearnOpenCV
// this does not produce desired result.
static void draw_voronoi(cv::Mat& img, cv::Subdiv2D& subdiv)
{
    std::vector<std::vector<cv::Point2f> > facets;
    std::vector<cv::Point2f> centers;
    subdiv.getVoronoiFacetList(std::vector<int>(), facets, centers); // expects cv::Point2f
 
    std::vector<cv::Point> ifacet;
    std::vector<std::vector<cv::Point> > ifacets(1);
 
    for ( size_t i = 0; i < facets.size(); i++ )
    {
        ifacet.resize(facets[i].size());
        for( size_t j = 0; j < facets[i].size(); j++ )
        {
            ifacet[j] = facets[i][j];
        }
 
        cv::Scalar color;
        color[0] = std::rand() & 255;
        color[1] = std::rand() & 255;
        color[2] = std::rand() & 255;
        //cv::fillConvexPoly(img, ifacet, color, 8, 0); // Draw regions in random colors
        cv::fillConvexPoly(img, ifacet, cv::Scalar(255, 255, 255), 8, 0); // Draw regions as white
 
        ifacets[0] = ifacet;
        cv::polylines(img, ifacets, true, cv::Scalar(), 1, CV_AA, 0); // color=black, line_width=1, anti-aliased
        //cv::circle(img, centers[i], 3, cv::Scalar(), CV_FILLED, CV_AA, 0); // Make the black areas solid
    }
}
*/

//----------------------------------------------------------------------------//

int main(int argc, char **argv)
{
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


  const std::vector<float> bounds = getPointCloudBounds<Point>(input_pointcloud);
  std::cout << "Cloud dimensions: " << std::endl
            << "  x_min: " << bounds.at(0) << " "
            << "  x_max: " << bounds.at(1) << std::endl
            << "  y_min: " << bounds.at(2) << " "
            << "  y_max: " << bounds.at(3) << std::endl
            << "  z_min: " << bounds.at(4) << " "
            << "  z_max: " << bounds.at(5) << std::endl;

  // Crop a z-slice of the PointCloud
  const float z_height = -1.0;
  const float slice_height = 0.50;
  const float z_min = z_height - slice_height;
  const float z_max = z_height + slice_height;
  PointCloud::Ptr cropped_cloud(new PointCloud());
  const Eigen::Vector3f min_extents(bounds.at(0), bounds.at(2), z_min);
  const Eigen::Vector3f max_extents(bounds.at(1), bounds.at(3), z_max);

  cropPointCloud<Point>(input_pointcloud, cropped_cloud, min_extents, max_extents);

  PointCloud::Ptr cloud_slice(new PointCloud(*cropped_cloud));
  flattenPointCloud<Point>(cloud_slice, z_height);


  const std::string png_output_path("/home/dbworth/slice.png");

  // This only works for organized PointClouds
  //pcl::io::savePNGFile<Point>(png_output_path, *cloud_slice, "z"); // "z", "rgb", "normal", "label"


// Load image into 8UC3 matrix
cv::Mat image;
image = cv::imread("/home/dbworth/binary_image1.png");
if (!image.data)
{
    std::cout <<  "Could not open or find the image" << std::endl;
    //return -1;
}
printCvMatType(image);
//cv::namedWindow("Display window", CV_WINDOW_AUTOSIZE);
//cv::imshow("Display window", image);
//cv::waitKey(0); 

// Convert to 8UC1
cv::Mat binary_image(image.size(), CV_8UC1);
cv::cvtColor(image, binary_image, CV_BGR2GRAY);
printCvMatType(binary_image);

// Voronoi diagram using SubDiv2D
//
// Input image binary_image2.png
// has white area for obstacles.
// from LearnOpenCV, this does not produce desired result.
/*
// Get locations of non-zero pixels
//
// Note cv::findNonZero function in general: if binaryImage contains zero non-zero elements, it will throw because
// it tries to allocate '1 x n' memory, 
// try calling cv::countNonZero beforehand 
std::vector<cv::Point2i> locations;
cv::findNonZero(binary_image, locations); // This requires 8UC1 image
std::cout << "Non-zero pixel locations:" << std::endl;
//for (size_t i = 0; i < locations.size(); ++i)
//{
//  std::cout << "  " << locations.at(i).x << "," << locations.at(i).y << std::endl;
//}
cv::Size size = binary_image.size();
cv::Rect rect(0, 0, size.width, size.height);
cv::Subdiv2D subdiv(rect);
for (std::vector<cv::Point2i>::iterator it = locations.begin(); it != locations.end(); ++it)
{
  subdiv.insert(*it);     
}
cv::Mat img_voronoi = cv::Mat::zeros(binary_image.rows, binary_image.cols, CV_8UC3);
draw_voronoi(img_voronoi, subdiv);
cv::namedWindow("Display window", CV_WINDOW_AUTOSIZE);
cv::imshow("Display window", img_voronoi);
cv::waitKey(0); 
*/


// Perform the distance transform algorithm
/*
cv::Mat dist;
cv::distanceTransform(binary_image, dist, CV_DIST_L2, 3);
// Normalize the distance image for range = {0.0, 1.0}
// so we can visualize and threshold it
cv::normalize(dist, dist, 0.0, 1.0, cv::NORM_MINMAX); // this is in core.hpp
cv::namedWindow("Display window", CV_WINDOW_AUTOSIZE);
cv::imshow("Distance Transform Image", dist);
cv::waitKey(0); 
*/


// Skeleton (3 implementations)
// This works, but there's some skeleton seeminly inside the obstacles.
// It matches the distance transform of the image with a border around the outside.

/*
    1984, Zhang - Suen explained in "A fast parallel algorithm for thinning digital patterns" by T.Y. Zhang and C.Y. Suen and based on implentation by http://opencv-code.com/quick-tips/implementation-of-thinning-algorithm-in-opencv/
    1989, Guo - Hall explained in "Parallel thinning with two sub-iteration algorithms" by Zicheng Guo and Richard Hall and based on implentation by http://opencv-code.com/quick-tips/implementation-of-guo-hall-thinning-algorithm/
    a morphological one, based on the erode() and dilate() operators.  
    Coming from: http://felix.abecassis.me/2011/09/opencv-morphological-skeleton/
*/
// There's also a paper by Zhang and Wang on another parallel implementation.
/*
//const std::string implementation_name("morph");
const std::string implementation_name("zhang_suen_fast");
//const std::string implementation_name("guo_hall_fast");
//if (!VoronoiThinner::is_implementation_valid(implementation_name)) {
VoronoiThinner thinner;
//cv::Mat1b file = cv::imread("/home/dbworth/voronoi_horse.png", CV_LOAD_IMAGE_GRAYSCALE);
//cv::Mat1b file = cv::imread("/home/dbworth/voronoi_opencv_src.png", CV_LOAD_IMAGE_GRAYSCALE);
cv::Mat1b file = cv::imread("/home/dbworth/binary_image1.png", CV_LOAD_IMAGE_GRAYSCALE);
if (file.empty())
{
  std::cout << "Could not load file" << std::endl;
}
bool ok = thinner.thin(file, implementation_name, true);
if (!ok)
{
  std::cout << "Failed thinning" << std::endl;
}
//cv::imwrite(out.str(), thinner.get_skeleton());
//cv::imshow("query", files[file_idx]);
const cv::Mat1b skel_image = thinner.get_skeleton(); // 8UC1
//printCvMatType(skel_image);
cv::imshow(std::string("skeleton-")+implementation_name, skel_image);
cv::waitKey(0);
*/
/*
// Find contours in mask, this needs 8uc1
//
// CV_RETR_LIST Retrieves all of the contours without establishing
//              any hierarchical relationships.
// CV_CHAIN_APPROX_NONE Stores absolutely all the contour points.
std::vector<std::vector<cv::Point> > contours;
std::vector<cv::Vec4i> hierarchy;
cv::findContours(skel_image, contours, hierarchy, CV_RETR_LIST,
                 CV_CHAIN_APPROX_NONE);  // retr tree, approx simple
std::cout << "Found " << contours.size() << " contours" << std::endl;
cv::Mat contours_image = cv::Mat(skel_image.size(), CV_8UC3);
cv::cvtColor(skel_image, contours_image, CV_GRAY2RGB);
cv::RNG rng(12345);  // for color codes
for (size_t i = 0; i < contours.size(); i++)
{
  // Draw each contour in shades of blue
  //cv::Scalar color = cv::Scalar(rng.uniform(0, 255), 0, 0);
  // random colors
  cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
  // 2 = line thickness, 8 = line type
  cv::drawContours(contours_image, contours, i, color,
                   1, 8, // line_thickness=1, 8-connected
                   hierarchy, 0, cv::Point());
}
cv::imshow("contours", contours_image);
cv::waitKey(0);
*/

// Manually set pixels to red color
/*
cv::Mat vertices_image = cv::Mat::zeros(skel_image.size(), CV_8UC3);
struct RGB
{
  uchar blue;
  uchar green;
  uchar red;
};
RGB& rgb = vertices_image.ptr<RGB>(0)[0]; // scanline y, pixel x
rgb.red = 255;
rgb = vertices_image.ptr<RGB>(0)[1]; // scanline y, pixel x
rgb.red = 255;
rgb = vertices_image.ptr<RGB>(1)[0]; // scanline y, pixel x
rgb.red = 255;
cv::imshow("vertices_image", vertices_image);
cv::waitKey(0);
*/


//cv::Mat vertices_image = cv::Mat::zeros(skel_image.size(), CV_8UC3); // empty image


// Find all pixels with more than 2 neighbours:
/*
// This works on 8UC1 image, pixel values are 0 or 255.
// The outer border of pixels are not checked.
cv::Mat vertices_image = cv::Mat(skel_image.size(), CV_8UC3);
cv::cvtColor(skel_image, vertices_image, CV_GRAY2RGB);
struct RGB
{
  uchar blue;
  uchar green;
  uchar red;
};
unsigned int num_4_conn_px = 0;
unsigned int num_8_conn_px = 0;
for (int i = 1; i < skel_image.rows-1; ++i)
{
  const uchar* row_ptr = skel_image.ptr<uchar>(i);
  for (int j = 1; j < skel_image.cols-1; ++j)
  {
    //std::cout << "Pixel value: " << static_cast<int>(row_ptr[j]) << std::endl;

    if (row_ptr[j] == 0)
    {
      // 0 = not the skeleton
      continue;
    }

    num_4_conn_px = 0;
    num_8_conn_px = 0;

    if (skel_image.at<uchar>(i+0,j+1) != 0) num_4_conn_px++;
    if (skel_image.at<uchar>(i+0,j-1) != 0) num_4_conn_px++;
    if (skel_image.at<uchar>(i+1,j+0) != 0) num_4_conn_px++;
    if (skel_image.at<uchar>(i-1,j+0) != 0) num_4_conn_px++;

    if (skel_image.at<uchar>(i+1,j+1) != 0) num_8_conn_px++;
    if (skel_image.at<uchar>(i+1,j-1) != 0) num_8_conn_px++;
    if (skel_image.at<uchar>(i-1,j+1) != 0) num_8_conn_px++;
    if (skel_image.at<uchar>(i-1,j-1) != 0) num_8_conn_px++;

    if ((num_4_conn_px + num_8_conn_px) > 2)
    {
      RGB& pixel = vertices_image.ptr<RGB>(i)[j]; // scanline y, pixel x
      pixel.red = 255;
      pixel.green = 0;
      pixel.blue = 0;
    }
  }
}
cv::imshow("vertices_image", vertices_image);
cv::waitKey(0);
*/

// Test of convolution filter, this just thickens the input image
/*
cv::Mat vertices_image = cv::Mat::zeros(skel_image.size(), CV_8UC1); // empty image
cv::Mat kernel = (cv::Mat_<uchar>(3, 3) << 0, 1, 0,
                                       0, 1, 0,
                                       0, 1, 0);
cv::Mat dst;
cv::filter2D(skel_image, dst, -1 , kernel, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
//dst.convertTo(dst, CV_8UC3);
cv::imshow("vertices_image2", dst);
cv::waitKey(0);
*/



// Find voronoi vertices:

cv::Mat image2 = cv::imread("/home/dbworth/skel_test2.png");
// Convert to 8UC1
cv::Mat skel_image(image2.size(), CV_8UC1);
cv::cvtColor(image2, skel_image, CV_BGR2GRAY);


// This works on 8UC1 image, pixel values are 0 or 255.
// The outer border of pixels are not checked.
cv::Mat vertices_image = cv::Mat(skel_image.size(), CV_8UC3);
cv::cvtColor(skel_image, vertices_image, CV_GRAY2RGB);
struct RGB
{
  uchar blue;
  uchar green;
  uchar red;
};
bool is_junction = false;
for (int i = 1; i < skel_image.rows-1; ++i)
{
  const uchar* row_ptr = skel_image.ptr<uchar>(i);
  for (int j = 1; j < skel_image.cols-1; ++j)
  {
    //std::cout << "Pixel value: " << static_cast<int>(row_ptr[j]) << std::endl;

    if (row_ptr[j] == 0)
    {
      // 0 = not the skeleton
      continue;
    }

    is_junction = false;

    // ToDo: delete +/- zero from below



    if ((skel_image.at<uchar>(i-1,j-1) != 0) &&
        (skel_image.at<uchar>(i-1,j+1) != 0) &&
        (skel_image.at<uchar>(i+1,j-1) != 0) &&
        (skel_image.at<uchar>(i+1,j+1) != 0))
    {
      // 4 branches, 8-connected
      is_junction = true;
      std::cout << "Matched 0 at " << i << ", " << j << std::endl;
    }



    else if ((skel_image.at<uchar>(i-1,j-1) != 0) &&
             (skel_image.at<uchar>(i-1,j+1) != 0) &&
             (skel_image.at<uchar>(i+1,j-1) != 0))
    {
      // 3 branches, 8-connected
      is_junction = true;
      std::cout << "Matched 1 at " << i << ", " << j << std::endl;
    }
    else if ((skel_image.at<uchar>(i-1,j-1) != 0) &&
             (skel_image.at<uchar>(i-1,j+1) != 0) &&
             (skel_image.at<uchar>(i+1,j+1) != 0))
    {
      // 3 branches, 8-connected
      is_junction = true;
      std::cout << "Matched 2 at " << i << ", " << j << std::endl;
    }
    else if ((skel_image.at<uchar>(i-1,j-1) != 0) &&
             (skel_image.at<uchar>(i+1,j-1) != 0) &&
             (skel_image.at<uchar>(i+1,j+1) != 0))
    {
      // 3 branches, 8-connected
      is_junction = true;
      std::cout << "Matched 3 at " << i << ", " << j << std::endl;
    }
    else if ((skel_image.at<uchar>(i-1,j+1) != 0) &&
             (skel_image.at<uchar>(i+1,j-1) != 0) &&
             (skel_image.at<uchar>(i+1,j+1) != 0))
    {
      // 3 branches, 8-connected
      is_junction = true;
      std::cout << "Matched 4 at " << i << ", " << j << std::endl;
    }




    else if ((skel_image.at<uchar>(i-1,j+0) != 0) &&
             (skel_image.at<uchar>(i+0,j-1) != 0) &&
             (skel_image.at<uchar>(i+0,j+1) != 0) &&
             (skel_image.at<uchar>(i+1,j+0) != 0))
    {
      // 4 branches, 4-connected
      is_junction = true;
      std::cout << "Matched 5 at " << i << ", " << j << std::endl;
    }



    else if ((skel_image.at<uchar>(i-1,j+0) != 0) &&
             (skel_image.at<uchar>(i+0,j-1) != 0) &&
             (skel_image.at<uchar>(i+0,j+1) != 0))
    {
      // 3 branches, 4-connected
      is_junction = true;
      std::cout << "Matched 6 at " << i << ", " << j << std::endl;
    }
    else if ((skel_image.at<uchar>(i+0,j-1) != 0) &&
             (skel_image.at<uchar>(i+0,j+1) != 0) &&
             (skel_image.at<uchar>(i+1,j+0) != 0))
    {
      // 3 branches, 4-connected
      is_junction = true;
      std::cout << "Matched 7 at " << i << ", " << j << std::endl;
    }
    else if ((skel_image.at<uchar>(i-1,j+0) != 0) &&
             (skel_image.at<uchar>(i+0,j+1) != 0) &&
             (skel_image.at<uchar>(i+1,j+0) != 0))
    {
      // 3 branches, 4-connected
      is_junction = true;
      std::cout << "Matched 8 at " << i << ", " << j << std::endl;
    }
    else if ((skel_image.at<uchar>(i-1,j+0) != 0) &&
             (skel_image.at<uchar>(i+0,j-1) != 0) &&
             (skel_image.at<uchar>(i+1,j+0) != 0))
    {
      // 3 branches, 4-connected
      is_junction = true;
      std::cout << "Matched 9 at " << i << ", " << j << std::endl;
    }



    else if ((skel_image.at<uchar>(i-1,j-1) != 0) &&
             (skel_image.at<uchar>(i+0,j+1) != 0) &&
             (skel_image.at<uchar>(i+1,j+0) != 0))
    {
      // two 4-connected, one 8-connected
      is_junction = true;
      std::cout << "Matched 10 at " << i << ", " << j << std::endl;
    }
    else if ((skel_image.at<uchar>(i-1,j+1) != 0) &&
             (skel_image.at<uchar>(i+0,j-1) != 0) &&
             (skel_image.at<uchar>(i+1,j+0) != 0))
    {
      // two 4-connected, one 8-connected
      is_junction = true;
      std::cout << "Matched 11 at " << i << ", " << j << std::endl;
    }
    else if ((skel_image.at<uchar>(i-1,j+0) != 0) &&
             (skel_image.at<uchar>(i+0,j+1) != 0) &&
             (skel_image.at<uchar>(i+1,j-1) != 0))
    {
      // two 4-connected, one 8-connected
      is_junction = true;
      std::cout << "Matched 12 at " << i << ", " << j << std::endl;
    }
    else if ((skel_image.at<uchar>(i-1,j+0) != 0) &&
             (skel_image.at<uchar>(i+0,j-1) != 0) &&
             (skel_image.at<uchar>(i+1,j+1) != 0))
    {
      // two 4-connected, one 8-connected
      is_junction = true;
      std::cout << "Matched 13 at " << i << ", " << j << std::endl;
    }



    else if ((skel_image.at<uchar>(i-1,j+0) != 0) &&
             (skel_image.at<uchar>(i+1,j-1) != 0) &&
             (skel_image.at<uchar>(i+1,j+1) != 0))
    {
      // one 4-connected, two 8-connected
      is_junction = true;
      std::cout << "Matched 14 at " << i << ", " << j << std::endl;
    }
    else if ((skel_image.at<uchar>(i-1,j-1) != 0) &&
             (skel_image.at<uchar>(i-1,j+1) != 0) &&
             (skel_image.at<uchar>(i+1,j+0) != 0))
    {
      // one 4-connected, two 8-connected
      is_junction = true;
      std::cout << "Matched 15 at " << i << ", " << j << std::endl;
    }
    else if ((skel_image.at<uchar>(i+0,j-1) != 0) &&
             (skel_image.at<uchar>(i-1,j+1) != 0) &&
             (skel_image.at<uchar>(i+1,j+1) != 0))
    {
      // one 4-connected, two 8-connected
      is_junction = true;
      std::cout << "Matched 16 at " << i << ", " << j << std::endl;
    }
    else if ((skel_image.at<uchar>(i-1,j-1) != 0) &&
             (skel_image.at<uchar>(i+0,j+1) != 0) &&
             (skel_image.at<uchar>(i+1,j-1) != 0))
    {
      // one 4-connected, two 8-connected
      is_junction = true;
      std::cout << "Matched 17 at " << i << ", " << j << std::endl;
    }





    if (is_junction)
    {
      RGB& pixel = vertices_image.ptr<RGB>(i)[j]; // scanline y, pixel x
      pixel.red = 255;
      pixel.green = 0;
      pixel.blue = 0;
    }
  }
}
cv::imshow("vertices_image", vertices_image);
cv::waitKey(0);


/*
// Manually create a binary image
unsigned char bits[] = {0,255,0,255,0,255,0,255, 255,0,255,0,255,0,255,0, 0,255,0,255,0,255,0,255, 255,0,255,0,255,0,255,0}; // 255 = white
cv::Mat m(4,8, CV_8UC1, bits); // height 4, width 8
// Write .png image using libPNG
const std::vector<int> params = {CV_IMWRITE_PNG_COMPRESSION, 0};
cv::imwrite(png_output_path, m, params);
*/





  // Choose both false, or one true
  //bool doPrune = false;
  bool doPrune = true;
  //bool doPruneAlternative = false;
  // LOAD PGM MAP AND INITIALIZE THE VORONOI
  //std::ifstream is("/home/dbworth/testmap.pgm");
  //std::ifstream is("/home/dbworth/binary_image1_ASCII.pgm"); // header has P2
  //std::ifstream is("/home/dbworth/binary_image1_raw.pgm"); // header has P5, this works
  std::ifstream is("/home/dbworth/binary_image3_raw.pgm");
  if (!is) {
    std::cerr << "Could not open map file for reading.\n";
    exit(-1);
  }
  bool **map=NULL;
  int sizeX, sizeY;
  loadPGM( is, &sizeX, &sizeY, &map );
  is.close();
  fprintf(stderr, "Map loaded (%dx%d).\n", sizeX, sizeY);

  // create the voronoi object and initialize it with the map
  DynamicVoronoi voronoi;
  voronoi.initializeMap(sizeX, sizeY, map);
  voronoi.update(); // update distance map and Voronoi diagram
  if (doPrune) voronoi.prune();  // prune the Voronoi
  //if (doPruneAlternative) voronoi.updateAlternativePrunedDiagram();  // prune the Voronoi, not supposed in this version
  // Save image file
  voronoi.visualize("/home/dbworth/output.ppm");
  std::cerr << "Generated initial frame.\n";






  //if (publish_ros_msg)
  {
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;

    //std::cout << "Publishing data as ROS msgs..." << std::endl;



    const sensor_msgs::PointCloud2ConstPtr input_cloud_msg = makePointCloudMsg(*input_pointcloud);
    const sensor_msgs::PointCloud2ConstPtr cropped_cloud_msg = makePointCloudMsg(*cropped_cloud);

    ros::Publisher input_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("input_cloud", 1);
    ros::Publisher cropped_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1);


    //ros::Publisher occupancy_grid_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("occupancy_grid_cloud", 1);


    ros::Rate loop_rate(10);
    while (ros::ok())
    {
      //occupancy_grid_cloud_pub.publish(occupancy_grid_cloud_msg);
      input_cloud_pub.publish(input_cloud_msg);
      cropped_cloud_pub.publish(cropped_cloud_msg);


      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  return 0;
}
