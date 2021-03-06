/*
Example 24:

Usage:

ex24 /home/dbworth/skeleton/binary_image1.png /home/dbworth/skeleton/test1.png

ex24 /home/dbworth/skeleton/binary_image2.png /home/dbworth/skeleton/test2.png

*/

#include <iostream> // cout
//#include <string>
//#include <vector>

//#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h> // PointCloud2ConstPtr

//#include <Eigen/Geometry>

//#include <boost/algorithm/string.hpp> // to_upper

#include <planning_through_pointclouds/utils.h> // splitString
//#include <planning_through_pointclouds/ompl_utils.h> // getPlannerGraphMarker
#include <planning_through_pointclouds/pcl_utils.h> // loadPointCloud, getPointCloudBounds
//#include <planning_through_pointclouds/ros_utils.h> // makePointCloudMsg
#include <planning_through_pointclouds/opencv_utils.h> // printCvMatType

// Remove PCL ?

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointXYZRGBA ColorPoint;
typedef pcl::PointCloud<ColorPoint> ColorPointCloud;

//#include <pcl/io/ply_io.h> // savePLYFileASCII
#include <pcl/io/png_io.h> // savePNGFile

#include <opencv2/core/core.hpp> // normalize
#include <opencv2/highgui/highgui.hpp> // imwrite
#include <opencv2/imgproc/imgproc.hpp> // cvtColor


////#include <voronoi_2d/voronoi.hpp> // WHY DOESNT THIS WORK ??
#include <voronoi_2d/voronoi/src/voronoi.h> // VoronoiThinner - Zhang & Guo algorithms
#include <voronoi_2d/evg-thin/evg-thin.hh>
//#include <voronoi_2d/evg-thin/fileio.hh> // TEMP
#include <voronoi_2d/dynamic_voronoi/src/dynamicvoronoi.h> // Boris Lau

//#include <dynamic_voronoi/dynamicvoronoi.h> // Modified version of Boris Lau

#include <limits>

#include <iostream>
#include <fstream>
#include <string.h>

#include <boost/lexical_cast.hpp>




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



// Find the voronoi approximation of an image, using one of 3 skeleton-ization algorithms.
// Writes the result to a .png file.
// Input image is 8UC1
void applyVoronoiThinner(const cv::Mat& binary_image,
                         const std::string implementation_name,
                         const bool draw_on_orig_image,
                         const std::string output_file_prefix)
{
  VoronoiThinner thinner;
  const bool crop_image_before = false;
  const bool ok = thinner.thin(binary_image, implementation_name, crop_image_before);
  if (!ok)
  {
    std::cout << "Failed thinning" << std::endl;
  }
  cv::Mat1b skel_image = thinner.get_skeleton(); // 8UC1

/*
  std::cout << "in applyVoronoiThinner()" << std::endl;
  std::cout << "  binary image size: " << binary_image.rows << "," << binary_image.cols
            << "  skel image size: " << skel_image.rows << "," << skel_image.cols << std::endl;

  if ((skel_image.rows < binary_image.rows) || (skel_image.cols < binary_image.cols))
  {
    // If the input image has a wide border of black pixels, the
    // skeleton algorithms seem to crop the image.
    // Therefore, pad the output skeleton image with black pixels
    // so its size matches that of the original image.
    const unsigned int row_diff = binary_image.rows - skel_image.rows;
    const unsigned int col_diff = binary_image.cols - skel_image.cols;
    const float half_row_diff = static_cast<float>(row_diff) / 2.0;
    const float half_col_diff = static_cast<float>(col_diff) / 2.0;

    unsigned int pad_top = static_cast<unsigned int>(std::ceil(half_row_diff));
    unsigned int pad_bottom = static_cast<unsigned int>(std::floor(half_row_diff));
    unsigned int pad_left = static_cast<unsigned int>(std::ceil(half_col_diff));
    unsigned int pad_right = static_cast<unsigned int>(std::floor(half_col_diff));

    // Shift it down and right by 1 pixel
    //if (row_diff > 2)
    //{
    //  pad_top += 1;
    //  pad_bottom -= 1;
    //}
    if (col_diff > 2)
    {
      pad_left += 1;
      pad_right -= 1;
    }

    std::cout << "row diff = " << row_diff << "   pad: " <<  pad_top << ", " <<  pad_bottom << std::endl;
    std::cout << "col diff = " << col_diff << "   pad: " <<  pad_left << ", " <<  pad_right << std::endl;


    cv::Mat padded_skel_image;
    //cv::copyMakeBorder(skel_image, padded_skel_image, pad_top, pad_bottom, pad_left, pad_right, cv::BORDER_CONSTANT, cv::Scalar(0));

cv::copyMakeBorder(skel_image, padded_skel_image, 63,0, pad_left, pad_right, cv::BORDER_CONSTANT, cv::Scalar(0));

    //skel_image = padded_skel_image;
    skel_image = padded_skel_image.clone();
  }
*/

  const std::vector<int> png_params = {CV_IMWRITE_PNG_COMPRESSION, 0}; // libPNG parameters
  const std::string outfile = output_file_prefix + std::string("_") + implementation_name + std::string(".png");

  if (!draw_on_orig_image)
  {
    cv::imwrite(outfile, skel_image, png_params);
  }
  else
  {
    cv::Mat out_image = cv::Mat::zeros(binary_image.size(), CV_8UC3);
    cv::cvtColor(binary_image, out_image, CV_GRAY2RGB);

    struct RGB
    {
      uchar blue;
      uchar green;
      uchar red;
    };

    for (int i = 0; i < skel_image.rows; ++i)
    {
      const uchar* row_ptr = skel_image.ptr<uchar>(i);
      for (int j = 0; j < skel_image.cols; ++j)
      {
        if (row_ptr[j] == 255)
        {
          // A white pixel
          RGB& pixel = out_image.ptr<RGB>(i)[j]; // scanline y, pixel x
          pixel.red = 255;
          pixel.green = 0;
          pixel.blue = 0;
        }
      }
    }

    cv::imwrite(outfile, out_image, png_params);
  }
}

// Convert a cv::Mat to an EVG-Thin grid.
// Input image is 8UC1
const grid_type makeGridFromCvMat(const cv::Mat& binary_image,
                                  const unsigned int unknown_min,
                                  const unsigned int unknown_max)
{
  // Create a grid where all cells are labelled "unknown"
  column_type columns(binary_image.rows, Unknown);
  grid_type grid(binary_image.cols, columns);

  for (int i = 0; i < binary_image.rows; ++i)
  {
    const uchar* row_ptr = binary_image.ptr<uchar>(i);
    for (int j = 0; j < binary_image.cols; ++j)
    {
      //std::cout << "Pixel value: " << static_cast<int>(row_ptr[j]) << std::endl;

      if (row_ptr[j] > unknown_max)
      {
        // Label this cell as free space
        grid[j][i] = Free; // cols,rows
      }
      else if (row_ptr[j] < unknown_min)
      {
        // Label this cell as occupied
        grid[j][i] = Occupied; // cols,rows
      }
    }
  }

  return grid;
}

// Draw an EVG-Thin skeleton on top of the original binary image
// and write the result to a .png file.
//
// Input image is 8UC1
// draw_on_orig_image = False, output white skeleton on black image.
//                  True, draw red skeleton on original image.
void saveSkel(const skeleton_type& skel,
              const cv::Mat& binary_image,
              const bool draw_on_orig_image,
              const std::string output_file_prefix)
{
  cv::Mat skeleton_image = cv::Mat::zeros(binary_image.size(), CV_8UC3);

  if (draw_on_orig_image)
  {
    cv::cvtColor(binary_image, skeleton_image, CV_GRAY2RGB);
  }

  struct RGB
  {
    uchar blue;
    uchar green;
    uchar red;
  };

  for (unsigned int i = 0; i < skel.size(); ++i)
  {
    RGB& pixel = skeleton_image.ptr<RGB>(skel[i].y)[skel[i].x]; // scanline y, pixel x
    if (draw_on_orig_image)
    {
      pixel.red = 255;
      pixel.green = 0;
      pixel.blue = 0;
    }
    else
    {
      pixel.red = 255;
      pixel.green = 255;
      pixel.blue = 255;
    }
  }

  const std::vector<int> png_params = {CV_IMWRITE_PNG_COMPRESSION, 0}; // libPNG parameters
  const std::string outfile = output_file_prefix + std::string(".png");
  cv::imwrite(outfile, skeleton_image, png_params);
}

// Apply the EVG-Thin algorithm to an image.
// Writes the result to a .png file.
// Input image is 8UC1
void applyEVGThin(const cv::Mat& binary_image,
                  const float distance_max,
                  const bool draw_on_orig_image,
                  const std::string output_file_prefix)
{
  const unsigned int unknown_min = 127; // Occupied space = 0 to 126
  const unsigned int unknown_max = 128; // Free space = 129 to 255
  const grid_type grid = makeGridFromCvMat(binary_image, unknown_min, unknown_max);

  // Don't prune, it removes the voronoi if doesn't touch edges
  const bool pruning = false;

  // Don't prune voronoi based on robot's location
  const bool robot_close = false;
  const int robot_locx = 0;
  const int robot_locy = 0;

  const float distance_min = 0.0;
  //float distance_max = std::numeric_limits<float>::max();
  //float distance_max = 50.0;

  evg_thin thin(grid, distance_min, distance_max, pruning, robot_close, robot_locx, robot_locy);

  const skeleton_type skel = thin.generate_skeleton();

  std::string prefix;
  if (distance_max < std::numeric_limits<float>::max()-0.01)
  {
    prefix = output_file_prefix + std::string("_evg-thin_max") + boost::lexical_cast<std::string>(distance_max);
  }
  else
  {
    prefix = output_file_prefix + std::string("_evg-thin_maxInf");
  }
  saveSkel(skel, binary_image, draw_on_orig_image, prefix);
}

// Convert a cv::Mat to a Dynamic Voronoi Map.
// Input image is 8UC1
void makeMapFromCvMat(const cv::Mat& binary_image,
                      int* size_x, int* size_y,
                      bool*** map)
{
  *size_x = binary_image.cols; // width
  *size_y = binary_image.rows; // height

  // Create the Map
  *map = new bool*[*size_x];
  for (int x = 0; x < *size_x; ++x)
  {
    (*map)[x] = new bool[*size_y];
  }

  for (int i = 0; i < binary_image.rows; ++i)
  {
    const uchar* row_ptr = binary_image.ptr<uchar>(i);
    for (int j = 0; j < binary_image.cols; ++j)
    {
      const unsigned int x = j; // col

      // Map is stored with rows flipped top-to-bottom
      const unsigned int y = *size_y - 1 - i; // row

      if (static_cast<double>(row_ptr[j]) < 255.0-255.0*0.2)
      {
        // Cell is occupied
        (*map)[x][y] = true;
      }
      else
      {
        // Cell is free space
        (*map)[x][y] = false;
      }
    }
  }
}

/*
// Load PGM image into a Dynamic Voronoi Map
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
*/

// HMT (Morphological Hit-or-Miss Transform)
// 
//
//  src     The source image, 8 bit single-channel matrix
//   dst     The destination image
//   kernel  The kernel matrix. 1=foreground, -1=background, 0=don't care

// foreground = hit, background = miss

// Based on:
// http://github.com/opencv/opencv/pull/5515
// https://github.com/opencv/opencv/pull/5622


// Based on OpenCV implementation, added after v2.4
// Returns values of 0 or 255
void hmt(const cv::Mat& src, const cv::Mat& kernel, cv::Mat& dst)
{
  CV_Assert(src.type() == CV_8UC1);

  cv::Mat k1 = (kernel == 1);
  cv::Mat k2 = (kernel == -1);

  cv::Mat e1;
  cv::Mat e2;

  // Default values for function morphologyEx()
  cv::Point anchor = cv::Point(-1, -1);
  int iterations = 1;
  int borderType = cv::BORDER_CONSTANT;
  cv::Scalar borderValue = cv::morphologyDefaultBorderValue(); //cv::Scalar(0);

  if (cv::countNonZero(k1) <= 0)
  {
      e1 = src;
  }
  else
  {
      cv::erode(src, e1, k1, anchor, iterations, borderType, borderValue);
  }
  if (countNonZero(k2) <= 0)
      e2 = src;
  else
  {
      cv::Mat src_complement;
      cv::bitwise_not(src, src_complement);
      cv::erode(src_complement, e2, k2, anchor, iterations, borderType, borderValue);
  }
  dst = e1 & e2;
}


// This also works, but returns values of 0 or 1
//
// http://answers.opencv.org/question/72802/hit-and-miss-transform/
// https://web.archive.org/web/20160228055449/http://opencv-code.com/tutorials/hit-or-miss-transform-in-opencv/
void hmt2(const cv::Mat& src, const cv::Mat& kernel, cv::Mat& dst)
{
  CV_Assert(src.type() == CV_8U && src.channels() == 1);

  const cv::Mat k1 = (kernel == 1) / 255;
  const cv::Mat k2 = (kernel == -1) / 255;

  // Normalize image of 0:255 values to 0:1
  cv::Mat normalized_src = cv::Mat::zeros(src.size(), CV_8UC1);
  cv::normalize(src, normalized_src, 0, 1, cv::NORM_MINMAX);

  cv::Mat e1;
  cv::Mat e2;
  cv::erode(src, e1, k1, cv::Point(-1, -1), 1, cv::BORDER_CONSTANT, cv::Scalar(0));
  cv::erode(1-src, e2, k2, cv::Point(-1, -1), 1, cv::BORDER_CONSTANT, cv::Scalar(0));
  if (countNonZero(k2) <= 0)
  {
    e2 = src;
  }
  dst = e1 & e2;
}


// The 3 best structing elements for 4-connected thinning:
    /* Sels for 4-connected thinning */
// o = miss
// x = hit
// otherwise "dont care"
/*
static const char *sel_4_1 = "  x"
                             "oCx"
                             "  x";

static const char *sel_4_2 = "  x"
                             "oCx"
                             " o ";

static const char *sel_4_3 = " o "
                             "oCx"
                             "  x";
*/


const int strel_4[3][9] = {
  // 4_1
  { 0, 0, 1,
   -1, 0, 1,
    0, 0, 1},
  // 4_2
  { 0, 0, 1,
   -1, 0, 1,
    0,-1, 0},
  // 4_3
  { 0,-1, 0,
   -1, 0, 1,
    0, 0, 1}
};

/*
void thin()
{
  const int max_iters = 10;
  for (uint i = 0; i < maxiters; ++i)
  {
    // Try 90 degree rotations
    for (uint r = 0; r < 4; ++r)
    {
      // 3 x elements
      for (uint j = 0; j < 3; ++j)
      {

      }

    }

}

    /* Thin the fg, with up to maxiters iterations
for (i = 0; i < maxiters; i++) {
    pixt = pixCopy(NULL, pixd);  /* test for completion
    for (r = 0; r < 4; r++) {  /* over 90 degree rotations of Sels
        for (j = 0; j < nsels; j++) {  /* over individual sels in sela
            sel = selaGetSel(sela, j);  /* not a copy
            selr = selRotateOrth(sel, r);
            pixHMT(pixhmt[j], pixd, selr);
            selDestroy(&selr);
            if (j > 0)
                pixOr(pixhmt[0], pixhmt[0], pixhmt[j]);  /* accum result
        }
        pixSubtract(pixd, pixd, pixhmt[0]);  /* remove result 
    }
    pixEqual(pixd, pixt, &same);
    pixDestroy(&pixt);
    if (same) {
        L_INFO("%d iterations to completion\n", procName, i);
        break;
    }
}
*/


//----------------------------------------------------------------------------//

int main(int argc, char **argv)
{
  
  std::string input_filename("");
  std::string output_filename("");
  if (argc < 3)
  {
    std::cout << "ERROR, wrong number of arguments!" << std::endl;
    std::cout << "Usage: " << argv[0] <<" </path/to/image.png> </output/file.png> " << std::endl;
    return EXIT_FAILURE;
  }
  else
  {
    input_filename = std::string(argv[1]);
    output_filename = std::string(argv[2]);
  }
  //std::cout << "pointcloud_file =  "<< pointcloud_file << std::endl;
  

  const std::vector<std::string> output_filename_tokens = splitString(output_filename);
  std::string outfile;

/*
cv::Mat a = (cv::Mat_<uchar>(8,8) << 0, 0, 0, 0, 0, 0, 0, 0,
                                     0, 1, 1, 1, 0, 0, 0, 0, 
                                     0, 1, 0, 1, 0, 0, 0, 1,
                                     0, 1, 1, 1, 0, 1, 0, 0,
                                     0, 0, 1, 0, 0, 0, 0, 0,
                                     0, 0, 1, 0, 0, 1, 1, 0,
                                     0, 1, 0, 1, 0, 0, 1, 0,
                                     0, 1, 1, 1, 0, 0, 0, 0 );
*/
cv::Mat a = (cv::Mat_<uchar>(8,8) << 0, 0, 0, 0, 0, 0, 0, 0,
                                     0, 255, 255, 255, 0, 0, 0, 0, 
                                     0, 255, 0, 255, 0, 0, 0, 255,
                                     0, 255, 255, 255, 0, 255, 0, 0,
                                     0, 0, 255, 0, 0, 0, 0, 0,
                                     0, 0, 255, 0, 0, 255, 255, 0,
                                     0, 255, 0, 255, 0, 0, 255, 0,
                                     0, 255, 255, 255, 0, 0, 0, 0 );

cv::Mat b = (cv::Mat_<char>(3,3) << 0, 1, 0, 1,-1, 1, 0, 1, 0 );


std::cout << "\n HMT" << std::endl;
cv::Mat hitmiss = cv::Mat::zeros(a.size(), CV_8UC1);
hmt(a, b, hitmiss);
std::cout << hitmiss << std::endl;

std::cout << "\n HMT2" << std::endl;
hitmiss = cv::Mat::zeros(a.size(), CV_8UC1);
hmt2(a, b, hitmiss);
std::cout << hitmiss << std::endl;



cv::Mat image = (cv::Mat_<uchar>(8, 8) << 
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 255, 255, 255, 0, 0, 0, 0,
    0, 255, 0, 255, 0, 0, 0, 255,
    0, 255, 255, 255, 0, 255, 0, 0,
    0, 0, 255, 0, 0, 0, 0, 0,
    0, 0, 255, 0, 0, 255, 255, 0,
    0, 255, 0, 255, 0, 0, 255, 0,
    0, 255, 255, 255, 0, 0, 0, 0);

  cv::Mat kernel = (cv::Mat_<char>(3, 3) << 
    0, -1, -1,
    0, 1, -1,
    0, 0, 0);

 // Mat hitmiss;
  hitmiss = cv::Mat::zeros(a.size(), CV_8UC1);
  //morphologyEx(image, hitmiss, MORPH_HITMISS, kernel);
  hmt(image, kernel, hitmiss);

  cv::namedWindow("Original", CV_WINDOW_NORMAL);
  cv::imshow("Original", image);
  cv::namedWindow("Hit and Miss", CV_WINDOW_NORMAL);
  cv::imshow("Hit and Miss", hitmiss);
  cv::waitKey();




/*

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


// Perform the distance transform:
//
// Uses: P. Felzenszwalb and D. Huttenlocher, "Distance transforms of sampled functions", 2004.
cv::Mat dist;
//cv::distanceTransform(binary_image, dist, CV_DIST_L2, 3); // Fast, coarse distance estimation with 3x3 mask
cv::distanceTransform(binary_image, dist, CV_DIST_L2, CV_DIST_MASK_PRECISE); // Exact method
// Normalize the distance image for range = {0.0, 1.0}
// so we can visualize and threshold it
cv::normalize(dist, dist, 0.0, 1.0, cv::NORM_MINMAX);
printCvMatType(dist); // CV_32FC1
// Convert 32FC1 to 8u
double min_val;
double max_val;
cv::minMaxLoc(dist, &min_val, &max_val);
cv::Mat dist_image_8u;
dist.convertTo(dist_image_8u, CV_8U, 255.0/(max_val - min_val), -min_val * 255.0/(max_val - min_val));
// Save image to file
outfile = output_filename_tokens[0] + std::string("_distance_transform.png");
cv::imwrite(outfile, dist_image_8u, png_params);

// Skeleton (3 implementations)



//    1984, Zhang - Suen explained in "A fast parallel algorithm for thinning digital patterns" by T.Y. Zhang and C.Y. Suen and based on implentation by http://opencv-code.com/quick-tips/implementation-of-thinning-algorithm-in-opencv/
//    1989, Guo - Hall explained in "Parallel thinning with two sub-iteration algorithms" by Zicheng Guo and Richard Hall and based on implentation by http://opencv-code.com/quick-tips/implementation-of-guo-hall-thinning-algorithm/
//    a morphological one, based on the erode() and dilate() operators.  
//    Coming from: http://felix.abecassis.me/2011/09/opencv-morphological-skeleton/

// There's also a paper by Zhang and Wang on another parallel implementation.

const bool draw_on_orig_image = true;
const std::string output_file_prefix = output_filename_tokens[0] + std::string("_skeleton");
applyVoronoiThinner(binary_image, "zhang_suen_fast", draw_on_orig_image, output_file_prefix);
applyVoronoiThinner(binary_image, "guo_hall_fast", draw_on_orig_image, output_file_prefix);
applyVoronoiThinner(binary_image, "morph", draw_on_orig_image, output_file_prefix);
// White on black skeleton:
outfile = output_file_prefix + std::string("_bw");
applyVoronoiThinner(binary_image, "zhang_suen_fast", false, outfile);
applyVoronoiThinner(binary_image, "guo_hall_fast", false, outfile);



// ToDo: Add namespace evg_thin

// Voronoi using EVG-thin
float distance_max;
distance_max = std::numeric_limits<float>::max();
applyEVGThin(binary_image, distance_max, draw_on_orig_image, output_filename_tokens[0]);
distance_max = 20.0;
applyEVGThin(binary_image, distance_max, draw_on_orig_image, output_filename_tokens[0]);
distance_max = 40.0;
applyEVGThin(binary_image, distance_max, draw_on_orig_image, output_filename_tokens[0]);
distance_max = 60.0;
applyEVGThin(binary_image, distance_max, draw_on_orig_image, output_filename_tokens[0]);
distance_max = 80.0;
applyEVGThin(binary_image, distance_max, draw_on_orig_image, output_filename_tokens[0]);


// Voronoi using DynamicVoronoi
bool** map = NULL;
int size_x;
int size_y;
DynamicVoronoi voronoi;
makeMapFromCvMat(binary_image, &size_x, &size_y, &map);
const std::string output_file_prefix2 = output_filename_tokens[0] + std::string("_dyn-voronoi");
// Get the 4-connected voronoi
voronoi.initializeMap(size_x, size_y, map);
voronoi.update(); // update distance map and voronoi diagram
outfile = output_file_prefix2 + std::string("_orig.ppm");
//voronoi.visualize(outfile.c_str());
voronoi.saveAsPPM(outfile.c_str()); // Doesn't draw distance gradient
// Pruning method #1
voronoi.initializeMap(size_x, size_y, map);
voronoi.update(); // update distance map and voronoi diagram
voronoi.prune();
outfile = output_file_prefix2 + std::string("_pruned.ppm");
voronoi.saveAsPPM(outfile.c_str()); // Doesn't draw distance gradient
// Pruning method #2
voronoi.initializeMap(size_x, size_y, map);
voronoi.update(); // update distance map and voronoi diagram
voronoi.updateAlternativePrunedDiagram(); // alternative pruning method
outfile = output_file_prefix2 + std::string("_pruned_alt.ppm");
voronoi.saveAsPPM(outfile.c_str()); // Doesn't draw distance gradient

// Get the pruned voronoi as white-on-black image,
// with 1 pixel of padding to account for the strange
// border added by DynamicVoronoi.
cv::Mat padded_binary_image;
cv::copyMakeBorder(binary_image, padded_binary_image, 1, 1, 1, 1, cv::BORDER_REPLICATE);
makeMapFromCvMat(padded_binary_image, &size_x, &size_y, &map);
voronoi.initializeMap(size_x, size_y, map); // Get the 4-connected voronoi
voronoi.update(); // update distance map and voronoi diagram
voronoi.prune();
outfile = output_file_prefix2 + std::string("_pruned2_bw_padded.ppm");
voronoi.saveAsPPM(outfile.c_str(), false); // Doesn't draw distance gradient
// Read in the .ppm file and crop the border
cv::Mat padded_image = cv::imread(outfile);
const int width = padded_image.cols - 2;
const int height = padded_image.rows - 2;
cv::Mat cropped_image = padded_image(cv::Rect(1, 1, width, height)).clone();
outfile = output_file_prefix2 + std::string("_pruned3_bw_cropped.png");
cv::imwrite(outfile, cropped_image, png_params);
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





/*
// Manually create a binary image
unsigned char bits[] = {0,255,0,255,0,255,0,255, 255,0,255,0,255,0,255,0, 0,255,0,255,0,255,0,255, 255,0,255,0,255,0,255,0}; // 255 = white
cv::Mat m(4,8, CV_8UC1, bits); // height 4, width 8
// Write .png image using libPNG
const std::vector<int> params = {CV_IMWRITE_PNG_COMPRESSION, 0};
cv::imwrite(png_output_path, m, params);
*/







// Voronoi tesselation using SubDiv2D
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

  return 0;
}
