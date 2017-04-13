/*
Example 28:

Load an image with vertices as red pixels

Usage:

ex28 4 /home/dbworth/skeleton/MyMap1_cropped_DynVoronoi_vertices_method3.png /home/dbworth/skeleton/test1.png


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



#include <planning_through_pointclouds/connected_components.h> // Wu's improved version of Scan Array Union Find


// Sort vector of cv::Point
// could also be point2f - make a template Arg

#include <algorithm>

// Lexicographic compare, same as for ordering words in a dictionnary:
// test first 'letter of the word' (x coordinate), if same, test 
// second 'letter' (y coordinate).
bool lexico_compare(const cv::Point& p1, const cv::Point& p2)
{
    if (p1.x < p2.x) { return true; }
    if (p1.x > p2.x) { return false; }
    return (p1.y < p2.y);
}


 bool points_are_equal(const cv::Point& p1, const cv::Point& p2)
 {
   return ((p1.x == p2.x) && (p1.y == p2.y));
 }

void remove_duplicates(std::vector<cv::Point>& points)
{
    // Note: std::unique leaves a 'queue' of duplicated elements
    // at the end of the vector, and returns an iterator that indicates
    // where to stop (and where to 'erase' the queue)
    std::sort(points.begin(), points.end(), lexico_compare);
    points.erase(std::unique(points.begin(), points.end(), points_are_equal), points.end());
}




// -------

// Store a pixel index (i,j) = row,col
struct Pixel
{
  int i;
  int j;
  Pixel(const int _i, const int _j) : i(_i), j(_j) {};
};

enum VERTEX_TYPE
{
  VERTEX = 1,
  ENDPOINT,
  EXTERIOR_ENDPOINT
};

// Vertex object:
//    Position cv::Point(x,y)
//    Connected pixels: vector of cv::Point(x,y)
//    Edges: vector of indices into 'contours' vector
struct Vertex
{
  cv::Point position;
  uint type; 
  //std::vector<cv::Point> connected_pixels_4;
  //std::vector<cv::Point> connected_pixels_8;
  std::vector<int> edges;
  //Vertex(const cv::Point _position,
  //       const std::vector<cv::Point> _connected_pixels_4)
  //        : position(_position)
  //        , connected_pixels_4(_connected_pixels_4) {};
  //Vertex(const int x, const int y) : position(cv::Point(x,y)) {};
  Vertex(const int i, const int j, const uint _type) : position(cv::Point(j,i)), type(_type) {};
};

// Edge object:
//     v0:
//     v1:
struct Edge
{
  int v0;
  int v1;
  std::vector<cv::Point> pixels;
  Edge(const int _v0, const int _v1) : v0(_v0), v1(_v1) {};
};

/*
// Add an edge (representing a connected component of pixels) to the list of vertices
void addEdge(const cv::Point& point, const int edge_idx, std::vector<Vertex>& vertices)
{
  // Find the vertex
  for (size_t i = 0; i < vertices.size(); ++i)
  {
    for (size_t j = 0; j < vertices.at(j).connected_pixels_4.size(); ++j) // 4 pixels to check
    {
      if (vertices.at(i).connected_pixels_4.at(j) == point)
      {
        vertices.at(i).edges.push_back(edge_idx);
      }
    }
  }
}
*/


const bool is4Connected(const cv::Point& p0, const cv::Point& p1)
{
  const unsigned int x_diff = std::abs(p0.x - p1.x);
  const unsigned int y_diff = std::abs(p0.y - p1.y);
  //std::cout << "       Pixel: " << p0 << "  cc p1: " << p1 << "   x_diff: " << x_diff << "  y_diff: " << y_diff << std::endl;
  if (((x_diff == 1) && (y_diff == 0)) || ((x_diff == 0) && (y_diff == 1)))
  {
    return true;
  }
  return false;
}

// For a connected-component that is a line, check if a pixel is at one end of the line
//
// WORKS FOR 4-conn
const bool isPixelAtEnd(const cv::Point& pixel, const std::vector<cv::Point>& component)
{
  if (component.size() == 2)
  {
    //std::cout << "    END pixel  (Total no. pixels = 2)" << std::endl;
    return true;
  }
  else if (component.size() == 1)
  {
    //std::cout << "    END pixel  (Total no. pixels = 1)" << std::endl;
    return true;
  }
  else if (component.size() == 0)
  {
    std::cout << "    ERROR: no pixels!" << std::endl;
    return true;
  }

  unsigned int num_neighbours = 0;
  for (size_t i = 0; i < component.size(); ++i)
  {
    if (is4Connected(pixel, component.at(i)))
    {
      num_neighbours++;
    }
  }
  //std::cout << "   No. neighbors = " << num_neighbours << std::endl;
  if (num_neighbours == 1)
  {
    //std::cout << "   END pixel  (Total no. pixels = " << component.size() << std::endl;
    return true;
  }
  return false;
}

// Get the index of the Vertex at some pixel position
//const int getVertexIndex(const cv::Point& pixel, const std::vector<Vertex>& vertices)
const int getVertexIndex(const int i, const int j, const std::vector<Vertex>& vertices)
{
  //if ((i == 489) && (j == 236)) std::cout << " TESTING" << std::endl;

  for (size_t idx = 0; idx < vertices.size(); ++idx)
  {
    //if (vertices.at(i).position == pixel)
    if ((vertices.at(idx).position.x == j) && (vertices.at(idx).position.y == i))
    {
      return idx;
    }
  }

  return -1; // Error
}

// for a pixel, check if it's 4-CONN neighbours are vertices or end-points
const std::vector<int> getNeighbouringVerticesAndEndpoints(const int i, const int j,
                                                           //const cv::Point& pixel,
                                                           const cv::Mat& vertices_image,
                                                           const cv::Mat& endpoints_image,
                                                           const cv::Mat& exterior_endpoints_image, // cv::Point(pixel.x, pixel.y)
                                                           const std::vector<Vertex>& vertices)
{
  // For debugging, this should never happen
  // disable this
  //const int max_y = vertices_image.rows - 1;
  //const int max_x = vertices_image.cols - 1;
  //if ((pixel.x < 0) || (pixel.x > max_x) || (pixel.y < 0) || (pixel.y > max_y))
  const int max_i = vertices_image.rows - 1;
  const int max_j = vertices_image.cols - 1;
  if ((i < 0) || (i > max_i) || (j < 0) || (j> max_j))
  {
    std::cout << "pixel indices are out of range in getNeighbouringVerticesAndEndpoints()" << std::endl;
  }

  std::vector<int> vertex_indices;

  /*
  std::vector<cv::Point> pixels_to_check;
  pixels_to_check.push_back(cv::Point(j+1,i+0));
  pixels_to_check.push_back(cv::Point(j-1,i+0));
  pixels_to_check.push_back(cv::Point(j+0,i+1));
  pixels_to_check.push_back(cv::Point(j+0,i-1));
  */
  std::vector<Pixel> pixels_to_check;
  pixels_to_check.push_back(Pixel(i+0,j+1));
  pixels_to_check.push_back(Pixel(i+0,j-1));
  pixels_to_check.push_back(Pixel(i+1,j+0));
  pixels_to_check.push_back(Pixel(i-1,j+0));

  for (size_t k = 0; k < pixels_to_check.size(); ++k)
  {
    /*
    cv::Point& pixel = pixels_to_check.at(k);
    if ((pixel.x < 0) || (pixel.x > max_x) || (pixel.y < 0) || (pixel.y > max_y))
    {
      continue;
    }
    */
    Pixel& pixel = pixels_to_check.at(k);

    int vertex_index = -2;

    if (vertices_image.at<uchar>(pixel.i, pixel.j) == 255)
    {
      //std::cout << "        vertex" << std::endl;
      vertex_index = getVertexIndex(pixel.i, pixel.j, vertices);
      vertex_indices.push_back(vertex_index);
    }
    else if (endpoints_image.at<uchar>(pixel.i, pixel.j) == 255)
    {
      //std::cout << "        end-point" << std::endl;
      vertex_index = getVertexIndex(pixel.i, pixel.j, vertices);
      vertex_indices.push_back(vertex_index);
    }
    else if (exterior_endpoints_image.at<uchar>(pixel.i, pixel.j) == 255)
    {
      //std::cout << "        exterior end-point" << std::endl;
      vertex_index = getVertexIndex(pixel.i, pixel.j, vertices);
      vertex_indices.push_back(vertex_index);
    }

    if (vertex_index == -1)
    {
      // Could be -1 if index not found
      std::cout << "ERROR: Failed to get Vertex index in getNeighbouringVerticesAndEndpoints(), "
                << vertex_index
                << "   i,j: " << i << "," << j << "   pixel.i,pixel.j: " << pixel.i << "," << pixel.j
                << std::endl;
    }
  }

  return vertex_indices;
}





/*
// Check if a pixel is equal to a specified value.
//
// (i,j) can be negative.
// This is a "safe check", if (i,j) is out of bounds then return False.
// Works with 8UC1.
const bool checkPixelEqual(const int i, const int j, const cv::Mat& image, const uchar value)
{
  if ((i < 0) || (i > image.rows-1) || (j < 0) || (j > image.cols-1))
  {
    // Index is outside of matrix range
    return false;
  }

  if (image.at<uchar>(i,j) == value)
  {
    return true;
  }

  return false;
}

// Check if a pixel is not equal to a specified value.
const bool checkPixelNotEqual(const int i, const int j, const cv::Mat& image, const uchar value)
{
  return !(checkPixelEqual(i, j, image, value));
}
*/

// Return the number of pixels which are 4-connected to the specific pixel.
const uint getNumNeighbours4Connected(const cv::Mat& image, const int i, const int j)
{
  uint num_neighbours = 0;
  if (image.at<uchar>(i+0,j+1) != 0) num_neighbours++;
  if (image.at<uchar>(i+0,j-1) != 0) num_neighbours++;
  if (image.at<uchar>(i+1,j+0) != 0) num_neighbours++;
  if (image.at<uchar>(i-1,j+0) != 0) num_neighbours++;
  return num_neighbours;
}
// Return the number of pixels which are 4-connected to the specific pixel.
// This is "safe", in that in checks if the indices are within the range of the image.
const uint getNumNeighbours4ConnectedSafe(const cv::Mat& image, const int i, const int j)
{
  uint num_neighbours = 0;

  const int max_y = image.rows - 1;
  const int max_x = image.cols - 1;

  std::vector<cv::Point> pixels_to_check;
  pixels_to_check.push_back(cv::Point(j+1,i+0));
  pixels_to_check.push_back(cv::Point(j-1,i+0));
  pixels_to_check.push_back(cv::Point(j+0,i+1));
  pixels_to_check.push_back(cv::Point(j+0,i-1));

  for (size_t k = 0; k < pixels_to_check.size(); ++k)
  {
    cv::Point& pixel = pixels_to_check.at(k);
    if ((pixel.x < 0) || (pixel.x > max_x) || (pixel.y < 0) || (pixel.y > max_y))
    {
      continue;
    }

    if (image.at<uchar>(pixel.y, pixel.x) != 0)
    {
      num_neighbours++;
    }
  }

  return num_neighbours;
}

//
// Vertices are drawn as green pixels on top of the skeleton.
//
// This expects out image is 3 channel.




// Find end-points of skeleton, by counting number of neighbors.
//
// Output is 8UC1 where 255 is an end-point.
// The outer border of pixels are not checked.
void findEndPoints(const cv::Mat& in_image, const uint connectivity, cv::Mat& endpoints_image, cv::Mat& exterior_endpoints_image)
{
  //if ((connectivity != 4) && (connectivity != 8))
  //{
    //throw
  //}

  //out_image = cv::Mat(in_image.size(), CV_8UC3);
  //cv::cvtColor(in_image, out_image, CV_GRAY2RGB);

  /*
  struct RGB
  {
    uchar blue;
    uchar green;
    uchar red;
  };
  */

  // For debugging
  int num_endpoints = 0;
  int num_exterior_endpoints = 0;

  const int max_i = in_image.rows - 1;
  const int max_j = in_image.cols - 1;

  unsigned int num_4_conn_px = 0;
  unsigned int num_8_conn_px = 0;

  for (int i = 0; i < in_image.rows; ++i)
  {
    const uchar* row_ptr = in_image.ptr<uchar>(i);
    for (int j = 0; j < in_image.cols; ++j)
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
        if ((i == 0) || (i == max_i) || (j == 0) || (j == max_j))
        {
          // A pixel around the border of the image
          num_4_conn_px = getNumNeighbours4ConnectedSafe(in_image, i, j);
          //std::cout << "  i,j " << i << "," << j << "  = " << num_4_conn_px << std::endl;
    
          if (num_4_conn_px == 1)
          {
            // An exterior end-point (on the border of the image)
            exterior_endpoints_image.at<uchar>(i,j) = 255;

            num_exterior_endpoints++;
          }
        }
        else
        {
          // Not a border pixel
          num_4_conn_px = getNumNeighbours4Connected(in_image, i, j);

          if (num_4_conn_px == 1)
          {
            // Normal (interior) end-point
            endpoints_image.at<uchar>(i,j) = 255;

            num_endpoints++;
          }
        }


        /*
        if (checkPixelNotEqual(i+0, j+1, in_image, 0)) num_4_conn_px++; // check that pixel is not equal to 0
        if (checkPixelNotEqual(i+0, j-1, in_image, 0)) num_4_conn_px++;
        if (checkPixelNotEqual(i+1, j+0, in_image, 0)) num_4_conn_px++;
        if (checkPixelNotEqual(i-1, j+0, in_image, 0)) num_4_conn_px++;

        if (i == 0)
        {
          std::cout << "   index " << i << "," << j << "  num_4_conn_px = " << num_4_conn_px << std::endl;
        }

        if (num_4_conn_px == 1)
        {
          //std::cout << "  Found endpoint at index " << i << "," << j << std::endl;

          if ((i == 0) || (i == max_i) || (j == 0) || (j == max_j))
          {
            // An exterior end-point (on the border of the image)
            exterior_endpoints_image.at<uchar>(i,j) = 255;

            num_exterior_endpoints++;
          }
          else
          {

            // Normal (interior) end-point
            //endpoints_image.at<uchar>(i,j) = 255;

            num_endpoints++;
          }
          
        }
        */
      }

      if (connectivity == 8)
      {
        std::cout << "\n\n 8-conn not handled in findEndPoints \n" << std::endl;
      }
      /*
      //if (connectivity == 8)
      {
        if (in_image.at<uchar>(i+1,j+1) != 0) num_8_conn_px++;
        if (in_image.at<uchar>(i+1,j-1) != 0) num_8_conn_px++;
        if (in_image.at<uchar>(i-1,j+1) != 0) num_8_conn_px++;
        if (in_image.at<uchar>(i-1,j-1) != 0) num_8_conn_px++;
      }


      if ((num_4_conn_px + num_8_conn_px) == 1)
      {
        out_image.at<uchar>(i,j) = 255;
        
        RGB& pixel = out_image.ptr<RGB>(i)[j]; // scanline y, pixel x
        pixel.red = 50;
        pixel.green = 200;
        pixel.blue = 50;
        
      }
      */
    }
  }

  std::cout << "  Found " << num_endpoints << " end-points, + " << num_exterior_endpoints << " exterior end-points." << std::endl;
}


// TODO: Future input will be black&white skel_image
void getGraphFromSkelImage(const cv::Mat& in_image, const uint connectivity, const std::string output_file_prefix)
{
  if ((connectivity != 4) && (connectivity != 8))
  {
    //throw
  }

  // TODO: Add findVertices()


  struct RGB
  {
    uchar blue;
    uchar green;
    uchar red;
  };




  // TEMP:
  // Extract white & red pixels into skel image,
  // and red pixels into vertices image:
  cv::Mat skel_image = cv::Mat::zeros(in_image.size(), CV_8UC1);
  //cv::Mat vertices_image = cv::Mat::zeros(in_image.size(), CV_8UC3);
  cv::Mat vertices_image = cv::Mat::zeros(in_image.size(), CV_8UC1);
  for (int i = 0; i < in_image.rows; ++i)
  {
    //const uchar* row_ptr = image.ptr<uchar>(i);
    for (int j = 0; j < in_image.cols; ++j)
    {
      const RGB& pixel = in_image.ptr<RGB>(i)[j]; // scanline y, pixel x

      if ((pixel.red == 255) && (pixel.green == 0) && (pixel.blue == 0)) // red
      {
        skel_image.at<uchar>(cv::Point(j,i)) = 255;

        vertices_image.at<uchar>(cv::Point(j,i)) = 255;
        //RGB& vertices_pixel = vertices_image.ptr<RGB>(i)[j];
        //vertices_pixel.red = 255;
        //vertices_pixel.green = 0;
        //vertices_pixel.blue = 0;
      }
      else if ((pixel.red == 255) && (pixel.green == 255) && (pixel.blue == 255)) // white
      {
        skel_image.at<uchar>(cv::Point(j,i)) = 255;
      }
    }
  }


  // Find the interior and exterior end-points
  cv::Mat endpoints_image = cv::Mat::zeros(skel_image.size(), CV_8UC1);
  cv::Mat exterior_endpoints_image = cv::Mat::zeros(skel_image.size(), CV_8UC1);
  findEndPoints(skel_image, connectivity, endpoints_image, exterior_endpoints_image);
  //cv::Mat exterior_endpoints_image = cv::Mat::zeros(skel_image.size(), CV_8UC1);
  //findExteriorEndPoints(skel_image, connectivity, exterior_endpoints_image);




  // Start a list of vertices:
  std::vector<Vertex> vertices;
  std::vector<Edge> edges;



  // Create copy of the binary skeleton but with the vertex pixels removed.
  // If 8-connected, then also remove more pixels surrounding the vertex.

  // Dis-connect vertices:
  // Remove vertice pixels and a 1 pixel border around each vertex:
  cv::Mat temp_image = skel_image.clone();
  cv::Mat expanded_pixels = cv::Mat::zeros(vertices_image.size(), CV_8UC1); // store the removed pixels

  for (int i = 0; i < vertices_image.rows; ++i)
  {
    //const uchar* row_ptr = vertices_image.ptr<uchar>(i);
    for (int j = 0; j < vertices_image.cols; ++j)
    {
      //if ((vertices_image.at<uchar>(i,j) == 255) || (endpoints_image.at<uchar>(i,j) == 255) || (exterior_endpoints_image.at<uchar>(i,j) == 255))

      if (vertices_image.at<uchar>(i,j) == 255)
      {
        // Remove vertex pixel in output image
        temp_image.at<uchar>(cv::Point(j+0,i+0)) = 0;

        Vertex vertex(i, j, VERTEX);
        vertices.push_back(vertex);
      }
      else if (endpoints_image.at<uchar>(i,j) == 255)
      {
        // Remove end-point pixel in output image
        temp_image.at<uchar>(cv::Point(j+0,i+0)) = 0;

        Vertex vertex(i, j, ENDPOINT);
        vertices.push_back(vertex);
      }
      else if (exterior_endpoints_image.at<uchar>(i,j) == 255)
      {
        // Remove exterior end-point pixel in output image
        temp_image.at<uchar>(cv::Point(j+0,i+0)) = 0;

        Vertex vertex(i, j, EXTERIOR_ENDPOINT);
        vertices.push_back(vertex);
      }




        


        /*
        // todo, this may not be safe for end-points
        if (connectivity == 8)
        {
          // Remove 3x3 block of pixels in output image
          // 4-connectivity
          temp_image.at<uchar>(cv::Point(j-1,i+0)) = 0;
          temp_image.at<uchar>(cv::Point(j+1,i+0)) = 0;
          temp_image.at<uchar>(cv::Point(j+0,i-1)) = 0;
          temp_image.at<uchar>(cv::Point(j+0,i+1)) = 0;
          // 8-connectivity
          //temp_image.at<uchar>(cv::Point(j-1,i-1)) = 0;
          //temp_image.at<uchar>(cv::Point(j-1,i+1)) = 0;
          //temp_image.at<uchar>(cv::Point(j+1,i-1)) = 0;
          //temp_image.at<uchar>(cv::Point(j+1,i+1)) = 0;

          // Keep track of the expanded pixels
          // 4-connectivity
          expanded_pixels.at<uchar>(cv::Point(j-1,i+0)) = 255;
          expanded_pixels.at<uchar>(cv::Point(j+1,i+0)) = 255;
          expanded_pixels.at<uchar>(cv::Point(j+0,i-1)) = 255;
          expanded_pixels.at<uchar>(cv::Point(j+0,i+1)) = 255;
          // 8-connectivity
          //expanded_pixels.at<uchar>(cv::Point(j-1,i-1)) = 255;
          //expanded_pixels.at<uchar>(cv::Point(j-1,i+1)) = 255;
          //expanded_pixels.at<uchar>(cv::Point(j+1,i-1)) = 255;
          //expanded_pixels.at<uchar>(cv::Point(j+1,i+1)) = 255;

          // Add to list of vertices
          std::vector<cv::Point> connected_pixels_4; // 4-connected
          connected_pixels_4.push_back(cv::Point(j-1,i+0));
          connected_pixels_4.push_back(cv::Point(j+1,i+0));
          connected_pixels_4.push_back(cv::Point(j+0,i-1));
          connected_pixels_4.push_back(cv::Point(j+0,i+1));
          //Vertex vertex(cv::Point(j,i), connected_pixels_4);
          //vertices.push_back(vertex);
        }
        */

      
    }
  }
  //cv::imshow("temp_image", temp_image);
  //cv::waitKey(0);
  //cv::imshow("expanded_pixels", expanded_pixels);
  //cv::waitKey(0);



  // Parameters for writing .png image using libPNG
  const std::vector<int> png_params = {CV_IMWRITE_PNG_COMPRESSION, 0};

  std::string outfile;





  // Find the connected-components
  // (each component will become an edge)
  //
  //cv::Mat ccomponent_labels = cv::Mat::zeros(temp_image.size(), CV_8UC1); // 8uc1 doesnt work
  cv::Mat ccomponent_labels = cv::Mat::zeros(temp_image.size(), CV_32S);
  int num_labels = findConnectedComponents(temp_image, connectivity, ccomponent_labels);
  // Note: range is 1 to (num_labels - 1)
  std::cout << "  Found " << (num_labels - 1) << " " << connectivity << "-connected components." << std::endl;
  double min;
  double max;
  cv::minMaxLoc(ccomponent_labels, &min, &max);
  std::cout << "  but max label ID = " << max << std::endl;
  std::vector<std::vector<cv::Point> > ccomponents;
  ccomponents.resize(num_labels - 1);
  std::cout << "init vector of cc" << std::endl;
  for (int i = 0; i < ccomponent_labels.rows; ++i)
  {
    const int* row_ptr = ccomponent_labels.ptr<int>(i);
    for (int j = 0; j < ccomponent_labels.cols; ++j)
    {
      if (row_ptr[j] != 0)
      {
        const int label = row_ptr[j] - 1; // label values start at 1
        //std::cout << "  pixel: " << i << "," << j << "  label: " << label << std::endl;
        ccomponents.at(label).push_back(cv::Point(j,i));
      }
    }
  }


  // Build the graph structure
  for (size_t i = 0; i < ccomponents.size(); ++i)
  {
    //std::cout << "  Component: " << i << "  size = " << ccomponents.at(i).size() << std::endl;

    std::vector<int> component_vertex_indices;
    for (size_t j = 0; j < ccomponents.at(i).size(); ++j)
    {
      cv::Point& pixel = ccomponents.at(i).at(j);

      // Check if this is an end pixel    CURRETLY 4-CONN ONLY
      bool end_pixel = isPixelAtEnd(cv::Point(pixel.x, pixel.y), ccomponents.at(i));
      if (end_pixel)
      {
        //std::cout << "    " << j << " = END " << std::endl;

        // this works for 4-CONN
        // Get any connected vertices or end-points
        std::vector<int> pixel_verts = getNeighbouringVerticesAndEndpoints(pixel.y, pixel.x,
                                                                        vertices_image,
                                                                        endpoints_image,
                                                                        exterior_endpoints_image,
                                                                        vertices); // cv::Point(pixel.x, pixel.y)
        //printStdVector(pixel_verts);
        // Append this pixel's vertices to our list.
        // This is because a 1 pixel long component will have 2 vertices,
        // where-as each end of a longer component will each have 1 vertex.
        for (int k = 0; k < pixel_verts.size(); ++k)
        {
          component_vertex_indices.push_back(pixel_verts.at(k));
        }
        /*
        // Foreach vertex, add the edge index as an Edge.
        // The edge index is the the connected-component label ID.
        for (int k = 0; k < component_verts.size(); ++k)
        {
          const int vert_idx = component_verts.at(k);
          const int edge_idx = i;
          std::cout << "  Adding Edge idx " << edge_idx << " to Vertex idx " << vert_idx << std::endl;
          vertices.at(vert_idx).edges.push_back(edge_idx);
        }
        */
      }
    }
    // This should never happen
    if (component_vertex_indices.size() != 2)
    {
      std::cout << "ERROR: connected-component idx " << i << " has " << component_vertex_indices.size() << " vertex indices" << std::endl;
    }

    // Foreach vertex, add the edge index as an Edge.
    // The edge index is the the connected-component label ID.
    for (int j = 0; j < component_vertex_indices.size(); ++j)
    {
      const int vert_idx = component_vertex_indices.at(j);
      const int edge_idx = i;
      //std::cout << "    Adding Edge idx " << edge_idx << " to Vertex idx " << vert_idx << std::endl;
      vertices.at(vert_idx).edges.push_back(edge_idx);
    }
    // TODO: Make an  addEdge() function which checks if an edge already exists
    Edge edge(component_vertex_indices.at(0), component_vertex_indices.at(1));
    edges.push_back(edge);
  }
  //
  // TODO: ITERATE OVER VERTICES, ADD ANY ZERO LENGTH EDGES
  //

  // debug, print vertices
  //for (size_t i = 0; i < vertices.size(); ++i)
  //{
  //  std::cout << "   Vertex pos: " << vertices.at(i).position << "  i,j: " << vertices.at(i).position.y << "," << vertices.at(i).position.x << std::endl;
  //}

cv::RNG rng(12345);  // for color codes



  cv::Mat graph_image = cv::Mat::zeros(vertices_image.size(), CV_8UC3); // empty image
  for (size_t i = 0; i < edges.size(); ++i)
  {
    const int v0_idx = edges.at(i).v0;
    const int v1_idx = edges.at(i).v1;
    // ToDo: make getVertexPosition() function
    cv::Point& v0 = vertices.at(v0_idx).position;
    cv::Point& v1 = vertices.at(v1_idx).position;

    std::cout << "  Edge from " << v0 << " to " << v1 << std::endl;
    //cv::Vec3b color((rand()&255), (rand()&255), (rand()&255));

    // Draw each contour in shades of blue
    //cv::Scalar color = cv::Scalar(rng.uniform(0, 255), 0, 0);
    // random colors
    cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));


    const int thickness = 1;
    const int line_type = 4;
    cv::line(graph_image, v0, v1, color, thickness, line_type);

  }
//C++: void line(Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
  outfile = output_file_prefix + std::string("_graph.png");
  cv::imwrite(outfile, graph_image, png_params);


  // Optional, write out image with colorized components
  //
  // Note: The ccomponent_image has the end-points removed, as required by the algorithm above.
  //       Therefore if you don't draw the end-point pixels, components may be 1 pixel shorter.
  {
    cv::Vec3b ccomponent_colors[num_labels];
    ccomponent_colors[0] = cv::Vec3b(0, 0, 0); // background
    for (int label = 1; label < num_labels; ++label)
    {
      ccomponent_colors[label] = cv::Vec3b((rand()&255), (rand()&255), (rand()&255));
    }
    cv::Mat ccomponent_image(temp_image.size(), CV_8UC3);
    for(int r = 0; r < ccomponent_image.rows; ++r)
    {
      for(int c = 0; c < ccomponent_image.cols; ++c)
      {
        int label = ccomponent_labels.at<int>(r, c);
        //int label = ccomponent_labels.at<uchar>(r, c);
        //std::cout << "r,c " << r << ", " << c << std::endl;
        cv::Vec3b& pixel = ccomponent_image.at<cv::Vec3b>(r, c);
        pixel = ccomponent_colors[label];
      }
    }

    // Draw the original vertex pixels in white, over the top
    // of the colored components
    for (int i = 1; i < vertices_image.rows-1; ++i)
    {
      for (int j = 1; j < vertices_image.cols-1; ++j)
      {
        //RGB& pixel = vertices_image.ptr<RGB>(i)[j]; // scanline y, pixel x
        //if ((pixel.red == 255) && (pixel.green == 0) && (pixel.blue == 0)) // red
        if (vertices_image.at<uchar>(i,j) == 255)
        {
          RGB& out_pixel = ccomponent_image.ptr<RGB>(i)[j]; // scanline y, pixel x
          out_pixel.red = 255;
          out_pixel.green = 255;
          out_pixel.blue = 255;
        }
      }
    }
    // and the white end-point pixels
    for (int i = 1; i < endpoints_image.rows-1; ++i)
    {
      for (int j = 1; j < endpoints_image.cols-1; ++j)
      {
        if (endpoints_image.at<uchar>(i,j) == 255)
        {
          RGB& out_pixel = ccomponent_image.ptr<RGB>(i)[j]; // scanline y, pixel x
          out_pixel.red = 255;
          out_pixel.green = 255;
          out_pixel.blue = 255;
        }
      }
    }
    // todo: there might be a faster way to index the border pixels, without checking interior pixels
    for (int i = 0; i < exterior_endpoints_image.rows; ++i)
    {
      for (int j = 0; j < exterior_endpoints_image.cols; ++j)
      {
        if (exterior_endpoints_image.at<uchar>(i,j) == 255)
        {
          RGB& out_pixel = ccomponent_image.ptr<RGB>(i)[j]; // scanline y, pixel x
          out_pixel.red = 255;
          out_pixel.green = 255;
          out_pixel.blue = 255;
        }
      }
    }

    outfile = output_file_prefix + std::string("_components0_Wu.png");
    cv::imwrite(outfile, ccomponent_image, png_params);
  }


  // TODO:  if 8-conn, expand the components back towards the vertices







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

  const std::vector<std::string> output_filename_tokens = splitString(output_filename);
  std::string outfile;


/*
struct RGB
{
  uchar blue;
  uchar green;
  uchar red;
};


// Parameters for writing .png image using libPNG
const std::vector<int> png_params = {CV_IMWRITE_PNG_COMPRESSION, 0};
*/

  // Load image into 8UC3 matrix
  cv::Mat image = cv::imread(input_filename); // default is CV_LOAD_IMAGE_COLOR
  if (!image.data)
  {
    std::cout <<  "Could not open or find the image" << std::endl;
    return -1;
  }
  //printCvMatType(image);




  // input is image where white=skel, red=vertex
  getGraphFromSkelImage(image, connectivity, output_filename_tokens[0]);






/*

// Find 8-connected components
// Note: the pixels in each component are not in a certain order!
//
// Use findContours() from OpenCV 2.4
// This expects 8uc1
std::vector<std::vector<cv::Point> > contours;
std::vector<cv::Vec4i> hierarchy;
cv::findContours(temp_image, // skel_image with vertices removed
                 contours, hierarchy,
                 CV_RETR_LIST, // A flat list of contours, no hierachy
                 CV_CHAIN_APPROX_NONE);  // Use exact pixels
std::cout << "Found " << contours.size() << " contours" << std::endl;
//cv::Mat components_image = cv::Mat(skel_image.size(), CV_8UC3);
//cv::cvtColor(temp_image, components_image, CV_GRAY2RGB);
cv::Mat components_image = cv::Mat::zeros(vertices_image.size(), CV_8UC3); // empty image
cv::RNG rng(12345);  // for color codes
std::vector<cv::Scalar> colors; // store the colors used
for (size_t i = 0; i < contours.size(); i++)
//for (size_t i = 465; i < 468; i++)
{
  // Draw each contour in shades of blue
  //cv::Scalar color = cv::Scalar(rng.uniform(0, 255), 0, 0);
  // random colors
  cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
  cv::drawContours(components_image, contours, i, color,
                   1, // line_thickness=1
                   8, // 8-connected
                   hierarchy, 0, cv::Point());
  colors.push_back(color);
  
  std::cout << "  Component " << i
            << "  1st pixel: " << contours.at(i).at(0).x << "," << contours.at(i).at(0).y
            << "  Last pixel: " << contours.at(i).at(contours.at(i).size()-1).x << "," << contours.at(i).at(contours.at(i).size()-1).y
            << std::endl;

  std::cout << "  Component " << i << std::endl;
  for (size_t j = 0; j < contours.at(i).size(); ++j)
  {
    std::cout << "  " << contours.at(i).at(j).x << "," << contours.at(i).at(j).y;
  }
  std::cout << std::endl;
  
}
//cv::imshow("contours", components_image);
//cv::waitKey(0);
outfile = output_filename_tokens[0] + std::string("_components1.png");
cv::imwrite(outfile, components_image, png_params);





// Expand the end of each component upto 1 pixel back towards its nearest vertex.
// (this is just for visualization)
// Also find the connectivity between vertices and components (edge)
std::vector<std::vector<cv::Point> > new_contours;
//for (size_t i = 0; i < contours.size(); ++i)
//for (size_t i = 0; i < 30; i++)
for (size_t i = 115; i < 117; i++)
{
  //std::cout << "  Component " << i << std::endl;
  //std::cout << "     size = " << contours.at(i).size() << std::endl;
  // "  1st pixel: " << contours.at(i).at(0).x << "," << contours.at(i).at(0).y

  // Make sure there's no duplicate pixels - findContours() will sometimes add a pixel twice!
  std::vector<cv::Point> unique_contour(contours.at(i));
  remove_duplicates(unique_contour);


  std::cout << "  Component " << i << "     size = " << unique_contour.size() << "    (was " << contours.at(i).size() << std::endl;

  std::vector<cv::Point> updated_contour; // The updated contour (vector of pixels)

  //std::vector<cv::Point> extended_pixels;
  unsigned int num_pixels_extended = 0; // TODO: JUST GET SIZE OF VECTOR INSTEAD
  unsigned int num_end_pixels = 0;



  // Check each pixel, because the pixels in each component are not in a specific order
  //for (size_t j = 0; j < contours.at(i).size(); ++j)
  for (size_t j = 0; j < unique_contour.size(); ++j)
  {
    
    //cv::Point& pixel = contours.at(i).at(j);
    cv::Point& pixel = unique_contour.at(j);
    const int x = pixel.x; // col
    const int y = pixel.y; // row

    updated_contour.push_back(cv::Point(x,y));

    //std::cout << j << "     pixel: " << x << "," << y << std::endl;

    // First, check if its an end pixel

    // Check if this is an end pixel    CURRETLY 4-CONN ONLY
    //bool res = isPixelAtEnd(cv::Point(x,y), contours.at(i));
    bool res = isPixelAtEnd(cv::Point(x,y), unique_contour);
    //if (res) std::cout << "  END " << std::endl;
    if (res) std::cout << "    " << j << " = END " << std::endl;

    if (res)
    {
      num_end_pixels++;
    }

    if (!res)
    {
      // not an end pixel
      continue;
    }

    // Check if the pixel was previously removed AND
    // was on the original voronoi diagram
    std::vector<cv::Point> pixels_to_check;
    pixels_to_check.push_back(cv::Point(x+1,y+0));
    pixels_to_check.push_back(cv::Point(x-1,y+0));
    pixels_to_check.push_back(cv::Point(x+0,y+1));
    pixels_to_check.push_back(cv::Point(x+0,y-1));
    for (size_t k = 0; k < pixels_to_check.size(); ++k)
    {
      if (expanded_pixels.at<uchar>(pixels_to_check.at(k).y, pixels_to_check.at(k).x) == 255)
      {
        //std::cout << "      j: " << j << "  k: " << k << "  was extended" << std::endl;

        if (skel_image.at<uchar>(pixels_to_check.at(k).y, pixels_to_check.at(k).x) == 255)
        {
          //std::cout << "         j: " << j << "  k: " << k << "  was extended AND on skel" << std::endl;

          // Add pixel to the current contour
          updated_contour.push_back(pixels_to_check.at(k));
          num_pixels_extended++;

          // If this contour is at least 2 pixels long, then we should only
          // try to extend each end pixel once:
          if (unique_contour.size() > 1)
          {
            goto done_checking;
          }
        }
      }
    }
    done_checking: ;


    

    // TODO: 8 conn.
  
  }

  new_contours.push_back(updated_contour);
  std::cout << "  Component " << i << "  extended " << num_pixels_extended << " pixels, size is now " << updated_contour.size() << std::endl;

  // TODO: ADD THIS DEBUGGING
  if (num_end_pixels > 2)
  {
    std::cout << "WARNING: More than 2 end pixels!" << std::endl;
  }
  //if ((unique_contour.size() == 1) && (num_end_pixels > 2))
  //else if ((unique_contour.size() == 2) && (num_end_pixels > 2))
  //else if ((unique_contour.size() == 2) && (num_end_pixels > 2))

    
  
  //std::cout << std::endl;

}
std::cout << "new_contours size = " << new_contours.size() << std::endl;


std::cout << "Graph:" << std::endl;
for (size_t i = 0; i < vertices.size(); ++i)
{
  std::cout << "Vertex " << i << "  edges: ";
  for (size_t j = 0; j < vertices.at(i).edges.size(); ++j)
  {
    std::cout << vertices.at(i).edges.at(j) << ", ";
  }
  std::cout << std::endl;
}



// Draw the new, re-extended components.
//
// we Don't use cv::drawContours() because it interpolates extra pixels.
components_image = cv::Mat::zeros(skel_image.size(), CV_8UC3);
//cv::cvtColor(temp_image, components_image, CV_GRAY2RGB);
//cv::RNG rng(12345);  // for color codes
for (size_t i = 0; i < new_contours.size(); ++i)
{
  for (size_t j = 0; j < new_contours.at(i).size(); ++j)
  {
    cv::Point& pixel = new_contours.at(i).at(j);
    RGB& pixel_value = components_image.ptr<RGB>(pixel.y)[pixel.x];

    // Set pixel color to be same as color used previously.
    // BGR ordering.
    pixel_value.blue = colors.at(i)[0];
    pixel_value.green = colors.at(i)[1];
    pixel_value.red = colors.at(i)[2];

    //components_image.at(0,0) = colors.at(i); // use same color as previously



  }

  // random colors
  //cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));

  //cv::drawContours(components_image, new_contours, i,
                   //color,
                   //colors.at(i)); //, // use same color as previously
                   //1, // line_thickness=1
                   //8, // 8-connected
                   //hierarchy, 0, cv::Point());
}




// Draw the original vertex pixels in white, over the top
// of the colored components
for (int i = 1; i < vertices_image.rows-1; ++i)
{
  //const uchar* row_ptr = vertices_image.ptr<uchar>(i);
  for (int j = 1; j < vertices_image.cols-1; ++j)
  {
    RGB& pixel = vertices_image.ptr<RGB>(i)[j]; // scanline y, pixel x
    if ((pixel.red == 255) && (pixel.green == 0) && (pixel.blue == 0)) // red
    {
      RGB& out_pixel = components_image.ptr<RGB>(i)[j]; // scanline y, pixel x
      out_pixel.red = 255;
      out_pixel.green = 255;
      out_pixel.blue = 255;
    }



  }
}

//cv::imshow("new_contours", components_image);
//cv::waitKey(0);
outfile = output_filename_tokens[0] + std::string("_components2.png");
cv::imwrite(outfile, components_image, png_params);
*/

  return 0;
}

