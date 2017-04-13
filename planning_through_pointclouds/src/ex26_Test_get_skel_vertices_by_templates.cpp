/*
Find vertices by scanning templates

METHOD 2 and 3, 
DOESNT REQUIRE PRE-THINNING:

NOTE: For results of method 3, need to set connectivity = 4 or 8 in here:

ex26 /home/dbworth/skeleton/MyMap1_cropped_DynVoronoi.png /home/dbworth/skeleton/MyMap1_cropped_DynVoronoi_vertices_method2.png
ex26 /home/dbworth/skeleton/MyMap1_Zhang.png /home/dbworth/skeleton/MyMap1_Zhang_vertices_method2.png


ex26 /home/dbworth/skeleton/MyMap2_DynVoronoi.png /home/dbworth/skeleton/MyMap2_DynVoronoi_vertices_method2.png
ex26 /home/dbworth/skeleton/MyMap2_Zhang.png /home/dbworth/skeleton/MyMap2_Zhang_vertices_method2.png


ex26 /home/dbworth/skeleton/beeson1-DynVoronoi.png /home/dbworth/skeleton/beeson1-DynVoronoi_vertices_method2.png
ex26 /home/dbworth/skeleton/beeson1-Zhang.png /home/dbworth/skeleton/beeson1-Zhang_vertices_method2.png




ex26 /home/dbworth/skeleton/FR079_DynVoronoi.png /home/dbworth/skeleton/FR079_DynVoronoi_vertices_method2.png
ex26 /home/dbworth/skeleton/FR079_Zhang.png /home/dbworth/skeleton/FR079_Zhang_vertices_method2.png




ex26 /home/dbworth/skeleton/FR101_DynVoronoi.png /home/dbworth/skeleton/FR101_DynVoronoi_vertices_method2.png
ex26 /home/dbworth/skeleton/FR101_Zhang.png /home/dbworth/skeleton/FR101_Zhang_vertices_method2.png



ex26 /home/dbworth/skeleton/Horse_Zhang.png  /home/dbworth/skeleton/horse_out_zhang.png

*/

#include <iostream> // cout
//#include <string>
//#include <vector>


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
      is_junction = matchBranchTemplate(in_image, i, j);

        if ((in_image.at<uchar>(i-1,j+0) != 0) &&
                 (in_image.at<uchar>(i+0,j-1) != 0) &&
                 (in_image.at<uchar>(i+0,j+1) != 0))
*/


// The output response to each of the 72 branch templates.
//
// A value from 0 to 8, being the index of the pixel in the 3x3 output that should be turned 'on'.
// [0,1,2]
// [3,4,5]
// [6,7,8]
//
// There can be at most 2 pixels which are turned on.
// -1 = do nothing
// Example:
// {4,-1} means that the response will be to make pixel index 4 (the center of the 3x3 region) to be 1.
const int branch_templates_output[72][2] = {
  {4,-1},
  {4,-1},
  {4,-1},
  {4,-1},

  {4,-1},
  {4,-1},

  {1,-1},
  {1,-1},
  {3,-1},
  {3,-1},
  {5,-1},
  {5,-1},
  {7,-1},
  {7,-1},

  {1,7},
  {3,5},

  {1,-1},
  {7,-1},
  {3,-1},
  {5,-1},

  {4,-1},
  {4,-1},
  {4,-1},
  {4,-1},

  {4,-1},
  {4,-1},
  {4,-1},
  {4,-1},

  {4,-1},
  {4,-1},
  {4,-1},
  {4,-1},

  {4,-1},
  {4,-1},
  {4,-1},
  {4,-1},
  {4,-1},
  {4,-1},
  {4,-1},
  {4,-1},

  {4,-1},
  {4,-1},
  {4,-1},
  {4,-1},
  {4,-1},
  {4,-1},
  {4,-1},
  {4,-1},

  {4,-1},
  {4,-1},
  {4,-1},
  {4,-1},
  {4,-1},
  {4,-1},
  {4,-1},
  {4,-1},

  {3,-1},
  {4,-1},
  {5,-1},
  {3,-1},
  {4,-1},
  {5,-1},
  {1,-1},
  {4,-1},
  {7,-1},
  {1,-1},
  {4,-1},
  {7,-1},

  {4,-1},
  {4,-1},
  {4,-1},
  {4,-1}
};

//const 
int branch_templates[72][9] = {
  {1,0,1,
   0,1,0,
   1,0,0},
  {1,0,1,
   0,1,0,
   0,0,1},
  {1,0,0,
   0,1,0,
   1,0,1},
  {0,0,1,
   0,1,0,
   1,0,1},


  {0,1,0,
   1,1,1,
   0,1,0},
  {1,0,1,
   0,1,0,
   1,0,1},


  {1,1,1,
   0,1,0,
   1,0,0},
  {1,1,1,
   0,1,0,
   0,0,1},
  {1,0,1,
   1,1,0,
   1,0,0},
  {1,0,0,
   1,1,0,
   1,0,1},
  {1,0,1,
   0,1,1,
   0,0,1},
  {0,0,1,
   0,1,1,
   1,0,1},
  {1,0,0,
   0,1,0,
   1,1,1},
  {0,0,1,
   0,1,0,
   1,1,1},


  {1,1,1,
   0,1,0,
   1,1,1},
  {1,0,1,
   1,1,0,
   1,0,1},


  {1,1,1,
   0,1,0,
   0,1,0}, // 17
  {0,1,0,
   0,1,0,
   1,1,1},
  {1,0,0,
   1,1,1,
   1,0,0},
  {0,0,1,
   1,1,1,
   0,0,1},


  {1,0,1,
   0,1,0,
   0,1,0},
  {0,1,0,
   0,1,0,
   1,0,1},
  {1,0,0,
   0,1,1,
   1,0,0},
  {0,0,1,
   1,1,0,
   0,0,1},


  {1,1,1,
   0,1,0,
   1,0,1},
  {1,0,1,
   0,1,0,
   1,1,1},
  {1,0,1,
   1,1,0,
   1,0,1},
  {1,0,1,
   0,1,1,
   1,0,1},


  {0,1,0,
   0,1,1,
   1,0,0},
  {0,1,0,
   1,1,0,
   0,0,1},
  {0,0,1,
   1,1,0,
   0,1,0},
  {1,0,0,
   0,1,1,
   0,1,0},


  {1,1,0,
   0,1,1,
   1,0,0},
  {0,1,1,
   1,1,0,
   0,0,1},
  {0,0,1,
   1,1,0,
   0,1,1},
  {1,0,0,
   0,1,1,
   1,1,0},
  {0,1,0,
   1,1,0,
   1,0,1},
  {1,0,1,
   1,1,0,
   0,1,0},
  {1,0,1,
   0,1,1,
   0,1,0},
  {0,1,0,
   0,1,1,
   1,0,1},


  {1,0,1,
   0,1,1,
   1,0,0},
  {1,0,0,
   0,1,1,
   1,0,1},
  {1,0,1,
   1,1,0,
   0,0,1},
  {0,0,1,
   1,1,0,
   1,0,1},
  {1,1,0,
   0,1,0,
   1,0,1},
  {0,1,1,
   0,1,0,
   1,0,1},
  {1,0,1,
   0,1,0,
   1,1,0},
  {1,0,1,
   0,1,0,
   0,1,1},


  {0,1,0,
   1,1,1,
   1,0,0},
  {0,1,0,
   1,1,1,
   0,0,1},
  {1,0,0,
   1,1,1,
   0,1,0},
  {0,0,1,
   1,1,1,
   0,1,0},
  {0,1,1,
   1,1,0,
   0,1,0},
  {0,1,0,
   1,1,0,
   0,1,1},
  {1,1,0,
   0,1,1,
   0,1,0},
  {0,1,0,
   0,1,1,
   1,1,0},


  {1,0,1,
   1,1,1,
   1,0,0},
  {1,0,1,
   1,1,1,
   0,1,0},
  {1,0,1,
   1,1,1,
   0,0,1},
  {1,0,0,
   1,1,1,
   1,0,1},
  {0,1,0,
   1,1,1,
   1,0,1},
  {0,0,1,
   1,1,1,
   1,0,1},
  {1,1,1,
   0,1,0,
   1,1,0},
  {1,1,0,
   0,1,1,
   1,1,0},
  {1,1,0,
   0,1,0,
   1,1,1},
  {1,1,1,
   0,1,0,
   0,1,1},
  {0,1,1,
   1,1,0,
   0,1,1},
  {0,1,1,
   0,1,0,
   1,1,1},


  {1,0,1,
   0,1,1,
   1,1,0},
  {1,0,1,
   1,1,0,
   0,1,1},
  {1,1,0,
   0,1,1,
   1,0,1},
  {0,0,1,
   1,1,0,
   1,0,1}
};


// Check if two cv::Mat are the same
const bool areCvMatEqual(const cv::Mat& a, const cv::Mat& b)
{
  if ((a.at<uchar>(0,0) == b.at<uchar>(0,0)) &&
      (a.at<uchar>(1,0) == b.at<uchar>(1,0)) &&
      (a.at<uchar>(2,0) == b.at<uchar>(2,0)) &&
      (a.at<uchar>(0,1) == b.at<uchar>(0,1)) &&
      (a.at<uchar>(1,1) == b.at<uchar>(1,1)) &&
      (a.at<uchar>(2,1) == b.at<uchar>(2,1)) &&
      (a.at<uchar>(0,2) == b.at<uchar>(0,2)) &&
      (a.at<uchar>(1,2) == b.at<uchar>(1,2)) &&
      (a.at<uchar>(2,2) == b.at<uchar>(2,2)))
  {
    return true;
  }
  return false;
  /*
  const cv::Mat diff = (a != b);
  // Equal if no elements disagree
  const bool equal = cv::countNonZero(diff) == 0;
  return equal;
  */
}


const uint num_templates = 72;
const uint num_template_responses = 2;
const uchar pixel_on = 255; // value of a pixel that is "1"

const int matchBranchTemplate(const cv::Mat& image, const uint row, const uint col) // y,x
{
  cv::Mat template3x3 = cv::Mat::zeros(3, 3, CV_8UC1);

  for (uint i = 0; i < num_templates; ++i)
  //for (uint i = 16; i < 17; ++i)
  {
    //std::cout << "Should be: " << branch_templates[i][0] << "," << branch_templates[i][1] << "," << branch_templates[i][2] << "," << std::endl;

    // Get 3x3 template, which has values of 0 or 255
    //cv::Mat template3x3 = cv::Mat(3, 3, CV_8UC1, (void *)branch_templates[i]).clone(); // Copy the data   DOESNT WORK, maybe due to array of arrays
    template3x3.at<uchar>(0,0) = branch_templates[i][0]; // at(row,col)
    template3x3.at<uchar>(0,1) = branch_templates[i][1];
    template3x3.at<uchar>(0,2) = branch_templates[i][2];
    template3x3.at<uchar>(1,0) = branch_templates[i][3];
    template3x3.at<uchar>(1,1) = branch_templates[i][4];
    template3x3.at<uchar>(1,2) = branch_templates[i][5];
    template3x3.at<uchar>(2,0) = branch_templates[i][6];
    template3x3.at<uchar>(2,1) = branch_templates[i][7];
    template3x3.at<uchar>(2,2) = branch_templates[i][8];
    template3x3 *= pixel_on; // 255
    //std::cout << "template3x3 = " << template3x3 << std::endl;

    //std::cout << "Image section = " << image(cv::Rect(col-1, row-1, 3, 3)) << std::endl;

    const bool equal = areCvMatEqual(template3x3, image(cv::Rect(col-1, row-1, 3, 3))); // x, y, width, height
    //std::cout << "  Result: " << equal << std::endl;

    if (equal)
    {
      //std::cout << "  Result: " << equal << std::endl;
      //return true;
      // Return the index of the matching template
      return i; 
    }
  }

  return -1; // no match found
}





// Find vertices of skeleton.  METHOD #2
//
// Vertices are drawn as red pixels on top of the skeleton.
// This works on 8UC1 image, pixel values are 0 or 255.
// The outer border of pixels are not checked.
void findVerticesUsingTemplates(const cv::Mat& in_image, cv::Mat& out_image)
//void findVerticesUsingTemplates(const cv::Mat& in_image, const uint connectivity, cv::Mat& out_image)
{
  // The outer border of pixels are not checked, because we use a 3x3 kernel.
  // TODO: PAD THE IMAGE

  out_image = cv::Mat(in_image.size(), CV_8UC3);
  cv::cvtColor(in_image, out_image, CV_GRAY2RGB);
  struct RGB
  {
    uchar blue;
    uchar green;
    uchar red;
  };
  bool is_junction = false;
  int template_idx = 0;
  //RGB& pixel = out_image.ptr<RGB>(0)[0]; // initialize
  RGB* pixel = 0;
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

      //is_junction = matchBranchTemplate(in_image, i, j);
      template_idx = matchBranchTemplate(in_image, i, j);

      //if (is_junction)
      if (template_idx < 0)
      {
        // -1 = no matching template
        continue;
      }

      // Matched a template
      for (int k = 0; k < num_template_responses; ++k)
      {

        const int output_pixel_idx = branch_templates_output[template_idx][k];

        //std::cout << "Matched template " << template_idx << "  response idx: " << output_pixel_idx << std::endl;


        if (output_pixel_idx == -1)
        {
          // Don't change an output pixel
          continue;
        }
        else if (output_pixel_idx == 4)
        {
          // Middle pixel of 3x3 block (the current location)
          pixel = &out_image.ptr<RGB>(i)[j]; // scanline y, pixel x
        }
        else if (output_pixel_idx == 1)
        {
          // One pixel ABOVE current location
          pixel = &out_image.ptr<RGB>(i-1)[j]; // scanline y, pixel x
        }
        else if (output_pixel_idx == 3)
        {
          // One pixel LEFT current location
          pixel = &out_image.ptr<RGB>(i)[j-1]; // scanline y, pixel x
        }
        else if (output_pixel_idx == 5)
        {
          // One pixel RIGHT current location
          pixel = &out_image.ptr<RGB>(i)[j+1]; // scanline y, pixel x
        }
        else if (output_pixel_idx == 7)
        {
          // One pixel BELOW current location
          pixel = &out_image.ptr<RGB>(i+1)[j]; // scanline y, pixel x
        }
        else
        {
          std::cout << "ERROR: un-expected output_pixel_idx = " << output_pixel_idx << std::endl;          
        }
        

        //RGB* 
        //pixel = &out_image.ptr<RGB>(i)[j];

        // Set output pixel to red
        pixel->red = 255;
        pixel->green = 0;
        pixel->blue = 0;
      }

      //const int branch_templates_output[72][2] = {
//  {4,-1},


    }
  }

}


// MEHOD 3  using 4 or 16 templates
void findVerticesUsingTemplates_v3(const cv::Mat& in_image, const uint connectivity, cv::Mat& out_image)
{
  // The outer border of pixels are not checked, because we use a 3x3 kernel.
  // TODO: PAD THE IMAGE

  out_image = cv::Mat(in_image.size(), CV_8UC3);
  cv::cvtColor(in_image, out_image, CV_GRAY2RGB);
  struct RGB
  {
    uchar blue;
    uchar green;
    uchar red;
  };
  bool is_junction = false;
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

      is_junction = false;

      // ToDo: delete +/- zero from below



      // 4-connected:
      if (connectivity == 4)
      {

        /*
        // Cross pattern, this is covered by another pattern below.
        if ((in_image.at<uchar>(i-1,j+0) != 0) &&
                 (in_image.at<uchar>(i+0,j-1) != 0) &&
                 (in_image.at<uchar>(i+0,j+1) != 0) &&
                 (in_image.at<uchar>(i+1,j+0) != 0))
        {
          // 4 branches, 4-connected
          is_junction = true;
          //std::cout << "Matched 5 at " << i << ", " << j << std::endl;
        }

        else 
        */
        if ((in_image.at<uchar>(i-1,j+0) != 0) &&
                 (in_image.at<uchar>(i+0,j-1) != 0) &&
                 (in_image.at<uchar>(i+0,j+1) != 0))
        {
          // 3 branches, 4-connected
          is_junction = true;
          //std::cout << "Matched 6 at " << i << ", " << j << std::endl;
        }
        else if ((in_image.at<uchar>(i+0,j-1) != 0) &&
                 (in_image.at<uchar>(i+0,j+1) != 0) &&
                 (in_image.at<uchar>(i+1,j+0) != 0))
        {
          // 3 branches, 4-connected
          is_junction = true;
          //std::cout << "Matched 7 at " << i << ", " << j << std::endl;
        }
        else if ((in_image.at<uchar>(i-1,j+0) != 0) &&
                 (in_image.at<uchar>(i+0,j+1) != 0) &&
                 (in_image.at<uchar>(i+1,j+0) != 0))
        {
          // 3 branches, 4-connected
          is_junction = true;
          //std::cout << "Matched 8 at " << i << ", " << j << std::endl;
        }
        else if ((in_image.at<uchar>(i-1,j+0) != 0) &&
                 (in_image.at<uchar>(i+0,j-1) != 0) &&
                 (in_image.at<uchar>(i+1,j+0) != 0))
        {
          // 3 branches, 4-connected
          is_junction = true;
          //std::cout << "Matched 9 at " << i << ", " << j << std::endl;
        }
      }


      // 8-connected

      else if (connectivity == 8) // && (is_junction == false))
      {

        // Four 4-connected patterns, like above
        if ((in_image.at<uchar>(i-1,j+0) != 0) &&
                 (in_image.at<uchar>(i+0,j-1) != 0) &&
                 (in_image.at<uchar>(i+0,j+1) != 0))// &&
                 //(in_image.at<uchar>(i+1,j+0) == 0))
        {
          // 3 branches, 4-connected
          is_junction = true;
          //std::cout << "Matched 6 at " << i << ", " << j << std::endl;
        }
        else if ((in_image.at<uchar>(i+0,j-1) != 0) &&
                 (in_image.at<uchar>(i+0,j+1) != 0) &&
                 (in_image.at<uchar>(i+1,j+0) != 0))// &&
                 //(in_image.at<uchar>(i-1,j+0) == 0))
        {
          // 3 branches, 4-connected
          is_junction = true;
          //std::cout << "Matched 7 at " << i << ", " << j << std::endl;
        }
        else if ((in_image.at<uchar>(i-1,j+0) != 0) &&
                 (in_image.at<uchar>(i+0,j+1) != 0) &&
                 (in_image.at<uchar>(i+1,j+0) != 0))// &&
                 //(in_image.at<uchar>(i+0,j-1) == 0))
        {
          // 3 branches, 4-connected
          is_junction = true;
          //std::cout << "Matched 8 at " << i << ", " << j << std::endl;
        }
        else if ((in_image.at<uchar>(i-1,j+0) != 0) &&
                 (in_image.at<uchar>(i+0,j-1) != 0) &&
                 (in_image.at<uchar>(i+1,j+0) != 0))// &&
                 //(in_image.at<uchar>(i+0,j+1) == 0))
        {
          // 3 branches, 4-connected
          is_junction = true;
          //std::cout << "Matched 9 at " << i << ", " << j << std::endl;
        }


        /*
        // 4 branches, 8-connected
        else if ((in_image.at<uchar>(i-1,j-1) != 0) &&
            (in_image.at<uchar>(i-1,j+1) != 0) &&
            (in_image.at<uchar>(i+1,j-1) != 0) &&
            (in_image.at<uchar>(i+1,j+1) != 0))
        {
          // 4 branches, 8-connected
          is_junction = true;
          //std::cout << "Matched 0 at " << i << ", " << j << std::endl;
        }
        */



        else if ((in_image.at<uchar>(i-1,j-1) != 0) &&
                 (in_image.at<uchar>(i-1,j+1) != 0) &&
                 (in_image.at<uchar>(i+1,j-1) != 0))
        {
          // 3 branches, 8-connected
          is_junction = true;
          //std::cout << "Matched 1 at " << i << ", " << j << std::endl;
        }
        else if ((in_image.at<uchar>(i-1,j-1) != 0) &&
                 (in_image.at<uchar>(i-1,j+1) != 0) &&
                 (in_image.at<uchar>(i+1,j+1) != 0))
        {
          // 3 branches, 8-connected
          is_junction = true;
          //std::cout << "Matched 2 at " << i << ", " << j << std::endl;
        }
        else if ((in_image.at<uchar>(i-1,j-1) != 0) &&
                 (in_image.at<uchar>(i+1,j-1) != 0) &&
                 (in_image.at<uchar>(i+1,j+1) != 0))
        {
          // 3 branches, 8-connected
          is_junction = true;
          //std::cout << "Matched 3 at " << i << ", " << j << std::endl;
        }
        else if ((in_image.at<uchar>(i-1,j+1) != 0) &&
                 (in_image.at<uchar>(i+1,j-1) != 0) &&
                 (in_image.at<uchar>(i+1,j+1) != 0))
        {
          // 3 branches, 8-connected
          is_junction = true;
          //std::cout << "Matched 4 at " << i << ", " << j << std::endl;
        }






        else if ((in_image.at<uchar>(i-1,j-1) != 0) &&
                 (in_image.at<uchar>(i+0,j+1) != 0) &&
                 (in_image.at<uchar>(i+1,j+0) != 0))
        {
          // two 4-connected, one 8-connected
          is_junction = true;
          //std::cout << "Matched 10 at " << i << ", " << j << std::endl;
        }
        else if ((in_image.at<uchar>(i-1,j+1) != 0) &&
                 (in_image.at<uchar>(i+0,j-1) != 0) &&
                 (in_image.at<uchar>(i+1,j+0) != 0))
        {
          // two 4-connected, one 8-connected
          is_junction = true;
          //std::cout << "Matched 11 at " << i << ", " << j << std::endl;
        }
        else if ((in_image.at<uchar>(i-1,j+0) != 0) &&
                 (in_image.at<uchar>(i+0,j+1) != 0) &&
                 (in_image.at<uchar>(i+1,j-1) != 0))
        {
          // two 4-connected, one 8-connected
          is_junction = true;
          //std::cout << "Matched 12 at " << i << ", " << j << std::endl;
        }
        else if ((in_image.at<uchar>(i-1,j+0) != 0) &&
                 (in_image.at<uchar>(i+0,j-1) != 0) &&
                 (in_image.at<uchar>(i+1,j+1) != 0))
        {
          // two 4-connected, one 8-connected
          is_junction = true;
          //std::cout << "Matched 13 at " << i << ", " << j << std::endl;
        }




        // Add a pixel=0 check for these four:

        else if ((in_image.at<uchar>(i-1,j+0) != 0) &&
                 (in_image.at<uchar>(i+1,j-1) != 0) &&
                 (in_image.at<uchar>(i+1,j+1) != 0) &&
                 (in_image.at<uchar>(i+1,j+0) == 0))
        {
          // one 4-connected, two 8-connected
          is_junction = true;
          //std::cout << "Matched 14 at " << i << ", " << j << std::endl;
        }
        else if ((in_image.at<uchar>(i-1,j-1) != 0) &&
                 (in_image.at<uchar>(i-1,j+1) != 0) &&
                 (in_image.at<uchar>(i+1,j+0) != 0) &&
                 (in_image.at<uchar>(i-1,j+0) == 0))
        {
          // one 4-connected, two 8-connected
          is_junction = true;
          std::cout << "Matched 15 at " << i << ", " << j << std::endl;
        }
        else if ((in_image.at<uchar>(i-1,j+1) != 0) &&
                 (in_image.at<uchar>(i+0,j-1) != 0) &&
                 (in_image.at<uchar>(i+1,j+1) != 0) &&
                 (in_image.at<uchar>(i+0,j+1) == 0))
        {
          // one 4-connected, two 8-connected
          is_junction = true;
          //std::cout << "Matched 16 at " << i << ", " << j << std::endl;
        }
        else if ((in_image.at<uchar>(i-1,j-1) != 0) &&
                 (in_image.at<uchar>(i+0,j+1) != 0) &&
                 (in_image.at<uchar>(i+1,j-1) != 0) &&
                 (in_image.at<uchar>(i+0,j-1) == 0))
        {
          // one 4-connected, two 8-connected
          is_junction = true;
          //std::cout << "Matched 17 at " << i << ", " << j << std::endl;
        }

      }



      if (is_junction)
      {
        RGB& pixel = out_image.ptr<RGB>(i)[j]; // scanline y, pixel x
        pixel.red = 255;
        pixel.green = 0;
        pixel.blue = 0;
      }
    }
  }

}


/*
// Find end-points of skeleton, by counting number of neighbors.
//
// Vertices are drawn as green pixels on top of the skeleton.
//
// This expects out image is 3 channel.
// The outer border of pixels are not checked.
void findEndPoints(const cv::Mat& in_image, cv::Mat& out_image)
{
  //if ((connectivity != 4) && (connectivity != 8))
  //{
    //throw
  //}

  //out_image = cv::Mat(in_image.size(), CV_8UC3);
  //cv::cvtColor(in_image, out_image, CV_GRAY2RGB);

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

      if (in_image.at<uchar>(i+0,j+1) != 0) num_4_conn_px++;
      if (in_image.at<uchar>(i+0,j-1) != 0) num_4_conn_px++;
      if (in_image.at<uchar>(i+1,j+0) != 0) num_4_conn_px++;
      if (in_image.at<uchar>(i-1,j+0) != 0) num_4_conn_px++;

      //if (connectivity == 8)
      {
        if (in_image.at<uchar>(i+1,j+1) != 0) num_8_conn_px++;
        if (in_image.at<uchar>(i+1,j-1) != 0) num_8_conn_px++;
        if (in_image.at<uchar>(i-1,j+1) != 0) num_8_conn_px++;
        if (in_image.at<uchar>(i-1,j-1) != 0) num_8_conn_px++;
      }

      if ((num_4_conn_px + num_8_conn_px) == 1)
      {
        RGB& pixel = out_image.ptr<RGB>(i)[j]; // scanline y, pixel x
        pixel.red = 50;
        pixel.green = 200;
        pixel.blue = 50;
      }
    }
  }
}
*/

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



// Find vertices   METHOD 2
cv::Mat vertices_image;
//const uint connectivity = 4;
//const uint connectivity = 8;
findVerticesUsingTemplates(binary_image, vertices_image);
//cv::imshow("vertices_image", vertices_image);
//cv::waitKey(0);
outfile = output_filename_tokens[0] + std::string("_vertices_method2.png");
cv::imwrite(outfile, vertices_image, png_params);
printNumVertices(vertices_image);
// Create another copy with white backward
cv::Mat inverted_vertices_image = invertBlackAndWhite(vertices_image);
outfile = output_filename_tokens[0] + std::string("_vertices_method2_inv.png");
cv::imwrite(outfile, inverted_vertices_image, png_params);

/*
// Find end-points
cv::Mat endpoints_image = vertices_image;
findEndPoints(binary_image, endpoints_image);
outfile = output_filename_tokens[0] + std::string("_vertices_n_endpoints_method2.png");
cv::imwrite(outfile, endpoints_image, png_params);
// Create another copy with white backward
cv::Mat inverted_endpoints_image = invertBlackAndWhite(endpoints_image);
outfile = output_filename_tokens[0] + std::string("_vertices_n_endpoints_method2_inv.png");
cv::imwrite(outfile, inverted_endpoints_image, png_params);
*/


// Find vertices   METHOD 3
cv::Mat vertices_image2;
//const uint connectivity = 4;
const uint connectivity = 8;
findVerticesUsingTemplates_v3(binary_image, connectivity, vertices_image2);
//cv::imshow("vertices_image2", vertices_image2);
//cv::waitKey(0);
outfile = output_filename_tokens[0] + std::string("_vertices_method3.png");
cv::imwrite(outfile, vertices_image2, png_params);
printNumVertices(vertices_image2);
// Create another copy with white backward
cv::Mat inverted_vertices_image2 = invertBlackAndWhite(vertices_image2);
outfile = output_filename_tokens[0] + std::string("_vertices_method3_inv.png");
cv::imwrite(outfile, inverted_vertices_image2, png_params);


  return 0;
}
