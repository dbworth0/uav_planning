//
// Helper functions for OpenCV
//
// David Butterworth
//

#include <planning_through_pointclouds/opencv_utils.h>

#include <iostream> // cout

void printCvMatType(const cv::Mat& image)
{
  std::cout << "cv::Mat type = ";
  switch (image.type())
  {
    case CV_8U:
      std::cout << "8U (CV_8UC1)";
      break;
    case CV_8UC2:
      std::cout << "8UC2";
      break;
    case CV_8UC3:
      std::cout << "8UC3";
      break;
    case CV_8UC4:
      std::cout << "8UC4";
      break;
    case CV_8S:
      std::cout << "8S";
      break;
    case CV_16U:
      std::cout << "16U";
      break;
    case CV_16S:
      std::cout << "16S";
      break;
    case CV_32S:
      std::cout << "32S";
      break;
    case CV_32F:
      std::cout << "32F (CV_32FC1)";
      break;
    case CV_32FC2:
      std::cout << "32FC2";
      break;
    case CV_32FC3:
      std::cout << "32FC3";
      break;
    case CV_32FC4:
      std::cout << "32FC4";
      break;
    case CV_64F:
      std::cout << "64F";
      break;
    default:
      std::cout << "unknown type = " << int(image.type()) << "";
      break;
  }
  std::cout << "  No. channels: " << image.channels();
  std::cout << std::endl;
}
