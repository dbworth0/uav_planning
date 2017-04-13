//
// Find the connected-components in a 4 or 8-connected binary image.
//
// David Butterworth
//

#ifndef CONNECTED_COMPONENTS_H
#define CONNECTED_COMPONENTS_H

#include <opencv2/core/core.hpp>

//#include <string>
//#include <vtkSmartPointer.h>

namespace cv
{


//int connectedComponents(const cv::Mat &I, cv::Mat &L, int connectivity);
int findConnectedComponents(const cv::Mat& I, const int connectivity, cv::Mat& L);

/*
//! connected components algorithm
enum ConnectedComponentsAlgorithmsTypes {
    CCL_WU      = 0,  //!< SAUF algorithm for 8-way connectivity, SAUF algorithm for 4-way connectivity
    CCL_DEFAULT = -1, //!< BBDT algortihm for 8-way connectivity, SAUF algorithm for 4-way connectivity
    CCL_GRANA   = 1   //!< BBDT algorithm for 8-way connectivity, SAUF algorithm for 4-way connectivity
};
*/

} // end namespace cv

#endif // CONNECTED_COMPONENTS_H
