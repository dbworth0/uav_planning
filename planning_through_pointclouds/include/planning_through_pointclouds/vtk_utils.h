//
// Helper functions for VTK (The Visualization Toolkit)
//
// David Butterworth
//

#ifndef VTK_UTILS_H
#define VTK_UTILS_H

#include <string>
#include <vtkSmartPointer.h>

// Load a file into a VTK PolyData object.
bool loadMeshFileToVtkPolydata(const std::string& file_path,
                               vtkSmartPointer<vtkPolyData>& polydata);

// Get the x,y,z extents of a cloud, relative to its origin.
bool getMeshFileBounds(const std::string& file_path, double(&bounds)[6]);

#endif // VTK_UTILS_H
