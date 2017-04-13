//
// Helper functions for VTK (The Visualization Toolkit)
//
// David Butterworth
//

#include <planning_through_pointclouds/vtk_utils.h>

#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkOBJReader.h>
#include <vtkPLYReader.h>
#include <vtkSTLReader.h>

// ToDo: Test with cropped_cloud.ply that contains scalar fields.
//       Need to catch exceptions.
bool loadMeshFileToVtkPolydata(const std::string& file_path,
                               vtkSmartPointer<vtkPolyData>& polydata)
{
  // ToDo: switch to using boost

  const std::string extension = file_path.substr(file_path.find_last_of(".") + 1);

  if (extension == "vtk")
  {
    vtkSmartPointer<vtkPolyDataReader> vtk_reader = vtkSmartPointer<vtkPolyDataReader>::New();
    vtk_reader->SetFileName(file_path.c_str());
    vtk_reader->Update();
    polydata = vtk_reader->GetOutput();
  }
  else if (extension == "ply")
  {
    vtkSmartPointer<vtkPLYReader> ply_reader =
        vtkSmartPointer<vtkPLYReader>::New();
    ply_reader->SetFileName(file_path.c_str());
    ply_reader->Update();
    polydata = ply_reader->GetOutput();
  }
  else if (extension == "obj")
  {
    vtkSmartPointer<vtkOBJReader> obj_reader = vtkSmartPointer<vtkOBJReader>::New();
    obj_reader->SetFileName(file_path.c_str());
    obj_reader->Update();
    polydata = obj_reader->GetOutput();
  }
  else if (extension == "stl")
  {
    vtkSmartPointer<vtkSTLReader> stl_reader =
        vtkSmartPointer<vtkSTLReader>::New();
    stl_reader->SetFileName(file_path.c_str());
    stl_reader->Update();
    polydata = stl_reader->GetOutput();
  }
  else
  {
    std::cout << "ERROR: Un-supported file extension " << extension << " in loadMeshFileToVtkPolydata()" << std::endl;
    return false;
  }

  vtkIdType num_points = polydata->GetPoints()->GetNumberOfPoints();
  if (num_points == 0)
  {
    std::cout << "ERROR: Failed to load mesh file in " << "loadMeshFileToVtkPolydata()" << std::endl;
    return false;
  }
  vtkIdType num_polygons = polydata->GetNumberOfPolys();
  //std::cout << "Loaded mesh with " << num_points << " points and " << num_polygons << " polygons" << std::endl;
  if (num_points == 0)
  {
    std::cout << "ERROR: Mesh has no points in loadMeshFileToVtkPolydata()" << std::endl;
    return false;
  }

  return true;
}

bool getMeshFileBounds(const std::string& file_path, double(&bounds)[6])
{
  vtkSmartPointer<vtkPolyData> polydata;

  bool result = loadMeshFileToVtkPolydata(file_path, polydata);
  if (result == false)
  {
    return false;
  }

  polydata->GetBounds(bounds);

  return true;
}
