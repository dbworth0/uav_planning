//
// Utility functions
//
// David Butterworth
//

#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <string>
#include <chrono>


// Generate a range (vector) of int values.
// Can be increasing or decreasing.
template <typename Type>
const std::vector<Type> range(const Type start, const Type end);

// Get a range (vector) of values between two numbers,
// with a minimum step size between each value.
// Can be increasing or decreasing.
const std::vector<double> range2(const double a, const double b, const double step_size, const bool include_endpoints);


// Print a std::vector
template <typename Type>
void printStdVector(const typename std::vector<Type> v);

// Initialize a multi-dimensional vector (array) with a specific value.
// For 2D Occupancy Grid:
template <typename Type>
void initializeVector(std::vector<std::vector<std::vector<Type> > >& vector,
                      const size_t dim_1_size, const size_t dim_2_size, const size_t dim_3_size,
                      const Type value);

// Initialize a multi-dimensional vector (array) with a specific value.
// For 3D Occupancy Grid:
template <typename Type>
void initializeVector(std::vector<std::vector<std::vector<Type> > >& vector,
                      const size_t dim_1_size, const size_t dim_2_size, const size_t dim_3_size, const size_t dim_4_size,
                      const Type value);

void initializeVectorOfBool(std::vector<std::vector<std::vector<bool> > >& vector,
                      const size_t dim_1_size, const size_t dim_2_size, const size_t dim_3_size,
                      const bool value);

// Print a multi-dimensional vector
template <typename Type>
void printVector(std::vector<std::vector<std::vector<Type> > >& vector);

template <typename Type>
void printVector(std::vector<std::vector<std::vector<std::vector<Type> > > >& vector);

// Process command-line arguments.
// Expected syntax:
// executable --input /path/to/pointcloud.pcd --start 0.0 0.0 0.0 --goal 1.0 1.0 1.0 --output /path/to/output/traj.ply
const bool processCommandLine(int argc, char** argv,
                              std::string& input_path,
                              std::vector<float>& start,
                              std::vector<float>& goal,
                              std::string& output_path);

// Process command-line arguments.
// Expected syntax:
// executable --input /path/to/pointcloud.pcd --start 0.0 0.0 0.0 --goal 1.0 1.0 1.0 --planner RRT --max_planning_time 60.0 --output /path/to/output/traj.ply
const bool processCommandLine(int argc, char** argv,
                              std::string& input_path,
                              std::vector<float>& start,
                              std::vector<float>& goal,
                              std::string& planner,
                              double& max_planning_time,
                              std::string& output_path);

// Process command-line arguments.
// Expected syntax:
// executable --input /path/to/pointcloud.pcd --start 0.0 0.0 0.0 --goal 1.0 1.0 1.0 --min_height 5.0 --max_height 7.0 --planner RRT --max_planning_time 60.0 --output /path/to/output/traj.ply
const bool processCommandLine3(int argc, char** argv,
                               std::string& input_path,
                               std::vector<float>& start,
                               std::vector<float>& goal,
                               double& min_height,
                               double& max_height,
                               std::string& planner,
                               double& max_planning_time,
                               std::string& output_path);

// Process command-line arguments.
// Expected syntax:
// executable --input /path/to/pointcloud.pcd --start 0.0 0.0 0.0 --goal 1.0 1.0 1.0 --planner RRT --max_planning_time 60.0 \
// --min_height 5.0 --turning_radius 1.0 --max_z_slope 0.15 --output /path/to/output/traj.ply
const bool processCommandLine(int argc, char** argv,
                              std::string& input_path,
                              std::vector<float>& start,
                              std::vector<float>& goal,
                              std::string& planner,
                              double& max_planning_time,
                              double& min_height,
                              double& turning_radius,
                              double& max_z_slope,
                              std::string& output_path);


// Process command-line arguments.
// Expected syntax:
// executable --input /path/to/pointcloud.pcd --start 0.0 0.0 0.0 --goal 1.0 1.0 1.0 --min_height 5.0 --max_height 7.0 \
// --takeoff_funnel <offset> <middle_radius> <top_radius> <cylinder_height> <cone_height> \
// --landing_funnel <offset> <middle_radius> <top_radius> <cylinder_height> <cone_height> \
// --planner RRT --max_planning_time 60.0 --output /path/to/output/traj.ply
const bool processCommandLine5(int argc, char** argv,
                               std::string& input_path,
                               std::vector<float>& start,
                               std::vector<float>& goal,
                               double& min_height,
                               double& max_height,
                               std::vector<float>& takeoff_funnel,
                               std::vector<float>& landing_funnel,
                               std::string& planner,
                               double& max_planning_time,
                               std::string& output_path);



// Process command-line arguments.
// Expected syntax:

// --takeoff_funnel <offset> <middle_radius> <top_radius> <funnel_height>

const bool processCommandLine6(int argc, char** argv,
                               std::string& path_to_pointcloud,
                               std::vector<float>& start,
                               std::vector<float>& goal,

                               double& robot_radius,

                               double& min_height,
                               double& max_height,
                               bool& use_height_constraints,

                               std::vector<float>& takeoff_funnel,
                               std::vector<float>& landing_funnel,
                               bool& use_funnel_constraints,

                               std::string& path_to_elevation_map,
                               double& min_altitude,
                               double& max_altitude,
                               bool& use_altitude_constraint,
                               bool& use_fixed_altitude_at_endpoints,

                               double& endpoints_z_offset,
                               bool& use_offset_endpoints_z_value,

                               std::string& planner,
                               double& max_planning_time,
                               std::string& output_path,
                               double& path_interpolation_step);



// A high-resolution timer, prints output in milliseconds.
class Timer
{
public:
  Timer();
  double read();
  void reset();
private:
  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
};

// Read a line from system status file.
// This assumes that a digit will be found and the line ends in " Kb".
const uint parseLine(char* line);

// Return the virtual memory in use by this process.
// Value is in kB.
const uint getProcessVirtualMemory();

// Return the physical memory (RAM) in use by this process.
// Value is in kB.
const uint getProcessPhysicalMemory();

// Split a string into substrings
const std::vector<std::string> splitString(const std::string& input, const char delimiter = '.');

template <typename Type>
const std::vector<Type> splitStringAsType(const std::string& input, const char delimiter);



// Linearly interpolate between two vectors of N dimensions.
// t = 0.0 to 1.0
const std::vector<std::vector<double> > lerp(const std::vector<double>& q0, const std::vector<double>& q1, const double resolution);

// Implementations:
#include <planning_through_pointclouds/utils.hpp>



#endif // UTILS_H
