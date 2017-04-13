//
// Helper functions for OMPL
//
// David Butterworth
//

#ifndef OMPL_UTILS_H
#define OMPL_UTILS_H

#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <visualization_msgs/MarkerArray.h>

#include <ompl/base/PlannerData.h>

// Create an Rviz Marker representing the sample positions and edges
// expanded by the OMPL planner.
// For a RealVectorStateSpace
const visualization_msgs::MarkerArray getPlannerGraphMarker(const std::string& name,
		                                                        const ompl::base::PlannerDataPtr planner_data,
		                                                        const double line_thickness = 0.005,
		                                                        const double sphere_radius = 0.1,
		                                                        const std::string& frame_id = "map");

// Get a ROS Point Msg from an OMPL State (RealVectorStateSpace)
const geometry_msgs::Point getPointMsgFromState(const int vertex_id, ompl::base::PlannerDataPtr planner_data);

#endif // OMPL_UTILS_H
