//
// Helper functions for OMPL
//
// David Butterworth
//

#include <planning_through_pointclouds/ompl_utils.h>

#include <visualization_msgs/Marker.h>

#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

const visualization_msgs::MarkerArray getPlannerGraphMarker(const std::string& name,
                                                            const ompl::base::PlannerDataPtr planner_data,
                                                            const double line_thickness,
                                                            const double sphere_radius,
                                                            const std::string& frame_id)
{
  unsigned int edge_resolution = 100;

  static int counter = 0;

  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker edges_marker;
  edges_marker.header.frame_id = frame_id;
  edges_marker.header.stamp = ros::Time::now();

  visualization_msgs::Marker vertices_marker;
  vertices_marker.header.frame_id = frame_id;
  vertices_marker.header.stamp = ros::Time::now();
  
  {
    edges_marker.action = visualization_msgs::Marker::ADD;
    edges_marker.type = visualization_msgs::Marker::LINE_LIST;

    edges_marker.ns = name;
    edges_marker.id = counter;
    counter++;

    edges_marker.pose.position.x = 0.0;
    edges_marker.pose.position.y = 0.0;
    edges_marker.pose.position.z = 0.0;
    edges_marker.pose.orientation.x = 0.0;
    edges_marker.pose.orientation.y = 0.0;
    edges_marker.pose.orientation.z = 0.0;
    edges_marker.pose.orientation.w = 1.0;

    edges_marker.scale.x = line_thickness;

    // Blue
    edges_marker.color.r = 0.0;
    edges_marker.color.g = 0.0;
    edges_marker.color.b = 1.0;
    edges_marker.color.a = 1.0; // solid color

    // Get graph
    ompl::base::PlannerData::Graph::Type graph = planner_data->toBoostGraph();
    ompl::base::PlannerData::Graph::EIterator ei;
    ompl::base::PlannerData::Graph::EIterator ei_end;
    boost::property_map<ompl::base::PlannerData::Graph::Type, vertex_type_t>::type vertices = boost::get(vertex_type_t(), graph);

    geometry_msgs::Point ps;
    for (boost::tie(ei, ei_end) = boost::edges(graph); ei != ei_end; ++ei)
    {
      // Get path corresponding to edge
      std::vector<ompl::base::State*> block;
      unsigned int ans = planner_data->getSpaceInformation()->getMotionStates(vertices[boost::source(*ei, graph)]->getState(), vertices[boost::target(*ei, graph)]->getState(), block, edge_resolution, true, true);
      BOOST_ASSERT_MSG((int)ans == ans && block.size() == ans, "Internal error in path interpolation. Incorrect number of intermediate states.");

      for (std::vector<ompl::base::State*>::iterator it = block.begin(); it != block.end(); ++it)
      {
        const ompl::base::RealVectorStateSpace::StateType* real_state = (*it)->as<ompl::base::RealVectorStateSpace::StateType>();

        ps.x = real_state->values[0];
        ps.y = real_state->values[1];
        ps.z = real_state->values[2];

        edges_marker.points.push_back(ps);
        edges_marker.colors.push_back(edges_marker.color);

        if (it != block.begin() && boost::next(it) != block.end())
        {
          edges_marker.points.push_back(ps);
          edges_marker.colors.push_back(edges_marker.color);
        }
      }
    }
  }
  
  {
    vertices_marker.ns = name;
    vertices_marker.id = counter;
    counter++;

    vertices_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    vertices_marker.action = visualization_msgs::Marker::ADD;
    
    vertices_marker.pose.position.x = 0.0;
    vertices_marker.pose.position.y = 0.0;
    vertices_marker.pose.position.z = 0.0;
    vertices_marker.pose.orientation.x = 0.0;
    vertices_marker.pose.orientation.y = 0.0;
    vertices_marker.pose.orientation.z = 0.0;
    vertices_marker.pose.orientation.w = 1.0;

    vertices_marker.scale.x = 2.0 * sphere_radius;
    vertices_marker.scale.y = 2.0 * sphere_radius;
    vertices_marker.scale.z = 2.0 * sphere_radius;

    // Red
    edges_marker.color.r = 1.0;
    edges_marker.color.g = 0.0;
    edges_marker.color.b = 0.0;
    edges_marker.color.a = 1.0; // solid color

    std_msgs::ColorRGBA color = edges_marker.color;
    for (std::size_t vertex_id = 0; vertex_id < planner_data->numVertices(); ++vertex_id)
    {
      vertices_marker.points.push_back(getPointMsgFromState(vertex_id, planner_data));
      vertices_marker.colors.push_back(color);
    }
  }

  marker_array.markers.push_back(edges_marker);
  marker_array.markers.push_back(vertices_marker);

  return marker_array;
}

const geometry_msgs::Point getPointMsgFromState(const int vertex_id, ompl::base::PlannerDataPtr planner_data)
{
  ompl::base::PlannerDataVertex* vertex = &planner_data->getVertex(vertex_id);

  if (!vertex->getState())
  {
    throw std::runtime_error("No state found for vertex");
  }

  const ompl::base::RealVectorStateSpace::StateType* real_state =
      static_cast<const ompl::base::RealVectorStateSpace::StateType*>(vertex->getState());

  geometry_msgs::Point point;
  point.x = real_state->values[0];
  point.y = real_state->values[1];
  point.z = real_state->values[2];

  return point;
}
