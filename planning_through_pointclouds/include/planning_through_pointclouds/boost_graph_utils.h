//
// Helper functions for Boost Graph
//
// David Butterworth
//

#ifndef BOOST_GRAPH_UTILS_H
#define BOOST_GRAPH_UTILS_H

#include <vector>

#include <boost/graph/graph_traits.hpp>

template <typename NameMap> class name_equals_t;

// Return true when found a matching vertex name
// in a Boost graph property_map.
template <typename NameMap>
  inline name_equals_t<NameMap>
nameEquals(const std::string& str, NameMap name);

// Output the adjacent vertices, for a vertex given by its name.
//
// Usage:
//   VertexIterator i, end;
//   boost::tie(i, end) = boost::vertices(graph_);
//   i = std::find_if(i, end, nameEquals("v22", boost::get(boost::vertex_name, graph_)));
//   outputAdjacentVertices(std::cout, *i, graph_, boost::get(boost::vertex_name, graph_));
template <typename Graph, typename VertexNameMap>
void outputAdjacentVertices(std::ostream& out,
                          typename boost::graph_traits<Graph>::vertex_descriptor u,
                          const Graph& g,
                          VertexNameMap name_map);

// Output the adjacent vertices, for a vertex given by its ID.
//
// Usage:
//   outputAdjacentVertices(std::cout, 22, graph_);
template <typename Graph>
void outputAdjacentVertices(std::ostream& out,
                        const int vertex_ID,
                        const Graph& g);

// Return the IDs of vertices adjacent to a specified vertex in a Boost graph.
template <typename Graph>
std::vector<int> getAdjacentVertices(const int vertex_ID,
                                     const Graph& g);

#endif // BOOST_GRAPH_UTILS_H
