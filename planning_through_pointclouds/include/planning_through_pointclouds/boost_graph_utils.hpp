//
// Helper functions for Boost Graph
//
// David Butterworth
//

#ifndef BOOST_GRAPH_UTILS_IMPL_HPP
#define BOOST_GRAPH_UTILS_IMPL_HPP

#include <string>
#include <iostream> // cout

template <typename NameMap> class name_equals_t
{
public:
  name_equals_t(const std::string& n, NameMap map)
    : m_name(n), m_name_map(map)
  {
  }
  template <typename Vertex> bool operator() (Vertex u) const
  {
    return get(m_name_map, u) == m_name;
  }
private:
  std::string m_name;
  NameMap m_name_map;
};

template <typename NameMap>
  inline name_equals_t<NameMap>
nameEquals(const std::string& str, NameMap name)
{
  return name_equals_t<NameMap>(str, name);
}

template <typename Graph, typename VertexNameMap>
void outputAdjacentVertices(std::ostream& out,
                            typename boost::graph_traits<Graph>::vertex_descriptor u,
                            const Graph& g,
                            VertexNameMap name_map)
{
  typename boost::graph_traits<Graph>::adjacency_iterator vi;
  typename boost::graph_traits<Graph>::adjacency_iterator vi_end;

  out << boost::get(name_map, u) << " -> { ";
  for (boost::tie(vi, vi_end) = boost::adjacent_vertices(u, g); vi != vi_end; ++vi)
  {
    out << boost::get(name_map, *vi) << " ";
  }
  out << "}" << std::endl;
}

template <typename Graph>
void outputAdjacentVertices(std::ostream& out,
                            const int vertex_ID,
                            const Graph& g)
{
  typename boost::graph_traits<Graph>::vertex_descriptor u = boost::vertex(vertex_ID, g);

  typename boost::graph_traits<Graph>::adjacency_iterator vi;
  typename boost::graph_traits<Graph>::adjacency_iterator vi_end;

  out << vertex_ID << " " << u << " -> { ";
  for (boost::tie(vi, vi_end) = boost::adjacent_vertices(u, g); vi != vi_end; ++vi)
  {
    out << *vi << " ";
  }
  out << "}" << std::endl;
}

template <typename Graph>
std::vector<int> getAdjacentVertices(const int vertex_ID,
                                     const Graph& g)
{
  typename boost::graph_traits<Graph>::vertex_descriptor v = boost::vertex(vertex_ID, g);
  typename boost::graph_traits<Graph>::adjacency_iterator vi;
  typename boost::graph_traits<Graph>::adjacency_iterator vi_end;

  std::vector<int> vertices;
  for (boost::tie(vi, vi_end) = boost::adjacent_vertices(v, g); vi != vi_end; ++vi)
  {
    //std::cout << "v_ID: " << id << std::endl;
    vertices.push_back( *vi );
  }

  return vertices;
}

#endif // BOOST_GRAPH_UTILS_IMPL_HPP
