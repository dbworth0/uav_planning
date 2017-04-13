//
// Helper functions for using splines
//
// David Butterworth
//

#ifndef SPLINE_UTILS_IMPL_HPP
#define SPLINE_UTILS_IMPL_HPP

#include <iostream>

#include <planning_through_pointclouds/spline_utils.h>

//temp, for debug
#include <planning_through_pointclouds/utils.h> // printStdVector

template <typename PointT>
inline const spline_library::Vector3 getVector3FromPath(const std::vector<PointT>& path, const unsigned int index)
{
  return spline_library::Vector3({path.at(index).x,
                                  path.at(index).y,
                                  path.at(index).z});
}

template <typename PointT>
std::shared_ptr<spline_library::Spline<spline_library::Vector3> > getCentripetalCatmullRomSpline(const std::vector<PointT>& path)
{
  const spline_library::Vector3 start_point_extend_direction = getVector3FromPath(path, 0) - getVector3FromPath(path, 1);
  const spline_library::Vector3 end_point_extend_direction = getVector3FromPath(path, (path.size() - 1)) - getVector3FromPath(path, (path.size() - 2));

  return getCentripetalCatmullRomSpline(path, start_point_extend_direction, end_point_extend_direction);


//oints.push_back(spline_library::Vector3({path.at(i).x, path.at(i).y, path.at(i).z}));

//const PointT p0()

  /*
  std::vector<spline_library::Vector3> points;
  for (size_t i = 0; i < path.size(); ++i)
  {
    points.push_back(spline_library::Vector3({path.at(i).x, path.at(i).y, path.at(i).z}));
  }

  // Add the first and last waypoints twice, with a small position change, so
  // the spline is generated correctly.
  const float extend_length = 0.05; // large enough step to calculate tangent at the endpoints
  spline_library::Vector3 first_point = points.at(0) - points.at(1);
  first_point  = points.at(0) + first_point.normalized() * extend_length;
  spline_library::Vector3 last_point = points.at(points.size()-1) - points.at(points.size()-2);
  last_point  = points.at(points.size()-1) + last_point.normalized() * extend_length;
  //std::cout << "  " << first_point[0] << "," << first_point[1] << "," << first_point[2] << std::endl;
  //std::cout << "  " << last_point[0] << "," << last_point[1] << "," << last_point[2] << std::endl;
  //std::cout << "  " << points.at(0)[0] << "," << points.at(0)[1] << "," << points.at(0)[2] << std::endl;
  //std::cout << "  " << points.back()[0] << "," << points.back()[1] << "," << points.back()[2] << std::endl;
  points.insert(points.begin(), first_point);
  points.push_back(last_point);
  //std::cout << "  " << points.at(0)[0] << "," << points.at(0)[1] << "," << points.at(0)[2] << std::endl;
  //std::cout << "  " << points.back()[0] << "," << points.back()[1] << "," << points.back()[2] << std::endl;

  const float alpha = 0.5; // Centripetal Catmull-Rom Spline
  std::shared_ptr<spline_library::Spline<spline_library::Vector3> > spline = std::make_shared<spline_library::CubicHermiteSpline<spline_library::Vector3> >(points, alpha);
  std::cout << "Spline length = " << spline->totalLength() << "  (time = " << spline->getMaxT() << ")" << std::endl;

  return spline;
*/
  //const unsigned int num_waypoints = static_cast<unsigned int>(std::ceil(spline->totalLength() / 1.0));
  //std::cout << "num_waypoints reqd = " << num_waypoints << std::endl;
  //const unsigned int num_points = 2000;
  //const unsigned int num_points = num_waypoints;
}

template <typename PointT>
std::shared_ptr<spline_library::Spline<spline_library::Vector3> > getCentripetalCatmullRomSpline(const std::vector<PointT>& path,
                                                                                                 const spline_library::Vector3& start_point_extend_direction,
                                                                                                 const spline_library::Vector3& end_point_extend_direction,
                                                                                                 const float extend_length)
{
  std::vector<spline_library::Vector3> points;
  for (size_t i = 0; i < path.size(); ++i)
  {
    points.push_back(spline_library::Vector3({path.at(i).x, path.at(i).y, path.at(i).z}));
  }

  // Add the first and last waypoints twice, with a small position change, so
  // the spline is generated correctly.
  const spline_library::Vector3 new_first_point = points.at(0) + start_point_extend_direction.normalized() * extend_length;
  const spline_library::Vector3 new_last_point  = points.at(points.size()-1) + end_point_extend_direction.normalized() * extend_length;

  points.insert(points.begin(), new_first_point);
  points.push_back(new_last_point);

  const float alpha = 0.5; // 0.5 = Centripetal Catmull-Rom Spline
  std::shared_ptr<spline_library::Spline<spline_library::Vector3> > spline = std::make_shared<spline_library::CubicHermiteSpline<spline_library::Vector3> >(points, alpha);
  std::cout << "Spline length = " << spline->totalLength() << "  (time = " << spline->getMaxT() << ")" << std::endl;

  return spline;
}

template <typename PointT>
std::shared_ptr<spline_library::Spline<spline_library::Vector3> > getCentripetalCatmullRomSpline(const std::vector<PointT>& path,
                                                                                                 const PointT& start_point_extend_direction,
                                                                                                 const PointT& end_point_extend_direction,
                                                                                                 const float extend_length)
{
  const spline_library::Vector3 start_point_extend_direction_vec3({start_point_extend_direction.x, start_point_extend_direction.y, start_point_extend_direction.z});
  const spline_library::Vector3 end_point_extend_direction_vec3({end_point_extend_direction.x, end_point_extend_direction.y, end_point_extend_direction.z});

  return getCentripetalCatmullRomSpline(path, start_point_extend_direction_vec3, end_point_extend_direction_vec3, extend_length);
}









// Chordal
//
// TODO: Make generic code with alpha as a parameter

template <typename PointT>
std::shared_ptr<spline_library::Spline<spline_library::Vector3> > getChordalCatmullRomSpline(const std::vector<PointT>& path,
                                                                                                 const spline_library::Vector3& start_point_extend_direction,
                                                                                                 const spline_library::Vector3& end_point_extend_direction,
                                                                                                 const float extend_length)
{
  std::vector<spline_library::Vector3> points;
  for (size_t i = 0; i < path.size(); ++i)
  {
    points.push_back(spline_library::Vector3({path.at(i).x, path.at(i).y, path.at(i).z}));
  }

  // Add the first and last waypoints twice, with a small position change, so
  // the spline is generated correctly.
  const spline_library::Vector3 new_first_point = points.at(0) + start_point_extend_direction.normalized() * extend_length;
  const spline_library::Vector3 new_last_point  = points.at(points.size()-1) + end_point_extend_direction.normalized() * extend_length;

  points.insert(points.begin(), new_first_point);
  points.push_back(new_last_point);

  const float alpha = 1.0; // 1.0 = Chordal Catmull-Rom Spline
  std::shared_ptr<spline_library::Spline<spline_library::Vector3> > spline = std::make_shared<spline_library::CubicHermiteSpline<spline_library::Vector3> >(points, alpha);
  std::cout << "Spline length = " << spline->totalLength() << "  (time = " << spline->getMaxT() << ")" << std::endl;

  return spline;
}

template <typename PointT>
std::shared_ptr<spline_library::Spline<spline_library::Vector3> > getChordalCatmullRomSpline(const std::vector<PointT>& path,
                                                                                                 const PointT& start_point_extend_direction,
                                                                                                 const PointT& end_point_extend_direction,
                                                                                                 const float extend_length)
{
  const spline_library::Vector3 start_point_extend_direction_vec3({start_point_extend_direction.x, start_point_extend_direction.y, start_point_extend_direction.z});
  const spline_library::Vector3 end_point_extend_direction_vec3({end_point_extend_direction.x, end_point_extend_direction.y, end_point_extend_direction.z});

  return getChordalCatmullRomSpline(path, start_point_extend_direction_vec3, end_point_extend_direction_vec3, extend_length);
}



// NOTE: the length calculated as we go is different to calculating it at the end, which is now
// what we use as final result.
// Need to check why there is a difference.
template <typename PointT>
void getPointsFromSpline(const std::shared_ptr<spline_library::Spline<spline_library::Vector3> >& spline,
                         const float t_start,
                         const float t_end,
                         const float dist_step,
                         std::vector<PointT>& splined_path_points,
                         double& spline_length)
{
  const float t_max = spline->getMaxT();

  if (t_start < 0.0)
  {
    std::cout << "ERROR: t_start must be between 0 and t_max" << std::endl;
  }
  else if ((t_end < t_start) || (t_end > t_max))
  {
    std::cout << "ERROR: t_end must be between t_start and t_max" << std::endl;
  }

  // Add first point
  const spline_library::Vector3 start_point = spline->getPosition(t_start);
  splined_path_points.push_back(PointT(start_point[0], start_point[1], start_point[2]));

  float accum_distance = 0.0;
  float delta_dist_accum = 0.0;
  float t = t_start;
  const float t_step = 0.00001; // this must be very small
  spline_library::Vector3 prev_point = spline->getPosition(0.0);

  //std::cout << "in getPointsFromSpline(),   accum_distance: " << accum_distance << std::endl;
//  std::vector<double> distances;
//  distances.push_back(accum_distance);

  bool done = false;
  while (!done)
  {
    const spline_library::Vector3 point = spline->getPosition(t);
    const float delta_dist = euclideanDistance(point, prev_point);
    delta_dist_accum += delta_dist;
    //std::cout << "delta_dist_accum: " << delta_dist_accum << std::endl;

    if (delta_dist_accum > dist_step)
    {
      // Add a point:
      // (The previous time value is used so our distance step
      //  is not greater than desired.)
      t -= t_step;
      const spline_library::Vector3 point_to_add = spline->getPosition( t ); // TODO: Check   point_to_add   is just  prev_point
      splined_path_points.push_back(PointT(point_to_add[0], point_to_add[1], point_to_add[2]));

      // For debugging:
      //const float delta_dist_actual = euclideanDistance(point_to_add, prev_point);

      delta_dist_accum -= delta_dist;

      prev_point = point_to_add;

      // debug, accumulate distance
      accum_distance += delta_dist_accum;

      //std::cout << "in getPointsFromSpline(),   accum_distance: " << accum_distance << std::endl;
//      distances.push_back(accum_distance);

      /*
      std::cout << "  t = " << t
              << "  " << point[0] << "," << point[1] << "," << point[2]
              //<< "  delta_dist: " << delta_dist
              //<< "  delta_dist_actual: " << delta_dist_actual
              << "  delta_dist_accum: " << delta_dist_accum
              //<< "  goal: " << goal_point.x << "," << goal_point.y
              << "  accum_distance: " << accum_distance
              << std::endl;
      */

      delta_dist_accum = 0.0; // reset accumulator
    }
    else
    {
      //std::cout << "  t = " << t << "  " << point[0] << "," << point[1] << "," << point[2] << std::endl;
      prev_point = point;
    }



    t += t_step;
    
    if (t > t_end)
    {
      done = true;
    }
  }

  // Add last point
  const spline_library::Vector3 end_point = spline->getPosition(t_end);
  splined_path_points.push_back(PointT(end_point[0], end_point[1], end_point[2]));

  t -= t_step;
  const float final_delta_dist = euclideanDistance(end_point, prev_point);
  //std::cout << "t: " << t << "  t_max: " << t_max << "  final_delta_dist: " << final_delta_dist << std::endl;
  accum_distance += final_delta_dist;

  //std::cout << "in getPointsFromSpline(),   accum_distance: " << accum_distance << std::endl;
  std::cout << "accum_distance: " << accum_distance << std::endl;
//  distances.push_back(accum_distance);
//  std::cout << "accum_distances: " << std::endl;
//  printStdVector(distances);



  double accum_distance2 = 0.0;
  for (unsigned int i = 0; i < splined_path_points.size()-1; ++i)
  {
    const double delta_dist = static_cast<double>(euclideanDistance(splined_path_points.at(i), splined_path_points.at(i+1)));
    accum_distance2 += delta_dist;
  }
  std::cout << "accum_distance2: " << accum_distance2 << std::endl;



  std::cout << "Splined path now has " << splined_path_points.size() << " waypoints" << std::endl;

  //spline_length = accum_distance;
  spline_length = accum_distance2;
}

template <typename PointT>
const std::vector<PointT> getPointsFromSpline(const std::shared_ptr<spline_library::Spline<spline_library::Vector3> >& spline,
                                              const float dist_step)
{
  const float t_start = 0.0;
  const float t_end = spline->getMaxT();

  std::vector<PointT> splined_path_points;
  double spline_length = 0.0;

  getPointsFromSpline<PointT>(spline, t_start, t_end, dist_step,
                              splined_path_points, spline_length);

  return splined_path_points;
}

template <typename PointT>
void getPointsFromSpline(const std::shared_ptr<spline_library::Spline<spline_library::Vector3> >& spline,
                         const float dist_step,
                         std::vector<PointT>& splined_path_points,
                         double& spline_length)
{
  const float t_start = 0.0;
  const float t_end = spline->getMaxT();

  getPointsFromSpline<PointT>(spline, t_start, t_end, dist_step,
                              splined_path_points, spline_length);
  //std::cout << "spline_length " << spline_length << std::endl;
}

#endif // SPLINE_UTILS_IMPL_HPP
