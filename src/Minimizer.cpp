#include "pch.h"
#include "Minimizer.h"
#include "Shape.h"
#include "Interval.h"

Minimizer::Minimizer(Ray &r) : ray(r) {}

// Called by BVMinimize to intersect the ray with a Shape. This
// should return the intersection t, but should also track
// the minimum t and it's corresponding intersection info.
// Return INF to indicate no intersection.
float Minimizer::minimumOnObject(Shape *obj)
{
  test_int.Reset();
  obj->Intersect(ray, test_int);
  if (test_int.t < closest_int.t)
    closest_int = test_int;
  return test_int.t;
}

// Called by BVMinimize to intersect the ray with a Bbox and
// returns the t value. This should be similar to the already
// written box (3 slab) intersection. (The difference being: a
// distance of zero should be returned if the ray starts within the Bbox.)
// Return INF to indicate no intersection.
float Minimizer::minimumOnVolume(const Eigen::AlignedBox<float, 3> &box)
{
  Eigen::Vector3f L = (box.min)(); // Box corner
  Eigen::Vector3f U = (box.max)(); // Box corner

  // intersect slabs
  Interval inter;
  Interval i_x(Eigen::Vector3f::UnitX(), -L.x(), -U.x(), ray);
  Interval i_y(Eigen::Vector3f::UnitY(), -L.y(), -U.y(), ray);
  Interval i_z(Eigen::Vector3f::UnitZ(), -L.z(), -U.z(), ray);

  inter.Intersect(i_x);
  inter.Intersect(i_y);
  inter.Intersect(i_z);

  //no intersection
  if (inter.t_0 > inter.t_1 || inter.t_1 < 0)
    return INF;

  // intersection
  if (inter.t_0 > 0)
    return inter.t_0;
  else
    return 0;
}