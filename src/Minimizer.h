#pragma once
#include "Ray.h"
#include "Intersection.h"
#include "Bbox.h"

class Shape;
class Intersection;

const float INF = (std::numeric_limits<float>::max)();

class Minimizer
{
public:
  typedef float Scalar; // KdBVH needs Minimizer::Scalar defined
  Ray &ray;
  float min_t;
  Intersection closest_int, test_int;

  Minimizer(Ray &r);
  float minimumOnObject(Shape *obj);
  float minimumOnVolume(const Eigen::AlignedBox<float, 3> &box);
};