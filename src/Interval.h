#pragma once
#include "Ray.h"

class Interval
{
public:
  Interval()
      : t_0(0), t_1(std::numeric_limits<float>().max()){};
  Interval(Eigen::Vector3f n, float d_0, float d_1, const Ray &in);
  ~Interval(){};

  float t_0, t_1;
  Eigen::Vector3f n_0, n_1;

  void Empty();
  void Full();
  void Intersect(Interval i);
  void Intersect(Eigen::Vector3f n, float d_0, float d_1, const Ray &in);
};
