#pragma once
#include "Shape.h"

class Cylinder : public Shape
{
public:
  Cylinder(){};
  Cylinder(Eigen::Vector3f _Base, Eigen::Vector3f _Axis, float _radius, Material *m)
      : Base(_Base), Axis(_Axis), radius(_radius)
  {
    this->material = m;
    this->name = "Cylinder";

    ResetSettings();
  };
  ~Cylinder(){};
  void ResetSettings();
  void Intersect(const Ray &in, Intersection &i);
  bool RenderGUI(size_t i);

  virtual Shape *Clone();
  virtual void SetFromInterpolation(Shape *a, Shape *b, float t);

  Eigen::Vector3f Base, Axis;
  float radius;
};