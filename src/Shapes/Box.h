#pragma once
#include "Shape.h"

class Material;

class Box : public Shape
{
public:
  Box(){};
  Box(Eigen::Vector3f _base, Eigen::Vector3f _extents, Material *m)
      : base(_base), extents(_extents)
  {
    this->material = m;
    this->name = "box";
    ResetSettings();
  };
  ~Box(){};
  void Intersect(const Ray &in, Intersection &i);
  void ResetSettings();
  bool RenderGUI(size_t i);
  virtual std::string Serialize();

  virtual Shape *Clone();
  virtual void SetFromInterpolation(Shape *a, Shape *b, float t);

  Eigen::Vector3f base, extents;
};