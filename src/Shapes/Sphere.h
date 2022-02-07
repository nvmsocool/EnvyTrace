#pragma once
#include "Shape.h"

class Material;

class Sphere : public Shape
{
public:
  Sphere(){};
  Sphere(float _Radius, Eigen::Vector3f _Center, Material *m)
      : Center(_Center), Radius(_Radius)
  {
    this->material = m;
    this->name = "Sphere";
    ResetSettings();
  };
  ~Sphere(){};
  void Intersect(const Ray &in, Intersection &i);
  void GetRandomPointOn(Intersection &I);
  void ResetSettings();
  bool RenderGUI(size_t i);
  virtual std::string Serialize();

  virtual Shape *Clone();
  virtual void SetFromInterpolation(Shape *a, Shape *b, float t);

  float Radius;
  Eigen::Vector3f Center;
};