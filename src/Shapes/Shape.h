#pragma once
#include "..\Ray.h"
#include "..\Intersection.h"
#include "..\Bbox.h"
#include "imgui.h"

class Material;

class Shape
{
public:
  Shape(){};
  ~Shape(){};
  virtual void Intersect(const Ray &in, Intersection &i) = 0;
  //TODO: write this for all shapes
  virtual void GetRandomPointOn(Intersection &I){};

  virtual bool RenderGUI(size_t shape_num) = 0;

  bool RenderGenericGUI(size_t shape_num);

  virtual std::string Serialize() { return ""; };

  Material *material;
  Eigen::AlignedBox<float, 3> BoundingBox;
  Eigen::Vector3f Position;
  float SurfaceArea;
  std::string name;

  virtual Shape* Clone() = 0;
  virtual void SetFromInterpolation(Shape *a, Shape *b, float t) = 0;

  float originErrorMargin {0.001f};
};