#pragma once
#include "Ray.h"
#include "Intersection.h"
#include "Bbox.h"
#include <functional>
#include "Eulers.h"
#include <list>
#include "imgui.h"

class Material;

class Shape
{
public:
  Shape() {};
  ~Shape() {};
  virtual void Intersect(const Ray &in, Intersection &i) = 0;
  //TODO: write this for all shapes
  virtual void GetRandomPointOn(Intersection &I) {};

  virtual bool RenderGUI(int shape_num) = 0;

  bool RenderGenericGUI(int shape_num);

  virtual std::string Serialize() { return ""; };

  Material *material;
  Eigen::AlignedBox<float, 3> BoundingBox;
  Eigen::Vector3f Position;
  float SurfaceArea;
  std::string name;

};

class Sphere : public Shape
{
public:
  Sphere() {};
  Sphere(float _Radius, Eigen::Vector3f _Center, Material *m)
    : Center(_Center)
    , Radius(_Radius)
  {
    this->material = m;
    this->name = "Sphere";
    ResetSettings();
  };
  ~Sphere() {};
  void Intersect(const Ray &in, Intersection &i);
  void GetRandomPointOn(Intersection &I);
  void ResetSettings();
  bool RenderGUI(int i);
  virtual std::string Serialize();

  float Radius;
  Eigen::Vector3f Center;

};

class Box : public Shape
{
public:
  Box() {};
  Box(Eigen::Vector3f _base, Eigen::Vector3f _extents, Material *m)
    : base(_base)
    , extents(_extents)
  {
    this->material = m;
    this->name = "box";
    ResetSettings();
  };
  ~Box() {};
  void Intersect(const Ray &in, Intersection &i);
  void ResetSettings();
  bool RenderGUI(int i);
  virtual std::string Serialize();

  Eigen::Vector3f base, extents;

};

class Cylinder : public Shape
{
public:
  Cylinder() {};
  Cylinder(Eigen::Vector3f _Base, Eigen::Vector3f _Axis, float _radius, Material *m) 
  : Base(_Base)
  , Axis(_Axis)
  , radius(_radius)
  {
    this->material = m;
    this->name = "Cylinder";

    ResetSettings();

  };
  ~Cylinder() {};
  void ResetSettings();
  void Intersect(const Ray &in, Intersection &i);
  bool RenderGUI(int i);

  Eigen::Vector3f Base, Axis;
  float radius;

};


class Fractal : public Shape
{
public:
  Fractal() {};
  Fractal(float _Scale, Eigen::Vector3f _Center, Eigen::Quaternionf _rot, Material *m)
  {
    this->name = "Fractal";
    this->material = m;
    Scale = _Scale;
    Center = _Center;
    this->BoundingBox = Eigen::AlignedBox<float, 3>(
      -Eigen::Vector3f::Ones() * 1000,
      Eigen::Vector3f::Ones() * 1000
      );
    this->Position = Center;
    rot_eulers = QuatToEuler(_rot);
    rot = _rot;
    rot_inv = rot.inverse();
  };
  ~Fractal() {};
  void Intersect(const Ray &in, Intersection &i);
  inline void SetRecursionProperties(int _max_iteration, int _subdivisions, float _min_dist) {
    max_iteration = _max_iteration;
    min_distance = _min_dist;
    num_subdivisions = _subdivisions;
  }
  bool RenderGUI(int i);
  virtual std::string Serialize();

  int max_iteration{ 100 };
  float min_distance{ 0.001f };
  int num_subdivisions{ 11 };

  float Scale;
  Eigen::Vector3f Center, rot_eulers;
  Eigen::Quaternionf rot, rot_inv;
  bool flat_color = false;

  struct ActionData {
    int action_type;
    Eigen::Vector3f DisplayOp, VecOp, VecOp2, Color;
    Eigen::Quaternionf QuatOp;
  };

  std::vector<ActionData> CombinedActions;

  std::function<float(Eigen::Vector3f)> ActiveDE;
  std::function<Eigen::Vector3f(Eigen::Vector3f)> ActiveColor;

  float color_it_scale = 3;
  float color_it_add = 0.0;
  float color_it_fold_ratio = 0.5;
  float color_intensity = 0.0;

  void Action_Fold(Eigen::Vector3f &p, int index);
  bool Action_Fold_Color(Eigen::Vector3f &p, int index);
  void Action_Rotate(Eigen::Vector3f &p, int index);
  void Action_Scale(Eigen::Vector3f &p, int index);
  void Action_Translate(Eigen::Vector3f &p, int index);

  float DE_Sphere(Eigen::Vector3f p);
  float DE_Generic(Eigen::Vector3f p);

  Eigen::Vector3f FoldBased(Eigen::Vector3f p);

  int NumFolds();

  void GenColors();
  Eigen::Vector3f ColorFromFloat(float f);
};