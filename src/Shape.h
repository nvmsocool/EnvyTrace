#pragma once
#include "Ray.h"
#include "Intersection.h"
#include "Bbox.h"
#include <functional>
#include "Eulers.h"
#include <list>

class Material;

class Shape
{
public:
  Shape() {};
  ~Shape() {};
  virtual void Intersect(const Ray &in, Intersection &i) = 0;
  //TODO: write this for all shapes
  virtual void GetRandomPointOn(Intersection &I) {};

  Material *material;
  Eigen::AlignedBox<float, 3> BoundingBox;
  Eigen::Vector3f Position;
  float SurfaceArea;

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
    this->BoundingBox = Eigen::AlignedBox<float, 3>(
      Center - Eigen::Vector3f::Ones() * Radius,
      Center + Eigen::Vector3f::Ones() * Radius
    );
    this->Position = Center;
    this->SurfaceArea = 4 * 3.14159 * Radius * Radius;
  };
  ~Sphere() {};
  void Intersect(const Ray &in, Intersection &i);
  void GetRandomPointOn(Intersection &I);

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
    this->BoundingBox = Eigen::AlignedBox<float, 3>(
      base,
      base + extents
    );
    this->Position = base + extents / 2.f;
    this->SurfaceArea =
      2 * extents.x() * extents.y() +
      2 * extents.x() * extents.z() +
      2 * extents.y() * extents.z();
  };
  ~Box() {};
  void Intersect(const Ray &in, Intersection &i);

  Eigen::Vector3f n_x, n_y, n_z;
  Eigen::Vector3f base, extents;

};

class Triangle : public Shape
{
public:
  Triangle() {};
  Triangle(Eigen::Vector3f _p1, Eigen::Vector3f _p2, Eigen::Vector3f _p3, Material *m)
    : p1(_p1), p2(_p2), p3(_p3)
    , e1(p2 - p1), e2(p3 - p1)
  {
    Eigen::Vector3f flatNorm = e2.cross(e1).normalized();
    n1 = flatNorm;
    n2 = flatNorm;
    n3 = flatNorm;
    this->material = m;

    this->BoundingBox = Eigen::AlignedBox<float, 3>(
      Eigen::Vector3f(
        (std::min)(p1.x(), (std::min)(p2.x(), p3.x())),
        (std::min)(p1.y(), (std::min)(p2.y(), p3.y())),
        (std::min)(p1.z(), (std::min)(p2.z(), p3.z()))
      ),
      Eigen::Vector3f(
        (std::max)(p1.x(), (std::max)(p2.x(), p3.x())),
        (std::max)(p1.y(), (std::max)(p2.y(), p3.y())),
        (std::max)(p1.z(), (std::max)(p2.z(), p3.z())))
    );
    this->Position = (p1 + p2 + p3) / 3;
    this->SurfaceArea = e2.cross(e1).norm() / 2;
  };
  ~Triangle() {};
  void Intersect(const Ray &in, Intersection &i);
  inline void SetNormals(Eigen::Vector3f _n1, Eigen::Vector3f _n2, Eigen::Vector3f _n3)
  {
    n1 = _n1.normalized();
    n2 = _n2.normalized();
    n3 = _n3.normalized();
  };
  inline void SetTextureUV(Eigen::Vector2f _t1, Eigen::Vector2f _t2, Eigen::Vector2f _t3)
  {
    t1 = _t1;
    t2 = _t2;
    t3 = _t3;
  };

  Eigen::Vector3f p1, p2, p3, e1, e2, n1, n2, n3;
  Eigen::Vector2f t1, t2, t3;

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

    Eigen::Vector3f minB = Base + Eigen::Vector3f::Ones() * _radius;
    Eigen::Vector3f maxB = Base - Eigen::Vector3f::Ones() * _radius;
    Eigen::Vector3f minA = Base + Axis + Eigen::Vector3f::Ones() * _radius;
    Eigen::Vector3f maxA = Base + Axis - Eigen::Vector3f::Ones() * _radius;

    this->BoundingBox = Eigen::AlignedBox<float, 3>(
      Eigen::Vector3f(
        (std::min)((std::min)(minB.x(), maxB.x()), (std::min)(minA.x(), maxA.x())),
        (std::min)((std::min)(minB.y(), maxB.y()), (std::min)(minA.y(), maxA.y())),
        (std::min)((std::min)(minB.z(), maxB.z()), (std::min)(minA.z(), maxA.z()))
      ),
      Eigen::Vector3f(
        (std::max)((std::max)(minB.x(), maxB.x()), (std::max)(minA.x(), maxA.x())),
        (std::max)((std::max)(minB.y(), maxB.y()), (std::max)(minA.y(), maxA.y())),
        (std::max)((std::max)(minB.z(), maxB.z()), (std::max)(minA.z(), maxA.z()))
      )
    );
    this->Position = Base + Axis / 2.f;
    this->SurfaceArea = 2 * 3.14159 * radius * (Axis.norm() + radius);

  };
  ~Cylinder() {};
  void Intersect(const Ray &in, Intersection &i);

  Eigen::Vector3f Base, Axis;
  float radius;

};


class Fractal : public Shape
{
public:
  Fractal() {};
  Fractal(float _Scale, Eigen::Vector3f _Center, Eigen::Quaternionf _rot, Material *m)
  {
    this->material = m;
    Scale = _Scale;
    Center = _Center;
    this->BoundingBox = Eigen::AlignedBox<float, 3>(
      Center - Eigen::Vector3f::Ones() * Scale * 10,
      Center + Eigen::Vector3f::Ones() * Scale * 10
      );
    this->Position = Center;
    rot = _rot;
    rot_inv = rot.inverse();
    transform_rot1 = EulerToQuat(Eigen::Vector3f(0, 0, 15));
    transform_rot1 = EulerToQuat(Eigen::Vector3f(0, 0, 15));
  };
  ~Fractal() {};
  void Intersect(const Ray &in, Intersection &i);
  inline void SetTrans(Eigen::Quaternionf r1, Eigen::Quaternionf r2)
  {
    transform_rot1 = r1;
    transform_rot2 = r2;
  }
  inline void SetRecursionProperties(int _max_iteration, int _subdivisions, float _min_dist) {
    max_iteration = _max_iteration;
    min_distance = _min_dist;
    num_subdivisions = _subdivisions;
  }
  int max_iteration{ 100 };
  float min_distance{ 0.001f };
  int num_subdivisions{ 11 };
  int num_folds{ 0 };

  float Scale;
  Eigen::Vector3f Center;
  Eigen::Quaternionf rot, rot_inv;
  Eigen::Quaternionf transform_rot1, transform_rot2;

  Eigen::Vector3f lum{ Eigen::Vector3f(0.2126f, 0.7152f, 0.0722f) };


  std::vector<int> ActionIndexes;

  std::function<float(Eigen::Vector3f)> ActiveDE;
  std::function<Eigen::Vector3f(Eigen::Vector3f)> ActiveColor;

  std::vector<Eigen::Vector3f> Folds;
  std::vector<Eigen::Vector3f> Folds_2;
  std::vector<Eigen::Quaternionf> Rotations;
  std::vector<Eigen::Vector3f> Scales;
  std::vector<Eigen::Vector3f> Translates;
  std::vector<Eigen::Vector3f> Colors;
  std::vector<int> ActionToColor;

  void Action_Fold(Eigen::Vector3f &p, int index);
  bool Action_Fold_Color(Eigen::Vector3f &p, int index);
  void Action_Rotate(Eigen::Vector3f &p, int index);
  void Action_Scale(Eigen::Vector3f &p, int index);
  void Action_Translate(Eigen::Vector3f &p, int index);

  float DE_Sphere(Eigen::Vector3f p);
  float DE_Triangle(Eigen::Vector3f p);
  float DE_Generic(Eigen::Vector3f p);

  Eigen::Vector3f GridColor(Eigen::Vector3f p);
  Eigen::Vector3f FlatColor(Eigen::Vector3f p);
  Eigen::Vector3f TriColor(Eigen::Vector3f p);
  Eigen::Vector3f FoldBased(Eigen::Vector3f p);

  void GenColors();
  Eigen::Vector3f ColorFromFloat(float f);
};