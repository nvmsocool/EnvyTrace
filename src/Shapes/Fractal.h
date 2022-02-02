#pragma once
#include "Shape.h"

class Material;

class Fractal : public Shape
{
public:
  Fractal(){};
  Fractal(float _Scale, Eigen::Vector3f _Center, Eigen::Quaternionf _rot, Material *m)
  {
    this->name = "Fractal";
    this->material = m;
    Scale = _Scale;
    Center = _Center;
    this->BoundingBox = Eigen::AlignedBox<float, 3>(
        -Eigen::Vector3f::Ones() * 1000,
        Eigen::Vector3f::Ones() * 1000);
    this->Position = Center;
    rot_eulers = QuatToEuler(_rot);
    rot = _rot;
    rot_inv = rot.inverse();
  };
  ~Fractal(){};
  void Intersect(const Ray &in, Intersection &i);
  inline void SetRecursionProperties(int _max_iteration, int _subdivisions, float _min_dist)
  {
    max_iteration = _max_iteration;
    min_distance = _min_dist;
    num_subdivisions = _subdivisions;
  }
  bool RenderGUI(size_t i);
  virtual std::string Serialize();

  int max_iteration{ 100 };
  float min_distance{ 0.001f };
  int num_subdivisions{ 11 };

  float Scale;
  Eigen::Vector3f Center, rot_eulers;
  Eigen::Quaternionf rot, rot_inv;
  bool flat_color = false;

  enum ACTION_TYPE
  {
    FOLD,
    ROTATION,
    SCALE,
    TRANSLATE,
    MODULO,
    POWER,
    NUM_TYPES
  };

  struct ActionData
  {
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

  void Action_Fold(Eigen::Vector3f &p, size_t index);
  bool Action_Fold_Color(Eigen::Vector3f &p, size_t index);
  void Action_Rotate(Eigen::Vector3f &p, size_t index);
  void Action_Scale(Eigen::Vector3f &p, size_t index);
  void Action_Translate(Eigen::Vector3f &p, size_t index);
  void Action_Modulo(Eigen::Vector3f &p, size_t index);
  void Action_Power(Eigen::Vector3f &p, size_t index);

  float DE_Sphere(Eigen::Vector3f p);
  float DE_Generic(Eigen::Vector3f p);

  Eigen::Vector3f FoldBased(Eigen::Vector3f p);

  int NumFolds();

  void GenColors();
  Eigen::Vector3f ColorFromFloat(float f);
};