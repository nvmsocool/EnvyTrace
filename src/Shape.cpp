#include "Shape.h"
#include "Interval.h"
#include "material.h"

#include <iostream>

static const float OriginErrorMargin = 0.001f;

void Sphere::Intersect(const Ray &in, Intersection &i)
{
  //qbar = Q - C
  //d.d * t^2 + qbar.d * t + (qbar.qbar - r^2) = 0
  //if D is normalized: t = -Qbar.d +- sqrt(qbar.d ^ 2 - qbar.qbar + r^2)

  Eigen::Vector3f qBar = in.origin - Center;

  float qBar_D = qBar.dot(in.direction);
  float qBar_qBar = qBar.dot(qBar);

  float disc = qBar_D * qBar_D - qBar_qBar + Radius * Radius;

  // whole line misses sphere: no intersection
  if (disc < 0)
    return;

  // else calc both roots
  float t_a = -qBar_D + std::sqrt(disc);
  float t_b = -qBar_D - std::sqrt(disc);
  float t_0 = std::min(t_a, t_b);
  float t_1 = std::max(t_a, t_b);

  // sphere behind ray: no intersection
  if (t_1 < OriginErrorMargin)
    return;

  //intersection
  i.t = t_0 < OriginErrorMargin ? t_1 : std::min(t_0, t_1);
  i.object = this;
  i.P = in.eval(i.t);
  i.N = (i.P - Center).normalized();
}

void Sphere::GetRandomPointOn(Intersection &I)
{
  float e1 = randf();
  float e2 = randf();
  float z = 2 * e1 - 1;
  float r = std::sqrt(1 - z * z);
  float a = 2 * 3.14159 * e2;
  I.N = Eigen::Vector3f(r * std::cos(a), r * std::sin(a), z).normalized();
  I.P = Center + Radius * I.N;
  I.object = this;
}

void Box::Intersect(const Ray &in, Intersection &i)
{
  // intersect slabs
  Interval inter;
  Interval i_x(Eigen::Vector3f::UnitX(), -base.x(), -base.x() - extents.x(), in);
  Interval i_y(Eigen::Vector3f::UnitY(), -base.y(), -base.y() - extents.y(), in);
  Interval i_z(Eigen::Vector3f::UnitZ(), -base.z(), -base.z() - extents.z(), in);

  inter.Intersect(i_x);
  inter.Intersect(i_y);
  inter.Intersect(i_z);

  //no intersection
  if (inter.t_0 > inter.t_1 || inter.t_1 < OriginErrorMargin)
    return;
  
  // intersection
  if (inter.t_0 > OriginErrorMargin)
  {
    i.t = inter.t_0;
    i.N = inter.n_0;
  }
  else
  {
    i.t = inter.t_1;
    i.N = inter.n_1;
  }

  i.object = this;
  i.P = in.eval(i.t);
  //get normal?
  //get uv

}

void Triangle::Intersect(const Ray &in, Intersection &i)
{
  //intersects if 
  // a solution exists for u,v s.t: intersection = p1 + u * e1 + v * e2
  // 0 <= u <= 1,
  // 0 <= v <= 1
  // 0 <= 1 - u - v <= 1

  //algorithm via triple product
  Eigen::Vector3f p = in.direction.cross(e2);
  float d = p.dot(e1);

  //parallel? return no intersection
  if (d == 0)
    return;

  Eigen::Vector3f S = in.origin - p1;
  float u = p.dot(S) / d;

  //outside e2? return no intersection
  if (u < 0 || u > 1)
    return;

  Eigen::Vector3f q = S.cross(e1);
  float v = in.direction.dot(q) / d;

  //outside edges? no intersection
  if (v < 0 || (u + v) > 1)
    return;

  float t = e2.dot(q) / d;

  //beyond midline? no intersection
  if (t < OriginErrorMargin)
    return;

  //intersection
  i.t = t;
  i.P = in.eval(i.t);
  i.N = (1 - u - v) * n1 + u * n2 + v * n3;
  i.object = this;
  i.uv = (1 - u - v) * t1 + u * t2 + v * t3;
}

void Cylinder::Intersect(const Ray &in, Intersection &i)
{
  //translate to origin, rotate to z up
  //todo:do this earlier, only once
  Eigen::Quaternionf q = Eigen::Quaternionf::FromTwoVectors(Axis, Eigen::Vector3f::UnitZ());

  //transform ray
  Ray rotated(q._transformVector(in.origin - Base), q._transformVector(in.direction));
  
  //full ray
  Interval All;

  //end plane slab
  Interval EndCaps(Eigen::Vector3f::UnitZ(), 0, -Axis.norm(), rotated);
  //normal is from slab?
  
  //circular top-down
  Interval TopDown;
  float a = rotated.direction.x() * rotated.direction.x() + rotated.direction.y() * rotated.direction.y();
  //dx*dx+dy*dy
  float b = 2 * (rotated.direction.x() * rotated.origin.x() + rotated.direction.y() * rotated.origin.y());
  //dx*qx+dy*qy
  float c = rotated.origin.x() * rotated.origin.x() + rotated.origin.y() * rotated.origin.y() - radius * radius;
  //qx*qx+qy*qy-r*r
  float discriminant = b * b - 4 * a * c;

  //misses circle alltogether, no intersection
  if (discriminant < 0)
    return;

  //TODO: make sure min/max
  float eval = std::sqrt(discriminant);
  float t_a = (-b - eval) / (2 * a);
  float t_b = (-b + eval) / (2 * a);
  TopDown.t_0 = std::min(t_a, t_b);
  TopDown.t_1 = std::max(t_a, t_b);

  Eigen::Vector3f M_0 = (rotated.origin + TopDown.t_0 * rotated.direction);
  Eigen::Vector3f M_1 = (rotated.origin + TopDown.t_1 * rotated.direction);
  M_0.z() = 0;
  M_1.z() = 0;
  TopDown.n_0 = q.conjugate()._transformVector(M_0).normalized();
  TopDown.n_1 = q.conjugate()._transformVector(M_1).normalized();

  All.Intersect(EndCaps);
  All.Intersect(TopDown);

  //no intersection
  if (All.t_0 > All.t_1 || All.t_1 < OriginErrorMargin)
    return;

  //intersection
  i.object = this;
  if (All.t_0 > OriginErrorMargin)
  {
    i.t = All.t_0;
    i.N = All.n_0;
  }
  else
  {
    i.t = All.t_1;
    i.N = All.n_1;
  }
  i.P = in.eval(i.t);

  //uv?
  
}

static const Eigen::Vector3f norm_step_x(1.f, 0.0f, 0.0f);
static const Eigen::Vector3f norm_step_y(0.0f, 1.f, 0.0f);
static const Eigen::Vector3f norm_step_z(0.0f, 0.0f, 1.f);

void Fractal::Intersect(const Ray &in, Intersection &i)
{
  // plan: step along hte ray, starting at the ray point, with the step size of DE. 
  // if it keeps getting bigger then return no intersection
  // else if its below a threshold, return an intersection?

  float dist = 0;
  int colorsteps = 0;
  int totalColorSteps = max_iteration;
  float lastEst;
  for (int steps = 0; steps < max_iteration; steps++)
  {
    Eigen::Vector3f p = in.origin + dist * in.direction;
    float estimate = DE_Generic(p);
    dist += estimate;
    if (dist > OriginErrorMargin && estimate < min_distance)
    {
      //intersection, huzzah
      i.t = dist;
      i.P = in.eval(i.t);
      i.object = this;
      //i.Kd = FoldBased(i.P);
      float f = (static_cast<float>(colorsteps) / static_cast<float>(max_iteration)) * 3.f;
      while (f > 1) f -= 1;
      //i.Kd = ((ColorFromFloat(f) + 3 * Eigen::Vector3f::Ones()) * 0.25f);

      //norm needs to be estimated
      Eigen::Vector3f step_back = in.origin + (dist - min_distance) * in.direction;
      float step_size = DE_Generic(step_back);
      i.N = Eigen::Vector3f(
        DE_Generic(step_back + step_size * norm_step_x) - DE_Generic(step_back - step_size * norm_step_x),
        DE_Generic(step_back + step_size * norm_step_y) - DE_Generic(step_back - step_size * norm_step_y),
        DE_Generic(step_back + step_size * norm_step_z) - DE_Generic(step_back - step_size * norm_step_z)
      ).normalized();
      return;
    }
    if (estimate > lastEst)
    {
      //colorsteps = 1;
    }
    colorsteps += 1;
    lastEst = estimate;
  }
  //no intersection
}

float Fractal::DE_Sphere(Eigen::Vector3f p)
{
  Eigen::Vector3f floored(std::fmod(p.x(), 1.f) - 0.5f, std::fmod(p.y(), 1.f) - 0.5f, p.z());
  
  return floored.norm() - Scale;
}

const float bailout_dist = 1000;


float Fractal::DE_Triangle(Eigen::Vector3f _z)
{
  Eigen::Vector3f z = rot_inv._transformVector(_z) - Center;
  float r = z.squaredNorm();
  int i = 0;
  for (i = 0; i < num_subdivisions && r < bailout_dist; i++)
  {
    z = transform_rot1._transformVector(z);
    if (z.x() + z.y() < 0) z = Eigen::Vector3f(-z.y(), -z.x(), z.z());
    if (z.x() + z.z() < 0) z = Eigen::Vector3f(-z.z(), z.y(), -z.x());
    if (z.y() + z.z() < 0) z = Eigen::Vector3f(z.x(), -z.z(), -z.y());

    z = transform_rot2._transformVector(z);

    z = Scale * z - Eigen::Vector3f::Ones() * (Scale - 1.f);
    r = z.squaredNorm();
  }
  return (std::sqrt(r) - 2.f) * std::pow(Scale, -i);
}


float Fractal::DE_Generic(Eigen::Vector3f _z)
{
  //Eigen::Vector3f z = rot_inv._transformVector(_z) - Center;
  Eigen::Vector3f z = _z;
  float r = z.squaredNorm();
  int i = 0;
  for (i = 0; i < num_subdivisions && r < bailout_dist; i++)
  {
    int fold_index = 0;
    int rotate_index = 0;
    int scale_index = 0;
    int translate_index = 0;
    for (int i = 0; i < ActionIndexes.size(); i++)
    {
      switch(ActionIndexes[i])
        {
        case 0: // fold
          Action_Fold(z, fold_index++);
          break;
        case 1: //rotation
          Action_Rotate(z, rotate_index++);
          break;
        case 2: //scale
          Action_Scale(z, scale_index++);
          break;
        case 3: //translation
          Action_Translate(z, translate_index++);
          break;
        }
    }

    //z = Scale * (z - Eigen::Vector3f::Ones()) + Eigen::Vector3f::Ones();
    z = z * Scale - Center * (Scale - 1);
    r = z.squaredNorm();
  }

  return (std::sqrt(r)-2.f) * std::pow(Scale, -i);
}

bool Fractal::Action_Fold_Color(Eigen::Vector3f &p, int fold_index)
{
  float dot = p.dot(Folds[fold_index]);
  if (dot < 0)
  {
    p -= dot * Folds_2[fold_index];
    return true;
  }
  return false;
}

void Fractal::Action_Fold(Eigen::Vector3f &p, int fold_index)
{
  float dot = p.dot(Folds[fold_index]);
  if (dot < 0)
    p -= dot * Folds_2[fold_index];
}

void Fractal::Action_Rotate(Eigen::Vector3f &p, int rot_index)
{
  p = Rotations[rot_index]._transformVector(p);
}

void Fractal::Action_Scale(Eigen::Vector3f &p, int scale_index)
{
  p = p.cwiseProduct(Scales[scale_index]);
}

void Fractal::Action_Translate(Eigen::Vector3f &p, int trans_index)
{
  p = p + Translates[trans_index];
}

Eigen::Vector3f Fractal::GridColor(Eigen::Vector3f p)
{
  float x, y;
  modf(p.x(), &x);
  modf(p.y(), &y);
  float x_rand = static_cast<float>(static_cast<int>(x * 123456.789f) % 100) / 100.f;
  float y_rand = static_cast<float>(static_cast<int>(y * 123456.789f) % 100) / 100.f;
  return Eigen::Vector3f(x_rand, y_rand, 0.5f);
}

Eigen::Vector3f Fractal::FlatColor(Eigen::Vector3f p)
{
  return this->material->Kd;
}

Eigen::Vector3f Fractal::TriColor(Eigen::Vector3f _z)
{
  Eigen::Vector3f c(Eigen::Vector3f::Ones());
  Eigen::Vector3f z = rot_inv._transformVector(_z) - Center;
  float r = z.squaredNorm();
  int i = 0;
  float w = 1;
  for (i = 0; i < num_subdivisions && r < bailout_dist; i++)
  {
    z = transform_rot1._transformVector(z);
    if (z.x() + z.y() < 0) { z = Eigen::Vector3f(-z.y(), -z.x(),  z.z()); c = Eigen::Vector3f(0,0,1) * w + c * (1-w); };
    if (z.x() + z.z() < 0) { z = Eigen::Vector3f(-z.z(),  z.y(), -z.x()); c = Eigen::Vector3f(0,1,0) * w + c * (1-w); };
    if (z.y() + z.z() < 0) { z = Eigen::Vector3f( z.x(), -z.z(), -z.y()); c = Eigen::Vector3f(1,0,0) * w + c * (1-w); };

    w *= 0.5f;
    z = transform_rot2._transformVector(z);

    z = Scale * z - Eigen::Vector3f::Ones() * (Scale - 1.f);
    r = z.squaredNorm();
  }
  return c;
}


Eigen::Vector3f Fractal::FoldBased(Eigen::Vector3f _z)
{
  Eigen::Vector3f c(1.0, 1.0, 1.0);
  Eigen::Vector3f z = rot_inv._transformVector(_z) - Center;
  float r = z.squaredNorm();
  int i = 0;
  float w = 1;
  for (i = 0; i < num_subdivisions && r < bailout_dist; i++)
  {
    int fold_index = 0;
    int rotate_index = 0;
    int scale_index = 0;
    int translate_index = 0;
    for (int action_num = 0; action_num < ActionIndexes.size(); action_num++)
    {
      switch (ActionIndexes[action_num])
      {
      case 0: // fold
        if (Action_Fold_Color(z, fold_index++))
          c = (1 - w) * c + w * Colors[ActionToColor[action_num]];
        break;
      case 1: //rotation
        Action_Rotate(z, rotate_index++);
        break;
      case 2: //scale
        Action_Scale(z, scale_index++);
        break;
      case 3: //translation
        Action_Translate(z, translate_index++);
        break;
      }
    }

    w *= 0.5f;
    z = Scale * z - Eigen::Vector3f::Ones() * (Scale - 1.f);
    r = z.squaredNorm();
  }

  //do a precise check here on a unit tetrahedron

  return c;
}

void Fractal::GenColors()
{
  Colors.reserve(num_folds);
  for (int i = 0; i < num_folds; i++)
  {
    float val = static_cast<float>(i) / static_cast<float>(num_folds);
    Colors.push_back(ColorFromFloat(val));
  }
  for (int i = 0; i < Folds.size(); i++)
  {
    Folds_2.push_back(Folds[i] * 2.f);
  }
}

Eigen::Vector3f Fractal::ColorFromFloat(float val)
{
  float r = std::sin(pi_2 * (val)) * 0.5 + 0.5;
  float g = std::sin(pi_2 * (val + 0.33333333f)) * 0.5 + 0.5;
  float b = std::sin(pi_2 * (val + 0.66666666f)) * 0.5 + 0.5;
  Eigen::Vector3f ret(r, g, b);
  return ret;
}
