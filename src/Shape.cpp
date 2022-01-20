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
  i.Calc_IOR_Ratio(-in.direction);
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
  i.Calc_IOR_Ratio(-in.direction);
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
  i.Calc_IOR_Ratio(-in.direction);
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
  i.Calc_IOR_Ratio(-in.direction);

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
  //float lastEst = 0;
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
      i.Calc_IOR_Ratio(-in.direction);
      float f = (static_cast<float>(colorsteps) / static_cast<float>(max_iteration)) * color_it_scale + color_it_add;
      while (f > 1) f -= 1;
      Eigen::Vector3f it_based = ColorFromFloat(f);

      if (color_it_intensity < 0)
        it_based *= (1 + color_it_intensity);
      else
        it_based += (Eigen::Vector3f::Ones() - it_based) * color_it_intensity;

      Eigen::Vector3f fold_based = FoldBased(i.P);

      i.Kd = color_it_fold_ratio * it_based + (1.f - color_it_fold_ratio) * fold_based;

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
    //if (estimate > lastEst)
    {
      //colorsteps = 1;
    }
    colorsteps += 1;
    //lastEst = estimate;
  }
  //no intersection
}

#include "Eulers.h"

bool Fractal::RenderGUI(int n)
{
  bool something_changed = false;

  something_changed |= ImGui::InputInt((std::string("max_iteration") + std::to_string(n)).data(), &max_iteration);
  something_changed |= ImGui::DragFloat((std::string("min_distance") + std::to_string(n)).data(), &min_distance, 0.00001f, 0.0f, 1000.0f, "%.5f");
  something_changed |= ImGui::InputInt((std::string("num_subdivisions") + std::to_string(n)).data(), &num_subdivisions);

  ImGui::Separator();

  something_changed |= ImGui::DragFloat3((std::string("center") + std::to_string(n)).data(), Center.data(), 0.01f, -10000, 10000, "%.2f");
  something_changed |= ImGui::DragFloat((std::string("scale") + std::to_string(n)).data(), &Scale, 0.001f, 0, 100000, "%.3f");
  if (ImGui::DragFloat3((std::string("rotation") + std::to_string(n)).data(), &rot_eulers[0], 0.1f, -180, 180, "%.1f"))
  {
    something_changed = true;
    rot = EulerToQuat(rot_eulers);
    rot_inv = rot.inverse();
  }

  ImGui::Separator();

  something_changed |= ImGui::SliderFloat((std::string("color_it_add") + std::to_string(n)).data(), &color_it_add, 0.f, 1.f);
  something_changed |= ImGui::DragFloat((std::string("color_it_scale") + std::to_string(n)).data(), &color_it_scale, 0.1f, 0.f, 1000.f, "%.1f");
  something_changed |= ImGui::SliderFloat((std::string("color_it_fold_ratio") + std::to_string(n)).data(), &color_it_fold_ratio, 0.f, 1.f);
  something_changed |= ImGui::SliderFloat((std::string("color_it_intensity") + std::to_string(n)).data(), &color_it_intensity, -1.f, 1.f);

  for (size_t i = 0; i < CombinedActions.size(); i++)
  {

    ImGui::Separator();
    const char *items[] = {
    "Fold",
    "Rotation",
    "Scale",
    "Translation", };

    if (ImGui::Combo((std::string("type##") + std::to_string(n) + std::to_string(i)).data(), &CombinedActions[i].action_type, items, IM_ARRAYSIZE(items), 4))
    {
      something_changed = true;
      switch (CombinedActions[i].action_type)
      {
      case 0:
        // fold
        CombinedActions[i].DisplayOp = Eigen::Vector3f(1, 0, 0);
        break;
      case 1:
        //rotation
        CombinedActions[i].DisplayOp = Eigen::Vector3f(0, 0, 0);
        CombinedActions[i].QuatOp = EulerToQuat(CombinedActions[i].DisplayOp);
        break;
      case 2:
        //scale
        CombinedActions[i].DisplayOp = Eigen::Vector3f(1, 1, 1);
        break;
      case 3:
        //translation
        CombinedActions[i].DisplayOp = Eigen::Vector3f(0, 0, 0);
        break;
      }
      CombinedActions[i].VecOp = CombinedActions[i].DisplayOp;
    }
    std::string label = "##action" + std::to_string(n) + std::to_string(i);
    switch (CombinedActions[i].action_type) {
    case 0:
      // fold
      if (ImGui::DragFloat3(label.data(), CombinedActions[i].DisplayOp.data(), 0.01f, -1.f, 1.f, "%.2f"))
      {
        something_changed = true;
        CombinedActions[i].VecOp = CombinedActions[i].DisplayOp.normalized();
      }
      if (ImGui::DragFloat3(("fold_color" + std::to_string(n) + std::to_string(i)).data(), CombinedActions[i].Color.data(), 0.01f, 0.f, 1.f, "%.2f"))
      {
        something_changed = true;
      }
      break;
    case 1:
      //rotation
      if (ImGui::DragFloat3(label.data(), CombinedActions[i].DisplayOp.data(), 0.1f, -180, 180.f, "%.1f"))
      {
        something_changed = true;
        CombinedActions[i].QuatOp = EulerToQuat(CombinedActions[i].DisplayOp);
      }
      break;
    case 2:
      //scale
      something_changed |= ImGui::DragFloat3(label.data(), CombinedActions[i].VecOp.data(), 0.01f, -1000, 1000, "%.2f");
      break;
    case 3:
      //translation
      something_changed |= ImGui::DragFloat3(label.data(), CombinedActions[i].VecOp.data(), 0.01f, -1000, 1000, "%.2f");
      break;
    }
    if (ImGui::Button((std::string("+##") + std::to_string(n) + std::to_string(i)).data()))
    {
      something_changed = true;
      //insert new operation
      CombinedActions.insert(CombinedActions.begin() + i, CombinedActions[i]);
      if (CombinedActions[i].action_type == 0)
      {
        GenColors();
      }
    }

    ImGui::SameLine();
    if (ImGui::Button((std::string("X##") + std::to_string(n) + std::to_string(i)).data()))
    {
      something_changed = true;
      //delete this operation
      CombinedActions.erase(CombinedActions.begin() + i);
      if (CombinedActions[i].action_type == 0)
      {
        GenColors();
      }
    }
  }
  return something_changed;
}

float Fractal::DE_Sphere(Eigen::Vector3f p)
{
  Eigen::Vector3f floored(std::fmod(p.x(), 1.f) - 0.5f, std::fmod(p.y(), 1.f) - 0.5f, p.z());

  return floored.norm() - Scale;
}

const float bailout_dist = 1000;


float Fractal::DE_Generic(Eigen::Vector3f _z)
{
  Eigen::Vector3f z = rot_inv._transformVector(_z) - Center;
  //Eigen::Vector3f z = _z;
  float r = z.squaredNorm();
  int i = 0;
  for (i = 0; i < num_subdivisions && r < bailout_dist; i++)
  {
    for (int i = 0; i < CombinedActions.size(); i++)
    {
      switch (CombinedActions[i].action_type)
      {
      case 0: // fold
        Action_Fold(z, i);
        break;
      case 1: //rotation
        Action_Rotate(z, i);
        break;
      case 2: //scale
        Action_Scale(z, i);
        break;
      case 3: //translation
        Action_Translate(z, i);
        break;
      }
    }

    z = Scale * (z - Eigen::Vector3f::Ones()) + Eigen::Vector3f::Ones();
    //z = z * Scale - Center * (Scale - 1);
    r = z.squaredNorm();
  }

  return (std::sqrt(r) - 2.f) * std::pow(Scale, -i);
}

bool Fractal::Action_Fold_Color(Eigen::Vector3f &p, int fold_index)
{
  float dot = p.dot(CombinedActions[fold_index].VecOp);
  if (dot < 0)
  {
    p -= dot * CombinedActions[fold_index].VecOp2;
    return true;
  }
  return false;
}

void Fractal::Action_Fold(Eigen::Vector3f &p, int fold_index)
{
  float dot = p.dot(CombinedActions[fold_index].VecOp);
  if (dot < 0)
    p -= dot * CombinedActions[fold_index].VecOp2;
}

void Fractal::Action_Rotate(Eigen::Vector3f &p, int rot_index)
{
  p = CombinedActions[rot_index].QuatOp._transformVector(p);
}

void Fractal::Action_Scale(Eigen::Vector3f &p, int scale_index)
{
  p = p.cwiseProduct(CombinedActions[scale_index].VecOp);
}

void Fractal::Action_Translate(Eigen::Vector3f &p, int trans_index)
{
  p = p + CombinedActions[trans_index].VecOp;
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


Eigen::Vector3f Fractal::FoldBased(Eigen::Vector3f _z)
{
  Eigen::Vector3f c(1.0, 1.0, 1.0);
  Eigen::Vector3f z = rot_inv._transformVector(_z) - Center;
  float r = z.squaredNorm();
  int i = 0;
  float w = 1;
  for (i = 0; i < num_subdivisions && r < bailout_dist; i++)
  {
    for (int action_num = 0; action_num < CombinedActions.size(); action_num++)
    {
      switch (CombinedActions[action_num].action_type)
      {
      case 0: // fold
        if (Action_Fold_Color(z, action_num))
          c = (1 - w) * c + w * CombinedActions[action_num].Color;
        break;
      case 1: //rotation
        Action_Rotate(z, action_num);
        break;
      case 2: //scale
        Action_Scale(z, action_num);
        break;
      case 3: //translation
        Action_Translate(z, action_num);
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

int Fractal::NumFolds()
{
  int ret = 0;
  for each (auto a in CombinedActions)
    if (a.action_type == 0)
      ret++;
  return ret;
}

void Fractal::GenColors()
{
  int num_folds = NumFolds();
  int current_color = 0;
  for (int i = 0; i < CombinedActions.size(); i++)
  {
    if (CombinedActions[i].action_type == 0)
    {
      float val = static_cast<float>(current_color) / static_cast<float>(num_folds);
      CombinedActions[i].Color = ColorFromFloat(val);
      current_color++;
      CombinedActions[i].VecOp2 = 2 * CombinedActions[i].VecOp;
    }
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

bool Shape::RenderGenericGUI(int shape_num)
{
  bool something_changed = false;
  if (ImGui::CollapsingHeader(name.data())) {
    ImGui::Indent(10.f);
    something_changed |= material->RenderGUI(shape_num);
    something_changed |= ImGui::InputFloat3((std::string("position") + std::to_string(shape_num)).data(), Position.data());
    something_changed |= this->RenderGUI(shape_num);
    ImGui::Unindent(10.f);
  }
  return something_changed;
}
