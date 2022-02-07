#include "Sphere.h"

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
  if (t_1 < originErrorMargin)
    return;

  //intersection
  i.t = t_0 < originErrorMargin ? t_1 : std::min(t_0, t_1);
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
  float a = pi_2 * e2;
  I.N = Eigen::Vector3f(r * std::cos(a), r * std::sin(a), z).normalized();
  I.P = Center + Radius * I.N;
  I.object = this;
}

void Sphere::ResetSettings()
{
  this->BoundingBox = Eigen::AlignedBox<float, 3>(
      Center - Eigen::Vector3f::Ones() * Radius,
      Center + Eigen::Vector3f::Ones() * Radius);
  this->Position = Center;
  this->SurfaceArea = 4.f * pi * Radius * Radius;
}

bool Sphere::RenderGUI(size_t i)
{
  bool changed = false;
  changed |= ImGui::DragFloat((std::string("radius##") + std::to_string(i)).data(), &Radius, 0.1f, 0.0f, 10000.f, "%.1");
  changed |= ImGui::DragFloat3((std::string("radius##") + std::to_string(i)).data(), Center.data(), 0.01f, -10000, 10000, "%.2f");

  if (changed)
  {
    ResetSettings();
  }
  return changed;
}

std::string Sphere::Serialize()
{
  std::string ret = "sphere ";
  for (size_t i = 0; i < 3; i++)
  {
    ret += std::to_string(Center[i]) + " ";
  }
  ret += std::to_string(Radius) + " ";
  return ret;
}

Shape *Sphere::Clone()
{
  Sphere *s = new Sphere(Radius, Center, material);
  return static_cast<Shape*>(s);
}

void Sphere::SetFromInterpolation(Shape *a, Shape *b, float t)
{
  Sphere *s0 = dynamic_cast<Sphere *>(a);
  Sphere *s1 = dynamic_cast<Sphere *>(b);
  Center = s0->Center * (1.f - t) + s1->Center * t;
  Radius = s0->Radius * (1.f - t) + s1->Radius * t;
  ResetSettings();
}
