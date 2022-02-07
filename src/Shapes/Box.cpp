#include "Box.h"
#include "..\Interval.h"

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
  if (inter.t_0 > inter.t_1 || inter.t_1 < originErrorMargin)
    return;

  // intersection
  if (inter.t_0 > originErrorMargin)
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
}

void Box::ResetSettings()
{
  this->BoundingBox = Eigen::AlignedBox<float, 3>(
      base,
      base + extents);
  this->Position = base + extents / 2.f;
  this->SurfaceArea =
      2 * extents.x() * extents.y() +
      2 * extents.x() * extents.z() +
      2 * extents.y() * extents.z();
}

bool Box::RenderGUI(size_t i)
{
  bool changed = false;
  changed |= ImGui::DragFloat3((std::string("base##") + std::to_string(i)).data(), base.data(), 0.01f, -10000, 10000, "%.2f");
  changed |= ImGui::DragFloat3((std::string("radius##") + std::to_string(i)).data(), extents.data(), 0.01f, -10000, 10000, "%.2f");

  if (changed)
  {
    ResetSettings();
  }
  return changed;
}

std::string Box::Serialize()
{
  std::string ret = "box ";
  for (size_t i = 0; i < 3; i++)
  {
    ret += std::to_string(base[i]) + " ";
  }
  for (size_t i = 0; i < 3; i++)
  {
    ret += std::to_string(extents[i]) + " ";
  }
  return ret;
}

Shape *Box::Clone()
{
  Box *b = new Box(base, extents, material);
  return static_cast<Shape*>(b);
}

void Box::SetFromInterpolation(Shape *a, Shape *b, float t)
{
  Box *b0 = dynamic_cast<Box *>(a);
  Box *b1 = dynamic_cast<Box *>(b);
  base = b0->base * (1.f - t) + b1->base * t;
  extents = b0->extents * (1.f - t) + b1->extents * t;
  ResetSettings();
}
