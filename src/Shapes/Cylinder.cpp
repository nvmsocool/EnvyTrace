#include "Cylinder.h"
#include "..\Interval.h"

void Cylinder::ResetSettings()
{

  Eigen::Vector3f minB = Base + Eigen::Vector3f::Ones() * radius;
  Eigen::Vector3f maxB = Base - Eigen::Vector3f::Ones() * radius;
  Eigen::Vector3f minA = Base + Axis + Eigen::Vector3f::Ones() * radius;
  Eigen::Vector3f maxA = Base + Axis - Eigen::Vector3f::Ones() * radius;

  this->BoundingBox = Eigen::AlignedBox<float, 3>(
      Eigen::Vector3f(
          (std::min)((std::min)(minB.x(), maxB.x()), (std::min)(minA.x(), maxA.x())),
          (std::min)((std::min)(minB.y(), maxB.y()), (std::min)(minA.y(), maxA.y())),
          (std::min)((std::min)(minB.z(), maxB.z()), (std::min)(minA.z(), maxA.z()))),
      Eigen::Vector3f(
          (std::max)((std::max)(minB.x(), maxB.x()), (std::max)(minA.x(), maxA.x())),
          (std::max)((std::max)(minB.y(), maxB.y()), (std::max)(minA.y(), maxA.y())),
          (std::max)((std::max)(minB.z(), maxB.z()), (std::max)(minA.z(), maxA.z()))));
  this->Position = Base + Axis / 2.f;
  this->SurfaceArea = 2.f * pi * radius * (Axis.norm() + radius);
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
  if (All.t_0 > All.t_1 || All.t_1 < originErrorMargin)
    return;

  //intersection
  i.object = this;
  if (All.t_0 > originErrorMargin)
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
}

bool Cylinder::RenderGUI(size_t i)
{
  bool changed = false;

  Eigen::Vector3f Base, Axis;
  float radius;

  changed |= ImGui::DragFloat3((std::string("Base##") + std::to_string(i)).data(), Base.data(), 0.01f, -10000, 10000, "%.2f");
  changed |= ImGui::DragFloat3((std::string("Axis##") + std::to_string(i)).data(), Axis.data(), 0.01f, -10000, 10000, "%.2f");
  changed |= ImGui::DragFloat((std::string("Radius##") + std::to_string(i)).data(), &radius, 0.01f, -10000, 10000, "%.2f");

  if (changed)
  {
    ResetSettings();
  }
  return changed;
}

Shape *Cylinder::Clone()
{
  Cylinder *c = new Cylinder(Base, Axis, radius, material);
  return static_cast<Shape*>(c);
}

void Cylinder::SetFromInterpolation(Shape *a, Shape *b, float t)
{
  Cylinder *c0 = dynamic_cast<Cylinder *>(a);
  Cylinder *c1 = dynamic_cast<Cylinder *>(b);
  Base = c0->Base * (1.f - t) + c1->Base * t;
  Axis = c0->Axis * (1.f - t) + c1->Axis * t;
  radius = c0->radius * (1.f - t) + c1->radius * t;
  ResetSettings();
}
