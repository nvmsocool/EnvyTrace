#include "Intersection.h"
#include "Shape.h"
#include "Material.h"

void Intersection::Calc_IOR_Ratio(Eigen::Vector3f w_o)
{
  if (N.dot(w_o) > 0)
  {
    ior_in = 1.0f;
    ior_out = object->material->ior;
  }
  else
  {
    ior_in = object->material->ior;
    ior_out = 1.0f;
  }
  ior_ratio = ior_in / ior_out;
}

void Intersection::Reset()
{
  t = (std::numeric_limits<float>::max)();
  object = nullptr;
  Kd = Eigen::Vector3f(-1.f, -1.f, -1.f);
}
