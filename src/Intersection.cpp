#include "Intersection.h"

void Intersection::Reset()
{
  t = (std::numeric_limits<float>::max)();
  object = nullptr;
  Kd = Eigen::Vector3f(-1.f, -1.f, -1.f);
}
