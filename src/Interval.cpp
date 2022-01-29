#include "Interval.h"

Interval::Interval(Eigen::Vector3f n, float d_0, float d_1, const Ray &in)
    : t_0(0), t_1(std::numeric_limits<float>().max())
{
  Intersect(n, d_0, d_1, in);
}

void Interval::Empty()
{
  t_0 = 0.f;
  t_1 = -1.f;
}

void Interval::Full()
{
  t_0 = 0.f;
  t_1 = std::numeric_limits<float>().max();
}

void Interval::Intersect(Interval i)
{
  if (i.t_0 > t_0)
  {
    t_0 = i.t_0;
    n_0 = i.n_0;
  }
  if (i.t_1 < t_1)
  {
    t_1 = i.t_1;
    n_1 = i.n_1;
  }
}

void Interval::Intersect(Eigen::Vector3f n, float d_0, float d_1, const Ray &in)
{
  float denom = n.dot(in.direction);
  float n_origin = n.dot(in.origin);
  if (denom == 0)
  {
    float s_0 = n_origin + d_0;
    float s_1 = n_origin + d_1;

    if ((s_0 < 0 && s_1 < 0) || (s_0 > 0 && s_1 > 0))
      Full();
    else
      Empty();
  }
  else
  {
    float t_a = -(d_0 + n_origin) / denom;
    float t_b = -(d_1 + n_origin) / denom;

    if (t_a < t_b)
    {
      t_0 = t_a;
      t_1 = t_b;
      n_0 = -n;
      n_1 = n;
    }
    else
    {
      t_0 = t_b;
      t_1 = t_a;
      n_0 = n;
      n_1 = -n;
    }
  }
}
