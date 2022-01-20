#pragma once

class Shape;

class Intersection
{
public:
  Intersection()
    : t((std::numeric_limits<float>::max)())
    , object(nullptr)
    , P(Eigen::Vector3f::Zero())
    , N(Eigen::Vector3f(1.f, 0.f, 0.f))
    , uv(Eigen::Vector2f::Zero())
    , Kd(Eigen::Vector3f(-1.f, -1.f, -1.f))
    , ior_ratio(1.0f)
    , ior_in(1.0f)
    , ior_out(1.0f)
  {};
  Intersection(const Intersection &rhs)
    : t(rhs.t)
    , object(rhs.object)
    , P(rhs.P)
    , N(rhs.N)
    , uv(rhs.uv)
    , Kd(rhs.Kd)
    , ior_ratio(rhs.ior_ratio)
    , ior_in(rhs.ior_in)
    , ior_out(rhs.ior_out)
  {};
  void Calc_IOR_Ratio(Eigen::Vector3f w_o);

  ~Intersection() {};
  void Reset();
  float t;
  float ior_ratio, ior_in, ior_out;
  Shape *object;
  Eigen::Vector3f P, N;
  Eigen::Vector2f uv;
  Eigen::Vector3f Kd{ Eigen::Vector3f(-1, -1, -1) };
};

