#pragma once

static const float pi = 3.1415926535897932384626433832795028841971693993751058209749445923078164062f;
static const float pi_2 = pi * 2.f;
static const float ToRad = pi / 180.f;
static const float ToDeg = 180.f / pi;

static Eigen::Vector3f QuatToEuler(Eigen::Quaternionf q) 
{
  return q.toRotationMatrix().eulerAngles(0, 1, 2) * ToDeg;
};

static Eigen::Quaternionf EulerToQuat(Eigen::Vector3f e)
{
  Eigen::Quaternionf rot =
      Eigen::AngleAxisf(e[0] * ToRad, Eigen::Vector3f::UnitX())
    * Eigen::AngleAxisf(e[1] * ToRad, Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(e[2] * ToRad, Eigen::Vector3f::UnitZ());
  rot.normalize();
  return rot;
}


//return relative luminance of a color
static float Luminance(Eigen::Vector3f &c)
{
  return (std::min)(1.f, 0.2126f * c.x() + 0.7152f * c.y() + 0.0722f * c.z());
}