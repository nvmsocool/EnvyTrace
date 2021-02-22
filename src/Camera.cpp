#include "Camera.h"
#include <iostream>
#include "Eulers.h"

void Camera::SetProperties(Eigen::Quaternionf r, Eigen::Vector3f p, float _ry, float w, float h, float fov_amt, float fov_dist)
{
  ry = _ry;
  rx = ry * w / h;
  position = p;
  rotation = r;
  f = fov_dist;
  w = fov_amt;
  ResetViews();
}

void Camera::Move(Eigen::Vector3f p)
{
  position += p;
  PrintSettings();
}

void Camera::Rotate(Eigen::Quaternionf r)
{
  rotation *= r;
  ResetViews();
  PrintSettings();
}

void Camera::ChangeFOV(float _w, float _f)
{
  w = (std::max)(0.f, w + _w);
  f = (std::max)(0.f, f + _f);
  PrintSettings();
}

void Camera::ResetViews()
{
  ViewX = rx * rotation._transformVector(Eigen::Vector3f::UnitX());
  ViewY = ry * rotation._transformVector(Eigen::Vector3f::UnitY());
  ViewZ = -1.f * rotation._transformVector(Eigen::Vector3f::UnitZ());
}

void Camera::PrintSettings()
{
  //print last camera config, for copying into scene file
  //format:
  // camera pos.x pos.y pos.z ry rot.x, rot.y, rot.z
  auto euler = QuatToEuler(rotation);
  std::cout
    << std::to_string(position.x()) << " "
    << std::to_string(position.y()) << " "
    << std::to_string(position.z()) << " "
    << std::to_string(ry) << " "
    << std::to_string(euler.x()) << " "
    << std::to_string(euler.y()) << " "
    << std::to_string(euler.z()) << " "
    << std::to_string(w) << " "
    << std::to_string(f) << " "
    << std::endl;
}