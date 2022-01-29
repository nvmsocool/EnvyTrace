#pragma once
#include <unordered_map>

class Camera
{
public:
  Camera();
  ~Camera(){};

  //position, rotation, ry
  float ry, rx;
  Eigen::Vector3f position, displayRotation, ViewX, ViewY, ViewZ;
  Eigen::Quaternionf rotation;

  std::unordered_map<char*, Eigen::Vector3f> moveKeyAmts;
  std::unordered_map<char*, Eigen::Vector3f> rotKeyAmts;

  //camera movement settings
  float speedMove = 0.1f;
  float speedRot = 2.f;
  bool controlsEnabled = true;

  // depth of field settings
  float f = 5.f;
  float w = 0.2f;

  void SetProperties(Eigen::Quaternionf r, Eigen::Vector3f p, float _ry, float w, float h, float fov_amt, float fov_dist);
  void Move(Eigen::Vector3f p);
  void Rotate(Eigen::Vector3f r);
  void ResetViews();
  std::string GetCameraString();
  void UpdateFOV(float w, float h);
  bool Update();
  void PurgeKeys();

private:
};
