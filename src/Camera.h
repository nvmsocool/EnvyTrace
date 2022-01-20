#pragma once
class Camera
{
public:
  Camera() {};
  ~Camera() {};

  //position, rotation, ry
  float ry, rx;
  Eigen::Vector3f position, ViewX, ViewY, ViewZ;
  Eigen::Quaternionf rotation;
  float f = 5.f;
  float w = 0.2f;

  void SetProperties(Eigen::Quaternionf r, Eigen::Vector3f p, float _ry, float w, float h, float fov_amt, float fov_dist);
  void Move(Eigen::Vector3f p);
  void Rotate(Eigen::Quaternionf r);
  void ChangeView(float w, float f);
  void ResetViews();
  std::string GetCameraString();
  void PrintSettings();
  void UpdateFOV(float w, float h);

private:
  
};

