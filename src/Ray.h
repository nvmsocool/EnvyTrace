#pragma once
class Ray
{
public:
  Ray(Eigen::Vector3f _origin, Eigen::Vector3f _direction)
      : origin(_origin), direction(_direction)
  {
    direction.normalize();
  };
  ~Ray(){};

  Eigen::Vector3f origin, direction;

  inline Eigen::Vector3f eval(float t) const
  {
    return origin + t * direction;
  }
};
