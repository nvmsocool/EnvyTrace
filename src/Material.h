#pragma once
const float PI = 3.14159f;

////////////////////////////////////////////////////////////////////////
// Material: encapsulates a BRDF and communication with a shader.
////////////////////////////////////////////////////////////////////////
class Material
{
public:
  Eigen::Vector3f Kd, Ks;
  float alpha;
  unsigned int texid;

  virtual bool isLight() { return _isLight; }

  Material() : Kd(Eigen::Vector3f(1.0, 0.5, 0.0)), Ks(Eigen::Vector3f(1, 1, 1)), alpha(1.0), texid(0), _isLight(false) {}
  Material(const Eigen::Vector3f d, const Eigen::Vector3f s, const float a)
    : Kd(d), Ks(s), alpha(a), texid(0), _isLight(false) {}
  Material(Material &o) { Kd = o.Kd;  Ks = o.Ks;  alpha = o.alpha;  texid = o.texid; _isLight = o._isLight; }

  void setTexture(const std::string path);

protected:
  bool _isLight;
  //virtual void apply(const unsigned int program);
};

////////////////////////////////////////////////////////////////////////
// Light: encapsulates a light and communiction with a shader.
////////////////////////////////////////////////////////////////////////
class Light : public Material
{
public:

  Light(const Eigen::Vector3f e, bool _isLightParm) : Material() { Kd = e; _isLight = _isLightParm; }
  virtual bool isLight() { return _isLight; }
  //virtual void apply(const unsigned int program);
};
