#pragma once

////////////////////////////////////////////////////////////////////////
// Material: encapsulates a BRDF and communication with a shader.
////////////////////////////////////////////////////////////////////////
class Material
{
public:
  Eigen::Vector3f Kd, Ks;
  float specularity;

  virtual bool isLight() { return _isLight; }
  virtual bool RenderGUI(size_t shape_num);

  Material() : Kd(Eigen::Vector3f(1.0, 0.5, 0.0)), Ks(Eigen::Vector3f(1, 1, 1)), specularity(0.0f), _isLight(false) {}
  Material(const Eigen::Vector3f d, const Eigen::Vector3f s, const float sp)
      : Kd(d), Ks(s), specularity(sp), _isLight(false) {}
  Material(Material &o)
  {
    Kd = o.Kd;
    Ks = o.Ks;
    specularity = o.specularity;
    _isLight = o._isLight;
  }

  float D(Eigen::Vector3f h, Eigen::Vector3f N);
  Eigen::Vector3f F(float a);
  float G(Eigen::Vector3f w_i, Eigen::Vector3f w_o, Eigen::Vector3f h, Eigen::Vector3f N);
  float G_1(Eigen::Vector3f w, Eigen::Vector3f h, Eigen::Vector3f N);
  inline float Charictaristic(float d)
  {
    return d > 0 ? 1.f : 0.f;
  };

  virtual std::string Serialize();

protected:
  bool _isLight;
};

////////////////////////////////////////////////////////////////////////
// Light: encapsulates a light and communiction with a shader.
////////////////////////////////////////////////////////////////////////
class Light : public Material
{
public:
  Light(const Eigen::Vector3f e, bool _isLightParm) : Material()
  {
    light_value = e;
    _isLight = _isLightParm;
    specularity = 0;
    Kd = Eigen::Vector3f::Ones();
    Ks = Eigen::Vector3f::Ones();
  }
  virtual bool isLight() { return _isLight; }
  virtual bool RenderGUI(size_t shape_num);
  virtual std::string Serialize();
  Eigen::Vector3f light_value;
};
