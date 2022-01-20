#pragma once
const float PI = 3.14159f;

////////////////////////////////////////////////////////////////////////
// Material: encapsulates a BRDF and communication with a shader.
////////////////////////////////////////////////////////////////////////
class Material
{
public:
  Eigen::Vector3f Kd, Ks, Kt;
  float alpha, specularity, ior, translucency;
  unsigned int texid;

  virtual bool isLight() { return _isLight; }
  virtual bool RenderGUI(int shape_num);

  Material() : Kd(Eigen::Vector3f(1.0, 0.5, 0.0)), Ks(Eigen::Vector3f(1, 1, 1)), Kt(Eigen::Vector3f(1, 1, 1)), alpha(1.0), specularity(0.0f), ior(1.0), translucency(0.0), texid(0), _isLight(false) {}
  Material(const Eigen::Vector3f d, const Eigen::Vector3f s, const float a, const float sp)
    : Kd(d), Ks(s), alpha(a), specularity(sp), ior(1.0), translucency(0.0), Kt(Eigen::Vector3f(1, 1, 1)), texid(0), _isLight(false) {
    NormalizeProbabilities();
  }
  Material(const Eigen::Vector3f d, const Eigen::Vector3f s, const float a, const float sp, const Eigen::Vector3f t, const float i, const float o)
    : Kd(d), Ks(s), alpha(a), specularity(sp), Kt(t), ior(i), translucency(o), texid(0), _isLight(false) {
    NormalizeProbabilities();
  }
  Material(Material &o) { Kd = o.Kd;  Ks = o.Ks; Kt = o.Kt; ior = o.ior; translucency = o.translucency; alpha = o.alpha; specularity = o.specularity; texid = o.texid; _isLight = o._isLight; }

  void setTexture(const std::string path);
  void NormalizeProbabilities()
  {
    float total = specularity + translucency;
    if (total > 1)
    {
      specularity /= total;
      translucency /= total;
    }
  }

  float D(Eigen::Vector3f h, Eigen::Vector3f N);
  Eigen::Vector3f F(float a);
  float G(Eigen::Vector3f w_i, Eigen::Vector3f w_o, Eigen::Vector3f h, Eigen::Vector3f N);
  float G_1(Eigen::Vector3f w, Eigen::Vector3f h, Eigen::Vector3f N);
  inline float Charictaristic(float d) {
    return d > 0 ? 1.0 : 0.0;
  };

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

  Light(const Eigen::Vector3f e, bool _isLightParm) : Material() { 
    light_value = e; 
    _isLight = _isLightParm; 
    alpha = 1;
    Kd = Eigen::Vector3f::Ones();
    Ks = Eigen::Vector3f::Ones();
  }
  virtual bool isLight() { return _isLight; }
  virtual bool RenderGUI(int shape_num);
  Eigen::Vector3f light_value;
};
