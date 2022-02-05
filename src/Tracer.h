///////////////////////////////////////////////////////////////////////
// A framework for a raytracer.
////////////////////////////////////////////////////////////////////////

#pragma once

class Shape;
#include "Camera.h"
#include "Material.h"
#include "Minimizer.h"
#include "Intersection.h"
#include "ImageData.h"

// shapes
#include "Shapes/Shape.h"
#include "Shapes/Sphere.h"
#include "Shapes/Box.h"
#include "Shapes/Cylinder.h"
#include "Shapes/Fractal.h"

class Tracer
{
public:
  int requested_width, requested_height;
  Material *currentMat;
  Camera camera;
  std::vector<Shape *> objects_p, lights_p;

  std::vector<Sphere> spheres;
  std::vector<Box> boxes;
  std::vector<Cylinder> cylinders;
  std::vector<Fractal> fractals;

  std::vector<Material> materials;
  std::vector<Light> lights;

  std::unordered_map<Material *, std::vector<Shape *>> shapes_by_material;

  Eigen::KdBVH<float, 3, Shape *> Tree;
  bool first_load{ true };
  bool isPaused{ false };

  enum TRACE_MODE
  {
    FULL,
    NORMAL,
    DEPTH,
    DIFFUSE,
    SIMPLE,
    POSITION,
    PATH_RATIO,
    NUM_MODES
  };

  TRACE_MODE DefaultMode = TRACE_MODE::SIMPLE;

  Tracer();
  void Finit();
  void ClearAll();

  // The scene reader-parser will call the Command method with the
  // contents of each line in the scene file.
  void Command(const std::vector<std::string> &strings,
      const std::vector<float> &f);

  // tracing functions
  float TraceImage(ImageData &id, bool update_pass, int n_threads);
  void SinglePixelInfoTrace(ImageData &id, int x, int y);
  std::string SinglePixelDebugTrace(ImageData &id, int x, int y);

  // ray direction selection vars/functions
  bool depth_of_field{ false };
  bool use_AA{ true };
  bool halfDome{ false };
  void SetRayDirect(ImageData &id, Ray &r, int x, int y);
  void SetRayAA(ImageData &id, Ray &r, int x, int y);
  void SetRayDOF(ImageData &id, Ray &r, int x, int y);
  void SetRayHalfDome(ImageData &id, Ray &r, int x, int y);

  // tracer support functions
  Color BVHTracePath(Ray &r, Minimizer &minimizer, bool option);
  Color BVHTraceDebug(Ray &r, Minimizer &minimizer, TRACE_MODE m);
  void SampleLight(Intersection &I);
  Eigen::Vector3f GetBeers(const float t, Eigen::Vector3f Kt);
  Eigen::Vector3f EvalScattering(Eigen::Vector3f &w_o, Eigen::Vector3f &N, Eigen::Vector3f &w_i, Intersection &s);
  float PdfBRDF(Eigen::Vector3f &w_o, Eigen::Vector3f &N, Eigen::Vector3f &w_i, Intersection &s);
  Eigen::Vector3f EvalRadiance(Intersection &Q);
  Eigen::Vector3f SampleBRDF(Eigen::Vector3f &w_o, Eigen::Vector3f &N, Intersection &s);
  Eigen::Vector3f SampleLobe(Eigen::Vector3f &N, float r1, float r2);
  float PdfLight(Shape *L);
  float GeometryFactor(Intersection &P, Intersection &L);
  Intersection &FireRayIntoScene(Minimizer &m, Eigen::Vector3f &direction);

  // vars for mouse over info
  Ray InfoRay;
  Minimizer InfoMinimizer;
  float info_dist;
  std::string info_name;
  Eigen::Vector3f info_pos;
};
