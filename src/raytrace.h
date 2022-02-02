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

////////////////////////////////////////////////////////////////////////
// Data structures for storing meshes -- mostly used for model files
// read in via ASSIMP.
//
// A MeshData holds two lists (stl::vector) one for vertices
// (VertexData: consisting of point, normal, texture, and tangent
// vectors), and one for triangles (TriData: consisting of three
// indices into the vertex array).
typedef Eigen::Matrix<unsigned int, 3, 1> TriData;

class VertexData
{
public:
  Eigen::Vector3f pnt;
  Eigen::Vector3f nrm;
  Eigen::Vector2f tex;
  Eigen::Vector3f tan;
  VertexData(const Eigen::Vector3f &p, const Eigen::Vector3f &n, const Eigen::Vector2f &t, const Eigen::Vector3f &a)
      : pnt(p), nrm(n), tex(t), tan(a) {}
};

struct MeshData
{
  std::vector<VertexData> vertices;
  std::vector<TriData> triangles;
  Material *mat;
};

////////////////////////////////////////////////////////////////////////////////
// Scene
class Realtime;

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

  enum DEBUG_MODE
  {
    NONE,
    NORMAL,
    DEPTH,
    DIFFUSE,
    SIMPLE,
    POSITION,
    NUM_MODES
  };

  DEBUG_MODE DefaultMode = DEBUG_MODE::SIMPLE;

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
  Color BVHTraceDebug(Ray &r, Minimizer &minimizer, DEBUG_MODE m);
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

  // for progress bar
  float pixel_num;
};
