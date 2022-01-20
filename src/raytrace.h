///////////////////////////////////////////////////////////////////////
// A framework for a raytracer.
////////////////////////////////////////////////////////////////////////

#pragma once

class Shape;
#include "Camera.h"
#include "Material.h"
#include "Shape.h";
#include "Minimizer.h"
#include "Intersection.h"
#include "ImageData.h"

//const float PI = 3.14159f;

////////////////////////////////////////////////////////////////////////
// Data structures for storing meshes -- mostly used for model files
// read in via ASSIMP.
//
// A MeshData holds two lists (stl::vector) one for vertices
// (VertexData: consisting of point, normal, texture, and tangent
// vectors), and one for triangles (TriData: consisting of three
// indices into the vertex array).
typedef Eigen::Matrix<unsigned int, 3, 1 > TriData;
    
class VertexData
{
 public:
    Vector3f pnt;
    Vector3f nrm;
    Vector2f tex;
    Vector3f tan;
    VertexData(const Vector3f& p, const Vector3f& n, const Vector2f& t, const Vector3f& a) 
        : pnt(p), nrm(n), tex(t), tan(a) 
    {}
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

class Scene {
public:
    int requested_width, requested_height;
    int gui_width{ 300 };
    Realtime *realtime{ nullptr };         // Remove this (realtime stuff)
    Material* currentMat;
    Camera camera;
    std::vector<Shape *> objects_p, lights_p;

    std::vector<Sphere> spheres;
    std::vector<Box> boxes;
    std::vector<Triangle> triangles;
    std::vector<Cylinder> cylinders;
    std::vector<Fractal> fractals;

    std::vector<Material> materials;
    std::vector<Light> lights;

    KdBVH<float, 3, Shape *> Tree;
    bool depth_of_field{ false };
    bool use_AA{ true };
    bool first_load{ true };
    bool halfDome{ false };

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

    Scene();
    void Finit();
    void ClearAll();
    // void ReCalcDirs();

    // The scene reader-parser will call the Command method with the
    // contents of each line in the scene file.
    void Command(const std::vector<std::string>& strings,
                 const std::vector<float>& f, bool hard);

    // To read a model file into the scene via ASSIMP, call ReadAssimpFile.  
    void ReadAssimpFile(const std::string& path, const Matrix4f& M);

    // Once ReadAssimpFile parses the information from the model file,
    // it will call:
    void triangleMesh(MeshData* mesh);

    // The main program will call the TraceImage method to generate
    // and return the image.  This is the Ray Tracer!
    float TraceImage(ImageData &id, bool update_pass, int n_threads);

    // Generates objects from mesh file
    void GenTris(MeshData *md);

    inline void MoveCamera(Eigen::Vector3f p) {
      camera.Move(p);
    };
    inline void RotateCamera(Eigen::Quaternionf r) {
      camera.Rotate(r);
    };
    inline void ChangeFOV(float w, float f) {
      camera.ChangeView(w, f);
    };

    void SetRayDirect(ImageData &id, Ray &r, int x, int y);
    void SetRayAA(ImageData &id, Ray &r, int x, int y);
    void SetRayDOF(ImageData &id, Ray &r, int x, int y);
    void SetRayHalfDome(ImageData &id, Ray &r, int x, int y);

    Color BVHTracePath(Ray &r, Minimizer &minimizer, bool option);
    Color BVHTraceDebug(Ray &r, Minimizer &minimizer, DEBUG_MODE m);
    void SampleLight(Intersection &I);
    Eigen::Vector3f GetBeers(const float t, Eigen::Vector3f Kt);
    Eigen::Vector3f EvalScattering(Eigen::Vector3f &w_o, Eigen::Vector3f &N, Eigen::Vector3f &w_i, Intersection &s);
    float PdfBRDF(Eigen::Vector3f &w_o, Eigen::Vector3f &N, Eigen::Vector3f &w_i, Intersection &s);
    Eigen::Vector3f EvalRadiance(Intersection &Q);
    Eigen::Vector3f SampleBRDF(Eigen::Vector3f &w_o, Eigen::Vector3f &N, Intersection &s);
    Eigen::Vector3f  SampleLobe(Eigen::Vector3f &N, float r1, float r2);
    float PdfLight(Shape *L);
    float GeometryFactor(Intersection &P, Intersection &L);
    Intersection &FireRayIntoScene(Minimizer &m, Eigen::Vector3f &direction);


};
