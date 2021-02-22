//////////////////////////////////////////////////////////////////////
// Provides the framework for a raytracer.
////////////////////////////////////////////////////////////////////////

#include <vector>

#ifdef _WIN32
    // Includes for Windows
#include <windows.h>
#include <cstdlib>
#include <limits>
#include <crtdbg.h>
#else
    // Includes for Linux
#endif

#include "geom.h"
#include "raytrace.h"
#include "realtime.h"
#include "Shape.h"
#include "Ray.h"
#include <Eigen_unsupported/Eigen/src/BVH/BVAlgorithms.h>
#include "Eulers.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <chrono>
#include <stack>
#include "random.h"

// A good quality *thread-safe* Mersenne Twister random number generator.
#include <random>
std::random_device device;
std::mt19937_64 RNGen(device());
std::uniform_real_distribution<> myrandom(0.0, 1.0);
// Call myrandom(RNGen) to get a uniformly distributed random number in [0,1].


Scene::Scene() : depth_of_field(false)
{
}

void Scene::Finit()
{
  std::srand(time(NULL));

  for (int i = 0; i < 1000; i++)
  {
    float rr = 1 - randf() * randf();
    float rg = 1 - randf() * randf();
    float rb = 1 - randf() * randf();
    //currentMat = new Material(Vector3f(rr, rg, rb), Vector3f(rr, rg, rb), 1);
    float rx = randf() * 6.f - 3.f;
    float ry = randf() * 6.f - 3.f;
    float rz = randf() * 6.f - 3.f;
    //objects.push_back(new Sphere(0.03f, Vector3f(rx, ry, rz), currentMat));
  }

  objects_p.clear();
  lights_p.clear();
  for (int i = 0; i < spheres.size(); i++)
    objects_p.push_back(static_cast<Shape *>(&(spheres[i])));
  for (int i = 0; i < boxes.size(); i++ )
    objects_p.push_back(&(boxes[i]));
  for (int i = 0; i < triangles.size(); i++)
    objects_p.push_back(&(triangles[i]));
  for (int i = 0; i < cylinders.size(); i++)
    objects_p.push_back(&(cylinders[i]));
  for (int i = 0; i < fractals.size(); i++)
  {
    fractals[i].GenColors();
    objects_p.push_back(&(fractals[i]));
  }
  for (int i = 0; i < objects_p.size(); i++)
    if (objects_p[i]->material->isLight())
      lights_p.push_back(objects_p[i]);

  Tree = KdBVH<float, 3, Shape *>(objects_p.begin(), objects_p.end());

}

void Scene::triangleMesh(MeshData *mesh)
{
  //realtime->triangleMesh(mesh);
  GenTris(mesh);
}

Quaternionf Orientation(int i,
  const std::vector<std::string> &strings,
  const std::vector<float> &f)
{
  Quaternionf q(1, 0, 0, 0); // Unit quaternion
  while (i < strings.size()) {
    std::string c = strings[i++];
    if (c == "x")
      q *= angleAxis(f[i++] * Radians, Vector3f::UnitX());
    else if (c == "y")
      q *= angleAxis(f[i++] * Radians, Vector3f::UnitY());
    else if (c == "z")
      q *= angleAxis(f[i++] * Radians, Vector3f::UnitZ());
    else if (c == "q") {
      q *= Quaternionf(f[i + 0], f[i + 1], f[i + 2], f[i + 3]);
      i += 4;
    }
    else if (c == "a") {
      q *= angleAxis(f[i + 0] * Radians, Vector3f(f[i + 1], f[i + 2], f[i + 3]).normalized());
      i += 4;
    }
  }
  return q;
}

////////////////////////////////////////////////////////////////////////
// Material: encapsulates surface properties
void Material::setTexture(const std::string path)
{
  int width, height, n;
  stbi_set_flip_vertically_on_load(true);
  unsigned char *image = stbi_load(path.c_str(), &width, &height, &n, 0);

  // Realtime code below:  This sends the texture in *image to the graphics card.
  // The raytracer will not use this code (nor any features of OpenGL nor the graphics card).
  glGenTextures(1, &texid);
  glBindTexture(GL_TEXTURE_2D, texid);
  glTexImage2D(GL_TEXTURE_2D, 0, n, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, image);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 100);
  glGenerateMipmap(GL_TEXTURE_2D);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, (int)GL_LINEAR);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, (int)GL_LINEAR_MIPMAP_LINEAR);
  glBindTexture(GL_TEXTURE_2D, 0);

  stbi_image_free(image);
}

static const float ToRad_f = 3.14159f / 180.f;

void Scene::Command(const std::vector<std::string> &strings,
  const std::vector<float> &f)
{
  if (strings.size() == 0) return;
  std::string c = strings[0];

  if (c == "screen") {
    // syntax: screen width height
    if (first_load)
    {
      realtime = new Realtime(int(f[1]), int(f[2]));
      realtime->setScreen(int(f[1]), int(f[2]));
      width = int(f[1]);
      height = int(f[2]);
      first_load = false;
    }
  }

  else if (c == "camera") {
    // syntax: camera x y z   ry   <orientation spec>
    // Eye position (x,y,z),  view orientation (qw qx qy qz),  frustum height ratio ry

    if (first_camera)
    {
      Eigen::Quaternionf rot = EulerToQuat(Eigen::Vector3f(f[5], f[6], f[7]));

      //realtime->setCamera(Vector3f(f[1], f[2], f[3]), Orientation(5, strings, f), f[4]);
      camera.SetProperties(rot, Vector3f(f[1], f[2], f[3]), f[4], width, height, f[8], f[9]);
      first_camera = false;
    }
    camera.w = f[8];
    camera.f = f[9];
  }

  else if (c == "ambient") {
    // syntax: ambient r g b
    // Sets the ambient color.  Note: This parameter is temporary.
    // It will be ignored once your raytracer becomes capable of
    // accurately *calculating* the true ambient light.
    //realtime->setAmbient(Vector3f(f[1], f[2], f[3]));
  }

  else if (c == "brdf") {
    // syntax: brdf  r g b   r g b  alpha
    // later:  brdf  r g b   r g b  alpha  r g b ior
    // First rgb is Diffuse reflection, second is specular reflection.
    // third is beer's law transmission followed by index of refraction.
    // Creates a Material instance to be picked up by successive shapes
    materials.push_back(Material(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7]));
    currentMat = &(materials[materials.size() - 1]);
  }

  else if (c == "light") {
    // syntax: light  r g b   
    // The rgb is the emission of the light
    // Creates a Material instance to be picked up by successive shapes
    lights.push_back(Light(Vector3f(f[1], f[2], f[3]), true));
    currentMat = &(lights[lights.size() - 1]);
  }

  else if (c == "sphere") {
    // syntax: sphere x y z   r
    // Creates a Shape instance for a sphere defined by a center and radius
    //realtime->sphere(Vector3f(f[1], f[2], f[3]), f[4], currentMat);
    spheres.push_back(Sphere(f[4], Vector3f(f[1], f[2], f[3]), currentMat));
  }

  else if (c == "box") {
    // syntax: box bx by bz   dx dy dz
    // Creates a Shape instance for a box defined by a corner point and diagonal vector
    //realtime->box(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), currentMat);
    boxes.push_back(Box(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), currentMat));
  }

  else if (c == "cylinder") {
    // syntax: cylinder bx by bz   ax ay az  r
    // Creates a Shape instance for a cylinder defined by a base point, axis vector, and radius
    //realtime->cylinder(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7], currentMat);
    cylinders.push_back(Cylinder(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7], currentMat));
  }


  else if (c == "mesh") {
    // syntax: mesh   filename   tx ty tz   s   <orientation>
    // Creates many Shape instances (one per triangle) by reading
    // model(s) from filename. All triangles are rotated by a
    // quaternion (qw qx qy qz), uniformly scaled by s, and
    // translated by (tx ty tz) .
    Matrix4f modelTr = translate(Vector3f(f[2], f[3], f[4]))
      * scale(Vector3f(f[5], f[5], f[5]))
      * toMat4(Orientation(6, strings, f));
    ReadAssimpFile(strings[1], modelTr);
  }

  else if (c == "fractal") {
    // syntax: fractal x y z   s
    // Creates a Fractal instance for a fractal defined by at x,y,z with scale s
    Eigen::Quaternionf rot = EulerToQuat(Eigen::Vector3f(f[5], f[6], f[7]));
    fractals.push_back(Fractal(f[4], Vector3f(f[1], f[2], f[3]), rot, currentMat));
    Fractal *fr = &fractals[fractals.size() - 1];
    fr->SetRecursionProperties(f[8], f[9], f[10]);

    int current_index = 11;
    while (current_index < f.size())
    {
      switch (static_cast<int>(f[current_index]))
      {
      case 0: // fold
        fr->Folds.push_back(Eigen::Vector3f(f[current_index + 1], f[current_index + 2], f[current_index + 3]).normalized());
        current_index += 4;
        fr->ActionToColor.push_back(fr->num_folds);
        fr->num_folds++;
        fr->ActionIndexes.push_back(0);
        break;
      case 1: //rotation
        fr->Rotations.push_back(EulerToQuat(Eigen::Vector3f(f[current_index + 1], f[current_index + 2], f[current_index + 3])));
        current_index += 4;
        fr->ActionToColor.push_back(0);
        fr->ActionIndexes.push_back(1);
        break;
      case 2: //scale
        fr->Scales.push_back(Eigen::Vector3f(f[current_index + 1], f[current_index + 2], f[current_index + 3]));
        current_index += 4;
        fr->ActionToColor.push_back(0);
        fr->ActionIndexes.push_back(2);
        break;
      case 3: //translation
        fr->Translates.push_back(Eigen::Vector3f(f[current_index + 1], f[current_index + 2], f[current_index + 3]));
        current_index += 4;
        fr->ActionToColor.push_back(0);
        fr->ActionIndexes.push_back(3);
        break;
      }
    }
  }


  else {
    fprintf(stderr, "\n*********************************************\n");
    fprintf(stderr, "* Unknown command: %s\n", c.c_str());
    fprintf(stderr, "*********************************************\n\n");
  }
}
std::chrono::steady_clock::time_point prior, current;

void tick(std::string msg, bool report = false)
{
  current = std::chrono::high_resolution_clock::now();
  float micro = static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(current - prior).count());
  if (report)
  {
    std::cout << msg << " @ " << micro / 1000000.f << " s/render" << std::endl;
  }
  prior = current;
}


void Scene::TraceImage(Color *image, const int pass, bool update_pass)
{
  if (update_pass)
    tick("start");

  float diff = 0;
  float weight = 1.f / static_cast<float>(pass);

#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
  for (int y = 0; y < height; y++) {

    Ray r(Eigen::Vector3f::Ones(), Eigen::Vector3f::Ones());
    Minimizer minimizer(r);

    for (int x = 0; x < width; x++) {
      if (depth_of_field)
        SetRayDOF(r, x, y);
      else
        SetRayAA(r, x, y);

      int pos = y * width + x;
      Color old = image[pos];
      if (DefaultMode == Scene::DEBUG_MODE::NONE)
        image[pos] = (1 - weight) * old + weight * BVHTracePath(r, minimizer, false);
      else
        image[pos] = (1 - weight) * old + weight * BVHTraceDebug(r, minimizer, DefaultMode);

      if (update_pass)
        diff += std::abs(old.x() - image[pos].x()) + std::abs(old.y() - image[pos].y()) + std::abs(old.z() - image[pos].z());
      
    }
  }

  if (update_pass)
    tick("Convergence: " + std::to_string(diff), true);

}


void Scene::GenTris(MeshData *md)
{
  for (auto t : md->triangles)
  {
    triangles.push_back(Triangle(md->vertices[t[0]].pnt, md->vertices[t[1]].pnt, md->vertices[t[2]].pnt, currentMat));
    Triangle *_t = &triangles[triangles.size() - 1];
    _t->SetNormals(md->vertices[t[0]].nrm, md->vertices[t[1]].nrm, md->vertices[t[2]].nrm);
    _t->SetTextureUV(md->vertices[t[0]].tex, md->vertices[t[1]].tex, md->vertices[t[2]].tex);
  }
}

void Scene::SetRayDirect(Ray &r, int x, int y)
{
  float dy = 2 * (y + 0.5f) / height - 1;
  float dx = 2 * (x + 0.5f) / width - 1;

  r.direction = (dx * camera.ViewX + dy * camera.ViewY + camera.ViewZ).normalized();
  r.origin = camera.position;
}

void Scene::SetRayAA(Ray &r, int x, int y)
{
  float dy = 2 * (y + randf()) / height - 1;
  float dx = 2 * (x + randf()) / width - 1;

  r.direction = (dx * camera.ViewX + dy * camera.ViewY + camera.ViewZ).normalized();
  r.origin = camera.position;
}

void Scene::SetRayDOF(Ray &r, int x, int y)
{
  //float rad = w * std::sqrt(-2 * std::log(randf())) * std::cos(pi_2 * randf());
  //float rad = w * std::sqrt(randf());
  float rad = camera.w * randf();
  //float rad = randf() + randf();
  //rad = rad > 1.f ? w * (2.f - rad) : w * rad;
  float theta = pi_2 * randf();
  float dx = rad * std::cos(theta);
  float dy = rad * std::sin(theta);
  Eigen::Vector3f E = camera.position + dx * camera.ViewX + dy * camera.ViewY;

  dy = 2 * (y + randf()) / height - 1;
  dx = 2 * (x + randf()) / width - 1;
  Eigen::Vector3f P = camera.position + camera.f * dx * camera.ViewX + camera.f * dy * camera.ViewY + camera.f * camera.ViewZ;

  r.direction = (P - E).normalized();
  r.origin = E;
}

Color Scene::BVHTraceDebug(Ray &r, Minimizer &minimizer, DEBUG_MODE mode)
{
  Eigen::Vector3f color(0.001, 0.001, 0.001);
  minimizer.closest_int.Reset();

  Eigen::BVMinimize(Tree, minimizer);

  if (minimizer.closest_int.object != nullptr)
  {
    if (mode == DEBUG_MODE::SIMPLE)
      for (auto l : lights_p)
        color += (std::max)(0.0f, minimizer.closest_int.N.dot((l->Position - minimizer.closest_int.object->Position).normalized())) * (minimizer.closest_int.Kd.x() > 0 ? minimizer.closest_int.Kd : minimizer.closest_int.object->material->Kd);
    else if (mode == DEBUG_MODE::NORMAL)
      color = Eigen::Vector3f(std::abs(minimizer.closest_int.N.x()), std::abs(minimizer.closest_int.N.y()), std::abs(minimizer.closest_int.N.z()));
    else if (mode == DEBUG_MODE::DEPTH)
      color = minimizer.closest_int.t * Eigen::Vector3f (1.f, 1.f / 10.f, 1.f / 100.f);
    else if (mode == DEBUG_MODE::DIFFUSE)
      color = minimizer.closest_int.Kd.x() > 0 ? minimizer.closest_int.Kd : minimizer.closest_int.object->material->Kd;
  }
  return color;
}


//assumes ray origin is set
Intersection &Scene::FireRayIntoScene(Minimizer &m, Eigen::Vector3f &direction)
{
  m.ray.direction = direction;
  m.closest_int.Reset();
  Eigen::BVMinimize(Tree, m);
  return m.closest_int;
}

Color Scene::BVHTracePath(Ray &r, Minimizer &minimizer, bool option)
{

  Eigen::Vector3f color(0.001, 0.001, 0.001);
  Eigen::Vector3f weight(1,1,1);

  //initial
  minimizer.closest_int.Reset();
  Eigen::BVMinimize(Tree, minimizer);
  if (minimizer.closest_int.object == nullptr)
    return color;
  Intersection P = minimizer.closest_int;
  float RussianRoulette = 0.8f;
  Eigen::Vector3f N = P.N;
  Intersection L;
  Eigen::Vector3f w_i;
  float p;

  while (randf() < RussianRoulette)
  {
    // always starts here
    r.origin = P.P;

    // explicit light
    SampleLight(L);
    p = PdfLight(L.object) / GeometryFactor(P, L);

    //check if p is positive, not necessary?

    w_i = (L.P - P.P).normalized();
    Intersection &I = FireRayIntoScene(minimizer, w_i);
    if (I.object == L.object)
    {
      Eigen::Vector3f rad = EvalRadiance(L);
      Eigen::Vector3f addon = 0.5f * weight.cwiseProduct(EvalScattering(N, w_i, P)).cwiseProduct(rad) / p;
      addon = Eigen::Vector3f((std::min)(addon.x(), rad.x()), (std::min)(addon.y(), rad.y()), (std::min)(addon.z(), rad.z()));
      color += addon;
    }


    //extend path
    N = P.N;
    w_i = SampleBRDF(N);
    Intersection &Q = FireRayIntoScene(minimizer, w_i);

    //if intersection doesn't exist, break
    if (Q.object == nullptr)
      break;

    Eigen::Vector3f f = EvalScattering(N, w_i, P);
    p = PdfBRDF(N, w_i) * RussianRoulette;

    if (p < 0.0001f)
      break;

    weight = weight.cwiseProduct(f / p);

    //light connection
    if (Q.object->material->isLight())
    {
      // after light
      color += 0.5 * weight.cwiseProduct(EvalRadiance(Q));
      break;
    }
    P = Q;
  }

  float mx = 5.f;

  //cap color
  float max_channel = (std::max)(color.x(), (std::max)(color.y(), color.z()));
  if (max_channel > mx)
    color *= mx / max_channel;
  return color;
}

void Scene::SampleLight(Intersection &I)
{
  int light_index = rand() % lights_p.size();
  lights_p[light_index]->GetRandomPointOn(I);
}

Eigen::Vector3f Scene::EvalScattering(Eigen::Vector3f &N, Eigen::Vector3f &w_i, Intersection &i)
{
  Eigen::Vector3f color = i.Kd.x() > 0 ? i.Kd : i.object->material->Kd;
  return std::abs(N.dot(w_i)) * color / pi;
}

Eigen::Vector3f Scene::SampleLobe(Eigen::Vector3f &N, float c, float phi)
{
  float s = std::sqrt(1 - c * c);
  Eigen::Vector3f K(s * std::cos(phi), s * std::sin(phi), c);
  Eigen::Quaternionf q = Eigen::Quaternionf::FromTwoVectors(Vector3f::UnitZ(), N);
  return q._transformVector(K);
}

float Scene::GeometryFactor(Intersection &P, Intersection &L)
{
  Eigen::Vector3f D = P.P - L.P;
  float D_D = D.dot(D);
  return std::abs(P.N.dot(D) * L.N.dot(D) / (D_D * D_D));
}

float Scene::PdfLight(Shape *L)
{
  return 1.f / (L->SurfaceArea * static_cast<float>(lights_p.size()));
}

float Scene::PdfBRDF(Eigen::Vector3f &N, Eigen::Vector3f &w_i)
{
  return std::abs(N.dot(w_i)) / pi;
}

Eigen::Vector3f Scene::EvalRadiance(Intersection &Q)
{
  return Q.object->material->Kd;
}

Eigen::Vector3f Scene::SampleBRDF(Eigen::Vector3f &N)
{
  float r1 = randf();
  float r2 = randf();
  return SampleLobe(N, std::sqrt(r1), pi_2 * r2);
}

Eigen::AlignedBox<float, 3> bounding_box(const Shape *obj)
{
  return obj->BoundingBox; // Assuming each Shape object has its own bbox method.
};

void Scene::MedianFilter(Color *noisy_image, Color *filtered_image)
{
#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int pos = y * width + x;
      //scan neighborhood, calculate med
      filtered_image[pos] = ScanNeighborhood(noisy_image, x, y, 3);
    }
  }
}

int clamp(int val, int min, int max)
{
  return (std::min)(max, (std::max)(min, val));
}

Eigen::Vector3f Scene::ScanNeighborhood(Color *image, int x, int y, int w)
{
  std::vector<std::priority_queue<float>> left(3);
  std::vector<std::priority_queue <float, std::vector<float>, std::greater<float> >> right(3);
  for (int i = clamp(y - w, 0, height); i < clamp(y + w + 1, 0, height); i++)
  {
    for (int j = clamp(x - w, 0, width); j < clamp(x + w + 1, 0, width); j++)
    {
      int pos = i * width + j;
      Color c = image[pos];
      for (int k = 0; k < 3; k++)
      {
        float val = c[k];
        if (left[k].size() == 0 || val < left[k].top())
        {
          left[k].push(val);
          if (left[k].size() - right[k].size() > 1)
          {
            right[k].push(left[k].top());
            left[k].pop();
          }
        }
        else
        {
          right[k].push(val);
          if (right[k].size() - left[k].size() > 0)
          {
            left[k].push(right[k].top());
            right[k].pop();
          }
        }
      }
    }
  }
  return Eigen::Vector3f(left[0].top(), left[1].top(), left[2].top());
}

void Scene::LocalReduction(Color *noisy_image, Color *filtered_image)
{
  float expVariance = 0.0001;

#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
  for (int y = 0; y < height; y++)
  {
    Eigen::Vector3f mean, variance;
    for (int x = 0; x < width; x++)
    {
      int pos = y * width + x;
      ScanNeighborhoodV(noisy_image, x, y, 3, mean, variance);
      Eigen::Vector3f val = noisy_image[pos];
      for (int k = 0; k < 3; k++)
      {
        if (variance[k] < .001f)
          filtered_image[pos][k] = val[k];
        else
          filtered_image[pos][k] = clamp(val[k] - (expVariance / variance[k]) * (val[k] - mean[k]), 0, 10);
      }
    }
  }
}

void Scene::ScanNeighborhoodV(Color *image, int x, int y, int w, Eigen::Vector3f &mean, Eigen::Vector3f &variance)
{
  mean = Eigen::Vector3f::Zero();
  variance = Eigen::Vector3f::Zero();
  int scanned = 0;
  for (int i = clamp(y - w, 0, height); i < clamp(y + w + 1, 0, height); i++)
  {
    for (int j = clamp(x - w, 0, width); j < clamp(x + w + 1, 0, width); j++)
    {
      int pos = y * width + x;
      scanned++;
      Eigen::Vector3f val = image[pos];
      for (int k = 0; k < 3; k++)
      {
        mean[k] += val[k];
        variance[k] += val[k] * val[k];
      }
    }
  }

  mean = mean / scanned;
  variance = (variance / scanned) - mean.cwiseProduct(mean);

}
