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
  std::srand(1234567);

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

  for (size_t i = 0; i < unique_pixels.size(); i++)
  {
    unique_pixels[i].clear();
    ray_directions[i].clear();
    unique_indexes[i].clear();
  }

  unique_pixels.clear();
  ray_directions.clear();
  exact_unique_pixel_counts.clear();
  unique_indexes.clear();

  unique_pixels.resize(height);
  ray_directions.resize(height);
  exact_unique_pixel_counts.resize(height);
  unique_indexes.resize(height);

  float even_half = height % 2 == 0 ? height / 2.f - 1.f : (height - 1.f) / 2.f;
  for (int i = 0; i < height; i++)
  {
    float a = std::floor(std::abs(i - (height - 1.f) / 2.f));
    float b = a / even_half;
    float c = std::sqrt(1 - b * b);
    float d = c * (width - 1) + 1;
    unique_pixels[i].resize(std::floor(d));
    ray_directions[i].resize(std::floor(d));
    unique_indexes[i].resize(width);
    exact_unique_pixel_counts[i] = d;

    for (int j = 0; j < width; j++)
    {
      unique_indexes[i][j] = (std::min)(static_cast<int>(std::floor(j * exact_unique_pixel_counts[i] / static_cast<float>(width))), static_cast<int>(unique_pixels[i].size()-1));
    }

  }

  ReCalcDirs();

}


void Scene::ReCalcDirs()
{
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < unique_pixels[i].size(); j++)
    {
      //degrees in from x axis
      float theta = (2.f * j + 1.f) * pi / static_cast<float>(2 * unique_pixels[i].size()) - (pi / 2);
      //degrees down from y axis
      float phi = (pi * i / height) - (pi / 2);

      Eigen::Quaternionf rot =
        Eigen::AngleAxisf(phi, camera.rotation._transformVector(Eigen::Vector3f::UnitX()))
        * Eigen::AngleAxisf(theta, camera.rotation._transformVector(Eigen::Vector3f::UnitY()));
      ray_directions[i][j] = rot._transformVector(camera.ViewZ).normalized();
    }
  }
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
      q *= angleAxis(f[i++] * ToRad, Vector3f::UnitX());
    else if (c == "y")
      q *= angleAxis(f[i++] * ToRad, Vector3f::UnitY());
    else if (c == "z")
      q *= angleAxis(f[i++] * ToRad, Vector3f::UnitZ());
    else if (c == "q") {
      q *= Quaternionf(f[i + 0], f[i + 1], f[i + 2], f[i + 3]);
      i += 4;
    }
    else if (c == "a") {
      q *= angleAxis(f[i + 0] * ToRad, Vector3f(f[i + 1], f[i + 2], f[i + 3]).normalized());
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

void Scene::Command(const std::vector<std::string> &strings,
  const std::vector<float> &f, bool hard)
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
    else if (width != f[1] || height != f[2])
    {
      width = int(f[1]);
      height = int(f[2]);
      realtime->RequestResize(width, height);

    }
  }

  else if (c == "camera") {
    // syntax: camera x y z   ry   <orientation spec>
    // Eye position (x,y,z),  view orientation (qw qx qy qz),  frustum height ratio ry

    if (hard)
    {
      Eigen::Quaternionf rot = EulerToQuat(Eigen::Vector3f(f[5], f[6], f[7]));

      //realtime->setCamera(Vector3f(f[1], f[2], f[3]), Orientation(5, strings, f), f[4]);
      camera.SetProperties(rot, Vector3f(f[1], f[2], f[3]), f[4], width, height, f[8], f[9]);
    }
    camera.w = f[8];
    camera.f = f[9];
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
    spheres.push_back(Sphere(f[4], Vector3f(f[1], f[2], f[3]), currentMat));
  }

  else if (c == "box") {
    // syntax: box bx by bz   dx dy dz
    // Creates a Shape instance for a box defined by a corner point and diagonal vector
    boxes.push_back(Box(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), currentMat));
  }

  else if (c == "cylinder") {
    // syntax: cylinder bx by bz   ax ay az  r
    // Creates a Shape instance for a cylinder defined by a base point, axis vector, and radius
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


float Scene::TraceImage(std::vector<Color> &image, const int pass, bool update_pass)
{
  if (update_pass)
    tick("start");

  float diff = 0;
  float weight = 1.f / static_cast<float>(pass);

#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
  for (int y = 0; y < height; y++) {

    Ray r(Eigen::Vector3f::Ones(), Eigen::Vector3f::Ones());
    Minimizer minimizer(r);
    int y_add = y * width;

    for (int x = 0; x < width; x++) {
      if (depth_of_field)
        SetRayDOF(r, x, y);
      else
        SetRayAA(r, x, y);

      int pos = y_add + x;
      Color old = image[pos];
      if (DefaultMode == Scene::DEBUG_MODE::NONE)
        image[pos] = (1 - weight) * old + weight * BVHTracePath(r, minimizer, false);
      else
        image[pos] = (1 - weight) * old + weight * BVHTraceDebug(r, minimizer, DefaultMode);

      if (update_pass)
        diff += std::abs(old.x() - image[pos].x()) + std::abs(old.y() - image[pos].y()) + std::abs(old.z() - image[pos].z());
      
    }
  }

  diff /= width * height;

  if (update_pass)
  {
    tick("Convergence: " + std::to_string(diff), true);
    return diff;
  }
  return 1;

}


float Scene::TraceHalfDome(std::vector<Color> &image, const int pass, bool update_pass)
{
  if (update_pass)
    tick("start");

  float diff = 0;
  float weight = 1.f / static_cast<float>(pass);



#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
  for (int y = 0; y < height; y++) {

    Ray r(camera.position, Eigen::Vector3f::Ones());
    Minimizer minimizer(r);

    int y_add = y * width;

    int halfway = width / 2;
    for (int x = 0; x < width; x++) {

      //create ray in that direction
      //r.direction = ray_directions[y][x];
      float theta = pi * (y + randf()) / height;
      float z_d = std::cos(theta);
      float sinTheta = std::sin(theta);
      float phi;
      if (x < halfway)
      {
        r.origin = camera.position;
        phi = pi * ((2.f * x) + 0.5f) / width;
      }
      else
      {
        r.origin = camera.position - camera.rotation._transformVector(Eigen::Vector3f(0.035, 0, 0));
        phi = pi * (2.f * (x- halfway) + randf()) / width;
      }
      float x_d = sinTheta * std::cos(phi);
      float y_d = sinTheta * std::sin(phi);
      r.direction = camera.rotation._transformVector(Eigen::Vector3f(x_d, y_d, z_d)).normalized();

      //move it a bit for AA

      int pos = y_add + x;
      Color old = image[pos];
      Color new_color;
      if (DefaultMode == Scene::DEBUG_MODE::NONE)
        new_color = BVHTracePath(r, minimizer, false);
      else
        new_color = BVHTraceDebug(r, minimizer, DefaultMode);

      image[pos] = (1 - weight) * old + weight * new_color;

      if (update_pass)
        diff += std::abs(old.x() - image[pos].x()) + std::abs(old.y() - image[pos].y()) + std::abs(old.z() - image[pos].z());

    }
  }

  diff /= width * height;

  if (update_pass)
  {
    tick("Convergence: " + std::to_string(diff), true);
    return diff;
  }
  return 1;
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
  float rad = camera.w * randf();
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
    {
      for (auto l : lights_p) {
        //color += (std::max)(0.0f, minimizer.closest_int.N.dot((l->Position - minimizer.closest_int.object->Position).normalized())) * (minimizer.closest_int.Kd.x() > 0 ? minimizer.closest_int.Kd : minimizer.closest_int.object->material->Kd);
        Eigen::Vector3f w_o = -r.direction;
        Intersection &i = minimizer.closest_int;
        Eigen::Vector3f w_i = (l->Position - i.P).normalized();
        color += static_cast<Light *>(l->material)->light_value.cwiseProduct(EvalScattering(w_o, i.N, w_i, i));
      }
    }
    else if (mode == DEBUG_MODE::NORMAL)
      color = Eigen::Vector3f(std::abs(minimizer.closest_int.N.x()), std::abs(minimizer.closest_int.N.y()), std::abs(minimizer.closest_int.N.z()));
    else if (mode == DEBUG_MODE::DEPTH)
      color = minimizer.closest_int.t * Eigen::Vector3f(1.f, 1.f / 10.f, 1.f / 100.f);
    else if (mode == DEBUG_MODE::DIFFUSE)
      color = minimizer.closest_int.Kd.x() > 0 ? minimizer.closest_int.Kd : minimizer.closest_int.object->material->Kd;
    else if (mode == DEBUG_MODE::POSITION)
      color = minimizer.closest_int.P / 5.f;
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

bool isNotValidVec(Eigen::Vector3f &v)
{
  return std::isnan(v.x()) || std::isnan(v.y()) || std::isnan(v.z()) || std::abs(v.x()) > 1000000 || std::abs(v.y()) > 1000000 || std::abs(v.z()) > 1000000;
}

void PrintMe(Eigen::Vector3f vec, std::string msg)
{
  std::cout << std::to_string(vec.x()) << "," << std::to_string(vec.y()) << "," << std::to_string(vec.z()) << msg << std::endl;
}

void PrintMe(float f, std::string msg)
{
  std::cout << std::to_string(f) << msg << std::endl;
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
  Eigen::Vector3f w_i, w_o;
  w_o = -r.direction;
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
      Eigen::Vector3f f = EvalScattering(w_o, N, w_i, P);
      color += 0.5f * weight.cwiseProduct(f).cwiseProduct(EvalRadiance(L)) / p;
    }


    //extend path
    N = P.N;
    w_i = SampleBRDF(w_o, N, P);
    Intersection &Q = FireRayIntoScene(minimizer, w_i);

    //if intersection doesn't exist, break
    if (Q.object == nullptr)
      break;

    Eigen::Vector3f f = EvalScattering(w_o, N, w_i, P);
    p = PdfBRDF(w_o, N, w_i, P) * RussianRoulette;

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
    w_o = -w_i;
  }

  if (isNotValidVec(color))
  {
    std::cout << "Path Resulted in a NAN..." << std::endl;
  }
  return color;
}

void Scene::SampleLight(Intersection &I)
{
  int light_index = rand() % lights_p.size();
  lights_p[light_index]->GetRandomPointOn(I);
}

Eigen::Vector3f Scene::EvalScattering(Eigen::Vector3f &w_o, Eigen::Vector3f &N, Eigen::Vector3f &w_i, Intersection &i)
{
  Eigen::Vector3f color = i.Kd.x() > 0 ? i.Kd : i.object->material->Kd;
  Eigen::Vector3f diffuse = color / pi;
  Eigen::Vector3f half = (w_o + w_i).normalized();
  float N_w_i = (std::max)(0.0f, N.dot(w_i));
  Material *m = i.object->material;
  float denom = (4 * std::abs(w_i.dot(N)) * std::abs(w_o.dot(N)));
  if (denom == 0)
    return N_w_i * diffuse;

  float D = m->D(half, N);
  Eigen::Vector3f F = m->F(w_i.dot(half));
  float G = m->G(w_i, w_o, half, N);

  Eigen::Vector3f specular = (D * G * F) / denom;

  return N_w_i * (diffuse + specular);
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

float Scene::PdfBRDF(Eigen::Vector3f &w_o, Eigen::Vector3f &N, Eigen::Vector3f &w_i, Intersection &s)
{
  float p_d = std::abs(N.dot(w_i)) / pi;
  Eigen::Vector3f half = (w_o + w_i).normalized();
  float denom = (4 * std::abs(w_i.dot(half)));
  if (denom == 0)
    return 0;
  float p_s = s.object->material->D(half, N) * std::abs(half.dot(N)) / denom;
  float a = s.object->material->alpha;
  return a * p_d + (1 - a) * p_s;
}

Eigen::Vector3f Scene::EvalRadiance(Intersection &Q)
{
  return static_cast<Light *>(Q.object->material)->light_value;
}

Eigen::Vector3f Scene::SampleBRDF(Eigen::Vector3f &w_o, Eigen::Vector3f &N, Intersection &s)
{
  float r1 = randf();
  float r2 = randf();
  float a = s.object->material->alpha;
  if (randf() <= a)
  {
    //diffuse
    return SampleLobe(N, std::sqrt(r1), pi_2 * r2);
  }
  else
  {
    //specular
    float theta = atan(a * std::sqrt(r1) / std::sqrt(1 - r1));
    Eigen::Vector3f m = SampleLobe(N, cos(theta), pi_2 * r2);
    return 2 * w_o.dot(m) * m - w_o;
  }
}

Eigen::AlignedBox<float, 3> bounding_box(const Shape *obj)
{
  return obj->BoundingBox; // Assuming each Shape object has its own bbox method.
};
