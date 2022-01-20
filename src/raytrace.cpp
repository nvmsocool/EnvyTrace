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

  objects_p.clear();
  lights_p.clear();
  for (int i = 0; i < spheres.size(); i++)
  {
    objects_p.push_back(static_cast<Shape *>(&(spheres[i])));
    spheres[i].name += std::to_string(i);
  }
  for (int i = 0; i < boxes.size(); i++)
  {
    objects_p.push_back(&(boxes[i]));
    boxes[i].name += std::to_string(i);
  }
  for (int i = 0; i < triangles.size(); i++)
  {
    objects_p.push_back(&(triangles[i]));
    triangles[i].name += std::to_string(i);
  }
  for (int i = 0; i < cylinders.size(); i++)
  {
    objects_p.push_back(&(cylinders[i]));
    cylinders[i].name += std::to_string(i);
  }
  for (int i = 0; i < fractals.size(); i++)
  {
    fractals[i].GenColors();
    objects_p.push_back(&(fractals[i]));
    fractals[i].name += std::to_string(i);
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

bool Material::RenderGUI(int shape_num)
{
  bool something_changed = false;

  if (ImGui::CollapsingHeader("material"))
  {
    ImGui::Indent(10.f);
    something_changed |= ImGui::SliderFloat3((std::string("Kd##") + std::to_string(shape_num)).data(), Kd.data(), 0, 1, "%.2f");
    something_changed |= ImGui::SliderFloat3((std::string("Ks##") + std::to_string(shape_num)).data(), Ks.data(), 0, 1, "%.2f");
    something_changed |= ImGui::SliderFloat3((std::string("Kt##") + std::to_string(shape_num)).data(), Kt.data(), 0, 1, "%.2f");

    something_changed |= ImGui::DragFloat((std::string("alpha##") + std::to_string(shape_num)).data(), &alpha, 0.01f, 0, 10000, "%.2f");
    something_changed |= ImGui::SliderFloat((std::string("specularity##") + std::to_string(shape_num)).data(), &specularity, 0, 1, "%.2f");
    ImGui::Unindent(10.f);
  }

  return something_changed;
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
      realtime = new Realtime(int(f[1]) + gui_width, int(f[2]));
      realtime->setScreen(int(f[1]) + gui_width, int(f[2]));
      requested_width = int(f[1]);
      requested_height = int(f[2]);
      first_load = false;
    }
    else if (requested_width != f[1] || requested_height != f[2])
    {
      requested_width = int(f[1]);
      requested_height = int(f[2]);
      realtime->RequestResize(requested_width + gui_width, requested_height);

    }
  }

  else if (c == "camera") {
    // syntax: camera x y z   ry   <orientation spec>
    // Eye position (x,y,z),  view orientation (qw qx qy qz),  frustum height ratio ry

    if (hard)
    {
      Eigen::Quaternionf rot = EulerToQuat(Eigen::Vector3f(f[5], f[6], f[7]));

      //realtime->setCamera(Vector3f(f[1], f[2], f[3]), Orientation(5, strings, f), f[4]);
      camera.SetProperties(rot, Vector3f(f[1], f[2], f[3]), f[4], requested_width, requested_height, f[8], f[9]);
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
    if (f.size() >= 14)
      materials.push_back(Material(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7], f[8], Vector3f(f[9], f[10], f[11]), f[12], f[13]));
    else
      materials.push_back(Material(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7], f[8]));
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

      Fractal::ActionData actionData;
      actionData.action_type = static_cast<int>(f[current_index]);
      actionData.DisplayOp = Eigen::Vector3f(f[current_index + 1], f[current_index + 2], f[current_index + 3]);
      switch (actionData.action_type)
      {
      case 0: // fold
        actionData.VecOp = actionData.DisplayOp.normalized();
        break;
      case 1: //rotation
        actionData.QuatOp = EulerToQuat(actionData.DisplayOp);
        break;
      default: //scale + translation
        actionData.VecOp = actionData.DisplayOp;
        break;
      }
      current_index += 4;
      fr->CombinedActions.push_back(actionData);
    }
  }


  else {
    fprintf(stderr, "\n*********************************************\n");
    fprintf(stderr, "* Unknown command: %s\n", c.c_str());
    fprintf(stderr, "*********************************************\n\n");
  }
}


float Scene::TraceImage(ImageData &id, bool update_pass, int n_threads)
{

  float diff = 0;
  float weight = 1.f / static_cast<float>(id.trace_num);

#pragma omp parallel for schedule(dynamic, 1) num_threads(n_threads)  // Magic: Multi-thread y loop
  for (int y = 0; y < id.h; y++) {

    Ray r(Eigen::Vector3f::Ones(), Eigen::Vector3f::Ones());
    Minimizer minimizer(r);
    int y_add = y * id.w;

    for (int x = 0; x < id.w; x++) {
      if (halfDome)
        SetRayHalfDome(id, r, x, y);
      else if (depth_of_field)
        SetRayDOF(id, r, x, y);
      else if (use_AA)
        SetRayAA(id, r, x, y);
      else
        SetRayDirect(id, r, x, y);

      int pos = y_add + x;
      Color old = id.data[pos];
      if (DefaultMode == Scene::DEBUG_MODE::NONE)
        id.data[pos] = (1 - weight) * old + weight * BVHTracePath(r, minimizer, false);
      else
        id.data[pos] = (1 - weight) * old + weight * BVHTraceDebug(r, minimizer, DefaultMode);

      if (update_pass)
        diff += std::abs(old.x() - id.data[pos].x()) + std::abs(old.y() - id.data[pos].y()) + std::abs(old.z() - id.data[pos].z());
    }
  }


  id.trace_num++;

  if (update_pass)
  {
    diff /= id.data.size();
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

void Scene::SetRayDirect(ImageData &id, Ray &r, int x, int y)
{
  float dy = 2 * (y + 0.5f) / id.h - 1;
  float dx = 2 * (x + 0.5f) / id.w - 1;

  r.direction = (dx * camera.ViewX + dy * camera.ViewY + camera.ViewZ).normalized();
  r.origin = camera.position;
}

void Scene::SetRayAA(ImageData &id, Ray &r, int x, int y)
{
  float dy = 2 * (y + randf()) / id.h - 1;
  float dx = 2 * (x + randf()) / id.w - 1;

  r.direction = (dx * camera.ViewX + dy * camera.ViewY + camera.ViewZ).normalized();
  r.origin = camera.position;
}

void Scene::SetRayDOF(ImageData &id, Ray &r, int x, int y)
{
  float rad = camera.w * std::sqrt(randf());
  float theta = pi_2 * randf();
  float dx = rad * std::cos(theta);
  float dy = rad * std::sin(theta);
  Eigen::Vector3f E = camera.position + dx * camera.ViewX + dy * camera.ViewY;

  dy = 2 * (y + randf()) / id.h - 1;
  dx = 2 * (x + randf()) / id.w - 1;
  Eigen::Vector3f P = camera.position + camera.f * dx * camera.ViewX + camera.f * dy * camera.ViewY + camera.f * camera.ViewZ;

  r.direction = (P - E).normalized();
  r.origin = E;
}

void Scene::SetRayHalfDome(ImageData &id, Ray &r, int x, int y)
{
  //create ray in that direction
  int halfway = id.w / 2;
  float theta = pi * (y + randf()) / id.h;
  float z_d = std::cos(theta);
  float sinTheta = std::sin(theta);
  float phi;
  if (x < halfway)
  {
    r.origin = camera.position;
    phi = pi * ((2.f * x) + 0.5f) / id.w;
  }
  else
  {
    r.origin = camera.position - camera.rotation._transformVector(Eigen::Vector3f(0.035, 0, 0));
    phi = pi * (2.f * (x - halfway) + randf()) / id.w;
  }
  float x_d = sinTheta * std::cos(phi);
  float y_d = sinTheta * std::sin(phi);
  r.direction = camera.rotation._transformVector(Eigen::Vector3f(x_d, y_d, z_d)).normalized();
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

        // VERY simple
        //Eigen::Vector3f L = (l->Position - minimizer.closest_int.object->Position).normalized();
        //float n_dot_l = minimizer.closest_int.N.dot(L);
        //Eigen::Vector3f obj_color = minimizer.closest_int.Kd.x() > 0 ? minimizer.closest_int.Kd : minimizer.closest_int.object->material->Kd;
        //color += (std::max)(0.0f, n_dot_l) * obj_color;

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
  return std::isnan(v.x()) || std::isnan(v.y()) || std::isnan(v.z()) || std::isinf(v.x()) || std::isinf(v.y()) || std::isinf(v.z());
}

bool isNotReasonableVec(Eigen::Vector3f &v, float thresh)
{
  return std::abs(v.x()) > thresh || std::abs(v.y()) > thresh || std::abs(v.z()) > thresh;
}

void PrintMe(Eigen::Vector3f vec, std::string msg)
{
  std::cout << std::to_string(vec.x()) << "," << std::to_string(vec.y()) << "," << std::to_string(vec.z()) << msg << std::endl;
}

void PrintMe(float f, std::string msg)
{
  std::cout << std::to_string(f) << msg << std::endl;
}

//#define LOGGIN

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

  std::string log = "";
  bool build_log = true;
#ifdef LOGGIN
  log += "start path\n";
  log += P.object->name;
  log += " (<-name)\n";
#endif

  while (randf() < RussianRoulette)
  {
#ifdef LOGGIN
    log += "start bounce\n";
#endif
    // always starts here
    r.origin = P.P;
    N = P.N;


#ifdef LOGGIN
      log += std::to_string(w_o.x());
      log += ",";
      log += std::to_string(w_o.y());
      log += ",";
      log += std::to_string(w_o.z());
      log += "(<-w_o)\n";
#endif

    // explicit light
    SampleLight(L);
    p = PdfLight(L.object) / GeometryFactor(P, L);
#ifdef LOGGIN
      log += std::to_string(p);
      log += "(<- explicit p)\n";
#endif
    
    //check if p is positive, not necessary?
    
    w_i = (L.P - P.P).normalized();
#ifdef LOGGIN
      log += std::to_string(w_i.x());
      log += ",";
      log += std::to_string(w_i.y());
      log += ",";
      log += std::to_string(w_i.z());
      log += "(<-explicit w_i)\n";
#endif
    Intersection &I = FireRayIntoScene(minimizer, w_i);
    if (I.object == L.object)
    {
      float q = PdfBRDF(w_o, N, w_i, P);
      float w_mis = std::pow(p, 2) / (std::pow(p, 2) + std::pow(q, 2));
      Eigen::Vector3f f = EvalScattering(w_o, N, w_i, P);
      color += 0.5f * w_mis * weight.cwiseProduct(f).cwiseProduct(EvalRadiance(L)) / p;

#ifdef LOGGIN
        log += std::to_string(f.x());
        log += ",";
        log += std::to_string(f.y());
        log += ",";
        log += std::to_string(f.z());
        log += "(<-explicit scattering)\n";
        log += std::to_string(color.x());
        log += ",";
        log += std::to_string(color.y());
        log += ",";
        log += std::to_string(color.z());
        log += "( <-explicit color added)\n";
#endif

    }


    //extend path
    w_i = SampleBRDF(w_o, N, P);

#ifdef LOGGIN
      log += std::to_string(w_i.x());
      log += ",";
      log += std::to_string(w_i.y());
      log += ",";
      log += std::to_string(w_i.z());
      log += "(<-implicit w_i)\n";
#endif

    Intersection &Q = FireRayIntoScene(minimizer, w_i);

    //if intersection doesn't exist, break
    if (Q.object == nullptr)
      break;

    Eigen::Vector3f f = EvalScattering(w_o, N, w_i, P);
    p = PdfBRDF(w_o, N, w_i, P) * RussianRoulette;

    if (p == 0)
    {
      int test = PdfBRDF(w_o, N, w_i, P);
    }

#ifdef LOGGIN
      log += std::to_string(f.x());
      log += ",";
      log += std::to_string(f.y());
      log += ",";
      log += std::to_string(f.z());
      log += "(<-implicit scattering)\n";
      log += std::to_string(p);
      log += " (<-implicit p)\n";
      log += Q.object->name;
      log += " (<-name)\n";
#endif

    if (p < 0.0001f)
      break;

    weight = weight.cwiseProduct(f / p);

#ifdef LOGGIN
      log += std::to_string(weight.x());
      log += ",";
      log += std::to_string(weight.y());
      log += ",";
      log += std::to_string(weight.z());
      log += "(<-current weight)\n";
#endif

    //light connection
    if (Q.object->material->isLight())
    {
      float q = PdfLight(Q.object) / GeometryFactor(P, Q);
      float w_mis = std::pow(p, 2) / (std::pow(p, 2) + std::pow(q, 2));
      // after light
      color += 0.5 * w_mis * weight.cwiseProduct(EvalRadiance(Q));

#ifdef LOGGIN
        log += std::to_string(color.x());
        log += ",";
        log += std::to_string(color.y());
        log += ",";
        log += std::to_string(color.z());
        log += "(<-implicit color added)\n";
#endif

      break;
    }
    P = Q;
    w_o = -w_i;
  }

  float mx = 100.f;
  
  //cap color
  float max_channel = (std::max)(color.x(), (std::max)(color.y(), color.z()));
  if (max_channel > mx)
    color *= mx / max_channel;
  if (isNotValidVec(color))
  {
    std::cout << "Path Resulted in a NAN:" << std::endl;
  }
  if (isNotReasonableVec(color, 10000))
  {
    std::cout << "Path Resulted in a big value:" << std::endl;
  }

#ifdef LOGGIN
  if (isNotValidVec(color) || isNotReasonableVec(color, 10000))
    std::cout << log << std::endl;
#endif

#ifndef LOGGIN
  if (isNotValidVec(color))
    return Eigen::Vector3f::Zero();
#endif

  return color;
}

void Scene::SampleLight(Intersection &I)
{
  int light_index = rand() % lights_p.size();
  lights_p[light_index]->GetRandomPointOn(I);
}

Eigen::Vector3f Scene::GetBeers(const float t, Eigen::Vector3f Kt)
{
  return Eigen::Vector3f(
    std::exp(t * std::log(Kt.x())),
    std::exp(t * std::log(Kt.y())),
    std::exp(t * std::log(Kt.z()))
  );
}

Eigen::Vector3f Scene::EvalScattering(Eigen::Vector3f &w_o, Eigen::Vector3f &N, Eigen::Vector3f &w_i, Intersection &i)
{
  Eigen::Vector3f color = i.Kd.x() > 0 ? i.Kd : i.object->material->Kd;
  Eigen::Vector3f diffuse = color / pi;
  Eigen::Vector3f half = (w_o + w_i).normalized();

  float N_w_i = std::abs(N.dot(w_i));
  Material *m = i.object->material;
  float denom = (4 * std::abs(w_i.dot(N)) * std::abs(w_o.dot(N)));
  Eigen::Vector3f ret;
  if (denom == 0)
  {
    ret = N_w_i * diffuse;
#ifdef LOGGIN
    if (isNotValidVec(ret))
    {
      PrintMe(diffuse, "diffuse");
      PrintMe(N_w_i, "N_w_i");
      PrintMe(half, "N_w_i");
      PrintMe(denom, "denom");
    }
#endif
    return ret;
  }

  float D = m->D(half, N);
  Eigen::Vector3f F = m->F(std::abs(w_i.dot(half)));
  float G = m->G(w_i, w_o, half, N);
  Eigen::Vector3f specular = (D * G * F) / denom;


  /*
  //transmissive color component
  Eigen::Vector3f transmissive = specular;
  Eigen::Vector3f trans_half = -(i.ior_out * w_i + i.ior_in * w_o).normalized();
  float r = 1 - std::pow(i.ior_ratio, 2) * (1 - std::pow(w_o.dot(trans_half), 2));
  if (r > 0)
  {
    D = m->D(trans_half, N);
    F = m->F(std::abs(w_i.dot(trans_half)));
    G = m->G(w_i, w_o, trans_half, N);
    denom *= std::pow(i.ior_out * w_i.dot(trans_half) + i.ior_in * w_o.dot(trans_half), 2) / 4.0f;
    if (denom == 0)
    {
      ret = N_w_i * (diffuse + specular);
      return ret;
    }
    transmissive = D * G * (Eigen::Vector3f::Ones() - F) * std::abs(w_i.dot(trans_half)) * std::abs(w_o.dot(trans_half)) * std::pow(i.ior_out, 2) / denom;

  }

  if (w_o.dot(N) < 0)
    transmissive = transmissive.cwiseProduct(GetBeers(i.t, m->Kt));
  ret = N_w_i * (diffuse + specular + transmissive);
    */

  ret = N_w_i * (diffuse + specular);

#ifdef LOGGIN
  if (isNotValidVec(ret))
  {
    PrintMe(diffuse, "diffuse");
    PrintMe(N_w_i, "N_w_i");
    PrintMe(half, "N_w_i");
    PrintMe(D, "D");
    PrintMe(G, "G");
    PrintMe(F, "F");
    PrintMe(denom, "denom");

    float a_2 = m->alpha * m->alpha;
    float h_N = half.dot(N);
    float tan_theta = std::sqrt(1 - h_N * h_N) / h_N;
    float denom2 = (PI * std::pow(h_N, 4) * std::pow(a_2 + tan_theta * tan_theta, 2));
    PrintMe(a_2, "a_2");
    PrintMe(h_N, "h_N");
    PrintMe(tan_theta, "tan_theta");
    PrintMe(denom2, "denom2");

  }
#endif

  return ret;
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

  float p_s = 0;
  float specularity = s.object->material->specularity;
  if (specularity > 0)
  {
    Eigen::Vector3f half = (w_o + w_i).normalized();
    float denom = (4 * std::abs(w_i.dot(half)));
    if (denom == 0)
      return 0;
    p_s = s.object->material->D(half, N) * std::abs(half.dot(N)) / denom;
  }

  /*
  float p_t = p_s;
  float translucency = s.object->material->translucency;
  if (translucency > 0)
  {
    Eigen::Vector3f trans_half = -(s.ior_out * w_i + s.ior_in * w_o).normalized();
    float r = 1.0 - std::pow(s.ior_ratio, 2) * (1.0 - std::pow(w_o.dot(trans_half), 2));
    if (r > 0)
    {
      float denom = std::pow(s.ior_out * w_i.dot(trans_half) + s.ior_in * w_o.dot(trans_half), 2);
      if (denom == 0)
        return 0;
      p_t = s.object->material->D(trans_half, N) * std::abs(trans_half.dot(N)) * std::pow(s.ior_out, 2) * std::abs(w_i.dot(trans_half)) / denom;
    }
    else
    {
      Eigen::Vector3f half = (w_o + w_i).normalized();
      float denom = (4 * std::abs(w_i.dot(half)));
      if (denom == 0)
        return 0;
      p_t = s.object->material->D(half, N) * std::abs(half.dot(N)) / denom;
    }
  }

  float diffusness = 1.0f - (specularity + translucency);
  float ret = diffusness * p_d + specularity * p_s + translucency * p_t;

  */

  float diffusness = 1.0f - (specularity);
  float ret = diffusness * p_d + specularity * p_s;

  if (std::isinf(ret))
    ret = (std::numeric_limits<float>::max)();
  return ret;
}

Eigen::Vector3f Scene::EvalRadiance(Intersection &Q)
{
  return static_cast<Light *>(Q.object->material)->light_value;
}

Eigen::Vector3f Scene::SampleBRDF(Eigen::Vector3f &w_o, Eigen::Vector3f &N, Intersection &s)
{
  float r1 = randf();
  float r2 = randf();
  float selector = randf();
  float prob_spec = s.object->material->specularity;
  //float prob_trans = s.object->material->translucency;
  //if (selector < prob_trans + prob_spec)
  if (selector < prob_spec)
  {
    float theta = atan(s.object->material->alpha * std::sqrt(r1) / std::sqrt(1 - r1));
    Eigen::Vector3f m = SampleLobe(N, cos(theta), pi_2 * r2);
    /*
    if (selector < prob_trans)
    {
      //transmissive
      float r = 1.0 - std::pow(s.ior_ratio, 2) * (1.0 - std::pow(w_o.dot(m), 2));
      if (r > 0)
      {
        float si = w_o.dot(N) >= 0 ? 1 : -1;
        Eigen::Vector3f ret = ((s.ior_ratio * w_o.dot(m) - si * std::sqrt(r)) * m - s.ior_ratio * w_o).normalized();
        return ret;
      }
    }
    */

    //specular
    return (2 * std::abs(w_o.dot(m)) * m - w_o).normalized();
  }
  else
  {
    //diffuse
    return SampleLobe(N, std::sqrt(r1), pi_2 * r2);
  }
}

Eigen::AlignedBox<float, 3> bounding_box(const Shape *obj)
{
  return obj->BoundingBox; // Assuming each Shape object has its own bbox method.
};
