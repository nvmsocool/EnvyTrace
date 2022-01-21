///////////////////////////////////////////////////////////////////////
// Provides the framework a raytracer.
//
// Gary Herron
//
// Copyright 2012 DigiPen Institute of Technology
////////////////////////////////////////////////////////////////////////
#pragma comment(linker, "/SUBSYSTEM:windows /ENTRY:mainCRTStartup")

#include <fstream>
#include <sstream>
#include <vector>
#include <string.h>
#include <ctime>
#include <iostream>
#include <chrono>
#include <future>
#include <thread>

#ifdef _WIN32
    // Includes for Windows
    #include <windows.h>
    #include <cstdlib>
    #include <limits>
    #include <crtdbg.h>
#else
    // Includes for Linux
    #include <stdlib.h>
    #include <time.h> 
#endif

#include "geom.h"
#include "raytrace.h"
#include "realtime.h"

#include "ImageData.h"

ImageData image, preview;

//tracer variables
Scene *scene;
std::string hdrName;
std::string bmpName;

//camera settings
float cam_speed_move = 0.1f;
float cam_speed_rot = 2.f;

//tracer state settings
bool can_receive_input = true;
bool isPaused = false;
bool shouldReset = false;
std::string scene_file_name;

float preview_ratio = 0.1;

float trace_duration = 0;
float trace_diff = 0;

float gui_fps = 0;

int max_ms_to_wait = 100;

float last_ms_to_wait = max_ms_to_wait;

//may return 0 when not able to detect
const auto processor_count = std::thread::hardware_concurrency() - 1;
int num_threads_to_use = processor_count;

void ResetTrace();

// Read a scene file by parsing each line as a command and calling
// scene->Command(...) with the results.
void ReadScene(const std::string inName, Scene* scene, bool Hard)
{
    std::ifstream input(inName.c_str());
    if (input.fail()) {
        std::cerr << "File not found: "  << inName << std::endl;
        fflush(stderr);
        exit(-1); }

    // For each line in file
    for (std::string line; getline(input, line); ) {
        std::vector<std::string> strings;
        std::vector<float> floats;
        
        // Parse as parallel lists of strings and floats
        std::stringstream lineStream(line);
        for (std::string s; lineStream >> s; ) { // Parses space-separated strings until EOL
            float f;
            //std::stringstream(s) >> f; // Parses an initial float into f, or zero if illegal
            if (!(std::stringstream(s) >> f)) f = nan(""); // An alternate that produced NANs
            floats.push_back(f);
            strings.push_back(s); }

        if (strings.size() == 0) continue; // Skip blanks lines
        if (strings[0][0] == '#') continue; // Skip comment lines
        
        // Pass the line's data to Command(...)
        scene->Command(strings, floats, Hard);
    }

    input.close();
}

// Read a scene file, overwrite camera positional info
void RewriteScene(const std::string inName, Scene *scene)
{
  std::ifstream input(inName.c_str());
  if (input.fail()) {
    std::cerr << "File not found: " << inName << std::endl;
    fflush(stderr);
    exit(-1);
  }

  std::string newFile;

  // For each line in file
  for (std::string line; getline(input, line); ) {
    std::vector<std::string> strings;

    std::stringstream lineStream(line);
    for (std::string s; lineStream >> s; )
      strings.push_back(s);

    if (strings.size() == 0)
      newFile = newFile + "\n";
    else if (strings[0] == "camera")
      newFile = newFile + scene->camera.GetCameraString() + "\n";
    else
      newFile = newFile + line + "\n";
  }

  input.close();

  std::ofstream output;
  output.open(inName);
  output << newFile;
  output.close();

}

// Write the image as a HDR(RGBE) image.  
#include "rgbe.h"
void generateHDRImage(ImageData &id)
{

  //write HDR
    // Turn image from a 2D-bottom-up array of Vector3D to an top-down-array of floats
  float *data = new float[id.data.size() * 3];
  float *dp = data;
  for (int y = id.h - 1; y >= 0; --y) {
    for (int x = 0; x < id.w; ++x) {
      Color pixel = id.data[y * id.w + x];
      *dp++ = pixel[0];
      *dp++ = pixel[1];
      *dp++ = pixel[2];
    }
  }

  // Write image to file in HDR (a.k.a RADIANCE) format
  rgbe_header_info info;
  char errbuf[100] = { 0 };

  FILE *fp = fopen(hdrName.c_str(), "wb");
  info.valid = false;
  int r = RGBE_WriteHeader(fp, id.w, id.h, &info, errbuf);
  if (r != RGBE_RETURN_SUCCESS)
    printf("error: %s\n", errbuf);

  r = RGBE_WritePixels_RLE(fp, data, id.w, id.h, errbuf);
  if (r != RGBE_RETURN_SUCCESS)
    printf("error: %s\n", errbuf);
  fclose(fp);

  delete data;

}

#include "BMP.h"
//void SaveImageToDisk(ImageData &id)
//{
//  
//  generateBitmapImage(id, bmpName.data());
//
//  return;
//}

void ClearImage(ImageData &id)
{
  for (int y = 0; y < id.h; y++)
    for (int x = 0; x < id.w; x++)
      id.data[y * id.w + x] = Color(0, 0, 0);
  id.trace_num = 1;
}

void ResizePreview()
{
  preview.w = preview_ratio * scene->requested_width;
  preview.h = preview_ratio * scene->requested_height;
  preview.data.resize(preview.w * preview.h);
  ClearImage(preview);
}

void ResizeImages()
{
  image.w = scene->requested_width;
  image.h = scene->requested_height;
  image.data.resize(image.w * image.h);
  ClearImage(image);

  ResizePreview();
}

void SaveCopy(std::string &inName)
{
  std::string priorBmpName = inName;
  std::string bmpBackup = inName;

  const auto p1 = std::chrono::system_clock::now();
  std::string newpostfix = std::to_string(std::chrono::duration_cast<std::chrono::seconds>(p1.time_since_epoch()).count()) + std::string(".bmp");
  bmpBackup.replace(bmpBackup.size() - 4, bmpBackup.size(), newpostfix);
  priorBmpName.replace(priorBmpName.size() - 3, priorBmpName.size(), "bmp");

  system((std::string("copy ") + priorBmpName + " " + bmpBackup).c_str());
}

void SetupScene(Scene *scene, std::string &inName, bool HardReset)
{

  scene->ClearAll();

  SaveCopy(inName);

  // Read the scene, calling scene.Command for each line.
  ReadScene(inName, scene, HardReset);

  scene->Finit();
}

void PurgeKeys()
{
  for (auto k : keyLookup)
    GetAsyncKeyState(k.second);
}

bool IsKeyDown(char *key)
{
  return GetAsyncKeyState(keyLookup[key]);
}

void SetInput(bool &can_input, bool val)
{
  can_input = val;
  if (can_input)
  {
    PurgeKeys();
  }
}



void DrawGUI()
{

  //ImGui::ShowDemoWindow();

  // Start the Dear ImGui frame
  ImGui_ImplOpenGL2_NewFrame();
  ImGui_ImplGLUT_NewFrame();

  ImGui::SetNextWindowSize(ImVec2(scene->gui_width, image.h));
  ImGui::SetNextWindowPos(ImVec2(image.w, 0));
  
  ImGui::Begin("Fractal Tracer", NULL, ImGuiWindowFlags_HorizontalScrollbar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize);                          // Create a window called "Hello, world!" and append into it.

  ImGui::Text("GUI (%.1f FPS)", gui_fps);
  ImGui::Text("Trace info:", image.trace_num);
  ImGui::Text("Frame %d, %.3fs/trace, conv=%.4f", image.trace_num, trace_duration, trace_diff);
  ImGui::Text("Preview Ratio: %.4f", preview_ratio);
  
  ImGui::BeginChild("settings");

  if (ImGui::CollapsingHeader("scene"))
  {
    ImGui::Indent(16.0f);

    // ImGui::InputText("scene_file", &scene_file_name);
    if (ImGui::Button("reload_scene"))
    {
      //reload scene
      SetupScene(scene, scene_file_name, true);
      ResizeImages();
      ResetTrace();
    }
    if (ImGui::Button("save_scene"))
    {
      SaveCopy(scene_file_name);
      RewriteScene(scene_file_name, scene);
    }
    if (ImGui::Button("BMP snap"))
    {
      generateBitmapImage(image, bmpName.data());
    }
    ImGui::SameLine();
    if (ImGui::Button("HDR snap"))
    {
      generateHDRImage(image);
    }

    ImGui::Unindent(16.0f);
  }
  if (ImGui::CollapsingHeader("tracer settings"))
  {
    ImGui::Indent(16.0f);

    const char *items[] = { 
      "NONE",
      "NORMAL",
      "DEPTH",
      "DIFFUSE",
      "SIMPLE",
      "POSITION", };
    static int item_current = scene->DefaultMode;
    if (ImGui::Combo("render_type", &item_current, items, IM_ARRAYSIZE(items), 4))
    {
      scene->DefaultMode = static_cast<Scene::DEBUG_MODE>(item_current);
      // starting render, disable inputs
      if (scene->DefaultMode == Scene::DEBUG_MODE::NONE)
      {
        SetInput(can_receive_input, false); 
        srand(427857);
      }
      ResetTrace();
    }
    ImGui::Checkbox("can_receive_input", &can_receive_input);
    ImGui::SliderInt("threads", &num_threads_to_use, 1, processor_count);
    if (ImGui::Checkbox("isPaused", &isPaused))
    {
      SetInput(can_receive_input, false);
    }
    if (ImGui::Checkbox("halfDome", &scene->halfDome))
    {
      ResetTrace();
    }

    ImGui::Unindent(16.0f);
  }
  if (ImGui::CollapsingHeader("camera settings"))
  {
    ImGui::Indent(16.0f);

    ImGui::DragFloat("move_speed", &cam_speed_move, 0.01f);
    ImGui::DragFloat("rot_speed", &cam_speed_rot, 0.01f);
    if (ImGui::Checkbox("depth_of_field", &scene->depth_of_field))
    {
      ResetTrace();
    }
    if (scene->depth_of_field)
    {
      ImGui::Indent(10.f);
      if (ImGui::DragFloat("dof_w", &scene->camera.w, 0.001f, 0.0, 100, "%.3f"))
        ResetTrace();
      if(ImGui::DragFloat("dof_f", &scene->camera.f, 0.01f, 0.f, 10000.f, "%2f"))
        ResetTrace();
      ImGui::Unindent(10.f);
    }
    if (ImGui::Checkbox("use_AA", &scene->use_AA))
    {
      ResetTrace();
    }
    if (ImGui::DragFloat("fov", &scene->camera.ry, 0.01f, 0.01f, 1.0f, "%.2f"))
    {
      scene->camera.UpdateFOV(image.w, image.h);
      ResetTrace();
    }

    ImGui::Unindent(16.0f);
  }
  if(ImGui::CollapsingHeader("shapes"))
  {
    ImGui::Indent(16.0f);

    for (size_t i = 0; i < scene->objects_p.size(); i++)
    {
      if (scene->objects_p[i]->RenderGenericGUI(i))
      {
        ResetTrace();
      }
    }

    ImGui::Unindent(16.0f);
  }

  ImGui::EndChild();

  ImGui::End();


  // Rendering
  ImGui::Render();
  ImGuiIO &io = ImGui::GetIO();


  ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
}

void ResetTrace()
{
  shouldReset = true;
  ClearImage(preview);
}

void InterfaceLoop()
{
  // state bools
  bool isWindowActive = true;
  while (!scene->realtime->closed)
  {
    if (!isPaused)
    {
      if (shouldReset)
      {
        ClearImage(image);
        shouldReset = false;
      }

      if (scene->realtime->needs_resize)
      {
        scene->requested_width = scene->realtime->window_width < scene->gui_width ? 1 : scene->realtime->window_width - scene->gui_width;
        scene->requested_height = scene->realtime->window_height;
        ResizeImages();
        ResetTrace();
        scene->realtime->needs_resize = false;
      }

      auto start_time = std::chrono::high_resolution_clock::now();

      bool update_pass = (image.trace_num - 1) % 10 == 0 || (IsKeyDown("Space") && isWindowActive);
      float diff = scene->TraceImage(image, update_pass, num_threads_to_use);
      if (update_pass)
        trace_diff = diff;

      auto end_time = std::chrono::high_resolution_clock::now();
      float ms = static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count());
      trace_duration = ms / 1000000.f;

      if (update_pass)
      {
        //just in case...
        generateBitmapImage(image, bmpName.data());
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{

    scene = new Scene();

    // Read the command line argument
    scene_file_name =  (argc > 1) ? argv[1] : "testscene.scn";
    hdrName = scene_file_name;
    hdrName.replace(hdrName.size() - 3, hdrName.size(), "hdr");
    bmpName = scene_file_name;
    bmpName.replace(bmpName.size() - 3, bmpName.size(), "bmp");


    SetupScene(scene, scene_file_name, true);

    // Allocate and clear an image array
    ResizeImages();

    scene->realtime->setup();

    // state bools
    bool autoStart = argc > 1 ? true : false;
    bool isWindowActive = true;

    if (autoStart)
      scene->DefaultMode = Scene::DEBUG_MODE::DIFFUSE;

    // run interface loop
    auto it = std::thread(InterfaceLoop);
    bool previewed_this_frame = false;

    float min_ratio = image.w;
    if (image.h < image.w)
      min_ratio = 2 / min_ratio;

    while (!(IsKeyDown("Esc") && IsKeyDown("Shift") && isWindowActive) && !scene->realtime->closed)
    {
      auto start_time = std::chrono::high_resolution_clock::now();

      isWindowActive = GetConsoleWindow() == GetForegroundWindow() || scene->realtime->isWindowActive();

      if (!scene->realtime->closed)
      {
        if ((image.trace_num < 2 && scene->DefaultMode != Scene::DEBUG_MODE::NONE) || shouldReset)
        {
          float diff = scene->TraceImage(preview, false, num_threads_to_use);
          scene->realtime->DrawArray(preview, scene->gui_width);
          previewed_this_frame = true;
        }
        else
        {
          scene->realtime->DrawArray(image, scene->gui_width);
        }
        DrawGUI();
        scene->realtime->FinishDrawing();
      }

      scene->realtime->UpdateEvent();

      if (can_receive_input && isWindowActive)
      {

        if (IsKeyDown("A")) { scene->MoveCamera(Eigen::Vector3f(-cam_speed_move, 0, 0)); ResetTrace(); }
        if (IsKeyDown("D")) { scene->MoveCamera(Eigen::Vector3f( cam_speed_move, 0, 0)); ResetTrace(); }
        if (IsKeyDown("E")) { scene->MoveCamera(Eigen::Vector3f(0, 0, -cam_speed_move)); ResetTrace(); }
        if (IsKeyDown("Q")) { scene->MoveCamera(Eigen::Vector3f(0, 0,  cam_speed_move)); ResetTrace(); }
        if (IsKeyDown("W")) { scene->MoveCamera(Eigen::Vector3f(0,  cam_speed_move, 0)); ResetTrace(); }
        if (IsKeyDown("S")) { scene->MoveCamera(Eigen::Vector3f(0, -cam_speed_move, 0)); ResetTrace(); }

        if (IsKeyDown("J")) { scene->RotateCamera(EulerToQuat(Eigen::Vector3f(0,  cam_speed_rot, 0))); ResetTrace(); }
        if (IsKeyDown("L")) { scene->RotateCamera(EulerToQuat(Eigen::Vector3f(0, -cam_speed_rot, 0))); ResetTrace(); }
        if (IsKeyDown("K")) { scene->RotateCamera(EulerToQuat(Eigen::Vector3f(-cam_speed_rot, 0, 0))); ResetTrace(); }
        if (IsKeyDown("I")) { scene->RotateCamera(EulerToQuat(Eigen::Vector3f( cam_speed_rot, 0, 0))); ResetTrace(); }
        if (IsKeyDown("O")) { scene->RotateCamera(EulerToQuat(Eigen::Vector3f(0, 0, -cam_speed_rot))); ResetTrace(); }
        if (IsKeyDown("U")) { scene->RotateCamera(EulerToQuat(Eigen::Vector3f(0, 0,  cam_speed_rot))); ResetTrace(); }
      }
      
      auto end_time = std::chrono::high_resolution_clock::now();

      float ms = static_cast<float>(std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count());

      float ms_to_wait = max_ms_to_wait - ms;

      gui_fps = 6000 / ms;

      if (previewed_this_frame)
      {
        //negative->make smaller, 0->no change, 100->make bigger
        float avg_ms_to_wait = (last_ms_to_wait + ms_to_wait) / 2.f;
        float ideal_ratio = avg_ms_to_wait / max_ms_to_wait;
        if (ideal_ratio < min_ratio)
          ideal_ratio = min_ratio;
        if (ideal_ratio > 1)
          ideal_ratio = 1;

        float suggested_ratio = (ideal_ratio + preview_ratio) / 2;
        if (std::abs(preview_ratio - suggested_ratio) > 0.1)
        {
          preview_ratio = suggested_ratio;
          ResizePreview();
        }
        previewed_this_frame = false;
      }

      if (ms_to_wait < 0) ms_to_wait = 0;

      last_ms_to_wait = ms_to_wait;

      Sleep(ms_to_wait);
    }

    // wait for the last render to finish
    it.join();

    // one for the road
    generateBitmapImage(image, bmpName.data());
}