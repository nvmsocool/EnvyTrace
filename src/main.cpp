///////////////////////////////////////////////////////////////////////
// Fractal Raytracer by Nick Miller
//
// Original framework by Gary Herron
//
// Copyright 2012 DigiPen Institute of Technology
////////////////////////////////////////////////////////////////////////
#pragma comment(linker, "/SUBSYSTEM:windows /ENTRY:mainCRTStartup")

#define no_init_all deprecated
#include <thread>

#define NOMINMAX
#include <windows.h>

#include "Tracer.h"
#include "Display.h"
#include "BMP.h"
#include "ImageData.h"
#include "Shapes/Shape.h"

//for imgui
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"


#include <vector>
#include <cstdint>
#include <gif-h-master/gif.h>


GifWriter *gif_writer;
bool wannaTraceGif{ false };
bool isTracingGif{ false };
int numGifFrames{ 500 };
int currentGifTrace{ 0 };
int currentGifFrame{ 0 };
int gifDelay{ 4 };
int tracesPerGifFrame{ 100 };

std::vector<ImageData> gifImages;
ImageData image, preview;

//tracer variables
Tracer *tracer;
Display *display;
std::vector<char> baseNameArr(255);
std::string baseName, bmpName, gifName;

std::string singleTrace;

//tracer state settings
bool shouldReset = false;
bool uiResized = false;
bool isEnteringText = false;
bool shouldReload = false;
std::string sceneName;
int tracer_mode;

// gui variables/settings
float traceDuration = 0;
float traceDiff = 0;

// fps vars
float guiFPS = 0;
int maxFPS = 90;
float maxMsToWait;

// preview vars
std::list<float> msToWaitHistory;
float totalFromMsHistory = 0;
float previewRatio = 0.1f;
float min_ratio;

//may return 0 when not able to detect
const auto processorCount = std::thread::hardware_concurrency();
int numThreadsToUse = std::max(1, (int)processorCount - 1);

void ResetTrace();
void ResetFileName();
void HandleGifFrame();


// Read a scene file by parsing each line as a command and calling
// scene->Command(...) with the results.
void ReadScene()
{
  std::ifstream input(sceneName.c_str());
  if (input.fail())
  {
    std::cerr << "File not found: " << sceneName << std::endl;
    fflush(stderr);
    exit(-1);
  }

  // For each line in file
  for (std::string line; getline(input, line);)
  {
    std::vector<std::string> strings;
    std::vector<float> floats;

    // Parse as parallel lists of strings and floats
    std::stringstream lineStream(line);
    for (std::string s; lineStream >> s;)
    { // Parses space-separated strings until EOL
      float f;
      //std::stringstream(s) >> f; // Parses an initial float into f, or zero if illegal
      if (!(std::stringstream(s) >> f))
        f = (float)nan(""); // An alternate that produced NANs
      floats.push_back(f);
      strings.push_back(s);
    }

    if (strings.size() == 0)
      continue; // Skip blanks lines
    if (strings[0][0] == '#')
      continue; // Skip comment lines

    // Pass the line's data to Command(...)
    tracer->Command(strings, floats);
  }

  input.close();
}

// save a scene file
void SaveScene(const std::string inName, Tracer *scene)
{
  std::string newFile;

  //screen
  newFile += "screen " + std::to_string(image.w) + " " + std::to_string(image.h) + "\n";

  //camera
  newFile += scene->camera.GetCameraString() + "\n";

  for (auto m : scene->shapes_by_material)
  {
    //material
    newFile += m.first->Serialize() + "\n";
    for (auto s : m.second)
    {
      //object(s)
      newFile += s->Serialize() + "\n";
    }
  }

  std::ofstream output;
  output.open(inName);
  output << newFile;
  output.close();
}

void ResizePreview()
{
  preview.Resize((int)(previewRatio * tracer->requested_width), (int)(previewRatio * tracer->requested_height));
}

void ResizeImage()
{
  image.Resize(tracer->requested_width, tracer->requested_height);
  display->SetRenderSize(image);

  // render at least one pixel
  min_ratio = 2.f / (float)image.w;
  if (image.h < image.w)
    min_ratio = 2.f / (float)image.h;

  tracer->camera.UpdateFOV((float)tracer->requested_width, (float)tracer->requested_height);
}

void ResizeImages()
{
  ResizeImage();
  ResizePreview();
}

void SaveCopy()
{
  std::string bmpBackup = baseName;

  const auto p1 = std::chrono::system_clock::now();
  std::string newpostfix = std::to_string(std::chrono::duration_cast<std::chrono::seconds>(p1.time_since_epoch()).count()) + std::string(".bmp");
  bmpBackup.replace(bmpBackup.size() - 4, bmpBackup.size(), newpostfix);

  system((std::string("copy ") + bmpName + " " + bmpBackup).c_str());
}

void ResetFPS()
{
  maxMsToWait = 1000.f / (float)maxFPS;
}

void SetupScene()
{

  tracer->ClearAll();

  SaveCopy();

  // Read the scene, calling scene.Command for each line.
  ReadScene();

  tracer->Finit();

  display->SetupWindow(tracer->requested_width, tracer->requested_height);

  // Allocate and clear an image array
  ResizeImages();

  ResetFPS();

  tracer_mode = tracer->DefaultMode;
}

void DrawGUI()
{

  //ImGui::ShowDemoWindow();

  // Start the Dear ImGui frame
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  ImGui::SetNextWindowSize(ImVec2((float)display->gui_width, (float)display->window_height));
  ImGui::SetNextWindowPos(ImVec2((float)(display->window_width - display->gui_width), 0.f));

  ImGui::Begin("Fractal Tracer", NULL, ImGuiWindowFlags_HorizontalScrollbar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize); // Create a window called "Hello, world!" and append into it.

  ImGui::Text("GUI (%.1f FPS)", guiFPS);
  if (isTracingGif)
  {
    ImGui::ProgressBar(gifImages[currentGifFrame].pctComplete, ImVec2(-1, 0), (std::string("Frame ") + std::to_string(currentGifFrame)).data());
  }
  else
  {
    ImGui::ProgressBar(image.pctComplete, ImVec2(-1, 0), (std::string("Frame ") + std::to_string(image.trace_num)).data());
  }
  ImGui::Text("%.3fs/trace, conv=%.4f", traceDuration, traceDiff);
  ImGui::Text("Preview Ratio: %.4f", previewRatio);
  if (ImGui::CollapsingHeader("under mouse", ImGuiTreeNodeFlags_DefaultOpen))
  {
    ImGui::Text("pos: (%d,%d)", display->mouse_x, display->mouse_y);
    ImGui::Text("object: %s", tracer->info_name.data());
    ImGui::Text("distance: %.4f", tracer->info_dist);
    ImGui::Text("position: (%.2f, %.2f, %.2f)", tracer->info_pos[0], tracer->info_pos[1], tracer->info_pos[2]);
    ImGui::Text(singleTrace.data());
  }

  ImGui::BeginChild("settings");

  if (ImGui::CollapsingHeader("scene"))
  {
    ImGui::Indent(16.0f);

    //ImGui::InputText("scene_file", &baseName);
    if (ImGui::InputText("Text", baseNameArr.data(), baseNameArr.size(),
            ImGuiInputTextFlags_CallbackCharFilter))
    {
      baseName = baseNameArr.data();
      ResetFileName();
    }
    if (ImGui::IsItemActive())
      isEnteringText = true;
    else
      isEnteringText = false;

    if (ImGui::Button("load_scene"))
    {
      //reload scene
      shouldReload = true;
    }
    if (ImGui::Button("save_scene"))
    {
      SaveCopy();
      SaveScene(sceneName, tracer);
    }
    if (ImGui::Button("BMP snap"))
    {
      generateBitmapImage(image, bmpName.data());
      SaveCopy();
    }

    ImGui::Unindent(16.0f);
  }
  if (ImGui::CollapsingHeader("tracer settings"))
  {
    ImGui::Indent(16.0f);
    bool size_changed = false;
    size_changed |= ImGui::InputInt("width##image_width", &tracer->requested_width);
    size_changed |= ImGui::InputInt("height##image_height", &tracer->requested_height);
    if (size_changed)
    {
      uiResized = true;
    }
    const char *items[] = {
      "FULL",
      "NORMAL",
      "DEPTH",
      "DIFFUSE",
      "SIMPLE",
      "POSITION",
      "DEPTH_RATIO",
    };
    if (ImGui::Combo("render_type", &tracer_mode, items, 7, 4))
    {
      ResetTrace();
    }
    if (ImGui::Checkbox("can_receive_input", &tracer->camera.controlsEnabled))
    {
      tracer->camera.PurgeKeys();
    }
    if (ImGui::SliderInt("guiFPS", &maxFPS, 1, 120))
      ResetFPS();
    ImGui::SliderInt("threads", &numThreadsToUse, 1, processorCount);
    ImGui::Checkbox("isPaused", &tracer->isPaused);
    ImGui::Checkbox("gif_render", &wannaTraceGif);
    if (wannaTraceGif)
    {
      if (isTracingGif)
      {
        ImGui::Text(("Trace: " + std::to_string(currentGifTrace)).data());
        ImGui::Text(("Frame: " + std::to_string(currentGifFrame)).data());
        if (ImGui::Button("Stop"))
        {
          isTracingGif = false;
        }
      }
      else
      {
        if (ImGui::Button("Frame0"))
        {
          tracer->TakeSnapshot(0);
        }
        ImGui::SameLine();
        if (ImGui::Button("Frame1"))
        {
          tracer->TakeSnapshot(1);
        }
        ImGui::InputInt("num_gif_frames", &numGifFrames);
        ImGui::InputInt("frame_delay", &gifDelay);
        ImGui::InputInt("traces_per_frame", &tracesPerGifFrame);
        if (ImGui::Button("Start"))
        {
          isTracingGif = true;
          tracer->camera.controlsEnabled = false;
          currentGifTrace = 0;
          gifImages.resize(numGifFrames);
          for (size_t i = 0; i < numGifFrames; i++)
          {
            gifImages[i].Clear();
            gifImages[i].Resize(image.w, image.h);
          }
        }
      }
    }

    ImGui::Unindent(16.0f);
  }
  if (ImGui::CollapsingHeader("camera settings"))
  {
    ImGui::Indent(16.0f);

    if (ImGui::DragFloat3("cam_pos", tracer->camera.position.data(), 0.01f, -10000, 10000, "%.3f"))
    {
      tracer->camera.ResetViews();
      ResetTrace();
    }
    if (ImGui::DragFloat3("cam_rot", tracer->camera.displayRotation.data(), 0.01f, -180, 180, "%.3f"))
    {
      tracer->camera.rotation = EulerToQuat(tracer->camera.displayRotation);
      tracer->camera.ResetViews();
      ResetTrace();
    }

    ImGui::DragFloat("move_speed", &tracer->camera.speedMove, 0.01f);
    ImGui::DragFloat("rot_speed", &tracer->camera.speedRot, 0.01f);
    if (ImGui::Checkbox("use_AA", &tracer->use_AA))
    {
      ResetTrace();
    }
    if (ImGui::Checkbox("depth_of_field", &tracer->depth_of_field))
    {
      ResetTrace();
    }
    if (tracer->depth_of_field)
    {
      ImGui::Indent(10.f);
      if (ImGui::DragFloat("amount", &tracer->camera.w, 0.001f, 0.0, 100, "%.3f"))
        ResetTrace();
      if (ImGui::DragFloat("distance", &tracer->camera.f, 0.01f, 0.f, 10000.f, "%2f"))
        ResetTrace();
      ImGui::Unindent(10.f);
    }
    if (ImGui::Checkbox("halfDome", &tracer->halfDome))
    {
      ResetTrace();
    }
    if (ImGui::DragFloat("fov", &tracer->camera.ry, 0.01f, 0.01f, 1.0f, "%.2f"))
    {
      tracer->camera.UpdateFOV((float)image.w, (float)image.h);
      ResetTrace();
    }

    ImGui::Unindent(16.0f);
  }
  if (ImGui::CollapsingHeader("shapes"))
  {
    ImGui::Indent(16.0f);

    for (size_t i = 0; i < tracer->objects_p.size(); i++)
    {
      if (tracer->objects_p[i]->RenderGenericGUI(i))
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

  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void ResetTrace()
{
  shouldReset = true;
  preview.Clear();
  traceDiff = 100;
}

void InterfaceLoop()
{
  // state bools
  bool isWindowActive = true;
  while (!display->closed)
  {
    if (!tracer->isPaused)
    {
      if (shouldReset)
      {
        image.Clear();

        //this has to happen here due to the potentially different trace times that modes can result in
        tracer->DefaultMode = static_cast<Tracer::TRACE_MODE>(tracer_mode);

        // starting render, disable inputs
        if (tracer->DefaultMode == Tracer::TRACE_MODE::FULL)
        {
          tracer->camera.controlsEnabled = false;
          srand(427857);
        }
        shouldReset = false;
      }

      if (uiResized)
      {
        ResizeImages();
        ResetTrace();
        uiResized = false;
      }

      if (shouldReload)
      {
        SetupScene();
        ResetTrace();
        shouldReload = false;
      }

      auto start_time = std::chrono::high_resolution_clock::now();
      bool update_pass = (image.trace_num - 1) % 10 == 0 || wannaTraceGif || isTracingGif;

      if (isTracingGif)
      {
        //trace each frame once
        for (currentGifFrame = 0; currentGifFrame < numGifFrames; currentGifFrame++)
        {
          //set current state by interpolate between start and end states
          tracer->InterpolateSnapshots((float)(currentGifFrame) / (float)(numGifFrames));
          float diff = tracer->TraceImage(gifImages[currentGifFrame], false, numThreadsToUse);
        }
      }
      else
      {
        float diff = tracer->TraceImage(image, update_pass, numThreadsToUse);
        if (update_pass)
          traceDiff = diff;
      }

      auto end_time = std::chrono::high_resolution_clock::now();
      float ms = static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count());
      traceDuration = ms / 1000000.f;

      if (isTracingGif)
      {
        //re-write gif
        GifBegin(gif_writer, gifName.data(), tracer->requested_width, tracer->requested_height, gifDelay);
        for (size_t i = 0; i < numGifFrames; i++)
        {
          if (!isTracingGif)
            continue;
          //write gif frame
          int width = tracer->requested_width;
          int height = tracer->requested_height;
          std::vector<uint8_t> frame(width * height * 4, 0);

          for (int y = 0; y < height; y++)
          {
            int y_add = y * width;
            for (int x = 0; x < width; x++)
            {
              int pos = y_add + x;
              for (size_t k = 0; k < 4; k++)
              {
                frame[pos * 4 + k] = (uint8_t)(255.f * gifImages[i].data[pos][k]);
              }
            }
          }

          GifWriteFrame(gif_writer, frame.data(), width, height, gifDelay);
        }
        GifEnd(gif_writer);
        currentGifTrace++;
        if (currentGifTrace > numGifFrames)
        {
          // pause trace
          tracer->isPaused = true;
          isTracingGif = false;
        }
      }
      else if (update_pass)
      {
        //just in case...
        generateBitmapImage(image, bmpName.data());
      }
    }
  }
}

void ResetFileName()
{
  sceneName = baseName + ".scn";
  bmpName = baseName + ".bmp";
  gifName = baseName + ".gif";
}

////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{

  tracer = new Tracer();
  display = new Display();
  gif_writer = new GifWriter();

  // Read the command line argument
  baseName = (argc > 1) ? argv[1] : "testscene";
  for (size_t i = 0; i < baseName.size(); i++)
  {
    baseNameArr[i] = baseName[i];
  }
  ResetFileName();

  SetupScene();

  // state bools
  bool autoStart = argc > 1 ? true : false;

  if (autoStart)
    tracer->DefaultMode = Tracer::TRACE_MODE::DIFFUSE;

  // run interface loop
  auto it = std::thread(InterfaceLoop);
  bool previewed_this_frame = false;

  while (!display->closed)
  {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (!display->closed)
    {
      if (!shouldReload)
        tracer->SinglePixelInfoTrace(image, display->mouse_x, display->mouse_y);
      if ((image.trace_num < 2 && tracer->DefaultMode != Tracer::TRACE_MODE::FULL) || shouldReset)
      {
        float diff = tracer->TraceImage(preview, false, numThreadsToUse);
        display->DrawArray(preview);
        previewed_this_frame = true;
      }
      else
      {
        display->DrawArray(image);
      }
      DrawGUI();
      display->FinishDrawing();
    }

    display->UpdateEvent();

    if (display->clickRequest)
    {
      if (display->real_mouse_x < display->window_width - display->gui_width)
        singleTrace = tracer->SinglePixelDebugTrace(image, display->mouse_x, display->mouse_y);
      display->clickRequest = false;
    }

    if (display->active && !isEnteringText && tracer->camera.controlsEnabled)
    {
      if (tracer->camera.Update())
        ResetTrace();
    }

    auto end_time = std::chrono::high_resolution_clock::now();

    float ms = static_cast<float>(std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count());

    float ms_to_wait = std::max(0.f, maxMsToWait - ms);

    guiFPS = 1000 / std::max(ms, maxMsToWait);

    if (previewed_this_frame)
    {
      //negative->make smaller, 0->no change, 100->make bigger
      totalFromMsHistory += ms_to_wait;
      msToWaitHistory.push_back(ms_to_wait);
      if (msToWaitHistory.size() > 100)
      {
        totalFromMsHistory -= msToWaitHistory.front();
        msToWaitHistory.pop_front();
      }
      float avg_ms_to_wait = totalFromMsHistory / msToWaitHistory.size();
      float ideal_ratio = std::max(min_ratio, std::min(1.f, avg_ms_to_wait / maxMsToWait));

      //lerp to smooth variance
      float suggested_ratio = (ideal_ratio + previewRatio) / 2;

      //as # preview traces increases, stop caring about traces
      if (std::abs(previewRatio - suggested_ratio) > 0.02f + 0.001f * (float)preview.trace_num)
      {
        previewRatio = suggested_ratio;
        ResizePreview();
      }
      previewed_this_frame = false;
    }

    Sleep((DWORD)ms_to_wait);
  }

  // wait for the last render to finish
  it.join();

  // one for the road
  generateBitmapImage(image, bmpName.data());

  delete tracer;
  delete display;
}