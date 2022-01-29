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

#include "raytrace.h"
#include "realtime.h"
#include "BMP.h"
#include "ImageData.h"

ImageData image, preview;

//tracer variables
Tracer *tracer;
Realtime *realtime;
std::vector<char> baseNameArr(255);
std::string baseName, bmpName;

//tracer state settings
bool shouldReset = false;
bool uiResized = false;
bool isEnteringText = false;
bool shouldReload = false;
std::string sceneName;

float previewRatio = 0.1f;
float min_ratio;

float traceDuration = 0;
float traceDiff = 0;

float guiFPS = 0;

float maxMsToWait = 50.f;

std::list<float> msToWaitHistory;
float totalFromMsHistory = 0;

//may return 0 when not able to detect
const auto processorCount = std::thread::hardware_concurrency();
int numThreadsToUse = processorCount - 1;

void ResetTrace();
void ResetFileName();


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
  realtime->SetRenderSize(image);

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

void SetupScene()
{

  tracer->ClearAll();

  SaveCopy();

  // Read the scene, calling scene.Command for each line.
  ReadScene();

  tracer->Finit();

  realtime->SetupWindow(tracer->requested_width, tracer->requested_height);

  // Allocate and clear an image array
  ResizeImages();
}

void DrawGUI()
{

  //ImGui::ShowDemoWindow();

  // Start the Dear ImGui frame
  ImGui_ImplOpenGL2_NewFrame();
  ImGui_ImplGLUT_NewFrame();

  ImGui::SetNextWindowSize(ImVec2((float)realtime->gui_width, (float)realtime->window_height));
  ImGui::SetNextWindowPos(ImVec2((float)(realtime->window_width - realtime->gui_width), 0.f));

  ImGui::Begin("Fractal Tracer", NULL, ImGuiWindowFlags_HorizontalScrollbar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize); // Create a window called "Hello, world!" and append into it.

  ImGui::Text("GUI (%.1f FPS)", guiFPS);
  ImGui::ProgressBar(image.pctComplete, ImVec2(-1, 0), (std::string("Frame ") + std::to_string(image.trace_num)).data());
  ImGui::Text("%.3fs/trace, conv=%.4f", traceDuration, traceDiff);
  ImGui::Text("Preview Ratio: %.4f", previewRatio);
  if (ImGui::CollapsingHeader("under mouse", ImGuiTreeNodeFlags_DefaultOpen))
  {
    ImGui::Text("pos: (%d,%d)", realtime->mouse_x, realtime->mouse_y);

    ImGui::Text("name: %s", tracer->info_name.data());
    ImGui::Text("distance: %.4f", tracer->info_dist);
    ImGui::Text("position: (%.2f, %.2f, %.2f)", tracer->info_pos[0], tracer->info_pos[1], tracer->info_pos[2]);
  }

  ImGui::BeginChild("settings");

  if (ImGui::CollapsingHeader("scene"))
  {
    ImGui::Indent(16.0f);

    // ImGui::InputText("scene_file", &baseName);
    if (ImGui::InputText("Text", baseNameArr.data(), baseNameArr.size(),
            ImGuiInputTextFlags_CallbackCharFilter,
            [](ImGuiTextEditCallbackData *data) {
              baseName = baseNameArr.data();
              return 0;
            }))
    {
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
      "NONE",
      "NORMAL",
      "DEPTH",
      "DIFFUSE",
      "SIMPLE",
      "POSITION",
    };
    static int item_current = tracer->DefaultMode;
    if (ImGui::Combo("render_type", &item_current, items, 6, 4))
    {
      tracer->DefaultMode = static_cast<Tracer::DEBUG_MODE>(item_current);
      // starting render, disable inputs
      if (tracer->DefaultMode == Tracer::DEBUG_MODE::NONE)
      {
        tracer->camera.controlsEnabled = false;
        srand(427857);
      }
      ResetTrace();
    }
    if (ImGui::Checkbox("can_receive_input", &tracer->camera.controlsEnabled))
    {
      tracer->camera.PurgeKeys();
    }
    ImGui::SliderInt("threads", &numThreadsToUse, 1, processorCount);
    ImGui::Checkbox("isPaused", &tracer->isPaused);
    if (ImGui::Checkbox("halfDome", &tracer->halfDome))
    {
      ResetTrace();
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
    if (ImGui::Checkbox("use_AA", &tracer->use_AA))
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


  ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
}

void ResetTrace()
{
  shouldReset = true;
  preview.Clear();
}

void InterfaceLoop()
{
  // state bools
  bool isWindowActive = true;
  while (!realtime->closed)
  {
    if (!tracer->isPaused)
    {
      if (shouldReset)
      {
        image.Clear();
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

      bool update_pass = (image.trace_num - 1) % 10 == 0;
      float diff = tracer->TraceImage(image, update_pass, numThreadsToUse);
      if (update_pass)
        traceDiff = diff;

      auto end_time = std::chrono::high_resolution_clock::now();
      float ms = static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count());
      traceDuration = ms / 1000000.f;

      if (update_pass)
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
}

////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{

  tracer = new Tracer();
  realtime = new Realtime();

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
  bool isWindowActive = true;

  if (autoStart)
    tracer->DefaultMode = Tracer::DEBUG_MODE::DIFFUSE;

  // run interface loop
  auto it = std::thread(InterfaceLoop);
  bool previewed_this_frame = false;

  while (!realtime->closed)
  {
    auto start_time = std::chrono::high_resolution_clock::now();

    isWindowActive = GetConsoleWindow() == GetForegroundWindow() || realtime->isWindowActive();

    if (!realtime->closed)
    {
      if (!shouldReload)
        tracer->SinglePixelInfoTrace(image, realtime->mouse_x, realtime->mouse_y);
      if ((image.trace_num < 2 && tracer->DefaultMode != Tracer::DEBUG_MODE::NONE) || shouldReset)
      {
        float diff = tracer->TraceImage(preview, false, numThreadsToUse);
        realtime->DrawArray(preview);
        previewed_this_frame = true;
      }
      else
      {
        realtime->DrawArray(image);
      }
      DrawGUI();
      realtime->FinishDrawing();
    }

    realtime->UpdateEvent();

    if (isWindowActive && !isEnteringText && tracer->camera.controlsEnabled)
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
  delete realtime;
}