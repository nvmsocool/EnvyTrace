///////////////////////////////////////////////////////////////////////
// Provides the framework a raytracer.
//
// Gary Herron
//
// Copyright 2012 DigiPen Institute of Technology
////////////////////////////////////////////////////////////////////////

#include <fstream>
#include <sstream>
#include <vector>
#include <string.h>
#include <ctime>
#include <iostream>
#include <chrono>

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
void WriteHdrImage(const std::string outName, const int width, const int height, std::vector<Color> &image)
{
    // Turn image from a 2D-bottom-up array of Vector3D to an top-down-array of floats
    float* data = new float[width*height*3];
    float* dp = data;
    for (int y=height-1;  y>=0;  --y) {
        for (int x=0;  x<width;  ++x) {
            Color pixel = image[y*width + x];
            *dp++ = pixel[0];
            *dp++ = pixel[1];
            *dp++ = pixel[2]; } }

    // Write image to file in HDR (a.k.a RADIANCE) format
    rgbe_header_info info;
    char errbuf[100] = {0};

    FILE* fp  =  fopen(outName.c_str(), "wb");
    info.valid = false;
    int r = RGBE_WriteHeader(fp, width, height, &info, errbuf);
    if (r != RGBE_RETURN_SUCCESS)
        printf("error: %s\n", errbuf);

    r = RGBE_WritePixels_RLE(fp, data, width,  height, errbuf);
    if (r != RGBE_RETURN_SUCCESS)
        printf("error: %s\n", errbuf);
    fclose(fp);
    
    delete data;
}

void ClearImage(std::vector<Color> &image, int w, int h, int &count)
{
  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++)
      image[y * w + x] = Color(0, 0, 0);
  count = 1;
}

void SaveCopy(std::string &inName)
{
  std::string hdrName = inName;
  std::string hdrBackup = inName;

  const auto p1 = std::chrono::system_clock::now();
  std::string newpostfix = std::to_string(std::chrono::duration_cast<std::chrono::seconds>(p1.time_since_epoch()).count()) + std::string(".hdr");
  hdrBackup.replace(hdrBackup.size() - 4, hdrBackup.size(), newpostfix);
  hdrName.replace(hdrName.size() - 3, hdrName.size(), "hdr");

  system((std::string("copy ") + hdrName + " " + hdrBackup).c_str());
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
    std::cout << "input enabled" << std::endl;
  }
  else
    std::cout << "input disabled" << std::endl;
}

////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
    Scene* scene = new Scene();

    // Read the command line argument
    std::string inName =  (argc > 1) ? argv[1] : "testscene.scn";
    std::string hdrName = inName;
    hdrName.replace(hdrName.size() - 3, hdrName.size(), "hdr");


    SetupScene(scene, inName, true);

    // Allocate and clear an image array
    int trace_num;
    std::vector<Color> image;
    image.resize(scene->width * scene->height);
    ClearImage(image, scene->width, scene->height, trace_num);
    trace_num = 1;

    scene->realtime->setup();

    // camera movement speeds
    float s = 0.1f;
    float r_s = 2.f;

    // state bools
    bool autoStart = argc > 1 ? true : false;
    bool can_receive_input = false;
    bool isWindowActive = true;
    bool isPaused = false;
    bool releasedF10 = true;
    
    bool halfDome = false;

    if (autoStart)
      scene->DefaultMode = Scene::DEBUG_MODE::NONE;

    while (!(IsKeyDown("Esc") && IsKeyDown("Shift") && isWindowActive))
    {
      isWindowActive = GetConsoleWindow() == GetForegroundWindow() || scene->realtime->isWindowActive();
      if (!isPaused)
      {
        bool update_pass = trace_num < 10 || (trace_num - 1) % 10 == 0 || (IsKeyDown("Space") && isWindowActive);
        float diff;
        if (halfDome)
          diff = scene->TraceHalfDome(image, trace_num++, update_pass);
        else
          diff = scene->TraceImage(image, trace_num++, update_pass);
        if (update_pass)
        {
          //just in case...
          scene->realtime->DrawArray(image);
          WriteHdrImage(hdrName, scene->width, scene->height, image);
          if (diff < 0.0000001f && autoStart)
            break;
        }
      }

      scene->realtime->UpdateEvent();

      if (GetAsyncKeyState(VK_F10) && isWindowActive)
      {
        if (releasedF10)
        {
          SetInput(can_receive_input, !can_receive_input);
          releasedF10 = false;
        }
      }
      else
        releasedF10 = true;

      if (can_receive_input && isWindowActive)
      {
        bool reset = false;
        if (IsKeyDown("A")) { scene->MoveCamera(Eigen::Vector3f(-s, 0, 0)); reset = true; }
        if (IsKeyDown("D")) { scene->MoveCamera(Eigen::Vector3f( s, 0, 0)); reset = true; }
        if (IsKeyDown("E")) { scene->MoveCamera(Eigen::Vector3f(0, 0, -s)); reset = true; }
        if (IsKeyDown("Q")) { scene->MoveCamera(Eigen::Vector3f(0, 0,  s)); reset = true; }
        if (IsKeyDown("W")) { scene->MoveCamera(Eigen::Vector3f(0,  s, 0)); reset = true; }
        if (IsKeyDown("S") && !IsKeyDown("Control")) { scene->MoveCamera(Eigen::Vector3f(0, -s, 0)); reset = true; }

        if (IsKeyDown("J")) { scene->RotateCamera(EulerToQuat(Eigen::Vector3f(0,  r_s, 0))); reset = true; }
        if (IsKeyDown("L")) { scene->RotateCamera(EulerToQuat(Eigen::Vector3f(0, -r_s, 0))); reset = true; }
        if (IsKeyDown("K")) { scene->RotateCamera(EulerToQuat(Eigen::Vector3f(-r_s, 0, 0))); reset = true; }
        if (IsKeyDown("I")) { scene->RotateCamera(EulerToQuat(Eigen::Vector3f( r_s, 0, 0))); reset = true; }
        if (IsKeyDown("O")) { scene->RotateCamera(EulerToQuat(Eigen::Vector3f(0, 0, -r_s))); reset = true; }
        if (IsKeyDown("U")) { scene->RotateCamera(EulerToQuat(Eigen::Vector3f(0, 0,  r_s))); reset = true; }

        if (IsKeyDown("Alphanumeric_0")) { scene->DefaultMode = Scene::DEBUG_MODE::NONE;    reset = true; SetInput(can_receive_input, false); }
        if (IsKeyDown("Alphanumeric_1")) { scene->DefaultMode = Scene::DEBUG_MODE::SIMPLE;  reset = true; }
        if (IsKeyDown("Alphanumeric_2")) { scene->DefaultMode = Scene::DEBUG_MODE::NORMAL;  reset = true; }
        if (IsKeyDown("Alphanumeric_3")) { scene->DefaultMode = Scene::DEBUG_MODE::DEPTH;   reset = true; }
        if (IsKeyDown("Alphanumeric_4")) { scene->DefaultMode = Scene::DEBUG_MODE::DIFFUSE; reset = true; }
        if (IsKeyDown("Alphanumeric_5")) { scene->DefaultMode = Scene::DEBUG_MODE::POSITION; reset = true; }


        if (IsKeyDown("F")) { scene->depth_of_field = !scene->depth_of_field; reset = true; }
        if (IsKeyDown("H")) { halfDome = !halfDome; reset = true; }
        if (IsKeyDown("P")) { std::cout << (isPaused ? "unpause" : "pause") << std::endl; isPaused = !isPaused; SetInput(can_receive_input, false); }

        if (IsKeyDown("R")) {
          //reload scene
          SetupScene(scene, inName, GetAsyncKeyState(VK_SHIFT));
          image.resize(scene->width *scene->height);
          reset = true;
        }

        if (IsKeyDown("Add")) { s *= 1.4; r_s *= 1.4; }
        if (IsKeyDown("Subtract")) { s *= 0.6; r_s *= 0.6; }

        if (IsKeyDown("Control") && IsKeyDown("S")) { SaveCopy(inName); RewriteScene(inName, scene); }

        if (reset)
        {
          ClearImage(image, scene->width, scene->height, trace_num);
          scene->ReCalcDirs();
        }
      }

    }

    // one for the road
    WriteHdrImage(hdrName, scene->width, scene->height, image);
}