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
void WriteHdrImage(const std::string outName, const int width, const int height, Color* image)
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

void ClearImage(Color *image, int w, int h, int &count)
{
  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++)
      image[y * w + x] = Color(0, 0, 0);
  count = 1;
}

Eigen::Quaternionf rot_from_euler(float x, float y, float z)
{
  Eigen::Quaternionf r = 
      Eigen::AngleAxisf(x, Eigen::Vector3f::UnitX())
    * Eigen::AngleAxisf(y, Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(z, Eigen::Vector3f::UnitZ());
  return r.normalized();
}

#include <iostream>
#include <chrono>

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

void PurgeUsedKeys()
{
  GetAsyncKeyState(0x41);
  GetAsyncKeyState(0x44);
  GetAsyncKeyState(0x45);
  GetAsyncKeyState(0x51);
  GetAsyncKeyState(0x53);
  GetAsyncKeyState(0x57);
  GetAsyncKeyState(0x4A);
  GetAsyncKeyState(0x4C);
  GetAsyncKeyState(0x4B);
  GetAsyncKeyState(0x49);
  GetAsyncKeyState(0x4F);
  GetAsyncKeyState(0x55);
  GetAsyncKeyState(0x30);
  GetAsyncKeyState(0x31);
  GetAsyncKeyState(0x32);
  GetAsyncKeyState(0x33);
  GetAsyncKeyState(0x34);
  GetAsyncKeyState(0x50);
  GetAsyncKeyState(0x59);
}

void SetInput(bool &can_input, bool val)
{
  can_input = val;
  if (can_input)
  {
    PurgeUsedKeys();
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
    bool autoStart = argc > 1 ? true : false;
    std::string hdrName = inName;
    hdrName.replace(hdrName.size() - 3, hdrName.size(), "hdr");


    SetupScene(scene, inName, true);

    // Allocate and clear an image array
    int trace_num;
    Color *image =  new Color[scene->width*scene->height];
    ClearImage(image, scene->width, scene->height, trace_num);
    trace_num = 1;

    scene->realtime->setup();

    float s = 0.1f;
    float r_s = 0.05f;

    bool can_receive_input = false;

    if (autoStart)
      scene->DefaultMode = Scene::DEBUG_MODE::NONE;

    bool isWindowActive = GetConsoleWindow() == GetForegroundWindow();

    while (!(GetAsyncKeyState(VK_ESCAPE) && GetAsyncKeyState(VK_SHIFT) && isWindowActive))
    {
      isWindowActive = GetConsoleWindow() == GetForegroundWindow() || scene->realtime->isWindowActive();
      bool update_pass = trace_num < 10 || (trace_num - 1) % 10 == 0 || (GetAsyncKeyState(VK_SPACE) && isWindowActive);
      float diff = scene->TraceImage(image, trace_num++, update_pass);
      if (update_pass)
      {
        //just in case...
        scene->realtime->DrawArray(image);
        WriteHdrImage(hdrName, scene->width, scene->height, image);
        if (diff < 0.0000001f && autoStart)
          break;
      }

      scene->realtime->UpdateEvent();

      if (GetAsyncKeyState(VK_F10) && isWindowActive)
        SetInput(can_receive_input, !can_receive_input);

      if (can_receive_input && isWindowActive)
      {
        bool reset = false;
        if (GetAsyncKeyState(0x41)) { scene->MoveCamera(Eigen::Vector3f(-s, 0, 0)); reset = true; } //"A"
        if (GetAsyncKeyState(0x44)) { scene->MoveCamera(Eigen::Vector3f( s, 0, 0)); reset = true; } //"D"
        if (GetAsyncKeyState(0x45)) { scene->MoveCamera(Eigen::Vector3f(0, 0, -s)); reset = true; } //"E"
        if (GetAsyncKeyState(0x51)) { scene->MoveCamera(Eigen::Vector3f(0, 0,  s)); reset = true; } //"Q"
        if (!GetAsyncKeyState(VK_CONTROL) && GetAsyncKeyState(0x53)) { scene->MoveCamera(Eigen::Vector3f(0, -s, 0)); reset = true; } //"S"
        if (GetAsyncKeyState(0x57) && isWindowActive) { scene->MoveCamera(Eigen::Vector3f(0,  s, 0)); reset = true; } //"W"

        if (GetAsyncKeyState(0x4A)) { scene->RotateCamera(rot_from_euler(0, r_s, 0)); reset = true; } // "J"
        if (GetAsyncKeyState(0x4C)) { scene->RotateCamera(rot_from_euler(0, -r_s, 0)); reset = true; } // "L"
        if (GetAsyncKeyState(0x4B)) { scene->RotateCamera(rot_from_euler(-r_s, 0, 0)); reset = true; } // "K"
        if (GetAsyncKeyState(0x49)) { scene->RotateCamera(rot_from_euler(r_s, 0, 0)); reset = true; } // "I"
        if (GetAsyncKeyState(0x4F)) { scene->RotateCamera(rot_from_euler(0, 0, -r_s)); reset = true; } // "O"
        if (GetAsyncKeyState(0x55)) { scene->RotateCamera(rot_from_euler(0, 0, r_s)); reset = true; } // "U"

        if (GetAsyncKeyState(0x30)) { scene->DefaultMode = Scene::DEBUG_MODE::NONE;    reset = true; SetInput(can_receive_input, false); } // "Alphanumeric_0"
        if (GetAsyncKeyState(0x31)) { scene->DefaultMode = Scene::DEBUG_MODE::SIMPLE;  reset = true; } // "Alphanumeric_1"
        if (GetAsyncKeyState(0x32)) { scene->DefaultMode = Scene::DEBUG_MODE::NORMAL;  reset = true; } // "Alphanumeric_2"
        if (GetAsyncKeyState(0x33)) { scene->DefaultMode = Scene::DEBUG_MODE::DEPTH;   reset = true; } // "Alphanumeric_3"
        if (GetAsyncKeyState(0x34)) { scene->DefaultMode = Scene::DEBUG_MODE::DIFFUSE; reset = true; } // "Alphanumeric_4"

        if (GetAsyncKeyState(0x50)) { scene->depth_of_field = !scene->depth_of_field; reset = true; } // "P"

        if (GetAsyncKeyState(0x59)) {  //"Y"
          //reload scene
          if (GetAsyncKeyState(VK_SHIFT))
          {
            SetupScene(scene, inName, true);
            delete[] image;
            image = new Color[scene->width * scene->height];
          }
          else
          {
            SetupScene(scene, inName, false);
          }
          reset = true;
          trace_num = 1;
        }

        if (GetAsyncKeyState(0x6B)) { s *= 1.4; r_s *= 1.4; } //"Add"
        if (GetAsyncKeyState(0x6D)) { s *= 0.6; r_s *= 0.6; } //"Subtract"

        if (GetAsyncKeyState(VK_CONTROL) && GetAsyncKeyState(0x53)) { SaveCopy(inName); RewriteScene(inName, scene); } //"S"

        if (reset)
          ClearImage(image, scene->width, scene->height, trace_num);
      }

    }

    // one for the road
    WriteHdrImage(hdrName, scene->width, scene->height, image);
}

/*
* std::array<std::pair<char*, BYTE>, 336> vkeys
{ {
    { "Backspace", 0x08 },
    { "Tab" , 0x09 },
    { "Clear" , 0x0C },
    { "Enter" , 0x0D },
    { "Shift" , 0x10 },
    { "Control" , 0x11 },
    { "Alt" , 0x12 },
    { "Pause" , 0x13 },
    { "Capslock" , 0x14 },
    { "Kana" , 0x15 },
    { "Hanguel" , 0x15 },
    { "Hangul" , 0x15 },
    { "Junja" , 0x17 },
    { "Final" , 0x18 },
    { "Hanja" , 0x19 },
    { "Kanji" , 0x19 },
    { "Esc" , 0x1B },
    { "Convert" , 0x1C },
    { "NonConvert" , 0x1D },
    { "Accept" , 0x1E },
    { "ModeChange" , 0x1F },
    { "Space" , 0x20 },
    { "PageUp" , 0x21 },
    { "PageDown" , 0x22 },
    { "End" , 0x23 },
    { "Home" , 0x24 },
    { "LeftArrow" , 0x25 },
    { "UpArrow" , 0x26 },
    { "RightArrow" , 0x27 },
    { "DownArrow" , 0x28 },
    { "Select" , 0x29 },
    { "Print" , 0x2A },
    { "Execute" , 0x2B },
    { "PrintScreen" , 0x2C },
    { "Insert" , 0x2D },
    { "Delete" , 0x2E },
    { "Help" , 0x2F },
    { "Alphanumeric_0" , 0x30 },
    { "Alphanumeric_1" , 0x31 },
    { "Alphanumeric_2" , 0x32 },
    { "Alphanumeric_3" , 0x33 },
    { "Alphanumeric_4" , 0x34 },
    { "Alphanumeric_5" , 0x35 },
    { "Alphanumeric_6" , 0x36 },
    { "Alphanumeric_7" , 0x37 },
    { "Alphanumeric_8" , 0x38 },
    { "Alphanumeric_9" , 0x39 },
    { "A" , 0x41 },
    { "B" , 0x42 },
    { "C" , 0x43 },
    { "D" , 0x44 },
    { "E" , 0x45 },
    { "F" , 0x46 },
    { "G" , 0x47 },
    { "H" , 0x48 },
    { "I" , 0x49 },
    { "J" , 0x4A },
    { "K" , 0x4B },
    { "L" , 0x4C },
    { "M" , 0x4D },
    { "N" , 0x4E },
    { "O" , 0x4F },
    { "P" , 0x50 },
    { "Q" , 0x51 },
    { "R" , 0x52 },
    { "S" , 0x53 },
    { "T" , 0x54 },
    { "U" , 0x55 },
    { "V" , 0x56 },
    { "W" , 0x57 },
    { "X" , 0x58 },
    { "Y" , 0x59 },
    { "Z" , 0x5A },
    { "LeftWin" , 0x5B },
    { "RightWin" , 0x5C },
    { "Apps" , 0x5D },
    { "Sleep" , 0x5F },
    { "Numpad_0" , 0x60 },
    { "Numpad_1" , 0x61 },
    { "Numpad_2" , 0x62 },
    { "Numpad_3" , 0x63 },
    { "Numpad_4" , 0x64 },
    { "Numpad_5" , 0x65 },
    { "Numpad_6" , 0x66 },
    { "Numpad_7" , 0x67 },
    { "Numpad_8" , 0x68 },
    { "Numpad_9" , 0x69 },
    { "Multiply" , 0x6A },
    { "Add" , 0x6B },
    { "Separator" , 0x6C },
    { "Subtract" , 0x6D },
    { "Decimal" , 0x6E },
    { "Divide" , 0x6F },
    { "F1" , 0x70 },
    { "F2" , 0x71 },
    { "F3" , 0x72 },
    { "F4" , 0x73 },
    { "F5" , 0x74 },
    { "F6" , 0x75 },
    { "F7" , 0x76 },
    { "F8" , 0x77 },
    { "F9" , 0x78 },
    { "F10" , 0x79 },
    { "F11" , 0x7A },
    { "F12" , 0x7B },
    { "F13" , 0x7C },
    { "F14" , 0x7D },
    { "F15" , 0x7E },
    { "F16" , 0x7F },
    { "F17" , 0x80 },
    { "F18" , 0x81 },
    { "F19" , 0x82 },
    { "F20" , 0x83 },
    { "F21" , 0x84 },
    { "F22" , 0x85 },
    { "F23" , 0x86 },
    { "F24" , 0x87 },
    { "Numlock" , 0x90 },
    { "ScrollLock" , 0x91 },
    { "LeftShift" , 0xA0 },
    { "RightShift" , 0xA1 },
    { "LeftCtrl" , 0xA2 },
    { "RightCtrl" , 0xA3 },
    { "LeftAlt" , 0xA4 },
    { "RightAlt" , 0xA5 },
    { "BrowserBack" , 0xA6 },
    { "BrowserForward" , 0xA7 },
    { "BrowserRefresh" , 0xA8 },
    { "BrowserStop" , 0xA9 },
    { "BrowserSearch" , 0xAA },
    { "BrowserFavorites" , 0xAB },
    { "BrowserHome" , 0xAC },
    { "VolumeMute" , 0xAD },
    { "VolumeDown" , 0xAE },
    { "VolumeUp" , 0xAF },
    { "MediaNextTrack" , 0xB0 },
    { "MediaPrevTrack" , 0xB1 },
    { "MediaStop" , 0xB2 },
    { "MediaPlayPause" , 0xB3 },
    { "LaunchMail" , 0xB4 },
    { "LaunchMedia" , 0xB5 },
    { "LaunchApp1" , 0xB6 },
    { "LaunchApp2" , 0xB7 },
    { "OEM_1" , 0xBA },
    { "OEM_Plus" , 0xBB },
    { "OEM_Comma" , 0xBC },
    { "OEM_Minus" , 0xBD },
    { "OEM_Period" , 0xBE },
    { "OEM_2" , 0xBF },
    { "OEM_3" , 0xC0 },
    { "OEM_4" , 0xDB },
    { "OEM_5" , 0xDC },
    { "OEM_6" , 0xDD },
    { "OEM_7" , 0xDE },
    { "OEM_8" , 0xDF },
    { "OEM_102" , 0xE2 },
    { "ProcessKey" , 0xE5 },
    { "Packet" , 0xE7 },
    { "Attn" , 0xF6 },
    { "CrSel" , 0xF7 },
    { "ExSel" , 0xF8 },
    { "EREOF" , 0xF9 },
    { "Play" , 0xFA },
    { "Zoom" , 0xFB },
    { "NONAME" , 0xFC },
    { "PA1" , 0xFD },
    { "OEM_Clear" , 0xFE },
    { "None" , 0x00 }
} };
*/