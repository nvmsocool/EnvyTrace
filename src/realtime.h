#pragma once

#include <fstream>

#include "imgui.h"
#include "backends/imgui_impl_opengl2.h"

#include <glbinding/gl/gl.h>
#include <glbinding/Binding.h>
using namespace gl;
#include <freeglut.h>


IMGUI_IMPL_API bool ImGui_ImplGLUT_Init();
IMGUI_IMPL_API void ImGui_ImplGLUT_InstallFuncs();
IMGUI_IMPL_API void ImGui_ImplGLUT_Shutdown();
IMGUI_IMPL_API void ImGui_ImplGLUT_NewFrame();

// You can call ImGui_ImplGLUT_InstallFuncs() to get all those functions installed automatically,
// or call them yourself from your own GLUT handlers. We are using the same weird names as GLUT for consistency..
//---------------------------------------- GLUT name --------------------------------------------- Decent Name ---------
IMGUI_IMPL_API void ImGui_ImplGLUT_ReshapeFunc(int w, int h);                         // ~ ResizeFunc
IMGUI_IMPL_API void ImGui_ImplGLUT_MotionFunc(int x, int y);                          // ~ MouseMoveFunc
IMGUI_IMPL_API void ImGui_ImplGLUT_MouseFunc(int button, int state, int x, int y);    // ~ MouseButtonFunc
IMGUI_IMPL_API void ImGui_ImplGLUT_MouseWheelFunc(int button, int dir, int x, int y); // ~ MouseWheelFunc
IMGUI_IMPL_API void ImGui_ImplGLUT_KeyboardFunc(unsigned char c, int x, int y);       // ~ CharPressedFunc
IMGUI_IMPL_API void ImGui_ImplGLUT_KeyboardUpFunc(unsigned char c, int x, int y);     // ~ CharReleasedFunc
IMGUI_IMPL_API void ImGui_ImplGLUT_SpecialFunc(int key, int x, int y);                // ~ KeyPressedFunc
IMGUI_IMPL_API void ImGui_ImplGLUT_SpecialUpFunc(int key, int x, int y);              // ~ KeyReleasedFunc



////////////////////////////////////////////////////////////////////////
// Realtime handles all realtime drawing/interaction
////////////////////////////////////////////////////////////////////////
class Realtime
{
public:
  int window_width, window_height;
  int render_width, render_height;
  int gui_width{ 350 };
  float top, bottom, left, right;
  int top_i, bottom_i, left_i, right_i;
  int mouse_x, mouse_y;
  bool closed;
  HWND display_handle;
  bool isWindowActive();

  void DrawArray(ImageData& id);
  void FinishDrawing();
  void UpdateEvent();
  void ReshapeWindow(int w, int h);
  void SetRenderSize(ImageData& id);
  void CalcImageViewport();

  Realtime();

  void SetupWindow(int w, int h);
};
