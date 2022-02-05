#pragma once
#include "ImageData.h"

struct GLFWwindow;

class Display
{
public:
  Display(){};

  GLFWwindow* window;

  int window_width, window_height;
  int render_width, render_height;
  int gui_width{ 350 };
  float top, bottom, left, right;
  int top_i, bottom_i, left_i, right_i;
  int mouse_x, mouse_y, real_mouse_x, real_mouse_y;
  bool closed{ true};
  bool active{ false };
  bool clickRequest{ false };

  void DrawArray(ImageData& id);
  void FinishDrawing();
  void UpdateEvent();
  void ReshapeWindow(int w, int h);
  void SetRenderSize(ImageData& id);
  void CalcImageViewport();

  void SetupWindow(int w, int h);
};