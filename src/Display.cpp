#include "Display.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW\glfw3.h>

// needed for imgui compatability
#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif


// global callbacks for glfw

static Display *globalDisplay = nullptr;

static void glfw_error_callback(int error, const char *description)
{
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

void ReshapeFunc(GLFWwindow *win, int w, int h)
{
  globalDisplay->ReshapeWindow(w, h);
}

void MoveFunc(GLFWwindow *win, double _x, double _y)
{
  int x = (int)_x;
  int y = (int)_y;
  globalDisplay->mouse_x = (int)((float)globalDisplay->render_width * (float)(x - globalDisplay->left_i) / (float)(globalDisplay->right_i - globalDisplay->left_i));
  globalDisplay->mouse_y = (int)((float)globalDisplay->render_height * (float)(y - globalDisplay->top_i) / (float)(globalDisplay->bottom_i - globalDisplay->top_i));
  globalDisplay->mouse_x = std::max(0, std::min(globalDisplay->mouse_x, globalDisplay->render_width));
  globalDisplay->mouse_y = std::max(0, std::min(globalDisplay->mouse_y, globalDisplay->render_height));
}

void CloseFunc(GLFWwindow *window)
{
  globalDisplay->closed = true;

  // Cleanup
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(globalDisplay->window);
  glfwTerminate();
}

void ActiveFunc(GLFWwindow *window, int focused)
{
  globalDisplay->active = (bool)focused;
}

// class functions

void Display::DrawArray(ImageData &id)
{

  if (closed)
    return;

  glEnable(GL_TEXTURE_2D);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, (int)GL_LINEAR);

  glTexImage2D(
      GL_TEXTURE_2D,
      0,
      (int)GL_RGB,
      id.w,
      id.h,
      0,
      GL_RGB,
      GL_FLOAT,
      &id.data[0][0]);

  glBegin(GL_QUADS);
  //lower left
  glTexCoord2f(0.0f, 0.0f);
  glVertex2f(left, bottom);
  //lower right
  glTexCoord2f(1.0f, 0.0f);
  glVertex2f(right, bottom);
  //upper right
  glTexCoord2f(1.0f, 1.0f);
  glVertex2f(right, top);
  //upper left
  glTexCoord2f(0.0f, 1.0f);
  glVertex2f(left, top);

  glEnd();
}

void Display::FinishDrawing()
{
  glFlush();
  glfwSwapBuffers(window);
}

void Display::UpdateEvent()
{
  glfwPollEvents();
}

void Display::SetRenderSize(ImageData &id)
{
  render_width = id.w;
  render_height = id.h;
  CalcImageViewport();
}

// Called when the window size is changed.
void Display::ReshapeWindow(int w, int h)
{
  // clear everything
  int display_w, display_h;
  glfwGetFramebufferSize(window, &display_w, &display_h);
  glViewport(0, 0, display_w, display_h);
  glClearColor(0,0,0,0);
  glClear(GL_COLOR_BUFFER_BIT);

  glfwSwapBuffers(window);

  //reset view
  window_width = w;
  window_height = h;
  if (w && h)
    glViewport(0, 0, w, h);

  CalcImageViewport();
}

void Display::CalcImageViewport()
{
  //re-calc image render area
  float useable_w = (float)(window_width - gui_width);

  float image_ar = (float)render_width / (float)render_height;
  float viewing_ar = useable_w / (float)window_height;

  top = 1;
  bottom = -1;
  left = -1;

  top_i = 0;
  bottom_i = window_height;
  left_i = 0;

  if (image_ar > viewing_ar)
  {
    //image is too wide, chop top/bottom
    right_i = window_width - gui_width;
    right = 2 * ((float)right_i / (float)window_width) - 1;
    float shrunken_y = useable_w / image_ar;
    float y_pad = (window_height - shrunken_y) / 2.f;
    top_i = (int)y_pad;
    bottom_i = (int)(window_height - y_pad);
    top = -(2 * (y_pad / window_height) - 1);
    bottom = -top;
  }
  else
  {
    // image is too tall, chop left/right
    float shrunken_x = window_height * image_ar;
    float x_pad = (window_width - (gui_width + shrunken_x)) / 2.f;
    left_i = (int)(x_pad);
    right_i = (int)(x_pad + shrunken_x);
    left = 2 * (x_pad / window_width) - 1;
    right = 2 * ((x_pad + shrunken_x) / window_width) - 1;
  }
}

void Display::SetupWindow(int _w, int h)
{
  int w = _w + gui_width;
  // Initialize the OpenGL bindings

  globalDisplay = this;

  // Setup window
  glfwSetErrorCallback(glfw_error_callback);
  if (!glfwInit())
    return;

  const char *glsl_version = "#version 130";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

  // Create window with graphics context
  window = glfwCreateWindow(w, h, "EnvyTrace", NULL, NULL);
  if (window == NULL)
    return;
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1); // Enable vsync

  glfwSetWindowFocusCallback(window, ActiveFunc);
  glfwSetWindowSizeCallback(window, ReshapeFunc);
  glfwSetWindowCloseCallback(window, CloseFunc);
  glfwSetCursorPosCallback(window, MoveFunc);

  ReshapeWindow(w, h);

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io;

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();

  // Setup Platform/Renderer backends
  ImGui_ImplGlfw_InitForOpenGL(window, true);

  ImGui_ImplOpenGL3_Init(glsl_version);

  closed = false;
  active = true;
}
