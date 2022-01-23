////////////////////////////////////////////////////////////////////////////////
// Temporary code.  Remove this from your raytracer.  This displays
// the contents of a scene file in realtime in a GLUT window.
////////////////////////////////////////////////////////////////////////////////

#include <string>
#include <fstream>
#include <vector>

#include "raytrace.h"
#include "realtime.h"



static int g_Time = 0;          // Current time, in milliseconds

bool ImGui_ImplGLUT_Init()
{
  ImGuiIO &io = ImGui::GetIO();

#ifdef FREEGLUT
  io.BackendPlatformName = "imgui_impl_glut (freeglut)";
#else
  io.BackendPlatformName = "imgui_impl_glut";
#endif

  g_Time = 0;

  // Glut has 1 function for characters and one for "special keys". We map the characters in the 0..255 range and the keys above.
  io.KeyMap[ImGuiKey_Tab] = '\t'; // == 9 == CTRL+I
  io.KeyMap[ImGuiKey_LeftArrow] = 256 + GLUT_KEY_LEFT;
  io.KeyMap[ImGuiKey_RightArrow] = 256 + GLUT_KEY_RIGHT;
  io.KeyMap[ImGuiKey_UpArrow] = 256 + GLUT_KEY_UP;
  io.KeyMap[ImGuiKey_DownArrow] = 256 + GLUT_KEY_DOWN;
  io.KeyMap[ImGuiKey_PageUp] = 256 + GLUT_KEY_PAGE_UP;
  io.KeyMap[ImGuiKey_PageDown] = 256 + GLUT_KEY_PAGE_DOWN;
  io.KeyMap[ImGuiKey_Home] = 256 + GLUT_KEY_HOME;
  io.KeyMap[ImGuiKey_End] = 256 + GLUT_KEY_END;
  io.KeyMap[ImGuiKey_Insert] = 256 + GLUT_KEY_INSERT;
  io.KeyMap[ImGuiKey_Delete] = 127;
  io.KeyMap[ImGuiKey_Backspace] = 8;  // == CTRL+H
  io.KeyMap[ImGuiKey_Space] = ' ';
  io.KeyMap[ImGuiKey_Enter] = 13; // == CTRL+M
  io.KeyMap[ImGuiKey_Escape] = 27;
  io.KeyMap[ImGuiKey_KeyPadEnter] = 13; // == CTRL+M
  io.KeyMap[ImGuiKey_A] = 'A';
  io.KeyMap[ImGuiKey_C] = 'C';
  io.KeyMap[ImGuiKey_V] = 'V';
  io.KeyMap[ImGuiKey_X] = 'X';
  io.KeyMap[ImGuiKey_Y] = 'Y';
  io.KeyMap[ImGuiKey_Z] = 'Z';

  return true;
}

void ImGui_ImplGLUT_InstallFuncs()
{
  glutMotionFunc(ImGui_ImplGLUT_MotionFunc);
  glutPassiveMotionFunc(ImGui_ImplGLUT_MotionFunc);
  glutMouseFunc(ImGui_ImplGLUT_MouseFunc);
#ifdef __FREEGLUT_EXT_H__
  glutMouseWheelFunc(ImGui_ImplGLUT_MouseWheelFunc);
#endif
  glutKeyboardFunc(ImGui_ImplGLUT_KeyboardFunc);
  glutKeyboardUpFunc(ImGui_ImplGLUT_KeyboardUpFunc);
  glutSpecialFunc(ImGui_ImplGLUT_SpecialFunc);
  glutSpecialUpFunc(ImGui_ImplGLUT_SpecialUpFunc);
}

void ImGui_ImplGLUT_Shutdown()
{
}

void ImGui_ImplGLUT_NewFrame()
{
  // Setup time step
  ImGuiIO &io = ImGui::GetIO();
  int current_time = GLUT_ELAPSED_TIME;
  int delta_time_ms = (current_time - g_Time);
  if (delta_time_ms <= 0)
    delta_time_ms = 1;
  io.DeltaTime = delta_time_ms / 1000.0f;
  g_Time = current_time;

  // Start the frame
  ImGui::NewFrame();
}

static void ImGui_ImplGLUT_UpdateKeyboardMods()
{
  ImGuiIO &io = ImGui::GetIO();
  int mods = glutGetModifiers();
  io.KeyCtrl = (mods & GLUT_ACTIVE_CTRL) != 0;
  io.KeyShift = (mods & GLUT_ACTIVE_SHIFT) != 0;
  io.KeyAlt = (mods & GLUT_ACTIVE_ALT) != 0;
}

void ImGui_ImplGLUT_KeyboardFunc(unsigned char c, int x, int y)
{
  // Send character to imgui
  //printf("char_down_func %d '%c'\n", c, c);
  ImGuiIO &io = ImGui::GetIO();
  if (c >= 32)
    io.AddInputCharacter((unsigned int)c);

  // Store letters in KeysDown[] array as both uppercase and lowercase + Handle GLUT translating CTRL+A..CTRL+Z as 1..26.
  // This is a hacky mess but GLUT is unable to distinguish e.g. a TAB key from CTRL+I so this is probably the best we can do here.
  if (c >= 1 && c <= 26)
    io.KeysDown[c] = io.KeysDown[c - 1 + 'a'] = io.KeysDown[c - 1 + 'A'] = true;
  else if (c >= 'a' && c <= 'z')
    io.KeysDown[c] = io.KeysDown[c - 'a' + 'A'] = true;
  else if (c >= 'A' && c <= 'Z')
    io.KeysDown[c] = io.KeysDown[c - 'A' + 'a'] = true;
  else
    io.KeysDown[c] = true;
  ImGui_ImplGLUT_UpdateKeyboardMods();
  (void)x; (void)y; // Unused
}

void ImGui_ImplGLUT_KeyboardUpFunc(unsigned char c, int x, int y)
{
  //printf("char_up_func %d '%c'\n", c, c);
  ImGuiIO &io = ImGui::GetIO();
  if (c >= 1 && c <= 26)
    io.KeysDown[c] = io.KeysDown[c - 1 + 'a'] = io.KeysDown[c - 1 + 'A'] = false;
  else if (c >= 'a' && c <= 'z')
    io.KeysDown[c] = io.KeysDown[c - 'a' + 'A'] = false;
  else if (c >= 'A' && c <= 'Z')
    io.KeysDown[c] = io.KeysDown[c - 'A' + 'a'] = false;
  else
    io.KeysDown[c] = false;
  ImGui_ImplGLUT_UpdateKeyboardMods();
  (void)x; (void)y; // Unused
}

void ImGui_ImplGLUT_SpecialFunc(int key, int x, int y)
{
  //printf("key_down_func %d\n", key);
  ImGuiIO &io = ImGui::GetIO();
  if (key + 256 < IM_ARRAYSIZE(io.KeysDown))
    io.KeysDown[key + 256] = true;
  ImGui_ImplGLUT_UpdateKeyboardMods();
  (void)x; (void)y; // Unused
}

void ImGui_ImplGLUT_SpecialUpFunc(int key, int x, int y)
{
  //printf("key_up_func %d\n", key);
  ImGuiIO &io = ImGui::GetIO();
  if (key + 256 < IM_ARRAYSIZE(io.KeysDown))
    io.KeysDown[key + 256] = false;
  ImGui_ImplGLUT_UpdateKeyboardMods();
  (void)x; (void)y; // Unused
}

void ImGui_ImplGLUT_MouseFunc(int glut_button, int state, int x, int y)
{
  ImGuiIO &io = ImGui::GetIO();
  io.MousePos = ImVec2((float)x, (float)y);
  int button = -1;
  if (glut_button == GLUT_LEFT_BUTTON) button = 0;
  if (glut_button == GLUT_RIGHT_BUTTON) button = 1;
  if (glut_button == GLUT_MIDDLE_BUTTON) button = 2;
  if (button != -1 && state == GLUT_DOWN)
    io.MouseDown[button] = true;
  if (button != -1 && state == GLUT_UP)
    io.MouseDown[button] = false;
}

#ifdef __FREEGLUT_EXT_H__
void ImGui_ImplGLUT_MouseWheelFunc(int button, int dir, int x, int y)
{
  ImGuiIO &io = ImGui::GetIO();
  io.MousePos = ImVec2((float)x, (float)y);
  if (dir > 0)
    io.MouseWheel += 1.0;
  else if (dir < 0)
    io.MouseWheel -= 1.0;
  (void)button; // Unused
}
#endif

void ImGui_ImplGLUT_ReshapeFunc(int w, int h)
{
  ImGuiIO &io = ImGui::GetIO();
  io.DisplaySize = ImVec2((float)w, (float)h);
}

void ImGui_ImplGLUT_MotionFunc(int x, int y)
{
  ImGuiIO &io = ImGui::GetIO();
  io.MousePos = ImVec2((float)x, (float)y);
}













static const char *WINDOW_NAME = "EnvyTrace";
static const LPCWSTR WINDOW_NAME_WSTR = L"EnvyTrace";

// Stupid C++ needs callbacks to be static functions.
static Realtime *globalRealtime = nullptr;
void CBReshapeWindow(int w, int h)
{
  globalRealtime->ReshapeWindow(w, h);
}

void DummyDisplayFunc() {};

void CloseFunc()
{
  globalRealtime->closed = true;

  // Cleanup
  ImGui_ImplOpenGL2_Shutdown();
  ImGui_ImplGLUT_Shutdown();
  ImGui::DestroyContext();
}

////////////////////////////////////////////////////////////////////////
// Realtime handles all realtime drawing/interaction
////////////////////////////////////////////////////////////////////////

// Constructor for Realtime.  Initializes OpenGL, GLUT,as well as the
// data elements of the class.
Realtime::Realtime(int w, int h)
{
  // Initialize the OpenGL bindings
  glbinding::Binding::initialize(false);

  globalRealtime = this;
  // Initialize GLUT
  int argc = 0;
  char *argv;
  closed = false;

  glutInit(&argc, &argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitContextVersion(3, 3);
  glutInitContextProfile(GLUT_COMPATIBILITY_PROFILE);

  glutInitWindowSize(w, h);
  glutCreateWindow(WINDOW_NAME);
  glutSetOption((GLenum)GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
  displaywindow = glutGetWindow();
  display_handle = FindWindow(NULL, WINDOW_NAME_WSTR);

  glutDisplayFunc(&DummyDisplayFunc);
  glutReshapeFunc(&CBReshapeWindow);
  glutCloseFunc(&CloseFunc);

  printf("OpenGL Version: %s\n", glGetString(GL_VERSION));
  printf("GLSL Version: %s\n", glGetString(GL_SHADING_LANGUAGE_VERSION));
  printf("Rendered by: %s\n", glGetString(GL_RENDERER));
  fflush(stdout);


  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io;

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();

  // Setup Platform/Renderer backends
  ImGui_ImplGLUT_Init();
  ImGui_ImplGLUT_InstallFuncs();
  ImGui_ImplOpenGL2_Init();

  ReshapeWindow(w, h);

}

bool Realtime::isWindowActive()
{
  return GetForegroundWindow() == FindWindow(NULL, WINDOW_NAME_WSTR) && !closed;
}

void Realtime::DrawArray(ImageData &id, int gui_w)
{

  if (closed) return;
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
    &id.data[0][0]
  );

  float useable_w = (float)(window_width - gui_w);

  float image_ar = (float)id.w / (float)id.h;
  float viewing_ar = useable_w / (float)window_height;

  float top = 1;
  float bottom = -1;
  float left = -1;
  float right = 1;
  if (image_ar > viewing_ar)
  {
    //image is too wide, chop top/bottom
    right = 2 * (useable_w / (useable_w + (float)gui_w)) - 1;
    float shrunken_y = useable_w / image_ar;
    top = 1.f - (1.f - (shrunken_y / (float)window_height)) / 2.f;
    bottom = -top;
  }
  else
  {
    // image is too tall, chop left/right
    float shrunken_x = window_height * image_ar;
    float x_pad = (window_width - (gui_w + shrunken_x)) / 2.f;
    left = 2 * (x_pad/window_width) - 1;
    right = 2 * ((x_pad + shrunken_x) / window_width) - 1;
  }

  glBegin(GL_QUADS);
  //lower left
  glTexCoord2f(0.0f, 0.0f); glVertex2f(left, bottom);
  //lower right
  glTexCoord2f(1.0f, 0.0f); glVertex2f(right, bottom);
  //upper right
  glTexCoord2f(1.0f, 1.0f); glVertex2f(right, top);
  //upper left
  glTexCoord2f(0.0f, 1.0f); glVertex2f(left, top);

  glEnd();


}

void Realtime::FinishDrawing()
{
  glFlush();
  glutSwapBuffers();
}

void Realtime::UpdateEvent()
{
  glutMainLoopEvent();
}

// Called by GLUT when the window size is changed.
void Realtime::ReshapeWindow(int w, int h)
{
  window_width = w;
  window_height = h;
  if (w && h)
    glViewport(0, 0, w, h);
  ImGui_ImplGLUT_ReshapeFunc(w, h);

  // Force a redraw
  glutPostRedisplay();
}
