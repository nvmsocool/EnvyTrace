////////////////////////////////////////////////////////////////////////////////
// Temporary code.  Remove this from your raytracer.  This displays
// the contents of a scene file in realtime in a GLUT window.
////////////////////////////////////////////////////////////////////////////////

#include <string>
#include <fstream>
#include <vector>

#include "geom.h"
#include "raytrace.h"
#include "realtime.h"

static const char *WINDOW_NAME = "EnvyTrace";
static const LPCWSTR WINDOW_NAME_WSTR = L"EnvyTrace";

// Stupid C++ needs callbacks to be static functions.
static Realtime* globalRealtime = nullptr;
void CBReshapeWindow(int w, int h)  { globalRealtime->ReshapeWindow(w,h); }

void CloseFunc()
{
  globalRealtime->closed = true;
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
    char* argv;
    closed = false;
    
    glutInit(&argc, &argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitContextVersion (3, 3);
    glutInitContextProfile(GLUT_COMPATIBILITY_PROFILE);

    setScreen(w, h);
    glutInitWindowSize(w, h);
    glutCreateWindow(WINDOW_NAME);
    glutSetOption((GLenum)GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
    displaywindow = glutGetWindow();
    display_handle = FindWindow(NULL, WINDOW_NAME_WSTR);

    glutReshapeFunc(&CBReshapeWindow);
    glutCloseFunc(&CloseFunc);

    printf("OpenGL Version: %s\n", glGetString(GL_VERSION));
    printf("GLSL Version: %s\n", glGetString(GL_SHADING_LANGUAGE_VERSION));
    printf("Rendered by: %s\n", glGetString(GL_RENDERER));
    fflush(stdout); 

}

void Realtime::setup()
{
  glutReshapeWindow(width, height);
  ReshapeWindow(width, height);
}

bool Realtime::isWindowActive()
{
  return GetForegroundWindow() == FindWindow(NULL, WINDOW_NAME_WSTR) && !closed;
}

void Realtime::DrawArray(std::vector<Color> &image)
{
  if (closed) return;
  glEnable(GL_TEXTURE_2D);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, (int)GL_LINEAR);

  glTexImage2D(
    GL_TEXTURE_2D,
    0,
    (int)GL_RGB,
    width,
    height,
    0,
    GL_RGB,
    GL_FLOAT,
    &image[0][0]
  );

  glBegin(GL_QUADS);
  glTexCoord2f(0.0f, 0.0f); glVertex2f(-1.0, -1.0);
  glTexCoord2f(1.0f, 0.0f); glVertex2f(1.0, -1.0);
  glTexCoord2f(1.0f, 1.0f); glVertex2f(1.0, 1.0);
  glTexCoord2f(0.0f, 1.0f); glVertex2f(-1.0, 1.0);
  glEnd();

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
    if (w && h)
        glViewport(0, 0, w, h);
    width = w;
    height = h;

    // Force a redraw
    glutPostRedisplay();
}

void Realtime::RequestResize(int w, int h)
{
  //destroy and start again
  setScreen(w, h);

  glutDestroyWindow(displaywindow);

  glutInitWindowSize(w, h);
  glutCreateWindow(WINDOW_NAME);
  glutSetOption((GLenum)GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
  glutReshapeWindow(width, height);
  ReshapeWindow(width, height);
  displaywindow = glutGetWindow();
  display_handle = FindWindow(NULL, WINDOW_NAME_WSTR);
}
