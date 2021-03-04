////////////////////////////////////////////////////////////////////////////////
// Temporary code.  Remove this from your raytracer.  This displays
// the contents of a scene file in realtime in a GLUT window.
////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <string>
#include <fstream>
#include <vector>

#include <glbinding/gl/gl.h>
#include <glbinding/Binding.h>
using namespace gl;
#include <freeglut.h>

////////////////////////////////////////////////////////////////////////
// Realtime handles all realtime drawing/interaction
////////////////////////////////////////////////////////////////////////
class Realtime
{
public:

    int width, height;
    int displaywindow, consolewindow;
    bool closed;
    HWND display_handle;
    void setScreen(const int _width, const int _height) { width=_width;  height=_height; }
    bool isWindowActive();

    void DrawArray(std::vector<Color> &image);
    void UpdateEvent();
    void ReshapeWindow(int w, int h);
    void RequestResize(int w, int h);

    Realtime(int w, int h);
    void setup();
};


