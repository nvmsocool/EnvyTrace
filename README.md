# EnvyTrace
A ray tracer for fractals

# About
This is a ray tracer that was written for CS500 at digipen institute. The original framework was developed by Garry Herron.

# Controls
F10: toggles controls

P: Pauses render

SHIFT-ESC: exits the program after writing the last render

QWEASD: controls the camera position

UIOJKL: controls the camera rotation

F: toggles FOV effect

R: re-loads scene file, ignoring camera position
SHIFT-R: re-load scene and camera position

Ctrl-S: saves camera position to scene file.

1/2/3/4/0: enable simple rendering/normal/depth/diffuse/raytracing

# Notes
If there the output from the last run exists, it will be backed up under a timestamped name.

Controls are only interpreted if a single render finishes while the window is active in the frame that a key was pressed.