RPIGears
========

OpengGL ES port of the classic gl gears demo for the Raspberry Pi.

Building
--------
The following files should be in the build directory:

RPIGears.c
Makefile

In a terminal session, change directory to the build directory where the 
RPIGears source file is located.  Use make at the commandline to build
RPIGears.  No special libraries/packages are required to build if using Raspbian.


Running
-------

To run the demo type ./RPIGears.bin.  The demo runs full screen.  To
exit the demo press any key on the keyboard.


Command line Options
--------------------
usage: ./RPIGears.bin [options]
options: -vsync | -exit | -info | -vbo | -gles2

-vsync : wait for vertical sync before new frame is displayed
-exit  : automatically exit RPIGears after 30 seconds
-info  : display opengl driver info
-vbo   : use vertex buffer object in GPU memory
-gles2 : use opengl es 2.0 rendering path (programable shaders)

Options can be used in any combination.
