/*
Copyright (c) 2012, Broadcom Europe Ltd
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
 * Copyright (C) 1999-2001  Brian Paul   All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * BRIAN PAUL BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
 * AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* $XFree86: xc/programs/glxgears/glxgears.c,v 1.2 2001/04/03 15:56:26 dawes Exp $ */

/*
 * This is a port of the infamous "gears" demo to straight GLX (i.e. no GLUT)
 * Port by Brian Paul  23 March 2001
 *
 * Command line options:
 *    -info      print GL implementation information
 *
 */

/* Conversion to use vertex buffer objects by Michael J. Clark */

/* This a port of glxgears to the Raspberry Pi
 * Port (cut and paste code monkey dude) by Jeff Doyle 18 Jul 2013
 *
 */
// three rotating gears rendered with OpenGL|ES 1.1.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <unistd.h>
#include <termio.h>
#include <sys/time.h>

#include "bcm_host.h"

#include "GLES/gl.h"
#include "EGL/egl.h"
#include "EGL/eglext.h"


// number of frames to draw before checking if a key on the keyboard was hit
#define FRAMES 30

#ifndef M_PI
   #define M_PI 3.141592654
#endif


typedef struct {
  GLfloat pos[3];
  GLfloat norm[3];
} vertex_t;

typedef struct {
  vertex_t *vertices;
  GLshort *indices;
  GLfloat color[4];
  int nvertices, nindices;
  GLuint ibo;
} gear_t;


typedef struct
{
   uint32_t screen_width;
   uint32_t screen_height;
// OpenGL|ES objects
   EGLDisplay display;
   EGLSurface surface;
   EGLContext context;
// current distance from camera
   GLfloat viewDist;
   GLfloat distance_inc;
// number of seconds to run the demo
   GLint timeToRun;

   gear_t *gear1, *gear2, *gear3;
// current angle of the gear
   GLfloat angle;
// the degrees that the angle should change each frame
   GLfloat angleFrame;
// the degrees per second the gear should rotate at
   GLfloat angleVel;
// Average Frames Per Second
   float avgfps;

} CUBE_STATE_T;

static CUBE_STATE_T _state, *state=&_state;


unsigned long getMilliseconds()
{
    struct timespec spec;

    clock_gettime(CLOCK_REALTIME, &spec);
	
	return spec.tv_sec * 1000 + spec.tv_nsec / 1000000;
}


// Check to see if a key has been pressed
// returns 0 if no key is being pressed
// returns >0 if a key has been pressed
int _kbhit(void) {
    static const int STDIN = 0;
    static int initialized = 0;

    if (! initialized) {
        // Use termios to turn off line buffering
        struct termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = 1;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    //if (bytesWaiting > 0) printf("key count: %d", bytesWaiting);
    return bytesWaiting;
}

/***********************************************************
 * Name: init_ogl
 *
 * Arguments:
 *       CUBE_STATE_T *state - holds OGLES model info
 *
 * Description: Sets the display, OpenGL|ES context and screen stuff
 *
 * Returns: void
 *
 ***********************************************************/
static void init_ogl(void)
{
   int32_t success = 0;
   EGLBoolean result;
   EGLint num_config;

   static EGL_DISPMANX_WINDOW_T nativewindow;

   DISPMANX_ELEMENT_HANDLE_T dispman_element;
   DISPMANX_DISPLAY_HANDLE_T dispman_display;
   DISPMANX_UPDATE_HANDLE_T dispman_update;
   VC_RECT_T dst_rect;
   VC_RECT_T src_rect;

   static const EGLint attribute_list[] =
   {
      EGL_RED_SIZE, 8,
      EGL_GREEN_SIZE, 8,
      EGL_BLUE_SIZE, 8,
      EGL_ALPHA_SIZE, 8,
      EGL_DEPTH_SIZE, 16,
      EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
      EGL_NONE
   };

   EGLConfig config;

   // get an EGL display connection
   state->display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
   assert(state->display!=EGL_NO_DISPLAY);

   // initialize the EGL display connection
   result = eglInitialize(state->display, NULL, NULL);
   assert(EGL_FALSE != result);

   // get an appropriate EGL frame buffer configuration
   result = eglChooseConfig(state->display, attribute_list, &config, 1, &num_config);
   assert(EGL_FALSE != result);

   // create an EGL rendering context
   state->context = eglCreateContext(state->display, config, EGL_NO_CONTEXT, NULL);
   assert(state->context!=EGL_NO_CONTEXT);

   // create an EGL window surface
   success = graphics_get_display_size(0 /* LCD */, &state->screen_width, &state->screen_height);
   assert( success >= 0 );

   dst_rect.x = 0;
   dst_rect.y = 0;
   dst_rect.width = state->screen_width;
   dst_rect.height = state->screen_height;

   src_rect.x = 0;
   src_rect.y = 0;
   src_rect.width = state->screen_width << 16;
   src_rect.height = state->screen_height << 16;

   dispman_display = vc_dispmanx_display_open( 0 /* LCD */);
   dispman_update = vc_dispmanx_update_start( 0 );

   dispman_element = vc_dispmanx_element_add ( dispman_update, dispman_display,
      0/*layer*/, &dst_rect, 0/*src*/,
      &src_rect, DISPMANX_PROTECTION_NONE, 0 /*alpha*/, 0/*clamp*/, 0/*transform*/);

   nativewindow.element = dispman_element;
   nativewindow.width = state->screen_width;
   nativewindow.height = state->screen_height;
   vc_dispmanx_update_submit_sync( dispman_update );

   state->surface = eglCreateWindowSurface( state->display, config, &nativewindow, NULL );
   assert(state->surface != EGL_NO_SURFACE);

   // connect the context to the surface
   result = eglMakeCurrent(state->display, state->surface, state->surface, state->context);
   assert(EGL_FALSE != result);

   // default to no vertical sync
   result = eglSwapInterval(state->display, 0 );
   assert(EGL_FALSE != result);

   // Set background color and clear buffers
   glClearColor(0.15f, 0.25f, 0.35f, 1.0f);

   // Enable back face culling.
   glEnable(GL_CULL_FACE);

   glMatrixMode(GL_MODELVIEW);
}

/**
 
  Draw a gear wheel.  You'll probably want to call this function when
  building a display list or filling a vertex buffer object
  since we do a lot of trig here.
 
  Input:  inner_radius - radius of hole at center
          outer_radius - radius at center of teeth
          width - width of gear
          teeth - number of teeth
          tooth_depth - depth of tooth
 
 **/

static gear_t* gear( const GLfloat inner_radius, const GLfloat outer_radius,
                     const GLfloat width, const GLint teeth,
                     const GLfloat tooth_depth, const GLfloat color[])
{
  GLint i, j;
  GLfloat r0, r1, r2;
  GLfloat ta, da;
  GLfloat u1, v1, u2, v2, len;
  GLfloat cos_ta, cos_ta_1da, cos_ta_2da, cos_ta_3da, cos_ta_4da;
  GLfloat sin_ta, sin_ta_1da, sin_ta_2da, sin_ta_3da, sin_ta_4da;
  GLshort ix0, ix1, ix2, ix3, ix4, ix5;
  vertex_t *vt, *nm;
  GLshort *ix;

  gear_t *gear = calloc(1, sizeof(gear_t));
  gear->nvertices = teeth * 40;
  gear->nindices = teeth * 66 * 3;
  gear->vertices = calloc(gear->nvertices, sizeof(vertex_t));
  gear->indices = calloc(gear->nindices, sizeof(GLshort));
  memcpy(&gear->color[0], &color[0], sizeof(GLfloat) * 4);

  r0 = inner_radius;
  r1 = outer_radius - tooth_depth / 2.0;
  r2 = outer_radius + tooth_depth / 2.0;
  da = 2.0 * M_PI / teeth / 4.0;

  vt = gear->vertices;
  nm = gear->vertices;
  ix = gear->indices;

#define VERTEX(x,y,z) ((vt->pos[0] = x),(vt->pos[1] = y),(vt->pos[2] = z), \
                       (vt++ - gear->vertices))
#define NORMAL(x,y,z) ((nm->norm[0] = x),(nm->norm[1] = y),(nm->norm[2] = z), \
                       (nm++))
#define INDEX(a,b,c) ((*ix++ = a),(*ix++ = b),(*ix++ = c))

  for (i = 0; i < teeth; i++) {
    ta = i * 2.0 * M_PI / teeth;

    cos_ta = cos(ta);
    cos_ta_1da = cos(ta + da);
    cos_ta_2da = cos(ta + 2 * da);
    cos_ta_3da = cos(ta + 3 * da);
    cos_ta_4da = cos(ta + 4 * da);
    sin_ta = sin(ta);
    sin_ta_1da = sin(ta + da);
    sin_ta_2da = sin(ta + 2 * da);
    sin_ta_3da = sin(ta + 3 * da);
    sin_ta_4da = sin(ta + 4 * da);

    u1 = r2 * cos_ta_1da - r1 * cos_ta;
    v1 = r2 * sin_ta_1da - r1 * sin_ta;
    len = sqrt(u1 * u1 + v1 * v1);
    u1 /= len;
    v1 /= len;
    u2 = r1 * cos_ta_3da - r2 * cos_ta_2da;
    v2 = r1 * sin_ta_3da - r2 * sin_ta_2da;

    /* front face */
    ix0 = VERTEX(r0 * cos_ta,          r0 * sin_ta,          width * 0.5);
    ix1 = VERTEX(r1 * cos_ta,          r1 * sin_ta,          width * 0.5);
    ix2 = VERTEX(r0 * cos_ta,          r0 * sin_ta,          width * 0.5);
    ix3 = VERTEX(r1 * cos_ta_3da,      r1 * sin_ta_3da,      width * 0.5);
    ix4 = VERTEX(r0 * cos_ta_4da,      r0 * sin_ta_4da,      width * 0.5);
    ix5 = VERTEX(r1 * cos_ta_4da,      r1 * sin_ta_4da,      width * 0.5);
    for (j = 0; j < 6; j++) {
      NORMAL(0.0,                  0.0,                  1.0);
    }
    INDEX(ix0, ix1, ix2);
    INDEX(ix1, ix3, ix2);
    INDEX(ix2, ix3, ix4);
    INDEX(ix3, ix5, ix4);

    /* front sides of teeth */
    ix0 = VERTEX(r1 * cos_ta,          r1 * sin_ta,          width * 0.5);
    ix1 = VERTEX(r2 * cos_ta_1da,      r2 * sin_ta_1da,      width * 0.5);
    ix2 = VERTEX(r1 * cos_ta_3da,      r1 * sin_ta_3da,      width * 0.5);
    ix3 = VERTEX(r2 * cos_ta_2da,      r2 * sin_ta_2da,      width * 0.5);
    for (j = 0; j < 4; j++) {
      NORMAL(0.0,                  0.0,                  1.0);
    }
    INDEX(ix0, ix1, ix2);
    INDEX(ix1, ix3, ix2);
    /* back face */
    ix0 = VERTEX(r1 * cos_ta,          r1 * sin_ta,          -width * 0.5);
    ix1 = VERTEX(r0 * cos_ta,          r0 * sin_ta,          -width * 0.5);
    ix2 = VERTEX(r1 * cos_ta_3da,      r1 * sin_ta_3da,      -width * 0.5);
    ix3 = VERTEX(r0 * cos_ta,          r0 * sin_ta,          -width * 0.5);
    ix4 = VERTEX(r1 * cos_ta_4da,      r1 * sin_ta_4da,      -width * 0.5);
    ix5 = VERTEX(r0 * cos_ta_4da,      r0 * sin_ta_4da,      -width * 0.5);
    for (j = 0; j < 6; j++) {
      NORMAL(0.0,                  0.0,                  -1.0);
    }
    INDEX(ix0, ix1, ix2);
    INDEX(ix1, ix3, ix2);
    INDEX(ix2, ix3, ix4);
    INDEX(ix3, ix5, ix4);
 /* back sides of teeth */
    ix0 = VERTEX(r1 * cos_ta_3da,      r1 * sin_ta_3da,      -width * 0.5);
    ix1 = VERTEX(r2 * cos_ta_2da,      r2 * sin_ta_2da,      -width * 0.5);
    ix2 = VERTEX(r1 * cos_ta,          r1 * sin_ta,          -width * 0.5);
    ix3 = VERTEX(r2 * cos_ta_1da,      r2 * sin_ta_1da,      -width * 0.5);

    for (j = 0; j < 4; j++) {
      NORMAL(0.0,                  0.0,                  -1.0);
    }
    INDEX(ix0, ix1, ix2);
    INDEX(ix1, ix3, ix2);

    /* draw outward faces of teeth */
    ix0 = VERTEX(r1 * cos_ta,          r1 * sin_ta,          width * 0.5);
    ix1 = VERTEX(r1 * cos_ta,          r1 * sin_ta,          -width * 0.5);
    ix2 = VERTEX(r2 * cos_ta_1da,      r2 * sin_ta_1da,      width * 0.5);
    ix3 = VERTEX(r2 * cos_ta_1da,      r2 * sin_ta_1da,      -width * 0.5);

    for (j = 0; j < 4; j++) {
      NORMAL(v1,                   -u1,                  0.0);
    }
    INDEX(ix0, ix1, ix2);
    INDEX(ix1, ix3, ix2);
    ix0 = VERTEX(r2 * cos_ta_1da,      r2 * sin_ta_1da,      width * 0.5);
    ix1 = VERTEX(r2 * cos_ta_1da,      r2 * sin_ta_1da,      -width * 0.5);
    ix2 = VERTEX(r2 * cos_ta_2da,      r2 * sin_ta_2da,      width * 0.5);
    ix3 = VERTEX(r2 * cos_ta_2da,      r2 * sin_ta_2da,      -width * 0.5);
    for (j = 0; j < 4; j++) {
      NORMAL(cos_ta,               sin_ta,               0.0);
    }
    INDEX(ix0, ix1, ix2);
    INDEX(ix1, ix3, ix2);
    ix0 = VERTEX(r2 * cos_ta_2da,      r2 * sin_ta_2da,      width * 0.5);
    ix1 = VERTEX(r2 * cos_ta_2da,      r2 * sin_ta_2da,      -width * 0.5);
    ix2 = VERTEX(r1 * cos_ta_3da,      r1 * sin_ta_3da,      width * 0.5);
    ix3 = VERTEX(r1 * cos_ta_3da,      r1 * sin_ta_3da,      -width * 0.5);
    for (j = 0; j < 4; j++) {
      NORMAL(v2,                   -u2,                  0.0);
    }
    INDEX(ix0, ix1, ix2);
    INDEX(ix1, ix3, ix2);
    ix0 = VERTEX(r1 * cos_ta_3da,      r1 * sin_ta_3da,      width * 0.5);
    ix1 = VERTEX(r1 * cos_ta_3da,      r1 * sin_ta_3da,      -width * 0.5);
    ix2 = VERTEX(r1 * cos_ta_4da,      r1 * sin_ta_4da,      width * 0.5);
    ix3 = VERTEX(r1 * cos_ta_4da,      r1 * sin_ta_4da,      -width * 0.5);
    for (j = 0; j < 4; j++) {
      NORMAL(cos_ta,               sin_ta,               0.0);
    }
    INDEX(ix0, ix1, ix2);
    INDEX(ix1, ix3, ix2);
 /* draw inside radius cylinder */
    ix0 = VERTEX(r0 * cos_ta,          r0 * sin_ta,          -width * 0.5);
    ix1 = VERTEX(r0 * cos_ta,          r0 * sin_ta,          width * 0.5);
    ix2 = VERTEX(r0 * cos_ta_4da,      r0 * sin_ta_4da,      -width * 0.5);
    ix3 = VERTEX(r0 * cos_ta_4da,      r0 * sin_ta_4da,      width * 0.5);
    NORMAL(-cos_ta,              -sin_ta,              0.0);
    NORMAL(-cos_ta,              -sin_ta,              0.0);
    NORMAL(-cos_ta_4da,          -sin_ta_4da,          0.0);
    NORMAL(-cos_ta_4da,          -sin_ta_4da,          0.0);
    INDEX(ix0, ix1, ix2);
    INDEX(ix1, ix3, ix2);
  }

  return gear;
}

void draw_gear(gear_t* gear) {

  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, gear->color);
  glVertexPointer(3, GL_FLOAT, sizeof(vertex_t), gear->vertices[0].pos);
  glNormalPointer(GL_FLOAT, sizeof(vertex_t), gear->vertices[0].norm);
  glDrawElements(GL_TRIANGLES, gear->nindices/3, GL_UNSIGNED_SHORT,
                   gear->indices);
}

static GLfloat view_rotx = 25.0, view_roty = 30.0, view_rotz = 0.0;


static void draw_scene(void)
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glPushMatrix();

    glTranslatef(0.9, 0.0, -state->viewDist);

    glRotatef(view_rotx, 1.0, 0.0, 0.0);
    glRotatef(view_roty, 0.0, 1.0, 0.0);
    glRotatef(view_rotz, 0.0, 0.0, 1.0);

    glPushMatrix();
      glTranslatef(-3.0, -2.0, 0.0);
      glRotatef(state->angle, 0.0, 0.0, 1.0);
      draw_gear(state->gear1);
    glPopMatrix();

    glPushMatrix();
      glTranslatef(3.1, -2.0, 0.0);
      glRotatef(-2.0 * state->angle - 9.0, 0.0, 0.0, 1.0);
      draw_gear(state->gear2);
    glPopMatrix();

    glPushMatrix();
      glTranslatef(-3.1, 4.2, 0.0);
      glRotatef(-2.0 * state->angle - 25.0, 0.0, 0.0, 1.0);
      draw_gear(state->gear3);
    glPopMatrix();

  glPopMatrix();
}

static void update_angleFrame(void)
{
	state->angleFrame = state->angleVel / state->avgfps;
}

static void setup_user_options(int argc, char *argv[])
{
  int i, printhelp = 0;
  EGLBoolean result;

  // setup some default states
  state->viewDist = 18.0;
  state->avgfps = 300;
  state->angleVel = 70;

  for ( i=1; i<argc; i++ ) {
    if (strcmp(argv[i], "-info")==0) {
      printf("GL_RENDERER   = %s\n", (char *) glGetString(GL_RENDERER));
      printf("GL_VERSION    = %s\n", (char *) glGetString(GL_VERSION));
      printf("GL_VENDOR     = %s\n", (char *) glGetString(GL_VENDOR));
      printf("GL_EXTENSIONS = %s\n", (char *) glGetString(GL_EXTENSIONS));
    }
    else if ( strcmp(argv[i], "-exit")==0) {
      state->timeToRun = 30;
      printf("Auto Exit after %i seconds.\n", state->timeToRun );
    }
    else if ( strcmp(argv[i], "-vsync")==0) {
      // want vertical sync
      result = eglSwapInterval(state->display, 1 );
      assert(EGL_FALSE != result);
      state->avgfps = 60;
    }
    else {
	  printf("\nunknown option: %s\n", argv[i]);
      printhelp = 1;
    }
  }

  if (printhelp) {
    printf("\nusage: RPIGears [options]\n");
    printf("options: -vsync -exit -info\n");
    printf("-vsync: wait for vertical sync before new frame is displayed\n");
    printf("-exit: automatically exit RPIGears after 30 seconds\n");
    printf("-info: display opengl driver info\n");

  }
  
  update_angleFrame();
  
}


static void init_scene()
{
  const GLfloat light_pos[4] = {5.0, 5.0, 10.0, 0.0};
  const GLfloat red[4] = {0.8, 0.1, 0.0, 1.0};
  const GLfloat green[4] = {0.0, 0.8, 0.2, 1.0};
  const GLfloat blue[4] = {0.2, 0.2, 1.0, 1.0};


  glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
  glEnable(GL_CULL_FACE);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_DEPTH_TEST);

  glShadeModel(GL_SMOOTH);

  glEnableClientState(GL_NORMAL_ARRAY);
  glEnableClientState(GL_VERTEX_ARRAY);


  /* make the gears */
  state->gear1 = gear(1.0, 4.0, 2.5, 20, 0.7, red);
  state->gear2 = gear(0.5, 2.0, 3.0, 10, 0.7, green);
  state->gear3 = gear(1.3, 2.0, 1.5, 10, 0.7, blue);
}

static void update_gear_rotation(void)
{
    /* advance gear rotation for next frame */
    state->angle += state->angleFrame;
    if (state->angle > 360.0)
      state->angle -= 360.0;
}

static void run_gears()
{
  const GLint ttr = state->timeToRun;
  const unsigned long st = getMilliseconds();
  unsigned long ct = st;
  unsigned long prevct = ct, seconds = st;
  float dt;
  float fps;
  int   frames = 0;
  int   active = FRAMES;

  // keep doing the loop while no key hit and ttr
  // is either 0 or time since start is less than time to run (ttr)
  while ( active && ((ttr == 0) || (ct - st < ttr)) )
  {
    ct = getMilliseconds();
    
    frames++;
    dt = (float)(ct - seconds)/1000.0f;
    // adjust angleFrame each half second
    if ((ct - prevct) > 500) {
	  if (dt > 0.0f) {
        state->avgfps = state->avgfps * 0.4f + (float)frames / dt * 0.6f;
	    update_angleFrame();
	  }
	  prevct = ct;	
	}

    update_gear_rotation();

    // draw the scene for the next new frame
    draw_scene();
    // swap the current buffer for the next new frame
    eglSwapBuffers(state->display, state->surface);

    if (dt >= 5.0f)
    {
      fps = (float)frames  / dt;
      printf("%d frames in %3.1f seconds = %3.1f FPS\n", frames, dt, fps);
      seconds = ct;
      frames = 0;
    }
    // once in a while check if the user hit the keyboard
    // stop the program if a key was hit
    if (active-- == 1)
    {
      if (_kbhit()) active = 0;
      else active = FRAMES;
    }

  }
}


/***********************************************************
 * Name: init_model_proj
 *
 * Arguments:
 *       CUBE_STATE_T *state - holds OGLES model info
 *
 * Description: Sets the OpenGL|ES model to default values
 *
 * Returns: void
 *
 ***********************************************************/
static void init_model_proj(void)
{
   // near clipping plane
   const float nearp = 1.0f;
   // far clipping plane
   const float farp = 50.0f;
   float hht;
   float hwd;


   glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );

   glViewport(0, 0, (GLsizei)state->screen_width, (GLsizei)state->screen_height);

   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();

   hht = nearp * (float)tan(45.0 / 2.0 / 180.0 * M_PI);
   hwd = hht * (float)state->screen_width / (float)state->screen_height;
   // set up the viewing frustum
   glFrustumf(-hwd, hwd, -hht, hht, nearp, farp);

   glMatrixMode(GL_MODELVIEW);

}


//------------------------------------------------------------------------------

static void free_gear(gear_t *gear)
{
   if (gear) {
     free(gear->vertices);
     free(gear->indices);
     free(gear);
   }
}

//------------------------------------------------------------------------------

static void exit_func(void)
// Function to be passed to atexit().
{
   // clear screen
   glClear( GL_COLOR_BUFFER_BIT );
   eglSwapBuffers(state->display, state->surface);

   // Release OpenGL resources
   eglMakeCurrent( state->display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT );
   eglDestroySurface( state->display, state->surface );
   eglDestroyContext( state->display, state->context );
   eglTerminate( state->display );

   // release memory used for gear and associated vertex arrays
   free_gear(state->gear1);
   free_gear(state->gear2);
   free_gear(state->gear3);
   
   printf("\nRPIGears finished\n");
   
} // exit_func()

//==============================================================================

int main (int argc, char *argv[])
{
   bcm_host_init();

   // Clear application state
   memset( state, 0, sizeof( *state ) );

   // Start OGLES 1.x
   init_ogl();

   setup_user_options(argc, argv);

   init_scene();

   // Setup the model projection/world
   init_model_proj();

   // animate the gears
   run_gears();

   exit_func();

   return 0;
}

