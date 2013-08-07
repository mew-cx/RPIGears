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
#define _GNU_SOURCE
#include <math.h>
#include <assert.h>
#include <unistd.h>
#include <termio.h>
#include <sys/time.h>

#include "bcm_host.h"

#include "GLES/gl.h"
#include "GLES2/gl2.h"
#include "EGL/egl.h"
#include "EGL/eglext.h"


// number of frames to draw before checking if a key on the keyboard was hit
#define FRAMES 30

#define check() assert(glGetError() == 0)

typedef struct {
  GLfloat pos[3];
  GLfloat norm[3];
  GLfloat texCoords[2];
} vertex_t;

typedef struct {
  vertex_t *vertices;
  GLshort *indices;
  GLfloat color[4];
  int nvertices, nindices;
  GLuint vboId; // ID for vertex buffer object
  GLuint iboId; // ID for index buffer object
  
  GLuint tricount; // number of triangles to draw
  GLvoid *vertex_p; // offset or pointer to first vertex
  GLvoid *normal_p; // offset or pointer to first normal
  GLvoid *index_p;  // offset or pointer to first index
  GLvoid *texCoords_p;  // offset or pointer to first texcoord
} gear_t;


typedef struct
{
   uint32_t screen_width;
   uint32_t screen_height;
// OpenGL|ES objects
   EGLDisplay display;
   EGLSurface surface;
   EGLContext context;
   GLenum drawMode;
// current distance from camera
   GLfloat viewDist;
   GLfloat distance_inc;
// number of seconds to run the demo
   uint timeToRun;
   GLuint texId;

   gear_t *gear1, *gear2, *gear3;
   
// The location of the shader uniforms 
   GLuint ModelViewProjectionMatrix_location,
      ModelViewMatrix_location,
      NormalMatrix_location,
      LightSourcePosition_location,
      MaterialColor_location,
      DiffuseMap_location;
// The projection matrix
   GLfloat ProjectionMatrix[16];
   
// current angle of the gear
   GLfloat angle;
// the degrees that the angle should change each frame
   GLfloat angleFrame;
// the degrees per second the gear should rotate at
   GLfloat angleVel;
// Average Frames Per Second
   float avgfps;
   int useVBO;
   int useGLES2;
   int useVSync;
   int wantInfo;

} CUBE_STATE_T;

static CUBE_STATE_T _state, *state = &_state;

#include "RPi_Logo256.c"

// vertex shader for gles2
static const char vertex_shader[] =
"attribute vec3 position;\n"
"attribute vec3 normal;\n"
"attribute vec2 uv;\n"
"\n"
"uniform mat4 ModelViewMatrix;\n"
"uniform mat4 ModelViewProjectionMatrix;\n"
"uniform mat4 NormalMatrix;\n"
"// light position in view space\n"
"uniform vec4 LightSourcePosition;\n"
"\n"
"varying lowp vec3 L;\n"
"varying lowp vec3 N;\n"
"varying lowp vec3 H;\n"
"varying lowp vec2 oUV;\n"
"\n"
"void main(void)\n"
"{\n"
"    vec4 pos = vec4(position, 1.0);\n"
"   // None of the vectors are normalized until in the fragment shader\n"
"// Calculate the normal vector for this vertex, in view space (\n"
"// multiply by NormalMatrix)\n"
"    N = vec3(NormalMatrix * vec4(normal, 0.0));\n"
"    // Calculate the light vector for this vertex\n"
"    L = vec3(LightSourcePosition - (ModelViewMatrix * pos));\n"
"    // Calculate the view vector\n"
"    lowp vec3 V = vec3(ModelViewMatrix * pos);\n"
"// calculate half angle\n"
"    H = L - V;\n"
"\n"
"    oUV = uv;\n"
"    // Transform the position to clip coordinates\n"
"    gl_Position = ModelViewProjectionMatrix * pos;\n"
"}";

// fragment shader for gles2
static const char fragment_shader[] =
"\n"
"uniform vec4 MaterialColor;\n"
"uniform sampler2D DiffuseMap;\n"
"\n"
"varying lowp vec3 L;\n"
"varying lowp vec3 N;\n"
"varying lowp vec3 H;\n"
"varying lowp vec2 oUV;\n"
"\n"
"void main(void)\n"
"{\n"
"    lowp vec3 l = normalize(L);\n"
"    lowp vec3 n = normalize(N);\n"
"    lowp vec3 h = normalize(H);\n"    
"\n"
"    lowp float diffuse = max(dot(l, n), 0.0);\n"
"    // get bump map vector, again expand from range-compressed\n"
"    vec4 diffCol = texture2D(DiffuseMap, oUV);\n"
"    // modulate diffuseMap with base material color\n"
"    gl_FragColor = vec4(MaterialColor.xyz * diffuse, 1.0) * diffCol;\n"
" //   add  specular\n"
"    // materials that have more red in them are shinnier\n"
"    gl_FragColor += pow(max(0.0, dot(n, h)), 7.0) * diffCol.r;\n"
"}";

uint getMilliseconds()
{
    struct timespec spec;

    clock_gettime(CLOCK_REALTIME, &spec);
	
	return (spec.tv_sec * 1000L + spec.tv_nsec / 1000000L);
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

/** 
 * Copies a 4x4 matrix.
 * 
 * 
 * @param md the destination matrix
 * @param ms the matrix to copy
 */
static void m4x4_copy(GLfloat *md, const GLfloat *ms)
{
   memcpy(md, ms, sizeof(GLfloat) * 16);
}

/** 
 * Multiplies two 4x4 matrices.
 * 
 * The result is stored in matrix m.
 * 
 * @param m the first matrix to multiply
 * @param n the second matrix to multiply
 */
static void m4x4_multiply(GLfloat *m, const GLfloat *n)
{
   GLfloat tmp[16];
   const GLfloat *row, *column;
   div_t d;
   int i, j;

   for (i = 0; i < 16; i++) {
      tmp[i] = 0;
      d = div(i, 4);
      row = n + d.quot * 4;
      column = m + d.rem;
      for (j = 0; j < 4; j++)
         tmp[i] += row[j] * column[j * 4];
   }
   m4x4_copy(m, tmp);
}

/** 
 * Rotates a 4x4 matrix.
 * 
 * @param[in,out] m the matrix to rotate
 * @param angle the angle to rotate in degrees
 * @param x the x component of the direction to rotate to
 * @param y the y component of the direction to rotate to
 * @param z the z component of the direction to rotate to
 */
static void m4x4_rotate(GLfloat *m, GLfloat angle, GLfloat x, GLfloat y, GLfloat z)
{
   float s, c;
   
   angle = 2.0f * M_PI * angle / 360.0f;
   s = sinf(angle);
   c = cosf(angle);

   GLfloat r[16] = {
      x * x * (1 - c) + c,     y * x * (1 - c) + z * s, x * z * (1 - c) - y * s, 0,
      x * y * (1 - c) - z * s, y * y * (1 - c) + c,     y * z * (1 - c) + x * s, 0, 
      x * z * (1 - c) + y * s, y * z * (1 - c) - x * s, z * z * (1 - c) + c,     0,
      0, 0, 0, 1
   };

   m4x4_multiply(m, r);
}


/** 
 * Translates a 4x4 matrix.
 * 
 * @param[in,out] m the matrix to translate
 * @param x the x component of the direction to translate to
 * @param y the y component of the direction to translate to
 * @param z the z component of the direction to translate to
 */
static void m4x4_translate(GLfloat *m, GLfloat x, GLfloat y, GLfloat z)
{
   GLfloat t[16] = { 1, 0, 0, 0,  0, 1, 0, 0,  0, 0, 1, 0,  x, y, z, 1 };

   m4x4_multiply(m, t);
}

/** 
 * Creates an identity 4x4 matrix.
 * 
 * @param m the matrix make an identity matrix
 */
static void m4x4_identity(GLfloat *m)
{
   static const GLfloat t[16] = {
      1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 1.0,
   };

   m4x4_copy(m, t);
}

/** 
 * Transposes a 4x4 matrix.
 *
 * @param m the matrix to transpose
 */
static void m4x4_transpose(GLfloat *m)
{
   const GLfloat t[16] = {
      m[0], m[4], m[8],  m[12],
      m[1], m[5], m[9],  m[13],
      m[2], m[6], m[10], m[14],
      m[3], m[7], m[11], m[15]};

   m4x4_copy(m, t);
}

/**
 * Inverts a 4x4 matrix.
 *
 * This function can currently handle only pure translation-rotation matrices.
 * Read http://www.gamedev.net/community/forums/topic.asp?topic_id=425118
 * for an explanation.
 */
static void m4x4_invert(GLfloat *m)
{
   GLfloat t[16];
   m4x4_identity(t);

   // Extract and invert the translation part 't'. The inverse of a
   // translation matrix can be calculated by negating the translation
   // coordinates.
   t[12] = -m[12]; t[13] = -m[13]; t[14] = -m[14];

   // Invert the rotation part 'r'. The inverse of a rotation matrix is
   // equal to its transpose.
   m[12] = m[13] = m[14] = 0;
   m4x4_transpose(m);

   // inv(m) = inv(r) * inv(t)
   m4x4_multiply(m, t);
}

/** 
 * Calculate a perspective projection transformation.
 * 
 * @param m the matrix to save the transformation in
 * @param fovy the field of view in the y direction
 * @param aspect the view aspect ratio
 * @param zNear the near clipping plane
 * @param zFar the far clipping plane
 */
void m4x4_perspective(GLfloat *m, GLfloat fovy, GLfloat aspect, GLfloat zNear, GLfloat zFar)
{
   GLfloat tmp[16];
   m4x4_identity(tmp);

   float sine, cosine, cotangent, deltaZ;
   GLfloat radians = fovy / 2.0 * M_PI / 180.0;

   deltaZ = zFar - zNear;
   sine = sinf(radians);
   cosine = cosf(radians);

   if ((deltaZ == 0) || (sine == 0) || (aspect == 0))
      return;

   cotangent = cosine / sine;

   tmp[0] = cotangent / aspect;
   tmp[5] = cotangent;
   tmp[10] = -(zFar + zNear) / deltaZ;
   tmp[11] = -1;
   tmp[14] = -2 * zNear * zFar / deltaZ;
   tmp[15] = 0;

   m4x4_copy(m, tmp);
}

/***********************************************************
 * Name: init_egl
 *
 * Arguments:
 *       CUBE_STATE_T *state - holds OGLES model info
 *
 * Description: Sets the display, OpenGL|ES context and screen stuff
 *
 * Returns: void
 *
 ***********************************************************/
static void init_egl(void)
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

  static EGLint context_attributes[] = 
   {
      EGL_CONTEXT_CLIENT_VERSION, 1,
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

   // bind the gles api to this thread - this is default so not required
   result = eglBindAPI(EGL_OPENGL_ES_API);
   assert(EGL_FALSE != result);

   // create an EGL rendering context
   // select es 1.x or 2.x based on user option
   context_attributes[1] = state->useGLES2 + 1;
   state->context = eglCreateContext(state->display, config, EGL_NO_CONTEXT, context_attributes);
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

   // default to no vertical sync but user option may turn it on
   result = eglSwapInterval(state->display, state->useVSync );
   assert(EGL_FALSE != result);

   // Set background color and clear buffers
   glClearColor(0.25f, 0.45f, 0.55f, 1.0f);

   // Enable back face culling.
   glEnable(GL_CULL_FACE);
   glFrontFace(GL_CCW);

   if (state->wantInfo) {
      printf("GL_RENDERER   = %s\n", (char *) glGetString(GL_RENDERER));
      printf("GL_VERSION    = %s\n", (char *) glGetString(GL_VERSION));
      printf("GL_VENDOR     = %s\n", (char *) glGetString(GL_VENDOR));
      printf("GL_EXTENSIONS = %s\n", (char *) glGetString(GL_EXTENSIONS));
	   
   }
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
  GLshort ix0, ix1, ix2, ix3, ix4;
  vertex_t *vt, *nm, *tx;
  GLshort *ix;

  gear_t *gear = calloc(1, sizeof(gear_t));
  gear->nvertices = teeth * 38;
  gear->nindices = teeth * 64 * 3;
  gear->vertices = calloc(gear->nvertices, sizeof(vertex_t));
  gear->indices = calloc(gear->nindices, sizeof(GLshort));
  memcpy(&gear->color[0], &color[0], sizeof(GLfloat) * 4);

  r0 = inner_radius;
  r1 = outer_radius - tooth_depth / 2.0;
  r2 = outer_radius + tooth_depth / 2.0;
  da = 2.0 * M_PI / teeth / 4.0;

  vt = gear->vertices;
  nm = gear->vertices;
  tx = gear->vertices;
  ix = gear->indices;

#define VERTEX(x,y,z) ((vt->pos[0] = x),(vt->pos[1] = y),(vt->pos[2] = z), \
    (tx->texCoords[0] = x / r2 * 0.8 + 0.5),(tx->texCoords[1] = y / r2 * 0.8 + 0.5), (tx++), \
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
    ix0 = VERTEX(r1 * cos_ta,          r1 * sin_ta,          width * 0.5);
    ix1 = VERTEX(r0 * cos_ta,          r0 * sin_ta,          width * 0.5);
    ix2 = VERTEX(r1 * cos_ta_3da,      r1 * sin_ta_3da,      width * 0.5);
    ix3 = VERTEX(r0 * cos_ta_4da,      r0 * sin_ta_4da,      width * 0.5);
    ix4 = VERTEX(r1 * cos_ta_4da,      r1 * sin_ta_4da,      width * 0.5);
    for (j = 0; j < 5; j++) {
      NORMAL(0.0,                  0.0,                  1.0);
    }
    INDEX(ix0, ix2, ix1);
    INDEX(ix1, ix2, ix3);
    INDEX(ix2, ix4, ix3);

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
    ix1 = VERTEX(r1 * cos_ta_3da,      r1 * sin_ta_3da,      -width * 0.5);
    ix2 = VERTEX(r0 * cos_ta,          r0 * sin_ta,          -width * 0.5);
    ix3 = VERTEX(r1 * cos_ta_4da,      r1 * sin_ta_4da,      -width * 0.5);
    ix4 = VERTEX(r0 * cos_ta_4da,      r0 * sin_ta_4da,      -width * 0.5);
    for (j = 0; j < 5; j++) {
      NORMAL(0.0,                  0.0,                  -1.0);
    }
    INDEX(ix0, ix2, ix1);
    INDEX(ix1, ix2, ix3);
    INDEX(ix2, ix4, ix3);
    
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

  // setup pointers/offsets for draw operations
  if (state->useVBO) {
	// for VBO use offsets into the buffer object
    gear->vertex_p = 0;
    gear->normal_p = (GLvoid *)sizeof(gear->vertices[0].pos);
    gear->texCoords_p = (GLvoid *)(sizeof(gear->vertices[0].pos) + sizeof(gear->vertices[0].norm));
    gear->index_p = 0;
  }
  else {
	// for Vertex Array use pointers to where the buffer starts
    gear->vertex_p = gear->vertices[0].pos;
    gear->normal_p = gear->vertices[0].norm;
    gear->texCoords_p = gear->vertices[0].texCoords;
    gear->index_p = gear->indices;
  }
  
  gear->tricount = gear->nindices / 3;

  return gear;
}



static GLfloat view_rotx = 25.0, view_roty = 30.0, view_rotz = 0.0;
/**
 * Draws a gear in GLES 2 mode.
 *
 * @param gear the gear to draw
 * @param transform the current transformation matrix
 * @param x the x position to draw the gear at
 * @param y the y position to draw the gear at
 * @param angle the rotation angle of the gear
 * @param color the color of the gear
 */
static void draw_gearGLES2(gear_t *gear, GLfloat *transform,
      GLfloat x, GLfloat y, GLfloat angle)
{
   // The direction of the directional light for the scene */
   static const GLfloat LightSourcePosition[4] = { 5.0, 5.0, 10.0, 1.0};

   GLfloat model_view[16];
   GLfloat normal_matrix[16];
   GLfloat model_view_projection[16];

   /* Translate and rotate the gear */
   m4x4_copy(model_view, transform);
   m4x4_translate(model_view, x, y, 0);
   m4x4_rotate(model_view, angle, 0, 0, 1);

   /* Create and set the ModelViewProjectionMatrix */
   m4x4_copy(model_view_projection, state->ProjectionMatrix);
   m4x4_multiply(model_view_projection, model_view);

   glUniformMatrix4fv(state->ModelViewProjectionMatrix_location, 1, GL_FALSE,
                      model_view_projection);
   glUniformMatrix4fv(state->ModelViewMatrix_location, 1, GL_FALSE,
                      model_view);
   /* Set the LightSourcePosition uniform in relation to the object */
   glUniform4fv(state->LightSourcePosition_location, 1, LightSourcePosition);

   glUniform1i(state->DiffuseMap_location, 0);

   /* 
    * Create and set the NormalMatrix. It's the inverse transpose of the
    * ModelView matrix.
    */
   m4x4_copy(normal_matrix, model_view);
   m4x4_invert(normal_matrix);
   m4x4_transpose(normal_matrix);
   glUniformMatrix4fv(state->NormalMatrix_location, 1, GL_FALSE, normal_matrix);

   /* Set the gear color */
   glUniform4fv(state->MaterialColor_location, 1, gear->color);

   if (state->useVBO) {
     glBindBuffer(GL_ARRAY_BUFFER, gear->vboId);
  	 glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gear->iboId);
   }

   /* Set up the position of the attributes in the vertex buffer object */
   // setup where vertex data is
   glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
         sizeof(vertex_t), gear->vertex_p);
   // setup where normal data is
   glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
         sizeof(vertex_t), gear->normal_p);
   // setup where uv data is
   glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE,
         sizeof(vertex_t), gear->texCoords_p);

   /* Enable the attributes */
   glEnableVertexAttribArray(0);
   glEnableVertexAttribArray(1);
   glEnableVertexAttribArray(2);

   // Bind texture surface to current vertices
   glBindTexture(GL_TEXTURE_2D, state->texId);
    
   glDrawElements(state->drawMode, gear->tricount, GL_UNSIGNED_SHORT,
                   gear->index_p);

   /* Disable the attributes */
   glDisableVertexAttribArray(2);
   glDisableVertexAttribArray(1);
   glDisableVertexAttribArray(0);
 
}

static void init_textures(void)
{
   // load a texture buffer but use them on six OGL|ES texture surfaces
   glGenTextures(1, &state->texId);

   // setup texture
   glBindTexture(GL_TEXTURE_2D, state->texId);
   glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, rpi_image.width, rpi_image.height, 0,
                GL_RGB, GL_UNSIGNED_BYTE, rpi_image.pixel_data);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
   
}

/** 
 * Draws the gears in GLES 2 mode.
 */
static void draw_sceneGLES2(void)
{
   GLfloat transform[16];
   m4x4_identity(transform);

   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

   /* Translate and rotate the view */
   m4x4_translate(transform, 0.9, 0.0, -state->viewDist);
   m4x4_rotate(transform, view_rotx, 1, 0, 0);
   m4x4_rotate(transform, view_roty, 0, 1, 0);
   m4x4_rotate(transform, view_rotz, 0, 0, 1);

   /* Draw the gears */
   draw_gearGLES2(state->gear1, transform, -3.0, -2.0, state->angle);
   draw_gearGLES2(state->gear2, transform, 3.1, -2.0, -2 * state->angle - 9.0);
   draw_gearGLES2(state->gear3, transform, -3.1, 4.2, -2 * state->angle - 25.0);
}

void draw_gearGLES1(gear_t* gear, GLfloat x, GLfloat y, GLfloat angle)
{

  glPushMatrix();
  glTranslatef(x, y, 0.0);
  glRotatef(angle, 0.0, 0.0, 1.0);
	
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, gear->color);
  
  if (state->useVBO) {
	glBindBuffer(GL_ARRAY_BUFFER, gear->vboId);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gear->iboId);
  }
  
  glNormalPointer(GL_FLOAT, sizeof(vertex_t), gear->normal_p);
  glVertexPointer(3, GL_FLOAT, sizeof(vertex_t), gear->vertex_p);
  glTexCoordPointer(2, GL_FLOAT, sizeof(vertex_t), gear->texCoords_p);

  // Bind texture surface to current vertices
  glBindTexture(GL_TEXTURE_2D, state->texId);
    
  glDrawElements(state->drawMode, gear->tricount, GL_UNSIGNED_SHORT,
                   gear->index_p);
  glPopMatrix();
  
}

static void draw_sceneGLES1(void)
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glPushMatrix();

    glTranslatef(0.9, 0.0, -state->viewDist);

    glRotatef(view_rotx, 1.0, 0.0, 0.0);
    glRotatef(view_roty, 0.0, 1.0, 0.0);
    glRotatef(view_rotz, 0.0, 0.0, 1.0);

    draw_gearGLES1(state->gear1, -3.0, -2.0, state->angle);
    draw_gearGLES1(state->gear2, 3.1, -2.0, -2.0 * state->angle - 9.0);
    draw_gearGLES1(state->gear3, -3.1, 4.2, -2.0 * state->angle - 25.0);

  glPopMatrix();
}

static void update_angleFrame(void)
{
	state->angleFrame = state->angleVel / state->avgfps;
}

static void setup_user_options(int argc, char *argv[])
{
  int i, printhelp = 0;

  // setup some default states
  state->viewDist = 18.0;
  state->avgfps = 300;
  state->angleVel = 70;
  state->useVBO = 0;
  state->drawMode = GL_TRIANGLES;

  for ( i=1; i<argc; i++ ) {
    if (strcmp(argv[i], "-info")==0) {
	  state->wantInfo = 1;
    }
    else if ( strcmp(argv[i], "-exit")==0) {
      state->timeToRun = 30000;
      printf("Auto Exit after %i seconds.\n", state->timeToRun/1000 );
    }
    else if ( strcmp(argv[i], "-vsync")==0) {
      // want vertical sync
      state->useVSync = 1;
      state->avgfps = 60;
    }
    else if ( strcmp(argv[i], "-vbo")==0) {
	  // use VBO instead of Vertex Array
	  state->useVBO = 1;
	}
    else if ( strcmp(argv[i], "-gles2")==0) {
	  // use opengl es 2.0
	  state->useGLES2 = 1;
	}
    else if ( strcmp(argv[i], "-line")==0) {
	  // use line mode draw ie wire mesh
	  state->drawMode = GL_LINES;
	}
    else if ( strcmp(argv[i], "-nospin")==0) {
	  // gears don't spin
	  state->angleVel = 0.0f;
	}
    else {
	  printf("\nunknown option: %s\n", argv[i]);
      printhelp = 1;
    }
  }

  if (printhelp) {
    printf("\nusage: RPIGears [options]\n");
    printf("options: -vsync | -exit | -info | -vbo | -gles2 | -line | -nospin\n");
    printf("-vsync: wait for vertical sync before new frame is displayed\n");
    printf("-exit: automatically exit RPIGears after 30 seconds\n");
    printf("-info: display opengl driver info\n");
    printf("-vbo: use vertex buffer object in GPU memory\n");
    printf("-gles2: use opengl es 2.0\n");
    printf("-line: draw lines only, wire frame mode\n");
    printf("-nospin: gears don't turn\n");

  }
  
  update_angleFrame();
  
}

static void make_gear_vbo(gear_t *gear)
{
   // setup the vertex buffer that will hold the vertices and normals 
   glGenBuffers(1, &gear->vboId);
   glBindBuffer(GL_ARRAY_BUFFER, gear->vboId);
   glBufferData(GL_ARRAY_BUFFER, sizeof(vertex_t) * gear->nvertices, gear->vertices, GL_STATIC_DRAW);
   
   // setup the index buffer that will hold the indices
   glGenBuffers(1, &gear->iboId);
   glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gear->iboId);
   glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLshort) * gear->nindices, gear->indices, GL_STATIC_DRAW);
   	
}

static void init_scene_GLES2(void)
{
   GLuint v, f, program;
   const char *p;
   char msg[512];


   glEnable(GL_CULL_FACE);
   glEnable(GL_DEPTH_TEST);

   /* Compile the vertex shader */
   p = vertex_shader;
   v = glCreateShader(GL_VERTEX_SHADER);
   glShaderSource(v, 1, &p, NULL);
   glCompileShader(v);
   glGetShaderInfoLog(v, sizeof msg, NULL, msg);
   printf("vertex shader info: %s\n", msg);

   /* Compile the fragment shader */
   p = fragment_shader;
   f = glCreateShader(GL_FRAGMENT_SHADER);
   glShaderSource(f, 1, &p, NULL);
   glCompileShader(f);
   glGetShaderInfoLog(f, sizeof msg, NULL, msg);
   printf("fragment shader info: %s\n", msg);

   /* Create and link the shader program */
   program = glCreateProgram();
   glAttachShader(program, v);
   glAttachShader(program, f);
   glBindAttribLocation(program, 0, "position");
   glBindAttribLocation(program, 1, "normal");
   glBindAttribLocation(program, 2, "uv");

   glLinkProgram(program);
   glGetProgramInfoLog(program, sizeof msg, NULL, msg);
   printf("info: %s\n", msg);

   /* Enable the shaders */
   glUseProgram(program);

   /* Get the locations of the uniforms so we can access them */
   state->ModelViewProjectionMatrix_location = glGetUniformLocation(program, "ModelViewProjectionMatrix");
   state->ModelViewMatrix_location = glGetUniformLocation(program, "ModelViewMatrix");
   state->NormalMatrix_location = glGetUniformLocation(program, "NormalMatrix");
   state->LightSourcePosition_location = glGetUniformLocation(program, "LightSourcePosition");
   state->MaterialColor_location = glGetUniformLocation(program, "MaterialColor");
   state->DiffuseMap_location = glGetUniformLocation(program, "DiffuseMap");

}

static void init_scene_GLES1()
{
  const GLfloat light_pos[4] = {5.0, 5.0, 10.0, 1.0};


  glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
  glEnable(GL_CULL_FACE);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_DEPTH_TEST);

  glShadeModel(GL_SMOOTH);

  // vertex and normal array will always be used so do it here once  
  glEnableClientState(GL_NORMAL_ARRAY);
  glEnableClientState(GL_VERTEX_ARRAY);

  // setup overall texture environment
  glEnableClientState(GL_TEXTURE_COORD_ARRAY);
   
  glEnable(GL_TEXTURE_2D);
  // setup blend mode for current bound texture unit
  glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);


}

static void build_gears()
{
  const GLfloat red[4] = {0.9, 0.3, 0.3, 1.0};
  const GLfloat green[4] = {0.3, 0.9, 0.3, 1.0};
  const GLfloat blue[4] = {0.3, 0.3, 0.9, 1.0};

  /* make the meshes for the gears */
  state->gear1 = gear(1.0, 4.0, 2.5, 20, 0.7, red);
  state->gear2 = gear(0.5, 2.0, 3.0, 10, 0.7, green);
  state->gear3 = gear(1.3, 2.0, 1.5, 10, 0.7, blue);
  
  // if VBO enabled then set them up for each gear
  if (state->useVBO) {
    make_gear_vbo(state->gear1);
    make_gear_vbo(state->gear2);
    make_gear_vbo(state->gear3);
  }

  
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
  const uint ttr = state->timeToRun;
  const uint st = getMilliseconds();
  uint ct = st;
  uint prevct = ct, seconds = st;
  float dt;
  float fps;
  int   frames = 0;
  int   active = FRAMES;

  // keep doing the loop while no key hit and ttr
  // is either 0 or time since start is less than time to run (ttr)
  while ( active && ((ttr == 0) || ((ct - st) < ttr)) )
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
    if (state->useGLES2) {
	  draw_sceneGLES2();
	}
	else {
      draw_sceneGLES1();
    }
    
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

static void init_model_projGLES2(void)
{
   /* Update the projection matrix */
   m4x4_perspective(state->ProjectionMatrix, 45.0, (float)state->screen_width / (float)state->screen_height, 1.0, 50.0);
   glViewport(0, 0, (GLsizei)state->screen_width, (GLsizei)state->screen_height);
	
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
static void init_model_projGLES1(void)
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
	 if (gear->vboId) {
	   glDeleteBuffers(1, &gear->vboId);
	 }
	 if (gear->iboId) {
	   glDeleteBuffers(1, &gear->iboId);
	 }
     free(gear->vertices);
     free(gear->indices);
     free(gear);
   }
}

//------------------------------------------------------------------------------

static void exit_func(void)
// Function to be passed to atexit().
{
   if (!state->useGLES2) {
     glDisableClientState(GL_NORMAL_ARRAY);
     glDisableClientState(GL_VERTEX_ARRAY);
   }

   glBindBuffer(GL_ARRAY_BUFFER, 0);
   glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

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

   setup_user_options(argc, argv);

   // Start OGLES
   init_egl();
   init_textures();
   build_gears();
   
   // setup the scene based on rendering mode
   if (state->useGLES2) {
	 init_scene_GLES2();
     // Setup the model projection/world
     init_model_projGLES2();
   }
   else { // using gles1
     init_scene_GLES1();
     // Setup the model projection/world
     init_model_projGLES1();
   }

   // animate the gears
   run_gears();

   exit_func();

   return 0;
}
