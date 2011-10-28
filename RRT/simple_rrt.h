#ifndef _simple_rrt_h
#define _simple_rrt_h

#ifdef MAC
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#include <GL/gle.h>
#else
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/gle.h> 
#endif

#define drawFromView(member_func_ptr) \
  glPushMatrix(); \
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); \
  glClearColor(0.4, 0.4, 0.4, 0.0); \
  glMatrixMode(GL_MODELVIEW); \
  glLoadMatrixd(model_view); \
  member_func_ptr(); \
  glPopMatrix(); \
  glutSwapBuffers();

extern GLdouble model_view[16];
extern GLdouble projection[16];
extern GLint viewport[4];

#endif //_simple_rrt_h
