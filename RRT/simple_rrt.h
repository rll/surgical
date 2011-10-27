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
  glMatrixMode(GL_MODELVIEW); \
  glPushMatrix(); \
  glLoadMatrixd(model_view); \
  member_func_ptr(); \
  glPopMatrix();

extern GLdouble model_view[16];
extern GLdouble projection[16];
extern GLint viewport[4];

#endif //_simple_rrt_h
