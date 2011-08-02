#ifndef _drawUtils_h
#define _drawUtils_h

#include <stdlib.h>

#ifdef MAC
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#include <GL/gle.h>
#else
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/gle.h> 
#endif

#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

void drawCylinder(Vector3d pos, Matrix3d rot, double h, double r, float color0, float color1, float color2);
void drawEndEffector(Vector3d pos, Matrix3d rot, double degrees, float color0, float color1, float color2);
void drawGrip(Vector3d pos, Matrix3d rot, double radius, double degrees, float color0, float color1, float color2);
void drawSphere(Vector3d position, float radius, float color0, float color1, float color2);
void drawCursor(int device_id, float color);
void drawAxes(Vector3d pos, Matrix3d rot);
void labelAxes(Vector3d pos, Matrix3d rot);

static double gCursorScale = 6;
static GLuint gCursorDisplayList = 0;

#endif //_drawUtils_h
