#ifndef _ControlBase_h
#define _ControlBase_h

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
#include <vector>

#include "../EnvObjects/EnvObject.h"

using namespace std;
USING_PART_OF_NAMESPACE_EIGEN

enum button_type { UP, DOWN };

class ControlBase
{
public:

  ControlBase()
  	: position(0.0, 0.0, 0.0)
  	, rotation(Matrix3d::Identity())
  {}

  ~ControlBase() {}

  void setTransform(const Vector3d& pos, const Matrix3d& rot)
  {
  	position = pos;
  	rotation = rot;
  }
  
  void setTransform(ControlBase* control)
  {
  	position = control->position;
  	rotation = control->rotation;
  }
  
  void setTransform(EnvObject* obj)
  {
  	position = obj->getPosition();
  	rotation = obj->getRotation();
  }
  
  void getTransform(Vector3d& pos, Matrix3d& rot)
  {
  	pos = position;
  	rot = rotation;
  }
  
  const Vector3d& getPosition() const
  {
  	return position;
  }

  const Matrix3d& getRotation() const
  {
  	return rotation;
  }

	virtual bool hasButtonPressedAndReset(button_type bttn_type) = 0;
  
protected:
  Vector3d position;
  Matrix3d rotation;
};

#endif // _ControlBase_h

