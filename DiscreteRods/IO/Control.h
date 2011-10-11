#ifndef _Control_h
#define _Control_h

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

#include "ControllerBase.h"

using namespace std;
USING_PART_OF_NAMESPACE_EIGEN

class Control
{
public:

	Control();
	Control(const Vector3d& p_translate, const Quaterniond& p_rotate);
	Control(const Vector3d& p_translate, const Matrix3d& p_rotate);
  Control(const Control& rhs);
	~Control();
	
	void setTranslate(const Vector3d& t);
	void setRotate(const Quaterniond& r);
	void setRotate(const Matrix3d& r);
	void setButton(button_type bttn_type, bool value);
	void getControlVector(VectorXd& control);
	void setNoMotion();
	
	const Vector3d& getTranslate() const;
	const Quaterniond& getRotate() const;
	bool getButton(button_type bttn_type);
	
	void writeToFile(ofstream& file);
	Control(ifstream& file);
  
protected:
  Vector3d translate;
  Quaterniond rotate;
  bool button[2];
};

#endif // _Control_h

