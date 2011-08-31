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

	Control(Vector3d start_position, Matrix3d start_rotation);
	~Control();
	
	void setControl(ControllerBase* controller);
	void getControlVector(VectorXd& control);
	void setInitialTransform(const Vector3d& pos, const Matrix3d& rot);
	void setNoMotion();
	
	const Vector3d& getPosition() const;
	const Matrix3d& getRotation() const;
	const Vector3d& getTranslate() const;
	const Quaterniond& getRotate() const;
	bool getButton(button_type bttn_type);
	
	void writeToFile(ofstream& file);
	Control(ifstream& file);
  
//protected:
	Vector3d position;
	Matrix3d rotation;
  
  Vector3d translate;
  Quaterniond rotate;
  bool button[2];
};

#endif // _Control_h

