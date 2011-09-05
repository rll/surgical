#ifndef _EnvObject_h
#define _EnvObject_h

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
#include <assert.h>

#include "../utils/drawUtils.h"
#include "../Collisions/intersectionStructs.h"
#include "../Collisions/collisionUtils.h"
#include "World.h"

using namespace std;
USING_PART_OF_NAMESPACE_EIGEN

/** Base class for objects in environment. */
class EnvObject
{
public:

  EnvObject() {}
  
  EnvObject(Vector3d pos, Matrix3d rot, float c0, float c1, float c2, object_type t) 
    : position(pos)
    , rotation(rot)
    , color0(c0)
    , color1(c1)
    , color2(c2)
    , type(t)
  {}
  
  EnvObject(float c0, float c1, float c2, object_type t) 
    : color0(c0)
    , color1(c1)
    , color2(c2)
    , type(t)
  {}
  
  virtual ~EnvObject() {}

  virtual void writeToFile(ofstream& file) = 0;
	EnvObject(ifstream& file, World* w);
  
  virtual void setTransform(const Vector3d& pos, const Matrix3d& rot)
  { 
    position = pos;
    rotation = rot;
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

	void setColor(float c0, float c1, float c2)
	{ 
	  color0 = c0;
	  color1 = c1;
	  color2 = c2;
	}

  virtual void getState(VectorXd& state)
  {
  	state.resize(7);
  	double angZ, angY, angX;
    int ind = 0; 
    state(ind) = 7;
    ind += 1;
  	euler_angles_from_rotation(rotation, angZ, angY, angX);
  	for (int i = 0; i < 3; i++) {
  		state(i+ind) = position(i);
  	}
    ind += 3; 
    //state.segment(3, 3) = 50 * rotation.col(0);
  	state(ind+0) = angZ;
  	state(ind+1) = angY;
  	state(ind+2) = angX;
  }

  virtual void setState(VectorXd& state) 
  {
    assert(state(0) == state.size());
    double angZ, angY, angX;
    Matrix3d rot;
    Vector3d pos; 
    pos(0) = state(1);
    pos(1) = state(2);
    pos(2) = state(3); 
    rotation_from_euler_angles(rot, state(4), state(5), state(6));

    setTransform(pos, rot); 
  }  
  /*virtual void applyControl(const VectorXd& u)
	{
		double max_ang = max( max(abs(u(3)), abs(u(4))), abs(u(5)));

		int number_steps = max ((int)ceil(max_ang / (M_PI/4.0)), 1);
		VectorXd u_for_translation = u/((double)number_steps);

		Vector3d translation;
		translation << u_for_translation(0), u_for_translation(1), u_for_translation(2);

		Matrix3d rotation;
		rotation_from_euler_angles(rotation, u(3), u(4), u(5));
		Quaterniond quat_rotation(rotation);
		rotation = Quaterniond::Identity().slerp(1.0/(double)number_steps, quat_rotation);

		for (int i=0; i < number_steps; i++)
		{
		  setTransform(getPosition() + translation, getRotation() * rotation);
		}
	}*/
  
  virtual void draw() = 0;
  
  object_type getType() { return type; }
  
  //backup
  virtual void backup() = 0;
  virtual void restore() = 0;
  
  //collision
  virtual bool capsuleIntersection(int capsule_ind, const Vector3d& start, const Vector3d& end, const double radius, vector<Intersection>& intersections) = 0;
  virtual double capsuleRepulsionEnergy(const Vector3d& start, const Vector3d& end, const double radius) = 0;
  virtual void capsuleRepulsionEnergyGradient(const Vector3d& start, const Vector3d& end, const double radius, Vector3d& gradient) = 0;
  
protected:
  Vector3d position;
  Matrix3d rotation;
  float color0, color1, color2;
  object_type type;
  
  Vector3d backup_position;
  Matrix3d backup_rotation;
};

#endif // _EnvObject_h
