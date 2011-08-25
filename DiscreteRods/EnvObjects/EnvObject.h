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
  
  virtual ~EnvObject() {}

  virtual void writeToFile(ofstream& file) = 0;
	EnvObject(ifstream& file);
	virtual void updateIndFromPointers(World* world) = 0;
	virtual void linkPointersFromInd(World* world) = 0;

  virtual void recomputeFromTransform(const Vector3d& pos, const Matrix3d& rot) = 0;
  
  void setTransform(const Vector3d& pos, const Matrix3d& rot)
  { 
    position = pos; 
    rotation = rot; 
    recomputeFromTransform(pos, rot);
  }
  
	void setColor(float c0, float c1, float c2)
	{ 
	  color0 = c0;
	  color1 = c1;
	  color2 = c2;
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
  
  virtual void draw() = 0;
  virtual bool capsuleIntersection(int capsule_ind, const Vector3d& start, const Vector3d& end, const double radius, vector<Intersection>& intersections) = 0;
  virtual double capsuleRepulsionEnergy(const Vector3d& start, const Vector3d& end, const double radius) = 0;
  virtual void capsuleRepulsionEnergyGradient(const Vector3d& start, const Vector3d& end, const double radius, Vector3d& gradient) = 0;
  
  object_type getType() { return type; }
  
protected:
  Vector3d position;
  Matrix3d rotation;
  float color0, color1, color2;
  object_type type;
};

#endif // _EnvObject_h
