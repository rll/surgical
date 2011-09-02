#ifndef _StateReader_h
#define _StateReader_h

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

#include "EnvObjects/World.h"

#define STATE_BASE_NAME "environmentFiles"

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN


class StateReader
{
	public:
		StateReader();
		StateReader(const char* fileName);
		void setFileName(const char* fileName);
		
		bool readWorldFromFile(World* world);
	
	private:
		char _fileName[256];
};

#endif
