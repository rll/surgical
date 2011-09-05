#ifndef _TrajectoryReader_h
#define _TrajectoryReader_h

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
#include "IO/Control.h"

#define TRAJECTORY_BASE_NAME "environmentFiles"
enum state_type { NO_STATE, STATE, CONTROL };

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN


class TrajectoryReader
{
	public:
		TrajectoryReader();
		TrajectoryReader(const char* fileName);
		void setFileName(const char* fileName);
		
		bool readWorldsFromFile(vector<World*>& worlds);
		bool readControlsFromFile(vector<vector<Control*> >& controls);
	
	private:
		char _fileName[256];
};

#endif
