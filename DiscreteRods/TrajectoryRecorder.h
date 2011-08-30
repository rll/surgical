#ifndef _TrajectoryRecorder_h
#define _TrajectoryRecorder_h

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
#include "StateRecorder.h"
#include "TrajectoryReader.h"

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

class TrajectoryRecorder
{
	public:
		TrajectoryRecorder();
		TrajectoryRecorder(const char* fileName);
    void setFileName(const char* newFileName);

		void start();
		void stop();
		void writeWorldToFile(World* world);
		bool hasStarted() { return started; }

	private:
		char _fileName[256];
		ofstream file;
		bool started;
};

#endif
