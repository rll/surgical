#ifndef _TrajectoryRecorder_h
#define _TrajectoryRecorder_h

#include <stdlib.h>
#include <string.h>

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
#include "TrajectoryReader.h"
#include "IO/Control.h"

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

class TrajectoryRecorder
{
	public:
		TrajectoryRecorder(const char* new_base_name = "environmentFiles/");
		TrajectoryRecorder(const char* new_file_name, const char* new_base_name);
    void setBaseName(const char* new_base_name);
    void setFileName(const char* new_file_name);
    void getFileName(char* name);
    void queryFileName();

		void start(StateType type);
		void stop();
		void writeWorldToFile(World* world);
		void writeControlToFile(Control* control0, Control* control1);
		bool hasStarted() { return (state_type != NO_STATE); }

	private:
		char base_name[256];
		char file_name[256];
		ofstream file;
		StateType state_type;
};

#endif
