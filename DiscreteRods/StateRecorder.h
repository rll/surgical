#ifndef _StateRecorder_h
#define _StateRecorder_h

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
#include "ThreadConstrained.h"
#include "EnvObjects/EnvObject.h"
#include "StateReader.h"

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

class StateRecorder
{
	public:
    StateRecorder(const char* new_base_name = "environmentFiles/");
		StateRecorder(const char* new_file_name, const char* new_base_name);
    void setBaseName(const char* new_base_name);
    void setFileName(const char* new_file_name);
    void getFileName(char* name);
    void queryFileName();

		void writeWorldToFile(World* world);

	private:
		char base_name[256];
		char file_name[256];		
};

#endif
