#ifndef _StateReader_h
#define _StateReader_h

#include <stdlib.h>
#include <string.h>
#include <string>

#ifdef MAC
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#include <GL/gle.h>
#else
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/gle.h>
#endif

#include <boost/algorithm/string.hpp>

#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>

#include "EnvObjects/World.h"

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

class StateReader
{
	public:
		StateReader(const char* new_base_name = "environmentFiles/");
		StateReader(const char* new_file_name, const char* new_base_name);
    void setBaseName(const char* new_base_name);
    void setFileName(const char* new_file_name);
    void getFileName(char* name);
    void queryFileName();
		
		bool readWorldFromFile(World*& world);
	
	private:
		void extension(char* ext, char* full_path);

		char base_name[256];
		char file_name[256];
};

#endif
