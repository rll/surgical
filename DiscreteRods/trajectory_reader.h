#ifndef _trajectory_reader_h
#define _trajectory_reader_h

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
#include "thread_discrete.h"

#define THREADS_BASE_NAME "saved_threads"

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN


class Trajectory_Reader
{
	public:
		Trajectory_Reader();
		Trajectory_Reader(const char* fileName_threads_in);


		void set_file(const char* fileName_threads_in);
		int read_threads_from_file();
		vector<Thread>& get_all_threads(){return _each_thread;};
		void get_all_threads(vector<Thread*>& threads_out);
	
	private:
		char _fileName_threads[256];
		vector<Thread> _each_thread;
};

#endif
