#ifndef _trajectory_recorder_h
#define _trajectory_recorder_h

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
#include "trajectory_reader.h"


// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN


class Trajectory_Recorder
{
	public:
		Trajectory_Recorder();
		Trajectory_Recorder(char* fileName_threads_in);

		void add_thread_to_list(const Thread& thread);

		void write_threads_to_file();
		void clear_threads();

        void setFileName(char* newFileName);

	private:
		char _fileName_threads[256];
		vector<Thread> _threads;
};

#endif
