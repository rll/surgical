#ifndef _trajectory_recorder_h
#define _trajectory_recorder_h

#include <stdlib.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/gle.h>
#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry> 
#include <math.h>
#include "thread_minenergy.h"

#include "globals_thread_param_estimation.h"

// import most common Eigen types 
USING_PART_OF_NAMESPACE_EIGEN


class Trajectory_Recorder
{
	public:
		Trajectory_Recorder();
		Trajectory_Recorder(char* fileName_motions_in, char* fileName_threads_in);



		void add_motion_to_list(Vector3d& start_pos, Vector3d& end_pos, Vector3d& start_tan, Vector3d& end_tan);
		void add_motion_to_list(Vector3d& pos_mov, Matrix3d& tan_rot);
		void add_motion_to_list(Thread_Motion& motion);
		void add_thread_to_list(Thread* thread);

		void write_motions_to_file();
		void write_threads_to_file();

	

	private:
		char _fileName_motions[256];
		char _fileName_threads[256];
		vector<Thread_Motion> _motions;
		vector<Thread*> _threads;




};

#endif
