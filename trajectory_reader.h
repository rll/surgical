#ifndef _trajectory_reader_h
#define _trajectory_reader_h

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


class Trajectory_Reader
{
	public:
		Trajectory_Reader();
		Trajectory_Reader(char* fileName_motions_in, char* fileName_threads_in);


		void read_motions_from_file();
		void read_threads_from_file();
		bool get_next_motion(Thread_Motion& nextMotion);
		bool get_next_points(MatrixXd points);
		void estimate_init_thread();
		vector<Thread_Motion>& get_all_motions(){return _motions;};
		vector<MatrixXd>& get_all_points(){return _points_each_thread;};
		Thread* start_thread(){return _start_thread;};
		double length(){return _length_thread;};

	private:
		char _fileName_motions[256];
		char _fileName_threads[256];
		vector<Thread_Motion> _motions;
		Thread* _start_thread;
		double _length_thread;
		vector<MatrixXd> _points_each_thread;

		int _currInd_motions;
		int _currInd_points;


};

#endif
