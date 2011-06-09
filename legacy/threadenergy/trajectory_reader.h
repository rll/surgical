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
#include "thread_minenergy.h"

#include "globals_thread_param_estimation.h"


// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

struct savedThread
{
	MatrixXd points;
	Vector3d positions[2];
	Vector3d tangents[2];

	savedThread(){};

	savedThread(const MatrixXd pointsIn, const Vector3d positionsIn[], const Vector3d tangentsIn[])
		: points(pointsIn)
	{
		positions[0] = positionsIn[0];
		positions[1] = positionsIn[1];
		tangents[0] = tangentsIn[0];
		tangents[1] = tangentsIn[1];
	}

	savedThread(const savedThread& toCopy)
		: points(toCopy.points)
	{
		positions[0] = toCopy.positions[0];
		positions[1] = toCopy.positions[1];
		tangents[0] = toCopy.tangents[0];
		tangents[1] = toCopy.tangents[1];
	}

	~savedThread(){};

};


class Trajectory_Reader
{
	public:
		Trajectory_Reader();
		Trajectory_Reader(char* fileName_motions_in, char* fileName_threads_in, char* fileName_points_in);


		void read_motions_from_file();
		void read_threads_from_file();
		bool get_next_motion(Thread_Motion& nextMotion);
		//bool get_next_points(MatrixXd points);
		void estimate_init_thread();
		vector<Thread_Motion>* get_all_motions(){return &_motions;};
		vector<savedThread>* get_all_points(){return &_points_each_thread;};
		vector<Thread*>* get_all_threads(){return &_ptr_each_thread;};

		Thread* start_thread(){return _start_thread;};
		double length(){return _length_thread;};

	private:
		char _fileName_motions[256];
		char _fileName_threads[256];
		char _fileName_thread_points[256];
		vector<Thread_Motion> _motions;
		Thread* _start_thread;
		double _length_thread;
		vector<savedThread> _points_each_thread;
		vector<Thread*> _ptr_each_thread;

		int _currInd_motions;
		int _currInd_points;


};

#endif
