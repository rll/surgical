#ifndef _controls_reader_h
#define _controls_reader_h

#include <stdlib.h>
#include <iostream>
#include <fstream>

#include "thread_discrete.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

class Controls_Reader
{
	public:
		Controls_Reader(const char* fileName_controls_in);
		void read_controls_from_file();
		void get_all_controls(vector<VectorXd>& controls_out);
	
	private:
		char _fileName_controls[256];
		vector<VectorXd> _each_control;
};

#endif
