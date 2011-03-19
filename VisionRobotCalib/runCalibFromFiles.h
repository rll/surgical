#ifndef _runCalibFromFiles_h
#define _runCalibFromFiles_h

#include "checkerboard_data_vis.h"
#include "../threadenergy/thread_utils.h"
#include <iostream>
#include <fstream>

#define CALIB_FILE_NAME_SLAVE1 "calib_data_slave1.txt"
#define CALIB_FILE_NAME_SLAVE2 "calib_data_slave2.txt"
#define CALIB_IMG_BASE_NAME "savedCalibIms/calibIm"


#define VIS_TO_ROBOT_FILENAME_SLAVE1 "vis_to_robot1.txt"
#define VIS_TO_ROBOT_FILENAME_SLAVE2 "vis_to_robot2.txt"
#define ROBOT_TO_VIS_FILENAME_SLAVE1 "robot_to_vis1.txt"
#define ROBOT_TO_VIS_FILENAME_SLAVE2 "robot_to_vis2.txt"

#define WEIGHT_OFFSET_ANG 0.0
#define WEIGHT_OFFSET_POS 0.0


struct Checkerboard_Data
{
	Vector3d checkerPositions[CHECKERS_PER_ROW*CHECKERS_PER_COL];

	Checkerboard_Data(vector<Vector3d>& posIn);
	Checkerboard_Data(vector<Point3f>& posIn);
	Checkerboard_Data(vector<double>& dataIn);

//	~Checkerboard_Data();
};

struct Robot_Data
{
	Matrix3d rotation;
	Vector3d translation;

	Robot_Data(vector<double>& dataIn);

	//~Robot_Data();
};



struct calib_optimization_info
{
	vector<Checkerboard_Data> checkerPoints;
	vector<Robot_Data> robotPoints;
	Checkerboard checkerboard;

	NEWMAT::ColumnVector orig_params;
		
};

static calib_optimization_info calib_opt_info;



void offsetGuessToVec(double guess1, double guess2, Vector3d& guessVec);

void calibErrorFunction_init(int ndim, NEWMAT::ColumnVector& x);
void calibErrorFunction(int ndim, const NEWMAT::ColumnVector& x, double& fx, int& result);

#endif
