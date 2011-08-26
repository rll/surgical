#include "runCalibFromFiles.h"


int main(int argc, char** argv)
{
	if (argc < 2)
	{
		std::cerr << "please specify slave number to calibrate for" << std::endl;
		exit(0);
	}

	int slaveNum = atoi(argv[1]);
	if (slaveNum != 1 && slaveNum != 2)
	{
		std::cerr << "slave num can only be 1 or 2" << std::endl;
		exit(0);
	}

	std::ifstream checkerDataFile;
	std::ofstream visToRobotFile;
	std::ofstream robotToVisFile;
	if (slaveNum == 1)
	{
		checkerDataFile.open(CALIB_FILE_NAME_SLAVE1);
    visToRobotFile.open(VIS_TO_ROBOT_FILENAME_SLAVE1);
    robotToVisFile.open(ROBOT_TO_VIS_FILENAME_SLAVE1);
	} else {
		checkerDataFile.open(CALIB_FILE_NAME_SLAVE2);
    visToRobotFile.open(VIS_TO_ROBOT_FILENAME_SLAVE2);
    robotToVisFile.open(ROBOT_TO_VIS_FILENAME_SLAVE2);
	}
  visToRobotFile.precision(20);
  robotToVisFile.precision(20);

	while (!checkerDataFile.eof())
	{
		//std::cout << "STARTIN NEW" << std::endl;
		vector<double> robotData;
		robotData.resize(6);
		vector<double> checkerData;
		checkerData.resize(3*CHECKERS_PER_COL*CHECKERS_PER_ROW);

		for (int i=0; i < 6; i++)
		{
			checkerDataFile >> robotData[i];
			//std::cout << robotData[i] << std::endl;
		}

		for (int i=0; i < 3*CHECKERS_PER_COL*CHECKERS_PER_ROW; i++)
		{
			checkerDataFile >> checkerData[i];
			//std::cout << checkerData[i] << std::endl;
		}

		calib_opt_info.checkerPoints.push_back(Checkerboard_Data(checkerData));
		calib_opt_info.robotPoints.push_back(Robot_Data(robotData));
	}

	//last point bad due to file structure, so remove it
	calib_opt_info.checkerPoints.pop_back();
	calib_opt_info.robotPoints.pop_back();

  //if we want to optimize over the first few images only
	/*for (int i=0; i < 3; i++)
	{
		calib_opt_info.checkerPoints.pop_back();
		calib_opt_info.robotPoints.pop_back();
	}*/

	/*
	calib_opt_info.checkerPoints.erase(calib_opt_info.checkerPoints.begin()+0);
	calib_opt_info.robotPoints.erase(calib_opt_info.robotPoints.begin()+0);
	/*
	calib_opt_info.checkerPoints.erase(calib_opt_info.checkerPoints.begin()+1);
	calib_opt_info.robotPoints.erase(calib_opt_info.robotPoints.begin()+1);
	*/

	calib_opt_info.checkerboard.initCamsFromFile(slaveNum);

	calib_opt_info.orig_params.ReSize(9);
	calib_opt_info.orig_params(1) = M_PI/2.0;
	calib_opt_info.orig_params(2) = 0.0;
	calib_opt_info.orig_params(3) = 0.0;
	calib_opt_info.orig_params(4) = 211.9;
	calib_opt_info.orig_params(5) = 82.21;
	calib_opt_info.orig_params(6) = 242;
	calib_opt_info.orig_params(7) = ((CHECKERS_PER_ROW-1.0)/2.0)*SIZE_EACH_CHECKER;
	calib_opt_info.orig_params(8) = 12.0;
	calib_opt_info.orig_params(9) = 0.0;


  NEWMAT::ColumnVector x_sol(9);

	std::cout << "optimizing" << std::endl;
	optimize_FDNLF(9, calibErrorFunction, calibErrorFunction_init, x_sol, 1e-7, 500000);
	/*for (int i=1; i <= 9; i++)
	{
		x_sol(i) = calib_opt_info.orig_params(i);
	}*/
	std::cout << "optimization done" << std::endl;

	//visualize

	Matrix3d RotToVis;
	Vector3d TransToVis;

	Matrix3d RotToRobot;
	Vector3d TransToRobot;


	EulerZYX(x_sol(1), x_sol(2), x_sol(3), RotToVis);
	std::cout << "Rot to Vis\n" << RotToVis << std::endl;
	TransToVis(0) = x_sol(4);
	TransToVis(1) = x_sol(5);
	TransToVis(2) = x_sol(6);
	Vector3d offsetGuess;
	offsetGuessToVec(x_sol(7), x_sol(8), offsetGuess);
	double offsetAngGuess = x_sol(9);

  for (int i=1; i <= 6; i++)
  {
    robotToVisFile << x_sol(i) << " ";
  }

  RotToRobot = RotToVis.transpose();
  double toOutputVisToRobot[3];
  RotToEulerZYX(toOutputVisToRobot[0], toOutputVisToRobot[1], toOutputVisToRobot[2], RotToRobot);
  TransToRobot = -1.0*(RotToRobot*TransToVis);
  for (int i=0; i < 3; i++)
  {
    visToRobotFile << toOutputVisToRobot[i] << " ";
  }
  for (int i=0; i < 3; i++)
  {
    visToRobotFile << TransToRobot(i) << " ";
  }




	/*
	offsetGuess(1) =  ((CHECKERS_PER_COL-1)/2.0)*SIZE_EACH_CHECKER;
	offsetGuess(0) = -10.0;
	double offsetAngGuess = 0.0;
*/


	for (int ptInd=0; ptInd < calib_opt_info.checkerPoints.size(); ptInd++)
	{
		
		calib_opt_info.checkerboard.updateImsNoUndistort();
		Matrix3d rotInFrame = RotToVis*calib_opt_info.robotPoints[ptInd].rotation;
		Vector3d transInFrame = RotToVis*calib_opt_info.robotPoints[ptInd].translation + TransToVis;
		//transInFrame = Vector3d::Zero();

		vector<Vector3d> checkerPtsRobotHand;
		calib_opt_info.checkerboard.estimatePointsInRobotHand(transInFrame, rotInFrame, offsetGuess, offsetAngGuess, checkerPtsRobotHand);


//		rotInFrame = RotToVis;


		for (int camNum = 0; camNum < NUMCAMS; camNum++)
		{
			Mat rotInFrame_cv;
			Mat transInFrame_cv;
			EigenToOpencv(rotInFrame, rotInFrame_cv);
			EigenToOpencv(transInFrame, transInFrame_cv);
			calib_opt_info.checkerboard._captures[camNum]->drawPose(rotInFrame_cv, transInFrame_cv);
		}

		/*

		for (int camNum = 0; camNum < NUMCAMS; camNum++)
		{
			Mat rotInFrame_cv;
			Mat transInFrame_cv;
			EigenToOpencv(RotToVis, rotInFrame_cv);
			Vector3d transForFrameShow(10.0, 0.0, 0.0);
			EigenToOpencv(transForFrameShow, transInFrame_cv);
			calib_opt_info.checkerboard._captures[camNum]->drawPose(rotInFrame_cv, transInFrame_cv);
		}*/





		calib_opt_info.checkerboard.drawCheckerboard();
		calib_opt_info.checkerboard.drawCheckerboard(calib_opt_info.checkerPoints[ptInd].checkerPositions);

		calib_opt_info.checkerboard.displayIms();
		waitKey(200000);
	}




}



void offsetGuessToVec(double guess1, double guess2, Vector3d& guessVec)
{
	guessVec(1) = -guess1;
	guessVec(0) = -guess2;
	guessVec(2) = 0.0;
}

void calibErrorFunction_init(int ndim, NEWMAT::ColumnVector& x)
{
	for (int i = 0; i < ndim; i++)
	{
		x.element(i) = calib_opt_info.orig_params.element(i);
	}
}


void calibErrorFunction(int ndim, const NEWMAT::ColumnVector& x, double& fx, int& result)
{
	Matrix3d RotToVis;
	Vector3d TransToVis;


	fx = 0.0;

	EulerZYX(x(1), x(2), x(3), RotToVis);
	TransToVis(0) = x(4);
	TransToVis(1) = x(5);
	TransToVis(2) = x(6);
  Vector3d offsetGuess;
	double offsetAngGuess;
	if (ndim == 9)
	{
		offsetGuessToVec(x(7), x(8), offsetGuess);
		offsetAngGuess = x(9);
		fx += WEIGHT_OFFSET_POS*(pow(calib_opt_info.orig_params.element(6)-x.element(6),2) + pow(calib_opt_info.orig_params.element(7)-x.element(7),2));
		fx += WEIGHT_OFFSET_ANG*pow(calib_opt_info.orig_params.element(8)-x.element(8),2);
	} else {
		offsetGuessToVec(calib_opt_info.orig_params.element(6),calib_opt_info.orig_params.element(7) ,offsetGuess);
		offsetAngGuess = calib_opt_info.orig_params.element(8);
	}



	

	for (int ptInd=0; ptInd < calib_opt_info.checkerPoints.size(); ptInd++)
	{
		Matrix3d rotInFrame = RotToVis*calib_opt_info.robotPoints[ptInd].rotation;
		Vector3d transInFrame = RotToVis*calib_opt_info.robotPoints[ptInd].translation + TransToVis;

    vector<Vector3d> checkerPtsRobotHand;
    calib_opt_info.checkerboard.estimatePointsInRobotHand(transInFrame, rotInFrame, offsetGuess, offsetAngGuess, checkerPtsRobotHand);

		for (int checkerPtInd=0; checkerPtInd < checkerPtsRobotHand.size(); checkerPtInd++)
		{
			fx += (checkerPtsRobotHand[checkerPtInd] - calib_opt_info.checkerPoints[ptInd].checkerPositions[checkerPtInd]).norm();
		}
	}

  result = OPTPP::NLPFunction;

}





Checkerboard_Data::Checkerboard_Data(vector<Vector3d>& posIn)
{
	for (int c=0; c < CHECKERS_PER_COL; c++)
	{
		for (int r=0; r < CHECKERS_PER_ROW; r++)
		{
			int posInd = c*CHECKERS_PER_ROW+r;
			checkerPositions[posInd] = posIn[posInd];
		}
	}
}


Checkerboard_Data::Checkerboard_Data(vector<Point3f>& posIn)
{
	for (int c=0; c < CHECKERS_PER_COL; c++)
	{
		for (int r=0; r < CHECKERS_PER_ROW; r++)
		{
			int posInd = c*CHECKERS_PER_ROW+r;
			checkerPositions[posInd](0) = posIn[posInd].x;
			checkerPositions[posInd](1) = posIn[posInd].y;
			checkerPositions[posInd](2) = posIn[posInd].z;
		}
	}
}


Checkerboard_Data::Checkerboard_Data(vector<double>& dataIn)
{
	for (int c=0; c < CHECKERS_PER_COL; c++)
	{
		for (int r=0; r < CHECKERS_PER_ROW; r++)
		{
			int posInd = c*CHECKERS_PER_ROW+r;
			checkerPositions[posInd](0) = dataIn[3*posInd+0];
			checkerPositions[posInd](1) = dataIn[3*posInd+1];
			checkerPositions[posInd](2) = dataIn[3*posInd+2];
		}
	}
}



Robot_Data::Robot_Data(vector<double>& dataIn)
{
	EulerZYX(dataIn[0], dataIn[1], dataIn[2], rotation);
	translation(0) = dataIn[3];
	translation(1) = dataIn[4];
	translation(2) = dataIn[5];
}
