#include "checkerboard_data_vis.h"
#include <iostream>
#include <fstream>

#define VIS_TO_ROBOT_FILENAME_SLAVE1 "vis_to_robot1.txt"
#define VIS_TO_ROBOT_FILENAME_SLAVE2 "vis_to_robot2.txt"
#define ROBOT_TO_VIS_FILENAME_SLAVE1 "robot_to_vis1.txt"
#define ROBOT_TO_VIS_FILENAME_SLAVE2 "robot_to_vis2.txt"

#define SAVED_POINTS_FILENAME "checkerboard_slave1_frame.txt"

int main(int argc, char** argv)
{
	Checkerboard checkerData;
	checkerData.initCams();


	//frame conversion matrices
	Matrix3d RotToRobot;
	Vector3d TransToRobot;

	//open params file, read params
	std::ifstream vis_to_slave1_file;

	vis_to_slave1_file.open(VIS_TO_ROBOT_FILENAME_SLAVE1);
	double vis_to_slave1_rot_params[3];
	for (int i = 0; i < 3; i++)
	{
		vis_to_slave1_file >> vis_to_slave1_rot_params[i];
	}
	EulerZYX(vis_to_slave1_rot_params[0], vis_to_slave1_rot_params[1], vis_to_slave1_rot_params[2], RotToRobot);

	for (int i = 0; i < 3; i++)
	{
		vis_to_slave1_file >> TransToRobot(i);
	}

	std::cout << "Rot to Robot:\n" << RotToRobot << std::endl;
	std::cout << "Trans to Robot:\n" << TransToRobot << std::endl;



	//file to save points
	std::ofstream saved_checkerpoints_slave1;
	saved_checkerpoints_slave1.open(SAVED_POINTS_FILENAME);
	saved_checkerpoints_slave1.precision(20);

	while (1)
	{
		char c = cvWaitKey(1000000);
		if (checkerData.findCheckerboard())
		{
			checkerData.calculate3dPoints();
			checkerData.drawCheckerboard();

			Mat startRotCheck_cv;
			Mat startTransCheck_cv;
			checkerData.getEstimated3dPose(startRotCheck_cv, startTransCheck_cv);

			Matrix3d startRotCheck;
			Vector3d startTransCheck;

			Matrix3d startRot = Matrix3d::Identity();
			for (int r=0; r < 3; r++)
			{
				for (int c=0; c < 3; c++)
				{
					startRotCheck(r,c) = startRotCheck_cv.at<float>(r,c);
				}
			}

			for (int r=0; r < 3; r++)
			{
				startTransCheck(r) = startTransCheck_cv.at<float>(r,0);
			}

			Matrix3d checkRotRobotFrame = RotToRobot*startRotCheck;
			double checkRotZYX[3];
			RotToEulerZYX(checkRotZYX[0], checkRotZYX[1], checkRotZYX[2], checkRotRobotFrame);

			vector<Point3f> checkerLocations_3d_visFrame = checkerData.checkerLocations_3d;
			vector<Vector3d> checkerLocations_3d_robotFrame(checkerLocations_3d_visFrame.size());
			for (int i=0; i < checkerLocations_3d_visFrame.size(); i++)
			{
				Vector3d thisPt(checkerLocations_3d_visFrame[i].x, checkerLocations_3d_visFrame[i].y, checkerLocations_3d_visFrame[i].z);
				checkerLocations_3d_robotFrame[i] = RotToRobot*thisPt + TransToRobot;
			}


			//output to file
			for (int i=0; i < 3; i++)
			{
				saved_checkerpoints_slave1 << checkRotZYX[i] << " ";
			}

			for (int i=0; i < checkerLocations_3d_robotFrame.size(); i++)
			{
				for (int ind=0; ind < 3; ind++)
				{
					saved_checkerpoints_slave1 << checkerLocations_3d_robotFrame[i](ind) << " ";
				}
			}
			saved_checkerpoints_slave1 << "\n";
			saved_checkerpoints_slave1.flush();

		}
		checkerData.displayIms();
	}
}

