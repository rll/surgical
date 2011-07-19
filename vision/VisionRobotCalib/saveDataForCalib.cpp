#include "checkerboard_data_vis.h"
#include <iostream>
#include <fstream>
#include "../master-slave/slaves/messaging.h"

#define CALIB_FILE_NAME_SLAVE1 "calib_data_slave1.txt"
#define CALIB_FILE_NAME_SLAVE2 "calib_data_slave2.txt"
#define CALIB_IMG_BASE_NAME "savedCalibIms/calibIm"

int main(int argc, char** argv)
{

	char calib_img_base_name[256];
	//setup port
	UDPSocket _incoming = UDPSocket();
	UDPSocket _outgoing = UDPSocket();

	if(!_incoming.create())
		std::cout << "error creating server socket" << std::endl;
	if(!_incoming.bind(VISION_IN_PORT))
		std::cout << "error binding to port " << VISION_IN_PORT << std::endl;

	if(!_outgoing.create())
		std::cout << "error creating server socket" << std::endl;
	if(!_outgoing.bind(CONTROLLER_IN_PORT))
		std::cout << "error binding to port " << CONTROLLER_IN_PORT << std::endl;

	_incoming.set_non_blocking(true);
	_incoming.set_timeout(0);             // using recv()

	_outgoing.set_non_blocking(true);
	_outgoing.set_timeout(0);
	_outgoing.setDestination(CONTROLLER_IP, CONTROLLER_IN_PORT);


  double slaveID, eulerZ, eulerY, eulerX, posZ, posY, posX;

	bool initialized = false;
	bool done = false;
	Checkerboard checkerData;
	checkerData.initCams();
	std::ofstream calibFile;

	int numImgs = 1;
	while (1)
	{

		Messaging::send_message(_outgoing, Messaging::NEXT_WAYPOINT_REQUEST);

		while (!Messaging::receive_message(_incoming, Messaging::WAYPOINT_RESPONSE, &slaveID, &eulerZ, &eulerY, &eulerX, &posX, &posY, &posZ))
		{
			char key_pressed = cvWaitKey(1000);
			if (key_pressed == 'q')
			{
				done = true;
				break;
			}
		}

		if (done)
			break;

		if (!initialized)
		{
			if (slaveID < 0.5)
			{
				calibFile.open(CALIB_FILE_NAME_SLAVE1);
				sprintf(calib_img_base_name, "%s_slave1_", CALIB_IMG_BASE_NAME);
			}
			else
			{
				calibFile.open(CALIB_FILE_NAME_SLAVE2);
				sprintf(calib_img_base_name, "%s_slave2_", CALIB_IMG_BASE_NAME);
			}
			calibFile.precision(20);
			initialized = true;
		}



		if (checkerData.findCheckerboard())
		{
			//save ims
			for (int camNum = 0; camNum < NUMCAMS; camNum++)
			{
				char imName[256];
				sprintf(imName, "%s%d-%d.tif", calib_img_base_name, (camNum+1), numImgs);
				imwrite(imName, checkerData._frames[camNum]);
			}

			checkerData.calculate3dPoints();
			checkerData.drawCheckerboard();

			vector<Point3f> checkerLocations_Ims = checkerData.get_checkerLocations_3d();

			//write info from received message
			calibFile << eulerZ << " " << eulerY << " " << eulerX << " " << posX << " " << posY << " " << posZ << " ";

			//write info for detected checkerboards
			for (int i=0; i < checkerLocations_Ims.size(); i++)
			{
				calibFile << checkerLocations_Ims[i].x << "  " << checkerLocations_Ims[i].y << "  " << checkerLocations_Ims[i].z << "  ";
			}
			calibFile << "\n";
			calibFile.flush();
			numImgs++;

		}

		checkerData.displayIms();

		//SEND DONE SIGNAL
	}

	calibFile.close();

}

