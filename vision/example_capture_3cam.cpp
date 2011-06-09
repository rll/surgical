#include "capture2.h"
#include "util2.h"
#include "../utils/clock.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <time.h>

#include <iostream>


using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
	if (argc < 4) {
		cerr << "Usage: example_capture <name> <startind>" << endl;
		return 1;
	}
	Capture* cam;
	int camNum = atoi(argv[3]);

	Mat frame;

	if (camNum == 1)
	{
		cam = new Capture(0, //id
				"cam1", // cam name
				650,  // gain
				"optimized", // optimized or measured
				107109);
		cam->setExposure(5000);
		cam->syncFrameCaptureSetCenter();
		cout << "c1 created" << endl;
	} else if (camNum == 2) {
		cam = new Capture(1, //id
				"cam2", // cam name
				650,  // gain
				"optimized", // optimized or measured
				107110);
		cam->setExposure(3000);
		cam->setSyncIn();
		cout << "c2 created" << endl;
	} else if (camNum == 3)
	{
		cam = new Capture(2, //id
				"cam3", // cam name
				650,  // gain
				"optimized", // optimized or measured
				107111);
		cout << "c3 created" << endl;  
		cam->setExposure(3500);
		cam->setSyncIn();
	} else {
		std::cerr << "incorrect camera number entered" << std::endl;
		return 1;
	}

	char camName[256];
	sprintf(camName, "%s%d", "cam", camNum);

	namedWindow(camName, CV_WINDOW_AUTOSIZE);
	//namedWindow("Cam2", CV_WINDOW_AUTOSIZE);
	//namedWindow("Cam3", CV_WINDOW_AUTOSIZE);

	cvWaitKey(2000);							// segfaulted without this

	cam->startCapture();
	cvWaitKey(2000);


	cout << "press c to save an image, q to exit" << endl;

	double totalTime = 0.0;
	double beforeTime = 0.0;
	double count = 0.0;

	char filename[256];
	int f=atoi(argv[2]);


	while(1)
	{

		beforeTime = GetClock();
		while (!cam->waitForFrame()){}
		frame = cam->currentFrame();

		/*
			 imshow("Cam1", frame1);
			 imshow("Cam2", frame2);
			 imshow("Cam3", frame3);
			 */      
		char c = cvWaitKey(1);
		if(c == 'q')
			break;
		//if(c == 'c') {
			sprintf(filename, "/media/ssd1/captures/%s%d-%d.tif", argv[1],camNum,f);
			imwrite(filename, frame);
			f++;
		//}

		// timing3576807
		totalTime += GetClock() - beforeTime;
		count += 1.0;
		if(count > 20) {
			cout << "Framerate: " << count/totalTime << endl;
			count = 0.0;
			totalTime = 0.0;
		}
	}

}

