#include "StereoOnClicks_withAuto.h"

using namespace std;
using namespace cv;

StereoOnClicks::StereoOnClicks()
{
	char imBase[NUMCAMS][256];
	for (int camNum=0; camNum < NUMCAMS; camNum++)
	{
		sprintf(imBase[camNum], "%s%d-", base_name, camNum+1);
	}


	Capture* c1 = new Capture(0, //id
			"cam1", // cam name
			650,  // gain
			"optimized", // optimized or measured
			107109,
			imBase[0]);
	//"./calib-mar12/calib1-");
	//"./captures/extrins-4-293-"); // camera uid
	cout << "c1 created" << endl;

	Capture* c2 = new Capture(1, //id
			"cam2", // cam name
			650,  // gain
			"optimized", // optimized or measured
			107110,
			imBase[1]);
	//"./calib-mar12/calib2-");
	//"./captures/extrins-4-293-"); // camera uid
	cout << "c2 created" << endl;

	Capture* c3 = new Capture(2, //id
			"cam3", // cam name
			650,  // gain
			"optimized", // optimized or measured
			107111,
			imBase[2]);
	//"./calib-mar12/calib3-");
	//"./captures/extrins-4-293-"); // camera uid
	cout << "c3 created" << endl;  

	namedWindow("Cam1", CV_WINDOW_AUTOSIZE);
	namedWindow("Cam2", CV_WINDOW_AUTOSIZE);
	namedWindow("Cam3", CV_WINDOW_AUTOSIZE);
	cvSetMouseCallback("Cam1", mouseHandlerWrap1, this);
	cvSetMouseCallback("Cam2", mouseHandlerWrap2, this);
	cvSetMouseCallback("Cam3", mouseHandlerWrap3, this);

	c1->init("../../vision/calib_params_ribbon/");
	c2->init("../../vision/calib_params_ribbon/");
	c3->init("../../vision/calib_params_ribbon/");

	cvWaitKey(100);							// segfaulted without this

	c1->AddOtherCameraInformation(*c2);
	c1->AddOtherCameraInformation(*c3);
	c2->AddOtherCameraInformation(*c1);
	c2->AddOtherCameraInformation(*c3);
	c3->AddOtherCameraInformation(*c1);
	c3->AddOtherCameraInformation(*c2);

	c1->startCapture();
	cout << "c1 started" << endl;
	c2->startCapture();
	cout << "c2 started" << endl;
	c3->startCapture();
	cout << "c3 started" << endl;
	cvWaitKey(100);

	captures[0] = c1;
	captures[1] = c2;
	captures[2] = c3;

	camNames[0] = "Cam1"; 
	camNames[1] = "Cam2"; 
	camNames[2] = "Cam3"; 

	for (int i = 0; i < MAX_POINTS; i++)
	{
		markerColors[i] = Scalar(0,0,255);
	}
	for (int i = 0; i < MAX_POINTS; i++)
	{
	 	textColors[i] = Scalar(0,255,0);
	}


	mousePointerColorThread = Scalar(255,0,0);
	mousePointerColorRobot = Scalar(0,255,0);
	epiLineColor = Scalar(127, 0, 127);

	clickCoords.resize(NUMCAMS);
	for (int camNum = 0; camNum < NUMCAMS; camNum++)
	{
		clickCoords[camNum].resize(MAX_POINTS);
	}

	for (int camNum=0; camNum < NUMCAMS; camNum++)
	{
		captures[camNum]->setImageNumber(START_NUM);
	}

}


StereoOnClicks::StereoOnClicks(Capture* cams[], string names[])
{
	captures[0] = cams[0];
	captures[1] = cams[1];
	captures[2] = cams[2];

	camNames[0] = names[0]; 
	camNames[1] = names[1]; 
	camNames[2] = names[2]; 

	namedWindow(camNames[0].c_str(), CV_WINDOW_AUTOSIZE);
	namedWindow(camNames[1].c_str(), CV_WINDOW_AUTOSIZE);
	namedWindow(camNames[2].c_str(), CV_WINDOW_AUTOSIZE);
	cvSetMouseCallback(camNames[0].c_str(), mouseHandlerWrap1, this);
	cvSetMouseCallback(camNames[1].c_str(), mouseHandlerWrap2, this);
	cvSetMouseCallback(camNames[2].c_str(), mouseHandlerWrap3, this);


	for (int i = 0; i < MAX_POINTS; i++)
	{
		markerColors[i] = Scalar(0,0,255);
	}
	for (int i = 0; i < MAX_POINTS; i++)
	{
	 	textColors[i] = Scalar(0,255,0);
	}
	mousePointerColorThread = Scalar(255,0,0);
	mousePointerColorRobot = Scalar(0,255,0);
	epiLineColor = Scalar(127, 0, 127);

}


StereoOnClicks::~StereoOnClicks()
{
	for (int i = 0; i < NUMCAMS; i++)
	{
		captures[i]->endCapture();
		delete captures[i];
	}
	/*c1->endCapture();
		c2->endCapture();
		c3->endCapture();
		delete c1;
		delete c2;
		delete c3;
		*/
}

void StereoOnClicks::getNewPoints(Point3f* points, int numPointsToClick)
{
	click2dPoints(numPointsToClick);

	//Calculate 3d Point from clicked coordinates
	for (int j = 0; j < numPointsToClick; j++)
	{
		Point p1coords(clickCoords[0][j]);
		Point p2coords(clickCoords[1][j]);
		Point p3coords(clickCoords[2][j]);

		get3dPoint(p1coords, p2coords, p3coords, points[j]);
	}
}

void StereoOnClicks::click2dPoints(int numPointsToClick)
{
	/****************************************************
	//  Set Initial Parameters
	 *****************************************************/
	//cout << "Click on on all three images on corresponding points. Use number keys to select which point you will be clicking, and press enter when satisfied." << endl; 
	int i, j;

	for (i = 0; i < NUMCAMS; i++)
	{
		mouseCoords[i] = Point(1,1);
		for (j = 0; j < MAX_POINTS; j++)
		{
			clickedOnCamera[i][j] = false;
		}
		zoomCenters[i] = Point(XWIDTH/2, YWIDTH/2);
		zoomAmounts[i] = 1.f;
	}

	mouseClickActive = false;
	pointCurrentlyClicking = 0;



	//Repeat this process until user is satisfied
	while(1)
	{

		Mat framesToShow[NUMCAMS];
		for (i = 0; i < NUMCAMS; i++)
		{
			//clone images to add display features
			framesToShow[i] = frames[i].clone();                             

			//draw epipolar lines
			for (j = 0; j < NUMCAMS; j++)
			{
				if (i == j)
					continue;

				/*for (int k =0; k < numPointsToClick; k++)
				{                   
					if (clickedOnCamera[j][k])
					{
						line(framesToShow[i], epipolarInfo[i][j][k].pt1, epipolarInfo[i][j][k].pt2, epiLineColor);
					}
				}*/

				if (clickedOnCamera[j][pointCurrentlyClicking])
				{
					line(framesToShow[i], epipolarInfo[i][j][pointCurrentlyClicking].pt1, epipolarInfo[i][j][pointCurrentlyClicking].pt2, epiLineColor);

				}

			}

			//add crosshair for location of mouse pointer
			/*            if (requestType == Slave1Request || requestType == Slave2Request)
										{
										drawCrosshair(framesToShow[i],mouseCoords[i], 2000, mousePointerColorRobot); 
										}else if (requestType == Thread1Request || requestType == Thread2Request)
										{
										drawCrosshair(framesToShow[i],mouseCoords[i], 2000, mousePointerColorThread); 
										}*/


			drawCrosshair(framesToShow[i], mouseCoords[i], 2000, mousePointerColorThread);



			//add crosshair for location of last click
			//for (j = 0; j < numPointsToClick; j++)
			//{
				if (clickedOnCamera[i][pointCurrentlyClicking])
				{
					drawCrosshair(framesToShow[i], clickCoords[i][pointCurrentlyClicking], 10, markerColors[pointCurrentlyClicking]); 
					drawPointNumText(framesToShow[i], clickCoords[i][pointCurrentlyClicking], pointCurrentlyClicking, textColors[pointCurrentlyClicking]);
				}
			//}

			//resize matrix - find zoomed in region of interest, then blow it up 
			resize(Mat(framesToShow[i].clone(), zoomRect(zoomCenters[i], zoomAmounts[i], frames[i].cols, frames[i].rows))
					, framesToShow[i], Size(frames[i].cols, frames[i].rows), zoomAmounts[i], zoomAmounts[i]);


			/*
			for (int camNum=0; camNum < NUMCAMS; camNum++)
			{
				vector<Mat> planes;
				split(frames[camNum], planes);

				Mat frame_hsv;
				vector<Mat> planesHSV;
				cvtColor(frames[camNum], frame_hsv, CV_BGR2HSV);
				split(frame_hsv, planesHSV);

				vector<Mat> planesToShow;
				split(framesToShow[camNum], planesToShow);
				for (int y=5; y < planesToShow[2].rows-5; y++)
				{
					for (int x=5; x < planesToShow[2].cols-5; x++)
					{
						if (planesHSV[1].at<uchar>(y,x) < 100 || planes[2].at<uchar>(y,x) < 100)
						{
							planesToShow[0].at<uchar>(y,x) = planesToShow[1].at<uchar>(y,x) = planesToShow[2].at<uchar>(y,x) = 0;
						}
					}
				}
				merge(planesToShow, framesToShow[camNum]);
			}
			*/

			//display images
			imshow(camNames[i], framesToShow[i]);
		}

		//Process keyboard inputs
		bool finish = false; 
		char c = waitKey(10);

		if (c == '\n')
		{
			finish = true;
			for (i = 0; i < NUMCAMS; i++)
			{
				for (j = 0; j < numPointsToClick; j++)
				{
					finish = finish * clickedOnCamera[i][j];
				}
			}
		}
        else if (c == '+') {
            int img_num = captures[0]->getImageNumber() - 1;
            char ofile_name[256];
            sprintf(ofile_name, "%s_points.txt", base_name);
            ifstream ofile(ofile_name);
            string line;
            int qq = 0;
            while(ofile.good()) {
                getline(ofile, line);
                stringstream line_stream(line);
                string id_str;
                string dump;
                line_stream >> qq;
                if (qq == img_num) {
                    cout << qq << endl;
                    double junk;
                    for(int i = 0; i < NUM_PTS*3; i++) {
                        line_stream >> junk; 
                    }
                    int x;
                    int y; 
                    for (int cnum = 0; cnum < NUMCAMS; cnum++) { 
                        for (int i = 0; i < NUM_PTS; i++) {
                            line_stream >> x;
                            line_stream >> y; 
                            cout << x << " " << y << endl;
                            pointCurrentlyClicking = i;
                            Point p(x,y);
                            mouseClickActive = true;
                            handleMouseLeftClick(p, cnum);
                        }
                    }
                }
            }
            ofile.close();

        }
		else if (c == ' ') {
			for (i = 0; i < NUMCAMS; i++)
			{
				zoomCenters[i] = Point(XWIDTH/2, YWIDTH/2);
				zoomAmounts[i] = 1.f;
			}
		} 
		else if (c >= '1' && c <= '5') {
			pointCurrentlyClicking = 2*(c - '1');            
		} else if (c == 'q') { pointCurrentlyClicking = 2*5; } 
		 else if (c == 'w') { pointCurrentlyClicking = 2*6; } 
		 else if (c == 'e') { pointCurrentlyClicking = 2*7; } 
		 else if (c == 'r') { pointCurrentlyClicking = 2*8; } 
		 else if (c == 't') { pointCurrentlyClicking = 2*9; } 
		 else if (c == 'a') { pointCurrentlyClicking = 2*10; } 
		 else if (c == 's') { pointCurrentlyClicking = 2*11; } 
		 else if (c == 'd') { pointCurrentlyClicking = 2*12; } 
		 else if (c == 'f') { pointCurrentlyClicking = 2*13; } 
		 else if (c == 'g') { pointCurrentlyClicking = 2*14; } 
		 else if (c == 'z') { pointCurrentlyClicking = 2*15; } 
		 else if (c == 'x') { pointCurrentlyClicking = 2*16; } 
		 else if (c == 'c') { pointCurrentlyClicking = 2*17; } 
		 else if (c == 'v') { pointCurrentlyClicking = 2*18; } 
		 else if (c == 'b') { pointCurrentlyClicking = 2*19; } 
		 else if (c == 'Q') { pointCurrentlyClicking = 2*20; } 
		 else if (c == 'W') { pointCurrentlyClicking = 2*21; } 
		 else if (c == 'E') { pointCurrentlyClicking = 2*22; } 
		 else if (c == 'R') { pointCurrentlyClicking = 2*23; } 
		 else if (c == 'T') { pointCurrentlyClicking = 2*24; } 
		 else if (c == 'A') { pointCurrentlyClicking = 2*25; } 
		 else if (c == 'S') { pointCurrentlyClicking = 2*26; } 
		 else if (c == 'D') { pointCurrentlyClicking = 2*27; } 
		 else if (c == 'F') { pointCurrentlyClicking = 2*28; } 
		 else if (c == 'G') { pointCurrentlyClicking = 2*29; } 
		 else if (c == 'Z') { pointCurrentlyClicking = 2*30; } 
		 else if (c == 'X') { pointCurrentlyClicking = 2*31; } 
		 else if (c == 'C') { pointCurrentlyClicking = 2*32; } 
		 else if (c == 'V') { pointCurrentlyClicking = 2*33; } 
		 else if (c == 'B') { pointCurrentlyClicking = 2*34; } 
		 else if (c == '`') { pointCurrentlyClicking = min(pointCurrentlyClicking+1, MAX_POINTS); }
		 else if (c == '~') { pointCurrentlyClicking = max (pointCurrentlyClicking-1, 0); }
		 else if (c == '\t') {try_to_intersct_epipolars(); }

/*
		} else if (c == 's') {
			pointCurrentlyClicking = 11;
		} else if (c == '#') {
			pointCurrentlyClicking = 12;
		} else if (c == '$') {
			pointCurrentlyClicking = 13;
		} else if (c == '%') {
			pointCurrentlyClicking = 14;
		} else if (c == '^') {
			pointCurrentlyClicking = 15;
		} else if (c == '&') {
			pointCurrentlyClicking = 16;
		} else if (c == '*') {
			pointCurrentlyClicking = 17;
		} else if (c == '(') {
			pointCurrentlyClicking = 18;
		} else if (c == ')') {
			pointCurrentlyClicking = 19;
		} else if (c == 'q') {
			pointCurrentlyClicking = 20;
		} else if (c == 'w') {
			pointCurrentlyClicking = 21;
		} else if (c == 'e') {
			pointCurrentlyClicking = 22;
		} else if (c == 'r') {
			pointCurrentlyClicking = 23;
		} else if (c == 't') {
			pointCurrentlyClicking = 24;
		} else if (c == 'y') {
			pointCurrentlyClicking = 25;
		} else if (c == 'u') {
			pointCurrentlyClicking = 26;
		} else if (c == 'i') {
			pointCurrentlyClicking = 27;
		} else if (c == 'o') {
			pointCurrentlyClicking = 28;
		} else if (c == 'p') {
			pointCurrentlyClicking = 29;
		} else if (c == 'Q') {
			pointCurrentlyClicking = 30;
		} else if (c == 'W') {
			pointCurrentlyClicking = 31;
		} else if (c == 'E') {
			pointCurrentlyClicking = 32;
		} else if (c == 'R') {
			pointCurrentlyClicking = 33;
		} else if (c == 'T') {
			pointCurrentlyClicking = 34;
		} else if (c == 'Y') {
			pointCurrentlyClicking = 35;
		} else if (c == 'U') {
			pointCurrentlyClicking = 36;
		} else if (c == 'I') {
			pointCurrentlyClicking = 37;
		} else if (c == 'O') {
			pointCurrentlyClicking = 38;
		} else if (c == 'P') {
			pointCurrentlyClicking = 39;
		}
		*/

		//Check if we are done getting points
		if (finish)
			break;

	}


}

void StereoOnClicks::getClickCoords(Point2i* points)
{
	for (int i=0; i < NUMCAMS; i++)
	{
		points[i] = clickCoords[i][0];
	}

}

void StereoOnClicks::updateImages()
{
	int numCamsDone = 0;
	bool thisCamDone[3] = {false, false, false};
	while(numCamsDone < NUMCAMS)
	{
		for (int i = 0; i < NUMCAMS; i++)
		{
			if (!thisCamDone[i])
			{
				if (captures[i]->waitForFrameUndistorted())
				{
					thisCamDone[i] = true;
					numCamsDone++; 
					frames[i] = captures[i]->currentFrame();
					imshow(camNames[i], frames[i]);

                }
			}
		}
	}

	waitKey(10);

}


void StereoOnClicks::updateImagesBlocking()
{
	for (int i = 0; i < NUMCAMS; i++)
	{
		captures[i]->startFrameGrab();
	}

	for (int i = 0; i < NUMCAMS; i++)
	{
		captures[i]->grabFrameAlreadyQueuedUndistorted();
		frames[i] = captures[i]->currentFrame();
	}
}

void StereoOnClicks::blurImages()
{
	Size2i siz(3,3);

	for (int camNum=0; camNum < NUMCAMS; camNum++)
	{
		GaussianBlur(frames[camNum], frames[camNum], siz, 1.5, 1.5);
	}

}

/*thread1coords.x = 849;
	thread1coords.y = 752;
	thread2coords.x = 796;
	thread2coords.y = 900;
	thread3coords.x = 933;
	thread3coords.y = 728;
	Point3f threadPoint3d(0,0,0);
	get3dPoint(thread1coords, thread2coords, thread3coords, threadPoint3d);
	cout << "Thread Point 3d: " << threadPoint3d.x << " " << threadPoint3d.y << " " <<  threadPoint3d.z << endl;

	Mat mat3d = Mat(3,1,CV_32FC1);
	mat3d.at<float>(0,0) = threadPoint3d.x;
	mat3d.at<float>(1,0) = threadPoint3d.y;
	mat3d.at<float>(2,0) = threadPoint3d.z;


	c1->projectPointUndistorted(mat3d, cam1clickcoords);
	c2->projectPointUndistorted(mat3d, cam2clickcoords);
	c3->projectPointUndistorted(mat3d, cam3clickcoords);

	cout << "Cam 1 coords: " << cam1clickcoords.x << " " << cam1clickcoords.y << endl;
	cout << "Cam 2 coords: " << cam2clickcoords.x << " " << cam2clickcoords.y << endl;
	cout << "Cam 3 coords: " << cam3clickcoords.x << " " << cam3clickcoords.y << endl;
	*/



void StereoOnClicks::drawCrosshair(Mat& img, Point& pt, int length, const Scalar& color)
{
	Point left(pt);
	left.x = max(0,pt.x-length);
	Point right(pt);
	right.x = min(img.cols-1, pt.x+length);
	Point up(pt);
	up.y = max(0,pt.y-length);
	Point down(pt);
	down.y = min(img.rows-1, pt.y+length);

	line(img, left, right, color);
	line(img, up, down, color);
}

void StereoOnClicks::drawPointNumText(Mat& img, Point& pt, int pt_num, Scalar& color)
{
	std::stringstream to_draw;
	to_draw << (pt_num+1);
	
	Point loc_to_draw = pt;
	loc_to_draw.y -= 8;
	loc_to_draw.x -= 12;


	putText(img, to_draw.str(), loc_to_draw, FONT_HERSHEY_PLAIN, 2.0, color);
}


void StereoOnClicks::mouseHandlerWrap1(int event, int x, int y, int flags, void *param)
{
	StereoOnClicks* stereo = (StereoOnClicks*)param;
	stereo->mouseHandlerCam(event, x, y, flags, param, 0);
}

void StereoOnClicks::mouseHandlerWrap2(int event, int x, int y, int flags, void *param)
{
	StereoOnClicks* stereo = (StereoOnClicks*)param;
	stereo->mouseHandlerCam(event, x, y, flags, param, 1);
}

void StereoOnClicks::mouseHandlerWrap3(int event, int x, int y, int flags, void *param)
{
	StereoOnClicks* stereo = (StereoOnClicks*)param;
	stereo->mouseHandlerCam(event, x, y, flags, param, 2);
}

//todo: pass camcoords into param. 
void StereoOnClicks::mouseHandlerCam(int event, int x, int y, int flags, void *param, int camNum)
{
	Point pt(x,y);
	switch(event) {
		case CV_EVENT_LBUTTONDOWN:
			mouseClickActive = true;
			handleMouseLeftClick(pt, camNum);
			break;
		case CV_EVENT_RBUTTONDOWN:
			mouseClickActive = true;
			handleMouseRightClick(pt, camNum);
			break;
		case CV_EVENT_MOUSEMOVE:
			handleMouseMovement(pt, camNum);
			break;
		case CV_EVENT_MBUTTONDOWN:
			mouseClickActive = true;
			handleMouseMiddleClick(pt, camNum);
			break; 
	}
	mouseClickActive = false;
}

Rect StereoOnClicks::zoomRect(Point& zoomCen, float zoom, int xwidth, int ywidth)
{
	int x = zoomCen.x - xwidth/(2*zoom);
	x = max(x,0);
	int y = zoomCen.y - ywidth/(2*zoom);
	y = max(y,0);
	int wx = xwidth/(zoom);
	wx = min(wx, xwidth-x);
	int wy = ywidth/(zoom);
	wy = min(wy, ywidth-y);
	return Rect(x,y,wx,wy);
}

Point StereoOnClicks::unzoomedPoint(Point& pt, int xwidth, int ywidth, Point& zoomCen, float zoom)
{
	int x = pt.x/zoom + zoomCen.x - xwidth/(2*zoom); 
	x = min(max(x,0),xwidth);
	int y = pt.y/zoom + zoomCen.y - ywidth/(2*zoom); 
	y = min(max(y,0),ywidth);
	return Point(x,y);

}

void StereoOnClicks::populateEpipolarLinePoints(int camWithPoint, Point& pointFromCam)
{
	Point2f newPoint(((float)pointFromCam.x), ((float)pointFromCam.y));
	populateEpipolarLinePoints(camWithPoint, newPoint);
}

void StereoOnClicks::populateEpipolarLinePoints(int camWithPoint, Point2f& pointFromCam)
{

	for (int i = 0; i < NUMCAMS; i++)
	{
		if (i == camWithPoint)
			continue;
		Mat lineParams = Mat(3,1,CV_32FC1);

		captures[i]->GetEpipolarLine(*captures[camWithPoint], pointFromCam, lineParams);
		float a = lineParams.at<float>(0,0);
		float b = lineParams.at<float>(1,0);
        float c = lineParams.at<float>(2,0);


        Point p1;
        Point p2;

        if (abs(a) > abs(b))
        {
            //line is more vertical
            p1.y = 0;
            p2.y = YWIDTH;
            p1.x = (int)(0.5 + (-c)/a);
            p2.x = (int)(0.5 + (-c-b*p2.y)/a);
		} else
		{
			//line is more horizontal
			p1.x = 0;
			p2.x = XWIDTH;
			p1.y = (int)(0.5 + (-c)/b);
			p2.y = (int)(0.5 + (-c-a*p2.x)/b);
		}

		epipolarInfo[i][camWithPoint][pointCurrentlyClicking].pt1 = p1;
		epipolarInfo[i][camWithPoint][pointCurrentlyClicking].pt2 = p2;
		epipolarInfo[i][camWithPoint][pointCurrentlyClicking].a = a;
		epipolarInfo[i][camWithPoint][pointCurrentlyClicking].b = b;
		epipolarInfo[i][camWithPoint][pointCurrentlyClicking].c = c;

		//if this wasn't clicked before, and >= 2 lines, automatically select point
		/*if (!clickedOnCamera[i][pointCurrentlyClicking])
		{
			int numClicked = 1;
			float a2, b2, c2;
			for (int j = 0; j < NUMCAMS; j++)
			{
				if (j!= camWithPoint && clickedOnCamera[j][pointCurrentlyClicking]) {
					numClicked++;        
					a2 = epipolarInfo[i][j][pointCurrentlyClicking].a; 
					b2 = epipolarInfo[i][j][pointCurrentlyClicking].b; 
					c2 = epipolarInfo[i][j][pointCurrentlyClicking].c; 
				}
			}

			if (numClicked >= 2)
			{
				float y = (a*c2 - a2*c )/(a2*b - a*b2);
				float x = (-c-b*y) / a;
				Point pNewPt( (int)(x+0.5), (int)(y+0.5));
				if (pNewPt.x >= 0 && pNewPt.x < XWIDTH && pNewPt.y >=0 && pNewPt.y < YWIDTH)
				{
					clickCoords[i][pointCurrentlyClicking].x = pNewPt.x;
					clickCoords[i][pointCurrentlyClicking].y = pNewPt.y;
					clickedOnCamera[i][pointCurrentlyClicking] = true;
					populateEpipolarLinePoints(i, pNewPt);
				}
			}

		}*/

	}

}

//assumes NUMCAMS=3
void StereoOnClicks::try_to_intersct_epipolars()
{
	int numNotClicked = 0;
	int im_not_clicked;

	for (int camNum=0; camNum < NUMCAMS; camNum++)
	{
		if (!clickedOnCamera[camNum][pointCurrentlyClicking])
		{
			numNotClicked++;
			im_not_clicked = camNum;
		}
	}

	if (numNotClicked != 1)
		return;

	double a[NUMCAMS-1];
	double b[NUMCAMS-1];
	double c[NUMCAMS-1];

	int epipolar_ind = 0;
	for (int camNum=0; camNum < NUMCAMS; camNum++)
	{
		if (camNum == im_not_clicked)
			continue;

		a[epipolar_ind] = epipolarInfo[im_not_clicked][camNum][pointCurrentlyClicking].a; 
		b[epipolar_ind] = epipolarInfo[im_not_clicked][camNum][pointCurrentlyClicking].b; 
		c[epipolar_ind] = epipolarInfo[im_not_clicked][camNum][pointCurrentlyClicking].c; 
		epipolar_ind++;

	}

	if (numNotClicked == 1)
	{
				float y = (a[0]*c[1] - a[1]*c[0] )/(a[1]*b[0] - a[0]*b[1]);
				float x = (-c[0]-b[0]*y) / a[0];
				Point pNewPt( (int)(x+0.5), (int)(y+0.5));
				if (pNewPt.x >= 0 && pNewPt.x < XWIDTH && pNewPt.y >=0 && pNewPt.y < YWIDTH)
				{
					clickCoords[im_not_clicked][pointCurrentlyClicking].x = pNewPt.x;
					clickCoords[im_not_clicked][pointCurrentlyClicking].y = pNewPt.y;
					clickedOnCamera[im_not_clicked][pointCurrentlyClicking] = true;
					populateEpipolarLinePoints(im_not_clicked, pNewPt);
				}

	}
	



}


/*
void StereoOnClicks::checkOtherCamsForOrange(int camWithPoint, Point2f& pointFromCam)
{
	//make sure the other images haven't been clicked yet, and this is the first time we clicked on this image
	int numClicked = 0;
	for (int i=0; i < NUMCAMS; i++)
	{
		if (clickedOnCamera[i][pointCurrentlyClicking])
			numClicked++;
	}
	if (numClicked != 0 )
		return;


	vector<Mat> planes_BGR_withPt;
	split(frames[camWithPoint], planes_BGR_withPt);

	Mat frame_hsv_withPt;
	vector<Mat> planes_HSV_withPt;
	cvtColor(frames[camWithPoint], frame_hsv_withPt, CV_BGR2HSV);
	split(frame_hsv_withPt, planes_HSV_withPt);

	vector<vector<pointCluster> > clusters_per_cam;
	clusters_per_cam.resize(NUMCAMS);
	
	

	clusters_per_cam[camWithPoint].resize(1);
	findNearbyOrange(planes_BGR_withPt, planes_HSV_withPt, (int)pointFromCam.y, (int)pointFromCam.x, clusters_per_cam[camWithPoint][0]);


	clickCoords[camWithPoint][pointCurrentlyClicking].x = clusters_per_cam[camWithPoint][0].centerOfPoints.x;
	clickCoords[camWithPoint][pointCurrentlyClicking].y = clusters_per_cam[camWithPoint][0].centerOfPoints.y;
	populateEpipolarLinePoints(camWithPoint, clusters_per_cam[camWithPoint][0].centerOfPoints);


	for (int i = 0; i < NUMCAMS; i++)
	{
		if (i == camWithPoint)
			continue;


		vector<Mat> planes_BGR;
		split(frames[i], planes_BGR);

		Mat frame_hsv;
		vector<Mat> planes_HSV;
		cvtColor(frames[i], frame_hsv, CV_BGR2HSV);
		split(frame_hsv, planes_HSV);

		LineIterator lineIter(frames[i], epipolarInfo[i][camWithPoint][pointCurrentlyClicking].pt1, epipolarInfo[i][camWithPoint][pointCurrentlyClicking].pt2, 8);


		for (int ptNum=0; ptNum < lineIter.count; ptNum++, ++lineIter)
		{
			int offset, x, y;
			offset = lineIter.ptr - (uchar*)(frames[i].data);
			y = offset/(frames[i].step);
			x = (offset - y*frames[i].step)/(3*sizeof(uchar));

			if (isOrange(planes_BGR, planes_HSV, y, x))
			{
					std::cout << "ORANGE AT " << i << ", " << y << ", " << x << std::endl;
			}

		}


	}


}


bool StereoOnClicks::isOrange(vector<Mat>& planes_BGR, vector<Mat>& planes_HSV, int y, int x)
{
	return (planes_HSV[1].at<uchar>(y,x) > 80 && planes_BGR[2].at<uchar>(y,x) > 80);
}

void StereoOnClicks::findNearbyOrange(vector<Mat>& planes_BGR, vector<Mat>& planes_HSV, int y, int x, pointCluster& clust)
{
	Point orig((float)y, (float)x);
	clust.points.resize(0);
	clust.points.push_back(convertToFloat(orig));
	
	stack<Point> toProcess;
	toProcess.push(orig);

	while (!toProcess.empty())
	{
		Point next = toProcess.top();
		toProcess.pop();

		for (int delY=-DIST_TO_CLUST; delY<= DIST_TO_CLUST; delY++)
		{
			for (int delX=-DIST_TO_CLUST; delX<= DIST_TO_CLUST; delX++)
			{
				if (isOrange(planes_BGR, planes_HSV, next.y+delY, next.x+delX))
				{
					Point newOrange(next.y+delY, next.x+delX);
					Point2f next_float = convertToFloat(newOrange);
					bool contains = false;
					for (int i=0; i < clust.points.size(); i++)
					{
						if (clust.points[i] == next_float)
						{
							contains = true;
							break;
						}
					}
					if (!contains)
					{
						toProcess.push(newOrange);
						clust.points.push_back(next_float);
					}

				}
			}
		}
	}

	clust.centerOfPoints.x = 0;
	clust.centerOfPoints.y = 0;
	std::cout << "num detected: " << clust.points.size() << std::endl;

	for (int i=0; i < clust.points.size(); i++)
	{
		clust.centerOfPoints.x += clust.points[i].y;
		clust.centerOfPoints.y += clust.points[i].x;
	}
	clust.centerOfPoints.x /= (float)clust.points.size();
	clust.centerOfPoints.y /= (float)clust.points.size();


	
}
*/


///////////////////////////////////////////////////////////////////////////////////////////
/******************************* Least squares utilities *********************************/
///////////////////////////////////////////////////////////////////////////////////////////

void StereoOnClicks::get3dPoint(const Point& p1, const Point& p2, const Point& p3, Point3f& point)
{
	Mat ray1(3,1,CV_32FC1);
	Mat ray2(3,1,CV_32FC1);
	Mat ray3(3,1,CV_32FC1);

	captures[0]->getWorldRay(p1, ray1);
	captures[1]->getWorldRay(p2, ray2);
	captures[2]->getWorldRay(p3, ray3);

	Mat matForPoint = Mat(6,1,CV_32FC1);

	LeastSquares3Cam(ray1, ray2, ray3, captures[0]->cameraPosition(), captures[1]->cameraPosition(), captures[2]->cameraPosition(), matForPoint); 

	point.x = matForPoint.at<float>(0,0);
	point.y = matForPoint.at<float>(1,0);
	point.z = matForPoint.at<float>(2,0);
}



///////////////////////////////////////////////////////////////////////////////////////////
/******************************* Mouse Event Handlers ************************************/
///////////////////////////////////////////////////////////////////////////////////////////
void StereoOnClicks::handleMouseLeftClick(Point pt, int camNum)
{
	Point ptUnzoomed = unzoomedPoint(pt, frames[camNum].cols, frames[camNum].rows, zoomCenters[camNum], zoomAmounts[camNum]);
	clickCoords[camNum][pointCurrentlyClicking].x = ptUnzoomed.x;
	clickCoords[camNum][pointCurrentlyClicking].y = ptUnzoomed.y;
	clickedOnCamera[camNum][pointCurrentlyClicking]= true;
	populateEpipolarLinePoints(camNum, ptUnzoomed);
	//checkOtherCamsForOrange(camNum, newPointUnzoomed);
}

void StereoOnClicks::handleMouseMiddleClick(Point pt, int camNum)
{
	Point ptUnzoomed = unzoomedPoint(pt, frames[camNum].cols, frames[camNum].rows, zoomCenters[camNum], zoomAmounts[camNum]);
	zoomAmounts[camNum] *= 2;
	zoomCenters[camNum].x = min(max(ptUnzoomed.x, (int) (XWIDTH/(2*zoomAmounts[camNum]))), (int)(XWIDTH - XWIDTH/(2*zoomAmounts[camNum])));
	zoomCenters[camNum].y = min(max(ptUnzoomed.y, (int) (YWIDTH/(2*zoomAmounts[camNum]))), (int)(YWIDTH - YWIDTH/(2*zoomAmounts[camNum])));
}


void StereoOnClicks::handleMouseRightClick(Point pt, int camNum)
{
	Point ptUnzoomed = unzoomedPoint(pt, frames[camNum].cols, frames[camNum].rows, zoomCenters[camNum], zoomAmounts[camNum]);
	zoomAmounts[camNum] *= 0.5;
	zoomCenters[camNum].x = min(max(ptUnzoomed.x, (int) (XWIDTH/(2*zoomAmounts[camNum]))), (int)(XWIDTH - XWIDTH/(2*zoomAmounts[camNum])));
	zoomCenters[camNum].y = min(max(ptUnzoomed.y, (int) (YWIDTH/(2*zoomAmounts[camNum]))), (int)(YWIDTH - YWIDTH/(2*zoomAmounts[camNum])));
}

void StereoOnClicks::handleMouseMovement(Point pt, int camNum)
{
	Point ptUnzoomed = unzoomedPoint(pt, frames[camNum].cols, frames[camNum].rows, zoomCenters[camNum], zoomAmounts[camNum]);
	mouseCoords[camNum].x = ptUnzoomed.x;
	mouseCoords[camNum].y = ptUnzoomed.y;
}


bool operator <(const point3d_from_clusters& a, const point3d_from_clusters& b)
{
	return a.score < b.score;
}

/*

	 TVector3 calculateWorldPoint3(TVector2 imagePoint1, TVector2 imagePoint2, TVector2 imagePoint3, Capture* cam1, Capture* cam2, Capture* cam3){
//std::cout << "worldray0 = "; imagePoint0.transpose().print(std::cout);
//std::cout << "worldray1 = "; imagePoint1.transpose().print(std::cout);
return calculateWorldPoint3(pixelToWorldRay(imagePoint1, cam1), pixelToWorldRay(imagePoint2, cam2), pixelToWorldRay(imagePoint3, cam3), getCameraPosition(cam1), getCameraPosition(cam2), getCameraPosition(cam3));
}

TVector3 calculateWorldPoint3(TVector3 worldRay1, TVector3 worldRay2, TVector3 worldRay3, TVector3 camPos1, TVector3 camPos2, TVector3 camPos3) {
//std::cout << "worldRay0 " << std::endl; worldRay0.print(std::cout);
//std::cout << "worldRay1 " << std::endl; worldRay1.print(std::cout);

TMatrix<9,6> leastSquaresInput;
/* In the form [1 0 0 x0 0 0;
 *              0 1 0 y0 0 0;
 *              0 0 1 z0 0 0;
 *              1 0 0 0 x1 0;
 *              0 1 0 0 y1 0;
 *              0 0 1 0 z1 0;
 *			  1 0 0 0 0 x2;
 *			  0 1 0 0 0 y2;
 *			  0 0 1 0 0 z2;]
 */ /*
			 fillLeastSquaresMatrix(leastSquaresInput);
			 leastSquaresInput[0][3] = worldRay1[0];
			 leastSquaresInput[1][3] = worldRay1[1];
			 leastSquaresInput[2][3] = worldRay1[2];
			 leastSquaresInput[3][4] = worldRay2[0];
			 leastSquaresInput[4][4] = worldRay2[1];
			 leastSquaresInput[5][4] = worldRay2[2];
			 leastSquaresInput[6][5] = worldRay3[0];
			 leastSquaresInput[7][5] = worldRay3[1];
			 leastSquaresInput[8][5] = worldRay3[2];

			 std::cout << "leastSquaresInput: " << std::endl; leastSquaresInput.print(std::cout);
			 TMatrix<9,1> cameraWorldCoordinates;
			 fillTargetVector(cameraWorldCoordinates, camPos1, camPos2, camPos3);
			 std::cout << "cameraWorldCoordinates: " << std::endl; cameraWorldCoordinates.print(std::cout);

//  TMatrix<6,1> estimatedParams = leastSquares(leastSquaresInput, cameraWorldCoordinates);
//  std::cout << "estimatedParams " << std::endl; estimatedParams.print(std::cout);
//  TVector3 worldPoint(estimatedParams[0], estimatedParams[1], estimatedParams[2]);
return NULL; //worldPoint;
}

/*
TMatrix<6,1> leastSquares(TMatrix<9,6> X, TMatrix<9,1> y) {
TMatrix<6,1> theta = (X.transpose() * X).inverse() * X.transpose() * y;
return theta;
}
*/
/*
	 void fillLeastSquaresMatrix(TMatrix<9,6> &X) {
	 X[0][0] = 1; X[1][1] = 1; X[2][2] = 1;
	 X[3][0] = 1; X[4][1] = 1; X[5][2] = 1;
	 X[6][0] = 1; X[7][1] = 1; X[8][2] = 1;

	 }

	 void fillTargetVector(TMatrix<9,1> &target, TMatrix<3,1> p0, TMatrix<3,1> p1, TMatrix<3,1> p2) {
	 target[0] = p0[0]; target[1] = p0[1]; target[2] = p0[2];
	 target[3] = p1[0]; target[4] = p1[1]; target[5] = p1[2];
	 target[6] = p1[0]; target[7] = p1[1]; target[8] = p2[2];
	 }

//////////////////////////////////////////////////////////////////////////////////////////////
/******************************** Transformation utilities **********************************/
//////////////////////////////////////////////////////////////////////////////////////////////
/*
// placeholder, stupid.
TVector3 pixelToWorldRay(TVector2 imagePoint, Capture *cam) {
return TVector3(imagePoint[0],imagePoint[1],0);	
}

// coordinates may not be in the right frame
TVector3 getCameraPosition(Capture * cam) {
CvMat* cameraPosition = cam->cameraPosition();
int x = cvGetReal1D(cameraPosition,0);
int y = cvGetReal1D(cameraPosition,1);
int z = cvGetReal1D(cameraPosition,2);
return TVector3(x,y,z);
}*/


