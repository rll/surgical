#include "ThreeCam.h"

int main(int argc, char** argv)
{
  Capture* captures[3];
  string names[] = {"cam1", "cam2", "cam3"};
  captures[0] = new Capture(0, //id
          names[0].c_str(), // cam name
          650,  // gain
          "optimized", // optimized or measured
          107109);
          //"./calib-mar12/calib1-");
          //"./captures/extrins-4-293-"); // camera uid
  cout << "c1 created" << endl;

  captures[1] = new Capture(1, //id
          names[1].c_str(),// cam name
          650,  // gain
          "optimized", // optimized or measured
          107110);
          //"./calib-mar12/calib2-");
          //"./captures/extrins-4-293-"); // camera uid
  cout << "c2 created" << endl;

  captures[2] = new Capture(2, //id
          names[2].c_str(),// cam name
          650,  // gain
          "optimized", // optimized or measured
          107111);
         // "./calib-mar12/calib3-");
          //"./captures/extrins-4-293-"); // camera uid
  cout << "c3 created" << endl;  

  namedWindow(names[0], CV_WINDOW_AUTOSIZE);
  namedWindow(names[1], CV_WINDOW_AUTOSIZE);
  namedWindow(names[2], CV_WINDOW_AUTOSIZE);

  captures[0]->init("./calib-apr21/");
  captures[1]->init("./calib-apr21/");
  captures[2]->init("./calib-apr21/");
  
  cvWaitKey(500);							// segfaulted without this
 
	
  captures[0]->AddOtherCameraInformation(*captures[1]);
  captures[0]->AddOtherCameraInformation(*captures[2]);
  captures[1]->AddOtherCameraInformation(*captures[0]);
  captures[1]->AddOtherCameraInformation(*captures[2]);
  captures[2]->AddOtherCameraInformation(*captures[0]);
  captures[2]->AddOtherCameraInformation(*captures[1]);


  ThreeCam cams = ThreeCam(captures);
	float width[] = {1.55, 1.55, 1.55};
	float edge_sigma[] = {0.6, 0.6, 0.6};
	float blur_sigma[] = {1.0, 1.0, 1.0};
	double thresh1[] = {15.0, 17.0, 14.0};
	double thresh2[] = {140.0, 140.0, 130.0};
	cams.initializeCanny (width, edge_sigma, blur_sigma, thresh1, thresh2);
	while(1)
	{
		cams.updateImagesBlocking();
		waitKey(10);
		cams.filterCanny();


		Mat* cannys = cams.cannyIms();
		Mat* frames = cams.frames();
		for (int i=0; i < NUMCAMS; i++)
		{
			cv::imshow(names[i], cannys[i]); 
		}

		waitKey(10);
	}
/* 
  captures[0] = c1;
  captures[1] = c2;
  captures[2] = c3;
*/    

}

