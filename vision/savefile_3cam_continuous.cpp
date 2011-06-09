
#include "capture2.h"
#include "ThreeCam.h"
#include "util2.h"
#include "../utils/clock.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <time.h>
#include <list>


#include <iostream>


using namespace std;


int main(int argc, char** argv)
{
		Capture* _captures[NUMCAMS];
		string _names[NUMCAMS];
		string _orig_display_names[NUMCAMS];

		Mat* _frames;
		ThreeCam* _cams;

  _names[0] = "cam1";
  _names[1] = "cam2";
  _names[2] = "cam3";

  _captures[0] = new Capture(0, //id
          _names[0].c_str(), // cam name
          650,  // gain
          "optimized", // optimized or measured
          107109);
          //"/home/pabbeel/code/master-slave/vision2/stereo_test/stereo_test1-");
          //"./suture_sets/set3/suture1_1-"); 
  _captures[0]->setExposure(5000);
  //_captures[0]->setExposure(11000);
  

  _captures[1] = new Capture(1, //id
          _names[1].c_str(),// cam name
          650,  // gain
          "optimized", // optimized or measured
          107110); // camera uid
          //"/home/pabbeel/code/master-slave/vision2/stereo_test/stereo_test2-");
          //"./suture_sets/set3/suture1_2-"); 
  _captures[1]->setExposure(3000);
  //_captures[1]->setExposure(7500);

  _captures[2] = new Capture(2, //id
          _names[2].c_str(),// cam name
          650,  // gain
          "optimized", // optimized or measured
          107111); // camera uid
          //"/home/pabbeel/code/master-slave/vision2/stereo_test/stereo_test3-");
          //"./suture_sets/set3/suture1_3-"); 
  _captures[2]->setExposure(3500);
  //_captures[2]->setExposure(8000);

  /*namedWindow(_names[0], CV_WINDOW_AUTOSIZE);
  namedWindow(_names[1], CV_WINDOW_AUTOSIZE);
  namedWindow(_names[2], CV_WINDOW_AUTOSIZE);
  */

/*  char names_char[NUMCAMS][256];
  for (int camNum=0; camNum < NUMCAMS; camNum++)
  {
    sprintf(names_char[camNum], "%s%d", DISPLAY_ORIG_BASE, camNum);
    _orig_display_names[camNum].assign(names_char[camNum]);
    namedWindow(_orig_display_names[camNum], CV_WINDOW_AUTOSIZE);
  }
*/


  //_captures[0]->init("./calib-apr21/");
  //_captures[1]->init("./calib-apr21/");
  //_captures[2]->init("./calib-apr21/");
  _captures[0]->init("./calib_params/");
  _captures[1]->init("./calib_params/");
  _captures[2]->init("./calib_params/");
  
  
  cvWaitKey(1000);							// segfaulted without this
 

  //add information about other cameras for stereo
  _captures[0]->AddOtherCameraInformation(*_captures[1]);
  _captures[0]->AddOtherCameraInformation(*_captures[2]);
  _captures[1]->AddOtherCameraInformation(*_captures[0]);
  _captures[1]->AddOtherCameraInformation(*_captures[2]);
  _captures[2]->AddOtherCameraInformation(*_captures[0]);
  _captures[2]->AddOtherCameraInformation(*_captures[1]);


  //initialize threecam wrapper
  _cams = new ThreeCam(_captures);
 /* float width[] = {1.50, 1.50, 1.50};
	float edge_sigma[] = {0.50, 0.50, 0.50};
	float blur_sigma[] = {1.5, 1.5, 1.5};
	double thresh1[] = {4.0, 4.0, 4.0};
	double thresh2[] = {80.0, 80.0, 80.0};
	_cams->initializeCanny(width, edge_sigma, blur_sigma, thresh1, thresh2);
*/


  if (argc < 3) {
    cerr << "Usage: example_capture <name> <startind>" << endl;
    return 1;
  }


  vector<Capture*> syncInCams;
  syncInCams.push_back(_captures[1]);
  syncInCams.push_back(_captures[2]);
  _captures[0]->syncFrameCaptureSetCenter(syncInCams);

 
  
  
//  c1->init("parameters/");

  waitKey(1000);
  namedWindow("Example Display1", CV_WINDOW_AUTOSIZE);
  namedWindow("Example Display2", CV_WINDOW_AUTOSIZE);
  namedWindow("Example Display3", CV_WINDOW_AUTOSIZE);


  cout << "press 'q' to exit, 'c' to capture a single frame, 'r' to capture frames continuously, and 's' to stop continuous capture\n" << endl;

  double totalTime = 0.0;
  double beforeTime = GetClock();
  double maxTime = DBL_MAX;
  double count = 0.0;

  char filename[256];
  int f=atoi(argv[2]);

 // list<Mat> imsSavedCam1;
  //list<Mat> imsSavedCam2;
 // list<Mat> imsSavedCam3;
  list<Mat> imsSaved[NUMCAMS];

  vector<int> writeParams;
  //writeParams.push_back(CV_IMWRITE_PNG_COMPRESSION);
  //writeParams.push_back(0);

  bool continuous=false;
  for(;;) {

    char c = cvWaitKey(10);
    _cams->updateImagesBlockingNoUndistort();
    _frames = _cams->frames();

   imshow("Example Display1", _frames[0]);
   imshow("Example Display2", _frames[1]);
   imshow("Example Display3", _frames[2]);

    if(c == 'q' || beforeTime > maxTime)
    {
      break;
    } else if (c == 'r')
    {
      if (continuous == false)
      {
        maxTime = GetClock()+120.0;
        continuous=true;
      }
      std::cout << "starting continuous capture" << std::endl;
    } else if (c == 's')
    {
      maxTime = DBL_MAX;
      continuous = false;
      std::cout << "stopping continuous capture" << std::endl;
    } else if ( c== 'c')
      std::cout << "capturing single frame" << std::endl;

    if (c == 'c' || continuous == true)
    {
      
      sprintf(filename, "/media/ssd1/captures/%s1-%d.xml", argv[1],f);
      imwrite(filename,_frames[0], writeParams);
      sprintf(filename, "/media/ssd1/captures/%s2-%d.xml", argv[1],f);
      imwrite(filename,_frames[1], writeParams);
      sprintf(filename, "/media/ssd1/captures/%s3-%d.xml", argv[1],f);
      imwrite(filename,_frames[2], writeParams);
      f++;
      
      /*
      for (int camNum=0; camNum < NUMCAMS; camNum++)
      {
        //imsSavedCam1.push_back(_frames[0].clone());
        //imsSavedCam2.push_back(_frames[1].clone());
        //imsSavedCam3.push_back(_frames[2].clone());
        imsSaved[camNum].push_back(_frames[camNum].clone());
      }*/

    }

    // timing3576807
    totalTime += GetClock() - beforeTime;
    beforeTime = GetClock();
    count += 1.0;
    if(count > 20) {
      cout << "Framerate: " << count/totalTime << endl;
      count = 0.0;
      totalTime = 0.0;
    }
  }

  /*for (int f=0; f < imsSavedCam1.size(); f++)
  {
    sprintf(filename, "captures/%s1-%d.png", argv[1],f);
    imwrite(filename,imsSavedCam1[f]);
    sprintf(filename, "captures/%s2-%d.png", argv[1],f);
    imwrite(filename,imsSavedCam2[f]);
    sprintf(filename, "captures/%s3-%d.png", argv[1],f);
    imwrite(filename,imsSavedCam3[f]);
  }*/
/*
  for (int camNum=0; camNum < NUMCAMS; camNum++)
  {
    list<Mat>::iterator it;

    int f = 1;
    for ( it=imsSaved[camNum].begin() ; it != imsSaved[camNum].end(); it++ ) 
    {
      sprintf(filename, "captures/%s%d-%d.png", argv[1],camNum+1,f);
      imwrite(filename,*it);
      f++;
    }
  }
*/


  for (int i=0; i < NUMCAMS; i++){
    _captures[i]->endCapture();
    delete _captures[i];
  }
}

