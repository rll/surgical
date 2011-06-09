#include "capture2.h"
#include "ThreeCam.h"
#include "util2.h"
#include "../utils/clock.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <time.h>
#include <list>
#include <stdio.h>

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
  //_captures[0]->setExposure(4900);
  //_captures[0]->setExposure(22000);
  _captures[0]->setExposure(5000);
  

  _captures[1] = new Capture(1, //id
          _names[1].c_str(),// cam name
          650,  // gain
          "optimized", // optimized or measured
          107110); // camera uid
          //"/home/pabbeel/code/master-slave/vision2/stereo_test/stereo_test2-");
          //"./suture_sets/set3/suture1_2-"); 
  //_captures[1]->setExposure(3000);
  //_captures[1]->setExposure(16000);
  _captures[1]->setExposure(3300);

  _captures[2] = new Capture(2, //id
          _names[2].c_str(),// cam name
          650,  // gain
          "optimized", // optimized or measured
          107111); // camera uid
          //"/home/pabbeel/code/master-slave/vision2/stereo_test/stereo_test3-");
          //"./suture_sets/set3/suture1_3-"); 
  //_captures[2]->setExposure(4000);
  //_captures[2]->setExposure(18000);
  _captures[2]->setExposure(3600);

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

 
 
  /*FileStorage file1;
  FileStorage file2;
  FileStorage file3;
  file1.open("/media/ssd1/captures/%s1.yml", FileStorage::WRITE);
  file2.open("/media/ssd1/captures/%s2.yml", FileStorage::WRITE);
  file3.open("/media/ssd1/captures/%s3.yml", FileStorage::WRITE);
  */

  
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

  /*
  _cams->updateImagesNoUndistort();
  _frames = _cams->frames();
  int bytesPerIm = _frames[0].elemSize()*_frames[0].rows*_frames[0].cols;
  */

  char filename[256];
  char filename1[256];
  char filename2[256];
  char filename3[256];
  int f=atoi(argv[2]);


  list<Mat> imsSaved[NUMCAMS];

  /*
  FILE* byteFiles[NUMCAMS];
  for (int camNum=0; camNum < NUMCAMS; camNum++)
  {
    sprintf(filename, "/media/ssd1/captures/%s%d.bin", argv[1],(camNum+1));
    byteFiles[camNum] = fopen(filename, "w+");
  }*/

  bool continuous=false;
  while(true) {

    char c = cvWaitKey(20);
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
        maxTime = GetClock()+240.0;
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
     
      /*    
      sprintf(filename1, "/media/ssd1/captures/%s1-%d.tif", argv[1],f);
      sprintf(filename2, "/media/ssd1/captures/%s2-%d.tif", argv[1],f);
      sprintf(filename3, "/media/ssd1/captures/%s3-%d.tif", argv[1],f);
      Mat im1 = _frames[0].clone();
      imwrite(filename1,im1);
      Mat im2 = _frames[1].clone();
      imwrite(filename2,im2);
      Mat im3 = _frames[2].clone();
      imwrite(filename3,im3);    
      f++;
      
      */
      

      /*
      sprintf(filename1, "%s1-%d", argv[1],f);
      sprintf(filename2, "%s2-%d", argv[1],f);
      sprintf(filename3, "%s3-%d", argv[1],f);
      file1 << "mtx" << _frames[0];
      file2 << "mtx" << _frames[1];
      file3 << "mtx" << _frames[2];
      f++;
*/


      
      for (int camNum=0; camNum < NUMCAMS; camNum++)
      {
        imsSaved[camNum].push_back(_frames[camNum].clone());
      }
      

      /*
      for (int camNum=0; camNum < NUMCAMS; camNum++)
      {
        fwrite(_frames[camNum].data, 1, bytesPerIm, byteFiles[camNum]);
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

  
  for (int camNum=0; camNum < NUMCAMS; camNum++)
  {
    list<Mat>::iterator it;

    int f = 1;
    for ( it=imsSaved[camNum].begin() ; it != imsSaved[camNum].end(); it++ ) 
    {
      sprintf(filename, "captures/%s%d-%d.tif", argv[1],camNum+1,f);
      imwrite(filename,*it);
      f++;
    }
  }
  

/*
  //FOR TESTING - save ims from file...
  for (int camNum=0; camNum < NUMCAMS; camNum++)
  {
    rewind(byteFiles[camNum]);
    fseek (byteFiles[camNum] , 0 , SEEK_END);
    int numIms = ftell(byteFiles[camNum])/bytesPerIm;
    std::cout << "num ims: " << numIms << std::endl;
    rewind(byteFiles[camNum]);

    Mat im = _frames[0].clone();
    for (int i=0; i < numIms; i++)
    {
      std::cout << i << std::endl;
      fread(im.data, 1, bytesPerIm, byteFiles[camNum]);
      sprintf(filename, "/media/ssd1/captures/%s%d-%d.tif", argv[1],(camNum+1),(i+atoi(argv[2])));
      imwrite(filename,im);
    }
  }*/


  /*
  file1.release();
  file2.release();
  file3.release();
  */

  for (int i=0; i < NUMCAMS; i++){
    _captures[i]->endCapture();
    delete _captures[i];
  }


}

