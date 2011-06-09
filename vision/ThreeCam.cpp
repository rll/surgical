#include "ThreeCam.h"

using namespace std;
using namespace cv;


ThreeCam::ThreeCam(Capture* cams[])
    : _saved_im_ind(1), _canny_saved_im_ind(1)
{
  //we assume the cameras have been initialized, and the cameras have each other's parameters added
  for (int i=0; i < NUMCAMS; i++)
  {
    _captures[i] = cams[i];
    _camNames[i] = string(cams[i]->name());
    _captures[i]->startCapture();
  }

  cvWaitKey(100);

  //capture one image - needed when not reading from file
  //updateImagesBlocking();


  //set camera syncs for synchronous frame capture
  vector<Capture*> syncInCams;
  syncInCams.push_back(_captures[1]);
  syncInCams.push_back(_captures[2]);
  _captures[0]->syncFrameCaptureSetCenter(syncInCams);


  for (int i=0; i < NUMCAMS; i++)
  { 
    _frames[i] = _captures[i]->currentFrame();
    _frames_gray[i] = Mat(_frames[i].size(), CV_8UC1); 
    //if (!_frames[i].empty())
    cvtColor(_frames[i], _frames_gray[i], CV_BGR2GRAY);
  }

}


ThreeCam::~ThreeCam()
{
  for (int i=0; i < NUMCAMS; i++)
    delete _cannyFilters[i];
}


void ThreeCam::updateImages()
{
  int numCamsDone = 0;
  bool thisCamDone[3] = {false, false, false};
  while(numCamsDone < NUMCAMS)
  {
    for (int i = 0; i < NUMCAMS; i++)
    {
      if (!thisCamDone[i])
      {
        if (_captures[i]->waitForFrameUndistorted())
        {
          thisCamDone[i] = true;
          numCamsDone++; 
          _frames[i] = _captures[i]->currentFrame();
          //cvtColor(_frames[i], _frames_gray[i], CV_BGR2GRAY);
        }
      }
    }
  }

  //waitKey(1);
}

void ThreeCam::updateImagesNoUndistort()
{
  int numCamsDone = 0;
  bool thisCamDone[3] = {false, false, false};
  while(numCamsDone < NUMCAMS)
  {
    for (int i = 0; i < NUMCAMS; i++)
    {
      if (!thisCamDone[i])
      {
        if (_captures[i]->waitForFrame())
        {
          thisCamDone[i] = true;
          numCamsDone++; 
          _frames[i] = _captures[i]->currentFrame();
          //cvtColor(_frames[i], _frames_gray[i], CV_BGR2GRAY);
        }
      }
    }
  }

  //waitKey(1);
}

void ThreeCam::updateImagesBlocking()
{
  for (int i = 0; i < NUMCAMS; i++)
  {
    _captures[i]->startFrameGrab();
  }

  for (int i = 0; i < NUMCAMS; i++)
  {
    _captures[i]->grabFrameAlreadyQueuedUndistorted();
    _frames[i] = _captures[i]->currentFrame();
    //cvtColor(_frames[i], _frames_gray[i], CV_BGR2GRAY);
  }
}

void ThreeCam::updateTimestamps()
{
  for (int camNum=0; camNum < NUMCAMS; camNum++)
  {
    _timestamps[camNum] = _captures[camNum]->currentTimestamp();
  }
}

void ThreeCam::updateImagesBlockingNoUndistort()
{
  for (int i = 0; i < NUMCAMS; i++)
  {
    _captures[i]->startFrameGrab();
  }

  for (int i = 0; i < NUMCAMS; i++)
  {
    _captures[i]->grabFrameAlreadyQueued();
    _frames[i] = _captures[i]->currentFrame();
    //cvtColor(_frames[i], _frames_gray[i], CV_BGR2GRAY);
  }
}

void ThreeCam::initializeCanny(float width[], float edge_sigma[], float blur_sigma[], double thresh1[], double thresh2[])
{
  for (int i = 0; i < NUMCAMS; i++)
  {
    _cannyFilters[i] = new CannyOrient(width[i], edge_sigma[i], blur_sigma[i]);
    _canny_Thresh1[i] = thresh1[i];
    _canny_Thresh2[i] = thresh2[i];

    _cannyIms[i] = Mat(_frames[i].size(), CV_8UC1);
    _cannyAngs[i] = Mat(_frames[i].size(), CV_32FC1);
  }

}

void ThreeCam::convertToGrayscale()
{


#ifdef NYLON 
  for (int i = 0; i < NUMCAMS; i++)
  {
    vector<Mat> planes;
    split(_frames[i], planes);

    for( int y = 0; y < _frames[i].rows; y++ )
    {
      for( int x = 0; x < _frames[i].cols; x++ )
      {
        _frames_gray[i].at<uchar>(y,x) = planes[0].at<uchar>(y,x);
      }
    }
  }
#elif defined PURPLE
  for (int i = 0; i < NUMCAMS; i++)
  {
    cvtColor(_frames[i], _frames_gray[i], CV_BGR2GRAY);
  }
  
#elif defined BLACK
  for (int i = 0; i < NUMCAMS; i++)
  {
    vector<Mat> planes;
    split(_frames[i], planes);

    for( int y = 0; y < _frames[i].rows; y++ )
    {
      for( int x = 0; x < _frames[i].cols; x++ )
      {
        _frames_gray[i].at<uchar>(y,x) = planes[0].at<uchar>(y,x);
      }
    }
  }

#else

  for (int i = 0; i < NUMCAMS; i++)
  {
    cvtColor(_frames[i], _frames_gray[i], CV_BGR2GRAY);
  }
  
#endif

}

void ThreeCam::filterCanny()
{
  for (int i=0; i < NUMCAMS; i++)
  {
    _cannyFilters[i]->filter(_frames_gray[i], _cannyIms[i], _cannyAngs[i], _canny_Thresh1[i], _canny_Thresh2[i]);
  }

}




///////////////////////////////////////////////////////////////////////////////////////////
/******************************* Least squares utilities *********************************/
///////////////////////////////////////////////////////////////////////////////////////////

void ThreeCam::get3dPoint(const Point& p1, const Point& p2, const Point& p3, Point3f& point)
{
  Mat ray1(3,1,CV_32FC1);
  Mat ray2(3,1,CV_32FC1);
  Mat ray3(3,1,CV_32FC1);

  _captures[0]->getWorldRay(p1, ray1);
  _captures[1]->getWorldRay(p2, ray2);
  _captures[2]->getWorldRay(p3, ray3);

  Mat matForPoint = Mat(6,1,CV_32FC1);

  LeastSquares3Cam(ray1, ray2, ray3, _captures[0]->cameraPosition(), _captures[1]->cameraPosition(), _captures[2]->cameraPosition(), matForPoint); 

  point.x = matForPoint.at<float>(0,0);
  point.y = matForPoint.at<float>(1,0);
  point.z = matForPoint.at<float>(2,0);
}

void ThreeCam::get3dPoint(const Point* pts2d, Point3f& point)
{
  get3dPoint(pts2d[0], pts2d[1], pts2d[2], point);
}


void ThreeCam::get3dPoint(const Point2f& p1, const Point2f& p2, const Point2f& p3, Point3f& point)
{
  Mat ray1(3,1,CV_32FC1);
  Mat ray2(3,1,CV_32FC1);
  Mat ray3(3,1,CV_32FC1);

  _captures[0]->getWorldRay(p1, ray1);
  _captures[1]->getWorldRay(p2, ray2);
  _captures[2]->getWorldRay(p3, ray3);

  Mat matForPoint = Mat(6,1,CV_32FC1);

  LeastSquares3Cam(ray1, ray2, ray3, _captures[0]->cameraPosition(), _captures[1]->cameraPosition(), _captures[2]->cameraPosition(), matForPoint); 

  point.x = matForPoint.at<float>(0,0);
  point.y = matForPoint.at<float>(1,0);
  point.z = matForPoint.at<float>(2,0);
}

void ThreeCam::get3dPoint(const Point2f* pts2d, Point3f& point)
{
  get3dPoint(pts2d[0], pts2d[1], pts2d[2], point);
}


void ThreeCam::project3dPoint(const Point3f& Point3d, Point2i* points2d) 
{
  Mat mat3d = Mat(3,1,CV_32FC1);
  mat3d.at<float>(0,0) = Point3d.x;
  mat3d.at<float>(1,0) = Point3d.y;
  mat3d.at<float>(2,0) = Point3d.z;
  
  for (int i=0; i < NUMCAMS; i++)
  {
    _captures[i]->projectPointUndistorted(mat3d, points2d[i]);
  }

}


void ThreeCam::project3dPoint(const Point3f& Point3d, Point2f* points2d) 
{
  Mat mat3d = Mat(3,1,CV_32FC1);
  mat3d.at<float>(0,0) = Point3d.x;
  mat3d.at<float>(1,0) = Point3d.y;
  mat3d.at<float>(2,0) = Point3d.z;

  for (int i=0; i < NUMCAMS; i++)
  {
    _captures[i]->projectPointUndistorted(mat3d, points2d[i]);
  }

}



/////////////////////////////////////////////////////////////////////////////////////////////
/******************************* Reprojection Scoring **************************************/
/////////////////////////////////////////////////////////////////////////////////////////////

void ThreeCam::scoreCorrespondingPts(corresponding_pts& pts, bool Point3dAlreadySet)
{
  if (!Point3dAlreadySet)
    get3dPoint(pts.pts2d, pts.pt3d);
  pts.score = scoreCorrespondingPts(pts.pts2d, pts.pt3d);
}


double ThreeCam::scoreCorrespondingPts(const Point2i* pts2d, const Point3f& pt3d)
{
  //score defined as squared sum of differences between reprojecting 3d point to 2d 
  double score = 0.0;
  Mat toProject = Mat(3,1, CV_32FC1);
  toProject.at<float>(0,0) = pt3d.x;
  toProject.at<float>(1,0) = pt3d.y;
  toProject.at<float>(2,0) = pt3d.z;

  for (int i=0; i < NUMCAMS; i++)
  {
    Point2f projected;

    _captures[i]->projectPointUndistorted(toProject, projected);
    score += (double)(pow((float)pts2d[i].x - projected.x,2) + pow((float)pts2d[i].y - projected.y,2));
  }

  return score;
}




/////////////////////////////////////////////////////////////////////////////////////////////
/***************************************** Display *****************************************/
/////////////////////////////////////////////////////////////////////////////////////////////


void ThreeCam::initializeOnClicks()
{
  string names[NUMCAMS];
  char names_char[NUMCAMS][256];
  for (int camNum=0; camNum < NUMCAMS; camNum++)
  {
    sprintf(names_char[camNum], "%s%d", CLICK_DISPLAY_BASE, camNum);
    names[camNum].assign(names_char[camNum]);
  }
  stereoOnClicks = new StereoOnClicks(_captures, names);
}

void ThreeCam::getClickedPoints(Point2i* coords)
{
  stereoOnClicks->click2dPoints();
  stereoOnClicks->getClickCoords(coords);
}

void ThreeCam::getClickedPoints(Point3f& coords)
{
  stereoOnClicks->getNewPoints(&coords, 1);
  std::cout << "clicked point: \n" << coords << std::endl;
}

void ThreeCam::setImageNumber(int imNum)
{
  for (int camNum=0; camNum < NUMCAMS; camNum++)
  {
    _captures[camNum]->setImageNumber(imNum);
  }
}


void ThreeCam::saveImages(bool incremend_ind, char* image_save_base)
{
 // std::cout << "WRITING FILES" << std::endl;
  char filename[256];
  for (int camNum=0; camNum < NUMCAMS; camNum++)
  { 
    sprintf(filename, "%s%d-%d.jpg", image_save_base, (camNum+1), _saved_im_ind);
    imwrite(filename, _frames[camNum]);
    sprintf(filename, "%s%d_canny-%d.jpg", image_save_base, (camNum+1), _saved_im_ind);
    imwrite(filename, _cannyIms[camNum]);
  }
  if (incremend_ind)
    _saved_im_ind++;

}

void ThreeCam::saveImages(char* image_save_base, int im_num)
{
 // std::cout << "WRITING FILES" << std::endl;
  char filename[256];
  for (int camNum=0; camNum < NUMCAMS; camNum++)
  { 
    sprintf(filename, "%s%d-%d.jpg", image_save_base, (camNum+1), im_num);
    imwrite(filename, _frames[camNum]);
    sprintf(filename, "%s_canny%d-%d.jpg", image_save_base, (camNum+1), im_num);
    imwrite(filename, _cannyIms[camNum]);
  }

}




void ThreeCam::saveCanny()
{
  //std::cout << "WRITING CANNY FILES" << std::endl;
  char filename[256];
  for (int camNum=0; camNum < NUMCAMS; camNum++)
  { 
    sprintf(filename, "%scanny_%d-%d.png", IMAGE_SAVE_BASE, (camNum+1), _canny_saved_im_ind);
    imwrite(filename, _cannyIms[camNum]);
  }
  _canny_saved_im_ind++;
}


bool operator <(const corresponding_pts& a, const corresponding_pts& b)
{
  return a.score < b.score;
}

