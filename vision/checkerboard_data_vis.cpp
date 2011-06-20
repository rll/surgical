#include "checkerboard_data_vis.h"


Checkerboard::Checkerboard()
{
  //initCams();
  checkerSize.width = CHECKERS_PER_ROW;
  checkerSize.height = CHECKERS_PER_COL;
  for (int camNum = 0; camNum < NUMCAMS; camNum++)
  {
    checkerLocations[camNum].resize(CHECKERS_PER_ROW*CHECKERS_PER_COL);
    checkerLocationsReversed[camNum].resize(CHECKERS_PER_ROW*CHECKERS_PER_COL);
  }
  checkerLocations_3d.resize(CHECKERS_PER_ROW*CHECKERS_PER_COL);
}

Checkerboard::~Checkerboard()
{
}

void Checkerboard::initCams()
{
  _names[0] = "cam1";
  _names[1] = "cam2";
  _names[2] = "cam3";

  _captures[0] = new Capture(0, //id
      _names[0].c_str(), // cam name
      650,  // gain
      "optimized", // optimized or measured
      107109);
  //"/home/pabbeel/code/master-slave/vision2/stereo_test/stereo_test1-");
  _captures[0]->setExposure(18000);


  _captures[1] = new Capture(1, //id
      _names[1].c_str(),// cam name
      650,  // gain
      "optimized", // optimized or measured
      107110); // camera uid
  //"/home/pabbeel/code/master-slave/vision2/stereo_test/stereo_test2-");
  _captures[1]->setExposure(14000);

  _captures[2] = new Capture(2, //id
      _names[2].c_str(),// cam name
      650,  // gain
      "optimized", // optimized or measured
      107111); // camera uid
  //"/home/pabbeel/code/master-slave/vision2/stereo_test/stereo_test3-");
  _captures[2]->setExposure(14500);

  namedWindow(_names[0], CV_WINDOW_AUTOSIZE);
  namedWindow(_names[1], CV_WINDOW_AUTOSIZE);
  namedWindow(_names[2], CV_WINDOW_AUTOSIZE);
    

  /*  char names_char[NUMCAMS][256];
      for (int camNum=0; camNum < NUMCAMS; camNum++)
      {
      sprintf(names_char[camNum], "%s%d", DISPLAY_ORIG_BASE, camNum);
      _orig_display_names[camNum].assign(names_char[camNum]);
      namedWindow(_orig_display_names[camNum], CV_WINDOW_AUTOSIZE);
      }
      */


  _captures[0]->init("../vision/calib_params/");
  _captures[1]->init("../vision/calib_params/");
  _captures[2]->init("../vision/calib_params/");


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


  vector<Capture*> syncInCams;
  syncInCams.push_back(_captures[1]);
  syncInCams.push_back(_captures[2]);
  _captures[0]->syncFrameCaptureSetCenter(syncInCams);


  waitKey(1000);

}

void Checkerboard::initCamsFromFile(int slaveNum)
{
  _names[0] = "cam1";
  _names[1] = "cam2";
  _names[2] = "cam3";

  char imNames[NUMCAMS][256];
  for (int camNum=0; camNum < NUMCAMS; camNum++)
  {
    sprintf(imNames[camNum], "/home/pabbeel/rll/code/trunk/surgical/VisionRobotCalib/savedCalibIms/calibIm_slave%d_%d-", slaveNum, (camNum+1)); 
  }

  _captures[0] = new Capture(0, //id
      _names[0].c_str(), // cam name
      650,  // gain
      "optimized", // optimized or measured
      107109,
      imNames[0]);
  _captures[0]->setExposure(25000);


  _captures[1] = new Capture(1, //id
      _names[1].c_str(),// cam name
      650,  // gain
      "optimized", // optimized or measured
      107110, // camera uid
      imNames[1]);
  _captures[1]->setExposure(18000);

  _captures[2] = new Capture(2, //id
      _names[2].c_str(),// cam name
      650,  // gain
      "optimized", // optimized or measured
      107111, // camera uid
      imNames[2]);
  //"/home/pabbeel/code/master-slave/vision2/stereo_test/stereo_test3-");
  _captures[2]->setExposure(18500);

  namedWindow(_names[0], CV_WINDOW_AUTOSIZE);
  namedWindow(_names[1], CV_WINDOW_AUTOSIZE);
  namedWindow(_names[2], CV_WINDOW_AUTOSIZE);
    
  _captures[0]->init("../vision/calib_params/");
  _captures[1]->init("../vision/calib_params/");
  _captures[2]->init("../vision/calib_params/");


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


  vector<Capture*> syncInCams;
  syncInCams.push_back(_captures[1]);
  syncInCams.push_back(_captures[2]);
  _captures[0]->syncFrameCaptureSetCenter(syncInCams);


  waitKey(1000);

}



bool Checkerboard::findCheckerboard()
{
  updateIms();
  bool allCheckersFound = true;

  for (int camNum = 0; camNum < NUMCAMS; camNum++)
  {
    bool foundThis = findChessboardCorners(_frames[camNum], checkerSize, checkerLocations[camNum]);
    allCheckersFound &= foundThis;
    
    //also save reversed points
    if (foundThis)
    {
      int numPts = checkerLocations[camNum].size();
      for (int i=0; i < numPts; i++)
      {
        checkerLocationsReversed[camNum][numPts-1-i] = checkerLocations[camNum][i];
      }
      vectorToMat(checkerLocations[camNum], checkerLocations_mat[camNum]);
      vectorToMat(checkerLocationsReversed[camNum], checkerLocationsReversed_mat[camNum]);
    }

  }

  return allCheckersFound;
}

void Checkerboard::calculate3dPoints()
{
  /*
  //(actually, seems like this never happens...)
  //since we might have multiple matches, first run solvePnP to get best match, then compute stereo

  //setup checkerboard indices
  Mat objPointsBase(checkerLocations[0].size(), 3, CV_32FC1);
  for (int r=0; r < CHECKERS_PER_ROW; r++)
  {
    double curr_row_loc = r*SIZE_EACH_CHECKER;
    for (int c=0; c < CHECKERS_PER_COL; c++)
    {
      double curr_col_loc = c*SIZE_EACH_CHECKER;
      objPointsBase.at<float>(r*CHECKERS_PER_COL+c, 0) = curr_row_loc;
      objPointsBase.at<float>(r*CHECKERS_PER_COL+c, 1) = curr_col_loc;
      objPointsBase.at<float>(r*CHECKERS_PER_COL+c, 2) = 0.0;
    }
  }

  //get base (first cam) PnP points
  Mat rvec_checkerboard[NUMCAMS];
  Mat tvec_checkerboard[NUMCAMS];
  Mat dist_coeffs = Mat::zeros(5,1,CV_32FC1);

  solvePnP(objPointsBase, checkerLocations_mat[0], _captures[0]->cameraMatrix(), dist_coeffs, rvec_checkerboard[0], tvec_checkerboard[0]);

  std::cout << rvec_checkerboard[0] << "\n" << tvec_checkerboard[0] << std::endl;

  //init direction
  int directions[NUMCAMS];

  for (int camNum = 0; camNum < NUMCAMS; camNum++)
  {
    directions[camNum] = 0;
  }

  */

  Point2f pts_each_cam[NUMCAMS];
  for (int pt_ind=0; pt_ind < CHECKERS_PER_ROW*CHECKERS_PER_COL; pt_ind++)
  {
    for (int camNum=0; camNum < NUMCAMS; camNum++)
    {
      pts_each_cam[camNum] = checkerLocations[camNum][pt_ind];
    }
    
    _cams->get3dPoint(pts_each_cam, checkerLocations_3d[pt_ind]);

    //FOR DEBUG = replace checker locations with 3d reprojected
    _cams->project3dPoint(checkerLocations_3d[pt_ind], pts_each_cam);
    for (int camNum=0; camNum < NUMCAMS; camNum++)
    {
      checkerLocations[camNum][pt_ind] = pts_each_cam[camNum];
    }

  }

  
}


void Checkerboard::estimatePointsInRobotHand(const Vector3d& startPt, const Matrix3d& startRot, const Vector3d& offsetGuess, const double offsetAngGuess, vector<Vector3d>& estimatedPoints)
{
  //rotate about z axis by offsetAng, then offsetGuess will lead to point in top-left corner
  Matrix3d rotatedAboutZ = Eigen::AngleAxisd(offsetAngGuess, Vector3d::UnitZ())*startRot;
  Vector3d offsetGuessRotated = rotatedAboutZ*offsetGuess + startPt;
  Vector3d xAxisRotated = rotatedAboutZ*Vector3d::UnitX();
  Vector3d yAxisRotated = rotatedAboutZ*Vector3d::UnitY();
 // Vector3d zAxisRotated = rotatedAboutZ*Vector3d::UnitZ();

  estimatedPoints.resize(CHECKERS_PER_ROW*CHECKERS_PER_COL);
  for (int c = 0; c < CHECKERS_PER_COL; c++)
  {
    int cInd = c*CHECKERS_PER_ROW;
    for (int r = 0; r < CHECKERS_PER_ROW; r++)
    {
      estimatedPoints[cInd+r] = offsetGuessRotated + yAxisRotated*r*SIZE_EACH_CHECKER - xAxisRotated*c*SIZE_EACH_CHECKER;
    }
  } 

 

  //FOR DEBUG - replace checker points with estimated points
  Point2f pts_each_cam[NUMCAMS];
  for (int i=0; i < estimatedPoints.size(); i++)
  {
    Point3f toProj;
    toProj.x = estimatedPoints[i](0);
    toProj.y = estimatedPoints[i](1);
    toProj.z = estimatedPoints[i](2);
    _cams->project3dPoint(toProj, pts_each_cam);
    for (int camNum=0; camNum < NUMCAMS; camNum++)
    {
      checkerLocations[camNum][i] = pts_each_cam[camNum];
    }
  }

}


void Checkerboard::drawCheckerboard()
{
  for (int camNum = 0; camNum < NUMCAMS; camNum++)
  {
    drawChessboardCorners(_frames[camNum], checkerSize, Mat(checkerLocations[camNum]), true);
  }
}


void Checkerboard::drawCheckerboard(Vector3d pts3d[])
{

    Point3f ptToProj;
    Point2f pts_each_cam[NUMCAMS];
    for (int i=0; i < CHECKERS_PER_COL*CHECKERS_PER_ROW; i++)
    {
      ptToProj.x = pts3d[i](0);
      ptToProj.y = pts3d[i](1);
      ptToProj.z = pts3d[i](2);

      _cams->project3dPoint(ptToProj, pts_each_cam);
      for (int camNum=0; camNum < NUMCAMS; camNum++)
      {
        checkerLocations[camNum][i] = pts_each_cam[camNum];
      }
    }

  for (int camNum = 0; camNum < NUMCAMS; camNum++)
  {
    drawChessboardCorners(_frames[camNum], checkerSize, Mat(checkerLocations[camNum]), true);
  }
}




void Checkerboard::displayIms()
{
  _frames = _cams->frames();
  for (int camNum = 0; camNum < NUMCAMS; camNum++)
  {
    imshow(_names[camNum], _frames[camNum]);
  }

  waitKey(100);
}

void Checkerboard::vectorToMat(const vector<Point2f>& pts, Mat& mat)
{
  mat.create(pts.size(), 2, CV_32FC1);
  for (int i=0; i < pts.size(); i++)
  {
    mat.at<float>(i,0) = pts[i].x;
    mat.at<float>(i,1) = pts[i].y;
  }

}

void Checkerboard::getEstimated3dPose(Mat& rotation, Mat& translation)
{
  //(actually, seems like this never happens...)
  //since we might have multiple matches, first run solvePnP to get best match, then compute stereo

  //setup checkerboard indices
  Mat objPointsBase(checkerLocations[0].size(), 3, CV_32FC1);
  for (int c=0; c < CHECKERS_PER_COL; c++)
  {
    double curr_col_loc = c*SIZE_EACH_CHECKER;
    double curr_col_ind = c*CHECKERS_PER_ROW;
    for (int r=0; r < CHECKERS_PER_ROW; r++)
    {
      double curr_row_loc = r*SIZE_EACH_CHECKER;
      objPointsBase.at<float>(curr_col_ind+r, 0) = curr_row_loc;
      objPointsBase.at<float>(curr_col_ind+r, 1) = curr_col_loc;
      objPointsBase.at<float>(curr_col_ind+r, 2) = 0.0;
    }
  }

  //get base (first cam) PnP points
  Mat rmat_checkerboard[NUMCAMS];
  Mat rvec_checkerboard[NUMCAMS];
  Mat tvec_checkerboard[NUMCAMS];
  Mat rmat_world[NUMCAMS];
  Mat tvec_world[NUMCAMS];
  Mat dist_coeffs = Mat::zeros(5,1,CV_32FC1);

  for (int camNum = 0; camNum < NUMCAMS; camNum++)
  {
    Mat rvec(3,1,CV_32FC1);
    Mat tvec(3,1,CV_32FC1);
    tvec_checkerboard[camNum] = Mat(3,1,CV_32FC1);
    solvePnP(objPointsBase, checkerLocations_mat[camNum], _captures[camNum]->cameraMatrix(), dist_coeffs, rvec, tvec);

    rvec.convertTo(rvec_checkerboard[camNum], CV_32FC1);
    tvec.convertTo(tvec_checkerboard[camNum], CV_32FC1);


    rmat_checkerboard[camNum] = Mat(3,3,CV_32FC1);
    Rodrigues(rvec_checkerboard[camNum], rmat_checkerboard[camNum]);

    rmat_world[camNum] = Mat(3,3,CV_32FC1);
    tvec_world[camNum] = Mat(3,1,CV_32FC1);
    rmat_world[camNum] = _captures[camNum]->rot_cam2world()*rmat_checkerboard[camNum];
    tvec_world[camNum] = _captures[camNum]->rot_cam2world()*tvec_checkerboard[camNum] + _captures[camNum]->cameraPosition();


    //std::cout << rmat_world[camNum] << "\n" << tvec_world[camNum] << std::endl;

    }

  rotation = rmat_world[0];
  translation = tvec_world[0];


}

void Checkerboard::updateIms()
{
  _cams->updateImagesBlocking();
  _frames = _cams->frames();
}


void Checkerboard::updateImsNoUndistort()
{
  _cams->updateImagesBlockingNoUndistort();
  _frames = _cams->frames();
}
