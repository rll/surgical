#include "thread_vision.h"

using namespace cv;

Thread_Vision::Thread_Vision()
{
  _names[0] = "cam1";
  _names[1] = "cam2";
  _names[2] = "cam3";

  _captures[0] = new Capture(0, //id
          _names[0].c_str(), // cam name
          650,  // gain
          "optimized", // optimized or measured
          107109); // camera uid
          //"/home/pabbeel/code/master-slave/vision2/stereo_test/stereo_test1-");
          //IM_DIR_1); 
  _captures[0]->setExposure(5000);
  //_captures[0]->setExposure(11000);
  

  _captures[1] = new Capture(1, //id
          _names[1].c_str(),// cam name
          650,  // gain
          "optimized", // optimized or measured
          107110); // camera uid
          //"/home/pabbeel/code/master-slave/vision2/stereo_test/stereo_test2-");
  //        IM_DIR_2); 
  _captures[1]->setExposure(3000);
  //_captures[1]->setExposure(7500);

  _captures[2] = new Capture(2, //id
          _names[2].c_str(),// cam name
          650,  // gain
          "optimized", // optimized or measured
          107111); // camera uid
          //"/home/pabbeel/code/master-slave/vision2/stereo_test/stereo_test3-");
//          IM_DIR_3); 
  _captures[2]->setExposure(3500);
  //_captures[2]->setExposure(8000);

  /*namedWindow(_names[0], CV_WINDOW_AUTOSIZE);
  namedWindow(_names[1], CV_WINDOW_AUTOSIZE);
  namedWindow(_names[2], CV_WINDOW_AUTOSIZE);
  */

  char names_char[NUMCAMS][256];
  char canny_char[NUMCAMS][256];
  for (int camNum=0; camNum < NUMCAMS; camNum++)
  {
    sprintf(names_char[camNum], "%s%d", DISPLAY_ORIG_BASE, camNum);
    _orig_display_names[camNum].assign(names_char[camNum]);
    namedWindow(_orig_display_names[camNum], CV_WINDOW_AUTOSIZE);
    sprintf(canny_char[camNum], "%s%d", DISPLAY_CANNY_BASE, camNum);
    _canny_display_names[camNum].assign(canny_char[camNum]);
    namedWindow(_canny_display_names[camNum], CV_WINDOW_AUTOSIZE);
  }



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
  //thread
  _cams = new ThreeCam(_captures);
  float width[] = {1.50, 1.50, 1.50};
	float edge_sigma[] = {0.50, 0.50, 0.50};
	float blur_sigma[] = {1.5, 1.5, 1.5};
	double thresh1[] = {4.0, 4.0, 4.0};
	double thresh2[] = {80.0, 80.0, 80.0};

  //shoestring
/*
  float width[] = {14.00, 12.00, 13.00};
	float edge_sigma[] = {0.50, 0.50, 0.50};
	float blur_sigma[] = {7.5, 7.5, 7.5};
	double thresh1[] = {1.0, 1.0, 1.0};
	double thresh2[] = {3.0, 3.0, 3.0};
*/  

	_cams->initializeCanny(width, edge_sigma, blur_sigma, thresh1, thresh2);

  //initialize distance image
  for (int i=0; i < NUMCAMS; i++)
  {
    _frames = _cams->frames();
    rows[i] = _frames[i].rows;
    cols[i] = _frames[i].cols;
  }



  length_thread_each_piece = INIT_LENGTH_THREAD_EACH_PIECE;
  total_length = -1.0;
}

Thread_Vision::~Thread_Vision()
{
  delete _cams;
  for (int i=0; i < NUMCAMS; i++){
    _captures[i]->endCapture();
    delete _captures[i];
  }
}


void Thread_Vision::initializeThreadSearch()
{
  updateCanny();

  //FOR DEBUG
  for (int i=0; i < NUMCAMS; i++)
  {
    display_for_debug[i].resize(0);
  }

}


bool Thread_Vision::findNextStartPoint(vector<corresponding_pts>& pts, Point3f& initPt)
{
  double minDist = -0.5;
  double maxDist = 0.5;
  double addDist = 0.1;

  for (double z = initPt.z+minDist; z <= initPt.z+maxDist; z+= addDist)
  {
    for (double y = initPt.y+minDist; y <= initPt.y+maxDist; y+= addDist)
    {
      for (double x = initPt.x+minDist; x <= initPt.x+maxDist; x+= addDist)
      {
        pts.resize(pts.size()+1);
        pts.back().pt3d.x = x;
        pts.back().pt3d.y = y;
        pts.back().pt3d.z = z;
        _cams->project3dPoint(pts.back().pt3d, pts.back().pts2d);
        pts.back().score = scoreProjection3dPoint(pts.back().pt3d);
      }
    }
  }
  
  int numPtsToResize = min((int)pts.size(),NUM_START_PTS_TO_INIT);
  partial_sort(pts.begin(), pts.begin()+numPtsToResize, pts.end());

  return (pts.size() > 0);

}

bool Thread_Vision::findNextStartPoint(vector<corresponding_pts>& pts, Point2i& initPtCenterIm)
{
  const int maxDist = 15;
  int dist;
  bool done = false;

  for (dist = 0; dist < maxDist 
                && _captures[CENTER_IM_IND]->inRange(initPtCenterIm.y-dist, initPtCenterIm.x-dist)
                && _captures[CENTER_IM_IND]->inRange(initPtCenterIm.y+dist, initPtCenterIm.x-dist) 
                && _captures[CENTER_IM_IND]->inRange(initPtCenterIm.y+dist, initPtCenterIm.x+dist) 
                && _captures[CENTER_IM_IND]->inRange(initPtCenterIm.y-dist, initPtCenterIm.x+dist); dist++)
  {
    vector<Point2i> points_to_check;
    PointsForSquare(initPtCenterIm, dist, points_to_check);
    for (int ptInd = 0; ptInd < points_to_check.size(); ptInd++)
    {
      if (_cannyIms[CENTER_IM_IND].at<uchar>(points_to_check[ptInd].y, points_to_check[ptInd].x) == IM_VALUE_NOT_SEEN)
      {
        _cannyIms[CENTER_IM_IND].at<uchar>(points_to_check[ptInd].y, points_to_check[ptInd].x) = IM_VALUE_CHECKED_FOR_BEGIN;
        vector<corresponding_pts> pts_this;
        if (findCorrespondingPointsOtherIms(pts_this, points_to_check[ptInd], CENTER_IM_IND))
        {
          done = true;
          pts.insert(pts.end(), pts_this.begin(), pts_this.end());
        }
      }
    }

    if (done){
      int numPtsToResize = min((int)pts.size(),NUM_START_PTS_TO_INIT);
      partial_sort(pts.begin(), pts.begin()+numPtsToResize, pts.end());
      break;
    }

  }

  return done;

}





//TODO: speedup by intersecting epipolar lines, and searching around that region
//TODO: speedup by not even adding point to list if below thresh 
bool Thread_Vision::findCorrespondingPointsOtherIms(vector<corresponding_pts>& pts, Point2i initPt, int camWithPt)
{
  vector<Point2i> pts_on_line[NUMCAMS];
  pts_on_line[camWithPt].push_back(initPt);
  for (int i=0; i < NUMCAMS; i++)
  {
    if (i == camWithPt)
      continue;

    //get epipolar line
    Mat lineParams = Mat(3,1,CV_32FC1);

    _captures[i]->GetEpipolarLine(*_captures[camWithPt], initPt, lineParams);
    float a = lineParams.at<float>(0,0);
    float b = lineParams.at<float>(1,0);
    float c = lineParams.at<float>(2,0);

    //searches across lines offset in this direction
    int offset_each_dir = 2;
    Point p1;
    Point p2;
    bool vertical;


    if (abs(a) > abs(b))
    {
      //line is more vertical
      vertical = true;
      p1.y = 0;
      p2.y = _cannyIms[i].rows;
      p1.x = floor(0.5 + (-c)/a);
      p2.x = floor(0.5 + (-c-b*p2.y)/a);
    } else {
      //line is more horizontal
      vertical = false;
      p1.x = 0;
      p2.x = _cannyIms[i].cols;
      p1.y = floor(0.5 + (-c)/b);
      p2.y = floor(0.5 + (-c-a*p2.x)/b);
    }

    //FOR DEBUG - draw epipolar lines
    /*for (int offset_each_dir_iter = -offset_each_dir; offset_each_dir_iter <= offset_each_dir; offset_each_dir_iter++)
    {
      display_for_debug[i].resize(display_for_debug[i].size()+1);
      display_for_debug[i][display_for_debug[i].size()-1].pts = new Point[2];
      int x_off_each_dir = (vertical ? offset_each_dir_iter : 0);
      int y_off_each_dir = (vertical ? 0 : offset_each_dir_iter);
      display_for_debug[i][display_for_debug[i].size()-1].pts[0] = Point(p1.x + x_off_each_dir, p1.y + y_off_each_dir);
      display_for_debug[i][display_for_debug[i].size()-1].pts[1] = Point(p2.x + x_off_each_dir, p2.y + y_off_each_dir);
      display_for_debug[i][display_for_debug[i].size()-1].color = Scalar(0,0,255);
      display_for_debug[i][display_for_debug[i].size()-1].size = 2;
    }*/



    int offset_each_dir_multiplier = (vertical ? sizeof(uchar) : _cannyIms[i].step);

    //iterate across this line
    cv::LineIterator it(_cannyIms[i], p1, p2, 8);
    for (int iter=0; iter < it.count; iter++, ++it)
    {

      for (int offset_each_dir_iter = -offset_each_dir; offset_each_dir_iter <= offset_each_dir; offset_each_dir_iter++)
      {
        int addr_off = offset_each_dir_multiplier*offset_each_dir_iter;



        if (*(it.ptr+addr_off) != 0)
        {
          int offset = it.ptr+addr_off - (uchar*)(_cannyIms[i].data);
          Point2i matchPt;
          matchPt.y =  offset/_cannyIms[i].step;
          matchPt.x = (offset - matchPt.y*_cannyIms[i].step)/(sizeof(uchar));
         
          pts_on_line[i].push_back(matchPt);


          //FOR DEBUG - mark points
          /*display_for_debug[i].resize(display_for_debug[i].size()+1);
            display_for_debug[i][display_for_debug[i].size()-1].pts = new Point[4];
            display_for_debug[i][display_for_debug[i].size()-1].pts[0] = Point(matchPt.x-1, matchPt.y+1);
            display_for_debug[i][display_for_debug[i].size()-1].pts[1] = Point(matchPt.x+1, matchPt.y+1);
            display_for_debug[i][display_for_debug[i].size()-1].pts[2] = Point(matchPt.x+1, matchPt.y-1);
            display_for_debug[i][display_for_debug[i].size()-1].pts[3] = Point(matchPt.x-1, matchPt.y-1);
            display_for_debug[i][display_for_debug[i].size()-1].color = Scalar(0,255,0);
            display_for_debug[i][display_for_debug[i].size()-1].size = 4;
            */

        }
      }
    }
  }


  //Check to see which triplets work
  int numPts = 1;
  for (int i=0; i < NUMCAMS; i++)
  {
    numPts *= pts_on_line[i].size();
  }

  if (numPts == 0)
  {
    return false;
  }

  pts.resize(numPts);

  int currMultiplier = 1;
  for (int i=0; i < NUMCAMS; i++)
  {
    int currInd = 0;
    for (int numTimesToCopy = 0; numTimesToCopy < numPts/(currMultiplier*pts_on_line[i].size()); numTimesToCopy++)
    {
      for (int j=0; j < pts_on_line[i].size(); j++)
      {
        int currIndBeforeAdds = currInd;
        for (; currInd < currIndBeforeAdds+currMultiplier; currInd++)
        {
          pts[currInd].pts2d[i] = pts_on_line[i].at(j);
        }
      }
    }

    currMultiplier *= pts_on_line[i].size();
  }



  for (int i=0; i < numPts; i++)
  {
    _cams->scoreCorrespondingPts(pts[i], false);
  }

  int numPtsToResize = min(numPts,NUM_START_PTS_TO_INIT);
  partial_sort(pts.begin(), pts.begin()+numPtsToResize, pts.end());
  pts.resize(numPtsToResize);

  //remove those with too much error
  corresponding_pts forInd;
  forInd.score = CORRESPONDING_PTS_ERROR_THRESH;
  int indToRemove = (int)(lower_bound(pts.begin(), pts.end(), forInd) - pts.begin());
  pts.resize(indToRemove);

  if (pts.size() == 0)
    return false;



  //FOR DEBUG - mark best match
  /*int minInd = 0;
 
  for (int j = 0; j < NUMCAMS; j++)
  {
    display_for_debug[j].resize(display_for_debug[j].size()+1);
    display_for_debug[j][display_for_debug[j].size()-1].pts = new Point[4];
    display_for_debug[j][display_for_debug[j].size()-1].pts[0] = Point(pts[minInd].pts2d[j].x, pts[minInd].pts2d[j].y);
    display_for_debug[j][display_for_debug[j].size()-1].pts[1] = Point(pts[minInd].pts2d[j].x, pts[minInd].pts2d[j].y);
    display_for_debug[j][display_for_debug[j].size()-1].pts[2] = Point(pts[minInd].pts2d[j].x, pts[minInd].pts2d[j].y);
    display_for_debug[j][display_for_debug[j].size()-1].pts[3] = Point(pts[minInd].pts2d[j].x, pts[minInd].pts2d[j].y);
    display_for_debug[j][display_for_debug[j].size()-1].color = Scalar(127,0,127);
    display_for_debug[j][display_for_debug[j].size()-1].size = 4;
  }*/



  return true;

}


bool Thread_Vision::findTangent(corresponding_pts& start, vector<tangent_and_score>& tangents)
{
  const double length_for_tan = 5.0;
  const double ang_to_rotate = M_PI/40.0;

  //rotate as in yaw, pitch, roll - no roll, since we only care about direction of tangent
  vector<tangent_and_score> tan_scores;
  Point3f tanLocation;
  for (double rot1_ang = -M_PI; rot1_ang <= M_PI; rot1_ang+=(ang_to_rotate))
  {
    Matrix3d rot1(Eigen::AngleAxisd(rot1_ang, Vector3d::UnitZ()));
    Vector3d axis2 = rot1*Vector3d::UnitY();
    axis2.normalize();
    for (double rot2_ang = -M_PI/2.0; rot2_ang <= M_PI/2.0; rot2_ang +=(ang_to_rotate))
    {
      Matrix3d currRotation = Eigen::AngleAxisd(rot2_ang, axis2)*rot1;
      Vector3d currTangent = Eigen::AngleAxisd(rot2_ang, axis2)*rot1*(Vector3d::UnitX());

      currTangent.normalize();
      
      tanLocation.x = ((float)(length_for_tan*currTangent(0))) + start.pt3d.x;
      tanLocation.y = ((float)(length_for_tan*currTangent(1))) + start.pt3d.y;
      tanLocation.z = ((float)(length_for_tan*currTangent(2))) + start.pt3d.z;

      double currScore = scoreProjection3dPoint(tanLocation);

      if (currScore < TANGENT_ERROR_THRESH)
      {
        tangent_and_score toAdd(currTangent, currScore);

        toAdd.rot1 = rot1_ang;
        toAdd.rot2 = rot2_ang;
        //toAdd.trans = currRotation;

        tan_scores.push_back(toAdd);
      }

    }
  }

  if (tan_scores.size() <= 0)
    return false;



  tangents.push_back(*min_element(tan_scores.begin(), tan_scores.end()));


  //FOR DEBUG - mark best match AND Tan
  Vector3d tangent;
  tangent = tangents[0].tan;


  Point2i proj_tan[NUMCAMS]; 
  Point3f bestTanLocation(  (float)(length_for_tan*tangent(0)) + start.pt3d.x,
      (float)(length_for_tan*tangent(1)) + start.pt3d.y, 
      (float)(length_for_tan*tangent(2)) + start.pt3d.z);


  _cams->project3dPoint(bestTanLocation, proj_tan);


  for (int j = 0; j < NUMCAMS; j++)
  {
    display_for_debug[j].resize(display_for_debug[j].size()+1);
    display_for_debug[j][display_for_debug[j].size()-1].pts = new Point[8];
    display_for_debug[j][display_for_debug[j].size()-1].pts[0] = Point(start.pts2d[j].x-1, start.pts2d[j].y+1);
    display_for_debug[j][display_for_debug[j].size()-1].pts[1] = Point(start.pts2d[j].x+1, start.pts2d[j].y+1);
    display_for_debug[j][display_for_debug[j].size()-1].pts[2] = Point(start.pts2d[j].x+1, start.pts2d[j].y-1);
    display_for_debug[j][display_for_debug[j].size()-1].pts[3] = Point(start.pts2d[j].x-1, start.pts2d[j].y-1);
    display_for_debug[j][display_for_debug[j].size()-1].pts[4] = Point(proj_tan[j].x-1, proj_tan[j].y+1);
    display_for_debug[j][display_for_debug[j].size()-1].pts[5] = Point(proj_tan[j].x+1, proj_tan[j].y+1);
    display_for_debug[j][display_for_debug[j].size()-1].pts[6] = Point(proj_tan[j].x+1, proj_tan[j].y-1);
    display_for_debug[j][display_for_debug[j].size()-1].pts[7] = Point(proj_tan[j].x-1, proj_tan[j].y-1);
    display_for_debug[j][display_for_debug[j].size()-1].color = Scalar(0,255,0);
    display_for_debug[j][display_for_debug[j].size()-1].size = 8;
  }

  gl_display_for_debug->resize(gl_display_for_debug->size()+1);
  gl_display_for_debug->at(gl_display_for_debug->size()-1).vertices = new Point3f[2];
  gl_display_for_debug->at(gl_display_for_debug->size()-1).vertices[0] = start.pt3d;
  gl_display_for_debug->at(gl_display_for_debug->size()-1).vertices[1] = bestTanLocation;
  gl_display_for_debug->at(gl_display_for_debug->size()-1).size = 2;
  gl_display_for_debug->at(gl_display_for_debug->size()-1).color[0] = 1.0;
  gl_display_for_debug->at(gl_display_for_debug->size()-1).color[1] = 0.0;
  gl_display_for_debug->at(gl_display_for_debug->size()-1).color[2] = 0.0;
  


  return true;

}



double Thread_Vision::scoreProjection3dPoint(const Point3f& pt3d, double* scores)
{
  //project 3d point to images, and see how far away the projection is (in pixels) to nearest thread pixel
  if (scores == NULL)
  {
    double scoresTemp[NUMCAMS];
    scores = scoresTemp;
  }
  double score = 0.0;
  Point2f pts2d[NUMCAMS];
  _cams->project3dPoint(pt3d, pts2d);

  for (int camNum=0; camNum < NUMCAMS; camNum++)
  {
    /*double thisMinScore = DIST_FOR_SCORE_CHECK*2;
    Point2i rounded = Point2i(floor(pts2d[camNum].x + 0.5), floor(pts2d[camNum].y + 0.5));

    if (_captures[camNum]->inRange(rounded.y, rounded.x))
    {
      int key = keyForHashMap(camNum, rounded.y, rounded.x);
      if (_cannyDistanceScores[camNum].count(key) > 0)
      {
        location_and_distance* curr = &_cannyDistanceScores[camNum][key];
        while (curr != NULL)
        {
          thisMinScore = min(thisMinScore, (double)(pow(pts2d[camNum].x-(float)curr->col,2) + pow(pts2d[camNum].y-(float)curr->row,2)));
          curr = curr->next;
        }
      }
    } else {
      thisMinScore = SCORE_OUT_OF_VIEW;
    }
    scores[camNum] = thisMinScore;
    */

    scores[camNum] = score2dPoint(pts2d[camNum], camNum);
    score += scores[camNum];
  }

  return score;
}


double Thread_Vision::score2dPoint(const Point2f& pt, int camNum)
{
  double thisMinScore = DIST_FOR_SCORE_CHECK*2;
  Point2i rounded = Point2i(floor(pt.x + 0.5), floor(pt.y + 0.5));

  if (_captures[camNum]->inRange(rounded.y, rounded.x))
  {
    int key = keyForHashMap(camNum, rounded.y, rounded.x);
    if (_cannyDistanceScores[camNum].count(key) > 0)
    {
      location_and_distance* curr = &_cannyDistanceScores[camNum][key];
      while (curr != NULL)
      {
        thisMinScore = min(thisMinScore, sqrt((double)(pow(pt.x-(float)curr->col,2) + pow(pt.y-(float)curr->row,2))));
        curr = curr->next;
      }
    }
  } else {
    thisMinScore = SCORE_OUT_OF_VIEW;
  }
  return thisMinScore;
}


bool Thread_Vision::isEndPiece(const Point3f pt)
{
  Point2i pts2d[NUMCAMS];
  _cams->project3dPoint(pt, pts2d);

  int numContinue = NUMCAMS;

  for (int camNum=0; camNum< NUMCAMS; camNum++)
  {
    if (isEndPiece(camNum, pts2d[camNum]))
      numContinue--;
//      return true;
  }

  //return false;
  return numContinue < 2;
}


bool Thread_Vision::isEndPiece(const int camNum, const Point2i pt)
{
  //check dist_for_thread_check pixels away (in a square) for thread pixels
  //for all we find, see if there are other pixels nearby
  //if we find 1 grouping only, it is an end piece

  const int dist_for_thread_check = 8;
  const double dist_limit = sqrt(2);

  if (!_captures[camNum]->inRange(pt.y-dist_for_thread_check, pt.x-dist_for_thread_check) ||
        !_captures[camNum]->inRange(pt.y+dist_for_thread_check, pt.x-dist_for_thread_check) ||
        !_captures[camNum]->inRange(pt.y+dist_for_thread_check, pt.x+dist_for_thread_check) ||
        !_captures[camNum]->inRange(pt.y-dist_for_thread_check, pt.x+dist_for_thread_check))
    return true;
  
  vector<Point2i> points_with_thread;
  vector<Point2i> points_to_check;
  PointsForSquare(pt, dist_for_thread_check, points_to_check);
  for (int i=0; i < points_to_check.size(); i++)
  {
    if (_cannyIms[camNum].at<uchar>(points_to_check[i]) > IM_VALUE_USED_IN_THREAD)
    {
      if (points_with_thread.size() == 0)
      {
        points_with_thread.push_back(points_to_check[i]);
      } else {
        bool canAttach = false;
        for (int j=0; j < points_with_thread.size(); j++)
        {
          if (norm(points_with_thread[j] - points_to_check[i]) <= dist_limit)
          {
            canAttach = true;
            points_with_thread.push_back(points_to_check[i]);
            break;
          }
        }

        //if we weren't able to group it with something we saw...this is a new piece!
        if (!canAttach)
          return false;
      }
    }
  }
  return true;



}


void Thread_Vision::setNextPointers(vector<ThreadPiece_Vision*> pieces)
{
  for (int i=0; i < pieces.size()-1; i++)
  {
    pieces[i]->_next_segment = pieces[i+1];
  }
}


void Thread_Vision::getPoints(vector<Vector3d>& points)
{
  if (threadPiecesCurr[0]->_next_segment == NULL)
    setNextPointers(threadPiecesCurr);

  Matrix4d TransBefore;
  threadPiecesCurr[0]->getTransformBefore(TransBefore);
	threadPiecesCurr[0]->getPoints(points, 0.0, total_length/((double)(points.size()-1)), 0, TransBefore); 
}

void Thread_Vision::getPoints(MatrixXd& points)
{
  double length = 0.0;
  ThreadPiece* currPiece = threadPiecesCurr[0];

  if (threadPiecesCurr[0]->_next_segment == NULL)
    setNextPointers(threadPiecesCurr);



  Matrix4d TransBefore;
  threadPiecesCurr[0]->getTransformBefore(TransBefore);
	threadPiecesCurr[0]->getPoints(points, 0.0, total_length/((double)(points.rows()-1)), 0, TransBefore); 
}



Thread* Thread_Vision::equivalentThreadMinEnergy()
{
  Matrix4d TransBefore;
  ThreadPiece* front = new ThreadPiece(threadPiecesCurr[0]->_curvature*total_length, threadPiecesCurr[0]->_torsion*total_length, threadPiecesCurr[0]->_length/total_length);
  ThreadPiece* currPiece = front;
  for (int pieceNum = 1; pieceNum < threadPiecesCurr.size(); pieceNum++)
  {
    currPiece->addSegmentAfter(new ThreadPiece(threadPiecesCurr[pieceNum]->_curvature*total_length, threadPiecesCurr[pieceNum]->_torsion*total_length, threadPiecesCurr[pieceNum]->_length/total_length));
    currPiece = currPiece->_next_segment;
  }


  threadPiecesCurr[0]->getTransformBefore(TransBefore);

  return new Thread(front, TransBefore, total_length);
  
  
/*  Vector3d positions[2];
  Vector3d tangents[2];

  threadPiecesCurr.front()->getFirstPoint(positions[0]);
  threadPiecesCurr.back()->getLastPoint(positions[1]);
  threadPiecesCurr.front()->getFirstTan(tangents[0]);
  threadPiecesCurr.back()->getLastTan(tangents[1]);


  MatrixXd points = points_with_thread(

  Thread* toRtn = new Thread(total_length, positions, tangents);
*/

}


void Thread_Vision::updateCanny()
{
  //FOR DEBUG - don't undistort
  _cams->updateImagesBlocking();
  _cams->convertToGrayscale();
  _frames = _cams->frames();
  _cams->filterCanny();
  _cannyIms = _cams->cannyIms();
  //gray_to_canny();
  precomputeDistanceScores();
}


void Thread_Vision::gray_to_canny()
{
  _cannyIms = _cams->frames_gray();
  for (int camNum=0; camNum < NUMCAMS; camNum++)
  {
    for (int row=0; row < _cannyIms[camNum].rows; row++)
    {
      unsigned char* ptr = _cannyIms[camNum].ptr<unsigned char>(row);
      for (int col=0; col < _cannyIms[camNum].cols; col++)
      {
        if (ptr[col] >= 127)
        {
          ptr[col] = 255;
        }
      }

    }
  }

}


void Thread_Vision::precomputeDistanceScores()
{
  for (int camNum=0; camNum < NUMCAMS; camNum++)
  {
    _cannyDistanceScores[camNum].clear();
    queue<location_and_distance_for_queue> loc_and_dist_to_add;

    for( int y = 0; y < _cannyIms[camNum].rows; y++)
    {
      uchar* Uptr = _cannyIms[camNum].ptr<uchar>(y);
      for( int x = 0; x < _cannyIms[camNum].cols; x++ )
      {
        if (Uptr[x] == (uchar)255)
        {
          location_and_distance toAdd(y,x,0.0);
          _cannyDistanceScores[camNum][keyForHashMap(camNum, y, x)] = toAdd;


          //add to queue
          for (int xadd=-1; xadd <= 1; xadd++)
          {
            int x_next = x+xadd;
            if (x_next >= 0 && x_next < cols[camNum])
            {
              for (int yadd=-1; yadd <= 1; yadd++)
              {
                int y_next = y+yadd;
                if (y_next >= 0 && y_next < rows[camNum])
                {
                  if (!(xadd == 0 && yadd == 0))
                  {
                    location_and_distance forQueue(y, x, (xadd*xadd + yadd*yadd));
                    location_and_distance_for_queue toAddToQueue(y_next,x_next,forQueue);
                    loc_and_dist_to_add.push(toAddToQueue);
                  }
                }
              }
            }
          }


        }
      }
    }


    //process queue
    while (!loc_and_dist_to_add.empty())
    {
      location_and_distance_for_queue fromQueue = loc_and_dist_to_add.front();
      location_and_distance ldFromQueue = fromQueue.ld;
      loc_and_dist_to_add.pop();

      int key = keyForHashMap(camNum, fromQueue.rowCheck, fromQueue.colCheck);
      //if we need to replace/add another thread distance
      bool toProcess = false;

      if (_cannyDistanceScores[camNum].count(key) == 0 || ldFromQueue.dist < _cannyDistanceScores[camNum][key].dist)
      {
        _cannyDistanceScores[camNum][key] = ldFromQueue;
        toProcess=true;
      } else if (ldFromQueue.dist == _cannyDistanceScores[camNum][key].dist) {
        location_and_distance* curr = &(_cannyDistanceScores[camNum][key]);
        toProcess = true;
        while (curr->next != NULL)
        {
          if (curr->row == ldFromQueue.row && curr->col == ldFromQueue.col)
          {
            toProcess = false;
            break;
          }
          curr = curr->next;
        }
        if (toProcess)
        {
          curr->next = new location_and_distance(ldFromQueue.row, ldFromQueue.col, ldFromQueue.dist);
        }
      }


      if (toProcess)
      {
        //add to queue
        for (int xadd=-1; xadd <= 1; xadd++)
        {
          int x_next = fromQueue.colCheck+xadd;
          if (x_next >= 0 && x_next < cols[camNum])
          {
            for (int yadd=-1; yadd <= 1; yadd++)
            {
              int y_next = fromQueue.rowCheck+yadd;
              if (y_next >= 0 && y_next < rows[camNum])
              {
                int nextDist = (pow(y_next-ldFromQueue.row,2) + pow(x_next-ldFromQueue.col,2));
                if (!(xadd == 0 && yadd == 0) && (nextDist <= DIST_FOR_SCORE_CHECK))
                {
                  location_and_distance forQueue(ldFromQueue.row, ldFromQueue.col, nextDist);
                  location_and_distance_for_queue toAddToQueue(y_next,x_next,forQueue);
                  loc_and_dist_to_add.push(toAddToQueue);
                }
              }
            }
          }
        }


      }

    }

  }

}



void Thread_Vision::initializeOnClicks()
{
  _cams->initializeOnClicks();
}

void Thread_Vision::clickOnPoints(Point2i* clickPoints)
{
  _cams->getClickedPoints(clickPoints);
}


void Thread_Vision::clickOnPoints(Point3f& clickPoints)
{
  _cams->getClickedPoints(clickPoints);
}










void Thread_Vision::addThreadPointsToDebug()
{
  //FOR DEBUG - mark the point of best thread
  if ( threadPiecesCurr.size() > 0 )
  {
    Point3f currPoint;
    Point2i imPoints[NUMCAMS];
    int numPieces = threadPiecesCurr.back()->_numPieces;
    std::cout << "num Pieces final: " << numPieces << std::endl;
    for (int j = 0; j < NUMCAMS; j++)
    {
      display_for_debug[j].resize(display_for_debug[j].size()+1);
      display_for_debug[j][display_for_debug[j].size()-1].pts = new Point[(numPieces+1)*4];
      display_for_debug[j][display_for_debug[j].size()-1].size = (numPieces+1)*4;
      display_for_debug[j][display_for_debug[j].size()-1].color = Scalar(127,0,127);
    }
    gl_display_for_debug->resize(gl_display_for_debug->size()+1);
    gl_display_for_debug->at(gl_display_for_debug->size()-1).vertices = new Point3f[numPieces+1];
    gl_display_for_debug->at(gl_display_for_debug->size()-1).size = numPieces+1;
    gl_display_for_debug->at(gl_display_for_debug->size()-1).color[0] = 0.7;
    gl_display_for_debug->at(gl_display_for_debug->size()-1).color[1] = 0.0;
    gl_display_for_debug->at(gl_display_for_debug->size()-1).color[2] = 0.7;
 

  std::cout << "length: " << total_length << std::endl;
//  std::cout << "numPieces: " << numPieces << std::endl;


    MatrixXd resampled_points(numPieces+1,3);
    Matrix4d first_trans;
    threadPiecesCurr[0]->getTransformBefore(first_trans);
    threadPiecesCurr[0]->getPoints(resampled_points, 0.0, total_length/((double)numPieces), 0, first_trans);    
   
 /*   for (int ind = 0; ind < threadPiecesCurr.size(); ind++)
    {
      std::cout << "curve params: " << threadPiecesCurr[ind]->_curvature << "  " << threadPiecesCurr[ind]->_torsion << std::endl;
    }*/


    for (int ind=0; ind < numPieces+1; ind++)
    {
      currPoint.x = (float)resampled_points(ind,0);
      currPoint.y = (float)resampled_points(ind,1);
      currPoint.z = (float)resampled_points(ind,2);
      _cams->project3dPoint(currPoint,imPoints);

  //    std::cout << "curr pos: " << currPoint << std::endl;
      //std::cout << "curr curve params: " << curr->_curvature << "  " << curr->_torsion << std::endl;     


      for (int j = 0; j < NUMCAMS; j++)
      {
        display_for_debug[j][display_for_debug[j].size()-1].pts[4*ind  ] = Point(imPoints[j].x-1, imPoints[j].y-1);
        display_for_debug[j][display_for_debug[j].size()-1].pts[4*ind+1] = Point(imPoints[j].x-1, imPoints[j].y+1);
        display_for_debug[j][display_for_debug[j].size()-1].pts[4*ind+2] = Point(imPoints[j].x+1, imPoints[j].y+1);
        display_for_debug[j][display_for_debug[j].size()-1].pts[4*ind+3] = Point(imPoints[j].x+1, imPoints[j].y-1);
      }
      gl_display_for_debug->at(gl_display_for_debug->size()-1).vertices[ind] = currPoint;
    }


    Matrix4d firstTransform;
    threadPiecesCurr.back()->getTransformBefore(firstTransform);
    //std::cout << "tangent: \n" << firstTransform.corner(Eigen::TopLeft,3,1) << std::endl; 
  }

}


void Thread_Vision::addThreadPointsToDebugImages(MatrixXd& points, Scalar& color)
{
  Point3f currPoint;
  Point2i imPoints[NUMCAMS];
  int numPoints = points.rows();

  for (int j = 0; j < NUMCAMS; j++)
  {
    display_for_debug[j].resize(display_for_debug[j].size()+1);
    display_for_debug[j][display_for_debug[j].size()-1].pts = new Point[(numPoints)*4];
    display_for_debug[j][display_for_debug[j].size()-1].size = (numPoints)*4;
    display_for_debug[j][display_for_debug[j].size()-1].color = color;
  }



  for (int ind=0; ind < numPoints; ind++)
  {
    currPoint.x = (float)points(ind,0);
    currPoint.y = (float)points(ind,1);
    currPoint.z = (float)points(ind,2);
    _cams->project3dPoint(currPoint,imPoints);

    //std::cout << "curr pos: " << currPoint << std::endl;
    //std::cout << "curr curve params: " << curr->_curvature << "  " << curr->_torsion << std::endl;     


    for (int j = 0; j < NUMCAMS; j++)
    {
      display_for_debug[j][display_for_debug[j].size()-1].pts[4*ind  ] = Point(imPoints[j].x-1, imPoints[j].y-1);
      display_for_debug[j][display_for_debug[j].size()-1].pts[4*ind+1] = Point(imPoints[j].x-1, imPoints[j].y+1);
      display_for_debug[j][display_for_debug[j].size()-1].pts[4*ind+2] = Point(imPoints[j].x+1, imPoints[j].y+1);
      display_for_debug[j][display_for_debug[j].size()-1].pts[4*ind+3] = Point(imPoints[j].x+1, imPoints[j].y-1);
    }
  }

}




void Thread_Vision::display()
{

  //FOR DEBUG - actually display
  
  for (int i=0; i < NUMCAMS; i++)
  {
    for (int disp_ind = 0; disp_ind < display_for_debug[i].size(); disp_ind++)
    {
      const Point* pt[] = {display_for_debug[i][disp_ind].pts};

      cv::polylines(_frames[i], pt, &display_for_debug[i][disp_ind].size, 1, false, display_for_debug[i][disp_ind].color, 2);

    }
    




    cv::imshow(_orig_display_names[i], _frames[i]); 
  }


 /* for (int i=0; i < NUMCAMS; i++)
  {
    for (int r = 100; r <  rows[i]-100; r++)
    {


      for (int c = 100; c < cols[i]-100; c++)
      {
        //std::cout << c << std::endl;
        Point2f toScore((float)c+randomMaxAbsValue(0.5), (float)r+randomMaxAbsValue(0.5));
        //double scoreThis = pow(toScore.x-(float)curr->col,2) + pow(toScore.y-(float)curr->row,2);
        double scoreThis = score2dPoint(toScore, i);
        
        _cannyIms[i].at<uchar>(r,c) = max(255 - 2*(int)scoreThis, 0);
       

      }
    }
  }*/



  _cams->saveImages(false);

  for (int i=0; i < NUMCAMS; i++)
  {
    cvtColor(_cannyIms[i], _frames[i], CV_GRAY2BGR);
    for (int disp_ind = 0; disp_ind < display_for_debug[i].size(); disp_ind++)
    {
      const Point* pt[] = {display_for_debug[i][disp_ind].pts};

      cv::polylines(_frames[i], pt, &display_for_debug[i][disp_ind].size, 1, false, display_for_debug[i][disp_ind].color, 2);

    }
    cv::imshow(_canny_display_names[i], _frames[i]); 

  }
 
  char cannyName[256];
  sprintf(cannyName, "%scanny_", IMAGE_SAVE_BASE);

  _cams->saveImages(true, cannyName);


  waitKey(1000);

}



















bool operator <(const tangent_and_score& a, const tangent_and_score& b)
{
  return a.score < b.score;
}


location_and_distance::~location_and_distance()
{
  if (next != NULL)
  {
    delete next;
    next = NULL;
  }

}
 
