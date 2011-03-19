#include "ThreadStereo_SpaceCurve_OptimizePoints.h"
#include "GenSet.h"
#include "OptGSS.h"
#include "NLF.h"

using namespace cv;

ThreadStereo_SpaceCurve::ThreadStereo_SpaceCurve()
{
  _names[0] = "cam1";
  _names[1] = "cam2";
  _names[2] = "cam3";

  _captures[0] = new Capture(0, //id
          _names[0].c_str(), // cam name
          650,  // gain
          "optimized", // optimized or measured
          107109,
          "/home/pabbeel/code/master-slave/vision2/stereo_test/stereo_test1-");
          //"./_captures/extrins-4-293-"); // camera uid

  _captures[1] = new Capture(1, //id
          _names[1].c_str(),// cam name
          650,  // gain
          "optimized", // optimized or measured
          107110,
          "/home/pabbeel/code/master-slave/vision2/stereo_test/stereo_test2-");
          //"./_captures/extrins-4-293-"); // camera uid

  _captures[2] = new Capture(2, //id
          _names[2].c_str(),// cam name
          650,  // gain
          "optimized", // optimized or measured
          107111,
          "/home/pabbeel/code/master-slave/vision2/stereo_test/stereo_test3-");
          //"./_captures/extrins-4-293-"); // camera uid

  namedWindow(_names[0], CV_WINDOW_AUTOSIZE);
  namedWindow(_names[1], CV_WINDOW_AUTOSIZE);
  namedWindow(_names[2], CV_WINDOW_AUTOSIZE);

  /*namedWindow("1 orig", CV_WINDOW_AUTOSIZE);
  namedWindow("2 orig", CV_WINDOW_AUTOSIZE);
  namedWindow("3 orig", CV_WINDOW_AUTOSIZE);
  */

  _captures[0]->init("./calib-apr21/");
  _captures[1]->init("./calib-apr21/");
  _captures[2]->init("./calib-apr21/");
  /*_captures[0]->init("./calib_params/");
  _captures[1]->init("./calib_params/");
  _captures[2]->init("./calib_params/");
  */
  
  cvWaitKey(500);							// segfaulted without this
 
	
  _captures[0]->AddOtherCameraInformation(*_captures[1]);
  _captures[0]->AddOtherCameraInformation(*_captures[2]);
  _captures[1]->AddOtherCameraInformation(*_captures[0]);
  _captures[1]->AddOtherCameraInformation(*_captures[2]);
  _captures[2]->AddOtherCameraInformation(*_captures[0]);
  _captures[2]->AddOtherCameraInformation(*_captures[1]);



  _cams = new ThreeCam(_captures);
  float width[] = {1.50, 1.50, 1.55};
	float edge_sigma[] = {0.5, 0.5, 0.6};
	float blur_sigma[] = {1.0, 1.0, 1.0};
	double thresh1[] = {5.0, 5.0, 5.0};
	double thresh2[] = {200.0, 200.0, 200.0};
	_cams->initializeCanny(width, edge_sigma, blur_sigma, thresh1, thresh2);

  //initialize distance image
  for (int i=0; i < NUMCAMS; i++)
  {
    _frames = _cams->frames();
    rows[i] = _frames[i].rows;
    cols[i] = _frames[i].cols;


  }



}

ThreadStereo_SpaceCurve::~ThreadStereo_SpaceCurve()
{
  delete _cams;
  for (int i=0; i < NUMCAMS; i++){
    delete _captures[i];
  }
}



void ThreadStereo_SpaceCurve::initializeThreadSearch()
{
  _begin_ptr_curr_row = 0;
  _begin_ptr_curr_col = 0;
  currHypoths = new vector<ThreadPiece_Vision*>();


  //FOR DEBUG
  for (int i=0; i < NUMCAMS; i++)
  {
    display_for_debug[i].resize(0);
  }


}



void ThreadStereo_SpaceCurve::getThreadPoints(vector<glline_draw_params>& gl_draw_params)
{
  gl_draw_params.resize(0);
  gl_display_for_debug = &gl_draw_params;
  waitKey(100);
  Scalar redColor = Scalar(0, 0, 255);
  updateCanny();

/*
  for (int i=0; i < NUMCAMS; i++)
    cv::imshow(_names[i], _cannyIms[i]);   
  
  cv::imshow("1 orig", _frames[0]);   
  cv::imshow("2 orig", _frames[1]);   
  cv::imshow("3 orig", _frames[2]);   


  waitKey(100000);
  return;
*/


  initializeThreadSearch();


  bool done = false;
  int numCheck = 0;
  int numdisp = 0;

  vector<corresponding_pts> start_pts;
  vector<tangent_and_score> tangent_to_start;
  vector<vector<tangent_and_score> > tangent_to_start_other_dir;

  while(findNextStartPoint(start_pts))
  {
    currHypoths->resize(0);
    tangent_to_start_other_dir.resize(0);
    for (int startInd=0; startInd < start_pts.size(); startInd++)
    {
      tangent_to_start.resize(0);
      vector<tangent_and_score> toPush;
      tangent_to_start_other_dir.push_back(toPush);
      if (findTangent(start_pts[startInd], tangent_to_start, tangent_to_start_other_dir.at(startInd)))
      { 
        for (int tanInd=0; tanInd < tangent_to_start.size(); tanInd++)
        {
          addStartPointsWithRotation(start_pts[startInd], tangent_to_start[tanInd]);
        }
      }

    }

    //first, process forward tans
    processHypothesesFromInit();

    //now, process backwards
    currHypoths->resize(0);
    for (int startInd=0; startInd < start_pts.size(); startInd++)
    {
        for (int tanInd=0; tanInd < tangent_to_start_other_dir.at(startInd).size(); tanInd++)
        {
          addStartPointsWithRotation(start_pts[startInd], tangent_to_start_other_dir.at(startInd).at(tanInd));
        }
    }
    processHypothesesFromInit();
    
  }

  /* Matrix4d one_trans;
  Matrix4d two_trans;
  getTransform(0.034*200,0.000444*200.0,0.0625,one_trans);
  getTransform(0.02,0.000000001,5.0,two_trans);
  two_trans *= 5.0;


  std::cout << "one: \n" << one_trans << std::endl;
  std::cout << "two: \n" << two_trans << std::endl;
  */



//FOR DEBUG - display the points
  for (int i=0; i < NUMCAMS; i++)
  {
  cvtColor(_cannyIms[i], _frames[i], CV_GRAY2BGR);
    for (int disp_ind = 0; disp_ind < display_for_debug[i].size(); disp_ind++)
    {
      const Point* pt[] = {display_for_debug[i][disp_ind].pts};

      cv::polylines(_frames[i], pt, &display_for_debug[i][disp_ind].size, 1, false, display_for_debug[i][disp_ind].color, 2);

    }
/*
    for (int r=0; r < rows[i]; r++)
    {
      for (int c=0; c < cols[i]; c++)
      {
        if (_cannyIms[i].at<uchar>(r,c) >= 127)
        {
          _frames[i].at<Vec3b>(r,c)[0] = (unsigned char)255;
          _frames[i].at<Vec3b>(r,c)[1] = (unsigned char)255;
          _frames[i].at<Vec3b>(r,c)[2] = (unsigned char)255;
        }
      }
    }
*/


    cv::imshow(_names[i], _frames[i]); 

  }

  waitKey(100);
  delete currHypoths;


}



void ThreadStereo_SpaceCurve::processHypothesesFromInit()
{
  vector<ThreadPiece_Vision*>* nextHypoths = new vector<ThreadPiece_Vision*>();
  while (updateHypoths(nextHypoths))
  {

    /*Point3f endPoint;
    currHypoths->at(0)->getFirstPoint(endPoint);
    std::cout << "point: " << endPoint << std::endl;*/


    if (currHypoths->at(0)->_numPieces > 60)
      break;
    vector<ThreadPiece_Vision*>::iterator it;
    for (it=currHypoths->begin() ; it < currHypoths->end(); it++)
    {
      if ( (*it)->_numPointingToMe != 0)
      {
        (*it)=NULL;
      }
    }
    delete currHypoths;

    currHypoths = nextHypoths;

    nextHypoths = new vector<ThreadPiece_Vision*>();
  }

  //optimize
  if (currHypoths->size() > 0)
  {
    ThreadPiece_Vision* end = currHypoths->at(0);
    optimizeCurveFromPoints(end);
  }

  //FOR DEBUG - mark the point of best thread
  if (currHypoths->size() > 0)
  {
    ThreadPiece_Vision* curr = currHypoths->at(0);
    Point3f currPoint;
    Point2i imPoints[NUMCAMS];
    int numPieces = currHypoths->at(0)->_numPieces;
    std::cout << "num Pieces final: " << numPieces << std::endl;
    removeUsedDistanceScores(curr);
    for (int j = 0; j < NUMCAMS; j++)
    {
      display_for_debug[j].resize(display_for_debug[j].size()+1);
      display_for_debug[j][display_for_debug[j].size()-1].pts = new Point[(1+numPieces)*4];
      display_for_debug[j][display_for_debug[j].size()-1].size = (1+numPieces)*4;
      display_for_debug[j][display_for_debug[j].size()-1].color = Scalar(127,0,127);
    }
    gl_display_for_debug->resize(gl_display_for_debug->size()+1);
    gl_display_for_debug->at(gl_display_for_debug->size()-1).vertices = new Point3f[numPieces+1];
    gl_display_for_debug->at(gl_display_for_debug->size()-1).size = numPieces+1;
    gl_display_for_debug->at(gl_display_for_debug->size()-1).color[0] = 0.7;
    gl_display_for_debug->at(gl_display_for_debug->size()-1).color[1] = 0.0;
    gl_display_for_debug->at(gl_display_for_debug->size()-1).color[2] = 0.7;


    int ind=0;

    while (curr != NULL)
    {
      curr->getLastPoint(currPoint);
      _cams->project3dPoint(currPoint,imPoints);


      std::cout << "curr pos: " << currPoint << std::endl;
      std::cout << "curr curve params: " << curr->_curvature << "  " << curr->_torsion << std::endl;      //so thread pieces that shouldn't be deleted won't be


      for (int j = 0; j < NUMCAMS; j++)
      {
        display_for_debug[j][display_for_debug[j].size()-1].pts[4*ind  ] = Point(imPoints[j].x-1, imPoints[j].y-1);
        display_for_debug[j][display_for_debug[j].size()-1].pts[4*ind+1] = Point(imPoints[j].x-1, imPoints[j].y+1);
        display_for_debug[j][display_for_debug[j].size()-1].pts[4*ind+2] = Point(imPoints[j].x+1, imPoints[j].y+1);
        display_for_debug[j][display_for_debug[j].size()-1].pts[4*ind+3] = Point(imPoints[j].x+1, imPoints[j].y-1);
      }
      gl_display_for_debug->at(gl_display_for_debug->size()-1).vertices[ind] = currPoint;
      ind++;

      curr = (ThreadPiece_Vision*)(curr->_last_segment);
    }

  }


}






bool ThreadStereo_SpaceCurve::findNextStartPoint(vector<corresponding_pts>& pts)
{
  bool found_first_pt = false;
  for (; _begin_ptr_curr_row < _cannyIms[CENTER_IM_IND].rows; _begin_ptr_curr_row++)
  {
    unsigned char* _canny_ptr = _cannyIms[CENTER_IM_IND].ptr<unsigned char>(_begin_ptr_curr_row);
    for (; _begin_ptr_curr_col < _cannyIms[CENTER_IM_IND].cols; _begin_ptr_curr_col++)
    {
      if (_canny_ptr[_begin_ptr_curr_col] >= IM_VALUE_NOT_SEEN)
      {
        _canny_ptr[_begin_ptr_curr_col] = IM_VALUE_CHECKED_FOR_BEGIN;
        Point2i initPt(_begin_ptr_curr_col, _begin_ptr_curr_row);

        //FOR DEBUG - mark the start point
        /*display_for_debug[CENTER_IM_IND].resize(1);
        display_for_debug[CENTER_IM_IND][0].pts = new Point[4];
        display_for_debug[CENTER_IM_IND][0].pts[0] = Point(initPt.x-1, initPt.y+1);
        display_for_debug[CENTER_IM_IND][0].pts[1] = Point(initPt.x+1, initPt.y+1);
        display_for_debug[CENTER_IM_IND][0].pts[2] = Point(initPt.x+1, initPt.y-1);
        display_for_debug[CENTER_IM_IND][0].pts[3] = Point(initPt.x-1, initPt.y-1);
        display_for_debug[CENTER_IM_IND][0].color = Scalar(0,0,255);
        display_for_debug[CENTER_IM_IND][0].size = 4;
        */

        
        if (findCorrespondingPointsOtherIms(pts, initPt, CENTER_IM_IND))
        {
          //we found one point! 
          found_first_pt = true;
          break;
        }

      }
    }
    
    if (found_first_pt)
      break;

    _begin_ptr_curr_col = 0;
  }


  //no more points to find
  if (!found_first_pt)
    return false;

  //first point is found
  return true;
}






//TODO: speedup by intersecting epipolar lines, and searching around that region
//TODO: speedup by not even adding point to list if below thresh 
bool ThreadStereo_SpaceCurve::findCorrespondingPointsOtherIms(vector<corresponding_pts>& pts, Point2i initPt, int camWithPt)
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


bool ThreadStereo_SpaceCurve::findTangent(corresponding_pts& start, vector<tangent_and_score>& tangents, vector<tangent_and_score>& opposite_tangents)
{
  const double length_for_tan = 5.0;
  const double ang_to_rotate = M_PI/40.0;

  //rotate as in yaw, pitch, roll - except no role, since we only care about direction of tangent
  vector<tangent_and_score> tan_scores;
  Point3f tanLocation;
  for (double rot1_ang = -M_PI; rot1_ang <= M_PI; rot1_ang+=(ang_to_rotate))
  {
    Matrix3d rot1(Eigen::AngleAxisd(rot1_ang, Vector3d::UnitZ()));
    Vector3d axis2 = rot1*Vector3d::UnitY();
    for (double rot2_ang = -M_PI; rot2_ang <= M_PI; rot2_ang +=(ang_to_rotate))
    {
      Matrix3d currRotation = rot1*Eigen::AngleAxisd(rot2_ang, axis2);
      Vector3d currTangent = currRotation*(Vector3d::UnitX());

      currTangent.normalize();

      
      tanLocation.x = ((float)(length_for_tan*currTangent(0))) + start.pt3d.x;
      tanLocation.y = ((float)(length_for_tan*currTangent(1))) + start.pt3d.y;
      tanLocation.z = ((float)(length_for_tan*currTangent(2))) + start.pt3d.z;
      //std::cout << "curr location " << tanLocation << std::endl;

      double currScore = scoreProjection3dPoint(tanLocation);

      if (currScore < TANGENT_ERROR_THRESH)
      {
        tangent_and_score toAdd = tangent_and_score(currTangent, currScore);
        toAdd.transform_to_tan.corner(Eigen::TopLeft,3,3) = currRotation;
        //toAdd.transform_to_tan.corner(Eigen::TopLeft,3,3) = rot1*Eigen::AngleAxisd(-rot2_ang, axis2);
        toAdd.transform_to_tan(0,3) = (double)start.pt3d.x;
        toAdd.transform_to_tan(1,3) = (double)start.pt3d.y;
        toAdd.transform_to_tan(2,3) = (double)start.pt3d.z;
        toAdd.transform_to_tan(3,3) = 1.0;
        toAdd.transform_to_tan(3,0) = toAdd.transform_to_tan(3,1) = toAdd.transform_to_tan(3,2) = 0.0;
        toAdd.rot1 = rot1_ang;
        toAdd.rot2 = rot2_ang;

        tan_scores.push_back(toAdd);
      }

    }
  }

  if (tan_scores.size() <= 0)
    return false;

  sort(tan_scores.begin(), tan_scores.end());

  tangents.push_back(tan_scores.at(0));

  for (int i = 1; i < min((int)NUM_TANS_TO_INIT*4, (int)tan_scores.size()); i++)
  {
    if (tangents.size() >= NUM_TANS_TO_INIT && opposite_tangents.size() >= NUM_TANS_TO_INIT)
      break;

    if (tangents.size() < NUM_TANS_TO_INIT && tangents.at(0).tan.dot(tan_scores.at(i).tan) > 0.5)
    {
      tangents.push_back(tan_scores.at(i));
    } else if (opposite_tangents.size() == 0 ||
        (opposite_tangents.size() < NUM_TANS_TO_INIT && opposite_tangents.at(0).tan.dot(tan_scores.at(i).tan) > 0.5))
    {
      opposite_tangents.push_back(tan_scores.at(i));
    }
  }


  /*
  tangent_and_score tangent_to_avg;
  int num_tangent_to_avg = 0;
  tangent_and_score opposite_tangent_to_avg;
  int opposite_num_tangent_to_avg = 0;

  for (int i = 1; i < NUM_TANS_TO_INIT*4; i++)
  {
    if (num_tangent_to_avg >= NUM_TANS_TO_INIT && opposite_num_tangent_to_avg >= NUM_TANS_TO_INIT)
      break;

    if (num_tangent_to_avg < NUM_TANS_TO_INIT && tangent_to_avg.tan.dot(tan_scores.at(i).tan) > 0.3)
    {
      tangent_to_avg.tan = (tangent_to_avg.tan*tangent_to_avg.score + tan_scores.at(i).tan*tan_scores.at(i).score);
      tangent_to_avg.score = tangent_to_avg.score + tan_scores.at(i).score;
      //tangent_to_avg.tan /= tangent_to_avg.score;
      tangent_to_avg.tan.normalize();
      num_tangent_to_avg++;
    } else
    {
      if (opposite_num_tangent_to_avg == 0)
      {
        opposite_tangent_to_avg.tan = tan_scores.at(i).tan;
        opposite_tangent_to_avg.score = tan_scores.at(i).score;
      } else if (opposite_num_tangent_to_avg < NUM_TANS_TO_INIT && opposite_tangent_to_avg.tan.dot(tan_scores.at(i).tan) > 0.3) {
        opposite_tangent_to_avg.tan = (opposite_tangent_to_avg.tan*opposite_tangent_to_avg.score + tan_scores.at(i).tan*tan_scores.at(i).score);
        opposite_tangent_to_avg.score = opposite_tangent_to_avg.score + tan_scores.at(i).score;
        //opposite_tangent_to_avg.tan /= opposite_tangent_to_avg.score;
        opposite_tangent_to_avg.tan.normalize();
        opposite_num_tangent_to_avg++;
      }

    }
  }

  
  //calculate transform
	Vector3d axis = Vector3d::UnitX().cross(tangent_to_avg.tan);
	if (axis.norm() != 0.0)
		axis.normalize();
  Matrix3d rot_to_start(Eigen::AngleAxisd(acos(Vector3d::UnitX().dot(tangent_to_avg.tan)),axis));
  tangent_to_avg.transform_to_tan.corner(Eigen::TopLeft,3,3) = rot_to_start;
  tangent_to_avg.transform_to_tan(0,3) = (double)start.pt3d.x;
  tangent_to_avg.transform_to_tan(1,3) = (double)start.pt3d.y;
  tangent_to_avg.transform_to_tan(2,3) = (double)start.pt3d.z;
  tangent_to_avg.transform_to_tan(3,3) = 1.0;
  tangent_to_avg.transform_to_tan(3,0) = tangent_to_avg.transform_to_tan(3,1) = tangent_to_avg.transform_to_tan(3,2) = 0.0;
  tangents.push_back(tangent_to_avg);


  if (opposite_num_tangent_to_avg >= 1)
  {
    Vector3d axis_opposite = Vector3d::UnitX().cross(opposite_tangent_to_avg.tan);
    if (axis.norm() != 0.0)
      axis.normalize();
    Matrix3d rot_to_start_opposite(Eigen::AngleAxisd(acos(Vector3d::UnitX().dot(opposite_tangent_to_avg.tan)),axis_opposite));
    opposite_tangent_to_avg.transform_to_tan.corner(Eigen::TopLeft,3,3) = rot_to_start_opposite;
    opposite_tangent_to_avg.transform_to_tan(0,3) = (double)start.pt3d.x;
    opposite_tangent_to_avg.transform_to_tan(1,3) = (double)start.pt3d.y;
    opposite_tangent_to_avg.transform_to_tan(2,3) = (double)start.pt3d.z;
    opposite_tangent_to_avg.transform_to_tan(3,3) = 1.0;
    opposite_tangent_to_avg.transform_to_tan(3,0) = opposite_tangent_to_avg.transform_to_tan(3,1) = opposite_tangent_to_avg.transform_to_tan(3,2) = 0.0;
    tangents.push_back(opposite_tangent_to_avg);

    opposite_tangents.push_back(opposite_tangent_to_avg);
  }
  */
 
  
  //FOR DEBUG - mark best match AND Tan
  Vector3d tangent = tangents[0].tan;
 

/*  
  std::cout << "best tan: " << tangent << "  with score " << tangents[0].score << std::endl;
  std::cout << "best tan transform: " << tangents[0].transform_to_tan << std::endl;
  std::cout << "start pt 3d: " << start.pt3d << std::endl;
  std::cout << "start pts: " << start.pts2d[0] << "\n" << start.pts2d[1] << "\n" << start.pts2d[2] << "\n"; 
*/
  Point2i proj_tan[NUMCAMS]; 
  Point3f bestTanLocation(  (float)(length_for_tan*tangent(0)) + start.pt3d.x,
                            (float)(length_for_tan*tangent(1)) + start.pt3d.y, 
                            (float)(length_for_tan*tangent(2)) + start.pt3d.z);


  _cams->project3dPoint(bestTanLocation, proj_tan);

/*  std::cout << "proj pts: " << proj_tan[0] << "\n" << proj_tan[1] << "\n" << proj_tan[2] << "\n"; 
  std::cout << "proj vals: ";
  for (int camNum=0; camNum < NUMCAMS; camNum++)
    std::cout << (int)(_cannyIms[camNum].at<unsigned char>(proj_tan[camNum].y, proj_tan[camNum].x));
  std::cout << std::endl;
*/

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


void ThreadStereo_SpaceCurve::addStartPoints(corresponding_pts& start, tangent_and_score& tanToStart, vector<ThreadPiece_Vision*>* hypothsToAddTo)
{
  for (double curvature = INIT_CURVATURE_MIN; curvature <= INIT_CURVATURE_MAX; curvature += INIT_CURVATURE_ADD)
  {
    for (double torsion = INIT_TORSION_MIN; torsion <= INIT_TORSION_MAX; torsion += INIT_TORSION_ADD)
    {
      ThreadPiece_Vision* toAdd = new ThreadPiece_Vision(curvature, torsion, LENGTH_THREAD_EACH_PIECE);
     // toAdd->_start_pt = start;
      toAdd->_transform_after = tanToStart.transform_to_tan*toAdd->_transform;
      toAdd->_numPieces = 0;

      Point3f pointAtEnd;
      toAdd->getLastPoint(pointAtEnd);
      double vis_score = scoreProjection3dPoint(pointAtEnd);
      if (vis_score <= THREAD_PIECE_PROJECTION_ERROR_THRESH)
      {
        toAdd->setScore(start.score + tanToStart.score + vis_score);
        hypothsToAddTo->push_back(toAdd);
      } else {
        delete toAdd;
      }


    }
  }

}


void ThreadStereo_SpaceCurve::addStartPointsWithRotation(corresponding_pts& start, tangent_and_score& tanToStart)
{
  Vector3d initAxis = tanToStart.transform_to_tan.corner(Eigen::TopLeft,3,3)*(Vector3d::UnitX());
  for (double first_rot = -M_PI; first_rot < M_PI; first_rot += (ROTATE_FIRST_AXIS_BY_ANG))
  {
    tangent_and_score tanNext = tangent_and_score(tanToStart);
    tanNext.transform_to_tan.corner(Eigen::TopLeft,3,3) = tanNext.transform_to_tan.corner(Eigen::TopLeft,3,3)* Eigen::AngleAxisd(first_rot, initAxis);

    addStartPoints(start, tanNext, currHypoths);
  }


}

bool ThreadStereo_SpaceCurve::updateHypoths(vector<ThreadPiece_Vision*>* newHypoths)
{
  newHypoths->resize(0);
  for (int hypothInd = 0; hypothInd < currHypoths->size(); hypothInd++)
  {
    double curvature = AUGMENT_CURVATURE_MIN + currHypoths->at(hypothInd)->_curvature;
    double curvature_max = AUGMENT_CURVATURE_MAX + currHypoths->at(hypothInd)->_curvature;
    for (; curvature <= curvature_max; curvature += AUGMENT_CURVATURE_ADD)
    {
      double torsion = AUGMENT_TORSION_MIN + currHypoths->at(hypothInd)->_torsion;
      double torsion_max = AUGMENT_TORSION_MAX + currHypoths->at(hypothInd)->_torsion;
      for (; torsion <= torsion_max; torsion += AUGMENT_TORSION_ADD)
      {
        ThreadPiece_Vision* toAdd = new ThreadPiece_Vision(curvature, torsion, LENGTH_THREAD_EACH_PIECE);
        toAdd->connectPrevSegment(currHypoths->at(hypothInd));

        Point3f pointAtEnd;
        toAdd->getLastPoint(pointAtEnd);
        double vis_score = scoreProjection3dPoint(pointAtEnd);
        if (vis_score <= THREAD_PIECE_PROJECTION_ERROR_THRESH)
        {
          toAdd->setScore(vis_score);
          newHypoths->push_back(toAdd);
        } else {
          toAdd->removeLastBeforeDelete();
          delete toAdd;
        }

      }
    }


  }


  if (newHypoths->size() == 0)
  {
    return false;

  }


  int numHypothsNext = min((int)newHypoths->size(), ((newHypoths->at(0)->_numPieces > NUMITERSWITHINITHYPOTHS) ? NUMHYPOTHS : NUMHYPOTHSINIT));
  partial_sort(newHypoths->begin(), newHypoths->begin()+numHypothsNext, newHypoths->end(), lessThanThreadPiecePointer);
  newHypoths->resize(numHypothsNext);


//  std::cout << newHypoths->at(0)->_score << " " <<  newHypoths->at(1)->_score << " " << newHypoths->at(2)->_score << " " << newHypoths->at(3)->_score << std::endl;

  return true;
  
}



void ThreadStereo_SpaceCurve::optimizeCurveFromPoints(ThreadPiece_Vision* lastPiece)
{
  opt_params_many_points.num_segments = lastPiece->_numPieces;
  opt_params_many_points.length_per_segment = LENGTH_THREAD_EACH_PIECE;
  opt_params_many_points.orig_params_each_piece = new NEWMAT::ColumnVector(2*lastPiece->_numPieces);
  

  ThreadPiece_Vision* firstPiece = lastPiece;
  while (firstPiece->_last_segment != NULL)
  {
    firstPiece->_last_segment->_next_segment = firstPiece;
    opt_params_many_points.orig_params_each_piece->element(2*(firstPiece->_numPieces-1)) = firstPiece->_curvature;
    opt_params_many_points.orig_params_each_piece->element(2*(firstPiece->_numPieces)-1) = firstPiece->_torsion;
    firstPiece = firstPiece->prev_segment();
  }

  firstPiece->getTransformBefore(opt_params_many_points.transform_back);

  opt_params_many_points.points.resize(lastPiece->_numPieces +1);
  firstPiece->getFirstPoint(opt_params_many_points.points[0]);
  ThreadPiece_Vision* currPiece = firstPiece;
  int currInd = 1;
  for (; currInd <= opt_params_many_points.num_segments; currInd++)
  {
    currPiece->getLastPoint(opt_params_many_points.points[currInd]);
    currPiece = currPiece->next_segment();
  }

 
  //optimize
  NEWMAT::ColumnVector x_sol;
  std::cout << "starting opt" << std::endl;
  optimizeVision_FDNLF(2*lastPiece->_numPieces, energyEvalFunctionManyPoints, energyEvalFunctionManyPoints_init, x_sol);

  for (int i=1; i <= lastPiece->_numPieces; i++)
  {
    std::cout << "curvature: " << x_sol(2*i-1) << "    torsion: " << x_sol(2*i) << std::endl;
  }


  //set params
  currPiece = firstPiece;
  while (currPiece != NULL)
  {
    int num = currPiece->_numPieces;
    currPiece->setParams(x_sol(2*num-1), x_sol(2*num), LENGTH_THREAD_EACH_PIECE);
    currPiece = currPiece->next_segment();
  }



}




double ThreadStereo_SpaceCurve::scoreProjection3dPoint(const Point3f& pt3d)
{
  double score = 0.0;
  //project 3d point to images, and see how far away the projection is (in pixels) to nearest thread pixel
  Point2f pts2d[NUMCAMS];
  _cams->project3dPoint(pt3d, pts2d);

  for (int camNum=0; camNum < NUMCAMS; camNum++)
  {
    double thisMinScore = DBL_MAX;
    Point2i rounded = Point2i(floor(pts2d[camNum].x + 0.5), floor(pts2d[camNum].y + 0.5));

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
    score += thisMinScore;
    /*if (score < 2.0)
    {
      std::cout << "point: " <<  pts2d[camNum] <<std::endl;
      std::cout << "this score " << score << std::endl;
    }*/
    //if we didn't find anything...
//    if(thisMinScore == INT_MAX)
//      return DBL_MAX;

  }

  return score;
}




void ThreadStereo_SpaceCurve::updateCanny()
{
  //FOR DEBUG - don't undistort
  _cams->updateImagesBlockingNoUndistort();
  _frames = _cams->frames();
  //_cams->filterCanny();
  //_cannyIms = _cams->cannyIms();
  gray_to_canny();
  precomputeDistanceScores();
}


void ThreadStereo_SpaceCurve::gray_to_canny()
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


void ThreadStereo_SpaceCurve::precomputeDistanceScores()
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


//TODO - process from this queue instead of re-running the distance calcualtor from scratch!
void ThreadStereo_SpaceCurve::removeUsedDistanceScores(ThreadPiece_Vision* last_piece)
{
  if (last_piece == NULL || last_piece->_last_segment == NULL)
    return;
 
  Point3f last_pt3d;
  Point2i* last_pts = new Point2i[NUMCAMS];
  last_piece->getLastPoint(last_pt3d);
  _cams->project3dPoint(last_pt3d, last_pts);

  Point3f curr_pt3d;
  Point2i* curr_pts;

  queue<Point2i> pointsToRemove[NUMCAMS];
  ThreadPiece_Vision* curr_piece = last_piece;


  //first, add a few points around the final piece (since sometimes thread won't go to end)
  for (int camNum = 0; camNum < NUMCAMS; camNum++)
  {
    int minrow = max(0, last_pts[camNum].y-5);
    int mincol = max(0, last_pts[camNum].x-5);
    int maxrow = min(rows[camNum], last_pts[camNum].y+5);
    int maxcol = min(cols[camNum], last_pts[camNum].x+5);

    //add to queue if it is a thread point
    for (int row=minrow; row <=maxrow; row++)
    {
      unsigned char* row_ptr = _cannyIms[camNum].ptr<unsigned char>(row);
      for (int col=mincol; col <=maxcol; col++)
      {
        if (row_ptr[col] >= IM_VALUE_CHECKED_FOR_BEGIN)
        {
          row_ptr[col] = IM_VALUE_USED_IN_THREAD;
          pointsToRemove[camNum].push(Point2i(col,row));
        }
      }
    }

  }



  bool doneAddingToQueue = false;
  while (!doneAddingToQueue)
  {
    //get and project new point
    if (curr_piece->_last_segment != NULL)
    {
      curr_piece = curr_piece->prev_segment();
      curr_piece->getLastPoint(curr_pt3d);
    } else {
      curr_piece->getFirstPoint(curr_pt3d);
      doneAddingToQueue = true;
    }
      std::cout << curr_pt3d << std::endl;
    curr_pts = new Point2i[NUMCAMS];
    _cams->project3dPoint(curr_pt3d, curr_pts);

    //add points from last that we want to check
    addThreadPointsBetweenToQueue(curr_pts, last_pts, pointsToRemove);
    
    last_pt3d = curr_pt3d;
    delete last_pts;
    last_pts = curr_pts;
  }

  //now, add start point


  //as before, mark the first piece (since thread sometimes will miss it)
  for (int camNum = 0; camNum < NUMCAMS; camNum++)
  {
    int minrow = max(0, last_pts[camNum].y-5);
    int mincol = max(0, last_pts[camNum].x-5);
    int maxrow = min(rows[camNum], last_pts[camNum].y+5);
    int maxcol = min(cols[camNum], last_pts[camNum].x+5);

    //add to queue if it is a thread point
    for (int row=minrow; row <=maxrow; row++)
    {
      unsigned char* row_ptr = _cannyIms[camNum].ptr<unsigned char>(row);
      for (int col=mincol; col <=maxcol; col++)
      {
        if (row_ptr[col] >= IM_VALUE_CHECKED_FOR_BEGIN)
        {
          row_ptr[col] = IM_VALUE_USED_IN_THREAD;
          pointsToRemove[camNum].push(Point2i(col,row));
        }
      }
    }

  }


  delete last_pts;

  precomputeDistanceScores();
}

void ThreadStereo_SpaceCurve::addThreadPointsBetweenToQueue(Point2i* curr_pts, Point2i* last_pts, queue<Point2i>* pointsToRemove)
{
  for (int camNum=0; camNum<NUMCAMS; camNum++)
  {
    int minrow = INT_MAX;
    int mincol = INT_MAX;
    int maxrow = 0;
    int maxcol = 0;
    location_and_distance* landd = &_cannyDistanceScores[camNum][keyForHashMap(camNum, curr_pts[camNum].y, curr_pts[camNum].x)];
    while (landd != NULL)
    {
      minrow = min(minrow, landd->row);
      maxrow = max(maxrow, landd->row);
      mincol = min(mincol, landd->col);
      maxcol = max(maxcol, landd->col);

      landd = landd->next;
    }
    landd = &_cannyDistanceScores[camNum][keyForHashMap(camNum, last_pts[camNum].y, last_pts[camNum].x)];
    while (landd != NULL)
    {
      minrow = min(minrow, landd->row);
      maxrow = max(maxrow, landd->row);
      mincol = min(mincol, landd->col);
      maxcol = max(maxcol, landd->col);

      landd = landd->next;
    }


    //add one and make sure it's within bounds
    minrow = max(0, minrow-1);
    mincol = max(0, mincol-1);
    maxrow = min(rows[camNum], maxrow+1);
    maxcol = min(cols[camNum], maxcol+1);

    //add to queue if it is a thread point
    for (int row=minrow; row <=maxrow; row++)
    {
      unsigned char* row_ptr = _cannyIms[camNum].ptr<unsigned char>(row);
      for (int col=mincol; col <=maxcol; col++)
      {
        if (row_ptr[col] >= IM_VALUE_CHECKED_FOR_BEGIN)
        {
          row_ptr[col] = IM_VALUE_USED_IN_THREAD;
          pointsToRemove[camNum].push(Point2i(col,row));
        }
      }
    }


  }

}


/*
bool ThreadStereo_SpaceCurve::connectPoints(int camNum, Point2i& start, Point2i& end, queue<Point2i> connected)
{
  const int ofs[][2] = {{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1},{0,1},{1,1}};
  map<int,bool> already_checked;
  
  point_and_pointer* toreturn = NULL;

  priority_queue<point_and_pointer*> toProcess;
  toProcess.push(new point_and_pointer(start, 0, -norm(start-end)) );
  bool found = false;
  while (!toProcess.empty())
  {
    point_and_pointer* curr = toProcess.top();
    toProcess.pop();

    for (int i=0; i < 9; i++)
    {
      Point2i next = Point2i(curr->point.x+ofs[i][0], curr->point.y+ofs[i][1]);
      int key = keyForHashMap(camNum, next.y, next.x);

      if (already_checked.count(key) > 0)
        continue;
      already_checked[key] = true;

      if (next == end)
      {
        connected->point = next;
        connected->numPts = curr->numPts+1;
        connected->distToEnd = 0.0;
        connected->next = curr;
        found = true;
        break;
      }

      if (curr->numPts >= NUM_POINT_AND_POINTER_THRESHOLD)
        continue;

      if (_cannyIms[camNum].at<unsigned char>(next.y, next.x) >= IM_VALUE_CHECKED_FOR_BEGIN)
      {
        toProcess.push(new point_and_pointer(next, curr->numPts+1, -norm(next-end), curr)); 
      }

    }

    if (found)
      break;
  }

  //delete everything left
  while (!toProcess.empty())
    toProcess.pop();

  return found;
}
*/





void ThreadStereo_SpaceCurve::optimizeVision_GSS(int numParams, OPTPP::USERFCN0 eval, OPTPP::INITFCN init, NEWMAT::ColumnVector& solution)
{
  OPTPP::NLF0 nlp(numParams, eval, init); 
  OPTPP::GenSetStd gs(numParams); 
  OPTPP::OptGSS optobj(&nlp, &gs);
  optobj.setMaxIter(5000); 
  optobj.setMaxFeval(20000); 
  optobj.setFcnTol(1e-8);  
  optobj.setFullSearch(true);
  optobj.optimize();

  solution = nlp.getXc();

}


void ThreadStereo_SpaceCurve::optimizeVision_FDNLF(int numParams, OPTPP::USERFCN0 eval, OPTPP::INITFCN init, NEWMAT::ColumnVector& solution)
{
  OPTPP::FDNLF1 nlp(numParams, eval, init);

  OPTPP::OptQNewton objfcn(&nlp);

  objfcn.setSearchStrategy(OPTPP::TrustRegion);
  objfcn.setMaxFeval(20000);
  objfcn.setMaxIter(5000);
  objfcn.setFcnTol(1.e-8);

  objfcn.optimize();
  //objfcn.printStatus("Solution");

  solution = nlp.getXc();

}







void energyEvalFunctionManyPoints_init(int ndim, NEWMAT::ColumnVector& x)
{
	for (int i = 0; i < ndim; i++)
	{
		x.element(i) = opt_params_many_points.orig_params_each_piece->element(i);
	}
}


void energyEvalFunctionManyPoints(int ndim, const NEWMAT::ColumnVector& x, double& fx, int& result)
{
  //We have ndim/2 curve segments in x, where x(2*i-1) and x(2*i) are curvature and torsion of segment i

	//error from energy
	fx = 0.0;
	for (int i=1; i <= ndim; i++)
	{
		fx += x(i)*x(i);
	}
	fx *= OPTIMIZATION_COEFF_ENERGY*opt_params_many_points.length_per_segment;
  //std::cout << "thread energy: " << fx << std::endl;
  
  //error from difference in curve params
  double diff_params = 0;
	for (int i=3; i <= ndim; i++)
	{
		diff_params += pow(x(i)-x(i-2),2);
	}
  fx += diff_params*OPTIMIZATION_COEFF_DIFF_PARAMS;
  //std::cout << "diff thread params: " << diff_params*OPTIMIZATION_COEFF_DIFF_PARAMS << std::endl;



  Matrix4d transform_to_end = opt_params_many_points.transform_back;
  //std::cout << "transform: " << transform_to_end;
 // std::cout << "angles: " << x(ndim-2) << " " << x(ndim-1) << " " << x(ndim) << std::endl;
  
  double curr_z = transform_to_end(2,3);

  //energy from gravity and vision
  double grav_energy = 0.0;
  double point_error_energy = 0.0;

	Matrix4d nextTransform;
	for (int i=1; i <= (ndim)/2; i++)
	{
			getTransform(x(2*i-1), x(2*i), opt_params_many_points.length_per_segment, nextTransform);
			transform_to_end *= nextTransform;
      grav_energy += ((transform_to_end(2,3)-curr_z)/2.0 + curr_z)*opt_params_many_points.length_per_segment;
      curr_z = transform_to_end(2,3);

      //point
      //error in position
      Vector3d new_end_position = transform_to_end.corner(Eigen::TopRight,3,1);
      point_error_energy += OPTIMIZATION_COEFF_POINT_EXPONENTIAL*pow(  ((new_end_position - opt_params_many_points.points.at(i-1)).norm()),2);
	}

  fx += grav_energy*OPTIMIZATION_COEFF_GRAVITY + point_error_energy*OPTIMIZATION_COEFF_POINT;
//  std::cout << "vision energy: " << vision_energy*OPTIMIZATION_COEFF_VISION << std::endl;
//  std::cout << "grav energy: " << grav_energy*OPTIMIZATION_COEFF_GRAVITY << std::endl;

  result = OPTPP::NLPFunction;


}







bool operator <(const tangent_and_score& a, const tangent_and_score& b)
{
  return a.score < b.score;
}

/*
bool operator <(const point_and_pointer& a, const point_and_pointer& b)
{
  return a.distToEnd < b.distToEnd;
}
*/


location_and_distance::~location_and_distance()
{
  if (next != NULL)
  {
    delete next;
    next = NULL;
  }

}

