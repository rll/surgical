#include "ThreadStereo_SpaceCurve_MinEnergy.h"
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

  //_captures[0]->init("./calib-apr21/");
  //_captures[1]->init("./calib-apr21/");
  //_captures[2]->init("./calib-apr21/");
  _captures[0]->init("./calib_params/");
  _captures[1]->init("./calib_params/");
  _captures[2]->init("./calib_params/");
  
  
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

  length_thread_each_piece = INIT_LENGTH_THREAD_EACH_PIECE;
  total_length = -1.0;
  num_pieces_wanted = NUM_PIECES_WANTED;

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


  //FOR DEBUG
  for (int i=0; i < NUMCAMS; i++)
  {
    display_for_debug[i].resize(0);
  }


}



void ThreadStereo_SpaceCurve::getThreadPoints(vector<glline_draw_params>& gl_draw_params, Point2i& initPtCenterIm)
{
  vision_return toReturn;
  
  gl_draw_params.resize(0);
  gl_display_for_debug = &gl_draw_params;
  Scalar redColor = Scalar(0, 0, 255);
  updateCanny();


  initializeThreadSearch();


  bool done = false;
  int numCheck = 0;
  int numdisp = 0;

  vector<corresponding_pts> start_pts;
  vector<tangent_and_score> tangent_to_start;

  toReturn.length_per_piece = length_thread_each_piece;

  if(findNextStartPoint(start_pts, initPtCenterIm))
  {
    tangent_to_start.resize(0);
    if (findTangent(start_pts[0], tangent_to_start))
    {
      //check to see if we have looked for this thread before
      if (total_length == TOTAL_LENGTH_INIT)
      {
        total_length = MAX_LENGTH_THREAD;
        if (processHypothesesFromInit(start_pts[0], tangent_to_start[0], toReturn))
        {
          //thread found - since this is the first time, set length
          total_length = toReturn.total_length;
          length_thread_each_piece = total_length/(double)num_pieces_wanted;
        } else {
          //reset length
          total_length = TOTAL_LENGTH_INIT;
        }
      } else {
        if (processHypothesesFromInit(start_pts[0], tangent_to_start[0], toReturn))
        {
        }
      }

    }
  }

  std::cout << "start tan:" << toReturn.init_tan << std::endl;
  std::cout << "end tan:" << toReturn.final_tan << std::endl;




//FOR DEBUG - display the points
  for (int i=0; i < NUMCAMS; i++)
  {
  cvtColor(_cannyIms[i], _frames[i], CV_GRAY2BGR);
    for (int disp_ind = 0; disp_ind < display_for_debug[i].size(); disp_ind++)
    {
      const Point* pt[] = {display_for_debug[i][disp_ind].pts};

      cv::polylines(_frames[i], pt, &display_for_debug[i][disp_ind].size, 1, false, display_for_debug[i][disp_ind].color, 2);

    }


    cv::imshow(_names[i], _frames[i]); 

  }

  waitKey(100);


}



bool ThreadStereo_SpaceCurve::processHypothesesFromInit(corresponding_pts& start, tangent_and_score& tan, vision_return& toReturn)
{
  vector<ThreadPiece_Vision*> currPieces;
  vector<ThreadPiece_Vision*> oppositePieces;
  int num_pieces_max = (int)(total_length/length_thread_each_piece);
  std::cout << "num pieces: " << num_pieces_max << std::endl;

  ThreadOptimizingModes mode = INITIALIZING;
  bool done = false;
  while (!done)
  {
    switch (mode)
    {
      case (INITIALIZING):
        if (initializeThread(start,tan,currPieces))
        {
          mode = CONTINUING;
        } else {
          if (currPieces.size() <= NUM_THREAD_PIECES_FIRST_OPT)
            currPieces.resize(0);

          mode = INIT_OPPOSITE;
        }
      break;
      
      case (CONTINUING):
        /*if (continueThreadUntilEnd(currPieces, num_pieces_max) && !isEndPiece(start.pt3d) && num_pieces_max > 0)
        {
          mode = INIT_OPPOSITE;
        } else {
          //could not continue
          toReturn.pieces.resize(currPieces.size());
          for (int i=0; i < currPieces.size(); i++)
          {
            toReturn.pieces[i] = currPieces[i];
            currPieces[i] = NULL;
          }
          done = true;
        }*/
        continueThreadUntilEnd(currPieces, num_pieces_max);
        toReturn.pieces.resize(currPieces.size());
        for (int i=0; i < currPieces.size(); i++)
        {
          toReturn.pieces[i] = currPieces[i];
          currPieces[i] = NULL;
        }
        done = true;
      break;

      /*
      case (INIT_OPPOSITE):
        ThreadPiece_Vision* firstPiece = currPieces[0];
        Matrix4d inv_trans;
        firstPiece->getTransformBefore(inv_trans);
        Matrix3d oldRot = inv_trans.corner(Eigen::TopLeft,3,3);
        inv_trans.corner(Eigen::TopLeft,3,3) = (Eigen::AngleAxisd(M_PI, inv_trans.block(0,1,3,1)))*oldRot;

        if (!continueThreadOpposite(oppositePieces, inv_trans, firstPiece->_curvature, firstPiece->_torsion, num_pieces_max))
        {
          done = true; //no pieces this way!
          break;
        }

        std::cout << "size opposite: " << oppositePieces.size() << std::endl;

        //now, we need to put the pieces together
        ThreadPiece_Vision* currPieceToAdd = oppositePieces.back();
        inv_trans = currPieceToAdd->_transform_after;
        oldRot = inv_trans.corner(Eigen::TopLeft,3,3);
        inv_trans.corner(Eigen::TopLeft,3,3) = (Eigen::AngleAxisd(M_PI, inv_trans.block(0,1,3,1)))*oldRot;
        toReturn.pieces.push_back(new ThreadPiece_Vision(currPieceToAdd->_curvature, currPieceToAdd->_torsion, currPieceToAdd->_length));
        toReturn.pieces.back()->setPrevTransform(inv_trans);
        toReturn.pieces.back()->_numPieces = 1;
        int pieceNum;
        for (pieceNum = 1; pieceNum < oppositePieces.size(); pieceNum++)
        {
          currPieceToAdd = oppositePieces[oppositePieces.size()-1-pieceNum];
          toReturn.pieces.push_back(new ThreadPiece_Vision(currPieceToAdd->_curvature, currPieceToAdd->_torsion, currPieceToAdd->_length, toReturn.pieces[pieceNum-1]));
        }

        for (pieceNum = 0; pieceNum < currPieces.size(); pieceNum++)
        {
          currPieceToAdd = currPieces[pieceNum];
          toReturn.pieces.push_back(new ThreadPiece_Vision(currPieceToAdd->_curvature, currPieceToAdd->_torsion, currPieceToAdd->_length, toReturn.pieces[toReturn.pieces.size()-1]));
        }

        done = true;
      
      break;
      */

    }
  }


  toReturn.total_length = toReturn.pieces.size()*toReturn.length_per_piece;
  toReturn.length_per_piece = length_thread_each_piece;
  Matrix4d transform_before;
  toReturn.pieces.front()->getTransformBefore(transform_before);
  toReturn.init_tan = transform_before.corner(Eigen::TopLeft,3,1);
  toReturn.final_tan = toReturn.pieces.back()->_transform_after.corner(Eigen::TopLeft,3,1);

  threadPiecesLast.resize(0);
  /*if ((int)toReturn.pieces.size() <= NUM_PIECES_WANTED+1)
  {
    threadPiecesLast.push_back(new ThreadPiece_Vision(toReturn.pieces[0]));
    for (int i=1; i < toReturn.pieces.size(); i++)
    {
      threadPiecesLast.push_back(new ThreadPiece_Vision(toReturn.pieces[i], toReturn.pieces[i-1]));
    }
  }*/

  //resample, reoptimize
  //resampleAndReoptimize(toReturn.pieces, NUM_PIECES_AFTER_REOPTIMIZATION, toReturn.pieces_reopt);
  setNextPointers(toReturn.pieces);



  delete opt_params_vision.orig_params_each_piece;

  //need to set the curr pieces




  //FOR DEBUG - mark the point of best thread
  if ( toReturn.pieces.size() > 0 )
  {
    Point3f currPoint;
    Point2i imPoints[NUMCAMS];
    int numPieces = toReturn.pieces.back()->_numPieces;
    std::cout << "num Pieces final: " << numPieces << std::endl;
    //removeUsedDistanceScores(curr);
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


    int ind=0;



    MatrixXd resampled_points(numPieces+1,3);
    Matrix4d first_trans;
    toReturn.pieces[0]->getTransformBefore(first_trans);
    toReturn.pieces[0]->getPoints(resampled_points, 0.0, toReturn.total_length/((double)numPieces), 0, first_trans);    
    
    for (ind = 0; ind < toReturn.pieces.size(); ind++)
    {
      std::cout << "curve params: " << toReturn.pieces[ind]->_curvature << "  " << toReturn.pieces[ind]->_torsion << std::endl;
    }


    for (ind=0; ind < numPieces+1; ind++)
    {
      currPoint.x = (float)resampled_points(ind,0);
      currPoint.y = (float)resampled_points(ind,1);
      currPoint.z = (float)resampled_points(ind,2);
      _cams->project3dPoint(currPoint,imPoints);

      std::cout << "curr pos: " << currPoint << std::endl;
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
    toReturn.pieces.back()->getTransformBefore(firstTransform);
    std::cout << "tangent: \n" << firstTransform.corner(Eigen::TopLeft,3,1) << std::endl; 

  }

  
 return true; 

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

bool ThreadStereo_SpaceCurve::findNextStartPoint(vector<corresponding_pts>& pts, Point2i& initPtCenterIm)
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
      int numPtsToResize = min(pts.size(),NUM_START_PTS_TO_INIT);
      partial_sort(pts.begin(), pts.begin()+numPtsToResize, pts.end());
      break;
    }

  }

  return done;

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


bool ThreadStereo_SpaceCurve::findTangent(corresponding_pts& start, vector<tangent_and_score>& tangents)
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

  /*
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
     }*/


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


bool ThreadStereo_SpaceCurve::initializeThread(corresponding_pts& start, tangent_and_score& tan, vector<ThreadPiece_Vision*>& currPieces)
{
  Point3f lastPoint;

  //set opt params
  opt_params_vision.length_per_segment = length_thread_each_piece;
  opt_params_vision.threadStereo = this;

  opt_params_vision.transform_back = Matrix4d::Identity();
  Vector3d startPt((double)start.pt3d.x, (double)start.pt3d.y, (double)start.pt3d.z);
  opt_params_vision.transform_back.corner(Eigen::TopRight,3,1) = startPt;

  opt_params_vision.orig_params_each_piece = new NEWMAT::ColumnVector(4*NUM_THREAD_PIECES_INIT_OPT+6); 


  //initialize x_sol as if we just finished one iteration
  NEWMAT::ColumnVector x_sol;
  x_sol.ReSize(2*NUM_THREAD_PIECES_FIRST_OPT+3);
  for (int i=0; i < NUM_THREAD_PIECES_FIRST_OPT; i++)
  {
    if (i < threadPiecesLast.size())
    {
      x_sol.element(2*i) = threadPiecesLast[i]->_curvature;
      x_sol.element(2*i+1) = threadPiecesLast[i]->_torsion;
    } else {
      x_sol.element(2*i) = OPTIMIZATION_INIT_CURVATURE;
      x_sol.element(2*i+1) = OPTIMIZATION_INIT_TORSION;
    }
  }
  x_sol.element(2*NUM_THREAD_PIECES_FIRST_OPT    ) = tan.rot1;
  x_sol.element(2*NUM_THREAD_PIECES_FIRST_OPT + 1) = tan.rot2;
  x_sol.element(2*NUM_THREAD_PIECES_FIRST_OPT + 2) = 0.0;


  //initialize thread as if we just finished one iteration
  Matrix4d transform_at_start;
  transformFromEulerAngles(transform_at_start, x_sol(x_sol.nrows()-2), x_sol(x_sol.nrows()-1), x_sol(x_sol.nrows()), startPt);
  currPieces.resize(0);
  currPieces.push_back(new ThreadPiece_Vision(x_sol(1), x_sol(2), length_thread_each_piece));
  currPieces.back()->setPrevTransform(transform_at_start);
  currPieces.back()->_numPieces = 1;
  for (int i=1; i < NUM_THREAD_PIECES_FIRST_OPT; i++)
  {
    currPieces.push_back(new ThreadPiece_Vision(x_sol.element(2*i), x_sol.element(2*i+1), length_thread_each_piece, currPieces[i-1]));
  } 



  for (int numPiecesThisOpt = NUM_THREAD_PIECES_FIRST_OPT; numPiecesThisOpt <= NUM_THREAD_PIECES_INIT_OPT; numPiecesThisOpt++)
  {
    opt_params_vision.num_segments = numPiecesThisOpt;

    for (int i=0; i < numPiecesThisOpt; i++)
    {
      if (i < (x_sol.nrows()-3)/2)
      {
        opt_params_vision.orig_params_each_piece->element(2*i) = x_sol.element(2*i);
        opt_params_vision.orig_params_each_piece->element(2*i+1) = x_sol.element(2*i+1);
      } else if (i < threadPiecesLast.size()) {
        opt_params_vision.orig_params_each_piece->element(2*i) = threadPiecesLast[i]->_curvature;
        opt_params_vision.orig_params_each_piece->element(2*i+1) = threadPiecesLast[i]->_torsion;
      } else {
        opt_params_vision.orig_params_each_piece->element(2*i) = x_sol.element(2*(numPiecesThisOpt-1));
        opt_params_vision.orig_params_each_piece->element(2*i+1) = x_sol.element(2*(numPiecesThisOpt)-1);
      }
    }
    opt_params_vision.orig_params_each_piece->element(2*numPiecesThisOpt    ) = x_sol(x_sol.nrows()-2);
    opt_params_vision.orig_params_each_piece->element(2*numPiecesThisOpt + 1) = x_sol(x_sol.nrows()-1);
    opt_params_vision.orig_params_each_piece->element(2*numPiecesThisOpt + 2) = x_sol(x_sol.nrows());

    //optimize
    if (numPiecesThisOpt == NUM_THREAD_PIECES_INIT_OPT)
      optimizeVision_GSS(2*opt_params_vision.num_segments+3, energyEvalFunctionVisionFirst, energyEvalFunctionVision_init, x_sol, OPTIMIZATION_TOLERANCE_INIT);
    else
      optimizeVision_GSS(2*opt_params_vision.num_segments+3, energyEvalFunctionVisionFirst, energyEvalFunctionVision_init, x_sol);

    //reset thread params, see if we are done
    transformFromEulerAngles(transform_at_start, x_sol(x_sol.nrows()-2), x_sol(x_sol.nrows()-1), x_sol(x_sol.nrows()), startPt);
    currPieces[0]->setParams(x_sol(1), x_sol(2), length_thread_each_piece);
    currPieces[0]->setPrevTransform(transform_at_start);
    for (int i=1; i < numPiecesThisOpt; i++)
    {
      if (i < currPieces.size())
        currPieces[i]->setParams(x_sol.element(2*i), x_sol.element(2*i+1), length_thread_each_piece);
      else
        currPieces.push_back(new ThreadPiece_Vision(x_sol.element(2*i), x_sol.element(2*i+1), length_thread_each_piece, currPieces.back()));
    } 
    currPieces.back()->getLastPoint(lastPoint);
    if (isEndPiece(lastPoint))
    {
      return false;
    }

  }

  return true;

}


bool ThreadStereo_SpaceCurve::continueThreadUntilEnd(vector<ThreadPiece_Vision*>& currPieces, int& num_pieces_max)
{
  //initialize x_sol as if we just finished an iteration, so we can use the parameters
  NEWMAT::ColumnVector x_sol(2*NUM_THREAD_PIECES_EACH_OPT);
  Point3f lastPoint;
  for (int i=0; i < NUM_THREAD_PIECES_EACH_OPT; i++)
  {
    x_sol.element(2*i) = currPieces[NUM_THREAD_PIECES_INIT_OPT-NUM_THREAD_PIECES_EACH_OPT+i]->_curvature;
    x_sol.element(2*i+1) = currPieces[NUM_THREAD_PIECES_INIT_OPT-NUM_THREAD_PIECES_EACH_OPT+i]->_torsion;
  }

  //now we continuously add new pieces
  //if we need to, resize the vector for optimization
  if (opt_params_vision.orig_params_each_piece->nrows() < 2*NUM_THREAD_PIECES_EACH_OPT)
  {
    opt_params_vision.orig_params_each_piece->ReSize(2*(2*NUM_THREAD_PIECES_EACH_OPT));
  } 
  opt_params_vision.num_segments = NUM_THREAD_PIECES_EACH_OPT;
  int num_pieces_curr = NUM_THREAD_PIECES_INIT_OPT;
  std::cout << "size limit this: " << num_pieces_max << std::endl;
  while ((int)currPieces.size() <= (int)num_pieces_max)
  {
    std::cout << "curr num pieces: " << num_pieces_curr << std::endl;
    num_pieces_curr+= NUM_NEW_PIECES_EACH_OPT;

    //set params
    opt_params_vision.curvature_before = x_sol.element(2*NUM_NEW_PIECES_EACH_OPT-1);
    opt_params_vision.torsion_before = x_sol.element(2*NUM_NEW_PIECES_EACH_OPT);

    int i = 0;
    for (; i < NUM_THREAD_PIECES_EACH_OPT-NUM_NEW_PIECES_EACH_OPT; i++)
    {
      opt_params_vision.orig_params_each_piece->element(2*i) = x_sol.element(2*(i+NUM_NEW_PIECES_EACH_OPT));
      opt_params_vision.orig_params_each_piece->element(2*i+1) = x_sol.element(2*(i+NUM_NEW_PIECES_EACH_OPT)+1);
    }
    for (; i < NUM_THREAD_PIECES_EACH_OPT; i++)
    {
      if (i+(num_pieces_curr - NUM_THREAD_PIECES_EACH_OPT) < threadPiecesLast.size()) {
        opt_params_vision.orig_params_each_piece->element(2*i) = threadPiecesLast[i]->_curvature;
        opt_params_vision.orig_params_each_piece->element(2*i+1) = threadPiecesLast[i]->_torsion;
      } else {
        opt_params_vision.orig_params_each_piece->element(2*i) = x_sol.element(2*NUM_THREAD_PIECES_EACH_OPT-2);
        opt_params_vision.orig_params_each_piece->element(2*i+1) = x_sol.element(2*NUM_THREAD_PIECES_EACH_OPT-1);
      }
    }
    opt_params_vision.transform_back = currPieces[num_pieces_curr-NUM_THREAD_PIECES_EACH_OPT-1]->_transform_after;


    //optimize
    if (currPieces.size() == num_pieces_max)
      optimizeVision_GSS(2*opt_params_vision.num_segments, energyEvalFunctionVision, energyEvalFunctionVision_init, x_sol, OPTIMIZATION_TOLERANCE_LAST);
    else
      optimizeVision_GSS(2*opt_params_vision.num_segments, energyEvalFunctionVision, energyEvalFunctionVision_init, x_sol);


    //set thread params
    for (int pieceNum=num_pieces_curr-NUM_THREAD_PIECES_EACH_OPT; pieceNum < num_pieces_curr; pieceNum++)
    {
      int pieceNum_x_sol = pieceNum-(num_pieces_curr-NUM_THREAD_PIECES_EACH_OPT);
      if (pieceNum < currPieces.size())
        currPieces[pieceNum]->setParams(x_sol.element(2*pieceNum_x_sol), x_sol.element(2*pieceNum_x_sol+1), length_thread_each_piece);
      else
        currPieces.push_back(new ThreadPiece_Vision(x_sol.element(2*pieceNum_x_sol), x_sol.element(2*pieceNum_x_sol+1), length_thread_each_piece, currPieces.back()));
    } 

    currPieces.back()->getLastPoint(lastPoint);
    if (isEndPiece(lastPoint))
    {
      std::cout << "done adding pieces" << std::endl;
      num_pieces_max -= currPieces.size();
      return true;
    }

  }

  /*
  for (int i=0; i < currPieces.size(); i++)
  {
    std::cout << "Thread params: " << currPieces[i]->_curvature << "  " << currPieces[i]->_torsion << std::endl;
    Vector3d pt = currPieces[i]->_transform_after.corner(Eigen::TopRight,3,1);
    std::cout << "End point: " << pt[0] << "  " << pt[1] << "  " << pt[2] << std::endl;
  } */


  num_pieces_max -= currPieces.size();
  //only get here if we exceeded max pieces
  return true;

 

}

bool ThreadStereo_SpaceCurve::continueThreadOpposite(vector<ThreadPiece_Vision*>& oppositePieces, Matrix4d& transformToStart, double curvature_before, double torsion_before, int& num_pieces_max)
{
 //initialize x_sol as if we just finished an iteration, so we can use the parameters
  NEWMAT::ColumnVector x_sol(2);
  Point3f lastPoint;
  x_sol.element(0) = OPTIMIZATION_INIT_CURVATURE;
  x_sol.element(1) = OPTIMIZATION_INIT_TORSION;

  //if we need to, resize the vector for optimization
  if (opt_params_vision.orig_params_each_piece->nrows() < 2*NUM_THREAD_PIECES_INIT_OPT)
  {
    opt_params_vision.orig_params_each_piece->ReSize(2*(2*NUM_THREAD_PIECES_INIT_OPT));
  } 
  opt_params_vision.curvature_before = curvature_before;
  opt_params_vision.torsion_before = torsion_before;
  opt_params_vision.transform_back = transformToStart;

  //initialize first thread piece
  oppositePieces.push_back(new ThreadPiece_Vision(x_sol(1), x_sol(2), length_thread_each_piece));
  oppositePieces.back()->setPrevTransform(transformToStart);
  oppositePieces.back()->_numPieces = 1;
      

  //now we continuously add new pieces
  int num_pieces_curr = 0;
  while (oppositePieces.size() < NUM_THREAD_PIECES_INIT_OPT && (int)oppositePieces.size() <= (int)num_pieces_max)
  {
    num_pieces_curr++;
    opt_params_vision.num_segments = num_pieces_curr;
    std::cout << "curr num pieces opposite: " << num_pieces_curr << std::endl;

    //set params
    int i = 0;
    for (; i < num_pieces_curr-1; i++)
    {
      opt_params_vision.orig_params_each_piece->element(2*i) = x_sol.element(2*i);
      opt_params_vision.orig_params_each_piece->element(2*i+1) = x_sol.element(2*i+1);
    }
    opt_params_vision.orig_params_each_piece->element(2*i) = x_sol.element(x_sol.nrows()-2);
    opt_params_vision.orig_params_each_piece->element(2*i+1) = x_sol.element(x_sol.nrows()-1);


    //optimize
    optimizeVision_GSS(2*opt_params_vision.num_segments, energyEvalFunctionVision, energyEvalFunctionVision_init, x_sol);

    //set thread params

    oppositePieces[0]->setParams(x_sol.element(0), x_sol.element(1), length_thread_each_piece);
    oppositePieces[0]->setPrevTransform(transformToStart);
    for (int pieceNum=1; pieceNum < num_pieces_curr; pieceNum++)
    {
      if (pieceNum < oppositePieces.size())
        oppositePieces[pieceNum]->setParams(x_sol.element(2*pieceNum), x_sol.element(2*pieceNum+1), length_thread_each_piece);
      else
        oppositePieces.push_back(new ThreadPiece_Vision(x_sol.element(2*pieceNum), x_sol.element(2*pieceNum+1), length_thread_each_piece, oppositePieces.back()));
    } 


    oppositePieces.back()->getLastPoint(lastPoint);
    if (isEndPiece(lastPoint))
    {
      num_pieces_max -= oppositePieces.size();
      std::cout << "done adding pieces opposite" << std::endl;
      return true;
    }

  }

  if ((int)oppositePieces.size() > (int)num_pieces_max)
  {
    oppositePieces.resize(num_pieces_max+1);
    num_pieces_max -= oppositePieces.size();
    return true;
  }

  //now, we need to do as before, and just add new pieces
  return continueThreadUntilEnd(oppositePieces, num_pieces_max);

}



bool ThreadStereo_SpaceCurve::resampleAndReoptimize(const vector<ThreadPiece_Vision*>& orig_pieces, int num_constraints, vector<ThreadPiece_Vision*>& new_pieces)
{
  int num_pieces_reopt = num_constraints*NUM_PIECES_BETWEEN_REOPTIMIZATION_POINTS;

  MatrixXd resampled_points(num_constraints+1,3);
  vector<double> curvatures_resampled;
  vector<double> torsions_resampled;
  double length_per_piece;
  resamplePoints(orig_pieces, num_constraints, resampled_points, length_per_piece, curvatures_resampled, torsions_resampled);
  length_per_piece /= (double)NUM_PIECES_BETWEEN_REOPTIMIZATION_POINTS;
  
  orig_pieces[0]->getTransformBefore(opt_params_many_points.transform_back);
  opt_params_many_points.num_segments = num_pieces_reopt;
  opt_params_many_points.length_per_segment = length_per_piece;


  opt_params_many_points.points.resize(num_constraints);
  opt_params_many_points.orig_params_each_piece = new NEWMAT::ColumnVector(2*num_pieces_reopt);
  for (int i=0; i < num_constraints; i++)
  {
    for (int j=0; j < NUM_PIECES_BETWEEN_REOPTIMIZATION_POINTS; j++)
    {
      int piece_num = i*NUM_PIECES_BETWEEN_REOPTIMIZATION_POINTS+j;
      opt_params_many_points.orig_params_each_piece->element(2*piece_num) = curvatures_resampled[i];
      opt_params_many_points.orig_params_each_piece->element(2*piece_num+1) = torsions_resampled[i];
    }
  }

  for (int i=0; i < num_constraints; i++)
  {
    opt_params_many_points.points[i] = resampled_points.block((i+1),0,1,3).transpose();
    //std::cout << "points: " << opt_params_many_points.points[i] << std::endl;
  }


  NEWMAT::ColumnVector x_sol;

  optimizeVision_FDNLF(2*opt_params_many_points.num_segments, energyEvalFunctionManyPoints, energyEvalFunctionManyPoints_init, x_sol);

  /*
  std::cout << "diff params: " << std::endl;
  for (int i=0; i < 2*num_pieces_reopt; i++)
  {
    std::cout << (x_sol.element(i) - opt_params_many_points.orig_params_each_piece->element(i))  << std::endl;
  }*/

  std::cout << "done re-optimizing" << std::endl;
  delete opt_params_many_points.orig_params_each_piece;


  /*
  new_pieces.resize(num_pieces_reopt);
  new_pieces[0]->setParams(x_sol(1), x_sol(2), length_per_piece);
  new_pieces[0]->setPrevTransform(opt_params_many_points.transform_back);
  for (int i=1; i < num_pieces_reopt; i++)
  {
    new_pieces[i]->setParams(x_sol.element(2*i), x_sol.element(2*i+1), length_per_piece);
  }*/
  
  new_pieces.resize(0);
  new_pieces.push_back(new ThreadPiece_Vision(x_sol(1), x_sol(2), length_per_piece));
  new_pieces.back()->setPrevTransform(opt_params_many_points.transform_back);
  new_pieces.back()->_numPieces = 1;
  for (int i=1; i < num_pieces_reopt; i++)
  {
    new_pieces.push_back(new ThreadPiece_Vision(x_sol.element(2*i), x_sol.element(2*i+1), length_per_piece, new_pieces.back()));
    new_pieces[i-1]->_next_segment = new_pieces[i];
  }



  //toReturn.pieces[0]->getTransformBefore(opt_params_many_points.transform_back);
  //std::cout << "start trans: " << opt_params_many_points.transform_back << std::endl;



}

bool ThreadStereo_SpaceCurve::resamplePoints(const vector<ThreadPiece_Vision*>& pieces, int numPieces, MatrixXd& resampled_points, double& length_per_piece, vector<double>& curvatures, vector<double>& torsions)
{
  //assumes curvatures and torsions already set to 0!
  double length_total = 0.0;
  for (int pieceNum = 0; pieceNum < pieces.size()-1; pieceNum++)
  {
    length_total += pieces[pieceNum]->_length;    //add each inidividually in case we decide not to have all lengths equal
    pieces[pieceNum]->_next_segment = pieces[pieceNum+1];
  }
  length_total += pieces.back()->_length;   


  curvatures.resize(numPieces+2);
  torsions.resize(numPieces+2);
  for (int i=0; i <= numPieces; i++)
  {
    curvatures[i] = torsions[i] = 0.0;
  }


  length_per_piece = length_total/((double)numPieces);

	Matrix4d first_trans;
  pieces[0]->getTransformBefore(first_trans);
	pieces[0]->getPointsAndParams(resampled_points, 0.0, length_per_piece, 0, first_trans, curvatures, torsions); 
 

}



double ThreadStereo_SpaceCurve::scoreProjection3dPoint(const Point3f& pt3d, double* scores)
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
    double thisMinScore = DIST_FOR_SCORE_CHECK*2;
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
    score += thisMinScore;
  }

  return score;

}


bool ThreadStereo_SpaceCurve::isEndPiece(const Point3f pt)
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


bool ThreadStereo_SpaceCurve::isEndPiece(const int camNum, const Point2i pt)
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


void ThreadStereo_SpaceCurve::setNextPointers(vector<ThreadPiece_Vision*> pieces)
{
  for (int i=0; i < pieces.size()-1; i++)
  {
    pieces[i]->_next_segment = pieces[i+1];
  }
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


void ThreadStereo_SpaceCurve::optimizeVision_GSS(int numParams, OPTPP::USERFCN0 eval, OPTPP::INITFCN init, NEWMAT::ColumnVector& solution, double fcnTol)
{
  OPTPP::NLF0 nlp(numParams, eval, init); 
  OPTPP::GenSetStd gs(numParams); 
  OPTPP::OptGSS optobj(&nlp, &gs);
  optobj.setMaxIter(5000); 
  optobj.setMaxFeval(50000); 
  optobj.setFcnTol(fcnTol);  
  optobj.setFullSearch(true);
  optobj.optimize();
  //optobj.printStatus("Solution Vision GSS");

  solution = nlp.getXc();

  /*
  std::cout << "solution \n " ;
  for (int i=1; i <= numParams; i++)
  {
    std::cout << solution(i) << std::endl;
  }*/

}


void ThreadStereo_SpaceCurve::optimizeVision_FDNLF(int numParams, OPTPP::USERFCN0 eval, OPTPP::INITFCN init, NEWMAT::ColumnVector& solution, double fcnTol)
{
  OPTPP::FDNLF1 nlp(numParams, eval, init);

  OPTPP::OptQNewton objfcn(&nlp);

  objfcn.setSearchStrategy(OPTPP::TrustRegion);
  objfcn.setMaxFeval(20000);
  objfcn.setMaxIter(5000);
  objfcn.setFcnTol(fcnTol);

  objfcn.optimize();
  objfcn.printStatus("Solution VISION");

  solution = nlp.getXc();
}





void energyEvalFunctionVision_init(int ndim, NEWMAT::ColumnVector& x)
{
	for (int i = 0; i < ndim; i++)
	{
		x.element(i) = opt_params_vision.orig_params_each_piece->element(i);
	}
}

void energyEvalFunctionVision(int ndim, const NEWMAT::ColumnVector& x, double& fx, int& result)
{
  //We have ndim/2 curve segments in x, where x(2*i-1) and x(2*i) are curvature and torsion of segment i

	//error from energy
	for (int i=1; i <= ndim; i++)
	{
		fx += x(i)*x(i);
	}
	fx *= OPTIMIZATION_COEFF_ENERGY*opt_params_vision.length_per_segment;
  //std::cout << "thread energy: " << fx << std::endl;
  
  //error from difference in curve params
  double diff_params = pow(x(1)-opt_params_vision.curvature_before,2) + pow(x(2)-opt_params_vision.torsion_before,2);
	for (int i=3; i <= ndim; i++)
	{
		diff_params += pow(x(i)-x(i-2),2);
	}
  fx += diff_params*OPTIMIZATION_COEFF_DIFF_PARAMS;
  //std::cout << "diff thread params: " << diff_params*OPTIMIZATION_COEFF_DIFF_PARAMS << std::endl;



  Matrix4d transform_to_end = opt_params_vision.transform_back;
  //std::cout << "transform: " << transform_to_end;
 // std::cout << "angles: " << x(ndim-2) << " " << x(ndim-1) << " " << x(ndim) << std::endl;
  
  double curr_z = transform_to_end(2,3);

  //energy from gravity and vision
  double grav_energy = 0.0;
  double vision_energy = 0.0;
  double distEnergy  = 0.0;

	Matrix4d nextTransform;
	for (int i=1; i <= (ndim)/2; i++)
	{
			getTransform(x(2*i-1), x(2*i), opt_params_vision.length_per_segment, nextTransform);
			transform_to_end *= nextTransform;
      grav_energy += ((transform_to_end(2,3)-curr_z)/2.0 + curr_z)*opt_params_vision.length_per_segment;
      curr_z = transform_to_end(2,3);

      //dist
      distEnergy += 1.0/pow(max(((opt_params_vision.transform_back.corner(Eigen::TopRight,3,1) - transform_to_end.corner(Eigen::TopRight,3,1)).norm()), 0.0000001),2);
      //distEnergy -= pow(max(((opt_params_vision.transform_back.corner(Eigen::TopRight,3,1) - transform_to_end.corner(Eigen::TopRight,3,1)).norm()), 0.0000001),2);
  
      //vision
      Point3f pointAtEnd = Point3f((float)transform_to_end(0,3), (float)transform_to_end(1,3), (float)transform_to_end(2,3));
      double dists[NUMCAMS];
      opt_params_vision.threadStereo->scoreProjection3dPoint(pointAtEnd, dists);
      for (int camNum=0; camNum < NUMCAMS; camNum++)
      {
        //std::cout << "dist: " << dists[camNum] << std::endl;
        //vision_energy +=  (dists[camNum] < (1.0/sqrt(2.0)) ? 0 : pow(dists[camNum],2));
        vision_energy +=  (dists[camNum] < 0.7 ? 0 : pow(dists[camNum],2));
        //vision_energy +=  pow(dists[camNum],2);
      }

	}

  fx += grav_energy*OPTIMIZATION_COEFF_GRAVITY + vision_energy*OPTIMIZATION_COEFF_VISION;
  //fx += vision_energy*OPTIMIZATION_COEFF_VISION;
  //std::cout << "vision energy: " << vision_energy*OPTIMIZATION_COEFF_VISION << std::endl;
  //std::cout << "grav energy: " << grav_energy*OPTIMIZATION_COEFF_GRAVITY << std::endl;

  distEnergy = OPTIMIZATION_COEFF_DIST*distEnergy;
  fx +=  distEnergy;

  //std::cout << "dist energy: " << distEnergy << std::endl;
  
  result = OPTPP::NLPFunction;

}

void energyEvalFunctionVisionFirst(int ndim, const NEWMAT::ColumnVector& x, double& fx, int& result)
{
  //We have (ndim-3)/2 curve segments in x, where x(2*i-1) and x(2*i) are curvature and torsion of segment i
  //the last 3 params represent the rotations to begin with


  //rotate as in yaw, pitch, roll
  Matrix3d rot1(Eigen::AngleAxisd(x(ndim-2), Vector3d::UnitZ()));
  Vector3d axis2 = rot1*(Vector3d::UnitY());
  Matrix3d rot2 = Eigen::AngleAxisd(x(ndim-1),axis2)*rot1;
  Vector3d axis3 = rot2*(Vector3d::UnitX());


  opt_params_vision.transform_back.corner(Eigen::TopLeft,3,3) = Eigen::AngleAxisd(x(ndim), axis3)*rot2;
  opt_params_vision.curvature_before = x(1);
  opt_params_vision.torsion_before = x(2);

  energyEvalFunctionVision(ndim-3, x, fx, result);

}




void energyEvalFunctionManyPoints_init(int ndim, NEWMAT::ColumnVector& x)
{
	for (int i = 0; i < ndim; i++)
	{
		x.element(i) = opt_params_many_points.orig_params_each_piece->element(i);
	}

}


//CONSIDER ADDING TANGENT AT EACH POINT AS WELL
void energyEvalFunctionManyPoints(int ndim, const NEWMAT::ColumnVector& x, double& fx, int& result)
{
  //We have ndim/2 curve segments in x, where x(2*i-1) and x(2*i) are curvature and torsion of segment i

	//error from energy
	fx = 0.0;
	for (int i=1; i <= ndim; i++)
	{
		fx += x(i)*x(i);
	}
	fx *= MANYPOINTS_COEFF_ENERGY*opt_params_many_points.length_per_segment;
  //std::cout << "thread energy: " << fx << std::endl;
  
  //error from difference in curve params
  double diff_params = 0.0;
	for (int i=3; i <= ndim; i++)
	{
		diff_params += pow(x(i)-x(i-2),2);
	}
  fx += diff_params*MANYPOINTS_COEFF_DIFF_PARAMS;
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
      if (i%NUM_PIECES_BETWEEN_REOPTIMIZATION_POINTS == 0)
      {
        Vector3d new_end_position = transform_to_end.corner(Eigen::TopRight,3,1);
        point_error_energy += MANYPOINTS_COEFF_POINT_EXPONENTIAL*pow(  ((new_end_position - opt_params_many_points.points.at(i/NUM_PIECES_BETWEEN_REOPTIMIZATION_POINTS-1)).norm()),2);
      }
	}

  fx += grav_energy*OPTIMIZATION_COEFF_GRAVITY + point_error_energy*MANYPOINTS_COEFF_POINT;
  //fx += point_error_energy*MANYPOINTS_COEFF_POINT;
  //std::cout << "point error: " << point_error_energy*MANYPOINTS_COEFF_POINT;

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
 
