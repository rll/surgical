#include "ThreadStereo_Optimization.h"



ThreadStereo::ThreadStereo()
{
  num_pieces_wanted = NUM_PIECES_WANTED;
}


ThreadStereo::~ThreadStereo()
{
}


void ThreadStereo::getThreadPoints(vector<Vector3d>& points)
{
  //
  myThread.getPoints(points);
}

void ThreadStereo::getThreadPoints(MatrixXd& points)
{
  myThread.getPoints(points);
}


bool ThreadStereo::optimizeThread(vector<glline_draw_params>& gl_draw_params)
{
   gl_draw_params.resize(0);
  myThread.gl_display_for_debug = &gl_draw_params;

  std::cout << "initializing search from ims" << std::endl;

  myThread.initializeThreadSearch();

  std::cout << "searching from ims" << std::endl;

  bool done = false;
  int numCheck = 0;
  int numdisp = 0;
  vector<corresponding_pts> start_pts;
  vector<tangent_and_score> tangent_to_start;

  if (myThread.findNextStartPoint(start_pts, _initPtSaved))
  {
    std::cout << "found start" << std::endl;
    tangent_to_start.resize(0);
    if (myThread.findTangent(start_pts[0], tangent_to_start))
    {
      std::cout << "found tan" << std::endl;
      //check to see if we have looked for this thread before
      if (myThread.total_length == TOTAL_LENGTH_INIT)
      {
        myThread.total_length = MAX_LENGTH_THREAD;
        if (processHypothesesFromInit(start_pts[0], tangent_to_start[0]))
        {
          //thread found - since this is the first time, set length
          myThread.total_length = myThread.threadPiecesCurr.size()*myThread.length_thread_each_piece;
          myThread.length_thread_each_piece = myThread.total_length/(double)num_pieces_wanted;
        } else {
          //reset length
          myThread.total_length = TOTAL_LENGTH_INIT;
        }
      } else {
        if (processHypothesesFromInit(start_pts[0], tangent_to_start[0]))
        {
        }
      }

    }
  }
  
  myThread.addThreadPointsToDebug();

  return (myThread.threadPiecesCurr.size() > 5);


}
bool ThreadStereo::optimizeThread(vector<glline_draw_params>& gl_draw_params, Point2i& initPtCenterIm)
{
  setInitPtCenterIm(initPtCenterIm);
  return optimizeThread(gl_draw_params);
}


void ThreadStereo::getEndsAndTans(Vector3d pts[], Vector3d tans[])
{
  myThread.threadPiecesCurr.front()->getFirstPoint(pts[0]);
  myThread.threadPiecesCurr.back()->getLastPoint(pts[1]);
  myThread.threadPiecesCurr.front()->getFirstTan(tans[0]);
  myThread.threadPiecesCurr.back()->getLastTan(tans[1]);
}

double ThreadStereo::getLength()
{
  return myThread.total_length;
}



bool ThreadStereo::processHypothesesFromInit(corresponding_pts& start, tangent_and_score& tan)
{
  vector<ThreadPiece_Vision*> currPieces;
  int num_pieces_max = (int)(myThread.total_length/myThread.length_thread_each_piece);
  //std::cout << "total length: " << myThread.total_length << std::endl;
  //std::cout << "length each piece: " << myThread.length_thread_each_piece << std::endl;
  std::cout << "num pieces max: " << num_pieces_max << std::endl;

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
          done = true;
        }
        break;
      
      case (CONTINUING):
        continueThreadUntilEnd(currPieces, num_pieces_max);
        (myThread.threadPiecesCurr).resize(currPieces.size());
        for (int i=0; i < currPieces.size(); i++)
        {
          myThread.threadPiecesCurr[i] = currPieces[i];
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

  //resample, reoptimize
  //resampleAndReoptimize(toReturn.pieces, NUM_PIECES_AFTER_REOPTIMIZATION, toReturn.pieces_reopt);


  delete opt_params_vision.orig_params_each_piece;

  if (myThread.threadPiecesCurr.size() > NUM_THREAD_PIECES_FIRST_OPT)
  {
    myThread.setNextPointers(myThread.threadPiecesCurr);
    return true;
  } else {
    return false;
  }

  
}


bool ThreadStereo::initializeThread(corresponding_pts& start, tangent_and_score& tan, vector<ThreadPiece_Vision*>& currPieces)
{
  Point3f lastPoint;

  //set opt params
  opt_params_vision.length_per_segment = myThread.length_thread_each_piece;
  opt_params_vision.threadStereo = &myThread;

  opt_params_vision.transform_back = Matrix4d::Identity();
  Vector3d startPt((double)start.pt3d.x, (double)start.pt3d.y, (double)start.pt3d.z);
  opt_params_vision.transform_back.corner(Eigen::TopRight,3,1) = startPt;

  opt_params_vision.orig_params_each_piece = new NEWMAT::ColumnVector(4*NUM_THREAD_PIECES_INIT_OPT+6); 


  //initialize x_sol as if we just finished one iteration
  NEWMAT::ColumnVector x_sol;
  x_sol.ReSize(2*NUM_THREAD_PIECES_FIRST_OPT+3);
  for (int i=0; i < NUM_THREAD_PIECES_FIRST_OPT; i++)
  {
      x_sol.element(2*i) = OPTIMIZATION_INIT_CURVATURE;
      x_sol.element(2*i+1) = OPTIMIZATION_INIT_TORSION;
  }
  x_sol.element(2*NUM_THREAD_PIECES_FIRST_OPT    ) = tan.rot1;
  x_sol.element(2*NUM_THREAD_PIECES_FIRST_OPT + 1) = tan.rot2;
  x_sol.element(2*NUM_THREAD_PIECES_FIRST_OPT + 2) = 0.0;


  //initialize thread as if we just finished one iteration
  Matrix4d transform_at_start;
  transformFromEulerAngles(transform_at_start, x_sol(x_sol.nrows()-2), x_sol(x_sol.nrows()-1), x_sol(x_sol.nrows()), startPt);
  currPieces.resize(0);
  currPieces.push_back(new ThreadPiece_Vision(x_sol(1), x_sol(2), myThread.length_thread_each_piece));
  currPieces.back()->setPrevTransform(transform_at_start);
  currPieces.back()->_numPieces = 1;
  for (int i=1; i < NUM_THREAD_PIECES_FIRST_OPT; i++)
  {
    currPieces.push_back(new ThreadPiece_Vision(x_sol.element(2*i), x_sol.element(2*i+1), myThread.length_thread_each_piece, currPieces[i-1]));
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
      } else {
        opt_params_vision.orig_params_each_piece->element(2*i) = x_sol.element(2*(numPiecesThisOpt-1));
        opt_params_vision.orig_params_each_piece->element(2*i+1) = x_sol.element(2*(numPiecesThisOpt)-1);
      }
    }
    opt_params_vision.orig_params_each_piece->element(2*numPiecesThisOpt    ) = x_sol(x_sol.nrows()-2);
    opt_params_vision.orig_params_each_piece->element(2*numPiecesThisOpt + 1) = x_sol(x_sol.nrows()-1);
    opt_params_vision.orig_params_each_piece->element(2*numPiecesThisOpt + 2) = x_sol(x_sol.nrows());

    //optimize
#ifdef PDS
    if (numPiecesThisOpt == NUM_THREAD_PIECES_INIT_OPT)
      optimize_PDS(2*opt_params_vision.num_segments+3, energyEvalFunctionVisionFirst, energyEvalFunctionVision_init, x_sol, OPTIMIZATION_TOLERANCE_INIT);
    else
      optimize_PDS(2*opt_params_vision.num_segments+3, energyEvalFunctionVisionFirst, energyEvalFunctionVision_init, x_sol, OPTIMIZATION_TOLERANCE_NORMAL);
#else
    if (numPiecesThisOpt == NUM_THREAD_PIECES_INIT_OPT)
      optimize_GSS(2*opt_params_vision.num_segments+3, energyEvalFunctionVisionFirst, energyEvalFunctionVision_init, x_sol, OPTIMIZATION_TOLERANCE_INIT);
    else
      optimize_GSS(2*opt_params_vision.num_segments+3, energyEvalFunctionVisionFirst, energyEvalFunctionVision_init, x_sol, OPTIMIZATION_TOLERANCE_NORMAL);
#endif
    //reset thread params, see if we are done
    transformFromEulerAngles(transform_at_start, x_sol(x_sol.nrows()-2), x_sol(x_sol.nrows()-1), x_sol(x_sol.nrows()), startPt);
    currPieces[0]->setParams(x_sol(1), x_sol(2), myThread.length_thread_each_piece);
    currPieces[0]->setPrevTransform(transform_at_start);
    for (int i=1; i < numPiecesThisOpt; i++)
    {
      if (i < currPieces.size())
        currPieces[i]->setParams(x_sol.element(2*i), x_sol.element(2*i+1), myThread.length_thread_each_piece);
      else
        currPieces.push_back(new ThreadPiece_Vision(x_sol.element(2*i), x_sol.element(2*i+1), myThread.length_thread_each_piece, currPieces.back()));
    } 
    currPieces.back()->getLastPoint(lastPoint);
    if (myThread.isEndPiece(lastPoint))
    {
      //return false;
    }

  }

  return true;

}


bool ThreadStereo::continueThreadUntilEnd(vector<ThreadPiece_Vision*>& currPieces, int& num_pieces_max)
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
  while ((int)currPieces.size() < (int)num_pieces_max)
  {
 //   std::cout << "curr num pieces: " << num_pieces_curr << std::endl;
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
      opt_params_vision.orig_params_each_piece->element(2*i) = x_sol.element(2*NUM_THREAD_PIECES_EACH_OPT-2);
      opt_params_vision.orig_params_each_piece->element(2*i+1) = x_sol.element(2*NUM_THREAD_PIECES_EACH_OPT-1);

    }
    opt_params_vision.transform_back = currPieces[num_pieces_curr-NUM_THREAD_PIECES_EACH_OPT-1]->_transform_after;


    //optimize
#ifdef PDS
    if (currPieces.size() == num_pieces_max-1)
      optimize_PDS(2*opt_params_vision.num_segments, energyEvalFunctionVision, energyEvalFunctionVision_init, x_sol, OPTIMIZATION_TOLERANCE_LAST);
    else
      optimize_PDS(2*opt_params_vision.num_segments, energyEvalFunctionVision, energyEvalFunctionVision_init, x_sol, OPTIMIZATION_TOLERANCE_NORMAL);
#else
    if (currPieces.size() == num_pieces_max-1)
      optimize_GSS(2*opt_params_vision.num_segments, energyEvalFunctionVision, energyEvalFunctionVision_init, x_sol, OPTIMIZATION_TOLERANCE_LAST);
    else
      optimize_GSS(2*opt_params_vision.num_segments, energyEvalFunctionVision, energyEvalFunctionVision_init, x_sol, OPTIMIZATION_TOLERANCE_NORMAL);
#endif


    //set thread params
    for (int pieceNum=num_pieces_curr-NUM_THREAD_PIECES_EACH_OPT; pieceNum < num_pieces_curr; pieceNum++)
    {
      int pieceNum_x_sol = pieceNum-(num_pieces_curr-NUM_THREAD_PIECES_EACH_OPT);
      if (pieceNum < currPieces.size())
        currPieces[pieceNum]->setParams(x_sol.element(2*pieceNum_x_sol), x_sol.element(2*pieceNum_x_sol+1), myThread.length_thread_each_piece);
      else
        currPieces.push_back(new ThreadPiece_Vision(x_sol.element(2*pieceNum_x_sol), x_sol.element(2*pieceNum_x_sol+1), myThread.length_thread_each_piece, currPieces.back()));
    } 

    currPieces.back()->getLastPoint(lastPoint);
    if (myThread.isEndPiece(lastPoint))
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

/*
bool ThreadStereo::resampleAndReoptimize(const vector<ThreadPiece_Vision*>& orig_pieces, int num_constraints, vector<ThreadPiece_Vision*>& new_pieces)
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

  
//  std::cout << "diff params: " << std::endl;
//  for (int i=0; i < 2*num_pieces_reopt; i++)
//  {
//    std::cout << (x_sol.element(i) - opt_params_many_points.orig_params_each_piece->element(i))  << std::endl;
//  }

  std::cout << "done re-optimizing" << std::endl;
  delete opt_params_many_points.orig_params_each_piece;


  
//  new_pieces.resize(num_pieces_reopt);
//  new_pieces[0]->setParams(x_sol(1), x_sol(2), length_per_piece);
//  new_pieces[0]->setPrevTransform(opt_params_many_points.transform_back);
//  for (int i=1; i < num_pieces_reopt; i++)
//  {
//    new_pieces[i]->setParams(x_sol.element(2*i), x_sol.element(2*i+1), length_per_piece);
//  }
  
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

bool ThreadStereo::resamplePoints(const vector<ThreadPiece_Vision*>& pieces, int numPieces, MatrixXd& resampled_points, double& length_per_piece, vector<double>& curvatures, vector<double>& torsions)
{
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

*/

void ThreadStereo::setInitPtCenterIm(Point2i& pt)
{
  _initPtCenterImSaved = pt;
}

void ThreadStereo::initializeOnClicks()
{
  myThread.initializeOnClicks();
}

void ThreadStereo::setInitPtCenterImFromClicks()
{
  Point2i pts[NUMCAMS];
  myThread.clickOnPoints(pts);
  _initPtCenterImSaved = pts[CENTER_IM_IND];
}

void ThreadStereo::setInitPtFromClicks()
{
  myThread.clickOnPoints(_initPtSaved);
}














/*

void ThreadStereo::optimize_GSS(int numParams, OPTPP::USERFCN0 eval, OPTPP::INITFCN init, NEWMAT::ColumnVector& solution, double fcnTol)
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

}


void ThreadStereo::optimizeVision_FDNLF(int numParams, OPTPP::USERFCN0 eval, OPTPP::INITFCN init, NEWMAT::ColumnVector& solution, double fcnTol)
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

*/



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
  //double distEnergy  = 0.0;

	Matrix4d nextTransform;
  Vector3d firstPos = transform_to_end.corner(Eigen::TopRight,3,1);
	for (int i=1; i <= (ndim)/2; i++)
	{
			getTransform(x(2*i-1), x(2*i), opt_params_vision.length_per_segment, nextTransform);
			transform_to_end *= nextTransform;
      grav_energy += ((transform_to_end(2,3)-curr_z)/2.0 + curr_z)*opt_params_vision.length_per_segment;
      curr_z = transform_to_end(2,3);

      //dist
      //distEnergy += 1.0/pow(max(((lastPos- transform_to_end.corner(Eigen::TopRight,3,1)).norm()), 0.0000001),2);
      //distEnergy -= pow(max(((lastPos - transform_to_end.corner(Eigen::TopRight,3,1)).norm()), 0.0000001),2);
  
      //vision
      Point3f pointAtEnd = Point3f((float)transform_to_end(0,3), (float)transform_to_end(1,3), (float)transform_to_end(2,3));
      double dists[NUMCAMS];
      opt_params_vision.threadStereo->scoreProjection3dPoint(pointAtEnd, dists);
      for (int camNum=0; camNum < NUMCAMS; camNum++)
      {
        //std::cout << "dist: " << dists[camNum] << std::endl;
        //vision_energy +=  (dists[camNum] < (1.0/sqrt(2.0)) ? 0 : pow(dists[camNum],2));
        //vision_energy +=  (dists[camNum] < 1.0 ? 0 : pow(dists[camNum],2));
        if (dists[camNum] > sqrt(0.5))
          vision_energy += pow(dists[camNum]-sqrt(0.5),2);
        //vision_energy +=  pow(dists[camNum],2);
      }

	}   

  fx += grav_energy*OPTIMIZATION_COEFF_GRAVITY + vision_energy*OPTIMIZATION_COEFF_VISION;
  //fx += vision_energy*OPTIMIZATION_COEFF_VISION;
  //std::cout << "vision energy: " << vision_energy*OPTIMIZATION_COEFF_VISION << std::endl;
  //std::cout << "grav energy: " << grav_energy*OPTIMIZATION_COEFF_GRAVITY << std::endl;

  //distEnergy = OPTIMIZATION_COEFF_DIST*distEnergy;
  //fx +=  distEnergy;

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



/*
 
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



*/
