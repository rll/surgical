#ifndef _ThreadStereo_Optimization_h
#define _ThreadStereo_Optimization_h

#include "../threadenergy/thread_utils.h"
#include "thread_vision.h"
//#include "threadpiece_vision.h"
#include <string.h>
#include <map>
#include <stack>
#include <queue>
#include "NLF.h"
#include "OptQNewton.h"
#include "NLP.h"
/*#include "GenSet.h"
#include "OptGSS.h"
#include "NLF.h"*/


#define NUM_THREAD_PIECES_EACH_OPT 8 //must be at most init . this is the number we optimize over each loop
#define NUM_THREAD_PIECES_FIRST_OPT 2 // we start with this, since too high at first diverges. so, keep adding 1 until init
#define NUM_THREAD_PIECES_INIT_OPT 9 //number of first params that also optimize over the euler angles
#define NUM_NEW_PIECES_EACH_OPT 1 //don't change this for now...
#define NUM_PIECES_WANTED 24

#define OPTIMIZATION_COEFF_ENERGY 2000.0 //2000 nylon, 5000 everything else
#define OPTIMIZATION_COEFF_DIFF_PARAMS 8000.0 //8000 nylon, 5000 everything else
#define OPTIMIZATION_COEFF_VISION 17.0 //20 nylon, 15 everything else
#define OPTIMIZATION_COEFF_DIST 0.0
#define OPTIMIZATION_COEFF_GRAVITY 0.002

/*
#define MANYPOINTS_COEFF_ENERGY 5000.0
#define MANYPOINTS_COEFF_DIFF_PARAMS 5000.0
#define MANYPOINTS_COEFF_POINT 75.0
#define MANYPOINTS_COEFF_POINT_EXPONENTIAL 1.0
*/
#define OPTIMIZATION_INIT_CURVATURE 0.01
#define OPTIMIZATION_INIT_TORSION 0.001
#define OPTIMIZATION_TOLERANCE_NORMAL  1e-7
#define OPTIMIZATION_TOLERANCE_INIT  1e-8
#define OPTIMIZATION_TOLERANCE_LAST  1e-8

#define NUM_PIECES_AFTER_REOPTIMIZATION 8
#define NUM_PIECES_BETWEEN_REOPTIMIZATION_POINTS 2

USING_PART_OF_NAMESPACE_EIGEN


enum ThreadOptimizingModes {INITIALIZING, CONTINUING, INIT_OPPOSITE};

class ThreadStereo
{
  public:
    ThreadStereo();
    ~ThreadStereo();

		Thread_Vision myThread;
		int num_pieces_wanted;
		Point2i _initPtCenterImSaved;
		Point3f _initPtSaved;



		bool optimizeThread(vector<glline_draw_params>& gl_draw_params, Point2i& initPtCenterIm);
    bool optimizeThread(vector<glline_draw_params>& gl_draw_params);
    void getThreadPoints(vector<Vector3d>& points);
    void getThreadPoints(MatrixXd& points);
		void getEndsAndTans(Vector3d pts[], Vector3d tans[]);
		double getLength();
		Thread* equivalentThreadMinEnergy(){return myThread.equivalentThreadMinEnergy();}

  
		//the optimizing to find thread
		bool initializeThread(corresponding_pts& start, tangent_and_score& tan, vector<ThreadPiece_Vision*>& currPieces);
		bool processHypothesesFromInit(corresponding_pts& start, tangent_and_score& tan);
    bool continueThreadUntilEnd(vector<ThreadPiece_Vision*>& currPieces, int& num_pieces_max);
		bool continueThreadOpposite(vector<ThreadPiece_Vision*>& oppositePieces, Matrix4d& transformToStart, double curvature_before, double torsion_before, int& num_pieces_max);
//		bool resampleAndReoptimize(const vector<ThreadPiece_Vision*>& orig_pieces, int num_constrains, vector<ThreadPiece_Vision*>& new_pieces);
//		bool resamplePoints(const vector<ThreadPiece_Vision*>& pieces, int numPieces, MatrixXd& resampled_points, double& length_per_piece, vector<double>& curvatures, vector<double>& torsions);

		//optimization functions
		void optimizeVision_GSS(int numParams, OPTPP::USERFCN0 eval, OPTPP::INITFCN init, NEWMAT::ColumnVector& solution, double fcnTol=OPTIMIZATION_TOLERANCE_NORMAL);
		void optimizeVision_FDNLF(int numParams, OPTPP::USERFCN0 eval, OPTPP::INITFCN init, NEWMAT::ColumnVector& solution, double fcnTol=OPTIMIZATION_TOLERANCE_NORMAL);


		//helpers for clicking
		void setInitPtCenterIm(Point2i& pt);
		void initializeOnClicks();
		void setInitPtCenterImFromClicks();
		void setInitPtFromClicks();
		void setInitPt(Point3f& init){_initPtSaved = init;};


		//for debugging
		void addThreadPointsToDebugImages(MatrixXd& points, Scalar& color){return myThread.addThreadPointsToDebugImages(points, color);}
		void display(){return myThread.display();}

};













struct optimization_info_vision
{

	Thread_Vision* threadStereo;

	Matrix4d transform_back;

	double curvature_before;
	double torsion_before;

	double length_per_segment;
	int num_segments;
	NEWMAT::ColumnVector* orig_params_each_piece; //each curvature and torsion, and lastly the angle to rotate around the x axis to start
  //double gravity_multipler;

};

static optimization_info_vision opt_params_vision; //will have to change this if we have multiple threads?

/*
struct optimization_info_many_points
{
	Matrix4d transform_back;

	vector<Vector3d> points;

	double length_per_segment;
	int num_segments;
	NEWMAT::ColumnVector* orig_params_each_piece; 
};

static optimization_info_many_points opt_params_many_points; //will have to change this if we have multiple threads?
*/




void energyEvalFunctionVision_init(int ndim, NEWMAT::ColumnVector& x);
void energyEvalFunctionVision(int ndim, const NEWMAT::ColumnVector& x, double& fx, int& result);
void energyEvalFunctionVisionFirst(int ndim, const NEWMAT::ColumnVector& x, double& fx, int& result);


//void energyEvalFunctionManyPoints_init(int ndim, NEWMAT::ColumnVector& x);
//void energyEvalFunctionManyPoints(int ndim, const NEWMAT::ColumnVector& x, double& fx, int& result);




#endif
