
#ifndef _ThreadStereo_SpaceCurve_MinEnergy_h
#define _ThreadStereo_SpaceCurve_MinEnergy_h

#include "ThreeCam.h"
#include "../threadenergy/thread_utils.h"
#include "threadpiece_vision.h"
#include <string.h>
#include <map>
#include <stack>
#include <queue>
#include "NLF.h"
#include "OptQNewton.h"
#include "NLP.h"
//#include "mpi.h"

#define CENTER_IM_IND 0
#define CORRESPONDING_PTS_ERROR_THRESH 4.0
#define TANGENT_ERROR_THRESH 5.0
#define DIST_FOR_SCORE_CHECK 70
#define THREAD_PIECE_PROJECTION_ERROR_THRESH 4.0

#define IM_VALUE_NOT_SEEN 255
#define IM_VALUE_CHECKED_FOR_BEGIN 200
#define IM_VALUE_USED_IN_THREAD 127

#define INIT_LENGTH_THREAD_EACH_PIECE 3.0
#define MAX_LENGTH_THREAD 190.0
#define TOTAL_LENGTH_INIT -1.0


#define NUM_START_PTS_TO_INIT 1
#define NUM_TANS_TO_INIT 1 //currently this doesn't do anything, hard coded for 1...


#define NUM_THREAD_PIECES_EACH_OPT 8 //must be at most init . this is the number we optimize over each loop
#define NUM_THREAD_PIECES_FIRST_OPT 2 // we start with this, since too high at first diverges. so, keep adding 1 until init
#define NUM_THREAD_PIECES_INIT_OPT 8 //number of first params that also optimize over the euler angles
#define NUM_NEW_PIECES_EACH_OPT 1
#define NUM_PIECES_WANTED 32

#define OPTIMIZATION_COEFF_ENERGY 5000.0
#define OPTIMIZATION_COEFF_DIFF_PARAMS 5000.0
#define OPTIMIZATION_COEFF_VISION 10.0
#define OPTIMIZATION_COEFF_DIST 0.0
#define OPTIMIZATION_COEFF_GRAVITY 0.01

#define MANYPOINTS_COEFF_ENERGY 5000.0
#define MANYPOINTS_COEFF_DIFF_PARAMS 5000.0
#define MANYPOINTS_COEFF_POINT 75.0
#define MANYPOINTS_COEFF_POINT_EXPONENTIAL 1.0

#define OPTIMIZATION_INIT_CURVATURE 0.01
#define OPTIMIZATION_INIT_TORSION 0.001
#define OPTIMIZATION_TOLERANCE_NORMAL  1e-7
#define OPTIMIZATION_TOLERANCE_INIT  1e-8
#define OPTIMIZATION_TOLERANCE_LAST  1e-8

#define SCORE_OUT_OF_VIEW 0.1
#define NUM_PIECES_AFTER_REOPTIMIZATION 8
#define NUM_PIECES_BETWEEN_REOPTIMIZATION_POINTS 2




USING_PART_OF_NAMESPACE_EIGEN


struct polyline_draw_params {
  Point* pts;
  int size;
  Scalar color;
};

struct glline_draw_params {
  Point3f* vertices;
  int size;
  float color[3];
};

struct tangent_and_score
{
  Vector3d tan;
  double rot1;
	double rot2;
  double score;
//	Matrix3d trans;

  tangent_and_score(const Vector3d& tanIn, const double scoreIn, const double rot1In, const double rot2In) :
    tan(tanIn), score(scoreIn), rot1(rot1In), rot2(rot2In) {}

  tangent_and_score(const Vector3d& tanIn, const double scoreIn) :
    tan(tanIn), score(scoreIn) {}

  tangent_and_score(const tangent_and_score& in) :
    tan(in.tan), score(in.score), rot1(in.rot1), rot2(in.rot2)/*, trans(in.trans)*/{}

  tangent_and_score(){}

};

bool operator <(const tangent_and_score& a, const tangent_and_score& b);


struct location_and_distance {
  //row and col are the location of thread, dist is the distance
  int row;
  int col;
  int dist;
  location_and_distance* next;

  location_and_distance(const int rowIn, const int colIn, const int distIn) :
    row(rowIn), col(colIn), dist(distIn), next(NULL) {}

  location_and_distance() :
    next(NULL){}

  ~location_and_distance();
};

struct location_and_distance_for_queue {
  //rowCheck and colCheck are the row and col of the new location to add to map
  int rowCheck, colCheck;
  location_and_distance ld;

  location_and_distance_for_queue(const int rowIn, const int colIn, const location_and_distance ldIn) :
    rowCheck(rowIn), colCheck(colIn), ld(ldIn) {}

  location_and_distance_for_queue(){}
};


struct vision_return
{
	vector<ThreadPiece_Vision*> pieces;
	vector<ThreadPiece_Vision*> pieces_reopt;
	double total_length;
	double length_per_piece;
	Vector3d init_tan;
	Vector3d final_tan;
};

enum ThreadOptimizingModes {INITIALIZING, CONTINUING, INIT_OPPOSITE};





class ThreadStereo_SpaceCurve
{
  public:
    ThreadStereo_SpaceCurve();
    ~ThreadStereo_SpaceCurve();

    void getThreadPoints(vector<glline_draw_params>& gl_draw_params, Point2i& initPtCenterIm);

		double length_thread_each_piece;
		double total_length;
		int num_pieces_wanted;

  //private:
    Capture* _captures[NUMCAMS];
    string _names[NUMCAMS];

    Mat* _frames;
    Mat* _cannyIms;
    Mat* _cannyAngs;
		ThreeCam* _cams;

		int rows[NUMCAMS];
		int cols[NUMCAMS];
		int _begin_ptr_curr_row, _begin_ptr_curr_col;
    
		vector<ThreadPiece_Vision*> threadPiecesLast;


    void updateCanny();
    void initializeThreadSearch();
    bool findNextStartPoint(vector<corresponding_pts>& pts);
    bool findNextStartPoint(vector<corresponding_pts>& pts, Point2i& initPtCenterIm);
    bool findTangent(corresponding_pts& start, vector<tangent_and_score>& tangent);
    bool findCorrespondingPointsOtherIms(vector<corresponding_pts>& pts, Point2i initPt, int camWithPt);

		//the optimizing to find thread
		bool initializeThread(corresponding_pts& start, tangent_and_score& tan, vector<ThreadPiece_Vision*>& currPieces);
		bool processHypothesesFromInit(corresponding_pts& start, tangent_and_score& tan, vision_return& toReturn);
    bool continueThreadUntilEnd(vector<ThreadPiece_Vision*>& currPieces, int& num_pieces_max);
		bool continueThreadOpposite(vector<ThreadPiece_Vision*>& oppositePieces, Matrix4d& transformToStart, double curvature_before, double torsion_before, int& num_pieces_max);
		bool resampleAndReoptimize(const vector<ThreadPiece_Vision*>& orig_pieces, int num_constrains, vector<ThreadPiece_Vision*>& new_pieces);
		bool resamplePoints(const vector<ThreadPiece_Vision*>& pieces, int numPieces, MatrixXd& resampled_points, double& length_per_piece, vector<double>& curvatures, vector<double>& torsions);
		void setNextPointers(vector<ThreadPiece_Vision*> pieces);



 //   double scoreProjection3dPoint(const Point3f& pt3d);
    double scoreProjection3dPoint(const Point3f& pt3d, double* scores=NULL);
		bool isEndPiece(const Point3f pt);
		bool isEndPiece(const int camNum, const Point2i pt);
    void precomputeDistanceScores();
    void removeUsedDistanceScores(ThreadPiece_Vision* last_piece);
    void addThreadPointsBetweenToQueue(Point2i* curr_pts, Point2i* last_pts, queue<Point2i>* pointsToRemove);
		int keyForHashMap(int camNum, int row, int col){return col+cols[camNum]*row;}
		map<int,location_and_distance> _cannyDistanceScores[NUMCAMS];
    

		void optimizeVision_GSS(int numParams, OPTPP::USERFCN0 eval, OPTPP::INITFCN init, NEWMAT::ColumnVector& solution, double fcnTol=OPTIMIZATION_TOLERANCE_NORMAL);
		void optimizeVision_FDNLF(int numParams, OPTPP::USERFCN0 eval, OPTPP::INITFCN init, NEWMAT::ColumnVector& solution, double fcnTol=OPTIMIZATION_TOLERANCE_NORMAL);

    //for debugging
    vector<polyline_draw_params> display_for_debug[NUMCAMS];
    vector<glline_draw_params>* gl_display_for_debug;

    void gray_to_canny();

};


struct optimization_info_vision
{

	ThreadStereo_SpaceCurve* threadStereo;

	Matrix4d transform_back;

	double curvature_before;
	double torsion_before;

	double length_per_segment;
	int num_segments;
	NEWMAT::ColumnVector* orig_params_each_piece; //each curvature and torsion, and lastly the angle to rotate around the x axis to start
  //double gravity_multipler;

};

static optimization_info_vision opt_params_vision; //will have to change this if we have multiple threads?

struct optimization_info_many_points
{
	Matrix4d transform_back;

	vector<Vector3d> points;

	double length_per_segment;
	int num_segments;
	NEWMAT::ColumnVector* orig_params_each_piece; 
};

static optimization_info_many_points opt_params_many_points; //will have to change this if we have multiple threads?





void energyEvalFunctionVision_init(int ndim, NEWMAT::ColumnVector& x);
void energyEvalFunctionVision(int ndim, const NEWMAT::ColumnVector& x, double& fx, int& result);
void energyEvalFunctionVisionFirst(int ndim, const NEWMAT::ColumnVector& x, double& fx, int& result);


void energyEvalFunctionManyPoints_init(int ndim, NEWMAT::ColumnVector& x);
void energyEvalFunctionManyPoints(int ndim, const NEWMAT::ColumnVector& x, double& fx, int& result);


#endif
