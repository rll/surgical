#ifndef _thread_vision_h
#define _thread_vision_h

#include "threadpiece_vision.h"
#include "ThreeCam.h"
#include "../threadenergy/thread_minenergy.h"
#include <string.h>
#include <map>
#include <stack>
#include <queue>

#define CENTER_IM_IND 0
#define CORRESPONDING_PTS_ERROR_THRESH 4.0
#define TANGENT_ERROR_THRESH 1000.0
#define DIST_FOR_SCORE_CHECK 75.0
#define SCORE_OUT_OF_VIEW 1.0

#define IM_VALUE_NOT_SEEN 255
#define IM_VALUE_CHECKED_FOR_BEGIN 200
#define IM_VALUE_USED_IN_THREAD 127

#define INIT_LENGTH_THREAD_EACH_PIECE 86.9/24.0
#define MAX_LENGTH_THREAD 87.0   //in mm
#define TOTAL_LENGTH_INIT -1.0

#define NUM_START_PTS_TO_INIT 1
#define NUM_TANS_TO_INIT 1 //currently this doesn't do anything, hard coded for 1...

#define DISPLAY_ORIG_BASE "orig cam"
#define DISPLAY_CANNY_BASE "canny cam"


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

class Thread_Vision
{
	public:
		Thread_Vision();
		~Thread_Vision();

		double length_thread_each_piece;
		double total_length;
		double this_time_length; //since we might not find the whole thread every time...
		Capture* _captures[NUMCAMS];
		string _names[NUMCAMS];
		string _orig_display_names[NUMCAMS];
		string _canny_display_names[NUMCAMS];

		Mat* _frames;
		Mat* _cannyIms;
		Mat* _cannyAngs;
		ThreeCam* _cams;

		int rows[NUMCAMS];
		int cols[NUMCAMS];

		vector<ThreadPiece_Vision*> threadPiecesCurr;
		vector<ThreadPiece_Vision*> threadPiecesReopt;

		//initialize the thread search
		void updateCanny();
    void initializeThreadSearch();
		bool findNextStartPoint(vector<corresponding_pts>& pts, Point3f& initPt);
    bool findNextStartPoint(vector<corresponding_pts>& pts, Point2i& initPtCenterIm);
    bool findTangent(corresponding_pts& start, vector<tangent_and_score>& tangent);
    bool findCorrespondingPointsOtherIms(vector<corresponding_pts>& pts, Point2i initPt, int camWithPt);

		//during optimizations
    double scoreProjection3dPoint(const Point3f& pt3d, double* scores=NULL);
		double score2dPoint(const Point2f& pt, int camNum);
		bool isEndPiece(const Point3f pt);
		bool isEndPiece(const int camNum, const Point2i pt);
    void precomputeDistanceScores();
		int keyForHashMap(int camNum, int row, int col){return col+cols[camNum]*row;}
		map<int,location_and_distance> _cannyDistanceScores[NUMCAMS];

		//post optimization
		void setNextPointers(vector<ThreadPiece_Vision*> pieces);
		void getPoints(vector<Vector3d>& points);
		void getPoints(MatrixXd& points);
		Thread* equivalentThreadMinEnergy();

		//stereo on clicks
		void initializeOnClicks();
		void clickOnPoints(Point2i* clickPoints);
		void clickOnPoints(Point3f& clickPoint);

		//random helpers
    void gray_to_canny();
		void addThreadPointsToDebug();
		void addThreadPointsToDebugImages(MatrixXd& points, Scalar& color);
		void display();
    vector<polyline_draw_params> display_for_debug[NUMCAMS];
    vector<glline_draw_params>* gl_display_for_debug;
		


};



#endif
