#ifndef _ThreadStereo_SpaceCurve_h
#define _ThreadStereo_SpaceCurve_h

#include "ThreeCam.h"
#include "../threadenergy/thread_utils.h"
#include "threadpiece_vision.h"
#include <string.h>
#include <map>
#include <stack>
#include <queue>




#define NUMHYPOTHS 100
#define NUMHYPOTHSINIT 100
#define NUMITERSWITHINITHYPOTHS 7

#define CENTER_IM_IND 0
#define CORRESPONDING_PTS_ERROR_THRESH 5.0
#define TANGENT_ERROR_THRESH 5.0
#define DIST_FOR_SCORE_CHECK 16

#define IM_VALUE_NOT_SEEN 255
#define IM_VALUE_CHECKED_FOR_BEGIN 200
#define IM_VALUE_USED_IN_THREAD 127

#define ROTATE_FIRST_AXIS_BY_ANG M_PI/15.0
#define LENGTH_THREAD_EACH_PIECE 5.0
#define THREAD_PIECE_PROJECTION_ERROR_THRESH 4.0

#define NUM_START_PTS_TO_INIT 1
#define NUM_TANS_TO_INIT 5

#define INIT_CURVATURE_MIN -2.0
#define INIT_CURVATURE_ADD 0.2
#define INIT_CURVATURE_MAX 2.0
#define INIT_TORSION_MIN -0.2
#define INIT_TORSION_ADD 0.05
#define INIT_TORSION_MAX 0.2

#define AUGMENT_CURVATURE_MIN -.2
#define AUGMENT_CURVATURE_ADD 0.04
#define AUGMENT_CURVATURE_MAX 0.2
#define AUGMENT_TORSION_MIN -0.1
#define AUGMENT_TORSION_ADD 0.04
#define AUGMENT_TORSION_MAX 0.1


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
  Matrix4d transform_to_tan;
  double score;

  tangent_and_score(const Vector3d& tanIn, const double scoreIn, const Matrix4d transformIn) :
    tan(tanIn), score(scoreIn), transform_to_tan(transformIn) {}

  tangent_and_score(const Vector3d& tanIn, const double scoreIn) :
    tan(tanIn), score(scoreIn) {}

  tangent_and_score(const tangent_and_score& in) :
    tan(in.tan), score(in.score), transform_to_tan(in.transform_to_tan) {}

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


/*#define NUM_POINT_AND_POINTER_THRESHOLD 50
struct point_and_pointer {
  Point2i point;
  int numPts;
  double distToEnd;
  point_and_pointer* next;

  point_and_pointer(const Point2i& pointIn, const int numPtsIn, const double distToEndIn):
    point(pointIn), numPts(numPtsIn), distToEnd(distToEndIn), next(NULL) {}

  point_and_pointer(const Point2i& pointIn, const int numPtsIn, const double distToEndIn, point_and_pointer* nextIn):
    point(pointIn), numPts(numPtsIn), distToEnd(distToEndIn), next(nextIn) {}
};

bool operator <(const point_and_pointer& a, const point_and_pointer& b);
*/




class ThreadStereo_SpaceCurve
{
  public:
    ThreadStereo_SpaceCurve();
    ~ThreadStereo_SpaceCurve();

    void getThreadPoints(vector<glline_draw_params>& gl_draw_params);
    void processHypothesesFromInit();

  //private:
    Capture* _captures[NUMCAMS];
    string _names[NUMCAMS];
    ThreeCam* _cams;

    Mat* _frames;
    Mat* _cannyIms;
    Mat* _cannyAngs;
    int rows[NUMCAMS];
    int cols[NUMCAMS];
    map<int,location_and_distance> _cannyDistanceScores[NUMCAMS];

    vector<ThreadPiece_Vision*>* currHypoths;


    void updateCanny();
    void initializeThreadSearch();
    bool findNextStartPoint(vector<corresponding_pts>& pts);
    bool findTangent(corresponding_pts& start, vector<tangent_and_score>& tangent, vector<tangent_and_score>& opposite_tangents);
    bool findCorrespondingPointsOtherIms(vector<corresponding_pts>& pts, Point2i initPt, int camWithPt);
    void addStartPoints(corresponding_pts& start, tangent_and_score& tanToStart, vector<ThreadPiece_Vision*>* hypothsToAddTo);
    void addStartPointsWithRotation(corresponding_pts& start, tangent_and_score& tanToStart);
    bool updateHypoths(vector<ThreadPiece_Vision*>* newHypoths);

    double scoreProjection3dPoint(const Point3f& pt3d);
    void precomputeDistanceScores();
    void removeUsedDistanceScores(ThreadPiece_Vision* last_piece);
    void addThreadPointsBetweenToQueue(Point2i* curr_pts, Point2i* last_pts, queue<Point2i>* pointsToRemove);
 //   bool connectPoints(int camNum, Point2i& start, Point2i& end, queue<Pointi> pts_between);
    int keyForHashMap(int camNum, int row, int col){return col+cols[camNum]*row;}

    int _begin_ptr_curr_row, _begin_ptr_curr_col;


    //for debugging
    vector<polyline_draw_params> display_for_debug[NUMCAMS];
    vector<glline_draw_params>* gl_display_for_debug;

    void gray_to_canny();

};




#endif
