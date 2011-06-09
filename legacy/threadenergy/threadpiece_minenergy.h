#ifndef _threadpiece_minenergy_h
#define _threadpiece_minenergy_h
using namespace std;

#include <Eigen/Core>
#include "thread_utils.h"
#include <vector>
#include <math.h>

// import most common Eigen types 
USING_PART_OF_NAMESPACE_EIGEN

class ThreadPiece
{
	public:
		ThreadPiece();
		ThreadPiece(double curvature, double torsion, double length);
    ThreadPiece(const ThreadPiece* toCopyPiece);
		//ThreadPiece(double curvature, double torsion, double length, ThreadPiece* prev, ThreadPiece* next);
		~ThreadPiece();

		void getPoints(MatrixXd& points, double currLength, double increment, int index, Matrix4d& currTransform);
		void getPoints(vector<Vector3d>& points, double currLength, double increment, int index, Matrix4d& currTransform);
    void getPointsAndParams(MatrixXd& points, double currLength, double increment, int index, Matrix4d& currTransform, std::vector<double>& curvatures, std::vector<double>& torsions);
    void getParams(double currLength, double increment, int index, std::vector<double>& curvatures, std::vector<double>& torsions);

		void addSegmentAfter(ThreadPiece* new_segment);
		void setParams(double curvature, double torsion, double length);
		
    void splitIntoSegments(double* curvature, double* torsion, int numSegments);
    void combineSegments(ThreadPiece* lastPiece);
    void replaceSegment(double curvature, double torsion, double length, ThreadPiece* next);

		Matrix4d _transform;
		double _curvature, _torsion, _length;

		ThreadPiece* _next_segment;
		ThreadPiece* _last_segment;

};

#endif
