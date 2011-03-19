#ifndef _threadpiece_vision_h
#define _threadpiece_vision_h

#include "../threadenergy/threadpiece_minenergy.h"
#include <Eigen/Geometry>
#include "ThreeCam.h"

#define SCORE_COEFF_VISION 5.0
#define SCORE_COEFF_ENERGY_CURVATURE 20.0
#define SCORE_COEFF_ENERGY_TORSION 100.0
//#define SCORE_COEFF_DISTANCE 0.1
#define SCORE_COEFF_DIF_PARAMS_CURVATURE 20.0
#define SCORE_COEFF_DIF_PARAMS_TORSION 200.0


class ThreadPiece_Vision : public ThreadPiece
{
	public:
		ThreadPiece_Vision();
		ThreadPiece_Vision(double curvature, double torsion, double length);
		ThreadPiece_Vision(double curvature, double torsion, double length, ThreadPiece_Vision* prev);
		ThreadPiece_Vision(ThreadPiece_Vision* toCopy, ThreadPiece_Vision* prev=NULL);
		~ThreadPiece_Vision();
		void removeLastBeforeDelete();


		void connectPrevSegment(ThreadPiece_Vision* prev);
    void setPrevTransform(Matrix4d& trans); 
		void getLastPoint(cv::Point3f& endPoint);
		void getFirstPoint(cv::Point3f& endPoint);
		void getLastPoint(Vector3d& endPoint);
		void getFirstPoint(Vector3d& endPoint);
		void getLastTan(Vector3d& tan);
		void getFirstTan(Vector3d& tan);
    void getTransformBefore(Matrix4d& transform);
		void setParams(double curvature, double torsion, double length);
		void setScore(double scoreFromVis);
		ThreadPiece_Vision* prev_segment(){return (ThreadPiece_Vision*)_last_segment;}
		ThreadPiece_Vision* next_segment(){return (ThreadPiece_Vision*)_next_segment;}
  

		//corresponding_pts _start_pt;
		double _score;
		
		Matrix4d _transform_after;

		int _numPointingToMe;
		int _numPieces;
};


bool operator <(const ThreadPiece_Vision& a, const ThreadPiece_Vision& b);
//bool operator < (const ThreadPiece_Vision* a, const ThreadPiece_Vision* b);
bool lessThanThreadPiecePointer (const ThreadPiece_Vision* a, const ThreadPiece_Vision* b);



#endif
