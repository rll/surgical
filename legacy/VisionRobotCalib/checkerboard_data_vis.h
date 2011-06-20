#ifndef _checkerboard_data_vis_h_
#define _checkerboard_data_vis_h_

#include "../vision/util2.h"
#include "../vision/capture2.h"
#include "../vision/ThreeCam.h"
#include "../../utils/clock.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <time.h>
#include <iostream>


#define CHECKERS_PER_ROW 14
#define CHECKERS_PER_COL 16
#define SIZE_EACH_CHECKER 6.6

USING_PART_OF_NAMESPACE_EIGEN


class Checkerboard
{
  public:
    Checkerboard();
    ~Checkerboard();
    bool findCheckerboard();
    void calculate3dPoints();
    void drawCheckerboard();
    void drawCheckerboard(Vector3d pts3d[]);
    void updateIms();
    void updateImsNoUndistort();
    void displayIms();
    void estimatePointsInRobotHand(const Vector3d& startPt, const Matrix3d& startRot, const Vector3d& offsetGuess, const double offsetAngGuess, vector<Vector3d>& estimatedPoints);
    void getEstimated3dPose(Mat& rotation, Mat& translation);

    void initCams();
    void initCamsFromFile(int slaveNum);


    vector<Point3f>& get_checkerLocations_3d(void){ return checkerLocations_3d;}

  //private:
    void vectorToMat(const vector<Point2f>& pts, Mat& mat);

    Capture* _captures[NUMCAMS];
    string _names[NUMCAMS];
    string _orig_display_names[NUMCAMS];

    Mat* _frames;
    ThreeCam* _cams;

    Size checkerSize;

    vector<Point2f> checkerLocations[NUMCAMS];
    vector<Point2f> checkerLocationsReversed[NUMCAMS];
    vector<Point3f> checkerLocations_3d;
    Mat checkerLocations_mat[NUMCAMS];
    Mat checkerLocationsReversed_mat[NUMCAMS];

};


#endif
