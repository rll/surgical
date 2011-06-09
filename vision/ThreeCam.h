#ifndef _ThreeCam_h
#define _ThreeCam_h


#include "capture2.h"
#include "util2.h"
#include "CannyOrient.h"
#include "StereoOnClicks2.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <iostream>
#include <string>
//#include "../threadenergy/globals_thread_param_estimation.h"


using namespace std;
using namespace cv;


#define XWIDTH 1280
#define YWIDTH 960

#define NUMCAMS 3

#define CLICK_DISPLAY_BASE "click image cam "
#define IMAGE_SAVE_BASE "./suture_sets/sutureblack_traj1/reproj/suture1_traj1_origParams_"


#define NYLON true
//#define PURPLE true
//#define BLACK true


struct corresponding_pts 
{
    Point2i pts2d[NUMCAMS];
    Point3f pt3d;
    double score;
};



bool operator <(const corresponding_pts& a, const corresponding_pts& b);


class ThreeCam
{

public:
    ThreeCam(Capture* cams[]);
    ~ThreeCam();
    void updateImages();
    void updateImagesNoUndistort();
    void updateImagesBlocking();
    void updateImagesBlockingNoUndistort();
    void updateTimestamps();
    void initializeCanny(float width[], float edge_sigma[], float blur_sigma[], double thresh1[], double thresh2[]);
    void convertToGrayscale();
    void filterCanny();

    Mat* frames(void) {return _frames;}
    Mat* frames_gray(void) {return _frames_gray;}
    Mat* cannyIms(void) {return _cannyIms;}
    Mat* cannyAngs(void) {return _cannyAngs;}
    Capture** captures(void) {return _captures;}
    double* timestamps(void) {return _timestamps;}

    void scoreCorrespondingPts(corresponding_pts& pts, bool Point3dAlreadySet = true);
    double scoreCorrespondingPts(const Point2i* pts2d, const Point3f& pt3d);

    void get3dPoint(const Point& p1, const Point& p2, const Point& p3, Point3f& Point);
    void get3dPoint(const Point2f& p1, const Point2f& p2, const Point2f& p3, Point3f& Point);
    void get3dPoint(const Point* pts2d, Point3f& Point);
    void get3dPoint(const Point2f* pts2d, Point3f& Point);
    void project3dPoint(const Point3f& Point3d, Point2i* points2d);
    void project3dPoint(const Point3f& Point3d, Point2f* points2d);

    void initializeOnClicks();
    void getClickedPoints(Point2i* coords);
    void getClickedPoints(Point3f& coords);
    void saveImages(bool increment_ind = true, char* name = IMAGE_SAVE_BASE);
    void saveImages(char* image_save_base, int im_num);
    void saveCanny();

    void setImageNumber(int imNum);


private:
    string _camNames[NUMCAMS];

    Capture* _captures[NUMCAMS];
    CannyOrient* _cannyFilters[NUMCAMS];

    Mat _frames[NUMCAMS];
    Mat _frames_gray[NUMCAMS];
    double _timestamps[NUMCAMS];

    Mat _cannyIms[NUMCAMS];
    Mat _cannyAngs[NUMCAMS];
    double _canny_Thresh1[NUMCAMS];
    double _canny_Thresh2[NUMCAMS];

    StereoOnClicks* stereoOnClicks;
    int _saved_im_ind;
    int _canny_saved_im_ind;

};


#endif
