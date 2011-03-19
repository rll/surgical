
#ifndef _StereoOnClicks_withAuto_h
#define _StereoOnClicks_withAuto_h

#include "../../vision/capture2.h"
#include "../../vision/util2.h"
#include "../../../utils/clock.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <time.h>
#include <iostream>
#include <string>
#include <stack>

using namespace std;
using namespace cv;


#define XWIDTH 1280
#define YWIDTH 960

#define NUMCAMS 3

#define MAX_POINTS 69
#define NUM_PTS 68


#define DIST_TO_CLUST 2

#define START_NUM 26

enum RequestType {
 Slave1Request, Slave2Request, Thread1Request, Thread2Request
};

static char base_name[256] = "../../vision/captures/ribbon_dots";

struct pointCluster
{
	vector<Point2f> points;
	Point2f centerOfPoints;

};

struct point3d_from_clusters
{
	pointCluster* clusters[NUMCAMS];
	double score;

};

bool operator <(const point3d_from_clusters& a, const point3d_from_clusters& b);




class StereoOnClicks
{
    public:
        StereoOnClicks();
        StereoOnClicks(Capture* cams[], string names[]); //assumes 3 cams for now
        ~StereoOnClicks();
        void getNewPoints(Point3f* points, int numPointsToClick=1);
        void click2dPoints(int numPointsToClick=1);
        void getClickCoords(Point2i* points);
        void updateImages();
        void updateImagesBlocking();
				void blurImages();

				vector<vector<Point> >& getClickCoords(){return clickCoords;};
    
    private:
        string camNames[NUMCAMS];
        
        Capture* captures[NUMCAMS];
        Mat frames[NUMCAMS];
    
        //information about points we are trying to find
        //Point clickCoords[NUMCAMS][MAX_POINTS];
        vector<vector<Point> > clickCoords;
        Point mouseCoords[NUMCAMS];
        bool clickedOnCamera[NUMCAMS][MAX_POINTS];
        int pointCurrentlyClicking;
       
        Point zoomCenters[NUMCAMS];
        float zoomAmounts[NUMCAMS];

        Scalar markerColors[MAX_POINTS];
        Scalar textColors[MAX_POINTS];
        Scalar mousePointerColorRobot;
        Scalar mousePointerColorThread;
        Scalar epiLineColor;

        bool mouseClickActive;

        void populateEpipolarLinePoints(int camWithPoint, Point& pointFromCam);
        void populateEpipolarLinePoints(int camWithPoint, Point2f& pointFromCam);
				void try_to_intersct_epipolars();
				/*void checkOtherCamsForOrange(int camWithPoint, Point2f& pointFromCam);
				bool isOrange(vector<Mat>& planes_BGR, vector<Mat>& planes_HSV, int y, int x);
				void findNearbyOrange(vector<Mat>& planes_BGR, vector<Mat>& planes_HSV, int y, int x, pointCluster& clust);
*/

        Rect zoomRect(Point& zoomCen, float zoom, int xwidth, int ywidth);
        Point unzoomedPoint(Point& pt, int xwidth, int ywidth, Point& zoomCen, float zoom);
				Point2f convertToFloat(Point& pt){return Point2f((float)pt.x, (float)pt.y);};

        void get3dPoint(const Point& p1, const Point& p2, const Point& p3, Point3f& Point);
 
        void drawCrosshair(Mat& img, Point& pt, int length, const Scalar& color);
				void drawPointNumText(Mat& img, Point& pt, int pt_num, Scalar& color);
        void handleMouseLeftClick(Point pt, int camNum);
        void handleMouseRightClick(Point pt, int camNum);
        void handleMouseMiddleClick(Point pt, int camNum);
        void handleMouseMovement(Point pt, int camNum);
        
        // ui functions
        static void mouseHandlerWrap1(int event, int x, int y, int flags, void *param);
        static void mouseHandlerWrap2(int event, int x, int y, int flags, void *param);
        static void mouseHandlerWrap3(int event, int x, int y, int flags, void *param);
        void mouseHandlerCam(int event, int x, int y, int flags, void *param, int camNum);

        struct EpipolarInfo {
            Point pt1, pt2;
            float a,b,c;
        };

        EpipolarInfo epipolarInfo[NUMCAMS+5][NUMCAMS+5][MAX_POINTS+5];


};

#endif

