
#ifndef _StereoOnClicks2_h
#define _StereoOnClicks2_h

#include "capture2.h"
#include "util2.h"
#include "../utils/clock.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <time.h>
#include <iostream>
#include <string>

using namespace std;
using namespace cv;


#define XWIDTH 1280
#define YWIDTH 960

#define NUMCAMS 3

enum RequestType {
 Slave1Request, Slave2Request, Thread1Request, Thread2Request
};

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
    
    private:

        string camNames[NUMCAMS];
        
        Capture* captures[NUMCAMS];
        Mat frames[NUMCAMS];
    
        //information about points we are trying to find
        Point clickCoords[NUMCAMS][10];
        Point mouseCoords[NUMCAMS];
        bool clickedOnCamera[NUMCAMS][10];
        int pointCurrentlyClicking;
       
        Point zoomCenters[NUMCAMS];
        float zoomAmounts[NUMCAMS];

        Scalar markerColors[10];
        Scalar mousePointerColorRobot;
        Scalar mousePointerColorThread;
        Scalar epiLineColor;

        bool mouseClickActive;

        void populateEpipolarLinePoints(int camWithPoint, Point& pointFromCam);
        void populateEpipolarLinePoints(int camWithPoint, Point2f& pointFromCam);

        Rect zoomRect(Point& zoomCen, float zoom, int xwidth, int ywidth);
        Point unzoomedPoint(Point& pt, int xwidth, int ywidth, Point& zoomCen, float zoom);

        void get3dPoint(const Point& p1, const Point& p2, const Point& p3, Point3f& Point);
 
        void drawCrosshair(Mat& img, Point& pt, int length, const Scalar& color);
        void handleMouseLeftClick(Point pt, int camNum);
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

        EpipolarInfo epipolarInfo[NUMCAMS][NUMCAMS][10];


};

#endif
