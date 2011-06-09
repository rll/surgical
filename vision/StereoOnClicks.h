#ifndef _StereoOnClicks_h
#define _StereoOnClicks_h

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
        ~StereoOnClicks();
        void getNewPoint(Point3f& point, RequestType requestType = Thread1Request);
        void getNewPoint_NoImageUpdate(Point3f& point, RequestType requestType = Thread1Request);
        void updateImages(bool blockingUpdate = false, bool displayHere = true);
    
    private:

        /*Capture* c1;
        Capture* c2;
        Capture* c3;

        Mat frame1;
        Mat frame2;
        Mat frame3;

        Point cam1mousecoords;
        Point cam2mousecoords;
        Point cam3mousecoords;
        Point cam1clickcoords;
        Point cam2clickcoords;
        Point cam3clickcoords;
        bool mouseClickActive;

        Point thread1coords, thread2coords, thread3coords;
        Point gripper1coords, gripper2coords, gripper3coords;

        Point frame1ZoomCenter;
        Point frame2ZoomCenter;
        Point frame3ZoomCenter;
        float frame1ZoomAmount;
        float frame2ZoomAmount;
        float frame3ZoomAmount;

        Scalar markerColor;
        Scalar mousePointerColorRobot;
        Scalar mousePointerColorThread;

        bool* clickedOnCam;
*/
        string camNames[NUMCAMS];
        
        Capture* captures[NUMCAMS];
        Mat frames[NUMCAMS];
        Mat framesToShow[NUMCAMS];
        Point clickCoords[NUMCAMS];
        Point mouseCoords[NUMCAMS];

       
        Point zoomCenters[NUMCAMS];
        float zoomAmounts[NUMCAMS];

        bool clickedOnCamera[NUMCAMS];

        Scalar markerColor;
        Scalar mousePointerColorRobot;
        Scalar mousePointerColorThread;
        Scalar epiLineColor;

        bool mouseClickActive;

        void setInitialClickParams();
        bool updateImagesFromClicks(RequestType requestType);
        



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

        EpipolarInfo epipolarInfo[NUMCAMS][NUMCAMS];


};

#endif
