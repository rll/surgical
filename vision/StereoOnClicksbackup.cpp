
#include "capture2.h"
#include "util2.h"
#include "../../utils/clock.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <time.h>
#include <iostream>
#include "StereoOnClicks.h"


using namespace std;
using namespace cv;

int main()
{

    cam1mousecoords = Point(1, 1);
    cam2mousecoords = Point(1, 1);
    cam3mousecoords = Point(1, 1);
    cam1clickcoords = Point(1, 1);
    cam2clickcoords = Point(1, 1);
    cam3clickcoords = Point(1, 1);
    mouseClickActive = false;

    c1 = new Capture(0, //id
            "cam1", // cam name
            650,  // gain
            "optimized", // optimized or measured
            107109);
            //"./calib-mar12/calib1-");
            //"./captures/stills1-"); // camera uid
    cout << "c1 created" << endl;

    c2 = new Capture(1, //id
            "cam2", // cam name
            650,  // gain
            "optimized", // optimized or measured
            107110);
            //"./calib-mar12/calib2-");
            //"./captures/stills2-"); // camera 
    cout << "c2 created" << endl;

    c3 = new Capture(2, //id
            "cam3", // cam name
            650,  // gain
            "optimized", // optimized or measured
            107111);
            //"./calib-mar12/calib3-");
            //"./captures/stills3-"); // camera uid
    cout << "c3 created" << endl;  

    namedWindow("Cam1", CV_WINDOW_AUTOSIZE);
    namedWindow("Cam2", CV_WINDOW_AUTOSIZE);
    namedWindow("Cam3", CV_WINDOW_AUTOSIZE);
    cvSetMouseCallback("Cam1", mouseHandlerCam1);
    cvSetMouseCallback("Cam2", mouseHandlerCam2);
    cvSetMouseCallback("Cam3", mouseHandlerCam3);

    c1->init("./captures/");
    c2->init("./captures/");
    c3->init("./captures/");

    cvWaitKey(10);							// segfaulted without this

    c1->startCapture();
    cout << "c1 started" << endl;
    c2->startCapture();
    cout << "c2 started" << endl;
    c3->startCapture();
    cout << "c3 started" << endl;
    cvWaitKey(10);


    /****************************************************
    //   Initialize Socket Connection with Controller
    *****************************************************/
    _incoming = UDPSocket();
    _outgoing = UDPSocket();

    if(!_incoming.create())
        std::cout << "error creating server socket" << std::endl;
    if(!_incoming.bind(VISION_IN_PORT))
        std::cout << "error binding to port " << VISION_IN_PORT << std::endl;

    if(!_outgoing.create())
        std::cout << "error creating server socket" << std::endl;
    if(!_outgoing.bind(CONTROLLER_IN_PORT))
        std::cout << "error binding to port " << CONTROLLER_IN_PORT << std::endl;

    _incoming.set_non_blocking(true);
    _incoming.set_timeout(0);             // using recv()

    _outgoing.set_non_blocking(true);
    _outgoing.set_timeout(0);
    _outgoing.setDestination(CONTROLLER_IP, CONTROLLER_IN_PORT);


    /*double x, y, z, a, b, c;
    while(1)
    {
    while (!Messaging::receive_message(_incoming, Messaging::CALIBRATION_REQUEST, &x, &y, &z, &a, &b, &c))
    {
        cout << "Still waiting, Stephen" << endl;
        usleep(100);
    }
    Messaging::send_message(_outgoing, Messaging::RECEIVED);

    cout << x << " " << y << " " << z << " " << a << " " << b << " " << c << endl;
    }*/
    

    /****************************************************
    //  Set constants
    *****************************************************/
    
    cout << "Click on thread on all three images and press enter when satisfied." << endl; 
    Scalar threadMarkerColor = Scalar(0,0,255);
    Scalar mousePointerColor = Scalar(255,0,0);

    frame1ZoomCenter = Point(1280/2, 960/2);
    frame2ZoomCenter = Point(1280/2, 960/2);
    frame3ZoomCenter = Point(1280/2, 960/2);
    frame1ZoomAmount = 1.f;
    frame2ZoomAmount = 1.f;
    frame3ZoomAmount = 1.f;

    while(1) {
        //while(!c1->waitForFrame() || !c2->waitForFrame() || !c3->waitForFrame()) {} // capture a frame asynch
       
        c1->startFrameGrab();
        c2->startFrameGrab();
        c3->startFrameGrab();

        //while(!c1->grabFrameAlreadyQueuedUndistorted() || !c2->grabFrameAlreadyQueuedUndistorted() || !c3->grabFrameAlreadyQueuedUndistorted()) {} // capture a frame asynch
        c1->grabFrameAlreadyQueuedUndistorted();
        c2->grabFrameAlreadyQueuedUndistorted();
        c3->grabFrameAlreadyQueuedUndistorted();

        frame1 = c1->currentFrame();
        frame2 = c2->currentFrame();
        frame3 = c3->currentFrame();
      
        bool nextIm = false;
        bool finish = false; 
        while (!nextIm)
        {     
            char c = waitKey(10);
            switch (c)
            {
                case 'c':
                    nextIm = true;
                    break; 
                case 27:
                    nextIm = true;
                    finish = true;
                    break;
                case ' ':
                    frame1ZoomCenter = Point(1280/2, 960/2);
                    frame2ZoomCenter = Point(1280/2, 960/2);
                    frame3ZoomCenter = Point(1280/2, 960/2);
                    frame1ZoomAmount = 1.f;
                    frame2ZoomAmount = 1.f;
                    frame3ZoomAmount = 1.f;
                    break;
                default:
                    nextIm = true;
                    break;
            }

            //clone images to add display features
            Mat frame1ToShow = frame1.clone();
            Mat frame2ToShow = frame2.clone();
            Mat frame3ToShow = frame3.clone();
            
            //add crosshair for location of mouse pointer
            drawCrosshair(frame1ToShow,cam1mousecoords, 2000, mousePointerColor); 
            drawCrosshair(frame2ToShow,cam2mousecoords, 2000, mousePointerColor); 
            drawCrosshair(frame3ToShow,cam3mousecoords, 2000, mousePointerColor); 

            //add crosshair for location of last click
            drawCrosshair(frame1ToShow, cam1clickcoords, 10, threadMarkerColor); 
            drawCrosshair(frame2ToShow, cam2clickcoords, 10, threadMarkerColor); 
            drawCrosshair(frame3ToShow, cam3clickcoords, 10, threadMarkerColor); 
           
            //resize matrix - find zoomed in region of interest, then blow it up 
            resize(Mat(frame1ToShow.clone(),zoomRect(frame1ZoomCenter, frame1ZoomAmount, frame1.cols, frame1.rows))
                        , frame1ToShow, Size(frame1.cols,frame1.rows), frame1ZoomAmount, frame1ZoomAmount);
            resize(Mat(frame2ToShow.clone(),zoomRect(frame2ZoomCenter, frame2ZoomAmount, frame2.cols, frame2.rows))
                        , frame2ToShow, Size(frame2.cols,frame2.rows), frame2ZoomAmount, frame2ZoomAmount);
            resize(Mat(frame3ToShow.clone(),zoomRect(frame3ZoomCenter, frame3ZoomAmount, frame3.cols, frame3.rows))
                        , frame3ToShow, Size(frame3.cols,frame3.rows), frame3ZoomAmount, frame3ZoomAmount);
            
            //display image
            imshow("Cam1", frame1ToShow);
            imshow("Cam2", frame2ToShow);
            imshow("Cam3", frame3ToShow);

        }

        if (finish)
            break;
            
    }
    cout << "Cam 1 coords: " << cam1clickcoords.x << " " << cam1clickcoords.y << endl;
    cout << "Cam 2 coords: " << cam2clickcoords.x << " " << cam2clickcoords.y << endl;
    cout << "Cam 3 coords: " << cam3clickcoords.x << " " << cam3clickcoords.y << endl;
    Point thread1coords(cam1clickcoords);
    Point thread2coords(cam2clickcoords);
    Point thread3coords(cam3clickcoords);

    /*thread1coords.x = 849;
    thread1coords.y = 752;
    thread2coords.x = 796;
    thread2coords.y = 900;
    thread3coords.x = 933;
    thread3coords.y = 728;
    Point3f threadPoint3d(0,0,0);
    get3dPoint(thread1coords, thread2coords, thread3coords, threadPoint3d);
    cout << "Thread Point 3d: " << threadPoint3d.x << " " << threadPoint3d.y << " " <<  threadPoint3d.z << endl;

    Mat mat3d = Mat(3,1,CV_32FC1);
    mat3d.at<float>(0,0) = threadPoint3d.x;
    mat3d.at<float>(1,0) = threadPoint3d.y;
    mat3d.at<float>(2,0) = threadPoint3d.z;


    c1->projectPointUndistorted(mat3d, cam1clickcoords);
    c2->projectPointUndistorted(mat3d, cam2clickcoords);
    c3->projectPointUndistorted(mat3d, cam3clickcoords);

    cout << "Cam 1 coords: " << cam1clickcoords.x << " " << cam1clickcoords.y << endl;
    cout << "Cam 2 coords: " << cam2clickcoords.x << " " << cam2clickcoords.y << endl;
    cout << "Cam 3 coords: " << cam3clickcoords.x << " " << cam3clickcoords.y << endl;
*/
    
    
    c1->endCapture();
    c2->endCapture();
    c3->endCapture();
    delete c1;
    delete c2;
    delete c3;
}

void drawCrosshair(Mat& img, Point& pt, int length, const Scalar& color)
{
    Point left(pt);
    left.x = max(0,pt.x-length);
    Point right(pt);
    right.x = min(img.cols-1, pt.x+length);
    Point up(pt);
    up.y = max(0,pt.y-length);
    Point down(pt);
    down.y = min(img.rows-1, pt.y+length);

    line(img, left, right, color);
    line(img, up, down, color);
}

//todo: pass camcoords into param. 
void mouseHandlerCam1(int event, int x, int y, int flags, void *param)
{
    Point toPass(x,y);
    Point newpt;
    newpt = unzoomedPoint(toPass, frame1.cols, frame1.rows, frame1ZoomCenter, frame1ZoomAmount);
    switch(event) {
        case CV_EVENT_LBUTTONDOWN:
            mouseClickActive = true;
            cam1clickcoords.x = newpt.x;
            cam1clickcoords.y = newpt.y;
            break;
        case CV_EVENT_MOUSEMOVE:

            cam1mousecoords.x = newpt.x;
            cam1mousecoords.y = newpt.y;
            break;
        case CV_EVENT_MBUTTONDOWN:
            mouseClickActive = true;
            frame1ZoomAmount *= 2;
            frame1ZoomCenter.x = min(max(newpt.x, (int) (XWIDTH/(2*frame1ZoomAmount))), (int)(XWIDTH - XWIDTH/(2*frame1ZoomAmount)));
            frame1ZoomCenter.y = min(max(newpt.y, (int) (YWIDTH/(2*frame1ZoomAmount))), (int)(YWIDTH - YWIDTH/(2*frame1ZoomAmount)));
            break; 
    }
    mouseClickActive = false;
}

void mouseHandlerCam2(int event, int x, int y, int flags, void *param)
{
    Point toPass(x,y);
    Point newpt;
    newpt = unzoomedPoint(toPass, frame2.cols, frame2.rows, frame2ZoomCenter, frame2ZoomAmount);
    switch(event) {
        case CV_EVENT_LBUTTONDOWN:
            mouseClickActive = true;
            cam2clickcoords.x = newpt.x;
            cam2clickcoords.y = newpt.y;
            break;
        case CV_EVENT_MOUSEMOVE:

            cam2mousecoords.x = newpt.x;
            cam2mousecoords.y = newpt.y;
            break;
        case CV_EVENT_MBUTTONDOWN:
            mouseClickActive = true;
            frame2ZoomAmount *= 2;
            frame2ZoomCenter.x = min(max(newpt.x, (int) (XWIDTH/(2*frame2ZoomAmount))), (int)(XWIDTH - XWIDTH/(2*frame2ZoomAmount)));
            frame2ZoomCenter.y = min(max(newpt.y, (int) (YWIDTH/(2*frame2ZoomAmount))), (int)(YWIDTH - YWIDTH/(2*frame2ZoomAmount)));
            break; 
    }
    mouseClickActive = false;
}

void mouseHandlerCam3(int event, int x, int y, int flags, void *param)
{
    Point toPass(x,y);
    Point newpt;
    newpt = unzoomedPoint(toPass, frame3.cols, frame3.rows, frame3ZoomCenter, frame3ZoomAmount);
    switch(event) {
        case CV_EVENT_LBUTTONDOWN:
            mouseClickActive = true;
            cam3clickcoords.x = newpt.x;
            cam3clickcoords.y = newpt.y;
            break;
        case CV_EVENT_MOUSEMOVE:

            cam3mousecoords.x = newpt.x;
            cam3mousecoords.y = newpt.y;
            break;
        case CV_EVENT_MBUTTONDOWN:
            mouseClickActive = true;
            frame3ZoomAmount *= 2;
            frame3ZoomCenter.x = min(max(newpt.x, (int) (XWIDTH/(2*frame3ZoomAmount))), (int)(XWIDTH - XWIDTH/(2*frame3ZoomAmount)));
            frame3ZoomCenter.y = min(max(newpt.y, (int) (YWIDTH/(2*frame3ZoomAmount))), (int)(YWIDTH - YWIDTH/(2*frame3ZoomAmount)));
            break; 
    }
    mouseClickActive = false;
}


Rect zoomRect(Point& zoomCen, float zoom, int xwidth, int ywidth)
{
    int x = zoomCen.x - xwidth/(2*zoom);
    x = max(x,0);
    int y = zoomCen.y - ywidth/(2*zoom);
    y = max(y,0);
    int wx = xwidth/(zoom);
    wx = min(wx, xwidth-x);
    int wy = ywidth/(zoom);
    wy = min(wy, ywidth-y);
    return Rect(x,y,wx,wy);
}

Point unzoomedPoint(Point& pt, int xwidth, int ywidth, Point& zoomCen, float zoom)
{
    int x = pt.x/zoom + zoomCen.x - xwidth/(2*zoom); 
    x = min(max(x,0),xwidth);
    int y = pt.y/zoom + zoomCen.y - ywidth/(2*zoom); 
    y = min(max(y,0),ywidth);
    return Point(x,y);
                 
}


///////////////////////////////////////////////////////////////////////////////////////////
/******************************* Least squares utilities *********************************/
///////////////////////////////////////////////////////////////////////////////////////////

void get3dPoint(const Point& p1, const Point& p2, const Point& p3, Point3f& point)
{
    Mat ray1(3,1,CV_32FC1);
    Mat ray2(3,1,CV_32FC1);
    Mat ray3(3,1,CV_32FC1);

    c1->getWorldRay(p1, ray1);
    c2->getWorldRay(p2, ray2);
    c3->getWorldRay(p3, ray3);

    Mat matForPoint = Mat(6,1,CV_32FC1);

    LeastSquares3Cam(ray1, ray2, ray3, c1->cameraPosition(), c2->cameraPosition(), c3->cameraPosition(), matForPoint); 

    point.x = matForPoint.at<float>(0,0);
    point.y = matForPoint.at<float>(1,0);
    point.z = matForPoint.at<float>(2,0);
}
/*

   TVector3 calculateWorldPoint3(TVector2 imagePoint1, TVector2 imagePoint2, TVector2 imagePoint3, Capture* cam1, Capture* cam2, Capture* cam3){
//std::cout << "worldray0 = "; imagePoint0.transpose().print(std::cout);
//std::cout << "worldray1 = "; imagePoint1.transpose().print(std::cout);
return calculateWorldPoint3(pixelToWorldRay(imagePoint1, cam1), pixelToWorldRay(imagePoint2, cam2), pixelToWorldRay(imagePoint3, cam3), getCameraPosition(cam1), getCameraPosition(cam2), getCameraPosition(cam3));
}

TVector3 calculateWorldPoint3(TVector3 worldRay1, TVector3 worldRay2, TVector3 worldRay3, TVector3 camPos1, TVector3 camPos2, TVector3 camPos3) {
//std::cout << "worldRay0 " << std::endl; worldRay0.print(std::cout);
//std::cout << "worldRay1 " << std::endl; worldRay1.print(std::cout);

TMatrix<9,6> leastSquaresInput;
/* In the form [1 0 0 x0 0 0;
 *              0 1 0 y0 0 0;
 *              0 0 1 z0 0 0;
 *              1 0 0 0 x1 0;
 *              0 1 0 0 y1 0;
 *              0 0 1 0 z1 0;
 *			  1 0 0 0 0 x2;
 *			  0 1 0 0 0 y2;
 *			  0 0 1 0 0 z2;]
 */ /*
       fillLeastSquaresMatrix(leastSquaresInput);
       leastSquaresInput[0][3] = worldRay1[0];
       leastSquaresInput[1][3] = worldRay1[1];
       leastSquaresInput[2][3] = worldRay1[2];
       leastSquaresInput[3][4] = worldRay2[0];
       leastSquaresInput[4][4] = worldRay2[1];
       leastSquaresInput[5][4] = worldRay2[2];
       leastSquaresInput[6][5] = worldRay3[0];
       leastSquaresInput[7][5] = worldRay3[1];
       leastSquaresInput[8][5] = worldRay3[2];

       std::cout << "leastSquaresInput: " << std::endl; leastSquaresInput.print(std::cout);
       TMatrix<9,1> cameraWorldCoordinates;
       fillTargetVector(cameraWorldCoordinates, camPos1, camPos2, camPos3);
       std::cout << "cameraWorldCoordinates: " << std::endl; cameraWorldCoordinates.print(std::cout);

//  TMatrix<6,1> estimatedParams = leastSquares(leastSquaresInput, cameraWorldCoordinates);
//  std::cout << "estimatedParams " << std::endl; estimatedParams.print(std::cout);
//  TVector3 worldPoint(estimatedParams[0], estimatedParams[1], estimatedParams[2]);
return NULL; //worldPoint;
}

/*
TMatrix<6,1> leastSquares(TMatrix<9,6> X, TMatrix<9,1> y) {
TMatrix<6,1> theta = (X.transpose() * X).inverse() * X.transpose() * y;
return theta;
}
     */
/*
   void fillLeastSquaresMatrix(TMatrix<9,6> &X) {
   X[0][0] = 1; X[1][1] = 1; X[2][2] = 1;
   X[3][0] = 1; X[4][1] = 1; X[5][2] = 1;
   X[6][0] = 1; X[7][1] = 1; X[8][2] = 1;

   }

   void fillTargetVector(TMatrix<9,1> &target, TMatrix<3,1> p0, TMatrix<3,1> p1, TMatrix<3,1> p2) {
   target[0] = p0[0]; target[1] = p0[1]; target[2] = p0[2];
   target[3] = p1[0]; target[4] = p1[1]; target[5] = p1[2];
   target[6] = p1[0]; target[7] = p1[1]; target[8] = p2[2];
   }

//////////////////////////////////////////////////////////////////////////////////////////////
/******************************** Transformation utilities **********************************/
//////////////////////////////////////////////////////////////////////////////////////////////
/*
// placeholder, stupid.
TVector3 pixelToWorldRay(TVector2 imagePoint, Capture *cam) {
return TVector3(imagePoint[0],imagePoint[1],0);	
}

// coordinates may not be in the right frame
TVector3 getCameraPosition(Capture * cam) {
CvMat* cameraPosition = cam->cameraPosition();
int x = cvGetReal1D(cameraPosition,0);
int y = cvGetReal1D(cameraPosition,1);
int z = cvGetReal1D(cameraPosition,2);
return TVector3(x,y,z);
}*/

