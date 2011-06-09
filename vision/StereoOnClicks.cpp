#include "capture2.h"
#include "util2.h"
#include "../utils/clock.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <time.h>
#include <iostream>
#include "StereoOnClicks.h"


using namespace std;
using namespace cv;

StereoOnClicks::StereoOnClicks()
{
    Capture* c1 = new Capture(0, //id
            "cam1", // cam name
            650,  // gain
            "optimized", // optimized or measured
            107109);
            //"./calib-mar12/calib1-");
            //"./captures/extrins-4-293-"); // camera uid
    cout << "c1 created" << endl;

    Capture* c2 = new Capture(1, //id
            "cam2", // cam name
            650,  // gain
            "optimized", // optimized or measured
            107110);
            //"./calib-mar12/calib2-");
            //"./captures/extrins-4-293-"); // camera uid
    cout << "c2 created" << endl;

    Capture* c3 = new Capture(2, //id
            "cam3", // cam name
            650,  // gain
            "optimized", // optimized or measured
            107111);
            //"./calib-mar12/calib3-");
            //"./captures/extrins-4-293-"); // camera uid
    cout << "c3 created" << endl;  

    namedWindow("Cam1", CV_WINDOW_AUTOSIZE);
    namedWindow("Cam2", CV_WINDOW_AUTOSIZE);
    namedWindow("Cam3", CV_WINDOW_AUTOSIZE);
    cvSetMouseCallback("Cam1", mouseHandlerWrap1, this);
    cvSetMouseCallback("Cam2", mouseHandlerWrap2, this);
    cvSetMouseCallback("Cam3", mouseHandlerWrap3, this);

    c1->init("./calib_params/");
    c2->init("./calib_params/");
    c3->init("./calib_params/");

    
    cvWaitKey(100);							// segfaulted without this
    
    c1->AddOtherCameraInformation(*c2);
    c1->AddOtherCameraInformation(*c3);
    c2->AddOtherCameraInformation(*c1);
    c2->AddOtherCameraInformation(*c3);
    c3->AddOtherCameraInformation(*c1);
    c3->AddOtherCameraInformation(*c2);

    c1->startCapture();
    cout << "c1 started" << endl;
    c2->startCapture();
    cout << "c2 started" << endl;
    c3->startCapture();
    cout << "c3 started" << endl;
    cvWaitKey(100);

    captures[0] = c1;
    captures[1] = c2;
    captures[2] = c3;
    
    camNames[0] = "Cam1"; 
    camNames[1] = "Cam2"; 
    camNames[2] = "Cam3"; 

    markerColor = Scalar(0,0,255);
    mousePointerColorThread = Scalar(255,0,0);
    mousePointerColorRobot = Scalar(0,255,0);
    epiLineColor = Scalar(127, 0, 127);

}

StereoOnClicks::~StereoOnClicks()
{
    for (int i = 0; i < NUMCAMS; i++)
    {
        captures[i]->endCapture();
        delete captures[i];
    }
    /*c1->endCapture();
    c2->endCapture();
    c3->endCapture();
    delete c1;
    delete c2;
    delete c3;
    */
}

void StereoOnClicks::getNewPoint(Point3f& point, RequestType requestType)
{
    setInitialClickParams();


    updateImages(false, false);

    //Repeat this process until user is satisfied
    while(!updateImagesFromClicks(requestType) ) 
    {
        updateImages(false ,false);
    }
    
    
    //Calculate 3d Point from clicked coordinates
    Point p1coords(clickCoords[0]);
    Point p2coords(clickCoords[1]);
    Point p3coords(clickCoords[2]);
    
    get3dPoint(p1coords, p2coords, p3coords, point);
}

void StereoOnClicks::getNewPoint_NoImageUpdate(Point3f& point, RequestType requestType)
{
    setInitialClickParams();


    //Repeat this process until user is satisfied
    while(!updateImagesFromClicks(requestType) ) {}
    
    //Calculate 3d Point from clicked coordinates
    Point p1coords(clickCoords[0]);
    Point p2coords(clickCoords[1]);
    Point p3coords(clickCoords[2]);
    
    get3dPoint(p1coords, p2coords, p3coords, point);
}






void StereoOnClicks::updateImages(bool blockingUpdate, bool displayHere)
{

    if (blockingUpdate == false)
    {

        int numCamsDone = 0;
        bool thisCamDone[3] = {false, false, false};
        while(numCamsDone < NUMCAMS)
        {
            for (int i = 0; i < NUMCAMS; i++)
            {
                if (!thisCamDone[i])
                {
                    if (captures[i]->waitForFrameUndistorted())
                    {
                        thisCamDone[i] = true;
                        numCamsDone++; 
                        frames[i] = captures[i]->currentFrame();
                    }
                }
                       
            }
        }
    } else {
        
        for (int i = 0; i < NUMCAMS; i++)
        {
            captures[i]->startFrameGrab();
        }
        

        for (int i = 0; i < NUMCAMS; i++)
        {
            captures[i]->grabFrameAlreadyQueuedUndistorted();
            frames[i] = captures[i]->currentFrame();
        }

    }
    
    if (displayHere)
    {
        for (int i=0; i <NUMCAMS; i++)
            imshow(camNames[i], frames[i]);

        waitKey(10);
    }
}


void StereoOnClicks::setInitialClickParams()
{
    /****************************************************
    //  Set Initial Parameters
    *****************************************************/
    cout << "Click on on all three images and press enter when satisfied." << endl; 
    int i;

    for (i = 0; i < NUMCAMS; i++)
    {
        mouseCoords[i] = Point(1,1);
        zoomCenters[i] = Point(XWIDTH/2, YWIDTH/2);
        zoomAmounts[i] = 1.f;
        clickedOnCamera[i] = false;
    }
        
    mouseClickActive = false;

}


bool StereoOnClicks::updateImagesFromClicks(RequestType requestType)
{
    int i;

    for (i = 0; i < NUMCAMS; i++)
    {
        //clone images to add display features
        framesToShow[i] = frames[i].clone();                             

        //draw epipolar lines
        for (int j = 0; j < NUMCAMS; j++)
        {
            if (i == j)
                continue;
            
            if (clickedOnCamera[j])
            {
                line(framesToShow[i], epipolarInfo[i][j].pt1, epipolarInfo[i][j].pt2, epiLineColor);
            }
            
        }

        //add crosshair for location of mouse pointer
        if (requestType == Slave1Request || requestType == Slave2Request)
        {
            drawCrosshair(framesToShow[i],mouseCoords[i], 2000, mousePointerColorRobot); 
        }else if (requestType == Thread1Request || requestType == Thread2Request)
        {
            drawCrosshair(framesToShow[i],mouseCoords[i], 2000, mousePointerColorThread); 
        }
            
        //add crosshair for location of last click
        if (clickedOnCamera[i])
            drawCrosshair(framesToShow[i], clickCoords[i], 10, markerColor); 

        //resize matrix - find zoomed in region of interest, then blow it up 
        resize(Mat(framesToShow[i].clone(), zoomRect(zoomCenters[i], zoomAmounts[i], frames[i].cols, frames[i].rows))
                , framesToShow[i], Size(frames[i].cols, frames[i].rows), zoomAmounts[i], zoomAmounts[i]);

        //display images
        imshow(camNames[i], framesToShow[i]);
    }

    //Process keyboard inputs
    bool finish = false; 
    char c = waitKey(10);
    switch (c)
    {
        case '\n':      //the 'enter' key                 
            finish = true;
            for (i = 0; i < NUMCAMS; i++)
            {
                finish = finish & clickedOnCamera[i];
            }
            break; 
        case ' ':
            for (i = 0; i < NUMCAMS; i++)
            {
                zoomCenters[i] = Point(XWIDTH/2, YWIDTH/2);
                zoomAmounts[i] = 1.f;
            }
            break;
        default:
            break;
    }

    //Check if we are done getting points
    return finish;
}

    


void StereoOnClicks::drawCrosshair(Mat& img, Point& pt, int length, const Scalar& color)
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


void StereoOnClicks::mouseHandlerWrap1(int event, int x, int y, int flags, void *param)
{
    StereoOnClicks* stereo = (StereoOnClicks*)param;
    stereo->mouseHandlerCam(event, x, y, flags, param, 0);
}

void StereoOnClicks::mouseHandlerWrap2(int event, int x, int y, int flags, void *param)
{
    StereoOnClicks* stereo = (StereoOnClicks*)param;
    stereo->mouseHandlerCam(event, x, y, flags, param, 1);
}

void StereoOnClicks::mouseHandlerWrap3(int event, int x, int y, int flags, void *param)
{
    StereoOnClicks* stereo = (StereoOnClicks*)param;
    stereo->mouseHandlerCam(event, x, y, flags, param, 2);
}

//todo: pass camcoords into param. 
void StereoOnClicks::mouseHandlerCam(int event, int x, int y, int flags, void *param, int camNum)
{
    Point pt(x,y);
    switch(event) {
        case CV_EVENT_LBUTTONDOWN:
            mouseClickActive = true;
            handleMouseLeftClick(pt, camNum);
            break;
        case CV_EVENT_MOUSEMOVE:
            handleMouseMovement(pt, camNum);
            break;
        case CV_EVENT_MBUTTONDOWN:
            mouseClickActive = true;
            handleMouseMiddleClick(pt, camNum);
            break; 
    }
    mouseClickActive = false;
}

Rect StereoOnClicks::zoomRect(Point& zoomCen, float zoom, int xwidth, int ywidth)
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

Point StereoOnClicks::unzoomedPoint(Point& pt, int xwidth, int ywidth, Point& zoomCen, float zoom)
{
    int x = pt.x/zoom + zoomCen.x - xwidth/(2*zoom); 
    x = min(max(x,0),xwidth);
    int y = pt.y/zoom + zoomCen.y - ywidth/(2*zoom); 
    y = min(max(y,0),ywidth);
    return Point(x,y);
                 
}

void StereoOnClicks::populateEpipolarLinePoints(int camWithPoint, Point& pointFromCam)
{
    Point2f newPoint(((float)pointFromCam.x), ((float)pointFromCam.y));
    populateEpipolarLinePoints(camWithPoint, newPoint);
}

void StereoOnClicks::populateEpipolarLinePoints(int camWithPoint, Point2f& pointFromCam)
{

    for (int i = 0; i < NUMCAMS; i++)
    {
        if (i == camWithPoint)
            continue;
        Mat lineParams = Mat(3,1,CV_32FC1);
         
        captures[i]->GetEpipolarLine(*captures[camWithPoint], pointFromCam, lineParams);
        float a = lineParams.at<float>(0,0);
        float b = lineParams.at<float>(1,0);
        float c = lineParams.at<float>(2,0);


        Point p1;
        Point p2;

        if (abs(a) > abs(b))
        {
            //line is more vertical
            p1.y = 0;
            p2.y = YWIDTH;
            p1.x = (int)(0.5 + (-c)/a);
            p2.x = (int)(0.5 + (-c-b*p2.y)/a);
        } else
        {
            //line is more horizontal
            p1.x = 0;
            p2.x = XWIDTH;
            p1.y = (int)(0.5 + (-c)/b);
            p2.y = (int)(0.5 + (-c-a*p2.x)/b);
        }

        epipolarInfo[i][camWithPoint].pt1 = p1;
        epipolarInfo[i][camWithPoint].pt2 = p2;
        epipolarInfo[i][camWithPoint].a = a;
        epipolarInfo[i][camWithPoint].b = b;
        epipolarInfo[i][camWithPoint].c = c;

        //if this wasn't clicked before, and >= 2 lines, automatically select point
        if (!clickedOnCamera[i])
        {
            int numClicked = 1;
            float a2, b2, c2;
            for (int j = 0; j < NUMCAMS; j++)
            {
                if (j!= camWithPoint && clickedOnCamera[j]) {
                    numClicked++;        
                    a2 = epipolarInfo[i][j].a; 
                    b2 = epipolarInfo[i][j].b; 
                    c2 = epipolarInfo[i][j].c; 
                }
            }

            if (numClicked >= 2)
            {
                float y = (a*c2 - a2*c )/(a2*b - a*b2);
                float x = (-c-b*y) / a;
                Point pNewPt( (int)(x+0.5), (int)(y+0.5));
                if (pNewPt.x >= 0 && pNewPt.x < XWIDTH && pNewPt.y >=0 && pNewPt.y < YWIDTH)
                {
                    clickCoords[i].x = pNewPt.x;
                    clickCoords[i].y = pNewPt.y;
                    clickedOnCamera[i] = true;
                    populateEpipolarLinePoints(i, pNewPt);
                }
                    //handleMouseLeftClick(pNewPt, i);    
            }

        }

    }
               
}






///////////////////////////////////////////////////////////////////////////////////////////
/******************************* Least squares utilities *********************************/
///////////////////////////////////////////////////////////////////////////////////////////

void StereoOnClicks::get3dPoint(const Point& p1, const Point& p2, const Point& p3, Point3f& point)
{
    Mat ray1(3,1,CV_32FC1);
    Mat ray2(3,1,CV_32FC1);
    Mat ray3(3,1,CV_32FC1);

    captures[0]->getWorldRay(p1, ray1);
    captures[1]->getWorldRay(p2, ray2);
    captures[2]->getWorldRay(p3, ray3);

    Mat matForPoint = Mat(6,1,CV_32FC1);

    LeastSquares3Cam(ray1, ray2, ray3, captures[0]->cameraPosition(), captures[1]->cameraPosition(), captures[2]->cameraPosition(), matForPoint); 

    point.x = matForPoint.at<float>(0,0);
    point.y = matForPoint.at<float>(1,0);
    point.z = matForPoint.at<float>(2,0);
}



///////////////////////////////////////////////////////////////////////////////////////////
/******************************* Mouse Event Handlers ************************************/
///////////////////////////////////////////////////////////////////////////////////////////
void StereoOnClicks::handleMouseLeftClick(Point pt, int camNum)
{
    Point ptUnzoomed = unzoomedPoint(pt, frames[camNum].cols, frames[camNum].rows, zoomCenters[camNum], zoomAmounts[camNum]);
    clickCoords[camNum].x = ptUnzoomed.x;
    clickCoords[camNum].y = ptUnzoomed.y;
    clickedOnCamera[camNum] = true;
    populateEpipolarLinePoints(camNum, ptUnzoomed);
}

void StereoOnClicks::handleMouseMiddleClick(Point pt, int camNum)
{
    Point ptUnzoomed = unzoomedPoint(pt, frames[camNum].cols, frames[camNum].rows, zoomCenters[camNum], zoomAmounts[camNum]);
    zoomAmounts[camNum] *= 2;
    zoomCenters[camNum].x = min(max(ptUnzoomed.x, (int) (XWIDTH/(2*zoomAmounts[camNum]))), (int)(XWIDTH - XWIDTH/(2*zoomAmounts[camNum])));
    zoomCenters[camNum].y = min(max(ptUnzoomed.y, (int) (YWIDTH/(2*zoomAmounts[camNum]))), (int)(YWIDTH - YWIDTH/(2*zoomAmounts[camNum])));
}

void StereoOnClicks::handleMouseMovement(Point pt, int camNum)
{
    Point ptUnzoomed = unzoomedPoint(pt, frames[camNum].cols, frames[camNum].rows, zoomCenters[camNum], zoomAmounts[camNum]);
    mouseCoords[camNum].x = ptUnzoomed.x;
    mouseCoords[camNum].y = ptUnzoomed.y;
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

