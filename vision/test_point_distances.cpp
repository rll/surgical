#include "capture2.h"
#include "util2.h"
#include "../utils/clock.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <time.h>
#include <iostream>
#include <string.h>
#include "StereoOnClicks.h"


using namespace std;
using namespace cv;


int main(int argc, char** argv)
{
    StereoOnClicks cams = StereoOnClicks();

    Point3f point1, point2;
    while(1)
    {
        cams.getNewPoint(point1);
        cout << "point1: " << point1 << endl;
        cams.getNewPoint(point2);
        cout << "point2: " << point2 << endl;
        cout << "norm: " << norm(point2-point1) << endl;

    }
}
