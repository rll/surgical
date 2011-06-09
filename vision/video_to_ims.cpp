#include "capture2.h"
#include "ThreeCam.h"
#include "util2.h"
#include "../utils/clock.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <time.h>
#include <list>


#include <iostream>


using namespace std;


int main(int argc, char** argv)
{
	char filename[256];
	int f = 1;
  sprintf(filename, "/media/ssd1/captures/%s.avi", argv[1]);
	VideoCapture vid;
	vid.open(filename);
	while (vid.grab())
	{
		std::cout << f << std::endl;
		Mat grabbed;
		vid.retrieve(grabbed);
		sprintf(filename, "/media/ssd1/captures/%s1-%d.png", argv[1],f);
		imwrite(filename, grabbed);
		f++;
	}

}
