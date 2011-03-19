#include "CannyOrient.h"

#include <iostream>
#include <time.h>


using namespace std;


int main(int argc, char** argv)
{
  if (argc < 2) {
    cerr << "Usage: testCanny <image_name>" << endl;
    return 1;
  }

  char imagePath[256];
  sprintf(imagePath, "captures/%s", argv[1]);

  cvNamedWindow( "Original Image", CV_WINDOW_AUTOSIZE);
  IplImage* origImage = cvLoadImage(imagePath,0); 
  IplImage* cannyImage = cvCreateImage( cvGetSize(origImage),origImage->depth, 1);
	std::cout << "depth: " << origImage->depth << std::endl;
  Mat origMat(origImage);
  Mat cannyMat(cannyImage);
  Mat cannyAngs = Mat(origMat.size(), CV_32FC1);
    
  std::cout << "Looking for image in relative path: " << imagePath << std::endl;

  cvShowImage("Original Image", origImage);
  //cvCanny(origImage, cannyImage, 10, 100, 3);

 

  std::cout << "Press q button to exit" <<std::endl;

  clock_t time1 = clock();
  CannyOrient* canny = new CannyOrient(1.7, 0.8, 1.5);
  clock_t time2 = clock();
  canny->filter(origMat, cannyMat, cannyAngs, 100.0, 850.0);
  clock_t time3 = clock();

  int diff1 = (time2-time1)/(CLOCKS_PER_SEC / 1000);
  int diff2 = (time3-time1)/(CLOCKS_PER_SEC / 1000);

  printf("Time to make filters: %d   Filter Time: %d  \n", diff1, diff2);
  //IplImage cannyImage = cannyMat;

  cvNamedWindow("Canny Image" );
  cvShowImage("Canny Image", cannyImage);

  uchar* data = (uchar *)cannyImage->imageData;

  /*for (int i = 0; i < 1000000; i++)
  {
      printf("%d", data[i]);
      if (i%100 == 0)
          printf("\n");
  }*/


  while(1)
  {
    char c = cvWaitKey(10);
    if (c == 'q') {
       break;
    }
  }

  return 1;




}
