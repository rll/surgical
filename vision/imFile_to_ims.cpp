#include "util2.h"
#include "../../utils/clock.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <time.h>
#include <list>
#include <stdio.h>

#include <iostream>
#include <fstream>

#define COLS 1280
#define ROWS 960
#define DEPTH 3
#define NUMCAMS 3

using namespace std;


int main(int argc, char** argv)
{
	int bytesPerIm = COLS*ROWS*DEPTH;

	char filename[256];
  char timestampString[256];
  int startInd=atoi(argv[2]);
  FILE* byteFiles[NUMCAMS];
  FILE* timestampFiles[NUMCAMS];


  for (int camNum=0; camNum < NUMCAMS; camNum++)
  {
    if (camNum == 0)
      sprintf(filename, "/media/ssd1/captures/%s%d.bin", argv[1],(camNum+1));
    else if (camNum == 1)
      sprintf(filename, "captures/%s%d.bin", argv[1],(camNum+1));
    else
      sprintf(filename, "/media/hdd1/captures/%s%d.bin", argv[1],(camNum+1));
    
    byteFiles[camNum] = fopen(filename, "r");


    sprintf(filename, "/media/hdd1/captures/%s%d_times.txt", argv[1],(camNum+1));
    timestampFiles[camNum] = fopen(filename, "w");


    rewind(byteFiles[camNum]);
    fseek (byteFiles[camNum] , 0 , SEEK_END);
    int numIms = ftell(byteFiles[camNum])/(bytesPerIm + sizeof(double));
    std::cout << "num ims: " << numIms << std::endl;
    rewind(byteFiles[camNum]);

    Mat im(ROWS,COLS,CV_8UC3);
    for (int i=0; i < numIms; i++)
    {
      std::cout << i << std::endl;
      fread(im.data, 1, bytesPerIm, byteFiles[camNum]);
      sprintf(filename, "/media/hdd1/captures/%s%d-%d.tif", argv[1],(camNum+1),(i+startInd));
      imwrite(filename,im);

      double timestamp;
      fread(&timestamp, sizeof(double), 1, byteFiles[camNum]);
      //sprintf(timestampString, "%f\n", timestamp);
      //timestampFiles[camNum] << timestampString;
      fwrite(&timestamp, sizeof(double), 1, timestampFiles[camNum]);
    }
  }

  for (int camNum=0; camNum < NUMCAMS; camNum++)
  {
		fclose(byteFiles[camNum]);
    fclose(timestampFiles[camNum]);
	}

}
