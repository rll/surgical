#include "../../vision/capture2.h"
#include "../../vision/util2.h"
#include "../../../utils/clock.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <time.h>
#include <iostream>
#include <string.h>
#include "StereoOnClicks_withAuto.h"
#include "string.h"


using namespace std;
using namespace cv;

#define NUM_PTS 68

void writeParamsToFile(const char* filename, vector<vector<Point> >& clickCoords );

vector<vector<Point3f> > points;
char outfile_name[256];

int main(int argc, char** argv)
{

	//Initialize Cameras
	StereoOnClicks cams = StereoOnClicks();

	sprintf(outfile_name, "%s_points.txt", base_name);

	//std::ofstream out;
	//out.open(outfile_name,ios_base::trunc);

	while(1)
	{      

		cams.updateImages();
		Point3f points_to_get[NUM_PTS];
		cams.getNewPoints(points_to_get,NUM_PTS);
	
		vector<Point3f> toAdd;
		for (int i=0; i < NUM_PTS; i++)
		{
			toAdd.push_back(points_to_get[i]);
		}
		points.push_back(toAdd);

		vector<vector<Point> >& clickCoords = cams.getClickCoords();

		writeParamsToFile(outfile_name, clickCoords);
	}



}




void writeParamsToFile(const char* filename, vector<vector<Point> >& clickCoords )
{
	//Assumes a Mat of floats
	std::ofstream out;
	out.open(filename,ios_base::app);

	
	out << (points.size()+START_NUM-1) << "   ";
	for (int i=0; i < points.back().size(); i++)
	{
		out << points.back()[i] << "   ";
	}
	for (int camNum = 0; camNum < clickCoords.size(); camNum++)
	{
		for (int i=0; i < NUM_PTS; i++)
		{
			out << clickCoords[camNum][i] << "   ";
		}
	}
	out << "\n";
	out.close();

}

