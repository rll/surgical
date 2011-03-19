#include "../../utils/clock.h"
#include <time.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <sys/types.h> 
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>

using namespace std;


int main(int argc, char** argv)
{
	char comment[512];
	char filename[256];
	FILE* commentFile;
	sprintf(filename, "/media/hdd1/surgical/ucsc_2010_10_8/%s_comments.bin", argv[1]);



  std::cout.precision(20);
	//parse...for test
	commentFile = fopen(filename, "r");
	while (!feof(commentFile))
	{
		double time;
		int numChars;
		fread(&time, sizeof(double), 1, commentFile);
		fread(&numChars, sizeof(int), 1, commentFile);
		fread(comment, 1, numChars, commentFile);
		comment[numChars] = '\0';
		std::cout << "comment: " << comment << "   at time: " << (time) << endl;
		
	}

}
