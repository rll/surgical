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
	sprintf(filename, "/media/ssd1/captures/%s_comments.bin", argv[1]);
	commentFile = fopen(filename, "w+");
	cout << "enter comments, separated by newlines. Enter 'quit' to exit:" << endl ;

	double initTime = GetClock();
	while (1)
	{
		cin.getline(comment,512);
		double time = GetClock();

		if (strcmp(comment,"quit") == 0)
			break;

		int numChars = 0;
		while (comment[numChars] != '\0')
			numChars++;

		fwrite(&time, sizeof(double), 1, commentFile);
		fwrite(&numChars, sizeof(int), 1, commentFile);
		fwrite(comment, 1, numChars, commentFile);
	}

	fclose(commentFile);





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
		std::cout << "comment: " << comment << "   at time: " << (time-initTime) << endl;
		
	}

}
