
#include <stdlib.h>

#ifdef MAC
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#include <GL/gle.h>
#else
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/gle.h>
#endif

#include <iostream>
#include <fstream>
#include <vector>


class CannyWriter
{
	public:
		CannyWriter() {};
		CannyWriter(char* outFile);
		void writeToFile();

		//map<int, vector<line_segment *>*>* precomputedCannySegments;
	private:
		char dstFile[256];
};

