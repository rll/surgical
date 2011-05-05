
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
#include <map>
#include "thread_vision_discrete.h"


class CannyReader
{
	public:
		CannyReader() {};
		CannyReader(const char* inFile);
		bool readFromFile();
		std::map<int, vector<Line_Segment *>*>* precomputedCannySegments;
	private:
		char srcFile[256];
};

