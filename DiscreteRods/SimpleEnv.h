#include <stdlib.h>
#include <algorithm>

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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include "ThreadConstrained.h"
#include "EnvObjects.h"

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

class SimpleEndEffector {
	public:
		SimpleEndEffector(Vector3d pos, Matrix3d rot, double deg, float c0, float c1, float c2);
		void draw();
	
	private:
		Vector3d position;
		Matrix3d rotation;
		float degrees;				// The opening angle of the end of the end effector.
		float color0, color1, color2;
		
		static const double pieces = 4.0;
		static const double h = 9.0/4.0; // (end-start)/pieces
		static const double start = -3.0;
		static const double handle_r = 1.2;
		static const double end = 6.0; // pieces*h + start;
		static const double grab_offset = 12.0;
};

class SimpleSphere {
	public:
		SimpleSphere(Vector3d pos, float r, float c0, float c1, float c2);
		void draw();
		
	private:
		Vector3d position;
		float radius;
		float color0, color1, color2;
};

class SimpleEnv {
	public:
		SimpleEnv();
		
		void addObj(ThreadConstrained* thread);
		void addObj(Cylinder* cyl);
		void addObj(EndEffector* end_eff);
		void addObj(Cursor* cursor);
		
		void addObj(SimpleEndEffector end_eff);
		void addObj(SimpleSphere sphere);
		
		void clearObjs();
		void drawObjs();
	
	private:
		vector<ThreadConstrained*> threads;
		vector<Cylinder*> cyls;
		vector<EndEffector*> end_effs;
		vector<Cursor*> cursors;
		
		vector<SimpleEndEffector> simple_end_effs;
		vector<SimpleSphere> simple_spheres;
};
