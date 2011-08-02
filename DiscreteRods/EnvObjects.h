#ifndef _EnvObjects_h
#define _EnvObjects_h

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
#include "thread_discrete.h"

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

class Cylinder;
class Cursor;
class EndEffector;

class Cylinder {
	public:
		Cylinder(const Vector3d& pos, const Matrix3d& rot, double h, double r, float c0, float c1, float c2);
		~Cylinder();
		void setTransform(const Vector3d& pos, const Matrix3d& rot);
		void setColor(float c0, float c1, float c2);
		void draw();
		Intersection_Object* cyl_obj;
		double height, radius;
		float color0, color1, color2;
};

class EndEffector {
	public:
		vector<Intersection_Object*> ee_objs;
		float color0, color1, color2;
		float degrees;				// The opening angle of the end of the end effector.
		int constraint;				// The vertex number of the constraint the end effector is holding. -1 if it isn't holding the thread.
		int constraint_ind;		// constained_vertices_nums[constrained_ind] is the vertex number of the constraint the end effector is holding. -1 if it isn't holding the thread.
		Cursor* attachment;
		
		EndEffector(const Vector3d& pos, const Matrix3d& rot);
		~EndEffector();
		void setTransform(const Vector3d& pos, const Matrix3d& rot);
		void attach(Cursor* cursor);
		void dettach();		
		void highlight() { color0 = color1 = color2 = 0.4; }
		void unhighlight() { color0 = color1 = color2 = 0.7; }
		void open() { degrees = 15.0; }
		void close() { degrees = 0.0; }
		void draw();
				
		static const double pieces = 4.0;
		static const double h = 9.0/4.0; // (end-start)/pieces
		static const double start = -3.0;
		static const double handle_r = 1.2;
		static const double end = 6.0; // pieces*h + start;
		static const double grab_offset = 12.0;
};

class Cursor {
	public:
		Cursor(const Vector3d& pos, const Matrix3d& rot);
		~Cursor();
		void setTransform(const Vector3d& pos, const Matrix3d& rot);
		void attach(EndEffector* ee);
		// Dettaches the cursor from the end effector it is holding. It has to be holding an end effector. If the end effector isn't holding the thread, it is removed from the environment
		void dettach();
		void draw();
		void openClose() { open = !open; }
		void saveLastOpen() { last_open = open; }
		void forceClose() { last_open = open = false; }
		bool isOpen() { return open; }
		bool justOpened() { return (open && !last_open); }
		bool justClosed() { return (!open && last_open); }
		bool isAttached() { return (end_eff!=NULL); }
		Intersection_Object* cursor_obj;
		EndEffector* end_eff;
		static const double height = 3, radius = 2;
		bool attach_dettach_attempt;
		bool open;
		bool last_open;
};

#endif //_EnvObjects_h
