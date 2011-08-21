#ifndef _Cursor_h
#define _Cursor_h

#include "EnvObject.h"
#include "EndEffector.h"

class EndEffector;

class Cursor : public EnvObject
{
	public:
		Cursor(const Vector3d& pos, const Matrix3d& rot);
		~Cursor();
		
		void recomputeFromTransform(const Vector3d& pos, const Matrix3d& rot);
		void draw();
		bool capsuleIntersection(int capsule_ind, const Vector3d& start, const Vector3d& end, const double radius, vector<Intersection>& intersections);
  	double capsuleRepulsionEnergy(const Vector3d& start, const Vector3d& end, const double radius);
  	void capsuleRepulsionEnergyGradient(const Vector3d& start, const Vector3d& end, const double radius, Vector3d& gradient);
		
		void attach(EndEffector* ee);
		// Dettaches the cursor from the end effector it is holding. It has to be holding an end effector. If the end effector isn't holding the thread, it is removed from the environment
		void dettach();
		void openClose() { open = !open; }
		void saveLastOpen() { last_open = open; }
		void forceClose() { last_open = open = false; }
		bool isOpen() { return open; }
		bool justOpened() { return (open && !last_open); }
		bool justClosed() { return (!open && last_open); }
		bool isAttached() { return (end_eff!=NULL); }
		
	//protected:
		Intersection_Object* i_obj;
		EndEffector* end_eff;
		double height, radius;
		bool attach_dettach_attempt;
		bool open;
		bool last_open;
};

#endif
