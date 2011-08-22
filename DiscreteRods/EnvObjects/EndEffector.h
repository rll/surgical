#ifndef _EndEffector_h
#define _EndEffector_h

#include "EnvObject.h"
#include "Cursor.h"

class Cursor;

class ThreadConstrained;

class EndEffector : public EnvObject
{
	//protected:	
	public:	
		vector<Intersection_Object*> i_objs;
		float degrees;				// The opening angle of the end of the end effector.
	
	public:
		int constraint;							// The vertex number of the constraint the end effector is holding. -1 if it isn't holding the thread.
		int constraint_ind;					// constained_vertices_nums[constrained_ind] is the vertex number of the constraint the end effector is holding. -1 if it isn't holding the thread.
		ThreadConstrained* thread;		// The thread this end effector is holding. NULL if it isn't holding a thread.
		Cursor* attachment;
	
		EndEffector(const Vector3d& pos, const Matrix3d& rot);
		~EndEffector();
		
		void recomputeFromTransform(const Vector3d& pos, const Matrix3d& rot);
		void draw();
		bool capsuleIntersection(int capsule_ind, const Vector3d& start, const Vector3d& end, const double radius, vector<Intersection>& intersections);
  	double capsuleRepulsionEnergy(const Vector3d& start, const Vector3d& end, const double radius);
  	void capsuleRepulsionEnergyGradient(const Vector3d& start, const Vector3d& end, const double radius, Vector3d& gradient);
		
		void attach(Cursor* cursor);
		void dettach();
		void highlight() { color0 = color1 = color2 = 0.4; }
		void unhighlight() { color0 = color1 = color2 = 0.7; }
		void open() { degrees = 15.0; }
		void close() { degrees = 0.0; }
		void updateConstraint();
		void updateConstraintIndex();
		
		void attachThread(ThreadConstrained* t) { thread = t; }
		void dettachThread() { thread = NULL; }
		bool isThreadAttached() { return (thread!=NULL); }
		ThreadConstrained* getThread() { return thread; }
		
		const Vector3d& getPosition() const { return position; }
		const Matrix3d& getRotation() const { return rotation; }
	
	//protected:
		static const double pieces = 3.0; //4.0;
		static const double h = 9.0/3.0; //9.0/4.0; // (end-start)/pieces
		static const double start = -3.0;
		static const double handle_r = 1.2;
		static const double short_handle_r = 1.6; // corresponds to the capsule where the cursor can get attached
		static const double end = 6.0; // pieces*h + start;
		static const double grab_offset = 12.0;
};

#endif
