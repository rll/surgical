#ifndef _EndEffector_h
#define _EndEffector_h

#include "EnvObject.h"

#define MAX_DISPLACEMENT 0.2 //THREAD_RADIUS
#define MAX_ANGLE_CHANGE 0.0036667160503703967 //arcsin(MAX_DISPLACEMENT/end_effector_length)

class ThreadConstrained;
class Needle;

class EndEffector : public EnvObject
{
	friend class World;
	friend class Cursor;

	protected:
		ThreadConstrained* thread;		// The thread this end effector is holding. NULL if it isn't holding a thread.
		int constraint;								// The vertex number of the constraint the end effector is holding. -1 if it isn't holding the thread.
		int constraint_ind;						// constained_vertices_nums[constrained_ind] is the vertex number of the constraint the end effector is holding. -1 if it isn't holding the thread.
		Needle* needle;								// The needle this end effector is holding. NULL if it isn't holding a needle.
		World* world;
		bool open;
		vector<Intersection_Object*> i_objs;
		
		//backup
		int backup_constraint;
		int backup_thread_ind;				// -1 if the end effector is not attached to the thread
		bool backup_open;
		
		void updateIntersectionObjects();

		//need to be backed up
		// position
		// rotation
		// constraint (constraint_ind doesn't need to be backup because it can be recomputed from constraint)
		// thread_ind (backup_thread_ind = world->threadIndex(thread)) This is equivalent to backing up the thread pointer
		// open
		
		//need to be restored
		//position
		//rotation
		//constraint
		//constraint_ind (updateConstraintIndex())
		//thread (thread = world->threadAtIndex(backup_thread_ind);
		//open
	
	public:		
		EndEffector(const Vector3d& pos, const Matrix3d& rot, World* w, ThreadConstrained* t = NULL, int constrained_vertex_num = -1);
		EndEffector(const EndEffector& rhs, World* w);
		~EndEffector();
		
		//saving and loading from and to file
		void writeToFile(ofstream& file);
		EndEffector(ifstream& file, World* w);
		
		void setTransform(const Vector3d& pos, const Matrix3d& rot, bool limit_displacement = false, double max_displacement = MAX_DISPLACEMENT, double max_angle_change = MAX_ANGLE_CHANGE);
		void updateTransformFromAttachment();
		
		void draw();
		void drawDebug();
		
		void setOpen() { open = true; }
		void setClose() { open = false; }
		bool isOpen() { return open; }
		void updateConstraint();
		void updateConstraintIndex();
		
		//thread attachment
		void attach(ThreadConstrained* t) { thread = t; }
		void dettachThread() { thread = NULL; }
		bool isThreadAttached() { return (thread!=NULL); }
		ThreadConstrained* getThread() { return thread; }
		//needle attachment
		void attach(Needle* n) { needle = n; }
		void dettachNeedle() { needle = NULL; }
		bool isNeedleAttached() { return (needle!=NULL); }
		Needle* getNeedle() { return needle; }
		
		//backup
		void backup();
		void restore();
		
		//collision
		bool capsuleIntersection(int capsule_ind, const Vector3d& start, const Vector3d& end, const double radius, vector<Intersection>& intersections);
  	double capsuleRepulsionEnergy(const Vector3d& start, const Vector3d& end, const double radius);
  	void capsuleRepulsionEnergyGradient(const Vector3d& start, const Vector3d& end, const double radius, Vector3d& gradient);
	
		static const double default_color0 = 0.7, default_color1 = 0.7, default_color2 = 0.7;
		static const double pieces = 3.0; //4.0;
		static const double h = 9.0/3.0; //9.0/4.0; // (end-start)/pieces
		static const double start = -3.0;
		static const double handle_r = 1.2;
		static const double short_handle_r = 1.6; // corresponds to the capsule where the cursor can get attached
		static const double end = 6.0; // pieces*h + start;
		static const double grab_offset = 0.0; //12.0;
};

#endif
