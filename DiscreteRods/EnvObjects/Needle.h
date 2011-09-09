#ifndef _Needle_h
#define _Needle_h

#define MIN_ANGLE 0.1
#define MIN_RADIUS 0.1

#include "EnvObject.h"

class ThreadConstrained;

class Needle : public EnvObject
{
	friend class World;

	public:
		Needle(const Vector3d& pos, const Matrix3d& rot, double degrees, double r, float c0, float c1, float c2, World* w, ThreadConstrained* t = NULL, int constrained_vertex_num = -1);
//		Needle(const Vector3d& pos, const Vector3d& start_pos, const Vector3d& end_pos, float c0, float c1, float c2, World* w, ThreadConstrained* t = NULL, int constrained_vertex_num = -1);
//		Needle(const Vector3d& pos, const Vector3d& center_start, double degrees, const Vector3d& axis, float c0, float c1, float c2, World* w, ThreadConstrained* t = NULL, int constrained_vertex_num = -1);
		Needle(const Needle& rhs, World* w);
		~Needle();
		
		//saving and loading from and to file
		void writeToFile(ofstream& file);
		Needle(ifstream& file, World* w);

		void setTransform(const Vector3d& pos, const Matrix3d& rot);
		void setTransformFromEndEffector(const Vector3d& ee_pos, const Matrix3d& ee_rot);
		void setTransformOffsetFromEndEffector(const Vector3d& ee_pos, const Matrix3d& ee_rot);
		void getEndEffectorTransform(Vector3d& ee_pos, Matrix3d& ee_rot);
		void updateTransformFromAttachment();
		Vector3d getStartPosition();
		Vector3d getEndPosition();
		Matrix3d getStartRotation();
		Matrix3d getEndRotation();
		double getAngle();
		double getCurvatureRadius();
		double getThicknessRadius();
		Vector3d getAxis();
		void rotateAboutAxis(double degrees);
		
		Vector3d nearestPosition(const Vector3d& pos);
		void setTransformFromEndEffectorBoxConstrained(const Vector3d& new_ee_pos, const Matrix3d& new_ee_rot);
		void updateTransformOffset(const Vector3d& pos, const Matrix3d& rot);
		
		void draw();
		void drawDebug();
		
		void updateConstraint();
		void updateConstraintIndex();
		
		//thread attachment
		void attach(ThreadConstrained* t) { thread = t; }
		void dettachThread() { thread = NULL; }
		bool isThreadAttached() { return (thread!=NULL); }
		ThreadConstrained* getThread() { return thread; }
		
		//backup
		void backup();
		void restore();
		
		//collision
		bool boxCollision(const Vector3d& b_center, const Vector3d& b_half_length);
		
		bool capsuleIntersection(int capsule_ind, const Vector3d& start, const Vector3d& end, const double radius, vector<Intersection>& intersections);
  	double capsuleRepulsionEnergy(const Vector3d& start, const Vector3d& end, const double radius);
  	void capsuleRepulsionEnergyGradient(const Vector3d& start, const Vector3d& end, const double radius, Vector3d& gradient);
	
	protected:
		double angle;
		double radius;
		ThreadConstrained* thread;
		int constraint;
		int constraint_ind;			
		World* world;
		vector<Intersection_Object*> i_objs;
		Vector3d position_offset;
		Matrix3d rotation_offset;
		
		//backup
		Vector3d backup_position_offset;
		Matrix3d backup_rotation_offset;
		int backup_constraint;
		int backup_thread_ind;				// -1 if the end effector is not attached to the thread

		//needs to be backup
		//position
		//rotation
		//position_offset
		//rotation_offset
		//constraint (constraint_ind doesn't need to be backup because it can be recomputed from constraint)
		//thread_ind (backup_thread_ind = world->threadIndex(thread)) This is equivalent to backing up the thread pointer
		
		//needs to be restored
		//position
		//rotation
		//position_offset
		//rotation_offset
		//constraint
		//constraint_ind (updateConstraintIndex())
		//thread (thread = world->threadAtIndex(backup_thread_ind);
};

#endif
