#ifndef _ThreadConstrained_h
#define _ThreadConstrained_h


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

#include "threadutils_discrete.h"
#include "../utils/drawUtils.h"
#include "EnvObjects/World.h"

//CONTOUR STUFF
#define SCALE 1.0
#define CONTOUR(x,y) {					\
   double ax, ay, alen;					\
   contour[i][0] = SCALE * (x);				\
   contour[i][1] = SCALE * (y);				\
   if (i!=0) {						\
      ax = contour[i][0] - contour[i-1][0];		\
      ay = contour[i][1] - contour[i-1][1];		\
      alen = 1.0 / sqrt (ax*ax + ay*ay);		\
      ax *= alen;   ay *= alen;				\
      contour_norms [i-1][0] = ay;				\
      contour_norms [i-1][1] = -ax;				\
   }							\
   i++;							\
}

#define NUM_PTS_CONTOUR (25)

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

class Thread;

class ThreadConstrained {
	public:
		//ThreadConstrained();
		ThreadConstrained(int num_vertices_init, World* w);
		ThreadConstrained(vector<Vector3d>& vertices, vector<double>& twist_angles, vector<double>& rest_lengths, Matrix3d& start_rot, Matrix3d& end_rot, World* w);
		ThreadConstrained(vector<Vector3d>& vertices, vector<double>& twist_angles, vector<double>& rest_lengths, Matrix3d& start_rot, World* w);
		ThreadConstrained(vector<Vector3d>& vertices, vector<double>& twist_angles, Matrix3d& start_rot, Matrix3d& end_rot, World* w);
		ThreadConstrained(const ThreadConstrained& rhs, World* w);
    //ThreadConstrained& operator=(const ThreadConstrained& rhs);
		
		//need to be initialized in constructor
		//num_vertices
		//threads
		//zero_angle
		//rot_diff
		//rot_offset
		//constrained_vertices_nums
		//world
		//examine_mode (false)
		
		void writeToFile(ofstream& file);
		ThreadConstrained(ifstream& file, World* w);
		
		int numVertices();
		void checkNumVertices();
		void get_thread_data(vector<Vector3d> &absolute_points);
		void get_thread_data(vector<Vector3d> &absolute_points, vector<double> &absolute_twist_angles);
		void get_thread_data(vector<Vector3d> &absolute_points, vector<double> &absolute_twist_angles, vector<Matrix3d> &absolute_material_frames);
		void get_thread_data(vector<Vector3d> &absolute_points, vector<Matrix3d> &absolute_material_frames);
		// parameters have to be of the right size, i.e. threads.size()+1
		void getConstrainedTransforms(vector<Vector3d> &positions, vector<Matrix3d> &rotations);
		void setConstrainedTransforms(vector<Vector3d> positions, vector<Matrix3d> rotations);
		void updateRotationOffset(int constraint_ind, Matrix3d rotation);
		void getAllTransforms(vector<Vector3d> &positions, vector<Matrix3d> &rotations);
		void setAllTransforms(vector<Vector3d> positions, vector<Matrix3d> rotations);
		// parameters have to be of the right size.
		void getConstrainedNormals(vector<Vector3d> &normals);
		void getConstrainedVerticesNums(vector<int> &vertices_num);
		void getConstrainedVertices(vector<Vector3d> &constrained_vertices);
		void getFreeVerticesNums(vector<int> &vertices_nums);
		void getFreeVertices(vector<Vector3d> &free_vertices);
		void getOperableFreeVertices(vector<int> &free_vertices_num);
		void getOperableVertices(vector<int> &operable_vertices_num, vector<bool> &constrained_or_free);
		Vector3d start_pos();
		Vector3d end_pos();
		Matrix3d start_rot();
		Matrix3d end_rot();
		void set_coeffs_normalized(double bend_coeff, double twist_coeff, double grav_coeff);
		void set_coeffs_normalized(const Matrix2d& bend_coeff, double twist_coeff, double grav_coeff);
		void minimize_energy();
		void adapt_links();
		void updateConstraints (vector<Vector3d> poss, vector<Matrix3d> rots);
		void updateConstrainedTransform(int constraint_ind, const Vector3d& pos, const Matrix3d& rot);
		void getConstrainedTransform(int constraint_ind, Vector3d& pos, Matrix3d& rot);
		const Vector3d& positionAtConstraint(int constraint_ind) const;
		Matrix3d rotationAtConstraint(int constraint_ind);
		void applyMotionAtConstraints(vector<Vector3d> translations, vector<Matrix3d> rotations);
		void applyControl(const VectorXd& u);
		void getState(VectorXd& state);
		int addConstraint (int absolute_vertex_num);
		void removeConstraint (int absolute_vertex_num);
		// Returns the number of the vertex that is nearest to pos. The chosen vertex have to be a free operable vertex.
		int nearestVertex(Vector3d pos);
		Vector3d position(int absolute_vertex_num);
		Matrix3d rotation(int absolute_vertex_num);
		void draw(bool mode = false);
		void setWorld(World* w);
		void getThreads(vector<Thread*>& ths);

		//backup
		void backup();
		void restore();




	//private:
		int num_vertices;
		vector<Thread*> threads;
		double zero_angle;
		World* world;
		bool examine_mode;
		vector<Matrix3d> rot_diff;
		vector<Matrix3d> rot_offset;
    vector<int> constrained_vertices_nums;
    object_type type;
    
    //backup
		int backup_num_vertices;
		vector<Thread*> backup_threads;
		double backup_zero_angle;
		vector<Matrix3d> backup_rot_diff;
		vector<Matrix3d> backup_rot_offset;
		vector<int> backup_constrained_vertices_nums;
		
		//need to be backup
		//num_vertices
		//threads
		//zero_angle
		//rot_diff
		//rot_offset
		//constrained_vertices_nums

		//need to be restored
		//num_vertices
		//threads
		//zero_angle
		//rot_diff
		//rot_offset
		//constrained_vertices_nums
		//examine_mode (false)
        
    double contour[NUM_PTS_CONTOUR][2];
		double contour_norms[NUM_PTS_CONTOUR][2];
    void initContour();
		void intermediateRotation(Matrix3d &inter_rot, Matrix3d end_rot, Matrix3d start_rot);
		// Splits the thread threads[thread_num] into two threads, which are stored at threads[thread_num] and threads[thread_num+1].  Threads in threads that are stored after thread_num now have a new thread_num which is one unit more than before. The split is done at vertex vertex of thread[thread_num]
		void splitThread(int thread_num, int vertex_num);
		// Merges the threads threads[thread_num] and threads[thread_num+1] into one thread, which is stored at threads[thread_num]. Threads in threads that are stored after thread_num+1 now have a new thread_num which is one unit less than before.
		void mergeThread(int thread_num);
		// Returns the thread number that owns the absolute_vertex number. absolute_vertex must not be a constrained vertex
		int threadOwner(int absolute_vertex_num);
		// Returns the local vertex number (i.e. vertex number within a thread), given the absolute vertex number (i.e. vertex number within all vertices).
		int localVertex(int absolute_vertex_num);
};

// Invalidates (sets to -1) the elements of v at indices 1. Indices are specified by constraintsNums. Indices in constraintsNums have to be in the range of v.
void invalidateAroundConstraintsNums(vector<int> &v, vector<int> constraintsNums);
// Invalidates (sets to -1) the elements of v at indices i. Indices i are specified by constraintsNums. Indices in constraintsNums have to be in the range of v.
void invalidateConstraintsNums(vector<int> &v, vector<int> constraintsNums);
template<typename T>
void mapAdd (vector<T> &v, T num);
// Last element of v1 and first element of v2 are equal to v[index].
template<typename T>
void splitVector (vector<T> &v1, vector<T> &v2, vector<T> v, int index);
// Last element of v1 is discarded since it is assumed to be the same as the first element of v2.
template<typename T>
void mergeVector (vector<T> &v, vector<T> v1, vector<T> v2);
// Last element of a vector in vectors is discarded since it is assumed to be the same as the first element of the consecutive vector in vectors. The last vector in vectors is the exception.
template<typename T>
void mergeMultipleVector(vector<T> &v, vector<vector<T> > vectors);
// Returns the position where the element was inserted.
int insertSorted (vector<int> &v, int e);
//Returns the position where the element was removed. Element to remove has to be in vector.
int removeSorted (vector<int> &v, int e);
//Returns the position of the element to be found.
int find(vector<int> v, int e);

#endif //_ThreadConstrained_h
