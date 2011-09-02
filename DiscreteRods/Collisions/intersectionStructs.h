#ifndef _intersectionStructs_h
#define _intersectionStructs_h

#include <Eigen/Core>
#include <Eigen/Geometry>

USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

struct Self_Intersection
{
  int _piece_ind_a;
  int _piece_ind_b;
  double _dist;
  Vector3d _direction;

  Self_Intersection() {}
  Self_Intersection(int piece_ind_a, int piece_ind_b, double dist, Vector3d& direction)
    : _piece_ind_a(piece_ind_a), _piece_ind_b(piece_ind_b), _dist(dist), _direction(direction) {}
  Self_Intersection(const Self_Intersection& cpy)
    : _piece_ind_a(cpy._piece_ind_a), _piece_ind_b(cpy._piece_ind_b), _dist(cpy._dist), _direction(cpy._direction) {}
  
};

struct Thread_Intersection
{
  int _piece_ind_a;
  int _piece_ind_b;
  int _thread_ind;
  double _dist;
  Vector3d _direction;

  Thread_Intersection() {}
  Thread_Intersection(int piece_ind_a, int piece_ind_b, int thread_ind, double dist, Vector3d& direction)
    : _piece_ind_a(piece_ind_a), _piece_ind_b(piece_ind_b), _thread_ind(thread_ind), _dist(dist), _direction(direction) {}
  Thread_Intersection(const Thread_Intersection& cpy)
    : _piece_ind_a(cpy._piece_ind_a), _piece_ind_b(cpy._piece_ind_b), _thread_ind(cpy._thread_ind), _dist(cpy._dist), _direction(cpy._direction) {}
};

struct Intersection
{
  int _piece_ind;
  double _dist;
  Vector3d _direction;

  Intersection() {}
  Intersection(int piece_ind, double dist, Vector3d& direction)
    : _piece_ind(piece_ind), _dist(dist), _direction(direction) {}
  Intersection(const Intersection& cpy)
    : _piece_ind(cpy._piece_ind), _dist(cpy._dist), _direction(cpy._direction) {}
};

struct Intersection_Object
{
  double _radius;
  Vector3d _start_pos;
  Vector3d _end_pos;
  Vector3d _direction;

  Intersection_Object() {}
  Intersection_Object(double radius, Vector3d& start_pos, Vector3d& end_pos, Vector3d& direction)
    : _radius(radius), _start_pos(start_pos), _end_pos(end_pos), _direction(direction) {}
  Intersection_Object(const Intersection_Object& cpy)
    : _radius(cpy._radius), _start_pos(cpy._start_pos), _end_pos(cpy._end_pos), _direction(cpy._direction) {}
};
/*
struct IntersectionThreadPiece
{   

  material_frame()
  edge_norm()
	vertex()
  
  btMatrix3x3(material_frame(0,0), material_frame(0,1), material_frame(0,1), material_frame(1,0), material_frame(1,1), material_frame(1,1), material_frame(2,0), material_frame(2,1), material_frame(2,1))
  
  int piece_ind;
  thread_piece* piece;
  thread* thread;
  CollisionObject* col_obj;
}*/

#endif
