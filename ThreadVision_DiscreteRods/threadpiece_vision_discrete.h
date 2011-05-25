#ifndef _threadpiece_vision_discrete_h
#define _threadpiece_vision_discrete_h

#include "../DiscreteRods/thread_discrete.h"
#include "../vision/ThreeCam.h"
#include "thread_vision_discrete.h"
#include "thread_vision_utils.h"



static double VISUAL_COEFF = 0.005;


class Thread_Vision;

class ThreadPiece_Vision : public ThreadPiece 
{
public:
    ThreadPiece_Vision();
    ~ThreadPiece_Vision();

    ThreadPiece_Vision(const Vector3d& vertex, const double angle_twist);
    ThreadPiece_Vision(const Vector3d& vertex, const double angle_twist, ThreadPiece_Vision* prev, ThreadPiece_Vision* next);
    ThreadPiece_Vision(const Vector3d& vertex, const double angle_twist, ThreadPiece_Vision* prev, ThreadPiece_Vision* next, Thread_Vision* my_thread);
    ThreadPiece_Vision(const ThreadPiece_Vision& rhs);

    double energy_vis();

    /* visual reprojection error - reprojection error is for entire edge,
     * not just vertex! dist = distance */
    double energy_dist();

    void gradient_vertex_vis(Vector3d& grad);
    void gradient_vertex_vis_numeric(Vector3d& grad);




    void set_my_thread(Thread_Vision* my_thread){_my_thread = my_thread;};

    //overloaded operators
    ThreadPiece_Vision& operator=(const ThreadPiece_Vision& rhs);
  //private:
    Thread_Vision* _my_thread;
};




#endif

