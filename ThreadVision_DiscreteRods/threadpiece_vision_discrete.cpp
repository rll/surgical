#include "threadpiece_vision_discrete.h"

ThreadPiece_Vision::ThreadPiece_Vision()
    :ThreadPiece()
{
    grad_offsets[0] = Vector3d(grad_eps, 0.0, 0.0);
    grad_offsets[1] = Vector3d(0.0, grad_eps, 0.0);
    grad_offsets[2] = Vector3d(0.0, 0.0, grad_eps);
    rot = Matrix3d::Zero();
}

ThreadPiece_Vision::ThreadPiece_Vision(const Vector3d& vertex, const double angle_twist, Thread* my_thread)
    :ThreadPiece(vertex, angle_twist, my_thread)
{
    _my_thread = my_thread;
    grad_offsets[0] = Vector3d(grad_eps, 0.0, 0.0);
    grad_offsets[1] = Vector3d(0.0, grad_eps, 0.0);
    grad_offsets[2] = Vector3d(0.0, 0.0, grad_eps);
    rot = Matrix3d::Zero();
}

/*
ThreadPiece_Vision::ThreadPiece_Vision(const Vector3d& vertex, const double angle_twist, ThreadPiece_Vision* prev, ThreadPiece_Vision* next)
    :ThreadPiece(vertex, angle_twist, prev, next)
{
    grad_offsets[0] = Vector3d(grad_eps, 0.0, 0.0);
    grad_offsets[1] = Vector3d(0.0, grad_eps, 0.0);
    grad_offsets[2] = Vector3d(0.0, 0.0, grad_eps);
    rot = Matrix3d::Zero();
}
*/

ThreadPiece_Vision::ThreadPiece_Vision(const Vector3d& vertex, const double angle_twist, ThreadPiece_Vision* prev, ThreadPiece_Vision* next, Thread* my_thread, Thread_Vision* my_vision)
    :ThreadPiece(vertex, angle_twist, prev, next, my_thread), _my_vision(my_vision)
{
    grad_offsets[0] = Vector3d(grad_eps, 0.0, 0.0);
    grad_offsets[1] = Vector3d(0.0, grad_eps, 0.0);
    grad_offsets[2] = Vector3d(0.0, 0.0, grad_eps);
    rot = Matrix3d::Zero();
}


ThreadPiece_Vision::ThreadPiece_Vision(const ThreadPiece_Vision& rhs)
    :ThreadPiece(rhs), _my_vision(rhs._my_vision)
{
    grad_offsets[0] = Vector3d(grad_eps, 0.0, 0.0);
    grad_offsets[1] = Vector3d(0.0, grad_eps, 0.0);
    grad_offsets[2] = Vector3d(0.0, 0.0, grad_eps);
    rot = Matrix3d::Zero();
}


ThreadPiece_Vision::~ThreadPiece_Vision()
{
}


double ThreadPiece_Vision::energy_vis()
{
  //_prev_piece->energy();

    return energy() + energy_dist();
}

double ThreadPiece_Vision::energy_dist()
{
    if (_prev_piece == NULL)
    {
        cv::Point3f toProj;
        EigenToOpencv(_vertex, toProj);
        return VISUAL_COEFF*(_my_vision->scoreProjection3dPoint(toProj));
    } else {
        return VISUAL_COEFF*(_my_vision->scoreProjection3dPointAndTanget(_prev_piece->vertex(), _prev_piece->edge()));
    }
}


void ThreadPiece_Vision::gradient_vertex_vis(Vector3d& grad)
{
    gradient_vertex_vis_numeric(grad);
}

void ThreadPiece_Vision::gradient_vertex_vis_numeric(Vector3d& grad)
{
    if (_prev_piece == NULL)
    {
        for (int grad_ind = 0; grad_ind < 3; grad_ind++)
        {
            offset_and_update_locally(grad_offsets[grad_ind]);
            grad[grad_ind] = energy_vis() + ((ThreadPiece_Vision*)_next_piece)->energy_vis();
            offset_and_update_locally(-2.0*grad_offsets[grad_ind]);
            grad[grad_ind] -=energy_vis() + ((ThreadPiece_Vision*)_next_piece)->energy_vis();
            offset_and_update_locally(grad_offsets[grad_ind]);
            grad[grad_ind] /= (2.0*grad_eps);
        }
    } else if (_next_piece == NULL) {
        for (int grad_ind = 0; grad_ind < 3; grad_ind++)
        {
            offset_and_update_locally(grad_offsets[grad_ind]);
            grad[grad_ind] = ((ThreadPiece_Vision*)_prev_piece)->energy_vis() + energy_vis();
            offset_and_update_locally(-2.0*grad_offsets[grad_ind]);
            grad[grad_ind] -= ((ThreadPiece_Vision*)_prev_piece)->energy_vis() + energy_vis();
            offset_and_update_locally(grad_offsets[grad_ind]);
            grad[grad_ind] /= (2.0*grad_eps);
        }

    } else {
        for (int grad_ind = 0; grad_ind < 3; grad_ind++)
        {
            offset_and_update_locally(grad_offsets[grad_ind]);
            grad[grad_ind] = ((ThreadPiece_Vision*)_prev_piece)->energy_vis() + energy_vis() + ((ThreadPiece_Vision*)_next_piece)->energy_vis();
            offset_and_update_locally(-2.0*grad_offsets[grad_ind]);
            grad[grad_ind] -= ((ThreadPiece_Vision*)_prev_piece)->energy_vis() + energy_vis() + ((ThreadPiece_Vision*)_next_piece)->energy_vis();
            offset_and_update_locally(grad_offsets[grad_ind]);
            grad[grad_ind] /= (2.0*grad_eps);
        }
    }

}

//this probably doesn't work correctly!
ThreadPiece_Vision& ThreadPiece_Vision::operator=(const ThreadPiece_Vision& rhs)
{
   //ThreadPiece::operator= (rhs);
    _my_vision = rhs._my_vision;
    //_my_thread = rhs._my_thread;

    return *this;
}

