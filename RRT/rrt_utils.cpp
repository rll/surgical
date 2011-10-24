#include "rrt_utils.h"

void uniformlyRandomTranslation(Vector3d& translation, double max_dist)
{
  translation << (2.0*drand48() - 1.0), (2.0*drand48() - 1.0), (2.0*drand48() - 1.0);
  translation *= max_dist / sqrt(3.0);
}

void uniformlyRandomRotation(Quaterniond& rotation)
{
  double u1 = drand48();
  double u2 = drand48();
  double u3 = drand48();
  rotation = Quaterniond(sqrt(1.0-u1) * sin(2.0*M_PI*u2), sqrt(1.0-u1) * cos(2.0*M_PI*u2), sqrt(u1) * sin(2.0*M_PI*u3), sqrt(u1) * cos(2.0*M_PI*u3));
}

void uniformlyRandomRotation(Matrix3d& rotation)
{
  Quaterniond q;
  uniformlyRandomRotation(q);
  rotation = (Matrix3d) q;
}

// assumes thread has already been created
void generateRandomThread(ThreadConstrained* thread, double translation_offset_limit)
{
  Vector3d start_position = Vector3d::Zero();
  vector<Matrix3d> material_frames(thread->numVertices());
  for (int i = 0; i < material_frames.size(); i++) {
    do {
      uniformlyRandomRotation(material_frames[i]);
    } while ((i != 0) && (angle_between(material_frames[i].col(0), material_frames[i-1].col(0)) > M_PI/4.0));
  }
  // after this call, the thread won't be in a minimun energy configuration 
  // but in the configuration specified by material_frames
  thread->set_thread_data(start_position, material_frames);
  
  Vector3d end_position;
  thread->get_thread_data(start_position, end_position, material_frames);
  Vector3d mid_position = (start_position + end_position)/2.0;
  Vector3d translation_offset = translation_offset_limit * Vector3d((2.0*drand48() - 1.0), (2.0*drand48() - 1.0), (2.0*drand48() - 1.0));
  start_position += - mid_position + translation_offset;
  end_position += - mid_position + translation_offset;
  thread->set_thread_data(start_position, end_position, material_frames);
}
