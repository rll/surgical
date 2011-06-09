#include "glThread.h"
//default 23 links
#define NUM_POINTS 11


GLThread::GLThread() {
  int numInit = (NUM_POINTS-3)/2;
  double noise_factor = 0.0;

  display_start_pos.setZero();

  vector<Vector3d> vertices;
  vector<double> angles;

  vertices.push_back(Vector3d::Zero());
  angles.push_back(0.0);
  //push back unitx so first tangent matches start_frame
  vertices.push_back(Vector3d::UnitX()*DEFAULT_REST_LENGTH);
  angles.push_back(0.0);

  Vector3d direction;
  direction(0) = 1.0;
  direction(1) = 0.0;
  direction(2) = -1.0;
  direction.normalize();
  for (int i=0; i < numInit; i++)
    {
      Vector3d noise( ((double)(rand()%10000)) / 10000.0, ((double)(rand()%10000)) / 10000.0, ((double)(rand()%10000)) / 10000.0);
      noise *= noise_factor;
      Vector3d next_Vec = vertices.back()+(direction+noise).normalized()*DEFAULT_REST_LENGTH;
      vertices.push_back(next_Vec);
      angles.push_back(0.0);

      //std::cout << positions[i] << std::endl << std::endl;
    }



  //change direction
  direction(0) = 1.0;
  direction(1) = 0.0;
  direction(2) = 1.0;
  direction.normalize();

  for (int i=0; i < numInit; i++)
    {
      Vector3d noise( ((double)(rand()%10000)) / 10000.0, ((double)(rand()%10000)) / 10000.0, ((double)(rand()%10000)) / 10000.0);
      noise *= noise_factor;
      Vector3d next_Vec = vertices.back()+(direction+noise).normalized()*DEFAULT_REST_LENGTH;
      vertices.push_back(next_Vec);
      angles.push_back(0.0);

    }

  //push back unitx so last tangent matches end_frame
  vertices.push_back(vertices.back()+Vector3d::UnitX()*DEFAULT_REST_LENGTH);
  angles.push_back(0.0);


  angles.resize(vertices.size());

  rotations[0] = Matrix3d::Identity();
  rotations[1] = Matrix3d::Identity();


  _thread = new Thread(vertices, angles, rotations[0], rotations[1]);
  _thread->set_rest_length(DEFAULT_REST_LENGTH);

#ifdef ISOTROPIC
  to_set_bend = _thread->get_bend_coeff();
#else
  to_set_B = _thread->get_bend_matrix();
#endif
  to_set_twist = _thread->get_twist_coeff();
  to_set_grav = _thread->get_grav_coeff();


#ifdef ISOTROPIC
    _thread->set_coeffs_normalized(to_set_bend, to_set_twist, to_set_grav);
#else
    _thread->set_coeffs_normalized(to_set_B, to_set_twist, to_set_grav);
#endif

  InitContour();
  strcpy(_display_name, "");
}

GLThread::GLThread(Thread* _thread) {

  // pass in transformed vertices, the same twists, identity start_rot

  // normalize the positions, rotations so that we are at 0,0, identity rot
  Vector3d start = _thread->start_pos();
  Matrix3d start_inv = _thread->start_rot().inverse();

  vector<Vector3d> vertices;
  vector<double> twists;
  _thread->get_thread_data(vertices, twists);
  for (int i=0; i < vertices.size(); i++)
  {
    Vector3d vert = vertices[i] - start;
    vertices[i] = start_inv*vert;
  }

  Matrix3d rot = Matrix3d::Identity();
  this->_thread = new Thread(vertices, twists, rot);

#ifdef ISOTROPIC
  to_set_bend = _thread->get_bend_coeff();
#else
  to_set_B = _thread->get_bend_matrix();
#endif
  to_set_twist = _thread->get_twist_coeff();
  to_set_grav = _thread->get_grav_coeff();




  InitContour();
}


//void GLThread::GetConfiguration(int* size, double* pts_cpy[][], double* twist_cpy[]) {
void GLThread::DrawThread() {
  updateThreadPoints();

  double pts_cpy[points.size()+2][3];
  double twist_cpy[points.size()+2];

  for (int i=0; i < points.size()-1; i++)
    {
      pts_cpy[i+1][0] = points[i](0)-display_start_pos(0);
      pts_cpy[i+1][1] = points[i](1)-display_start_pos(1);
      pts_cpy[i+1][2] = points[i](2)-display_start_pos(2);
      twist_cpy[i+1] = -(360.0/(2.0*M_PI))*twist_angles[i];
    }

  //std::cout << (points.back() - points.front()).norm() << std::endl;

  //add first and last point
  pts_cpy[0][0] = pts_cpy[1][0] -rotations[0](0,0);
  pts_cpy[0][1] = pts_cpy[1][1] -rotations[0](1,0);
  pts_cpy[0][2] = pts_cpy[1][2] -rotations[0](2,0);
  twist_cpy[0] = -(360.0/(2.0*M_PI))*twist_angles[0];

  pts_cpy[points.size()][0] = (double)positions[1](0)-display_start_pos(0);
  pts_cpy[points.size()][1] = (double)positions[1](1)-display_start_pos(1);
  pts_cpy[points.size()][2] = (double)positions[1](2)-display_start_pos(2);
  twist_cpy[points.size()] = twist_cpy[points.size()-1];


  pts_cpy[points.size()+1][0] = pts_cpy[points.size()][0]+rotations[1](0,0);
  pts_cpy[points.size()+1][1] = pts_cpy[points.size()][1]+rotations[1](1,0);
  pts_cpy[points.size()+1][2] = pts_cpy[points.size()][2]+rotations[1](2,0);
  twist_cpy[points.size()+1] = twist_cpy[points.size()];

  //(*size) = points.size()+2;
  gleTwistExtrusion_c4f(20,
                    contour,
                    contour_norms,
                    NULL,
                    points.size()+2,
                    pts_cpy,
                    0x0,
                    twist_cpy);



/*
#ifdef ISOTROPIC
    _thread->set_coeffs_normalized(to_set_bend, to_set_twist, to_set_grav);
#else
    _thread->set_coeffs_normalized(to_set_B, to_set_twist, to_set_grav);
    std::cout << "coeffs " << to_set_B << " " << to_set_twist << " " << to_set_grav << std::endl;
#endif
*/
}

void GLThread::minimize_energy() {
  _thread->minimize_energy();
  updateThreadPoints();
}



void GLThread::updateThreadPoints()
{
  _thread->get_thread_data(points, twist_angles);
  positions[0] = points.front();
  positions[1] = points.back();

  tangents[0] = points[1] - points[0];
  tangents[0].normalize();
  tangents[1] = points[points.size()-1] - points[points.size()-2];
  tangents[1].normalize();

  rotations[0] = _thread->start_rot();
  rotations[1] = _thread->end_rot();
}


void GLThread::ApplyUserInput(float move_end[], float tangent_end[], float tangent_rotation_end[])
{
	float move_start[2];
	move_start[0] = 0.0;
	move_start[1] = 0.0;
	float tangent_start[2];
	tangent_start[0] = 0.0;
	tangent_start[1] = 0.0;
	float tangent_rotation_start[2];
	tangent_rotation_start[0] = 0.0;
	tangent_rotation_start[1] = 0.0;

	ApplyUserInput(move_end, tangent_end, tangent_rotation_end, move_start, tangent_start, tangent_rotation_start);
}

void GLThread::ApplyUserInput(float move_end[], float tangent_end[], float tangent_rotation_end[], float move_start[], float tangent_start[], float tangent_rotation_start[])
{
  GLdouble model_view[16];
  glGetDoublev(GL_MODELVIEW_MATRIX, model_view);

  GLdouble projection[16];
  glGetDoublev(GL_PROJECTION_MATRIX, projection);

  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);


  double winX, winY, winZ;

  Two_Motions motion_to_apply;
  motion_to_apply._start.set_nomotion();


  //change end positions
  Vector3d new_end_pos;
  gluProject(positions[1](0), positions[1](1), positions[1](2), model_view, projection, viewport, &winX, &winY, &winZ);
  winX += move_end[0];
  winY += move_end[1];
  move_end[0] = 0.0;
  move_end[1] = 0.0;
  gluUnProject(winX, winY, winZ, model_view, projection, viewport, &new_end_pos(0), &new_end_pos(1), &new_end_pos(2));
  //    std::cout << "X: " << positions[1](0) << " Y: " << positions[1](1) << " Z: " << positions[1](2) << std::endl;

  //change tangents
  Vector3d new_end_tan;
  gluProject(positions[1](0)+tangents[1](0),positions[1](1)+tangents[1](1), positions[1](2)+tangents[1](2), model_view, projection, viewport, &winX, &winY, &winZ);
  winX += tangent_end[0];
  winY += tangent_end[1];
  tangent_end[0] = 0.0;
  tangent_end[1] = 0.0;
  gluUnProject(winX, winY, winZ, model_view, projection, viewport, &new_end_tan(0), &new_end_tan(1), &new_end_tan(2));
  new_end_tan -= positions[1];
  new_end_tan.normalize();

  //positions[1] = new_end_pos;
  motion_to_apply._end._pos_movement = new_end_pos-positions[1];

  Matrix3d rotation_new_tan;
  rotate_between_tangents(tangents[1], new_end_tan, rotation_new_tan);
  //rotations[1] = rotation_new_tan*rotations[1];
  motion_to_apply._end._frame_rotation = rotation_new_tan;


  //check rotation around tangent
  Vector3d tangent_normal_rotate_around = rotations[1].col(1);
  Vector3d new_end_tan_normal;
  gluProject(positions[1](0)+tangent_normal_rotate_around(0), positions[1](1)+tangent_normal_rotate_around(1), positions[1](2)+tangent_normal_rotate_around(2), model_view, projection, viewport, &winX, &winY, &winZ);
  winX += tangent_rotation_end[0];
  winY += tangent_rotation_end[1];
  tangent_rotation_end[0] = 0.0;
  tangent_rotation_end[1] = 0.0;
  gluUnProject(winX, winY, winZ, model_view, projection, viewport, &new_end_tan_normal(0), &new_end_tan_normal(1), &new_end_tan_normal(2));
  new_end_tan_normal -= positions[1];
  //project this normal onto the plane normal to X (to ensure Y stays normal to X)
  new_end_tan_normal -= new_end_tan_normal.dot(rotations[1].col(0))*rotations[1].col(0);
  new_end_tan_normal.normalize();


  rotations[1].col(1) = new_end_tan_normal;
  rotations[1].col(2) = rotations[1].col(0).cross(new_end_tan_normal);
  motion_to_apply._end._frame_rotation = Eigen::AngleAxisd(angle_mismatch(rotations[1], _thread->end_rot()), rotations[1].col(0).normalized())*motion_to_apply._end._frame_rotation;


  //change start positions
  Vector3d new_start_pos;
  gluProject(positions[0](0), positions[0](1), positions[0](2), model_view, projection, viewport, &winX, &winY, &winZ);
  winX += move_start[0];
  winY += move_start[1];
  move_start[0] = 0.0;
  move_start[1] = 0.0;
  gluUnProject(winX, winY, winZ, model_view, projection, viewport, &new_start_pos(0), &new_start_pos(1), &new_start_pos(2));
  //    std::cout << "X: " << positions[1](0) << " Y: " << positions[1](1) << " Z: " << positions[1](2) << std::endl;



  //change start tangents
  Vector3d new_start_tan;
  gluProject(positions[0](0)+tangents[0](0),positions[0](1)+tangents[0](1), positions[0](2)+tangents[0](2), model_view, projection, viewport, &winX, &winY, &winZ);
  winX += tangent_start[0];
  winY += tangent_start[1];
  tangent_start[0] = 0.0;
  tangent_start[1] = 0.0;
  gluUnProject(winX, winY, winZ, model_view, projection, viewport, &new_start_tan(0), &new_start_tan(1), &new_start_tan(2));
  new_start_tan -= positions[0];
  new_start_tan.normalize();


  //positions[0] = new_start_pos;
  motion_to_apply._start._pos_movement = new_start_pos-positions[0];

  Matrix3d rotation_new_start_tan;
  rotate_between_tangents(tangents[0], new_start_tan, rotation_new_tan);
  //rotations[0] = rotation_new_start_tan*rotations[0];
  motion_to_apply._start._frame_rotation = rotation_new_tan;
 

  //check rotation around start tangent
  tangent_normal_rotate_around = rotations[0].col(1);
  Vector3d new_start_tan_normal;
  gluProject(positions[0](0)+tangent_normal_rotate_around(0), positions[0](1)+tangent_normal_rotate_around(1), positions[0](2)+tangent_normal_rotate_around(2), model_view, projection, viewport, &winX, &winY, &winZ);
  winX += tangent_rotation_start[0];
  winY += tangent_rotation_start[1];
  tangent_rotation_start[0] = 0.0;
  tangent_rotation_start[1] = 0.0;
  gluUnProject(winX, winY, winZ, model_view, projection, viewport, &new_start_tan_normal(0), &new_start_tan_normal(1), &new_start_tan_normal(2));
  new_start_tan_normal -= positions[0];
  //project this normal onto the plane normal to X (to ensure Y stays normal to X)
  new_start_tan_normal -= new_start_tan_normal.dot(rotations[0].col(0))*rotations[0].col(0);
  new_start_tan_normal.normalize();


  rotations[0].col(1) = new_start_tan_normal;
  rotations[0].col(2) = rotations[0].col(0).cross(new_start_tan_normal);
  double angle_change_start = angle_mismatch(rotations[0], _thread->start_rot());
  motion_to_apply._start._frame_rotation = Eigen::AngleAxisd(angle_change_start, rotations[0].col(0).normalized())*motion_to_apply._start._frame_rotation;





  //change thread
  //_thread->set_constraints(positions[0], rotations[0], positions[1], rotations[1]);
  //thread->set_end_constraint(positions[1], rotations[1]);
  _thread->apply_motion_nearEnds(motion_to_apply);

  // std::cout <<"CONSTRAINT END:\n" << rotations[1] << std::endl;
  _thread->minimize_energy();
  updateThreadPoints();
  // std::cout <<"minimized END:\n" << rotations[1] << std::endl;
}


void GLThread::DrawAxes()
{
 //Draw Axes at Start
  Vector3d diff_pos = positions[0]-display_start_pos;
  double rotation_scale_factor = 10.0;
  Matrix3d rotations_project = rotations[0]*rotation_scale_factor;
  glBegin(GL_LINES);
  glEnable(GL_LINE_SMOOTH);
  glColor3d(1.0, 0.0, 0.0); //red
  glVertex3f((float)diff_pos(0), (float)diff_pos(1), (float)diff_pos(2)); //x
  glVertex3f((float)(diff_pos(0)+rotations_project(0,0)), (float)(diff_pos(1)+rotations_project(1,0)), (float)(diff_pos(2)+rotations_project(2,0)));
  glColor3d(0.0, 1.0, 0.0); //green
  glVertex3f((float)diff_pos(0), (float)diff_pos(1), (float)diff_pos(2)); //y
  glVertex3f((float)(diff_pos(0)+rotations_project(0,1)), (float)(diff_pos(1)+rotations_project(1,1)), (float)(diff_pos(2)+rotations_project(2,1)));
  glColor3d(0.0, 0.0, 1.0); //blue
  glVertex3f((float)diff_pos(0), (float)diff_pos(1), (float)diff_pos(2)); //z
  glVertex3f((float)(diff_pos(0)+rotations_project(0,2)), (float)(diff_pos(1)+rotations_project(1,2)), (float)(diff_pos(2)+rotations_project(2,2)));
  glEnd();


  //Draw Axes at End
  diff_pos = positions[1]-display_start_pos;
  rotation_scale_factor = 10.0;
  rotations_project = rotations[1]*rotation_scale_factor;
  glBegin(GL_LINES);
  glEnable(GL_LINE_SMOOTH);
  glColor3d(1.0, 0.0, 0.0); //red
  glVertex3f((float)diff_pos(0), (float)diff_pos(1), (float)diff_pos(2)); //x
  glVertex3f((float)(diff_pos(0)+rotations_project(0,0)), (float)(diff_pos(1)+rotations_project(1,0)), (float)(diff_pos(2)+rotations_project(2,0)));
  glColor3d(0.0, 1.0, 0.0); //green
  glVertex3f((float)diff_pos(0), (float)diff_pos(1), (float)diff_pos(2)); //y
  glVertex3f((float)(diff_pos(0)+rotations_project(0,1)), (float)(diff_pos(1)+rotations_project(1,1)), (float)(diff_pos(2)+rotations_project(2,1)));
  glColor3d(0.0, 0.0, 1.0); //blue
  glVertex3f((float)diff_pos(0), (float)diff_pos(1), (float)diff_pos(2)); //z
  glVertex3f((float)(diff_pos(0)+rotations_project(0,2)), (float)(diff_pos(1)+rotations_project(1,2)), (float)(diff_pos(2)+rotations_project(2,2)));

  glEnd();



}

void GLThread::InitContour ()
{
  int style;
  /* configure the pipeline */
  style = TUBE_JN_CAP;
  style |= TUBE_CONTOUR_CLOSED;
  style |= TUBE_NORM_FACET;
  style |= TUBE_JN_ANGLE;
  gleSetJoinStyle (style);

  int i;
  double contour_scale_factor = 0.3;



#ifdef ISOTROPIC
   // outline of extrusion
   i=0;
   CONTOUR (1.0 *contour_scale_factor, 1.0 *contour_scale_factor);
   CONTOUR (1.0 *contour_scale_factor, 2.9 *contour_scale_factor);
   CONTOUR (0.9 *contour_scale_factor, 3.0 *contour_scale_factor);
   CONTOUR (-0.9*contour_scale_factor, 3.0 *contour_scale_factor);
   CONTOUR (-1.0*contour_scale_factor, 2.9 *contour_scale_factor);

   CONTOUR (-1.0*contour_scale_factor, 1.0 *contour_scale_factor);
   CONTOUR (-2.9*contour_scale_factor, 1.0 *contour_scale_factor);
   CONTOUR (-3.0*contour_scale_factor, 0.9 *contour_scale_factor);
   CONTOUR (-3.0*contour_scale_factor, -0.9*contour_scale_factor);
   CONTOUR (-2.9*contour_scale_factor, -1.0*contour_scale_factor);

   CONTOUR (-1.0*contour_scale_factor, -1.0*contour_scale_factor);
   CONTOUR (-1.0*contour_scale_factor, -2.9*contour_scale_factor);
   CONTOUR (-0.9*contour_scale_factor, -3.0*contour_scale_factor);
   CONTOUR (0.9 *contour_scale_factor, -3.0*contour_scale_factor);
   CONTOUR (1.0 *contour_scale_factor, -2.9*contour_scale_factor);

   CONTOUR (1.0 *contour_scale_factor, -1.0*contour_scale_factor);
   CONTOUR (2.9 *contour_scale_factor, -1.0*contour_scale_factor);
   CONTOUR (3.0 *contour_scale_factor, -0.9*contour_scale_factor);
   CONTOUR (3.0 *contour_scale_factor, 0.9 *contour_scale_factor);
   CONTOUR (2.9 *contour_scale_factor, 1.0 *contour_scale_factor);

   CONTOUR (1.0 *contour_scale_factor, 1.0 *contour_scale_factor);   // repeat so that last normal is computed

#else
   // outline of extrusion
   i=0;
   CONTOUR (1.0*contour_scale_factor, 0.0*contour_scale_factor);
   CONTOUR (1.0*contour_scale_factor, 0.5*contour_scale_factor);
   CONTOUR (1.0*contour_scale_factor, 1.0*contour_scale_factor);
   CONTOUR (1.0*contour_scale_factor, 2.0*contour_scale_factor);
   CONTOUR (1.0*contour_scale_factor, 2.9*contour_scale_factor);
   CONTOUR (0.9*contour_scale_factor, 3.0*contour_scale_factor);
   CONTOUR (0.0*contour_scale_factor, 3.0*contour_scale_factor);
   CONTOUR (-0.9*contour_scale_factor, 3.0*contour_scale_factor);
   CONTOUR (-1.0*contour_scale_factor, 2.9*contour_scale_factor);

   CONTOUR (-1.0*contour_scale_factor, 2.0*contour_scale_factor);
   CONTOUR (-1.0*contour_scale_factor, 1.0*contour_scale_factor);
   CONTOUR (-1.0*contour_scale_factor, 0.5*contour_scale_factor);
   CONTOUR (-1.0*contour_scale_factor, 0.0*contour_scale_factor);
   CONTOUR (-1.0*contour_scale_factor, -0.5*contour_scale_factor);
   CONTOUR (-1.0*contour_scale_factor, -1.0*contour_scale_factor);
   CONTOUR (-1.0*contour_scale_factor, -2.0*contour_scale_factor);
   CONTOUR (-1.0*contour_scale_factor, -2.9*contour_scale_factor);
   CONTOUR (-0.9*contour_scale_factor, -3.0*contour_scale_factor);
   CONTOUR (0.0*contour_scale_factor, -3.0*contour_scale_factor);
   CONTOUR (0.9*contour_scale_factor, -3.0*contour_scale_factor);
   CONTOUR (1.0*contour_scale_factor, -2.9*contour_scale_factor);
   CONTOUR (1.0*contour_scale_factor, -2.0*contour_scale_factor);
   CONTOUR (1.0*contour_scale_factor, -1.0*contour_scale_factor);
   CONTOUR (1.0*contour_scale_factor, -0.5*contour_scale_factor);

   CONTOUR (1.0*contour_scale_factor, 0.0*contour_scale_factor);   // repeat so that last normal is computed

#endif


}

void GLThread::DrawName()
{
  GLdouble model_view[16];
  glGetDoublev(GL_MODELVIEW_MATRIX, model_view);

  GLdouble projection[16];
  glGetDoublev(GL_PROJECTION_MATRIX, projection);

  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);



  double winX, winY, winZ;
  double coordX, coordY, coordZ;
  coordX = points[1](0)-display_start_pos(0);
  coordY = points[1](1)-display_start_pos(1);
  coordZ = points[1](2)-display_start_pos(2);
  gluProject(coordX, coordY, coordZ, model_view, projection, viewport, &winX, &winY, &winZ);

  

  for (int i=0; i < strlen(_display_name); i++)
  {
    double x_add = 7.0*(double)i;
    double y_add = 15.0;
    //glRasterPos2f(winX+x_add, winY+y_add);
    gluUnProject(winX+x_add, winY+y_add, winZ, model_view, projection, viewport, &coordX, &coordY, &coordZ);
    glRasterPos3f(coordX, coordY, coordZ);
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, _display_name[i]);
  }




}

void GLThread::set_end_constraint(Vector3d end_pos, Matrix3d end_rot) {
  _thread->set_end_constraint(end_pos, end_rot);
}


void GLThread::printThreadData()
{
  updateThreadPoints();

  std::cout << "points:\n";
  for (int piece_ind=0; piece_ind < points.size(); piece_ind++)
  {
    std::cout << points[piece_ind].transpose() << "  ";
  }
  std::cout << "\nangles:\n";
  for (int piece_ind=0; piece_ind < twist_angles.size()-1; piece_ind++)
  {
    std::cout << twist_angles[piece_ind] << "  ";
  }
  std::cout << std::endl;

}
