#include <stdlib.h>
#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include "thread_discrete.h"
#include "trajectory_recorder.h"
#include "../../utils/clock.h"



#define TRAJ_BASE_NAME "rot1"

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN



void DrawStuff();
void updateThreadPoints();
void initThread();
void InitStuff();
void initMotions();


#define NUM_PTS 500
#define THREAD_RADII 1.0
#define MOVE_POS_CONST 1.0
#define MOVE_TAN_CONST 0.2
#define ROTATE_TAN_CONST 0.2


float rotate_frame[2];
float move_end[2];
float tangent_end[2];
float tangent_rotation_end[2];

Thread* thread;
vector<Vector3d> points;
vector<double> twist_angles;

Vector3d positions[2];
Vector3d tangents[2];
Matrix3d rotations[3];

vector<Frame_Motion> motions;
int motion_ind=0;
Trajectory_Recorder traj_recorder(TRAJ_BASE_NAME);

vector<double> total_rotations;


/* set up a light */
GLfloat lightOnePosition[] = {140.0, 0.0, 200.0, 0.0};
GLfloat lightOneColor[] = {0.99, 0.99, 0.99, 1.0};

GLfloat lightTwoPosition[] = {-140.0, 0.0, 200.0, 0.0};
GLfloat lightTwoColor[] = {0.99, 0.99, 0.99, 1.0};

GLfloat lightThreePosition[] = {140.0, 0.0, -200.0, 0.0};
GLfloat lightThreeColor[] = {0.99, 0.99, 0.99, 1.0};

GLfloat lightFourPosition[] = {-140.0, 0.0, -200.0, 0.0};
GLfloat lightFourColor[] = {0.99, 0.99, 0.99, 1.0};



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

double contour[NUM_PTS_CONTOUR][2];
double contour_norms[NUM_PTS_CONTOUR][2];


 void JoinStyle (int msg)
{
	exit (0);
}


void init_contour (void)
{
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



int main (int argc, char * argv[])
{
  StartClock();
  srand(time(NULL));
  srand((unsigned int)time((time_t *)NULL));

/*  int me, numprocs;
  MPI_Init(&argc, &argv);
  MPI_Comm_size(MPI_COMM_WORLD, &numprocs);
  std::cout << "num processors: " << numprocs << std::endl;
  MPI_Comm_rank(MPI_COMM_WORLD, &me);*/

  printf("Instructions:\n"
      "Hold down the left mouse button to rotate image: \n"
      "\n"
      "Hold 'm' while holding down the right mouse to move the end\n"
      "Hold 't' while holding down the right mouse to rotate the tangent \n"
      "\n"
      "Press 'q' to quit\n"
      );


	/* initialize glut */
	glutInit (&argc, argv); //can i do that?
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(900,900);
	glutCreateWindow ("Thread");
	glutDisplayFunc (DrawStuff);
	/*glutMotionFunc (MouseMotion);
  glutMouseFunc (processMouse);
  glutKeyboardFunc(processNormalKeys);
  glutKeyboardUpFunc(processKeyUp);
*/
	/* create popup menu */
	glutCreateMenu (JoinStyle);
	glutAddMenuEntry ("Exit", 99);
	glutAttachMenu (GLUT_MIDDLE_BUTTON);

	/* initialize GL */
	glClearDepth (1.0);
	glEnable (GL_DEPTH_TEST);
	glClearColor (0.0, 0.0, 0.0, 0.0);
	glShadeModel (GL_SMOOTH);

	glMatrixMode (GL_PROJECTION);
	/* roughly, measured in centimeters */
	glFrustum (-30.0, 30.0, -30.0, 30.0, 50.0, 500.0);
	glMatrixMode(GL_MODELVIEW);


	/* initialize lighting */
	glLightfv (GL_LIGHT0, GL_POSITION, lightOnePosition);
	glLightfv (GL_LIGHT0, GL_DIFFUSE, lightOneColor);
	glEnable (GL_LIGHT0);
	glLightfv (GL_LIGHT1, GL_POSITION, lightTwoPosition);
	glLightfv (GL_LIGHT1, GL_DIFFUSE, lightTwoColor);
	glEnable (GL_LIGHT1);
	glLightfv (GL_LIGHT2, GL_POSITION, lightThreePosition);
	glLightfv (GL_LIGHT2, GL_DIFFUSE, lightThreeColor);
	glEnable (GL_LIGHT2);
	glLightfv (GL_LIGHT3, GL_POSITION, lightFourPosition);
	glLightfv (GL_LIGHT3, GL_DIFFUSE, lightFourColor);
	glEnable (GL_LIGHT3);
	glEnable (GL_LIGHTING);
	glColorMaterial (GL_FRONT_AND_BACK, GL_DIFFUSE);
	glEnable (GL_COLOR_MATERIAL);

	InitStuff ();








  initThread();
  initMotions();
	thread->minimize_energy();
  updateThreadPoints();



  DrawStuff();


  double start_time = GetClock(); //time(NULL);
  while (motion_ind < motions.size())
  {
    std::cout << "motion ind " << motion_ind << std::endl;
    Vector3d end_pos = points.back();
    Matrix3d end_rot = thread->end_rot();
    motions[motion_ind].applyMotion(end_pos, end_rot);

    //change thread
    //thread->set_constraints(positions[0], rotations[0], positions[1], rotations[1]);
    thread->set_end_constraint(end_pos, end_rot);

    thread->minimize_energy();
    updateThreadPoints();

    DrawStuff();
    //sleep(2);

    double holonomy = thread->calculate_holonomy();
    double last_ang = thread->end_angle();
/*
    std::cout << "angle rotated: " << total_rotations[motion_ind] << "  sum of angs: " << (holonomy + last_ang) << " holonomy: " << holonomy << " last ang: " << last_ang << std::endl;
*/
    traj_recorder.add_thread_to_list(*thread);

    motion_ind++;
  }
  double end_time = GetClock();//time(NULL);
  std::cout << "num seconds: " << end_time - start_time << std::endl;



  vector<Vector3d> points;
  vector<double> twist_angles;
  thread->get_thread_data(points, twist_angles);
  std::cout << "Positions:\n" << std::endl;
  for (int i=0; i < points.size(); i++)
  {
    std::cout << points[i].transpose() << std::endl;
  }

  std::cout << "\nAngles:\n" << std::endl;
  for (int i=0; i < twist_angles.size(); i++)
  {
    std::cout << twist_angles[i] << std::endl;
  }


  std::cout << "writing to file"  << std::endl;
  traj_recorder.write_threads_to_file();

}

void DrawStuff (void)
{
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glColor3f (0.8, 0.3, 0.6);

  glPushMatrix ();

  /* set up some matrices so that the object spins with the mouse */
  glTranslatef (0.0,0.0,-100.0);
  glRotatef (rotate_frame[1], 1.0, 0.0, 0.0);
  glRotatef (rotate_frame[0], 0.0, 0.0, 1.0);


  //change thread, if necessary
  if (move_end[0] != 0.0 || move_end[1] != 0.0 || tangent_end[0] != 0.0 || tangent_end[1] != 0.0 || tangent_rotation_end[0] != 0 || tangent_rotation_end[1] != 0)
  {
    GLdouble model_view[16];
    glGetDoublev(GL_MODELVIEW_MATRIX, model_view);

    GLdouble projection[16];
    glGetDoublev(GL_PROJECTION_MATRIX, projection);

    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);


    double winX, winY, winZ;


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

    positions[1] = new_end_pos;

    Matrix3d rotation_new_tan;
    rotate_between_tangents(tangents[1], new_end_tan, rotation_new_tan);
    rotations[1] = rotation_new_tan*rotations[1];


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



    //change thread
    thread->set_constraints(positions[0], rotations[0], positions[1], rotations[1]);
    //thread->set_end_constraint(positions[1], rotations[1]);

    //std::cout <<"CONSTRAINT END:\n" << rotations[1] << std::endl;
    thread->minimize_energy();
    updateThreadPoints();

    //std::cout <<"ACTUAL END:\n" << thread->end_rot() << std::endl;

    //std::cout << "objX: " <<objX << " objY: " << objY << " objZ: " << objZ << " winX: " << winX << " winY: " << winY << " winZ: " << winZ << std::endl;



  }


  //Draw Axes
  glBegin(GL_LINES);
  glEnable(GL_LINE_SMOOTH);
  glColor3d(1.0, 0.0, 0.0); //red
  glVertex3f(0.0f, 0.0f, 0.0f); //x
  glVertex3f(10.0f, 0.0f, 0.0f);
  glColor3d(0.0, 1.0, 0.0); //green
  glVertex3f(0.0f, 0.0f, 0.0f); //y
  glVertex3f(0.0f, 10.0f, 0.0f);
  glColor3d(0.0, 0.0, 1.0); //blue
  glVertex3f(0.0f, 0.0f, 0.0f); //z
  glVertex3f(0.0f, 0.0f, 10.0f);

/*  glColor3d(0.5, 0.5, 1.0);
  glVertex3f(positions[1](0)-positions[0](0), positions[1](1)-positions[0](1), positions[1](2)-positions[0](2)); //z
  glVertex3f(positions[1](0)-positions[0](0)+tangents[1](0)*4.0, positions[1](1)-positions[0](1)+tangents[1](1)*4.0, positions[1](2)-positions[0](2)+tangents[1](2)*4.0); //z
*/


  //Draw Axes at End
  Vector3d diff_pos = positions[1]-positions[0];
  double rotation_scale_factor = 10.0;
  Matrix3d rotations_project = rotations[1]*rotation_scale_factor;
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


  glEnd( );



  //label axes
  void * font = GLUT_BITMAP_HELVETICA_18;
  glColor3d(1.0, 0.0, 0.0); //red
  glRasterPos3i(20.0, 0.0, -1.0);
  glutBitmapCharacter(font, 'X');
  glColor3d(0.0, 1.0, 0.0); //red
  glRasterPos3i(0.0, 20.0, -1.0);
  glutBitmapCharacter(font, 'Y');
  glColor3d(0.0, 0.0, 1.0); //red
  glRasterPos3i(-1.0, 0.0, 20.0);
  glutBitmapCharacter(font, 'Z');




  //Draw Thread
  glColor3f (0.5, 0.5, 0.2);

  /*
  double pts_cpy[points.size()][3];
  double twist_cpy[points.size()];
  pts_cpy[0][0] = 0.0;
  pts_cpy[0][1] = 0.0;
  pts_cpy[0][2] = 0.0;
  twist_cpy[0] = -(360.0/(2.0*M_PI))*twist_angles[0];
  //std::cout << twist_cpy[0] << std::endl;

  for (int i=1; i < points.size(); i++)
  {
    pts_cpy[i][0] = points[i](0)-(double)positions[0](0);
    pts_cpy[i][1] = points[i](1)-(double)positions[0](1);
    pts_cpy[i][2] = points[i](2)-(double)positions[0](2);
    twist_cpy[i] = -(360.0/(2.0*M_PI))*twist_angles[i];
    //std::cout << twist_cpy[i] << std::endl;
  }

  gleTwistExtrusion(20,
      contour,
      contour_norms,
      NULL,
      points.size(),
      pts_cpy,
      0x0,
      twist_cpy);
*/


  double pts_cpy[points.size()+2][3];
  double twist_cpy[points.size()+2];
  pts_cpy[1][0] = 0.0;
  pts_cpy[1][1] = 0.0;
  pts_cpy[1][2] = 0.0;
  twist_cpy[1] = -(360.0/(2.0*M_PI))*twist_angles[0];
  //std::cout << twist_cpy[0] << std::endl;

  for (int i=1; i < points.size()-1; i++)
  {
    pts_cpy[i+1][0] = points[i](0)-(double)positions[0](0);
    pts_cpy[i+1][1] = points[i](1)-(double)positions[0](1);
    pts_cpy[i+1][2] = points[i](2)-(double)positions[0](2);
    twist_cpy[i+1] = -(360.0/(2.0*M_PI))*twist_angles[i];
    //std::cout << twist_cpy[i+1] - twist_cpy[i] << std::endl;
  }

  //add first and last point
  pts_cpy[0][0] = -rotations[0](0,0);
  pts_cpy[0][1] = -rotations[0](1,0);
  pts_cpy[0][2] = -rotations[0](2,0);
  twist_cpy[0] = -(360.0/(2.0*M_PI))*twist_angles[0];

 pts_cpy[points.size()][0] = (double)positions[1](0)-(double)positions[0](0);
 pts_cpy[points.size()][1] = (double)positions[1](1)-(double)positions[0](1);
 pts_cpy[points.size()][2] = (double)positions[1](2)-(double)positions[0](2);
 twist_cpy[points.size()] = twist_cpy[points.size()-1];


  pts_cpy[points.size()+1][0] = pts_cpy[points.size()][0]+rotations[1](0,0);
  pts_cpy[points.size()+1][1] = pts_cpy[points.size()][1]+rotations[1](1,0);
  pts_cpy[points.size()+1][2] = pts_cpy[points.size()][2]+rotations[1](2,0);
  twist_cpy[points.size()+1] = twist_cpy[points.size()];

  gleTwistExtrusion(20,
      contour,
      contour_norms,
      NULL,
      points.size()+2,
      pts_cpy,
      0x0,
      twist_cpy);



  glPopMatrix ();

  glutSwapBuffers ();
}

void InitStuff (void)
{
	glEnable(GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//gleSetJoinStyle (TUBE_NORM_PATH_EDGE | TUBE_JN_ANGLE );
  rotate_frame[0] = -90.0;
  rotate_frame[1] = -90.0;

  int style;

  /* pick model-vertex-cylinder coords for texture mapping */
  //TextureStyle (509);

  /* configure the pipeline */
  style = TUBE_JN_CAP;
  style |= TUBE_CONTOUR_CLOSED;
  style |= TUBE_NORM_FACET;
  style |= TUBE_JN_ANGLE;
  gleSetJoinStyle (style);

//  lastx = 121.0;
//  lasty = 121.0;

  init_contour();

}




void updateThreadPoints()
{
  thread->get_thread_data(points, twist_angles);
  positions[0] = points.front();
  positions[1] = points.back();

  tangents[0] = points[1] - points[0];
  tangents[0].normalize();
  tangents[1] = points[points.size()-1] - points[points.size()-2];
  tangents[1].normalize();

  rotations[0] = thread->start_rot();
  rotations[1] = thread->end_rot();
}


void initThread()
{
  int numInit = 15;
  double noise_factor = 0.0;

  vector<Vector3d> vertices;
  vector<double> angles;

  vertices.push_back(Vector3d::Zero());
  angles.push_back(0.0);
  //push back unitx so first tangent matches start_frame
  vertices.push_back(Vector3d::UnitX()*_rest_length);
  angles.push_back(0.0);

  Vector3d direction;
  direction(0) = 0.3;
  direction(1) = 0.0;
  direction(2) = -1.0;
  direction.normalize();
  for (int i=0; i < numInit; i++)
  {
    Vector3d noise( ((double)(rand()%10000)) / 10000.0, ((double)(rand()%10000)) / 10000.0, ((double)(rand()%10000)) / 10000.0);
    noise *= noise_factor;
    Vector3d next_Vec = vertices.back()+(direction+noise).normalized()*_rest_length;
    vertices.push_back(next_Vec);
    angles.push_back(0.0);

    //std::cout << positions[i] << std::endl << std::endl;
  }



  //change direction
  direction(0) = 0.3;
  direction(1) = 0.0;
  direction(2) = 1.0;
  direction.normalize();

  for (int i=0; i < numInit; i++)
  {
    Vector3d noise( ((double)(rand()%10000)) / 10000.0, ((double)(rand()%10000)) / 10000.0, ((double)(rand()%10000)) / 10000.0);
    noise *= noise_factor;
    Vector3d next_Vec = vertices.back()+(direction+noise).normalized()*_rest_length;
    vertices.push_back(next_Vec);
    angles.push_back(0.0);

  }


  //push back unitx so last tangent matches end_frame
  vertices.push_back(vertices.back()+Vector3d::UnitX()*_rest_length);
  angles.push_back(0.0);


  angles.resize(vertices.size());

  rotations[0] = Matrix3d::Identity();
  rotations[1] = Matrix3d::Identity();

  /*
  Vector3d tan = (_thread_pieces[1].vertex() - _thread_pieces[0].vertex()).normalized();
  Vector3d toRotAxis;
  Matrix3d rot_for_frame;
  if ( (tan -Vector3d::UnitX()).norm() < 0.01 )
  {
    rot_for_frame = Matrix3d::Identity();
  }
  else
  {
    toRotAxis = Vector3d::UnitX().cross(tan);
    double Axisnorm = toRotAxis.norm();
    double toRotAng = asin(Axisnorm);

    toRotAxis /= Axisnorm;
   // std::cout << "axis: " << toRotAxis << std::endl;
   // std::cout << "ang: " << toRotAng << std::endl;

    rot_for_frame = (Eigen::AngleAxisd(toRotAng, toRotAxis));
  }
  //std::cout << "rot for frame: " << rot_for_frame << std::endl;

  _start_rot = rot_for_frame*Matrix3d::Identity();

  //std::cout << "start frame: " << _start_rot << std::endl;

  std::cout << "energy: " << calculate_energy() << std::endl;
  */


  thread = new Thread(vertices, angles, rotations[0], rotations[1]);
  updateThreadPoints();
}


void initMotions()
{

  Vector3d pos_movement = Vector3d::Zero();
  Matrix3d frame_rotation = Matrix3d::Identity();
  double total_rotation = 0.0;

  //Frame_Motion toAdd(pos_movement, frame_rotation);
  //motions.push_back(toAdd);

 // total_rotations.push_back(total_rotation);
/*
  frame_rotation = Matrix3d(Eigen::AngleAxisd(M_PI/30.0, Vector3d::UnitX()));
  for (int i=0; i < 1; i++)
  {
    total_rotation += M_PI/30;
    Frame_Motion toAdd(pos_movement, frame_rotation);
    motions.push_back(toAdd);

    total_rotations.push_back(total_rotation);
  }

*/

  pos_movement(0) = -0.5;
  pos_movement(1) = 0.5;
  pos_movement(2) = 0.0;
  frame_rotation = Matrix3d::Identity();
  for (int i=0; i < 10; i++)
  {
    Frame_Motion toAdd(pos_movement, frame_rotation);
    motions.push_back(toAdd);

    total_rotations.push_back(total_rotation);
  }



  // pos_movement(0) = 0.0;
  // pos_movement(1) = 0.05;
  // pos_movement(2) = 0.0;
  // frame_rotation = Matrix3d(Eigen::AngleAxisd(M_PI/50.0, Vector3d::UnitY()));
  // for (int i=0; i < 10; i++)
  // {
  //   Frame_Motion toAdd(pos_movement, frame_rotation);
  //   motions.push_back(toAdd);

  //   total_rotations.push_back(total_rotation);
  // }

  // pos_movement(0) = 0.0;
  // pos_movement(1) = 0.05;
  // pos_movement(2) = 0.0;
  // frame_rotation = Matrix3d(Eigen::AngleAxisd(M_PI/50.0, Vector3d::UnitZ()));
  // for (int i=0; i < 10; i++)
  // {
  //   Frame_Motion toAdd(pos_movement, frame_rotation);
  //   motions.push_back(toAdd);

  //   total_rotations.push_back(total_rotation);
  // }

  pos_movement(0) = 0.0;
  pos_movement(1) = 0.0;
  pos_movement(2) = 0.0;
  frame_rotation = Matrix3d(Eigen::AngleAxisd(M_PI/50.0, Vector3d::UnitX()));
  for (int i=0; i < 50; i++) {
      Frame_Motion toAdd(pos_movement, frame_rotation);
      motions.push_back(toAdd);

      total_rotations.push_back(total_rotation);
  }

  /*
  pos_movement(0) = 0.0;
  pos_movement(1) = 0.05;
  pos_movement(2) = 0.0;
  frame_rotation = Matrix3d(Eigen::AngleAxisd(M_PI/50.0, (Vector3d::UnitZ()+2*Vector3d::UnitY()).normalized()));
  for (int i=0; i < 3; i++)
  {
    Frame_Motion toAdd(pos_movement, frame_rotation);
    motions.push_back(toAdd);

    total_rotations.push_back(total_rotation);
  }
/*
  pos_movement(0) = -0.3;
  pos_movement(1) = 0.0;
  pos_movement(2) = 0.0;
  frame_rotation = Matrix3d::Identity();
  for (int i=0; i < 100; i++)
  {
    Frame_Motion toAdd(pos_movement, frame_rotation);
    motions.push_back(toAdd);
  }


  pos_movement(0) = 0.0;
  pos_movement(1) = 0.0;
  pos_movement(2) = 0.0;
  frame_rotation = Matrix3d(Eigen::AngleAxisd(-M_PI/100.0, Vector3d::UnitY()));
  for (int i=0; i < 100; i++)
  {
    Frame_Motion toAdd(pos_movement, frame_rotation);
    motions.push_back(toAdd);
  }

  pos_movement(0) = 0.0;
  pos_movement(1) = 0.0;
  pos_movement(2) = 0.0;
  frame_rotation = Matrix3d(Eigen::AngleAxisd(M_PI/70.0, Vector3d::UnitX()));
  for (int i=0; i < 100; i++)
  {
    Frame_Motion toAdd(pos_movement, frame_rotation);
    motions.push_back(toAdd);
  }

  pos_movement(0) = 0.0;
  pos_movement(1) = 0.0;
  pos_movement(2) = -0.2;
  frame_rotation = Matrix3d(Eigen::AngleAxisd(M_PI/70.0, Vector3d::UnitX()));
  for (int i=0; i < 100; i++)
  {
    Frame_Motion toAdd(pos_movement, frame_rotation);
    motions.push_back(toAdd);
  }
*/


}
