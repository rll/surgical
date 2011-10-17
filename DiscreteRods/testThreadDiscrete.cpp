#include <stdio.h>
#include <stdlib.h>

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
#include "thread_discrete.h"
#include "trajectory_reader.h"
#include "trajectory_recorder.h"


// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

void init_contour();
void InitStuff();
void DrawStuff();
void updateThreadPoints();
void initThread();
void initThread_closedPolygon();

#define NUM_PTS 500
#define THREAD_RADII 1.0
#define MOVE_POS_CONST 1.0
#define MOVE_TAN_CONST 0.2
#define ROTATE_TAN_CONST 0.2


enum key_code {NONE, MOVEPOS, MOVETAN, ROTATETAN, MOVEPOSSTART, MOVETANSTART, ROTATETANSTART};




float lastx_L=0;
float lasty_L=0;
float lastx_R=0;
float lasty_R=0;

float rotate_frame[2];

float move_end[2];
float tangent_end[2];
float tangent_rotation_end[2];

float move_start[2];
float tangent_start[2];
float tangent_rotation_start[2];

Vector3d zero_location;
double zero_angle;

Thread* thread;
Thread* thread_saved;
vector<Vector3d> points;
vector<double> twist_angles;
vector<Matrix3d> material_frames;

double radii[NUM_PTS];
int pressed_mouse_button;

Vector3d positions[2];
Vector3d tangents[2];
Matrix3d rotations[3];

key_code key_pressed;
bool examine_mode = false;
bool print_mode_permanent = false;
bool print_mode_instant = false;
bool only_hor_spin = false;
bool only_ver_spin = false;

#define NUM_STEPS_FEW 10
bool few_minimization_steps = false;

double currentTime = 0;

int main_window = 0;

double offset_3d = 10.0;

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


void processLeft(int x, int y)
{
  if (key_pressed == MOVEPOS)
  {
    move_end[0] += (x-lastx_L)*MOVE_POS_CONST;
    move_end[1] += (lasty_L-y)*MOVE_POS_CONST;
  } else if (key_pressed == MOVETAN)
  {
    tangent_end[0] += (x-lastx_L)*MOVE_TAN_CONST;
    tangent_end[1] += (lasty_L-y)*MOVE_TAN_CONST;
  } else if (key_pressed == ROTATETAN)
  {
    tangent_rotation_end[0] += (x-lastx_L)*ROTATE_TAN_CONST;
    tangent_rotation_end[1] += (lasty_L-y)*ROTATE_TAN_CONST;
  } else if (key_pressed == MOVEPOSSTART)
  {
    move_start[0] += (x-lastx_L)*MOVE_POS_CONST;
    move_start[1] += (lasty_L-y)*MOVE_POS_CONST;
  } else if (key_pressed == MOVETANSTART)
  {
    tangent_start[0] += (x-lastx_L)*MOVE_TAN_CONST;
    tangent_start[1] += (lasty_L-y)*MOVE_TAN_CONST;
  } else if (key_pressed == ROTATETANSTART)
  {
    tangent_rotation_start[0] += (x-lastx_L)*ROTATE_TAN_CONST;
    tangent_rotation_start[1] += (lasty_L-y)*ROTATE_TAN_CONST;
  } else {
  	if (!only_ver_spin)
	    rotate_frame[0] += x-lastx_L;
    if (!only_hor_spin)
    	rotate_frame[1] += lasty_L-y;
  }

  lastx_L = x;
  lasty_L = y;
}

void processRight(int x, int y)
{
  //rotate_frame[0] += x-lastx_R;
  //rotate_frame[1] += y-lasty_R;

  if (key_pressed == MOVEPOS)
  {
    move_end[0] += (x-lastx_L)*MOVE_POS_CONST;
    move_end[1] += (lasty_L-y)*MOVE_POS_CONST;
  } else if (key_pressed == MOVETAN)
  {
    tangent_end[0] += (x-lastx_L)*MOVE_TAN_CONST;
    tangent_end[1] += (lasty_L-y)*MOVE_TAN_CONST;
  } else if (key_pressed == ROTATETAN)
  {
    tangent_rotation_end[0] += (x-lastx_L)*ROTATE_TAN_CONST;
    tangent_rotation_end[1] += (lasty_L-y)*ROTATE_TAN_CONST;
  } else if (key_pressed == MOVEPOSSTART)
  {
    move_start[0] += (x-lastx_L)*MOVE_POS_CONST;
    move_start[1] += (lasty_L-y)*MOVE_POS_CONST;
  } else if (key_pressed == MOVETANSTART)
  {
    tangent_start[0] += (x-lastx_L)*MOVE_TAN_CONST;
    tangent_start[1] += (lasty_L-y)*MOVE_TAN_CONST;
  } else if (key_pressed == ROTATETANSTART)
  {
    tangent_rotation_start[0] += (x-lastx_L)*ROTATE_TAN_CONST;
    tangent_rotation_start[1] += (lasty_L-y)*ROTATE_TAN_CONST;
  } else {
  	if (!only_ver_spin)
	    rotate_frame[0] += x-lastx_L;
    if (!only_hor_spin)
	    rotate_frame[1] += lasty_L-y;
  }

  lastx_L = x;
  lasty_L = y;

}

void MouseMotion (int x, int y)
{
  if (pressed_mouse_button == GLUT_LEFT_BUTTON) {
    processLeft(x, y);
  } else if (pressed_mouse_button == GLUT_RIGHT_BUTTON) {
    processRight(x,y);
  }
	glutPostRedisplay ();
}

void processMouse(int button, int state, int x, int y)
{
	if (state == GLUT_DOWN)
  {
    pressed_mouse_button = button;
    if (button == GLUT_LEFT_BUTTON)
    {
      lastx_L = x;
      lasty_L = y;
    }
    if (button == GLUT_RIGHT_BUTTON)
   {
      lastx_R = x;
      lasty_R = y;
    }
    glutPostRedisplay ();
	}
}


void processNormalKeys(unsigned char key, int x, int y)
{
	if (key == 't')
	  key_pressed = MOVETAN;
  else if (key == 'm')
    key_pressed = MOVEPOS;
  else if (key == 'r')
    key_pressed = ROTATETAN;
  else if (key == 'T')
	  key_pressed = MOVETANSTART;
  else if (key == 'M')
    key_pressed = MOVEPOSSTART;
  else if (key == 'R')
    key_pressed = ROTATETANSTART;
  else if ((key == '1') || (key == '2') || (key == '3') || (key == '4') || (key == '5') || 
  				 (key == '6') || (key == '7') || (key == '8') || (key == '9') || (key == '0')) {
  	Trajectory_Reader traj_reader;
  	char *fullPath = new char[256];
  	if (key == '1')
	    sprintf(fullPath, "%s%s", "saved_threads/", "vl_demo1");
	  else if (key == '2')
	  	sprintf(fullPath, "%s%s", "saved_threads/", "vl_demo2");
	  else if (key == '3')
	  	sprintf(fullPath, "%s%s", "saved_threads/", "vl_demo3");
	  else if (key == '4')
	  	sprintf(fullPath, "%s%s", "saved_threads/", "vl_demo4");
	  else if (key == '5')
	  	sprintf(fullPath, "%s%s", "saved_threads/", "vl_demo5");
	  else if (key == '6')
	  	sprintf(fullPath, "%s%s", "saved_threads/", "vl_demo6");
	  else if (key == '7')
	  	sprintf(fullPath, "%s%s", "saved_threads/", "vl_demo7");
	  else if (key == '8')
	  	sprintf(fullPath, "%s%s", "saved_threads/", "vl_demo8");
	  else if (key == '9')
	  	sprintf(fullPath, "%s%s", "saved_threads/", "vl_demo9");
	  else
	  	sprintf(fullPath, "%s%s", "saved_threads/", "vl_demo0");
    traj_reader.set_file(fullPath);
    if (traj_reader.read_threads_from_file() == 0) {
			vector<Thread*> threads_out;
			threads_out.clear();
			traj_reader.get_all_threads(threads_out);
			if (threads_out.size() > 0) {    
				delete thread;
				thread = new Thread(*(threads_out.front()));
				cout << "Thread loading was sucessful." << endl;
				updateThreadPoints();
			  glutPostRedisplay();  
			} else {	
				cout << "Specified file does not have a thread. Failed to load thread from file." << endl;
			}
		}    	
  }	else if (key == 'a') {
  	Trajectory_Reader traj_reader;
  	cout << "Loading...\n";
    cout << "Please enter source file name (without extension): ";
    char *srcFileName = new char[256];
    cin >> srcFileName;
    char *fullPath = new char[256];
    sprintf(fullPath, "%s%s", "saved_threads/", srcFileName);
    traj_reader.set_file(fullPath);
    if (traj_reader.read_threads_from_file() == 0) {
			vector<Thread*> threads_out;
			threads_out.clear();
			traj_reader.get_all_threads(threads_out);
			if (threads_out.size() > 0) {    
				delete thread;
				thread = new Thread(*(threads_out.front()));
				cout << "Thread loading was sucessful." << endl;
				updateThreadPoints();
			  glutPostRedisplay();  
			} else {	
				cout << "Specified file does not have a thread. Failed to load thread from file." << endl;
			}
		}    	
  } else if (key == 's') {
    delete thread_saved;
    thread_saved = new Thread(*thread);
    /* Save current trajectory */
    Trajectory_Recorder traj_recorder;
    cout << "Saving...\n";
    cout << "Please enter destination file name (without extension): ";
    char *dstFileName = new char[256];
    cin >> dstFileName;
    char *fullPath = new char[256];
    sprintf(fullPath, "%s%s", "saved_threads/", dstFileName);
    traj_recorder.setFileName(fullPath);
    Thread *newThread = thread_saved;
    Thread copiedThread(*newThread);
    traj_recorder.add_thread_to_list(copiedThread);
    traj_recorder.write_threads_to_file();
  } else if (key == 'S')  {
    Thread* temp = thread;
    thread = thread_saved;
    thread_saved = temp;
    updateThreadPoints();
    glutPostRedisplay();
  } else if (key == 'i')
  {
    thread->match_start_and_end_constraints(*thread_saved, 10);
    updateThreadPoints();
    glutPostRedisplay();
  }
  else if (key == 'q')
  {
    delete thread;
    glutPostRedisplay ();
  } else if (key == 27)
  {
    exit(0);
  } else if(key == 'd') { //toggle stepping to allow debugging
    few_minimization_steps = !few_minimization_steps;
    cout << "Debugging is now set to: " << few_minimization_steps << endl;
  } else if(key == 'n') {
    cout << "Stepping " << endl;
    thread->minimize_energy(NUM_STEPS_FEW);
    DrawStuff();
  } else if(key == 'l') {
    cout << "Stepping though project length constraint" << endl;
    thread->project_length_constraint();
    DrawStuff();
  } else if(key == 'e') {
  	examine_mode = !examine_mode;
  	init_contour();
  	glutPostRedisplay ();
  } else if(key == 'p') {
  	print_mode_permanent = !print_mode_permanent;
  	glutPostRedisplay ();
  } else if(key == 'o') {
  	print_mode_instant = !print_mode_instant;
  	glutPostRedisplay ();
  } else if(key == 'h' || key == 'H') {
  	only_hor_spin = !only_hor_spin;
  } else if(key == 'v' || key == 'V') {
  	only_ver_spin = !only_ver_spin;
  } else if (key == 'w') {
    rotate_frame[0] = 0.0;
    rotate_frame[1] = -111.0;
    glutPostRedisplay ();
  }

  lastx_R = x;
  lasty_R = y;

}

void processKeyUp(unsigned char key, int x, int y)
{
  key_pressed = NONE;
  move_end[0] = move_end[1] = tangent_end[0] = tangent_end[1] = tangent_rotation_end[0] = tangent_rotation_end[1] = 0.0;
  move_start[0] = move_start[1] = tangent_start[0] = tangent_start[1] = tangent_rotation_start[0] = tangent_rotation_start[1] = 0.0;
}



 void JoinStyle (int msg)
{
	exit (0);
}

void init_contour (void)
{
  int style;

  /* pick model-vertex-cylinder coords for texture mapping */
  //TextureStyle (509);

  /* configure the pipeline */
  style = TUBE_JN_CAP;
  style |= TUBE_CONTOUR_CLOSED;
  style |= TUBE_NORM_FACET;
  style |= TUBE_JN_ANGLE;
  gleSetJoinStyle (style);

   int i;
   double contour_scale_factor= 0.3;
   if (examine_mode)
    contour_scale_factor = 0.05;

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
	glutInitWindowSize(1680 / 2,900);
	main_window = glutCreateWindow ("Thread");
  glutPositionWindow(0,0);
	glutDisplayFunc (DrawStuff);
	glutMotionFunc (MouseMotion);
  glutMouseFunc (processMouse);
  glutKeyboardFunc(processNormalKeys);
  glutKeyboardUpFunc(processKeyUp);

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
//  thread->stepping = false;
//  thread->step = false;
	thread->minimize_energy();
  updateThreadPoints();
  thread_saved = new Thread(*thread);

  //zero_location = points[0];
  zero_location = points[0];// + 0.5*(points[0] + points.back());
  zero_location[0] += 0.5*(points[points.size()-1][0] - points[0][0]);
  zero_location[2] += 0.5*(points[(points.size()-1)/2][2] - points[0][2]);
  zero_angle = 0.0;



	for (int i=0; i < NUM_PTS; i++)
	{
		radii[i]=THREAD_RADII;
	}

  glutMainLoop ();
	//   return 0;             /* ANSI C requires main to return int. */
}


void InitStuff (void)
{
	glEnable(GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//gleSetJoinStyle (TUBE_NORM_PATH_EDGE | TUBE_JN_ANGLE );
  rotate_frame[0] = 0.0;
  rotate_frame[1] = -111.0;

//  lastx = 121.0;
//  lasty = 121.0;

  init_contour();
}

/* draw the helix shape */
void DrawStuff (void)
{
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glColor3f (0.8, 0.3, 0.6);

  glPushMatrix ();

  /* set up some matrices so that the object spins with the mouse */
  glTranslatef (0.0,0.0,-110.0);
  glRotatef (rotate_frame[1], 1.0, 0.0, 0.0);
  glRotatef (rotate_frame[0], 0.0, 0.0, 1.0);

	glTranslatef (-offset_3d,0.0,0.0);

  //change thread, if necessary
  if (move_end[0] != 0.0 || move_end[1] != 0.0 || tangent_end[0] != 0.0 || tangent_end[1] != 0.0 || tangent_rotation_end[0] != 0 || tangent_rotation_end[1] != 0 || move_start[0] != 0.0 || move_start[1] != 0.0 || tangent_start[0] != 0.0 || tangent_start[1] != 0.0 || tangent_rotation_start[0] != 0 || tangent_rotation_start[1] != 0)
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
    motion_to_apply._end._frame_rotation = Eigen::AngleAxisd(angle_mismatch(rotations[1], thread->end_rot()), rotations[1].col(0).normalized())*motion_to_apply._end._frame_rotation;





 //change start positions
    Vector3d new_start_pos;
    gluProject(positions[0](0), positions[0](1), positions[0](2), model_view, projection, viewport, &winX, &winY, &winZ);
    winX += move_start[0];
    winY += move_start[1];
    move_start[0] = 0.0;
    move_start[1] = 0.0;
    gluUnProject(winX, winY, winZ, model_view, projection, viewport, &new_start_pos(0), &new_start_pos(1), &new_start_pos(2));
    //    std::cout << "X: " << positions[1](0) << " Y: " << positions[1](1) << " Z: " << positions[1](2) << std::endl;

    //change tangents
    Vector3d new_start_tan;
    gluProject(positions[0](0)+tangents[0](0),positions[0](1)+tangents[0](1), positions[0](2)+tangents[0](2), model_view, projection, viewport, &winX, &winY, &winZ);
    winX += tangent_start[0];
    winY += tangent_start[1];
    tangent_start[0] = 0.0;
    tangent_start[1] = 0.0;
    gluUnProject(winX, winY, winZ, model_view, projection, viewport, &new_start_tan(0), &new_start_tan(1), &new_start_tan(2));
    new_start_tan -= positions[0];
    new_start_tan.normalize();

//    positions[0] = new_start_pos;
    motion_to_apply._start._pos_movement = new_start_pos-positions[0];

    Matrix3d rotation_new_start_tan;
    rotate_between_tangents(tangents[0], new_start_tan, rotation_new_start_tan);
    //rotations[0] = rotation_new_start_tan*rotations[0];
    motion_to_apply._start._frame_rotation = rotation_new_start_tan;

    //check rotation around tangent
    Matrix3d old_rot_start = rotations[0];
    Vector3d tangent_normal_rotate_around_start = rotations[0].col(1);
    Vector3d new_start_tan_normal;
    gluProject(positions[0](0)+tangent_normal_rotate_around_start(0), positions[0](1)+tangent_normal_rotate_around_start(1), positions[0](2)+tangent_normal_rotate_around_start(2), model_view, projection, viewport, &winX, &winY, &winZ);
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
    double angle_change_start = angle_mismatch(rotations[0], thread->start_rot());
    motion_to_apply._start._frame_rotation = Eigen::AngleAxisd(angle_change_start, rotations[0].col(0).normalized())*motion_to_apply._start._frame_rotation;


    zero_angle += angle_change_start;

    //change thread
    //thread->set_constraints(positions[0], rotations[0], positions[1], rotations[1]);
    thread->apply_motion_nearEnds(motion_to_apply);
    //thread->set_end_constraint(positions[1], rotations[1]);
    if(!few_minimization_steps) {
      thread->minimize_energy();
    }

		//thread->adapt_links();

    //std::cout <<"ACTUAL END:\n" << thread->end_rot() << std::endl;

    //std::cout << "objX: " <<objX << " objY: " << objY << " objZ: " << objZ << " winX: " << winX << " winY: " << winY << " winZ: " << winZ << std::endl;



  }
 
  updateThreadPoints();

  drawAxes(positions[0]-zero_location, rotations[0]);
	drawAxes(positions[1]-zero_location, rotations[1]);
	labelAxes(positions[0]-zero_location, rotations[0]);

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
 /* pts_cpy[1][0] = 0.0;
  pts_cpy[1][1] = 0.0;
  pts_cpy[1][2] = 0.0;
  twist_cpy[1] = -(360.0/(2.0*M_PI))*twist_angles[0];*/
  //std::cout << twist_cpy[0] << std::endl;
  for (int i=0; i < points.size(); i++)
  {
    pts_cpy[i+1][0] = points[i](0)-(double)zero_location(0);
    pts_cpy[i+1][1] = points[i](1)-(double)zero_location(1);
    pts_cpy[i+1][2] = points[i](2)-(double)zero_location(2);
    twist_cpy[i+1] = -(360.0/(2.0*M_PI))*(twist_angles[i]+zero_angle);
    //std::cout << twist_cpy[i+1] - twist_cpy[i] << std::endl;
  }
  //add first and last point
  pts_cpy[0][0] = pts_cpy[1][0]-rotations[0](0,0);
  pts_cpy[0][1] = pts_cpy[1][1]-rotations[0](1,0);
  pts_cpy[0][2] = pts_cpy[1][2]-rotations[0](2,0);
  twist_cpy[0] = twist_cpy[1]; //-(360.0/(2.0*M_PI))*(zero_angle);

/* pts_cpy[points.size()][0] = (double)positions[1](0)-(double)zero_location(0);
 pts_cpy[points.size()][1] = (double)positions[1](1)-(double)zero_location(1);
 pts_cpy[points.size()][2] = (double)positions[1](2)-(double)zero_location(2);
 twist_cpy[points.size()] = twist_cpy[points.size()-1];
*/

  pts_cpy[points.size()+1][0] = pts_cpy[points.size()][0]+rotations[1](0,0);
  pts_cpy[points.size()+1][1] = pts_cpy[points.size()][1]+rotations[1](1,0);
  pts_cpy[points.size()+1][2] = pts_cpy[points.size()][2]+rotations[1](2,0);
  twist_cpy[points.size()+1] = twist_cpy[points.size()]; //twist_cpy[points.size()]-(360.0/(2.0*M_PI))*zero_angle;

  gleTwistExtrusion(20,
      contour,
      contour_norms,
      NULL,
      points.size()+2,
      pts_cpy,
      0x0,
      twist_cpy);

	if (examine_mode) {
		glColor3f(0.0, 0.5, 0.5);
		for (int i=0; i<points.size(); i++)
			drawSphere(points[i]-zero_location, 0.7);
	}

  glPopMatrix ();

  glutSwapBuffers ();
}

void updateThreadPoints()
{
  thread->get_thread_data(points, twist_angles, material_frames);
  positions[0] = points.front();
  positions[1] = points.back();

  tangents[0] = points[1] - points[0];
  tangents[0].normalize();
  tangents[1] = points[points.size()-1] - points[points.size()-2];
  tangents[1].normalize();

  rotations[0] = thread->start_rot();
  rotations[1] = thread->end_rot();

  //rotations[0] = material_frames.front();
  //rotations[1] = material_frames[material_frames.size()-2];

	//std::cout << "last ang: " << thread->end_angle() << std::endl;
  //
  twist_angles.back() = 2.0*twist_angles[twist_angles.size()-2] - twist_angles[twist_angles.size()-3];
}


void initThread()
{
  int numInit = 15;
  double noise_factor = 0.0;

	double end_length = DEFAULT_REST_LENGTH;
	double start = DEFAULT_REST_LENGTH;
	double end = DEFAULT_REST_LENGTH;
	double m = (start-end)/(numInit-1);

  vector<Vector3d> vertices;
  vector<double> angles;
  vector<double> lengths;
  
  int i;
  Vector3d next_Vec;

  vertices.push_back(Vector3d::Zero());
  angles.push_back(0.0);
  lengths.push_back(end_length);
  //push back unitx so first tangent matches start_frame
  vertices.push_back(Vector3d::UnitX()*lengths.back());
  angles.push_back(0.0);
  lengths.push_back(end_length);

  Vector3d direction;
  direction(0) = 1.0;
  direction(1) = 0.0;
  direction(2) = -2.0;
  direction.normalize();
  for (i=0; i < numInit; i++)
  {
    Vector3d noise( ((double)(rand()%10000)) / 10000.0, ((double)(rand()%10000)) / 10000.0, ((double)(rand()%10000)) / 10000.0);
    noise *= noise_factor;
    if (i==0)
    	next_Vec = vertices.back()+Vector3d::UnitX()*lengths.back();
    else
    	next_Vec = vertices.back()+(direction+noise).normalized()*lengths.back();
    vertices.push_back(next_Vec);
    angles.push_back(0.0);
    lengths.push_back(-m*((double) i)+start);
    //std::cout << positions[i] << std::endl << std::endl;
  }
	
	i = 0;
	next_Vec = vertices.back()+(direction).normalized()*lengths.back();
  vertices.push_back(next_Vec);
  angles.push_back(0.0);
  lengths.push_back(m*((double) i)+end);

  //change direction
  direction(0) = 1.0;
  direction(1) = 0.0;
  direction(2) = 2.0;
  direction.normalize();

  for (i=1; i < numInit; i++)
  {
    Vector3d noise( ((double)(rand()%10000)) / 10000.0, ((double)(rand()%10000)) / 10000.0, ((double)(rand()%10000)) / 10000.0);
    noise *= noise_factor;
    next_Vec = vertices.back()+(direction+noise).normalized()*lengths.back();
    vertices.push_back(next_Vec);
    angles.push_back(0.0);
		lengths.push_back(m*((double) i)+end);
  }
	
	next_Vec = vertices.back()+(direction).normalized()*lengths.back();
	vertices.push_back(next_Vec);
  angles.push_back(0.0);
	lengths.push_back(end_length);
	
	vertices.push_back(vertices.back()+Vector3d::UnitX()*lengths.back());
  angles.push_back(0.0);
	lengths.push_back(end_length);
	
  //push back unitx so last tangent matches end_frame
  vertices.push_back(vertices.back()+Vector3d::UnitX()*lengths.back());
  angles.push_back(0.0);
	lengths.push_back(end_length);
	
  //angles.resize(vertices.size());

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
	
  thread = new Thread(vertices, angles, lengths, rotations[0], rotations[1]);
  updateThreadPoints();


#ifndef ISOTROPIC
	Matrix2d B = Matrix2d::Zero();
	B(0,0) = 10.0;
	B(1,1) = 1.0;
	thread->set_coeffs_normalized(B, 3.0, 1e-4);
#else
  thread->set_coeffs_normalized(1.0, 3.0, 1e-4);
#endif

 /* Vector3d start_pos_obj(10.0, 5.0, -40.0);
  Vector3d end_pos_obj(10.0, 5.0, 40.0);
  //Intersection_Object obj(1.5, start_pos_obj, end_pos_obj);
  //add_object_to_env(obj);
*/
}


void initThread_closedPolygon()
{
  int numVertices = 15;
  double angle_between = M_PI - (((double)(numVertices-2))*(M_PI))/((double)numVertices);
  //note that edge length is equal to rest_length

  double noise_factor = 1.0;

  Matrix3d currRot = Matrix3d::Identity();
  Vector3d currPoint(0.0, 0.0, 0.0);
  Vector3d startTan = Vector3d::UnitX();

  //setup first rotation matrix
  Matrix3d rotToStartTan;
  rotate_between_tangents(currRot.col(0), startTan, rotToStartTan);
  currRot = rotToStartTan*currRot;

  rotations[0] = currRot;

  vector<Vector3d> vertices;
  vector<double> angles;

  vertices.push_back(currPoint);
  angles.push_back(0.0);

  Matrix3d rotation_next_point = Matrix3d(Eigen::AngleAxisd(angle_between, Vector3d::UnitZ()));

  for (int vert_ind=1; vert_ind < numVertices; vert_ind++)
  {
    currPoint = currPoint + DEFAULT_REST_LENGTH*currRot.col(0);
    vertices.push_back(currPoint);
    angles.push_back(0.0);

    currRot = rotation_next_point*currRot;
  }

 //push back unitx so last tangent matches end_frame
  currPoint = currPoint + DEFAULT_REST_LENGTH*currRot.col(0);
  vertices.push_back(currPoint);
  angles.push_back(0.0);

  //add noise
  for (int vert_ind = 2; vert_ind < vertices.size()-2; vert_ind++)
  {
    //Vector3d noise( (double(rand()))/(double(RAND_MAX)+1.0)*2.0 - 1.0, (double(rand()))/(double(RAND_MAX)+1.0)*2.0 - 1.0, (double(rand()))/(double(RAND_MAX)+1.0)*2.0 - 1.0);
    Vector3d noise(0.0, 0.0, (double(rand()))/(double(RAND_MAX)+1.0)*2.0 - 1.0);
    noise *= noise_factor;
    vertices[vert_ind] += noise;
  }

  //print vertices
  /*std::cout << "vertices:" << std::endl;
  for (int vert_ind = 0; vert_ind < vertices.size(); vert_ind++)
  {
    std::cout << vertices[vert_ind].transpose() << std::endl;
  }*/

  rotations[1] = currRot;

  thread = new Thread(vertices, angles, rotations[0], rotations[1]);
  thread->project_length_constraint();
  updateThreadPoints();
}



