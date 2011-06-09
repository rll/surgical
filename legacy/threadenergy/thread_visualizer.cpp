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
#include "thread_minenergy.h"
#include "trajectory_recorder.h"
#include "trajectory_reader.h"
#include "globals_thread_param_estimation.h"


// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN


void InitStuff();
void DrawStuff();
void saveThreadState(vector<Thread*> threads, int num_pts_per_thread);


#define NUM_PTS 200
#define THREAD_RADII 1.0
#define MOVE_POS_CONST 1.0


/*
enum TrajectoryModes {NONE_TRAJ, RECORD, PLAYBACK};
#define traj_mode PLAYBACK
#define TRAJ_FILE_PATH "rrt_traj.txt"
Trajectory_Recorder traj_recorder(TRAJ_FILE_PATH);
Trajectory_Reader traj_reader(TRAJ_FILE_PATH);


enum ImageRecordModes {NONE_IMAGE, RECORD_IMAGE, SAVE_POINTS};
#define image_record_mode SAVE_POINTS
#define IMAGE_BASE_NAME "./model_training_ims/rrt_traj_imgs"
#define NUM_PTS_TO_SAVE 65
vector<Thread*> intermediateThread;
*/





enum key_code {NONE, MOVEPOS, MOVETAN};


float lastx_L=0;
float lasty_L=0;
float lastx_R=0;
float lasty_R=0;

float rotate_frame[2];
float move_end[2];
float tangent_end[2];

Thread* thread;
MatrixXd points(NUM_PTS,3);

//for recording trajectory
Trajectory_Recorder traj_recorder;
Trajectory_Reader traj_reader;


int curr_im_ind = 1;

double radii[NUM_PTS];
int pressed_mouse_button;

Vector3d positions[2];
Vector3d tangents[2];

key_code key_pressed;


int num=2;
double curvature [2] = {1.0, 2.0};
double torsion [2] = {0.5, 0.0};
double length [2] = {75.0, 75.0};

/* set up a light */
GLfloat lightOnePosition[] = {140.0, 0.0, 200.0, 0.0};
GLfloat lightOneColor[] = {0.99, 0.99, 0.99, 1.0};

GLfloat lightTwoPosition[] = {-140.0, 0.0, 200.0, 0.0};
GLfloat lightTwoColor[] = {0.99, 0.99, 0.99, 1.0};

GLfloat lightThreePosition[] = {140.0, 0.0, -200.0, 0.0};
GLfloat lightThreeColor[] = {0.99, 0.99, 0.99, 1.0};

GLfloat lightFourPosition[] = {-140.0, 0.0, -200.0, 0.0};
GLfloat lightFourColor[] = {0.99, 0.99, 0.99, 1.0};



void processLeft(int x, int y)
{
  if (key_pressed == MOVEPOS)
  {
    move_end[0] += (x-lastx_L)*MOVE_POS_CONST;
    move_end[1] += (lasty_L-y)*MOVE_POS_CONST;
  } else if (key_pressed == MOVETAN)
  {
    tangent_end[0] += x-lastx_L;
    tangent_end[1] += lasty_L-y;
  } else {
    rotate_frame[0] += x-lastx_L;
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
    move_end[0] += (x-lastx_R)*MOVE_POS_CONST;
    move_end[1] += (lasty_R-y)*MOVE_POS_CONST;
  } else if (key_pressed == MOVETAN)
  {
    tangent_end[0] += x-lastx_R;
    tangent_end[1] += lasty_R-y;
  }

  lastx_R = x;
  lasty_R = y;
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
  else if (key == 'q')
  {
    delete thread;
    thread = new Thread(curvature, torsion, length, 2, positions, tangents);
    thread->minimize_energy_fixedPieces();
    glutPostRedisplay ();
  } else if (key == '1'){
    delete thread;
    length[0] = length[1] = 100.0;
    positions[0](0) = 50.0;
    positions[0](1) = 0.0;
    positions[0](2) = 0.0;
    positions[1](0) = -40.0;
    positions[1](1) = 80.0;
    positions[1](2) = 80.0;

    tangents[0](0) = 0.0;
    tangents[0](1) = 0.0;
    tangents[0](2) = 1.0;
    tangents[1](0) = 1.0;
    tangents[1](1) = 0.0;
    tangents[1](2) = 1.0;
    thread = new Thread(curvature, torsion, length, 2, positions, tangents);
    thread->minimize_energy_fixedPieces();
    glutPostRedisplay ();
  } else if (key == '2') {
    length[0] = length[1] = 150.0;
    positions[0](0) = -10.0;
    positions[0](1) = 140.0;
    positions[0](2) = 40.0;
    positions[1](0) = 70.0;
    positions[1](1) = 0.0;
    positions[1](2) = -20.0;

    tangents[0](0) = 0.0;
    tangents[0](1) = 0.0;
    tangents[0](2) = 1.0;
    tangents[1](0) = 1.0;
    tangents[1](1) = 0.0;
    tangents[1](2) = 0.5;
    thread = new Thread(curvature, torsion, length, 2, positions, tangents);
    thread->minimize_energy_fixedPieces();
    glutPostRedisplay ();
  } else if (key == '3') {
    length[0] = length[1] = 190.0;
    positions[0](0) = 0.0;
    positions[0](1) = 120.0;
    positions[0](2) = -40.0;
    positions[1](0) = 80.0;
    positions[1](1) = -30.0;
    positions[1](2) = 20.0;

    tangents[0](0) = 0.0;
    tangents[0](1) = 1.0;
    tangents[0](2) = 0.0;
    tangents[1](0) = 1.0;
    tangents[1](1) = 0.0;
    tangents[1](2) = -1.0;
    thread = new Thread(curvature, torsion, length, 2, positions, tangents);
    thread->minimize_energy_fixedPieces();
    glutPostRedisplay ();
  } else if (key == ' ') {
    Thread_Motion nextMotion;
    if (traj_reader.get_next_motion(nextMotion))
    {

      Thread* next = nextMotion.applyMotion(thread);
      positions[1] += nextMotion.pos_movement;
      tangents[1] = nextMotion.tan_rotation*tangents[1];
      delete thread;
      thread = next;

      traj_recorder.add_thread_to_list(new Thread(thread));
      DrawStuff();
    }
  } else if (key == 27) {
    if (traj_mode == RECORD)
      traj_recorder.write_motions_to_file();
    if (image_record_mode == SAVE_POINTS)
      traj_recorder.write_threads_to_file();
    exit(0);
  } else if (key=='p') {
    thread->initEnergyParams();
    thread->minimize_energy();
  }

  lastx_R = x;
  lasty_R = y;

}

void processKeyUp(unsigned char key, int x, int y)
{
  key_pressed = NONE;
  move_end[0] = move_end[1] = tangent_end[0] = tangent_end[1] = 0.0;
}



 void JoinStyle (int msg)
{
	exit (0);
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
      "Press 'q' to recalculate the thread from initial\n"
      );


	/* initialize glut */
	glutInit (&argc, argv); //can i do that?
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(900,900);
	glutCreateWindow ("Thread");
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

/*  positions[0](0) = 50.0;
	positions[0](1) = 0.0;
	positions[0](2) = 0.0;
	positions[1](0) = -40.0;
	positions[1](1) = 80.0;
	positions[1](2) = 80.0;
	tangents[0](0) = 0.0;
	tangents[0](1) = 0.0;
	tangents[0](2) = 1.0;
	tangents[1](0) = 1.0;
	tangents[1](1) = 0.0;
	tangents[1](2) = 1.0;*/

  positions[0](0) = 10.0;
	positions[0](1) = 0.0;
	positions[0](2) = 0.0;
	positions[1](0) = -50.0;
	positions[1](1) = 50.0;
	positions[1](2) = 10.0;
	tangents[0](0) = 0.0;
	tangents[0](1) = 0.0;
	tangents[0](2) = 1.0;
	tangents[1](0) = -1.0;
	tangents[1](1) = 1.0;
	tangents[1](2) = 0.0;



  tangents[0].normalize();
  tangents[1].normalize();

  thread = new Thread(curvature, torsion, length, num, positions, tangents);
  // thread->turnOnNoise(0.4);
  

	thread->minimize_energy_fixedPieces();


  if (image_record_mode != NONE_IMAGE)
    traj_recorder.add_thread_to_list(new Thread(thread));

  //see if we need to load motions
  if (traj_mode == PLAYBACK)
  {
    traj_reader.read_motions_from_file();
 /*   traj_reader.read_threads_from_file();
    traj_reader.estimate_init_thread();

    thread = new Thread(traj_reader.start_thread());

    thread->getWantedEndPosition(positions[1]);
    thread->getWantedEndTangent(tangents[1]);
    Matrix4d startTransEst;
    thread->getStartTransform(startTransEst);
    positions[0] = startTransEst.corner(Eigen::TopRight,3,1);
    tangents[0] = startTransEst.corner(Eigen::TopLeft,3,1);
    */
  }

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
	gleSetJoinStyle (TUBE_NORM_PATH_EDGE | TUBE_JN_ANGLE );
  rotate_frame[0] = 0.0;
  rotate_frame[1] = -111.0;
}

/* draw the helix shape */
void DrawStuff (void)
{
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glColor3f (0.8, 0.3, 0.6);

	glPushMatrix ();

	/* set up some matrices so that the object spins with the mouse */
	glTranslatef (0.0,0.0,-300.0);
  glRotatef (rotate_frame[1], 1.0, 0.0, 0.0);
  glRotatef (rotate_frame[0], 0.0, 0.0, 1.0);


  //change thread, if necessary
  if (traj_mode == PLAYBACK)// && trajectory_play_back_ind < trajectory.size())
  {
    /*
    Thread_Motion nextMotion;
    traj_reader.get_next_motion(nextMotion);
    positions[1] += nextMotion.pos_movement;
    tangents[1] = nextMotion.tan_rotation*tangents[1];

    //change thread
    thread->setConstraints(positions, tangents);
    //thread->upsampleAndOptimize_minLength(0.065);

    thread->minimize_energy_fixedPieces();
    */
  }
   else if (move_end[0] != 0.0 || move_end[1] != 0.0 || tangent_end[0] != 0.0 || tangent_end[1] != 0.0)
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
    gluProject(tangents[1](0), tangents[1](1), tangents[1](2), model_view, projection, viewport, &winX, &winY, &winZ);
    winX += tangent_end[0];
    winY += tangent_end[1];
    tangent_end[0] = 0.0;
    tangent_end[1] = 0.0;
    gluUnProject(winX, winY, winZ, model_view, projection, viewport, &new_end_tan(0), &new_end_tan(1), &new_end_tan(2));
    new_end_tan.normalize();




    if (traj_mode == RECORD)
    {
      traj_recorder.add_motion_to_list(positions[1], new_end_pos, tangents[1], new_end_tan);
    }

    positions[1] = new_end_pos;
    tangents[1] = new_end_tan;


    //change thread
    thread->setConstraints(positions, tangents);
    //thread->upsampleAndOptimize_minLength(0.065);

    thread->minimize_energy_fixedPieces();



    //std::cout << "objX: " <<objX << " objY: " << objY << " objZ: " << objZ << " winX: " << winX << " winY: " << winY << " winZ: " << winZ << std::endl;



  }


  //Draw Axes
	glBegin(GL_LINES);
	glEnable(GL_LINE_SMOOTH);
	glColor3d(1.0, 0.0, 0.0); //red
	glVertex3f(0.0f, 0.0f, 0.0f); //x
	glVertex3f(20.0f, 0.0f, 0.0f);
	glColor3d(0.0, 1.0, 0.0); //green
	glVertex3f(0.0f, 0.0f, 0.0f); //y
	glVertex3f(0.0f, 20.0f, 0.0f);
	glColor3d(0.0, 0.0, 1.0); //blue
	glVertex3f(0.0f, 0.0f, 0.0f); //z
	glVertex3f(0.0f, 0.0f, 20.0f);

	glColor3d(0.5, 0.5, 1.0);
	glVertex3f(positions[1](0)-positions[0](0), positions[1](1)-positions[0](1), positions[1](2)-positions[0](2)); //z
	glVertex3f(positions[1](0)-positions[0](0)+tangents[1](0)*4.0, positions[1](1)-positions[0](1)+tangents[1](1)*4.0, positions[1](2)-positions[0](2)+tangents[1](2)*4.0); //z


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

	thread->getPoints(points);
	double pts_cpy[NUM_PTS][3];

	for (int i=0; i < NUM_PTS; i++)
	{
		pts_cpy[i][0] = points(i,0)-(double)positions[0](0);
		pts_cpy[i][1] = points(i,1)-(double)positions[0](1);
		pts_cpy[i][2] = points(i,2)-(double)positions[0](2);
	}
	glePolyCone (NUM_PTS, pts_cpy, 0x0, radii);





	glPopMatrix ();

	glutSwapBuffers ();
}

/*
void saveThreadState(vector<Thread*> threads, int num_pts_per_thread)
{
  char thread_state_filename[256];
  sprintf(thread_state_filename, "%s.txt", IMAGE_BASE_NAME);

  ofstream threadFile;
  threadFile.precision(10);
  threadFile.open(thread_state_filename);
  MatrixXd points(num_pts_per_thread,3);

  //first line consists of length of thread, and pts per thread
  threadFile << threads[0]->length() << "  " << num_pts_per_thread << "\n";

  //write each point for each thread
  for (int i=0; i < threads.size(); i++)
  {
    Thread* thread = threads[i];

    thread->getPoints(points);

    for (int i=0; i < points.rows(); i++)
    {
      threadFile << points(i,0) << " " << points(i,1) << " " << points(i,2) << "   ";
    }
    threadFile << "\n";


  }
  threadFile.close();


}


*/
