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
#include <Eigen/Cholesky>
#include <math.h>
#include "../DiscreteRods/glThread.h"
#include "../DiscreteRods/thread_discrete.h"
#include "../DiscreteRods/trajectory_reader.h"
#include "linearization_utils.h"
#include "iterative_control.h"
#include "trajectory_follower.h"
#include "../utils/clock.h"



#define TRAJ_BASE_NAME_NYLON "../DiscreteRods/LearnParams/config/suturenylon_processed_projected"
#define TRAJ_BASE_NAME_PURPLE "../DiscreteRods/LearnParams/config/suturepurple_processed_projected"
#define TRAJ_BASE_NAME_BLACK "../DiscreteRods/LearnParams/config/sutureblack_processed_projected"

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

void InitLights();
void InitGLUT(int argc, char * argv[]);
void InitStuff();
void DrawStuff();
void DrawAxes();
void DrawRRT();
void InitThread(int argc, char* argv[]);
void MinimizeThreadEnergies();
double calculate_thread_error(Thread* start, Thread* goal);
void InitMotions();
double playbackmotions(int max_linearizations = 1);

// #define THREAD_RADII 1.0
#define MOVE_POS_CONST 1.0
#define MOVE_TAN_CONST 0.2
#define ROTATE_TAN_CONST 0.2


#define NUM_THREADS 2

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

GLThread* glThreads[NUM_THREADS];
int simulated = 0;
int reality = 1;

int curThread = 1;

vector<Frame_Motion> motions;

// double radii[NUM_PTS];
int pressed_mouse_button;

key_code key_pressed;

int total_saved_threads;
int curr_thread_ind;
vector<Thread> all_threads;

/* set up a light */
GLfloat lightOnePosition[] = {140.0, 0.0, 200.0, 0.0};
GLfloat lightOneColor[] = {0.99, 0.99, 0.99, 1.0};

GLfloat lightTwoPosition[] = {-140.0, 0.0, 200.0, 0.0};
GLfloat lightTwoColor[] = {0.99, 0.99, 0.99, 1.0};

GLfloat lightThreePosition[] = {140.0, 0.0, -200.0, 0.0};
GLfloat lightThreeColor[] = {0.99, 0.99, 0.99, 1.0};

GLfloat lightFourPosition[] = {-140.0, 0.0, -200.0, 0.0};
GLfloat lightFourColor[] = {0.99, 0.99, 0.99, 1.0};


/*
void generateRandomThread() {
  // randomly stick out in a direction
  vector<Vector3d> vertices;
  vector<double> angles;
  int N = glThreads[realThread]->getThread()->num_pieces();


  vertices.push_back(Vector3d::Zero());
  angles.push_back(0.0);

  vertices.push_back(Vector3d::UnitX()*DEFAULT_REST_LENGTH);
  angles.push_back(0.0);

  double angle;
  Vector3d inc;
  Vector3d goal = glThreads[endThread]->getEndPosition() - glThreads[endThread]->getStartPosition();
  Vector3d noise;
  goal += (noise << Normal(0,1),Normal(0,1),Normal(0,1)).finished()*5.0;
  do {
    inc << Normal(0,1), Normal(0,1), Normal(0,1);
    //inc = goal;
    angle = acos(inc.dot(goal)/(goal.norm()*inc.norm()));
  } while(abs(angle) > M_PI/4.0);

  inc.normalize();
  inc *= DEFAULT_REST_LENGTH;

  for(int i = 0; i < N-2; i++) {
    vertices.push_back(vertices[vertices.size()-1] + inc);
    angles.push_back(0.0);
    // time to move toward the goal
    if ((vertices[vertices.size()-1] - goal).squaredNorm() > (N-2-i-1)*(N-2-i-1)*DEFAULT_REST_LENGTH*DEFAULT_REST_LENGTH) {
      inc = (goal - vertices[vertices.size()-1]).normalized()*DEFAULT_REST_LENGTH;
    }
  }

  Matrix3d rot = Matrix3d::Identity();
  glThreads[realThread]->setThread(new Thread(vertices, angles, rot));

  // better:
  // starting from 0,0, identity,
  // randomly sample links of thread given last link: maintain direction

  // minimize the energy on it
  glThreads[realThread]->minimize_energy();
  glThreads[realThread]->updateThreadPoints();
  glutPostRedisplay();
}
*/

void selectThread(int inc) {
  curThread = (curThread + inc) % NUM_THREADS;
	if (curThread < 0)
		curThread += NUM_THREADS;
  glutPostRedisplay();
}

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
  }
  else {
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
  }   else {
    rotate_frame[0] += x-lastx_L;
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
  if (key == 't') {
    key_pressed = MOVETAN;
  } else if (key == 'm') {
    key_pressed = MOVEPOS;
  } else if (key == 'r') {
    key_pressed = ROTATETAN;
  } else if (key == 'T') {
	  key_pressed = MOVETANSTART;
  } else if (key == 'M') {
    key_pressed = MOVEPOSSTART;
  } else if (key == 'R') {
    key_pressed = ROTATETANSTART;
  } else if ((key == 'n' || key == 'N') && curr_thread_ind < total_saved_threads-1) {
    curr_thread_ind++;
    std::cout << "displaying thread " << curr_thread_ind  << std::endl;
		glThreads[reality]->setThread((new Thread(all_threads[curr_thread_ind])));
    if (key == 'n')
      MinimizeThreadEnergies();
		glutPostRedisplay();
	} else if ((key == 'b' || key == 'B') && curr_thread_ind > 0) {
		curr_thread_ind--;
		std::cout << "displaying thread " << curr_thread_ind  << std::endl;
		glThreads[reality]->setThread((new Thread(all_threads[curr_thread_ind])));
    if (key == 'b')
      MinimizeThreadEnergies();
		glutPostRedisplay();
  } else if (key == 'c') {
		glThreads[simulated]->setThread((new Thread(*glThreads[reality]->getThread())));
  } else if (key == 'l') {
    solveLinearizedControl(glThreads[simulated]->getThread(), glThreads[reality]->getThread());
    std::cout << "new error: " << calculate_thread_error(glThreads[simulated]->getThread(), glThreads[reality]->getThread()) << std::endl;
    glutPostRedisplay();
	} else if (key == 'k') {
    solveLinearizedControl(glThreads[simulated]->getThread(), glThreads[reality]->getThread(), START);
    std::cout << "new error: " << calculate_thread_error(glThreads[simulated]->getThread(), glThreads[reality]->getThread()) << std::endl;
    glutPostRedisplay();
	} else if (key == 'L') {
    solveLinearizedControl(glThreads[simulated]->getThread(), glThreads[reality]->getThread(), START_AND_END);
    std::cout << "new error: " << calculate_thread_error(glThreads[simulated]->getThread(), glThreads[reality]->getThread()) << std::endl;
    glutPostRedisplay();
	} 
  else if (key == '1') {
    std::cout << "running experiment no noise" << std::endl;
    InitMotions();
    glThreads[reality]->to_set_bend = 1.0;
    glThreads[reality]->to_set_twist = 2.0;
    glThreads[reality]->to_set_grav = 0.0004;
    glThreads[reality]->setThreadCoeffs();
    glThreads[reality]->minimize_energy();
    glThreads[simulated]->to_set_bend = 1.0;
    glThreads[simulated]->to_set_twist = 2.0;
    glThreads[simulated]->to_set_grav = 0.0004;
    glThreads[simulated]->setThreadCoeffs();
    glThreads[simulated]->minimize_energy();
    double avg_error = playbackmotions(4);
    std::cout << "average error: " << avg_error << std::endl;
  } else if (key == '2') {
    std::cout << "running experiment offset params" << std::endl;
    InitMotions();
    glThreads[reality]->to_set_bend = 1.0;
    glThreads[reality]->to_set_twist = 1.0;
    glThreads[reality]->to_set_grav = 0.0004;
    glThreads[reality]->setThreadCoeffs();
    glThreads[reality]->minimize_energy();
    glThreads[simulated]->to_set_bend = 1.0;
    glThreads[simulated]->to_set_twist = 3.0;
    glThreads[simulated]->to_set_grav = 0.0004;
    glThreads[simulated]->setThreadCoeffs();
    glThreads[simulated]->minimize_energy();
    double avg_error = playbackmotions(4);
    std::cout << "average error: " << avg_error << std::endl;
  } else if (key == 27) {
    exit(0);
  }
  lastx_R = x;
  lasty_R = y;
}

void processKeyUp(unsigned char key, int x, int y)
{
  if (key == '=') {
    selectThread(1);
  } else if (key == '-') {
    selectThread(-1);
  }
  key_pressed = NONE;
  move_end[0] = move_end[1] = tangent_end[0] = tangent_end[1] = tangent_rotation_end[0] = tangent_rotation_end[1] = 0.0;
}



void JoinStyle (int msg)
{
  exit (0);
}


int main (int argc, char * argv[])
{

  srand(time(NULL));
  srand((unsigned int)time((time_t *)NULL));

  printf("Instructions:\n"
      "Hold down the left mouse button to rotate image: \n"
      "\n"
      "Hold 'm' while holding down the right mouse to move the end\n"
      "Hold 't' while holding down the right mouse to rotate the tangent \n"
      "\n"
      "Press 'Esc' to quit\n"
      );

  InitGLUT(argc, argv);
  InitLights();
  InitStuff ();

  vector<Two_Motions*> motions;
  VectorXd controls(12);
  double max_trans = 5;
  double max_angle = 2*M_PI;
  for (int i=0; i < 12; i++)
  {
    if (i % 6 < 3)
      controls(i) = ((double)(rand() % 10000))/(10000.0)*max_trans;
    else
      controls(i) = ((double)(rand() % 10000))/(10000.0)*max_angle;
  }

  std::cout << "controls:\n" << controls.transpose() << std::endl;

  VectorXd controls_after(12);
  control_to_TwoMotion(controls, motions);
  std::cout << "num motiosn: " << motions.size() << std::endl;
  TwoMotion_to_control(motions, controls_after);
  std::cout << "controls after:\n" << controls_after.transpose() << std::endl;

  InitThread(argc, argv);

  Matrix3d start_rotation_oldcontrol = Matrix3d::Identity();
  Matrix3d start_rotation_newcontrol = Matrix3d::Identity();

  Two_Motions old_summed;
  old_summed.set_nomotion();
  Two_Motions new_summed;
  new_summed.set_nomotion();

  vector<Two_Motions*> old_motions;
  vector<Two_Motions*> new_motions;

  control_to_TwoMotion(controls, old_motions);
  control_to_TwoMotion(controls_after, new_motions);

  std::cout << "old rotations:\n";
  for (int i=0; i < old_motions.size(); i++)
  {
    std::cout << old_motions[i]->_start._frame_rotation << std::endl << std::endl;
    start_rotation_oldcontrol *= old_motions[i]->_start._frame_rotation;
    old_summed = *old_motions[i] + old_summed;
  }
  std::cout << "\n\nnew rotations:\n";
  for (int i=0; i < new_motions.size(); i++)
  {
    std::cout << new_motions[i]->_start._frame_rotation << std::endl << std::endl;
    start_rotation_newcontrol *= new_motions[i]->_start._frame_rotation;
    new_summed = *new_motions[i] + new_summed;
  }
  std::cout << std::endl;

  Matrix3d expected_rotation;
  rotation_from_euler_angles(expected_rotation, controls(3), controls(4), controls(5));

  std::cout << "old end rotation:\n" << start_rotation_oldcontrol << "\nnew end rotation:\n" << start_rotation_newcontrol << "\nexpected:\n" << expected_rotation << std::endl;

  std::cout << "old end summed:\n" << old_summed._start._frame_rotation << std::endl;
  std::cout << "enw end summed:\n" << new_summed._start._frame_rotation << std::endl;
 
  // for (int i=0; i < NUM_PTS; i++)
  // {
  //   radii[i]=THREAD_RADII;
  // }


  glutMainLoop ();
}


GLuint sphereList;
void InitStuff (void)
{
  glEnable(GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  //gleSetJoinStyle (TUBE_NORM_PATH_EDGE | TUBE_JN_ANGLE );
  rotate_frame[0] = 0.0;
  rotate_frame[1] = -111.0;

  sphereList = glGenLists(1);
  glNewList(sphereList, GL_COMPILE);
  glutSolidSphere(0.5,16,16);
  glEndList();
}

/* draw the helix shape */
void DrawStuff (void)
{
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glColor3f (0.8, 0.3, 0.6);

  glPushMatrix ();

  /* set up some matrices so that the object spins with the mouse */
  glTranslatef (0.0,0.0,-170.0);
  glRotatef (rotate_frame[1], 1.0, 0.0, 0.0);
  glRotatef (rotate_frame[0], 0.0, 0.0, 1.0);


  //change thread, if necessary
  if (move_end[0] != 0.0 || move_end[1] != 0.0 || tangent_end[0] != 0.0 || tangent_end[1] != 0.0 || tangent_rotation_end[0] != 0 || tangent_rotation_end[1] != 0 || move_start[0] != 0.0 || move_start[1] != 0.0 || tangent_start[0] != 0.0 || tangent_start[1] != 0.0 || tangent_rotation_start[0] != 0 || tangent_rotation_start[1] != 0)
  {
    glThreads[curThread]->ApplyUserInput(move_end, tangent_end, tangent_rotation_end, move_start, tangent_start, tangent_rotation_start);
  }

  //Draw Axes
  DrawAxes();

  glThreads[reality]->updateThreadPoints();
  Vector3d display_start_pos = glThreads[reality]->getStartPosition();

	for(int i = 0; i < NUM_THREADS; i++) {

    //Draw Thread
    if (i==reality) {
      glColor4f (0.8, 0.5, 0.0, 0.9);
    } else if (i==simulated) {
      glColor4f (0.1, 0.1, 0.8, 0.9);
    } else {
      glColor4f (0.5, 0.5, 0.5, 0.9);
		}

		glThreads[i]->display_start_pos = display_start_pos;
    glThreads[i]->DrawThread();
		glThreads[i]->DrawAxes();
  }

  glPopMatrix ();
  glutSwapBuffers ();
}



void InitThread(int argc, char* argv[])
{
	Trajectory_Reader start_reader;
  if (argc < 3) {
	  start_reader.set_file(TRAJ_BASE_NAME_NYLON);
	} else {
		start_reader.set_file(argv[1]);
	}
	start_reader.read_threads_from_file();
	total_saved_threads = start_reader.get_all_threads().size();
	all_threads = start_reader.get_all_threads();
	curr_thread_ind = 0;
	for (int thread_ind=0; thread_ind < NUM_THREADS; thread_ind++)
	{
		glThreads[thread_ind] = new GLThread();
		glThreads[thread_ind]->setThread(new Thread(all_threads[curr_thread_ind]));
	}

  MinimizeThreadEnergies();

}

void DrawAxes()
{
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
  glEnd();
}

void MinimizeThreadEnergies()
{
	for (int thread_ind=0; thread_ind < NUM_THREADS; thread_ind++)
  {
		glThreads[thread_ind]->getThread()->minimize_energy();
  }
}

void InitGLUT(int argc, char * argv[]) {
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
}


void InitLights() {
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
}

double calculate_thread_error(Thread* start, Thread* goal)
{
  vector<Vector3d> points_start;
  vector<double> angles_start;
  start->get_thread_data(points_start, angles_start);
  vector<Vector3d> points_goal;
  vector<double> angles_goal;
  goal->get_thread_data(points_goal, angles_goal);
  
  return calculate_vector_diff_norm(points_start, points_goal);
}


void InitMotions()
{
  motions.clear();

  Vector3d pos_movement = Vector3d::Zero();
  Matrix3d frame_rotation = Matrix3d::Identity();

  pos_movement(0) = 1.5;
  pos_movement(1) = 0.75;
  pos_movement(2) = 0.0;
  frame_rotation = Matrix3d::Identity();
  for (int i=0; i < 10; i++)
  {
    Frame_Motion toAdd(pos_movement, frame_rotation);
    motions.push_back(toAdd);
  }

  pos_movement(0) = 0.0;
  pos_movement(1) = 0.0;
  pos_movement(2) = 0.0;
  frame_rotation = Matrix3d(Eigen::AngleAxisd(M_PI/20.0, Vector3d::UnitX()));
  for (int i=0; i < 20; i++) {
      Frame_Motion toAdd(pos_movement, frame_rotation);
      motions.push_back(toAdd);
  }

  pos_movement(0) = 0.0;
  pos_movement(1) = 0.0;
  pos_movement(2) = 0.0;
  frame_rotation = Matrix3d(Eigen::AngleAxisd(M_PI/30.0, Vector3d::UnitZ())) * Matrix3d(Eigen::AngleAxisd(M_PI/40.0, Vector3d::UnitY()));
  for (int i=0; i < 15; i++) {
      Frame_Motion toAdd(pos_movement, frame_rotation);
      motions.push_back(toAdd);
  }

  pos_movement(0) = 1.0;
  pos_movement(1) = 0.0;
  pos_movement(2) = 1.0;
  frame_rotation = Matrix3d::Identity();
  for (int i=0; i < 10; i++)
  {
    Frame_Motion toAdd(pos_movement, frame_rotation);
    motions.push_back(toAdd);
  }


}


double playbackmotions(int max_linearizations)
{
  vector<vector<VectorXd> > all_motions; 
  vector<Thread*> saved_threads;
  saved_threads.push_back(new Thread(*glThreads[simulated]->getThread()));
  
  double start_time = GetClock(); //time(NULL);
  //record motions
  for (int i=0; i < motions.size(); i++)
  {
    Two_Motions tmp_motion;
    tmp_motion._start.set_nomotion();
    tmp_motion._end = motions[i];

    vector<VectorXd> this_motion_wrapper(1);
    this_motion_wrapper[0].resize(12);
    TwoMotion_to_control(&tmp_motion, this_motion_wrapper[0]);

    all_motions.push_back(this_motion_wrapper);
    saved_threads.push_back(new Thread(*saved_threads.back()));

    applyControl(saved_threads.back(), this_motion_wrapper[0]);
  }


  //now play them back
  double total_error = 0;
  Trajectory_Follower trajectory_follower(saved_threads, all_motions, glThreads[reality]->getThread());
  std::cout << "num states: " << saved_threads.size() << " " << all_motions.size() << std::endl;
  while (!trajectory_follower.is_done())
  {
    trajectory_follower.Take_Step();

    *glThreads[simulated]->getThread() = *saved_threads[trajectory_follower.curr_ind()];
    std::cout << "curr ind: " << trajectory_follower.curr_ind() << std::endl;
    *glThreads[reality]->getThread() = *trajectory_follower.curr_state();

    total_error += calculate_thread_error(glThreads[simulated]->getThread(), glThreads[reality]->getThread());
    DrawStuff();
  }

  double end_time = GetClock();//time(NULL);
  std::cout << "num seconds: " << end_time - start_time << std::endl;



  return total_error/((double)motions.size());

}
