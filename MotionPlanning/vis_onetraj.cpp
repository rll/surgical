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

#include "global_filenames.h"
#include <math.h>
#include "../DiscreteRods/glThread.h"
#include "../DiscreteRods/thread_discrete.h"
#include "../DiscreteRods/trajectory_reader.h"
#include "linearization_utils.h"
#include "iterative_control.h"
#include "trajectory_follower.h"
#include "../utils/clock.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "planner_lib.h"


using namespace cv;

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

void InitLights();
void InitGLUT(int argc, char * argv[]);
void InitStuff();
void DrawStuff();
void DrawAxes();

void Load_Init_Data();
void Load_Traj_Data();
void Update_Thread_Displays();
//void save_opengl_image();

double calculate_thread_error(Thread* start, Thread* goal);

// #define THREAD_RADII 1.0
#define MOVE_POS_CONST 1.0
#define MOVE_TAN_CONST 0.2
#define ROTATE_TAN_CONST 0.2

#define IMAGE_BASE_NAME "../saved_ims/glim"
#define NUM_TOTAL_DISPLAY 5
char image_save_base_opengl [256];
int im_save_ind = 1;


//#define NUM_THREADS 10
#define NUM_THREADS 18

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
bool display_thread[NUM_THREADS];
int init_start = 0;
int init_goal = 1;
int linearize_only_ind = 2;
int interpolate_point_and_linearize_ind = 3;
int sqp_openloop_ind = 4;
int sqp_closedloop_ind =5;
int RRT_ind = 6;
int RRT_SQP_openloop_ind = 7;
int RRT_SQP_closedloop_ind = 8;
int RRT_SQP_closedloop_onlylast_ind = 9;
int RRT_dim1_ind = 10;
int RRT_dim1_SQP_openloop_ind = 11;
int RRT_dim1_SQP_closedloop_ind = 12;
int RRT_dim1_SQP_closedloop_onlylast_ind = 13;
int RRT_dim2_ind = 14;
int RRT_dim2_SQP_openloop_ind = 15;
int RRT_dim2_SQP_closedloop_ind = 16; 
int RRT_dim2_SQP_closedloop_onlylast_ind = 17;

int curThread = RRT_ind;



int curr_thread_ind;
int num_links;
vector<Thread> start_threads;
vector<Thread> goal_threads;
char start_threads_filename[256];
char goal_threads_filename[256];
  

int curr_trajectory_ind;
char linearize_only_filename[256];
char interpolate_end_and_linear_filename[256];
char interpolate_point_and_linearize_filename[256];
char sqp_openloop_filename[256];
char sqp_closedloop_filename[256];
char RRT_filename[256];
char RRT_SQP_openloop_filename[256];
char RRT_SQP_closedloop_filename[256];
char RRT_SQP_closedloop_onlylast_filename[256];
char RRT_dim1_filename[256];
char RRT_dim1_SQP_openloop_filename[256];
char RRT_dim1_SQP_closedloop_filename[256];
char RRT_dim1_SQP_closedloop_onlylast_filename[256];
char RRT_dim2_filename[256];
char RRT_dim2_SQP_openloop_filename[256];
char RRT_dim2_SQP_closedloop_filename[256];
char RRT_dim2_SQP_closedloop_onlylast_filename[256];



vector<vector< Thread> > all_trajs(NUM_THREADS-2);



/*vector<Thread*> linearize_only_traj(0);
//vector<Thread*> interpolate_end_and_linear_traj;
vector<Thread*> interpolate_point_and_linearize_traj(0);
vector<Thread*> sqp_openloop_traj(0);
vector<Thread*> sqp_closedloop_traj(0);
vector<Thread*> RRT_traj(0);
vector<Thread*> RRT_SQP_openloop_traj(0);
vector<Thread*> RRT_SQP_closedloop_traj(0);
vector<Thread*> RRT_SQP_closedloop_onlylast_traj(0);
*/



// double radii[NUM_PTS];
int pressed_mouse_button;

key_code key_pressed;

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
  } else if ((key == 'n' || key == 'N') && curr_thread_ind < start_threads.size()-1) {
    curr_thread_ind++;
		Load_Traj_Data();
	} else if ((key == 'b' || key == 'B') && curr_thread_ind > 0) {
		curr_thread_ind--;
		Load_Traj_Data();
  } else if (key == '>') {
		curThread++;
		Load_Traj_Data();
	} else if (key == '<') {
		curThread--;
		Load_Traj_Data();
	}
	
	else if (key == 27) {
    exit(0);
  }

		std::cout << "thread_ind ind: " << curr_thread_ind << "  thread ind: " << curThread << std::endl;
  lastx_R = x;
  lasty_R = y;
	glutPostRedisplay();
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


	if (argc < 3)
	{
		std::cerr << "please provide arguments: start_ind   dimension" << std::endl;
	}

	srand(0);
	srand48(0);

  curr_thread_ind = atoi (argv[1]);
  num_links = atoi (argv[2]);

	std::cout << "curThread: " << curr_thread_ind << " " << num_links << std::endl;


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


	sprintf(image_save_base_opengl, "%s_%d_%d", IMAGE_BASE_NAME, num_links);


	Load_Init_Data();


  // for (int i=0; i < NUM_PTS; i++)
  // {
  //   radii[i]=THREAD_RADII;
  // }


  glutMainLoop ();
}


GLuint sphereList;
void InitStuff (void)
{
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
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
  glTranslatef (0.0,0.0,-110.0);
  glRotatef (rotate_frame[1], 1.0, 0.0, 0.0);
  glRotatef (rotate_frame[0], 0.0, 0.0, 1.0);


  //change thread, if necessary
  if (move_end[0] != 0.0 || move_end[1] != 0.0 || tangent_end[0] != 0.0 || tangent_end[1] != 0.0 || tangent_rotation_end[0] != 0 || tangent_rotation_end[1] != 0 || move_start[0] != 0.0 || move_start[1] != 0.0 || tangent_start[0] != 0.0 || tangent_start[1] != 0.0 || tangent_rotation_start[0] != 0 || tangent_rotation_start[1] != 0)
  {
    glThreads[curThread]->ApplyUserInput(move_end, tangent_end, tangent_rotation_end, move_start, tangent_start, tangent_rotation_start);
  }

  //Draw Axes
  DrawAxes();

  glThreads[init_start]->updateThreadPoints();
  Vector3d display_start_pos = glThreads[init_start]->getStartPosition();

	for(int i = 0; i < NUM_TOTAL_DISPLAY+2; i++) {
		if (!display_thread[i])
			continue;
	
    //Draw Thread
    if (i==init_start) {
      glColor4f (0.99, 0.48, 0.1, 0.9); //gold
    } else if (i==init_goal) {
      glColor4f (0.1, 0.1, 0.8, 0.9); //blue
    } else {
			glColor4f (0.8, 0.1, 0.1, 0.9); //red
		}
		/*
		else if (i == linearize_only_ind) {
      glColor4f (0.2, 0.8, 0.2, 0.9); //green
		} else if (i== interpolate_point_and_linearize_ind) {
      glColor4f (0.4, 0.8, 0.55, 0.9); //aquamarine
		} else if (i == sqp_openloop_ind) {
      glColor4f (0.5, 0.4, 0.99, 0.9); //light purple
		} else if (i == sqp_closedloop_ind) {
      glColor4f (0.5, 0.2, 0.9, 0.9); //blueviolet
		} else if (i == RRT_ind) {
      glColor4f (0.8, 0.1, 0.1, 0.9); //red
		} else if (i == RRT_SQP_openloop_ind) {
      glColor4f (0.9, 0.4, 0.1, 0.9); //orange
		} else if (i == RRT_SQP_closedloop_ind) {
      glColor4f (0.7, 0.6, 0.05, 0.9); //yellow
		} else if (i == RRT_SQP_closedloop_onlylast_ind) {
      glColor4f (0.7, 0.99, 0.25, 0.9); //yellow-green
		}	else {
      glColor4f (0.5, 0.5, 0.5, 0.9);
		}*/

		glThreads[i]->display_start_pos = display_start_pos;
    glThreads[i]->DrawThread();
		//glThreads[i]->DrawName();
		//glThreads[i]->DrawAxes();
  }

  glPopMatrix ();
  glutSwapBuffers ();
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



void Load_Init_Data()
{
	sprintf(start_threads_filename, "%s/%s_%d", BASEFOLDER_INITDATA, BASENAME_STARTTHREADS, num_links);
	sprintf(goal_threads_filename, "%s/%s_%d", BASEFOLDER_INITDATA, BASENAME_GOALTHREADS, num_links);

	Trajectory_Reader start_threads_reader(start_threads_filename);
	Trajectory_Reader goal_threads_reader(goal_threads_filename);
	start_threads_reader.read_threads_from_file();
	goal_threads_reader.read_threads_from_file();
	start_threads = start_threads_reader.get_all_threads();
	goal_threads = goal_threads_reader.get_all_threads();

	

///////write out thread energies to file
/*
char init_start_energies_filename[256];
char init_goal_energies_filename[256];

sprintf(init_start_energies_filename, "./benchmark_data/init_data/%s_%d.txt", "start_energies", num_links);
sprintf(init_goal_energies_filename, "./benchmark_data/init_data/%s_%d.txt", "goal_energies", num_links);


ofstream init_start_energies(init_start_energies_filename);
ofstream init_goal_energies(init_goal_energies_filename);

	for (int thread_ind=0; thread_ind < start_threads.size(); thread_ind++)
	{
		init_start_energies << start_threads[thread_ind].calculate_energy() << "\n";
		init_goal_energies << goal_threads[thread_ind].calculate_energy() << "\n";
	}

		init_start_energies << "\n";
		init_goal_energies <<  "\n";

init_start_energies.close();
init_goal_energies.close();

*/

////////////////////////////////////////////

	for (int thread_ind=0; thread_ind < NUM_THREADS; thread_ind++)
	{
		glThreads[thread_ind] = new GLThread();
		glThreads[thread_ind]->setThread(new Thread(start_threads[curr_thread_ind]));
	}

	glThreads[init_start]->setThread(new Thread(start_threads[curr_thread_ind]));
	glThreads[init_goal]->setThread(new Thread(goal_threads[curr_thread_ind]));


	for (int i=0; i < NUM_THREADS; i++)
	{
		display_thread[i] = true;
	}

	glThreads[init_start]->setName("Start");
	glThreads[init_goal]->setName("Goal");
	glThreads[linearize_only_ind]->setName("Linearize Only");
	glThreads[interpolate_point_and_linearize_ind]->setName("Interpolate and Linearize");
	glThreads[sqp_openloop_ind]->setName("SQP Openloop");
	glThreads[sqp_closedloop_ind]->setName("SQP Closedloop");
	glThreads[RRT_ind]->setName("RRT");
	glThreads[RRT_SQP_openloop_ind]->setName("RRT + SQP Openloop");
	glThreads[RRT_SQP_closedloop_ind]->setName("RRT + SQP Closedloop");
	glThreads[RRT_SQP_closedloop_onlylast_ind]->setName("RRT + SQP Finish");
	glThreads[RRT_dim1_ind]->setName("RRT_dim1");
	glThreads[RRT_dim1_SQP_openloop_ind]->setName("RRT_dim1 + SQP Openloop");
	glThreads[RRT_dim1_SQP_closedloop_ind]->setName("RRT_dim1 + SQP Closedloop");
	glThreads[RRT_dim1_SQP_closedloop_onlylast_ind]->setName("RRT_dim1 + SQP Finish");
	glThreads[RRT_dim2_ind]->setName("RRT_dim2");
	glThreads[RRT_dim2_SQP_openloop_ind]->setName("RRT_dim2 + SQP Openloop");
	glThreads[RRT_dim2_SQP_closedloop_ind]->setName("RRT_dim2 + SQP Closedloop");
	glThreads[RRT_dim2_SQP_closedloop_onlylast_ind]->setName("RRT_dim2 + SQP Finish");
	/*
int init_start = 0;
int init_goal = 1;
int linearize_only_ind = 2;
int interpolate_point_and_linearize_ind = 3;
int sqp_openloop_ind = 4;
int sqp_closedloop_ind =5;
int RRT_ind = 6;
int RRT_SQP_openloop_ind = 7;
int RRT_SQP_closedloop_ind = 8;
int RRT_SQP_closedloop_onlylast_ind = 9;
*/
	


	Load_Traj_Data();
	//display_thread[interpolate_end_and_linear_ind] = false;

}


void Load_Traj_Data()
{
	glThreads[init_start]->setThread((new Thread(start_threads[curr_thread_ind])));
	glThreads[init_goal]->setThread((new Thread(goal_threads[curr_thread_ind])));

	sprintf(linearize_only_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_LINEARIZE_ONLY, curr_thread_ind);
	//sprintf(interpolate_end_and_linear_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_INTERPOLATE_ENDS_AND_LINEARIZE, curr_thread_ind);
	sprintf(interpolate_point_and_linearize_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_INTERPOLATE_POINT_AND_LINEARIZE, curr_thread_ind);
	sprintf(sqp_openloop_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_SQP_OPENLOOP, curr_thread_ind);
	sprintf(sqp_closedloop_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_SQP_CLOSEDLOOP, curr_thread_ind);
	sprintf(RRT_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_RRT, curr_thread_ind);
	sprintf(RRT_SQP_openloop_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_RRT_SQP_OPENLOOP, curr_thread_ind);
	sprintf(RRT_SQP_closedloop_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_RRT_SQP_CLOSEDLOOP, curr_thread_ind);
	sprintf(RRT_SQP_closedloop_onlylast_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_RRT_SQP_CLOSEDLOOP_ONLYLAST, curr_thread_ind);
	sprintf(RRT_dim1_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_RRT_dim1, curr_thread_ind);
	sprintf(RRT_dim1_SQP_openloop_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_RRT_dim1_SQP_OPENLOOP, curr_thread_ind);
	sprintf(RRT_dim1_SQP_closedloop_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_RRT_dim1_SQP_CLOSEDLOOP, curr_thread_ind);
	sprintf(RRT_dim1_SQP_closedloop_onlylast_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_RRT_dim1_SQP_CLOSEDLOOP_ONLYLAST, curr_thread_ind);
	sprintf(RRT_dim2_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_RRT_dim2, curr_thread_ind);
	sprintf(RRT_dim2_SQP_openloop_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_RRT_dim2_SQP_OPENLOOP, curr_thread_ind);
	sprintf(RRT_dim2_SQP_closedloop_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_RRT_dim2_SQP_CLOSEDLOOP, curr_thread_ind);
	sprintf(RRT_dim2_SQP_closedloop_onlylast_filename, "%s/%d_%s_%d",  BASEFOLDER, num_links, BASENAME_RRT_dim2_SQP_CLOSEDLOOP_ONLYLAST, curr_thread_ind);

	Trajectory_Reader linearize_only_reader(linearize_only_filename);
	//Trajectory_Reader interpolate_end_and_linear_reader(interpolate_end_and_linear_filename);
	Trajectory_Reader interpolate_point_and_linearize_reader(interpolate_point_and_linearize_filename);
	Trajectory_Reader sqp_openloop_reader(sqp_openloop_filename);
	Trajectory_Reader sqp_closedloop_reader(sqp_closedloop_filename);
	Trajectory_Reader RRT_reader(RRT_filename);
	Trajectory_Reader RRT_SQP_openloop_reader(RRT_SQP_openloop_filename);
	Trajectory_Reader RRT_SQP_closedloop_reader(RRT_SQP_closedloop_filename);
	Trajectory_Reader RRT_SQP_closedloop_onlylast_reader(RRT_SQP_closedloop_onlylast_filename);
		Trajectory_Reader RRT_dim1_reader(RRT_dim1_filename);
		Trajectory_Reader RRT_dim1_SQP_openloop_reader(RRT_dim1_SQP_openloop_filename);
		Trajectory_Reader RRT_dim1_SQP_closedloop_reader(RRT_dim1_SQP_closedloop_filename);
		Trajectory_Reader RRT_dim1_SQP_closedloop_onlylast_reader(RRT_dim1_SQP_closedloop_onlylast_filename);
		Trajectory_Reader RRT_dim2_reader(RRT_dim2_filename);
		Trajectory_Reader RRT_dim2_SQP_openloop_reader(RRT_dim2_SQP_openloop_filename);
		Trajectory_Reader RRT_dim2_SQP_closedloop_reader(RRT_dim2_SQP_closedloop_filename);
		Trajectory_Reader RRT_dim2_SQP_closedloop_onlylast_reader(RRT_dim2_SQP_closedloop_onlylast_filename);

	linearize_only_reader.read_threads_from_file();
	interpolate_point_and_linearize_reader.read_threads_from_file();
	sqp_openloop_reader.read_threads_from_file();
	sqp_closedloop_reader.read_threads_from_file();
	RRT_reader.read_threads_from_file();
	RRT_SQP_openloop_reader.read_threads_from_file();
	RRT_SQP_closedloop_reader.read_threads_from_file();
	RRT_SQP_closedloop_onlylast_reader.read_threads_from_file();

	if (num_links > 11)
	{
	RRT_dim1_reader.read_threads_from_file();
	RRT_dim1_SQP_openloop_reader.read_threads_from_file();
	RRT_dim1_SQP_closedloop_reader.read_threads_from_file();
	RRT_dim1_SQP_closedloop_onlylast_reader.read_threads_from_file();
	}
	if (num_links > 19)
	{
	RRT_dim2_reader.read_threads_from_file();
	RRT_dim2_SQP_openloop_reader.read_threads_from_file();
	RRT_dim2_SQP_closedloop_reader.read_threads_from_file();
	RRT_dim2_SQP_closedloop_onlylast_reader.read_threads_from_file();
	}

	all_trajs[linearize_only_ind-2] = linearize_only_reader.get_all_threads();
	all_trajs[interpolate_point_and_linearize_ind-2] = interpolate_point_and_linearize_reader.get_all_threads();
	all_trajs[sqp_openloop_ind-2] = sqp_openloop_reader.get_all_threads();
	all_trajs[sqp_closedloop_ind-2] = sqp_closedloop_reader.get_all_threads();
	all_trajs[RRT_ind-2] = RRT_reader.get_all_threads();
	all_trajs[RRT_SQP_openloop_ind-2] = RRT_SQP_openloop_reader.get_all_threads();
	all_trajs[RRT_SQP_closedloop_ind-2] = RRT_SQP_closedloop_reader.get_all_threads();
	all_trajs[RRT_SQP_closedloop_onlylast_ind-2] = RRT_SQP_closedloop_onlylast_reader.get_all_threads();

		all_trajs[RRT_dim1_ind-2] = RRT_dim1_reader.get_all_threads();
		all_trajs[RRT_dim1_SQP_openloop_ind-2] = RRT_dim1_SQP_openloop_reader.get_all_threads();
		all_trajs[RRT_dim1_SQP_closedloop_ind-2] = RRT_dim1_SQP_closedloop_reader.get_all_threads();
		all_trajs[RRT_dim1_SQP_closedloop_onlylast_ind-2] = RRT_dim1_SQP_closedloop_onlylast_reader.get_all_threads();
		all_trajs[RRT_dim2_ind-2] = RRT_dim2_reader.get_all_threads();
		all_trajs[RRT_dim2_SQP_openloop_ind-2] = RRT_dim2_SQP_openloop_reader.get_all_threads();
		all_trajs[RRT_dim2_SQP_closedloop_ind-2] = RRT_dim2_SQP_closedloop_reader.get_all_threads();
		all_trajs[RRT_dim2_SQP_closedloop_onlylast_ind-2] = RRT_dim2_SQP_closedloop_onlylast_reader.get_all_threads();


	curr_trajectory_ind = 0;






	Update_Thread_Displays();
}


void Update_Thread_Displays()
{
	
	/*for (int i=2; i < NUM_THREADS; i++)
	{
		if (curr_trajectory_ind < all_trajs[i-2].size())
		{
			glThreads[i]->setThread(new Thread(all_trajs[i-2][curr_trajectory_ind]));
		}
	}
*/

	int num_total = (int)all_trajs[curThread-2].size();
	int resample_freq = num_total / NUM_TOTAL_DISPLAY;

	int dis_thread_ind = 2;
	for (int i=resample_freq; i < all_trajs[curThread-2].size(); i+= resample_freq)
	{
		glThreads[dis_thread_ind]->setThread(new Thread(all_trajs[curThread-2][i]));
		dis_thread_ind++;
	}
	glThreads[dis_thread_ind-1]->setThread(new Thread(all_trajs[curThread-2][all_trajs[curThread-2].size()-5]));


	glutPostRedisplay();

	//save_opengl_image();



}
/*
void save_opengl_image()
{
    const int IMG_COLS_TOTAL = 900;
    const int IMG_ROWS_TOTAL = 900;
  //playback and save images
    Mat img(900, 900, CV_8UC3);
    vector<Mat> img_planes;
    split(img, img_planes);

    uchar tmp_data[3][900*900];

    GLenum read_formats[3];
    read_formats[0] = GL_BLUE;
    read_formats[1] = GL_GREEN;
    read_formats[2] = GL_RED;

    for (int i=0; i < 3; i++)
    {
        glReadPixels(0, 0, IMG_COLS_TOTAL, IMG_ROWS_TOTAL, read_formats[i], GL_UNSIGNED_BYTE, tmp_data[i]);
        img_planes[i].data = tmp_data[i];
    }


    int p[3];

    p[0] = CV_IMWRITE_PNG_COMPRESSION;
    p[1] = 20;
    p[2] = 0;


    merge(img_planes, img);
    flip(img, img, 0);

    char im_name[256];
    sprintf(im_name, "%s-%d.png", image_save_base_opengl, thread_ind);
    im_save_ind++;
    imwrite(im_name, img, p);
    waitKey(1);
  //sleep(0);
} 

*/

