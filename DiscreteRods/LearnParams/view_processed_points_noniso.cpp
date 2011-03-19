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
#include "../thread_discrete.h"
#include "../trajectory_reader.h"
#include "../../vision/ThreeCam.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

#define TRAJ_BASE_NAME "ribbon_dots_processed"
#define IMG_BASE_DIR "/home/pabbeel/rll/code/trunk/surgical/vision/captures/ribbon_dots"

#define DISPLAY_ORIG_BASE "orig cam"

#define IMG_ROWS_TOTAL 600
#define IMG_COLS_TOTAL 800


// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

using namespace cv;


void InitStuff();
void DrawStuff();
void updateThreadPoints();
void initThread();
void InitStuff();
void display_ims();

#define NUM_PTS 500
#define THREAD_RADII 1.0
#define MOVE_POS_CONST 1.0
#define MOVE_TAN_CONST 0.2
#define ROTATE_TAN_CONST 0.2


enum key_code {NONE, MOVEPOS, MOVETAN, ROTATETAN};

float lastx_L=0;
float lasty_L=0;
float lastx_R=0;
float lasty_R=0;

float rotate_frame[2];
float move_end[2];
float tangent_end[2];
float tangent_rotation_end[2];

Thread* thread;
vector<Vector3d> points;
vector<double> twist_angles;
vector<Matrix3d> material_frames;

double radii[NUM_PTS];
int pressed_mouse_button;

Vector3d positions[2];
Vector3d tangents[2];
Matrix3d rotations[3];

vector<Thread> all_threads;
int thread_ind = 0;
Trajectory_Reader traj_reader(TRAJ_BASE_NAME);

string _names[NUMCAMS];
Capture* _captures[NUMCAMS];
string _orig_display_names[NUMCAMS];
Mat* _frames;
ThreeCam* _cams;

int rows[NUMCAMS];
int cols[NUMCAMS];



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
    move_end[0] += (x-lastx_R)*MOVE_POS_CONST;
    move_end[1] += (lasty_R-y)*MOVE_POS_CONST;
  } else if (key_pressed == MOVETAN)
  {
    tangent_end[0] += (x-lastx_R)*MOVE_TAN_CONST;
    tangent_end[1] += (lasty_R-y)*MOVE_TAN_CONST;
  } else if (key_pressed == ROTATETAN)
  {
    tangent_rotation_end[0] += (x-lastx_R)*ROTATE_TAN_CONST;
    tangent_rotation_end[1] += (lasty_R-y)*ROTATE_TAN_CONST;
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
  else if (key == 'r')
    key_pressed = ROTATETAN;
	else if (key == 'n')
	{
		thread_ind++;
		thread = &all_threads[thread_ind];
		updateThreadPoints();
		DrawStuff();
		_cams->updateImages();
		display_ims();
	}
	else if (key == 'b')
	{
		thread_ind--;
		thread = &all_threads[thread_ind];
		updateThreadPoints();
		DrawStuff();
		for (int camNum = 0; camNum < NUMCAMS; camNum++)
		{
			_captures[camNum]->subtractImageNumber();
			_captures[camNum]->subtractImageNumber();
		}
		_cams->updateImages();
		display_ims();
	}
  else if (key == 'q')
  {
    delete thread;
    glutPostRedisplay ();
  } else if (key == 27)
  {
    exit(0);
  }

  lastx_R = x;
  lasty_R = y;

}

void processKeyUp(unsigned char key, int x, int y)
{
  key_pressed = NONE;
  move_end[0] = move_end[1] = tangent_end[0] = tangent_end[1] = tangent_rotation_end[0] = tangent_rotation_end[1] = 0.0;
}



 void JoinStyle (int msg)
{
	exit (0);
}

void init_contour (void)
{
   int i;

/*	 
   // outline of extrusion 
   i=0;
   CONTOUR (1.0, 1.0);
   CONTOUR (1.0, 2.9);
   CONTOUR (0.9, 3.0);
   CONTOUR (-0.9, 3.0);
   CONTOUR (-1.0, 2.9);

   CONTOUR (-1.0, 1.0);
   CONTOUR (-2.9, 1.0);
   CONTOUR (-3.0, 0.9);
   CONTOUR (-3.0, -0.9);
   CONTOUR (-2.9, -1.0);

   CONTOUR (-1.0, -1.0);
   CONTOUR (-1.0, -2.9);
   CONTOUR (-0.9, -3.0);
   CONTOUR (0.9, -3.0);
   CONTOUR (1.0, -2.9);

   CONTOUR (1.0, -1.0);
   CONTOUR (2.9, -1.0);
   CONTOUR (3.0, -0.9);
   CONTOUR (3.0, 0.9);
   CONTOUR (2.9, 1.0);

   CONTOUR (1.0, 1.0);   // repeat so that last normal is computed 

   */
   // outline of extrusion 
   i=0;
   CONTOUR (1.0, 0.0);
   CONTOUR (1.0, 0.5);
   CONTOUR (1.0, 1.0);
   CONTOUR (1.0, 2.0);
   CONTOUR (1.0, 2.9);
   CONTOUR (0.9, 3.0);
   CONTOUR (0.0, 3.0);
   CONTOUR (-0.9, 3.0);
   CONTOUR (-1.0, 2.9);

   CONTOUR (-1.0, 2.0);
   CONTOUR (-1.0, 1.0);
   CONTOUR (-1.0, 0.5);
   CONTOUR (-1.0, 0.0);
   CONTOUR (-1.0, -0.5);
   CONTOUR (-1.0, -1.0);
   CONTOUR (-1.0, -2.0);
   CONTOUR (-1.0, -2.9);
   CONTOUR (-0.9, -3.0);
   CONTOUR (0.0, -3.0);
   CONTOUR (0.9, -3.0);
   CONTOUR (1.0, -2.9);
   CONTOUR (1.0, -2.0);
   CONTOUR (1.0, -1.0);
   CONTOUR (1.0, -0.5);

   CONTOUR (1.0, 0.0);   // repeat so that last normal is computed 


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
	glutInitWindowSize(600,600);
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
  
  initThread();
 

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

/* draw the helix shape */
void DrawStuff (void)
{
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glColor3f (0.8, 0.3, 0.6);

  glPushMatrix ();

  /* set up some matrices so that the object spins with the mouse */
  glTranslatef (0.0,0.0,-200.0);
  glRotatef (rotate_frame[1], 1.0, 0.0, 0.0);
  glRotatef (rotate_frame[0], 0.0, 0.0, 1.0);


  //change thread, if necessary
  /*if (move_end[0] != 0.0 || move_end[1] != 0.0 || tangent_end[0] != 0.0 || tangent_end[1] != 0.0 || tangent_rotation_end[0] != 0 || tangent_rotation_end[1] != 0)
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
*/

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
 // std::cout << "FINAL TWIST" << twist_cpy[points.size()-2] << std::endl;
  //glePolyCone (points.size(), pts_cpy, 0x0, radii);
/*  gleTwistExtrusion(20,
      contour,
      contour_norms,
      NULL,
      points.size(),
      pts_cpy,
      0x0,
      twist_cpy);
*/
  gleTwistExtrusion(20,
      contour,
      contour_norms,
      NULL,
      points.size(),
      pts_cpy,
      0x0,
      twist_cpy);




  glPopMatrix ();

  glutSwapBuffers ();

	display_ims();
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
}


void initThread()
{
 //read threads from file
  traj_reader.read_threads_from_file();
	all_threads.resize(0);
  for (int i=0; i < traj_reader.get_all_threads().size(); i++)
  {
		all_threads.push_back(traj_reader.get_all_threads()[i]);
  }
  
	for (int i=0; i < NUM_PTS; i++)
	{
		radii[i]=THREAD_RADII;
	}

	thread = &all_threads[thread_ind];
	updateThreadPoints();


	//setup images
  _names[0] = "cam1";
  _names[1] = "cam2";
  _names[2] = "cam3";


	char each_im_base[NUMCAMS][256];
	for (int camNum=0; camNum < NUMCAMS; camNum++)
	{
		sprintf(each_im_base[camNum], "%s%d-",IMG_BASE_DIR, camNum+1);  
	}

  _captures[0] = new Capture(0, //id
          _names[0].c_str(), // cam name
          650,  // gain
          "optimized", // optimized or measured
          107109, // camera uid
					each_im_base[0]);
          //IM_DIR_1); 
  _captures[0]->setExposure(5000);
  //_captures[0]->setExposure(11000);
  

  _captures[1] = new Capture(1, //id
          _names[1].c_str(),// cam name
          650,  // gain
          "optimized", // optimized or measured
          107110, // camera uid
					each_im_base[1]);
  //        IM_DIR_2); 
  _captures[1]->setExposure(3000);
  //_captures[1]->setExposure(7500);

  _captures[2] = new Capture(2, //id
          _names[2].c_str(),// cam name
          650,  // gain
          "optimized", // optimized or measured
          107111, // camera uid
					each_im_base[2]);
//          IM_DIR_3); 
  _captures[2]->setExposure(3500);
  //_captures[2]->setExposure(8000);


  char names_char[NUMCAMS][256];
  for (int camNum=0; camNum < NUMCAMS; camNum++)
  {
    sprintf(names_char[camNum], "%s%d", DISPLAY_ORIG_BASE, camNum);
    _orig_display_names[camNum].assign(names_char[camNum]);
    namedWindow(_orig_display_names[camNum], CV_WINDOW_AUTOSIZE);
  }



  //_captures[0]->init("./calib-apr21/");
  //_captures[1]->init("./calib-apr21/");
  //_captures[2]->init("./calib-apr21/");
  _captures[0]->init("../../vision/calib_params_ribbon/");
  _captures[1]->init("../../vision/calib_params_ribbon/");
  _captures[2]->init("../../vision/calib_params_ribbon/");
  
  
  cvWaitKey(1000);							// segfaulted without this
 

  //add information about other cameras for stereo
  _captures[0]->AddOtherCameraInformation(*_captures[1]);
  _captures[0]->AddOtherCameraInformation(*_captures[2]);
  _captures[1]->AddOtherCameraInformation(*_captures[0]);
  _captures[1]->AddOtherCameraInformation(*_captures[2]);
  _captures[2]->AddOtherCameraInformation(*_captures[0]);
  _captures[2]->AddOtherCameraInformation(*_captures[1]);


  //initialize threecam wrapper
  //thread
  _cams = new ThreeCam(_captures);
  float width[] = {1.50, 1.50, 1.50};
	float edge_sigma[] = {0.50, 0.50, 0.50};
	float blur_sigma[] = {1.5, 1.5, 1.5};
	double thresh1[] = {4.0, 4.0, 4.0};
	double thresh2[] = {80.0, 80.0, 80.0};

	_cams->initializeCanny(width, edge_sigma, blur_sigma, thresh1, thresh2);

  //initialize distance image
  for (int i=0; i < NUMCAMS; i++)
  {
    _frames = _cams->frames();
    rows[i] = _frames[i].rows;
    cols[i] = _frames[i].cols;
  }


	_cams->updateImages();
	display_ims();

}

void display_ims()
{
	
	//project point locations
	Point3f point3d;
	Point2i points2d[NUMCAMS];
	Point2i y_diff; y_diff.x=0;y_diff.y=5;
	Scalar y_diff_color(0, 0, 255);
	Point2i x_diff; x_diff.x=5;x_diff.y=0;
	Scalar x_diff_color(0, 0, 255);
	for (int point_ind=0; point_ind < points.size(); point_ind++)
	{
		EigenToOpencv(points[point_ind], point3d);
		_cams->project3dPoint(point3d, points2d);
		for (int camNum=0; camNum < NUMCAMS; camNum++)
		{
      cv::line(_frames[camNum], points2d[camNum]-x_diff, points2d[camNum]+x_diff, x_diff_color);
      cv::line(_frames[camNum], points2d[camNum]-y_diff, points2d[camNum]+y_diff, y_diff_color);
		}

	}


	//project frames
	Scalar axis_colors[3];
  axis_colors[0] = Scalar(0,0,255);
  axis_colors[1] = Scalar(0,255,0);
  axis_colors[2] = Scalar(255,0,0);

	Vector3d axis_points_eigen[4];
	Point3f axis_points[4];
	Point2i axis_points_2d[4][NUMCAMS];
	
	for (int frame_ind=0; frame_ind < 2; frame_ind++)
	{
		int data_ind;
		double frame_length = 10;
		if (frame_ind == 0)
			data_ind = 0;
		else
			data_ind = points.size()-1;

		axis_points_eigen[0] = points[data_ind];
		axis_points_eigen[1] = axis_points_eigen[0] + frame_length*material_frames[data_ind].col(0);
		axis_points_eigen[2] = axis_points_eigen[0] + frame_length*material_frames[data_ind].col(1);
		axis_points_eigen[3] = axis_points_eigen[0] + frame_length*material_frames[data_ind].col(2);

		for (int point_ind=0; point_ind < 4; point_ind++)
		{
			EigenToOpencv(axis_points_eigen[point_ind], axis_points[point_ind]);
			_cams->project3dPoint(axis_points[point_ind], axis_points_2d[point_ind]);
		}
		
		for (int point_ind=0; point_ind < 3; point_ind++)
		{
			for (int camNum=0; camNum < NUMCAMS; camNum++)
			{
				cv::line(_frames[camNum], axis_points_2d[point_ind+1][camNum], axis_points_2d[0][camNum], axis_colors[point_ind]);
			}
		}

	}


  for (int camNum=0; camNum < NUMCAMS; camNum++)
  {
/*    for (int disp_ind = 0; disp_ind < display_for_debug[i].size(); disp_ind++)
    {
      const Point* pt[] = {display_for_debug[i][disp_ind].pts};

      cv::polylines(_frames[i], pt, &display_for_debug[i][disp_ind].size, 1, false, display_for_debug[i][disp_ind].color, 2);

    }*/


    
    cv::imshow(_orig_display_names[camNum], _frames[camNum]); 

  }
	waitKey(10);
}




