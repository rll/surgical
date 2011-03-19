
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

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

#define TRAJ_BASE_NAME "LearnParams/sutureblack_processed"
#define IMG_BASE_DIR "LearnParams/saved_trajs"

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
void initMotions();
void playbackThread();


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

double radii[NUM_PTS];
int pressed_mouse_button;

Vector3d positions[2];
Vector3d tangents[2];
Matrix3d rotations[3];

vector<Thread> all_threads;
int thread_ind = 0;
Trajectory_Reader traj_reader(TRAJ_BASE_NAME);

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

   /*
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
   */


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
	//thread->minimize_energy();
  //updateThreadPoints();
  
  playbackThread(); 

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
}

void playbackThread()
{
  char im_name[256];
  //read threads from file
  traj_reader.read_threads_from_file();
  all_threads = traj_reader.get_all_threads();

  //initialize data for different viewpoints
  int rotate_frames[4][2];
  rotate_frames[0][0] = -99;
  rotate_frames[0][1] = -32;
  rotate_frames[1][0] = -181;
  rotate_frames[1][1] = -19;
  rotate_frames[2][0] = -97;
  rotate_frames[2][1] = -82;
  rotate_frames[3][0] = -178;
  rotate_frames[3][1] = -69;

  int start_pixels_gl[4][2];
  start_pixels_gl[0][0] = 100;
  start_pixels_gl[0][1] = 75;
  start_pixels_gl[1][0] = 50;
  start_pixels_gl[1][1] = 130;
  start_pixels_gl[2][0] = 100;
  start_pixels_gl[2][1] = 140;
  start_pixels_gl[3][0] = 50;
  start_pixels_gl[3][1] = 140;


  //playback and save images
  Mat img(IMG_ROWS_TOTAL, IMG_COLS_TOTAL, CV_8UC3);
  vector<Mat> img_planes;
  split(img, img_planes);
  namedWindow("topleft", CV_WINDOW_AUTOSIZE);

  uchar tmp_data[3][400*300];

  GLenum read_formats[3];
  read_formats[0] = GL_BLUE;
  read_formats[1] = GL_GREEN;
  read_formats[2] = GL_RED;

  while (thread_ind < all_threads.size())
  {
    std::cout << "displaying " << (thread_ind+1) << " of " << all_threads.size() << std::endl;
    thread = &all_threads[thread_ind];
    updateThreadPoints();
    for (int im_row=0; im_row < 2; im_row++)
    {
      for (int im_col=0; im_col < 2; im_col++)
      {
        int im_num = im_row*2 + im_col;
        rotate_frame[0] = rotate_frames[im_num][0];
        rotate_frame[1] = rotate_frames[im_num][1];
        DrawStuff();

        int start_row = im_row*(IMG_ROWS_TOTAL/2);
        int start_col = im_col*(IMG_COLS_TOTAL/2);

        for (int i=0; i < 3; i++)
        {
          glReadPixels(start_pixels_gl[im_num][0], start_pixels_gl[im_num][1], IMG_COLS_TOTAL/2, IMG_ROWS_TOTAL/2, read_formats[i], GL_UNSIGNED_BYTE, tmp_data[i]);
          for (int r=0; r < IMG_ROWS_TOTAL/2; r++)
          {
            for (int c=0; c < IMG_COLS_TOTAL/2; c++)
            {
              img_planes[i].data[(r+start_row)*IMG_COLS_TOTAL+(c+start_col)] = tmp_data[i][(IMG_ROWS_TOTAL/2-1-r)*IMG_COLS_TOTAL/2+c];
            }
          }
        }
      }
    }
      /*glReadPixels(0, 0, 400, 300, GL_BLUE, GL_UNSIGNED_BYTE, img_planes[0].data);
        glReadPixels(0, 0, 400, 300, GL_GREEN, GL_UNSIGNED_BYTE, img_planes[1].data);
        glReadPixels(0, 0, 400, 300, GL_RED, GL_UNSIGNED_BYTE, img_planes[2].data);*/

  
    merge(img_planes, img);

    imshow("topleft", img);
    sprintf(im_name, "%s/%s-%04d.png", IMG_BASE_DIR, TRAJ_BASE_NAME, thread_ind);
    imwrite(im_name, img);
    waitKey(100);
    //sleep(0);

    thread_ind++;

  } 

  


}
