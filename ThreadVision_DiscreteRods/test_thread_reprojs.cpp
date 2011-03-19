
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
#include "../DiscreteRods/thread_discrete.h"
#include "../DiscreteRods/trajectory_reader.h"
#include "../DiscreteRods/glThread.h"
#include "thread_vision_discrete.h"

#define TRAJ_BASE_NAME_NYLON "../DiscreteRods/LearnParams/config/suturenylon_processed_projected"
#define TRAJ_BASE_NAME_PURPLE "../DiscreteRods/LearnParams/config/suturepurple_processed_projected"
#define TRAJ_BASE_NAME_BLACK "../DiscreteRods/LearnParams/config/sutureblack_processed_projected"
#define MAX_LENGTH_VIS 100



// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

void InitLights();
void InitGLUT(int argc, char * argv[]);
void InitStuff();
void DrawStuff();
void DrawAxes();
void InitThread(int argc, char* argv[]);
void updateIms(cv::Point3f& start_pt);
void findThreadInIms();




#define NUM_PTS 500
// #define THREAD_RADII 1.0
#define MOVE_POS_CONST 1.0
#define MOVE_TAN_CONST 0.2
#define ROTATE_TAN_CONST 0.2


enum key_code {NONE, MOVEPOS, MOVETAN, ROTATETAN};
enum thread_type {black, nylon, purple};

float lastx_L=0;
float lasty_L=0;
float lastx_R=0;
float lasty_R=0;

float rotate_frame[2];
float move_end[2];
float tangent_end[2];
float tangent_rotation_end[2];

GLThread* glThreads[3];
int totalThreads = 3;
int curThread = 0;
int startThread = 0;
int optimize_thread = 1;
int just_vis_error_thread = 2;

thread_type _thread_type;

cv::Point3f _start_pt;
int curr_im_ind = 1;
Thread_Vision thread_vision;

Trajectory_Reader traj_reader[3];
vector<Thread> all_threads[3];
int thread_ind = 0;


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


void applyControl(Thread* start, const VectorXd& u, VectorXd* res) {
  int N = glThreads[startThread]->getThread()->num_pieces();
  res->setZero(3*N);

  Vector3d translation;
  translation << u(0), u(1), u(2);

  double dw = 1.0 - u(3)*u(3) - u(4)*u(4) - u(5)*u(5);
  if (dw < 0) {
    cout << "Huge differential quaternion: " << endl;
  }
  dw = (dw > 0) ? dw : 0.0;
  dw = sqrt(dw);
  Eigen::Quaterniond q(dw, u(3),u(4),u(5));
  Matrix3d rotation(q);

  // apply the control u to thread start, and return the new config in res
  Frame_Motion toMove(translation, rotation);

  Vector3d end_pos = start->end_pos();
  Matrix3d end_rot = start->end_rot();
  toMove.applyMotion(end_pos, end_rot);
  start->set_end_constraint(end_pos, end_rot);
  start->minimize_energy();

  start->toVector(res);
}

void computeDifference(Thread* a, Thread* b, VectorXd* res) {
  VectorXd avec;
  a->toVector(&avec);
  VectorXd bvec;
  b->toVector(&bvec);

  (*res) = avec - bvec;
}


void selectThread(int inc) {
  curThread = (curThread + inc) % totalThreads;
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


void processSpecialKeys(int key, int x, int y) {
 /*if (key == GLUT_KEY_LEFT) {
   if(initialized) {
     if (curNode->prev != NULL) {
       curNode = curNode->prev;
       glThreads[7]->setThread(new Thread(curNode->x, VectorXd::Zero(curNode->N/3), Matrix3d::Identity()));
       glThreads[7]->minimize_energy();
       glutPostRedisplay();
     }
   }
 } else if (key == GLUT_KEY_RIGHT) {
   if(initialized) {
     if (curNode->next != NULL) {
       curNode = curNode->next;
       glThreads[7]->setThread(new Thread(curNode->x, VectorXd::Zero(curNode->N/3), Matrix3d::Identity()));
       glThreads[7]->minimize_energy();
       glutPostRedisplay();
     }
   }
 }
 */
}

void processNormalKeys(unsigned char key, int x, int y)
{
  if (key == 't') {
    key_pressed = MOVETAN;
  }
  else if (key == 'm') {
    key_pressed = MOVEPOS;
  }
  else if (key == 'r') {
    key_pressed = ROTATETAN;
  }
  else if (key == 's') {
    thread_ind++;
    for (int i=0; i < totalThreads; i++)
    {
      if (thread_ind < all_threads[i].size()-1)
      {
        glThreads[i]->setThread((new Thread(all_threads[i][thread_ind])));
        glutPostRedisplay();
      }
    }
  } else if (key == 'b') {
    thread_ind--;
    for (int i=0; i < totalThreads; i++)
    {
      if (thread_ind < all_threads[i].size()-1)
      {
        glThreads[i]->setThread((new Thread(all_threads[i][thread_ind])));
        glutPostRedisplay();
      }
    }


  }
  else if (key == 'v') {
    //updateIms();
    findThreadInIms();
    glutPostRedisplay();
  } else if (key == 27) {
    exit(0);
  }

  lastx_R = x;
  lasty_R = y;

}

void processKeyUp(unsigned char key, int x, int y)
{
  if (key == '+') {
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



  InitThread(argc, argv);


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
  glTranslatef (0.0,0.0,-150.0);
  glRotatef (rotate_frame[1], 1.0, 0.0, 0.0);
  glRotatef (rotate_frame[0], 0.0, 0.0, 1.0);


  //change thread, if necessary
  if (move_end[0] != 0.0 || move_end[1] != 0.0 || tangent_end[0] != 0.0 || tangent_end[1] != 0.0 || tangent_rotation_end[0] != 0 || tangent_rotation_end[1] != 0)
  {
    glThreads[curThread]->ApplyUserInput(move_end, tangent_end, tangent_rotation_end);
  }


  //Draw Axes
  DrawAxes();



  for(int i = 0; i < totalThreads; i++) {
    //Draw Thread
    if (i==0) {
      glColor4f (1.0, 0.0, 0.0, 0.5);
    } else if (i==1) {
      glColor4f (0.0, 1.0, 0.0, 0.5);
    } else if (i==2) {
      glColor4f (0.0, 0.0, 1.0, 0.5);
    } else {
      glColor4f (0.5, 0.5, 0.1, 0.5);
    }
    glThreads[i]->DrawThread();
    if (i != just_vis_error_thread)
    {
      glThreads[i]->DrawAxes();
    }
  }

  glPopMatrix ();
  glutSwapBuffers ();
}



void InitThread(int argc, char* argv[])
{
  if (argc < 2)
    _thread_type = nylon;
  else if (strcmp(argv[1], "nylon")==0)
    _thread_type = nylon;
  else if (strcmp(argv[1], "purple")==0)
    _thread_type = purple;
  else if (strcmp(argv[1], "black")==0)
    _thread_type = black;
  else
    _thread_type = nylon;

  traj_reader[0].set_file(TRAJ_BASE_NAME_NYLON);
  traj_reader[1].set_file(TRAJ_BASE_NAME_PURPLE);
  traj_reader[2].set_file(TRAJ_BASE_NAME_BLACK);


  for (int i=0; i < totalThreads; i++)
  {
    traj_reader[i].read_threads_from_file();
    all_threads[i] = traj_reader[i].get_all_threads();
    glThreads[i] = new GLThread();
    glThreads[i]->setThread(new Thread(all_threads[i][thread_ind]));
    glThreads[i]->updateThreadPoints();


    if (_thread_type == nylon)
    {
      glThreads[i]->to_set_bend = 232.0000;
      glThreads[i]->to_set_twist = 847.000;
      glThreads[i]->to_set_grav = 1e-4;
    } else if (_thread_type == purple) {
      glThreads[i]->to_set_bend = 17.0000;
      glThreads[i]->to_set_twist = 54.6250;
      glThreads[i]->to_set_grav = 1e-4;
    } else if (_thread_type == black) {
      glThreads[i]->to_set_bend = 123.5000;
      glThreads[i]->to_set_twist = 209.250;
      glThreads[i]->to_set_grav = 1e-4;
    }


  }
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
  glutSpecialFunc(processSpecialKeys);
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




void updateIms(Point3f& start_pt)
{
  glThreads[startThread]->updateThreadPoints();
  vector<Vector3d> points = glThreads[startThread]->points;

  int num_pts = 300;

  vector<cv::Point3f> points_to_proj;

  for (int i=0; i < points.size()-1; i++)
  {
    Vector3d vec_between = points[i+1]-points[i];
    for (int j=0; j <= num_pts; j++)
    {
      Vector3d nextPoint = points[i]+( ((double)j)/((double)num_pts))*vec_between;
      cv::Point3f toAdd((float)nextPoint(0), (float)nextPoint(1), (float)nextPoint(2));
      points_to_proj.push_back(toAdd);
    }
  }

  Mat ims[3];
  for (int i=0; i < 3; i++){
    ims[i] = cv::Mat::zeros(thread_vision._frames[i].size(), CV_8UC3);
  }

  Point2i points2d[NUMCAMS];
  for (int i=0; i < points_to_proj.size(); i++)
  {
    thread_vision._cams->project3dPoint(points_to_proj[i], points2d);
    for (int j=0; j < NUMCAMS; j++)
    {
      if (thread_vision._captures[j]->inRange(points2d[j].y, points2d[j].x))
      {
        ims[j].at<Vec3b>(points2d[j].y, points2d[j].x)[0] = (unsigned char)255;
        ims[j].at<Vec3b>(points2d[j].y, points2d[j].x)[1] = (unsigned char)255;
        ims[j].at<Vec3b>(points2d[j].y, points2d[j].x)[2] = (unsigned char)255;
      }
    }

  }

  start_pt = points_to_proj[0];

 /* imshow("1", ims[0]);
  imshow("2", ims[1]);
  imshow("3", ims[2]);
*/

  char im_name[256];
  for (int cam_ind = 0; cam_ind < NUMCAMS; cam_ind++)
  {
    sprintf(im_name, "./stereo_test/stereo_test%d-%d.tif", cam_ind+1, curr_im_ind);
    imwrite(im_name, ims[cam_ind]);
  }
  curr_im_ind++;


}

void findThreadInIms()
{
  thread_vision.set_max_length(MAX_LENGTH_VIS);
  updateIms(_start_pt);
  thread_vision.setInitPt(_start_pt);
  if (thread_vision.optimizeThread())
  {
    std::cout << "found thread" << std::endl;
    glThreads[optimize_thread]->setThread(new Thread(thread_vision));
    //glThreads[optimize_thread]->updateThreadPoints();
  }
  thread_vision.display();


  updateIms(_start_pt);
  thread_vision.setInitPt(_start_pt);
  if (thread_vision.optimizeThread(true))
  {
    std::cout << "found thread" << std::endl;
    glThreads[just_vis_error_thread]->setThread(new Thread(thread_vision));
    //glThreads[optimize_thread]->updateThreadPoints();
  }

  glThreads[startThread]->printThreadData();


}
