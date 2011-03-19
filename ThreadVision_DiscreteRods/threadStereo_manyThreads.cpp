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


#define POINTFILE_NYLON "../vision/captures/suturenylon_points.txt" 
#define POINTFILE_PURPLE "../vision/captures/suturepurple_points.txt" 
#define POINTFILE_BLACK "../vision/captures/sutureblack_points.txt" 

#define IMG_SAVE_BASE "saved_results/manyThreads_set/"
#define IMG_SAVE_BASE_BOTHRES "saved_results/manyThreads_set/both_"
#define IMG_SAVE_BASE_VIS "saved_results/manyThreads_set/visonly_"
#define POINTS_ERR_SAVE_BASE "saved_results/manyThreads_set/points_"

#define IMG_SAVE_OPENGL "saved_results/manyThreads_set/opengl_"

#define MAX_LENGTH_VIS 100

#define TRY_ALL false


// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

void InitLights();
void InitGLUT(int argc, char * argv[]);
void InitStuff();
void DrawStuff();
void DrawAxes();
void InitThread(int argc, char* argv[]);
void updateIms(cv::Point3f& start_pt, Vector3d& start_tan, cv::Point3f& end_pt, Vector3d& end_tan);
void findThreadInIms();
void addThreadDebugInfo();
void save_opengl_image();




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

bool show_threads[3];

thread_type _thread_type;

cv::Point3f _start_pt;
cv::Point3f _end_pt;
Vector3d _start_tan;
Vector3d _end_tan;
int curr_im_ind = 1;
Thread_Vision thread_vision;
bool thread_vision_searched = false;
char image_save_base[256];
char image_save_base_both[256];
char image_save_base_vis[256];
char image_save_base_opengl[256];;

Trajectory_Reader traj_reader;
vector<Thread> all_threads;
int thread_ind = 0;
double err_visiononly;
double err_fullopt;
double twistAngle_correct;
double twistAngle_best;
double score_correct_twist;
double score_best_twist;


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
  else if (key == 'n') {
    thread_ind++;
    std::cout << "displaying thread " << thread_ind  << std::endl;
    if (thread_ind >= 0 && thread_ind < all_threads.size())
    {
      glThreads[startThread]->setThread((new Thread(all_threads[thread_ind])));
      glutPostRedisplay();
    }
  } else if (key == 'b') {
    thread_ind--;
    std::cout << "displaying thread " << thread_ind  << std::endl;
    if (thread_ind >= 0 && thread_ind < all_threads.size())
    {
      glThreads[startThread]->setThread((new Thread(all_threads[thread_ind])));
      glutPostRedisplay();
    }
  } else if (key == 'v') {
    //updateIms();
    findThreadInIms();
    glutPostRedisplay();
  } else if (key == '1' && key <= '3')
  {
    show_threads[((int)key-'1')] = !show_threads[((int)key-'1')];
  } else if (key == 'q') {
		thread_vision.prev_hypoth();
    glThreads[optimize_thread]->setThread((new Thread(*thread_vision.curr_thread())));
    glutPostRedisplay();
  } else if (key == 'w') {
		thread_vision.next_hypoth();
    glThreads[optimize_thread]->setThread((new Thread(*thread_vision.curr_thread())));
    glutPostRedisplay();
  }
  
  else if (key == 27) {
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

#ifdef NYLON
  thread_vision.set_reproj_fix_canny(POINTFILE_NYLON);
#elif defined PURPLE
  thread_vision.set_reproj_fix_canny(POINTFILE_PURPLE);
#elif defined BLACK
  thread_vision.set_reproj_fix_canny(POINTFILE_BLACK);
#endif

  // for (int i=0; i < NUM_PTS; i++)
  // {
  //   radii[i]=THREAD_RADII;
  // }

  if (TRY_ALL)
  {
    char filename_errs[256];
#ifdef NYLON
    sprintf(filename_errs, "%s%s.txt", POINTS_ERR_SAVE_BASE, "nylon");
#elif defined PURPLE
    sprintf(filename_errs, "%s%s.txt", POINTS_ERR_SAVE_BASE, "purple");
#elif defined BLACK
    sprintf(filename_errs, "%s%s.txt", POINTS_ERR_SAVE_BASE, "black");
#else
    sprintf(filename_errs, "%s%s.txt", POINTS_ERR_SAVE_BASE, "notype");
#endif
    std::ofstream points_err_out;
    points_err_out.precision(10);
    points_err_out.open(filename_errs);
    while (thread_ind < all_threads.size())
    {
      processNormalKeys('v', lastx_R, lasty_R);

      points_err_out << thread_ind << " " << err_fullopt << " " << err_visiononly << " " << twistAngle_correct << " " <<score_correct_twist << " " << " " << twistAngle_best << " " << score_best_twist << "\n";
      points_err_out.flush();

      processNormalKeys('s', lastx_R, lasty_R);
    }
    points_err_out.close();
    exit(0);
  }




  glutMainLoop ();
}


GLuint sphereList;
void InitStuff (void)
{
  glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
  glEnable(GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  //gleSetJoinStyle (TUBE_NORM_PATH_EDGE | TUBE_JN_ANGLE );
  rotate_frame[0] = 30.0;
  rotate_frame[1] = -75.0;

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
  glTranslatef (50.0,20.0,-120.0);
  glRotatef (rotate_frame[1], 1.0, 0.0, 0.0);
  glRotatef (rotate_frame[0], 0.0, 0.0, 1.0);

  //change thread, if necessary
  if (move_end[0] != 0.0 || move_end[1] != 0.0 || tangent_end[0] != 0.0 || tangent_end[1] != 0.0 || tangent_rotation_end[0] != 0 || tangent_rotation_end[1] != 0)
  {
    glThreads[curThread]->ApplyUserInput(move_end, tangent_end, tangent_rotation_end);
  }


  //Draw Axes
  DrawAxes();
  addThreadDebugInfo();

  glThreads[startThread]->updateThreadPoints();
  Vector3d display_start_pos = glThreads[startThread]->getStartPosition();


  for(int i = 0; i < totalThreads; i++) {
    //Draw Thread
    if (!show_threads[i])
      continue;
    if (i==startThread) {
      glColor4f (0.8, 0.5, 0.0, 1.0);
    } else if (i==just_vis_error_thread) {
      glColor4f (0.8, 0.0, 0.0, 1.0);
    } else if (i==optimize_thread) {
      glColor4f (0.0, 0.0, 0.8, 1.0);
    } else {
      glColor4f (0.5, 0.5, 0.5, 1.0);
    }
    glThreads[i]->display_start_pos = display_start_pos;
    glThreads[i]->DrawThread();
    if (i != just_vis_error_thread)
    {
      //glThreads[i]->DrawAxes();
    }
  }
 
  if (thread_vision_searched)
  {
    thread_vision.display();
    save_opengl_image();
  }


  glPopMatrix ();
  glutSwapBuffers ();
}



void InitThread(int argc, char* argv[])
{
#ifdef NYLON
  _thread_type = nylon;
#elif defined PURPLE
  _thread_type = purple;
#elif defined BLACK
  _thread_type = black;
#else
  _thread_type = black;
#endif

  std::cout << "thread type: " << _thread_type << std::endl;

 if (_thread_type == nylon)
  {
    traj_reader.set_file(TRAJ_BASE_NAME_NYLON);
    sprintf(image_save_base, "%s%s", IMG_SAVE_BASE, "nylon");
    sprintf(image_save_base_both, "%s%s", IMG_SAVE_BASE_BOTHRES, "nylon");
    sprintf(image_save_base_vis, "%s%s", IMG_SAVE_BASE_VIS, "nylon");
    sprintf(image_save_base_opengl, "%s%s", IMG_SAVE_OPENGL, "nylon");
  } else if (_thread_type == purple) {
    traj_reader.set_file(TRAJ_BASE_NAME_PURPLE);
    sprintf(image_save_base, "%s%s", IMG_SAVE_BASE, "purple");
    sprintf(image_save_base_both, "%s%s", IMG_SAVE_BASE_BOTHRES, "purple");
    sprintf(image_save_base_vis, "%s%s", IMG_SAVE_BASE_VIS, "purple");
    sprintf(image_save_base_opengl, "%s%s", IMG_SAVE_OPENGL, "purple");
  } else if (_thread_type == black) {
    traj_reader.set_file(TRAJ_BASE_NAME_BLACK);
    sprintf(image_save_base, "%s%s", IMG_SAVE_BASE, "black");
    sprintf(image_save_base_both, "%s%s", IMG_SAVE_BASE_BOTHRES, "black");
    sprintf(image_save_base_vis, "%s%s", IMG_SAVE_BASE_VIS, "black");
    sprintf(image_save_base_opengl, "%s%s", IMG_SAVE_OPENGL, "black");
  }


  traj_reader.read_threads_from_file();
  all_threads = traj_reader.get_all_threads(); 

  for (int i=0; i < totalThreads; i++)
  {
    glThreads[i] = new GLThread();
    glThreads[i]->setThread(new Thread(all_threads[thread_ind]));
    glThreads[i]->updateThreadPoints();
    if (i == just_vis_error_thread)
      show_threads[i] = false;
    else
      show_threads[i] = true;

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



//New version for search from both ends
void updateIms(Point3f& start_pt, Vector3d& start_tan, Point3f& end_pt, Vector3d& end_tan)
{
  glThreads[startThread]->updateThreadPoints();
  vector<Vector3d> points = glThreads[startThread]->points;

#ifdef FAKEIMS
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
#else
  thread_vision._cams->setImageNumber(thread_ind+1);
#endif
  EigenToOpencv(points.front(), start_pt);
  EigenToOpencv(points.back(), end_pt);

  start_tan = (points[1]-points[0]).normalized();
  end_tan = (points[points.size() - 2] - points[points.size() - 1]).normalized();

}

/*
void findThreadInIms()
{
  thread_vision_searched = true;

  vector<Vector3d> points_im;
  vector<double> angle_im;
  vector<Vector3d> points_real;
  vector<double> angle_real;

  glThreads[startThread]->getThread()->get_thread_data(points_real, angle_real);


  thread_vision.set_max_length(MAX_LENGTH_VIS);
  thread_vision.clear_display();
 
  updateIms(_start_pt, _start_tan);
  thread_vision.setInitPt(_start_pt);
  thread_vision.setInitTan(_start_tan);



//  if (thread_vision.optimizeThread(true))
//  {
//    std::cout << "found thread vis only" << std::endl;
//    glThreads[just_vis_error_thread]->setThread(new Thread(*thread_vision.curr_thread()));
//    glThreads[just_vis_error_thread]->printThreadData();
//    //glThreads[optimize_thread]->updateThreadPoints();
//  }
//  thread_vision.addThreadPointsToDebugImages(Scalar(0,0,200));
//  thread_vision.add_debug_points_to_ims();
//  thread_vision.saveImages(image_save_base_vis, thread_ind+1);
// 
//  thread_vision.get_thread_data(points_im, angle_im);
//  err_visiononly = calculate_vector_norm_avg(points_im, points_real)/points_im.size();



  thread_vision.clear_display();
  updateIms(_start_pt, _start_tan);
  thread_vision.setInitPt(_start_pt);
  thread_vision.setInitTan(_start_tan);
  if (thread_vision.optimizeThread(true))
  {
    std::cout << "found thread full opt" << std::endl;
    glThreads[optimize_thread]->setThread(new Thread(*thread_vision.curr_thread()));
    glThreads[just_vis_error_thread]->setThread(new Thread(*thread_vision.curr_thread()));
    //glThreads[optimize_thread]->updateThreadPoints();

    thread_vision.addThreadPointsToDebugImages(Scalar(200,0,0));
    thread_vision.add_debug_points_to_ims();

    //thread_vision.display();
    thread_vision.saveImages(image_save_base, thread_ind+1);


    thread_vision.clear_display();
    thread_vision.addThreadPointsToDebugImages(Scalar(0,0,200), glThreads[just_vis_error_thread]->getThread());
    thread_vision.addThreadPointsToDebugImages(Scalar(200,0,0));
    thread_vision.add_debug_points_to_ims();
    thread_vision.saveImages(image_save_base_both, thread_ind+1);


    thread_vision.get_thread_data(points_im, angle_im);
    err_fullopt = calculate_vector_norm_avg(points_im, points_real)/points_im.size();
    std::cout << "err: " << err_fullopt<< "\t\tvision only: " << err_visiononly << std::endl;

    twistAngle_correct = glThreads[startThread]->getThread()->end_angle();
    twistAngle_best = thread_vision.end_angle();

    double closest_angle_diff = DBL_MAX;
    double score_closest;
    double best_score = DBL_MAX;
    for (int i=0; i < thread_vision.twist_scores.size(); i++)
    {
      if ( abs(thread_vision.twist_scores[i].twist_angle - glThreads[startThread]->getThread()->end_angle()) < closest_angle_diff)
      {
        closest_angle_diff = abs(thread_vision.twist_scores[i].twist_angle - glThreads[startThread]->getThread()->end_angle());
        score_closest = thread_vision.twist_scores[i].score;
      }

      best_score = min(best_score, thread_vision.twist_scores[i].score);
    }

    score_correct_twist = score_closest;
    score_best_twist = best_score;

  }

  glThreads[startThread]->printThreadData();
}
*/


void findThreadInIms()
{
  thread_vision_searched = true;

  vector<Vector3d> points_im;
  vector<double> angle_im;
  vector<Vector3d> points_real;
  vector<double> angle_real;

  glThreads[startThread]->getThread()->get_thread_data(points_real, angle_real);


  thread_vision.set_max_length(MAX_LENGTH_VIS);
  thread_vision.clear_display();
 
  updateIms(_start_pt, _start_tan, _end_pt, _end_tan);
  thread_vision.addStartData(_start_pt, _start_tan);
  thread_vision.addStartData(_end_pt, _end_tan);


  if (thread_vision.optimizeThread())
  {
    std::cout << "found thread full opt" << std::endl;
    glThreads[optimize_thread]->setThread(new Thread(*thread_vision.curr_thread()));

    thread_vision.addThreadPointsToDebugImages(Scalar(200,0,0));
    thread_vision.add_debug_points_to_ims();

    //thread_vision.display();
    thread_vision.saveImages(image_save_base, thread_ind+1);


    thread_vision.get_thread_data(points_im, angle_im);
    err_fullopt = calculate_vector_norm_avg(points_im, points_real)/points_im.size();
    std::cout << "err: " << err_fullopt << std::endl;

    twistAngle_correct = glThreads[startThread]->getThread()->end_angle();
    twistAngle_best = thread_vision.end_angle();

  }

}



void addThreadDebugInfo()
{
Vector3d display_start_pos = glThreads[startThread]->getStartPosition();
  for (int i=0; i < thread_vision.gl_display_for_debug.size(); i++)
  {
    glline_draw_params& currP = thread_vision.gl_display_for_debug[i];;
    glBegin(GL_LINE_STRIP);
    glColor4f(currP.color[0], currP.color[1], currP.color[2], 1.0);
    for (int j=0; j < currP.size; j++)
    {
      glVertex3f(currP.vertices[j].x-display_start_pos(0), currP.vertices[j].y-display_start_pos(1), currP.vertices[j].z-display_start_pos(2));
    }
    glEnd();
    
  }


}


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


  merge(img_planes, img);
  flip(img, img, 0);

  char im_name[256];
  sprintf(im_name, "%s-%d.jpg", image_save_base_opengl, thread_ind+1);
  imwrite(im_name, img);
  waitKey(1);
  //sleep(0);


} 

  

