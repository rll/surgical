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
#include "thread_socket_interface.h"


// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN


void initStuff();
void drawStuff();
void changeStartThreadHaptic();
void changeEndThreadHaptic();
void changeThreadHaptic();
void changeStartThreadMouse();
void changeEndThreadMouse();
void changeThreadMouse();
void drawAxes();
void drawThread();
void drawCursor(int device_id, float color);
void updateThreadPoints();
void initThread();
void initThread_closedPolygon();
void glutMenu(int ID);
void initContour ();
void initGL();

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
float last_rotate_frame[2];

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

static double gCursorScale = 6;
static GLuint gCursorDisplayList = 0;
double start_proxyxform[16] = {-0.1018,-0.9641,0.2454,0.0000,0.0806,0.2379,0.9680,0.0000,-0.9915,0.1183,0.0534,0.0000,-0.5611,-0.1957,-0.8401,1.0000};
double end_proxyxform[16] = {0.5982,0.7791,-0.1877,0.0000,0.3575,-0.0498,0.9326,0.0000,0.7172,-0.6250,-0.3083,0.0000,1.1068,0.2221,-0.7281,1.0000};
Vector3d end_proxy_pos;
Vector3d start_proxy_pos;
Matrix3d end_proxy_rot;
Matrix3d start_proxy_rot;
bool start_haptics_mode = false, end_haptics_mode = false;
bool start_proxybutton = true, end_proxybutton = true;
bool last_start_proxybutton = true, last_end_proxybutton = true;
Matrix3d rot_ztox, rot_ztonegx;

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
  else if (key == 's')
  {
    delete thread_saved;
    thread_saved = new Thread(*thread);
  } else if (key == 'S')  {
    Thread* temp = thread;
    thread = thread_saved;
    thread_saved = temp;
    updateThreadPoints();
    glutPostRedisplay();
  } else if (key == 'w') {
    rotate_frame[0] = 0.0;
    rotate_frame[1] = 0.0;
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
  } else if(key == 'n') {
        cout << "Stepping " << endl;
        thread->minimize_energy();
        drawStuff();
  } else if(key == 'l') {
        cout << "Stepping though project length constraint" << endl;
        thread->project_length_constraint();
        drawStuff();
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

void processStartProxybutton() {
    if (!last_start_proxybutton && start_proxybutton) {
        start_haptics_mode = !start_haptics_mode;
    }
    last_start_proxybutton = start_proxybutton;
}

void processEndProxybutton() {
    if (!last_end_proxybutton && end_proxybutton) {
        end_haptics_mode = !end_haptics_mode;
    }
    last_end_proxybutton = end_proxybutton;
}            

void processHapticDevice(int value)
{
    getDeviceState (start_proxyxform, start_proxybutton, end_proxyxform, end_proxybutton);
    
    //cout << start_proxyxform[12] << "\t" <<  start_proxyxform[13] << "\t" <<  start_proxyxform[14] << endl;
    start_proxyxform[12] *= 30;
    end_proxyxform[12]   *= 30;
    start_proxyxform[13] *= 30;
    end_proxyxform[13]   *= 30;
    start_proxyxform[14] *= 60;
    end_proxyxform[14]   *= 60;
    start_proxyxform[14] -= 6; //104;
    end_proxyxform[14]   -= 6; //104;
    //cout << start_proxyxform[12] << "\t" <<  start_proxyxform[13] << "\t" <<  start_proxyxform[14] << endl;
    for(int i=0; i<3; i++) {
        end_proxy_pos(i)   = end_proxyxform[i+12];
        start_proxy_pos(i) = start_proxyxform[i+12];
    }
    
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            start_proxy_rot(j,i) = start_proxyxform[i*4+j];
            end_proxy_rot(j,i) = end_proxyxform[i*4+j];
        }
    }
        
    start_proxy_rot = start_proxy_rot * rot_ztox;
    end_proxy_rot = end_proxy_rot * rot_ztonegx;
    
    processStartProxybutton();
    processEndProxybutton();
    
    //cout << "mode:\t" << start_proxybutton << "\t" << end_proxybutton << "\t" << start_haptics_mode << "\t" << end_haptics_mode << endl;

    glutPostRedisplay ();
    glutTimerFunc(100, processHapticDevice, value);
}

int main (int argc, char * argv[])
{
  srand(time(NULL));
  srand((unsigned int)time((time_t *)NULL));

  printf("Instructions:\n"
      "Hold down the left mouse button to rotate view.\n"
      "\n"
      "Each end can be controlled either by the Phantom or by the mouse and keyboard.\n"
      "Initially, the ends are controlled by the Phantoms.\n"
      "To change the control input for an end, press the dark gray button of the respective Phantom.\n"
      "\n"
      "\tPhantom control input:\n"
      "\tMove the Phantom.\n"
      "\n"
      "\tMouse and keyboard control input:\n"
      "\tHold 'm' while holding down the left or right mouse button to move the end.\n"
      "\tHold 't' while holding down the left or right mouse button to orientate the tangent.\n"
      "\tHold 'r' while holding down the left or right mouse button to roll the tangent.\n"
      "\n"
      "Press 'w' to reset view.\n"
      "Press 'q' to quit.\n"
      );

	/* initialize glut */
	glutInit (&argc, argv); //can i do that?
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(900,900);
	glutCreateWindow ("Thread");
	glutDisplayFunc (drawStuff);
	glutMotionFunc (MouseMotion);
    glutMouseFunc (processMouse);
    glutKeyboardFunc(processNormalKeys);
    glutKeyboardUpFunc(processKeyUp);
    glutTimerFunc(100, processHapticDevice, 0);

	/* create popup menu */
	glutCreateMenu (glutMenu);
	glutAddMenuEntry ("Exit", 99);
	glutAttachMenu (GLUT_MIDDLE_BUTTON);

	initGL();
	initStuff ();

    initThread();
	thread->minimize_energy();
    updateThreadPoints();
    thread_saved = new Thread(*thread);

    zero_location = points[0];
    zero_angle = 0.0;

	for (int i=0; i < NUM_PTS; i++)
	{
		radii[i]=THREAD_RADII;
	}

	connectionInit();

    glutMainLoop ();
	//   return 0;             /* ANSI C requires main to return int. */
}

void drawStuff (void)
{
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glColor3f (0.8, 0.3, 0.6);

  glPushMatrix ();
  
  /* set up some matrices so that the object spins with the mouse */
  glTranslatef (0.0,0.0,-110.0);
  glRotatef (rotate_frame[1], 1.0, 0.0, 0.0);
  glRotatef (rotate_frame[0], 0.0, 0.0, 1.0);
  
  drawCursor(0, 1.0);
  drawCursor(1, 0.5);
  /*
  cout << "start_proxy_pos:\n" << start_proxy_pos << endl;
  cout << "start_proxy_rot:\n" << start_proxy_rot << endl;
  cout << "vertex_at_ind(3):\n" << thread->vertex_at_ind(3) << endl;
  cout << "material_at_ind(3):\n" << thread->material_at_ind(3) << endl;
  */
  if (start_haptics_mode && end_haptics_mode) {
    changeThreadHaptic();
  } else if (start_haptics_mode && !end_haptics_mode) {
    changeStartThreadHaptic();
    changeEndThreadMouse();
  } else if (!start_haptics_mode && end_haptics_mode) {
    changeEndThreadHaptic();
    changeStartThreadMouse(); 
  } else if (!start_haptics_mode && !end_haptics_mode) {
    changeThreadMouse();
  }
  
  cout << start_proxy_pos << endl;
  changeThreadMouse();
  
  thread->minimize_energy();
  
  updateThreadPoints();
  drawAxes();
  drawThread();
  
  //glRotatef (-rotate_frame[0], 0.0, 0.0, 1.0);
  //glRotatef (-rotate_frame[1], 1.0, 0.0, 0.0);
  
  glPopMatrix ();

  glutSwapBuffers ();
}

void changeStartThreadHaptic() {
  zero_angle += angle_mismatch(start_proxy_rot, thread->start_rot());
  thread->set_constraints_nearEnds(start_proxy_pos, start_proxy_rot, positions[1], rotations[1]);
}

void changeEndThreadHaptic() {
  thread->set_constraints_nearEnds(positions[0], rotations[0], end_proxy_pos, end_proxy_rot);
}

void changeThreadHaptic() {
  zero_angle += angle_mismatch(start_proxy_rot, thread->start_rot());
  thread->set_constraints_nearEnds(start_proxy_pos, start_proxy_rot, end_proxy_pos, end_proxy_rot);
}

void changeStartThreadMouse() {
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

    //change start positions
    Vector3d new_start_pos;
    gluProject(positions[0](0), positions[0](1), positions[0](2), model_view, projection, viewport, &winX, &winY, &winZ);
    winX += move_start[0];
    winY += move_start[1];
    move_start[0] = 0.0;
    move_start[1] = 0.0;
    gluUnProject(winX, winY, winZ, model_view, projection, viewport, &new_start_pos(0), &new_start_pos(1), &new_start_pos(2));

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

    motion_to_apply._start._pos_movement = new_start_pos-positions[0];

    Matrix3d rotation_new_start_tan;
    rotate_between_tangents(tangents[0], new_start_tan, rotation_new_start_tan);
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
    
    motion_to_apply._end.set_nomotion();

    //change thread
    thread->apply_motion_nearEnds(motion_to_apply);
  }
}

void changeEndThreadMouse() {
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

    //change end positions
    Vector3d new_end_pos;
    gluProject(positions[1](0), positions[1](1), positions[1](2), model_view, projection, viewport, &winX, &winY, &winZ);
    winX += move_end[0];
    winY += move_end[1];
    move_end[0] = 0.0;
    move_end[1] = 0.0;
    gluUnProject(winX, winY, winZ, model_view, projection, viewport, &new_end_pos(0), &new_end_pos(1), &new_end_pos(2));

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

    motion_to_apply._end._pos_movement = new_end_pos-positions[1];

    Matrix3d rotation_new_tan;
    rotate_between_tangents(tangents[1], new_end_tan, rotation_new_tan);
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

    motion_to_apply._start.set_nomotion();

    //change thread
    thread->apply_motion_nearEnds(motion_to_apply);
  }
}

void changeThreadMouse() {
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

    //change end positions
    Vector3d new_end_pos;
    gluProject(positions[1](0), positions[1](1), positions[1](2), model_view, projection, viewport, &winX, &winY, &winZ);
    winX += move_end[0];
    winY += move_end[1];
    move_end[0] = 0.0;
    move_end[1] = 0.0;
    gluUnProject(winX, winY, winZ, model_view, projection, viewport, &new_end_pos(0), &new_end_pos(1), &new_end_pos(2));

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

    motion_to_apply._end._pos_movement = new_end_pos-positions[1];

    Matrix3d rotation_new_tan;
    rotate_between_tangents(tangents[1], new_end_tan, rotation_new_tan);
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

    motion_to_apply._start._pos_movement = new_start_pos-positions[0];

    Matrix3d rotation_new_start_tan;
    rotate_between_tangents(tangents[0], new_start_tan, rotation_new_start_tan);
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
    thread->apply_motion_nearEnds(motion_to_apply);
  }
}


void drawAxes() {
  glPushMatrix();
  
  //Draw Axes at Start
  Vector3d diff_pos = positions[0]-zero_location;
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

  //Draw Axes at End
  diff_pos = positions[1]-zero_location;
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
  glEnd( );

  //label axes
  diff_pos = positions[0]-zero_location;
  rotation_scale_factor = 15.0;
  rotations_project = rotations[0]*rotation_scale_factor;
  void * font = GLUT_BITMAP_HELVETICA_18;
  glColor3d(1.0, 0.0, 0.0); //red
  //glRasterPos3i(20.0, 0.0, -1.0);
  glRasterPos3i((float)(diff_pos(0)+rotations_project(0,0)), (float)(diff_pos(1)+rotations_project(1,0)), (float)(diff_pos(2)+rotations_project(2,0)));
  glutBitmapCharacter(font, 'X');
  glColor3d(0.0, 1.0, 0.0); //red
  //glRasterPos3i(0.0, 20.0, -1.0);
  glRasterPos3i((float)(diff_pos(0)+rotations_project(0,1)), (float)(diff_pos(1)+rotations_project(1,1)), (float)(diff_pos(2)+rotations_project(2,1)));
  glutBitmapCharacter(font, 'Y');
  glColor3d(0.0, 0.0, 1.0); //red
  //glRasterPos3i(-1.0, 0.0, 20.0);
  glRasterPos3i((float)(diff_pos(0)+rotations_project(0,2)), (float)(diff_pos(1)+rotations_project(1,2)), (float)(diff_pos(2)+rotations_project(2,2)));
  glutBitmapCharacter(font, 'Z');
  
  glPopMatrix();
}

void drawThread() {
  glPushMatrix();
  
  glColor3f (0.5, 0.5, 0.2);

  double pts_cpy[points.size()+2][3];
  double twist_cpy[points.size()+2];
  for (int i=0; i < points.size(); i++)
  {
    pts_cpy[i+1][0] = points[i](0)-(double)zero_location(0);
    pts_cpy[i+1][1] = points[i](1)-(double)zero_location(1);
    pts_cpy[i+1][2] = points[i](2)-(double)zero_location(2);
    twist_cpy[i+1] = -(360.0/(2.0*M_PI))*(twist_angles[i]+zero_angle);
  }
  //add first and last point
  pts_cpy[0][0] = pts_cpy[1][0]-rotations[0](0,0);
  pts_cpy[0][1] = pts_cpy[1][1]-rotations[0](1,0);
  pts_cpy[0][2] = pts_cpy[1][2]-rotations[0](2,0);
  twist_cpy[0] = -(360.0/(2.0*M_PI))*(zero_angle);

  pts_cpy[points.size()+1][0] = pts_cpy[points.size()][0]+rotations[1](0,0);
  pts_cpy[points.size()+1][1] = pts_cpy[points.size()][1]+rotations[1](1,0);
  pts_cpy[points.size()+1][2] = pts_cpy[points.size()][2]+rotations[1](2,0);
  twist_cpy[points.size()+1] = twist_cpy[points.size()]-(360.0/(2.0*M_PI))*zero_angle;

  gleTwistExtrusion(20,
      contour,
      contour_norms,
      NULL,
      points.size()+2,
      pts_cpy,
      0x0,
      twist_cpy);
  
  glPopMatrix ();
}

void drawCursor(int device_id, float color)
{
    static const double kCursorRadius = 0.5;
    static const double kCursorHeight = 1.5;
    static const int kCursorTess = 4;
    
    GLUquadricObj *qobj = 0;

    glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT);
    glPushMatrix();

    if (!gCursorDisplayList)
    {
        gCursorDisplayList = glGenLists(1);
        glNewList(gCursorDisplayList, GL_COMPILE);
        qobj = gluNewQuadric();
               
        gluCylinder(qobj, 0.0, kCursorRadius, kCursorHeight,
                    kCursorTess, kCursorTess);
        glTranslated(0.0, 0.0, kCursorHeight);
        gluCylinder(qobj, kCursorRadius, 0.0, kCursorHeight / 5.0,
                    kCursorTess, kCursorTess);
    
        gluDeleteQuadric(qobj);
        glEndList();
    }
    
    /* Get the proxy transform in world coordinates */
    if (device_id) {
        glMultMatrixd(end_proxyxform);
    } else {
        glMultMatrixd(start_proxyxform);
    }
    
    /* Apply the local cursor scale factor. */
    glScaled(gCursorScale, gCursorScale, gCursorScale);

    glEnable(GL_COLOR_MATERIAL);
    glColor3f(0.0, color, 1.0);

    glCallList(gCursorDisplayList);

    glPopMatrix(); 
    
    glPopAttrib();
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
  
  //printf("%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n", twist_angles[0],
  //  twist_angles[1], twist_angles[2], twist_angles[3], twist_angles[4], twist_angles[5], twist_angles[6],
  //  twist_angles[7], twist_angles[8], twist_angles[9], twist_angles[10], twist_angles[11], twist_angles[12],
  //  twist_angles[13], twist_angles[14], twist_angles[15]);
  
  twist_angles.back() = 2.0*twist_angles[twist_angles.size()-2] - twist_angles[twist_angles.size()-3];
  
  
  
}

void initStuff (void)
{
  glEnable(GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  rotate_frame[0] = 0.0;
  rotate_frame[1] = 0.0; //-111.0;
  
  for(int i=0; i<3; i++) {
    for(int j=0; j<3; j++) {
      rot_ztox(i,j) = 0;
      rot_ztonegx(i,j) = 0;
    }
  }
  rot_ztox(0,2) = 1;
  rot_ztox(1,1) = 1;
  rot_ztox(2,0) = -1;
  rot_ztonegx(0,2) = -1;
  rot_ztonegx(1,1) = 1;
  rot_ztonegx(2,0) = 1;
  
  initContour();
}

void initThread()
{
  int numInit = 4; //14  there are 2*numInit+3 vertices
  double noise_factor = 0.0;

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
  direction(2) = -2.0;
  direction.normalize();
  for (int i=0; i < numInit; i++)
  {
    Vector3d noise( ((double)(rand()%10000)) / 10000.0, ((double)(rand()%10000)) / 10000.0, ((double)(rand()%10000)) / 10000.0);
    noise *= noise_factor;
    Vector3d next_Vec = vertices.back()+(direction+noise).normalized()*DEFAULT_REST_LENGTH;
    vertices.push_back(next_Vec);
    angles.push_back(0.0);
  }

  //change direction
  direction(0) = 1.0;
  direction(1) = 0.0;
  direction(2) = 2.0;
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

  thread = new Thread(vertices, angles, rotations[0], rotations[1]);
  updateThreadPoints();

#ifndef ISOTROPIC
	Matrix2d B = Matrix2d::Zero();
	B(0,0) = 10.0;
	B(1,1) = 1.0;
	thread->set_coeffs_normalized(B, 3.0, 1e-4);
#else
  thread->set_coeffs_normalized(1.0, 3.0, 1e-4);
#endif
}


void initThread_closedPolygon()
{
  int numVertices = 20;
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

void initGL()        
{
    static const GLfloat light_model_ambient[] = {0.3f, 0.3f, 0.3f, 1.0f};    
    static const GLfloat lightOnePosition[] = {140.0, 0.0, 200.0, 0.0};
    static const GLfloat lightOneColor[] = {0.99, 0.99, 0.99, 1.0};
    static const GLfloat lightTwoPosition[] = {-140.0, 0.0, 200.0, 0.0};
    static const GLfloat lightTwoColor[] = {0.99, 0.99, 0.99, 1.0};
    static const GLfloat lightThreePosition[] = {140.0, 0.0, -200.0, 0.0};
    static const GLfloat lightThreeColor[] = {0.99, 0.99, 0.99, 1.0};
    static const GLfloat lightFourPosition[] = {-140.0, 0.0, -200.0, 0.0};
    static const GLfloat lightFourColor[] = {0.99, 0.99, 0.99, 1.0};
    
	// Change background color.
	glClearColor (0.0, 0.0, 0.0, 0.0);
    
    // Enable depth buffering for hidden surface removal.
    //glClearDepth (1.0);
    glDepthFunc(GL_LEQUAL);
	glEnable (GL_DEPTH_TEST);
	
	// Cull back faces.
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);
	
	// Other misc features
	glEnable (GL_LIGHTING);
	glEnable(GL_NORMALIZE);
	glShadeModel (GL_SMOOTH);

	glMatrixMode (GL_PROJECTION);
	glFrustum (-30.0, 30.0, -30.0, 30.0, 50.0, 500.0); // roughly, measured in centimeters
	glMatrixMode(GL_MODELVIEW);

    // initialize lighting
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_FALSE);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);    
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, light_model_ambient);
    glLightfv (GL_LIGHT0, GL_POSITION, lightOnePosition);
	glLightfv (GL_LIGHT0, GL_DIFFUSE, lightOneColor);
	//glEnable (GL_LIGHT0);
	glLightfv (GL_LIGHT1, GL_POSITION, lightTwoPosition);
	glLightfv (GL_LIGHT1, GL_DIFFUSE, lightTwoColor);
	glEnable (GL_LIGHT1);
	glLightfv (GL_LIGHT2, GL_POSITION, lightThreePosition);
	glLightfv (GL_LIGHT2, GL_DIFFUSE, lightThreeColor);
	glEnable (GL_LIGHT2); //right
	glLightfv (GL_LIGHT3, GL_POSITION, lightFourPosition);
	glLightfv (GL_LIGHT3, GL_DIFFUSE, lightFourColor);
	glEnable (GL_LIGHT3); //left
	
	glColorMaterial (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable (GL_COLOR_MATERIAL);
}

void initContour (void)
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

void glutMenu(int ID)
{
    switch(ID) {
        case 0:
            exit(0);
            break;
    }
}

