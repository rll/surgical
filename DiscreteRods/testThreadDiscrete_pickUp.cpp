#include <stdlib.h>
#include <pthread.h>

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
#include <algorithm> // for debugging purposes. to be removed.

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include "thread_discrete.h"
#include "thread_socket_interface.h"
#include "ThreadConstrained.h"
#include "EnvObjects.h"


// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

void initStuff();
void drawStuff();
void updateState(const Vector3d& proxy_pos, const Matrix3d& proxy_rot, Cursor* cursor);
bool closeEnough(Vector3d my_pos, Matrix3d my_rot, Vector3d pos, Matrix3d rot);
void mouseTransform(Vector3d &new_pos, Matrix3d &new_rot, vector<Vector3d> &positions, vector<Matrix3d> &rotations, int cvnum);
void drawAxes(Vector3d pos, Matrix3d rot);
void labelAxes(int constrained_vertex_num);
void drawThread();
//void drawCylinder(Vector3d pos, Matrix3d rot, double h, double r, float color0, float color1, float color2);
//void drawGrip(Vector3d pos, Matrix3d rot, double degrees, float color0, float color1, float color2);
void drawEndEffector(Vector3d pos, Matrix3d rot, double degrees, float color0, float color1, float color2);
void drawSphere(Vector3d position, float radius, float color0, float color1, float color2);
//void drawCursor(int device_id, float color);
void initThread();
void glutMenu(int ID);
void initContour ();
void initGL();

#define NUM_PTS 500
#define THREAD_RADII 1.0
#define MOVE_POS_CONST 1.0
#define MOVE_TAN_CONST 0.2
#define ROTATE_TAN_CONST 0.2
#define HAPTICS true

enum key_code {NONE, MOVEPOS, MOVETAN, ROTATETAN};

float lastx_L=0;
float lasty_L=0;
float lastx_R=0;
float lasty_R=0;

float rotate_frame[2];
float last_rotate_frame[2];

float move[2];
float tangent[2];
float tangent_rotation[2];

vector<Vector3d> points;
vector<double> twist_angles;

Vector3d zero_location;
double zero_angle;

ThreadConstrained* thread;

int pressed_mouse_button;

vector<Vector3d> positions;
vector<Matrix3d> rotations;

vector<Vector3d> all_positions;
vector<Matrix3d> all_rotations;

key_code key_pressed;

static double gCursorScale = 6;
static GLuint gCursorDisplayList = 0;
Vector3d start_proxy_pos, end_proxy_pos, start_tip_pos, end_tip_pos;
Matrix3d start_proxy_rot = Matrix3d::Identity(), end_proxy_rot = Matrix3d::Identity();
bool start_proxybutton[2] = {false, false}, end_proxybutton[2] = {false, false};
bool last_start_proxybutton[2] = {false, false}, last_end_proxybutton[2] = {false, false};
double start_feedback_pos[3], end_feedback_pos[3];
bool start_feedback_enabled = false, end_feedback_enabled = false;
bool change_constraint = false;
bool choose_mode = false;
int toggle = 0, selected_vertex_num = 0;
vector<int> constrained_vertices_nums;
Vector3d extra_end_effectors_pos = Vector3d(-40.0, -30.0, 0.0);
Matrix3d extra_end_effectors_rot = Matrix3d::Identity();
double grab_offset = EndEffector::grab_offset;

//Objects in environment
Cursor *cursor0, *cursor1;
Cylinder* base;
EndEffector* extra_end_effector;
vector<EndEffector*> constrained_ee;

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


void processLeft(int x, int y) {
	if (key_pressed == MOVEPOS)	{
		move[0] += (x-lastx_L)*MOVE_POS_CONST;
		move[1] += (lasty_L-y)*MOVE_POS_CONST;
	} else if (key_pressed == MOVETAN) {
		tangent[0] += (x-lastx_L)*MOVE_TAN_CONST;
		tangent[1] += (lasty_L-y)*MOVE_TAN_CONST;
	} else if (key_pressed == ROTATETAN) {
		tangent_rotation[0] += (x-lastx_L)*ROTATE_TAN_CONST;
		tangent_rotation[1] += (lasty_L-y)*ROTATE_TAN_CONST;
	} else {
		rotate_frame[0] += x-lastx_L;
		rotate_frame[1] += lasty_L-y;
	}
	lastx_L = x;
	lasty_L = y;
}

void processRight(int x, int y) {
	if (key_pressed == MOVEPOS)	{
		move[0] += (x-lastx_L)*MOVE_POS_CONST;
		move[1] += (lasty_L-y)*MOVE_POS_CONST;
	} else if (key_pressed == MOVETAN) {
		tangent[0] += (x-lastx_L)*MOVE_TAN_CONST;
		tangent[1] += (lasty_L-y)*MOVE_TAN_CONST;
	} else if (key_pressed == ROTATETAN) {
		tangent_rotation[0] += (x-lastx_L)*ROTATE_TAN_CONST;
		tangent_rotation[1] += (lasty_L-y)*ROTATE_TAN_CONST;
	} else {
		rotate_frame[0] += x-lastx_L;
		rotate_frame[1] += lasty_L-y;
	}
	lastx_L = x;
	lasty_L = y;
}

void MouseMotion (int x, int y) {
  if (pressed_mouse_button == GLUT_LEFT_BUTTON) {
    processLeft(x, y);
  } else if (pressed_mouse_button == GLUT_RIGHT_BUTTON) {
    processRight(x,y);
  }
	glutPostRedisplay ();
}

void processMouse(int button, int state, int x, int y) {
	if (state == GLUT_DOWN) {
    pressed_mouse_button = button;
    if (button == GLUT_LEFT_BUTTON) {
      lastx_L = x;
      lasty_L = y;
    }
    if (button == GLUT_RIGHT_BUTTON) {
      lastx_R = x;
      lasty_R = y;
    }
    glutPostRedisplay ();
	}
}

void processNormalKeys(unsigned char key, int x, int y) {
	if (key == 't')
	  key_pressed = MOVETAN;
  else if (key == 'm')
    key_pressed = MOVEPOS;
  else if (key == 'r')
    key_pressed = ROTATETAN;
  else if (key == 'w') {
    rotate_frame[0] = 0.0;
    rotate_frame[1] = 0.0;
  } else if (key == 'c') {
  	if (!choose_mode)	//from normal mode to choose mode
  		thread->getAllTransforms(all_positions, all_rotations);
  	else
  		thread->setAllTransforms(all_positions, all_rotations);
  	toggle = 0;
  	if (cursor0->isAttached())
  		cursor0->dettach();
  	if (cursor1->isAttached())
  		cursor1->dettach();
    choose_mode = !choose_mode;
    glutPostRedisplay();
  } else if (key == 'v' && choose_mode) {
    change_constraint = true;
    glutPostRedisplay();
  } else if (key == 'b') {
  	toggle--;
    glutPostRedisplay();
  } else if (key == 'n') {
  	toggle++;
    glutPostRedisplay();
  } else if (key == 'd') {
    vector<Vector3d> vv3d; 
    vector <double> vd;
    thread->get_thread_data(vv3d, vd);
    cout << "get_thread_data: absolute_points: " << vv3d.size() << endl;
    for (int i=0; i<(vv3d.size()+4)/5; i++) {
      for (int k=0; k<3; k++) {
        for (int j=0; j<5 && (i*5+j)<vv3d.size(); j++) {
          printf("%d:%3.4f\t",i*5+j,vv3d[i*5+j](k));
        }
        printf("\n");
      }
      printf("\n");
    }
    cout << "get_thread_data: absolute_twist_angles: " << vd.size() << endl;
    for (int i=0; i<(vd.size()+4)/5; i++) {
      for (int j=0; j<5 && (i*5+j)<vd.size(); j++) {
        printf("%d:%3.4f\t",i*5+j,vd[i*5+j]);
      }
      printf("\n");
    }
  } else if (key == 'f') {
    vector<Vector3d> vv3d(positions.size());
    vector<Matrix3d> vm3d(positions.size());
    thread->getConstrainedTransforms(vv3d, vm3d);
    cout << "getConstrainedTransforms: positions: " << vv3d.size() << endl;
    for (int i=0; i<(vv3d.size()+4)/5; i++) {
      for (int k=0; k<3; k++) {
        for (int j=0; j<5 && (i*5+j)<vv3d.size(); j++) {
          printf("%d:%3.4f\t",i*5+j,vv3d[i*5+j](k));
        }
        printf("\n");
      }
      printf("\n");
    }
  } else if ( key == 'g') {
    vector<int> vi;
    thread->getConstrainedVerticesNums(vi);
    cout << "getConstrainedVerticesNums: vertices_num: " << vi.size() << endl;
    for (int i=0; i<(vi.size()+4)/5; i++) {
      for (int j=0; j<5 && (i*5+j)<vi.size(); j++) {
        printf("%d:%d\t",i*5+j,vi[i*5+j]);
      }
      printf("\n");
    }
  } else if ( key == 'h') {
    vector<int> vi;
    thread->getOperableFreeVertices(vi);
    cout << "getOperableFreeVertices: vertices_num: " << vi.size() << endl;
    for (int i=0; i<(vi.size()+4)/5; i++) {
      for (int j=0; j<5 && (i*5+j)<vi.size(); j++) {
        printf("%d:%d\t",i*5+j,vi[i*5+j]);
      }
      printf("\n");
    }
  } else if (key == 'j') {
    vector<Vector3d> vv3d;
    thread->getConstrainedVertices(vv3d);
		cout << "getConstrainedVertices: constrained_vertices: " << vv3d.size() << endl;
    for (int i=0; i<(vv3d.size()+4)/5; i++) {
      for (int k=0; k<3; k++) {
        for (int j=0; j<5 && (i*5+j)<vv3d.size(); j++) {
          printf("%d:%3.4f\t",i*5+j,vv3d[i*5+j](k));
        }
        printf("\n");
      }
      printf("\n");
    }
  } else if (key == 'k') {
		vector<Vector3d> vv3d;
    thread->getFreeVertices(vv3d);
		cout << "getFreeVertices: free_vertices: " << vv3d.size() << endl;
    for (int i=0; i<(vv3d.size()+4)/5; i++) {
      for (int k=0; k<3; k++) {
        for (int j=0; j<5 && (i*5+j)<vv3d.size(); j++) {
          printf("%d:%3.4f\t",i*5+j,vv3d[i*5+j](k));
        }
        printf("\n");
      }
      printf("\n");
    }
  } else if (key == 'l') {
		vector<int> vi;
		vector<bool> vb;
    thread->getOperableVertices(vi, vb);
    cout << "getOperableVertices: operable_vertices_num: " << vi.size() << endl;
    for (int i=0; i<(vi.size()+4)/5; i++) {
      for (int j=0; j<5 && (i*5+j)<vi.size(); j++) {
        printf("%d:%d\t",i*5+j,vi[i*5+j]);
      }
      printf("\n");
    }
    cout << "getOperableVertices: constrained_or_free: " << vb.size() << endl;
    for (int i=0; i<(vb.size()+4)/5; i++) {
      for (int j=0; j<5 && (i*5+j)<vb.size(); j++) {
        printf("%d:%d\t",i*5+j,vb[i*5+j]?1:0);
      }
      printf("\n");
    }
  }	else if (key == 'q' || key == 27) {
    exit(0);
  }
  lastx_R = x;
  lasty_R = y;
}

void processKeyUp(unsigned char key, int x, int y)
{
  key_pressed = NONE;
  move[0] = move[1] = tangent[0] = tangent[1] = tangent_rotation[0] = tangent_rotation[1] = 0.0;
}

void processProxybutton() {
	cursor0->saveLastOpen();
	if (last_start_proxybutton[0] && !start_proxybutton[0])
		cursor0->openClose();
	cursor0->attach_dettach_attempt = (last_start_proxybutton[1] && !start_proxybutton[1]);
	last_start_proxybutton[0] = start_proxybutton[0];
	last_start_proxybutton[1] = start_proxybutton[1];
	
	cursor1->saveLastOpen();
  if (last_end_proxybutton[0] && !end_proxybutton[0])
		cursor1->openClose();
	cursor1->attach_dettach_attempt = (last_end_proxybutton[1] && !end_proxybutton[1]);
	last_end_proxybutton[0] = end_proxybutton[0];
	last_end_proxybutton[1] = end_proxybutton[1];
}          

void processHapticDevice(int value)
{
  getDeviceState (start_proxy_pos, start_proxy_rot, start_proxybutton, end_proxy_pos, end_proxy_rot, end_proxybutton);
  
  processProxybutton();
  
	//start_feedback_enabled = end_feedback_enabled = true;	
	//sendDeviceState (positions[0] + grab_offset * rotations[0].col(0), start_feedback_enabled, positions[1] + grab_offset * rotations[1].col(0), end_feedback_enabled);
	
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
    "Each end can be controlled either by the haptic device or by the mouse and keyboard.\n"
    "There are two modes: normal and choose modes. Initially, normal mode is selected.\n"
    "To change between modes, press 'c'.\n"
    "\n"
    "In normal mode:\n"
    "\tPhantom control input:\n"
    "\t\tThe red/green small cylinders (cursors) represent the true position and \n\t\t\torientation of the haptic device.\n"
    "\t\tThe cursor can be open or closed, and this is represented by a green or \n\t\t\tred cursor respectively.\n"
    "\t\tThe cursor can be attached to an end effector, which inherits the \n\t\t\topen/close status of the cursor that holds it.\n"
    "\t\tTo attach the cursor to an end effector, the cursor have to be near and \n\t\t\taligned (orientation) to the end effector's yellow cylinder.\n"
    "\t\tPress the haptic's upper button to change between open and closed.\n"
    "\t\tPress the haptic's lower button to attach or dettach the cursor to or from \n\t\t\tthe end effector.\n"
    "\n"
    "\tMouse and keyboard control input:\n"
    "\t\tOnly one end effector can be selected at a time. The selected end effector \n\t\t\thas a darker color.\n"
    "\t\tPress 'n' or 'b' to select the next or previous end effector, respectively.\n"
    "\t\tHold 'm' while holding down the left or right mouse button to move the \n\t\t\tselected end effector.\n"
    "\t\tHold 't' while holding down the left or right mouse button to orientate \n\t\t\tthe tangent of the selected end effector.\n"
    "\t\tHold 'r' while holding down the left or right mouse button to roll the \n\t\t\ttangent of the selected end effector.\n"
    "\n"
    "In choose mode:\n"
    "\tUse this mode to add constraints (with end effectors) to the thread using \n\t\tonly the mouse and keyboard.\n"
    "\tAny end effectors held by the cursor and not holding the thread are dropped \n\t\twhen changing into this mode\n"
    "\tConstrained vertices are represented by red spheres. Free vertices where an \n\t\tend effector can be attached are represented by green spheres.\n"
    "\tThere is one end effector that has a darker color. Use this end effector to \n\t\ttoggle between vertices.\n"
    "\tIf the darker end effector is open, there is not a constraint in the \n\t\tselected vertex, and viceversa.\n"
    "\tThe orientation of the end effector _relative_ to the thread can be changed \n\t\tusing 't' and 'r' and the mouse in a similar way as in normal mode.\n"
    "\tPress 'b' to close the end effector and thus constraint the vertex, or to open \n\t\tthe end effector and thus remove the constraint from the vertex.\n"
    "\tPress 'n' or 'b' to select the next or previous vertex, respectively.\n"
    "\n"
    "Press 'w' to reset view.\n"
    "Press 'q' or 'ESC' to quit.\n");

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
  if (HAPTICS)
  	glutTimerFunc(100, processHapticDevice, 0);
	
	/* create popup menu */
	glutCreateMenu (glutMenu);
	glutAddMenuEntry ("Exit", 99);
	glutAttachMenu (GLUT_MIDDLE_BUTTON);

	initGL();
	initStuff ();

  initThread();
	thread->minimize_energy();
  thread->getConstrainedTransforms(positions, rotations);

  zero_location = Vector3d::Zero(); //points[0];
  zero_angle = 0.0;
	
	if (HAPTICS) {
		cursor0 = new Cursor(Vector3d::Zero(), Matrix3d::Identity());
		cursor1 = new Cursor(Vector3d::Zero(), Matrix3d::Identity());
		base = new Cylinder(extra_end_effectors_pos+Vector3d(20.0, -1.2, 0.0), (Matrix3d) AngleAxisd(0.5*M_PI, Vector3d::UnitZ()), 15, 8, 0.0, 1.0, 0.0);
		extra_end_effector = new EndEffector(extra_end_effectors_pos, extra_end_effectors_rot);
		EndEffector* end_effector0 = new EndEffector(positions[0], rotations[0]);
		EndEffector* end_effector1 = new EndEffector(positions[1], rotations[1]);
		end_effector0->constraint = constrained_vertices_nums[0];
		end_effector0->constraint = constrained_vertices_nums[1];
		end_effector1->constraint_ind = 0;
		end_effector1->constraint_ind = 1;
		constrained_ee.push_back(end_effector0);
	 	constrained_ee.push_back(end_effector1);

		connectionInit();
	}
  glutMainLoop ();
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
  
  //thread->getConstrainedTransforms(positions, rotations);
  
  if (choose_mode) {
	  vector<int> operable_vertices_num;
	  vector<bool> constrained_or_free;
	  thread->getOperableVertices(operable_vertices_num, constrained_or_free);
		toggle = (toggle+operable_vertices_num.size())%operable_vertices_num.size();
		selected_vertex_num = operable_vertices_num[toggle];
		if (change_constraint) {
	  	if (selected_vertex_num==0 || selected_vertex_num==(thread->numVertices()-1)) {
	  		cout << "You cannot remove the constraint at the ends" << endl;
	  	} else {
		  	if (constrained_or_free[toggle]) {
		  		int selected_vertex_ind = find(constrained_vertices_nums, selected_vertex_num);
		  		EndEffector* ee = constrained_ee[selected_vertex_ind];
			    positions.resize(positions.size()-1);
			    rotations.resize(rotations.size()-1);
			    thread->removeConstraint(selected_vertex_num);
					constrained_ee.erase(constrained_ee.begin() + ee->constraint_ind);
				 	ee->constraint = ee->constraint_ind = -1;
					thread->getConstrainedVerticesNums(constrained_vertices_nums);
			  } else {
			    positions.resize(positions.size()+1);
			    rotations.resize(rotations.size()+1);
			    thread->addConstraint(selected_vertex_num);
			    thread->getConstrainedTransforms(positions, rotations);
					thread->getConstrainedVerticesNums(constrained_vertices_nums);
					int selected_vertex_ind = find(constrained_vertices_nums, selected_vertex_num);
					EndEffector* ee = new EndEffector(positions[selected_vertex_ind], rotations[selected_vertex_ind]);
					ee->constraint_ind = selected_vertex_ind;
					ee->constraint = selected_vertex_num;
					constrained_ee.insert(constrained_ee.begin() + ee->constraint_ind, ee);
			  }
			  thread->getOperableVertices(operable_vertices_num, constrained_or_free);
			  for (int i=0; i<operable_vertices_num.size(); i++) {					// adjust toggle such that selected vertex after change is the same as the one before change
			  	if(selected_vertex_num == operable_vertices_num[i]) {
			  		toggle = i;
			  		break;
			  	}
			  }
			}
		  change_constraint = false;
		  //thread->getConstrainedVerticesNums(constrained_vertices_nums);
		  thread->setAllTransforms(all_positions, all_rotations);
		  thread->getAllTransforms(all_positions, all_rotations);
	  }
	  Vector3d dummy_pos;
		mouseTransform(dummy_pos, all_rotations[selected_vertex_num], all_positions, all_rotations, selected_vertex_num);
	  for (int i=0; i<operable_vertices_num.size(); i++) {
	  	if (constrained_or_free[i]) {
		  	drawSphere(all_positions[operable_vertices_num[i]], 1.2, 1.0, 0.0, 0.0);
		  } else {
		  	drawSphere(all_positions[operable_vertices_num[i]], 1.2, 0.0, 1.0, 0.0);
		  }
		}
		for (int i=0; i<constrained_vertices_nums.size(); i++) {
			constrained_ee[i]->setTransform(all_positions[constrained_vertices_nums[i]], all_rotations[constrained_vertices_nums[i]]);
		}
	  int selected_vertex_ind = find(constrained_vertices_nums, selected_vertex_num);
		if (constrained_or_free[toggle]) {
	  	drawSphere(all_positions[selected_vertex_num], 1.4, 0.5, 0.0, 0.0);
	  	if (selected_vertex_ind == -1)
	  		cout << "Internal error: drawStuff(): if (constrained_or_free[toggle]): selected vertex should be constrained by an end effector." << endl;
	  	constrained_ee[selected_vertex_ind]->highlight();
	  } else {
	  	drawSphere(all_positions[selected_vertex_num], 1.4, 0.0, 0.5, 0.0);
	  	if (selected_vertex_ind != -1)
	  		cout << "Internal error: drawStuff(): if (constrained_or_free[toggle]): selected vertex should be constrained by an end effector." << endl;
  		drawEndEffector(all_positions[selected_vertex_num], all_rotations[selected_vertex_num], 15, 0.4, 0.4, 0.4);
	  }
	} else {
		// starts non-choose mode
  	if (HAPTICS) {
  		updateState(start_proxy_pos, start_proxy_rot, cursor0);
  		updateState(end_proxy_pos, end_proxy_rot, cursor1);
		}
		
		toggle = (toggle+constrained_vertices_nums.size())%constrained_vertices_nums.size();
		
  	thread->getConstrainedTransforms(positions, rotations);
		vector<Vector3d> positionConstraints = positions;
		vector<Matrix3d> rotationConstraints = rotations;
		if ((!cursor0->isAttached() || (cursor0->isAttached() && toggle!=cursor0->end_eff->constraint_ind)) && 
				(!cursor1->isAttached() || (cursor1->isAttached() && toggle!=cursor1->end_eff->constraint_ind)))
			mouseTransform(positionConstraints[toggle], rotationConstraints[toggle], positions, rotations, toggle);
		
		if (HAPTICS) {
		  if (cursor0->isAttached() && cursor0->end_eff->constraint_ind>=0) {
		  	positionConstraints[cursor0->end_eff->constraint_ind] = start_proxy_pos - grab_offset * start_proxy_rot.col(0);
		  	rotationConstraints[cursor0->end_eff->constraint_ind] = start_proxy_rot;
		  }
		  if (cursor1->isAttached() && cursor1->end_eff->constraint_ind>=0) {
		  	positionConstraints[cursor1->end_eff->constraint_ind] = end_proxy_pos - grab_offset * end_proxy_rot.col(0);
		  	rotationConstraints[cursor1->end_eff->constraint_ind] = end_proxy_rot;
		  }
		}
		thread->updateConstraints(positionConstraints, rotationConstraints);
		
		thread->getConstrainedTransforms(positions, rotations);
		
		constrained_ee[toggle]->highlight();
		for(int i=0; i<positions.size(); i++)
			constrained_ee[i]->setTransform(positions[i], rotations[i]);
			
		if (cursor0->isAttached() && cursor0->end_eff->constraint < 0)
			cursor0->end_eff->draw();
		if (cursor1->isAttached() && cursor1->end_eff->constraint < 0)
			cursor1->end_eff->draw();
		cursor0->draw();
		cursor1->draw();
	}
	
	drawThread();
	base->draw();
	extra_end_effector->draw();
	for(int i=0; i<positions.size(); i++)
		constrained_ee[i]->draw();
	
  glPopMatrix ();
  glutSwapBuffers ();
}

void updateState(const Vector3d& proxy_pos, const Matrix3d& proxy_rot, Cursor* cursor) {
	Vector3d tip_pos = proxy_pos - grab_offset * proxy_rot.col(0);
	
	cursor->setTransform(proxy_pos, proxy_rot);
	if (cursor->isAttached()) {																																				// cursor has an end effector
		EndEffector* ee = cursor->end_eff;
	  if (ee->constraint<0) {																																					// cursor has an end effector but it isn't holding the thread
	  	int nearest_vertex = thread->nearestVertex(tip_pos);
	  	if (cursor->justClosed() && ((thread->position(nearest_vertex) - tip_pos).squaredNorm() < 16.0)) {	// cursor has an end effector which just started holding the thread
	  		ee->constraint = nearest_vertex;
		  	positions.resize(positions.size()+1);
		    rotations.resize(rotations.size()+1);
		    thread->addConstraint(nearest_vertex);
		    thread->getConstrainedTransforms(positions, rotations);
				thread->getConstrainedVerticesNums(constrained_vertices_nums);
		    ee->constraint_ind = find(constrained_vertices_nums, nearest_vertex);
		    constrained_ee.insert(constrained_ee.begin() + ee->constraint_ind, ee);
		    positions[ee->constraint_ind] = tip_pos;
		    rotations[ee->constraint_ind] = proxy_rot;
		    thread->setConstrainedTransforms(positions, rotations);			// the end effector's orientation matters when it grips the thread. This updates the offset rotation.
		    //for (int i=0; i<constrained_ee.size(); i++)
		    	//constrained_ee[i]->setTransform(positions[i], rotations[i]);
		  } else {																																											// cursor has an end effector which remains without holding the thread
	  		ee->setTransform(tip_pos, proxy_rot);
		  }
	  } else {																																												// cursor has an end effector which is holding the thread
	  	if ((ee->constraint==0 || ee->constraint==(thread->numVertices()-1)) && cursor->isOpen()) {		// cursor has an end effector which is holding the thread end and trying to be opened
				cout << "You cannot open the end effector holding the end constraint" << endl;
			 	cursor->forceClose();
			} else if (cursor->isOpen()) {																																// cursor has an end effector which is holding the thread and trying to be opened
				positions.resize(positions.size()-1);
			  rotations.resize(rotations.size()-1);
			  thread->removeConstraint(ee->constraint);
			  constrained_ee.erase(constrained_ee.begin() + ee->constraint_ind);
			 	ee->constraint = ee->constraint_ind = -1;
				thread->getConstrainedVerticesNums(constrained_vertices_nums);
			}
		}
		if (cursor->attach_dettach_attempt) {
			cursor->dettach();
		}
	} else {																				// cursor doesn't have an end effector, THUS it isn't holding the thread i.e. ee->constraint = ee->constraint_ind = -1;
  	if (cursor->attach_dettach_attempt && closeEnough(tip_pos, proxy_rot, extra_end_effectors_pos, extra_end_effectors_rot)) {
  		EndEffector* ee = new EndEffector(extra_end_effectors_pos, extra_end_effectors_rot);
  		cursor->attach(ee);
  	} else {
  		for (int i=0; i<positions.size(); i++) {
  			if (cursor->attach_dettach_attempt && closeEnough(tip_pos, proxy_rot, positions[i], rotations[i])) {
  				EndEffector* ee = constrained_ee[i];
  				cursor->attach(ee);
  				ee->constraint = constrained_vertices_nums[i];
  				ee->constraint_ind = i;
  				if ((ee->constraint==0 || ee->constraint==(thread->numVertices()-1)) && cursor->isOpen()) 
	  				cursor->forceClose();
  				break;
  			}
  		}
  	}
 	}
}

bool closeEnough(Vector3d my_pos, Matrix3d my_rot, Vector3d pos, Matrix3d rot) {
	double angle = 2*asin((my_rot.col(0) - rot.col(0)).norm()/2);
  return (((my_pos - pos).norm() < 4.0) && angle < 0.25*M_PI);
}

//cvnum stands for constrained vertex number or choosen vertex number
void mouseTransform(Vector3d &new_pos, Matrix3d &new_rot, vector<Vector3d> &positions, vector<Matrix3d> &rotations, int cvnum) {
	if (move[0] != 0.0 || move[1] != 0.0 || tangent[0] != 0.0 || tangent[1] != 0.0 || tangent_rotation[0] != 0.0 || tangent_rotation[1] != 0.0) { 
		GLdouble model_view[16];
		glGetDoublev(GL_MODELVIEW_MATRIX, model_view);
		GLdouble projection[16];
		glGetDoublev(GL_PROJECTION_MATRIX, projection);
		GLint viewport[4];
		glGetIntegerv(GL_VIEWPORT, viewport);

		double winX, winY, winZ;

		//change positions
		gluProject(positions[cvnum](0), positions[cvnum](1), positions[cvnum](2), model_view, projection, viewport, &winX, &winY, &winZ);
		winX += move[0];
		winY += move[1];
		move[0] = 0.0;
		move[1] = 0.0;
		gluUnProject(winX, winY, winZ, model_view, projection, viewport, &new_pos(0), &new_pos(1), &new_pos(2));

		//change tangents
		vector<Vector3d> tangents(rotations.size());
		for (int i=0; i<rotations.size(); i++) {
			tangents[i] = rotations[i].col(0);
			tangents[i].normalize();
		}
		Vector3d new_tan;
		gluProject(positions[cvnum](0)+tangents[cvnum](0),positions[cvnum](1)+tangents[cvnum](1), positions[cvnum](2)+tangents[cvnum](2), model_view, projection, viewport, &winX, &winY, &winZ);
		winX += tangent[0];
		winY += tangent[1];
		tangent[0] = 0.0;
		tangent[1] = 0.0;
		gluUnProject(winX, winY, winZ, model_view, projection, viewport, &new_tan(0), &new_tan(1), &new_tan(2));
		new_tan -= positions[cvnum];
		new_tan.normalize();

		Matrix3d rotation_new_tan;
		rotate_between_tangents(tangents[cvnum], new_tan, rotation_new_tan);

		//check rotation around tangent
		Matrix3d old_rot = rotations[cvnum];
		Vector3d tangent_normal_rotate_around = rotations[cvnum].col(1);
		Vector3d new_tan_normal;
		gluProject(positions[cvnum](0)+tangent_normal_rotate_around(0), positions[cvnum](1)+tangent_normal_rotate_around(1), positions[cvnum](2)+tangent_normal_rotate_around(2), model_view, projection, viewport, &winX, &winY, &winZ);
		winX += tangent_rotation[0];
		winY += tangent_rotation[1];
		tangent_rotation[0] = 0.0;
		tangent_rotation[1] = 0.0;
		gluUnProject(winX, winY, winZ, model_view, projection, viewport, &new_tan_normal(0), &new_tan_normal(1), &new_tan_normal(2));
		new_tan_normal -= positions[cvnum];
		//project this normal onto the plane normal to X (to ensure Y stays normal to X)
		new_tan_normal -= new_tan_normal.dot(rotations[cvnum].col(0))*rotations[cvnum].col(0);
		new_tan_normal.normalize();

		rotations[cvnum].col(1) = new_tan_normal;
		rotations[cvnum].col(2) = rotations[cvnum].col(0).cross(new_tan_normal);
		new_rot = Eigen::AngleAxisd(angle_mismatch(rotations[cvnum], old_rot), rotations[cvnum].col(0).normalized())*rotation_new_tan* old_rot;
	} else {
		new_pos = positions[cvnum];
		new_rot = rotations[cvnum];
	}
}

void drawThread() {
	thread->get_thread_data(points, twist_angles);

  glPushMatrix();
  glColor3f (0.5, 0.5, 0.2);
  double pts_cpy[points.size()+2][3];
  double twist_cpy[points.size()+2]; 
  for (int i=0; i < points.size(); i++)
  {
    pts_cpy[i+1][0] = points[i](0)-(double)zero_location(0);
    pts_cpy[i+1][1] = points[i](1)-(double)zero_location(1);
    pts_cpy[i+1][2] = points[i](2)-(double)zero_location(2);
   	twist_cpy[i+1] = -(360.0/(2.0*M_PI))*(twist_angles[i]);
  }
  //add first and last point
  pts_cpy[0][0] = 2*pts_cpy[1][0] - pts_cpy[2][0];//pts_cpy[1][0]-rotations[0](0,0);
  pts_cpy[0][1] = 2*pts_cpy[1][1] - pts_cpy[2][1];//pts_cpy[1][1]-rotations[0](1,0);
  pts_cpy[0][2] = 2*pts_cpy[1][2] - pts_cpy[2][2];//pts_cpy[1][2]-rotations[0](2,0);
  twist_cpy[0] = twist_cpy[1];

  pts_cpy[points.size()+1][0] = 2*pts_cpy[points.size()][0] - pts_cpy[points.size()-1][0];//pts_cpy[points.size()][0]+rotations[1](0,0);
  pts_cpy[points.size()+1][1] = 2*pts_cpy[points.size()][1] - pts_cpy[points.size()-1][1];//pts_cpy[points.size()][1]+rotations[1](1,0);
  pts_cpy[points.size()+1][2] = 2*pts_cpy[points.size()][2] - pts_cpy[points.size()-1][2];//pts_cpy[points.size()][2]+rotations[1](2,0);
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
}

/*void drawCylinder(Vector3d pos, Matrix3d rot, double h, double r, float color0, float color1, float color2) {
	glPushMatrix();
	double transform[16] = { rot(0,0) , rot(1,0) , rot(2,0) , 0 ,
													 rot(0,1) , rot(1,1) , rot(2,1) , 0 ,
													 rot(0,2) , rot(1,2) , rot(2,2) , 0 ,
													 pos(0)   , pos(1)   , pos(2)   , 1 };
	glMultMatrixd(transform);
	glColor3f(color0, color1, color2);
	double cylinder[4][3] = { {-h-1.0, 0.0, 0.0} , {-h, 0.0, 0.0} , {0.0, 0.0, 0.0} ,
															 {1.0, 0.0, 0.0} };
	glePolyCylinder(4, cylinder, NULL, r);
	glPopMatrix();
}*/

void drawEndEffector(Vector3d pos, Matrix3d rot, double degrees, float color0, float color1, float color2) {
	glPushMatrix();
	double transform[16] = { rot(0,0) , rot(1,0) , rot(2,0) , 0 ,
													 rot(0,1) , rot(1,1) , rot(2,1) , 0 ,
													 rot(0,2) , rot(1,2) , rot(2,2) , 0 ,
													 pos(0)   , pos(1)   , pos(2)   , 1 };
	glMultMatrixd(transform);
			
	int pieces = 6;
	double h = 1.5;
	double start = -3;
	double handle_r = 1.2;
	double end = ((double) pieces)*h + start;
	
	glColor3f(0.3, 0.3, 0.0);
	double cylinder[4][3] = { {grab_offset-4.0, 0.0, 0.0} , {grab_offset-3.0, 0.0, 0.0} , {grab_offset, 0.0, 0.0} ,
														{grab_offset+1.0, 0.0, 0.0} };
	glePolyCylinder(4, cylinder, NULL, 1.6);
	
	glColor3f(color0, color1, color2);
	double grip_handle[4][3] = { {end-1.0, 0.0, 0.0} , {end, 0.0, 0.0} , {end+30.0, 0.0, 0.0} ,
															 {end+31.0, 0.0, 0.0} };
	glePolyCylinder(4, grip_handle, NULL, handle_r);

	glTranslatef(end, 0.0, 0.0);
	glRotatef(-degrees, 0.0, 0.0, 1.0);
	glTranslatef(-end, 0.0, 0.0);
	Matrix3d open_rot = (Matrix3d) AngleAxisd(-degrees*M_PI/180, rot*Vector3d::UnitZ());
	Vector3d new_pos;
	Matrix3d new_rot;
	for (int piece=0; piece<pieces; piece++) {
		double r = 0.5+((double) piece)*((0.8*handle_r)-0.5)/((double) pieces-1);
		gleDouble grip_tip[4][3] = { {start+((double) piece)*h-1.0, r, 0.0} , {start+((double) piece)*h, r, 0.0} , {start+((double) piece+1)*h, r, 0.0} , {start+((double) piece+1)*h+1.0, r, 0.0} };
 		glePolyCylinder(4, grip_tip, NULL, r); 		
		new_rot = open_rot * rot;
		new_pos = open_rot * rot * Vector3d(-end, 0.0, 0.0) + rot * Vector3d(end, 0.0, 0.0) + pos;
	}
	
	glTranslatef(end, 0.0, 0.0);
	glRotatef(2*degrees, 0.0, 0.0, 1.0);
	glTranslatef(-end, 0.0, 0.0);
	for (int piece=0; piece<pieces; piece++) {
		double r = 0.5+((double) piece)*((0.8*handle_r)-0.5)/((double) pieces-1);
		gleDouble grip_tip[4][3] = { {start+((double) piece)*h-1.0, -r, 0.0} , {start+((double) piece)*h, -r, 0.0} , {start+((double) piece+1)*h, -r, 0.0} , {start+((double) piece+1)*h+1.0, -r, 0.0} };
 		glePolyCylinder(4, grip_tip, NULL, r);
 		new_rot = open_rot.transpose() * rot;
		new_pos = open_rot.transpose() * rot * Vector3d(-end, 0.0, 0.0) + rot * Vector3d(end, 0.0, 0.0) + pos;
	}
	glPopMatrix();
}

/*void drawGrip(Vector3d pos, Matrix3d rot, double degrees, float color0, float color1, float color2) {
	glPushMatrix();
	double transform[16] = { rot(0,0) , rot(1,0) , rot(2,0) , 0 ,
													 rot(0,1) , rot(1,1) , rot(2,1) , 0 ,
													 rot(0,2) , rot(1,2) , rot(2,2) , 0 ,
													 pos(0)   , pos(1)   , pos(2)   , 1 };
	glMultMatrixd(transform);
	glRotated(-90,0,0,1);
	glColor3f(color0, color1, color2);
	double grip_handle[4][3] = { {0.0, 9.0, 0.0} , {0.0,11.0, 0.0} , {0.0,19.0, 0.0} ,
															 {0.0,21.0, 0.0} };
	glePolyCylinder(4, grip_handle, NULL, 1);
	
	glTranslatef(0.0, 11.0, 0.0);
	glRotatef(-degrees, 1.0, 0.0, 0.0);
	glTranslatef(0.0, -11.0, 0.0);
	double grip_tip0[4][3] = { {0.0, 0.0,-2.0} , {0.0, 0.0, 0.0} , {0.0, 0.0, 2.0} ,
														 {0.0, 0.0, 4.0} };
	glePolyCylinder(4, grip_tip0, NULL, 0.95*thread->rest_length());
	double grip_side0[8][3] = { {0.0, 0.0,-2.0} , {0.0, 0.0, 0.0} , {0.0, 0.0, 2.0} ,
															{0.0, 3.0, 5.0} , {0.0, 6.0, 5.0} , {0.0,11.0, 0.0} ,
															{0.0,13.0, 0.0} };
	glePolyCylinder(8, grip_side0, NULL, 1);
															
	glTranslatef(0.0, 11.0, 0.0);
	glRotatef(2*degrees, 1.0, 0.0, 0.0);
	glTranslatef(0.0, -11.0, 0.0);	
	double grip_tip1[4][3] = { {0.0, 0.0, 2.0} , {0.0, 0.0, 0.0} , {0.0, 0.0,-2.0} ,
														 {0.0, 0.0,-4.0} };
	glePolyCylinder(4, grip_tip1, NULL, 0.95*thread->rest_length());
	double grip_side1[8][3] = { {0.0, 0.0, 2.0} , {0.0, 0.0, 0.0} , {0.0, 0.0,-2.0} ,
															{0.0, 3.0,-5.0} , {0.0, 6.0,-5.0} , {0.0,11.0, 0.0} ,
															{0.0,13.0, 0.0} };
	glePolyCylinder(8, grip_side1, NULL, 1);
	glPopMatrix();
}*/

void drawSphere(Vector3d position, float radius, float color0, float color1, float color2) {
	glPushMatrix();
	double transform[16] = {1,0,0,0,
													0,1,0,0,
													0,0,1,0,
													position(0), position(1), position(2), 1};
	glMultMatrixd(transform);
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_COLOR_MATERIAL);
  glColor3f(color0, color1, color2);
  glutSolidSphere(radius, 20, 16);
  //glFlush ();
  glPopMatrix();
}

/*void drawCursor(int device_id, float color) {
  static const double kCursorRadius = 0.5;
  static const double kCursorHeight = 1.5;
  static const int kCursorTess = 4;
  
  GLUquadricObj *qobj = 0;

  glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT);
  glPushMatrix();

  if (!gCursorDisplayList) {
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
  
  // Get the proxy transform in world coordinates
  if (device_id) {
  	glMultMatrixd(end_proxyxform);
  } else {
  	glMultMatrixd(start_proxyxform);
  }
  
  // Apply the local cursor scale factor.
  glScaled(gCursorScale, gCursorScale, gCursorScale);

  glEnable(GL_COLOR_MATERIAL);
  glColor3f(0.0, color, 1.0);

  glCallList(gCursorDisplayList);
  
  glPopMatrix();
  glPopAttrib();
}*/

void drawAxes(Vector3d pos, Matrix3d rot) {
	glPushMatrix();
	double transform[] = { rot(0,0) , rot(1,0) , rot(2,0) , 0 ,
												 rot(0,1) , rot(1,1) , rot(2,1) , 0 ,
												 rot(0,2) , rot(1,2) , rot(2,2) , 0 ,
												 pos(0)   , pos(1)   , pos(2)   , 1 };
	glMultMatrixd(transform);
	glBegin(GL_LINES);
	glEnable(GL_LINE_SMOOTH);
	glColor3d(1.0, 0.0, 0.0); //red
	glVertex3f(0.0, 0.0, 0.0); //x
	glVertex3f(10.0, 0.0, 0.0);
	glColor3d(0.0, 1.0, 0.0); //green
	glVertex3f(0.0, 0.0, 0.0); //y
	glVertex3f(0.0, 10.0, 0.0);
	glColor3d(0.0, 0.0, 1.0); //blue
	glVertex3f(0.0, 0.0, 0.0); //z
	glVertex3f(0.0, 0.0, 10.0);
	glEnd();
	glPopMatrix();
}

void labelAxes(Vector3d pos, Matrix3d rot) {
  glPushMatrix();
  Vector3d diff_pos = pos-zero_location;
	double rotation_scale_factor = 20.0;
	Matrix3d rotations_project = rot*rotation_scale_factor;
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

void initStuff (void)
{
  glEnable(GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  rotate_frame[0] = 0.0;
  rotate_frame[1] = 0.0; //-111.0;
      
  initContour();
}

void initThread()
{	
	int num_vertices=15;
  thread = new ThreadConstrained(num_vertices);
  positions.resize(2);
  rotations.resize(2);
  thread->getConstrainedTransforms(positions, rotations);
  all_positions.resize(num_vertices);
  all_rotations.resize(num_vertices);
  thread->getConstrainedVerticesNums(constrained_vertices_nums);

#ifndef ISOTROPIC
	Matrix2d B = Matrix2d::Zero();
	B(0,0) = 10.0;
	B(1,1) = 1.0;
	thread->set_coeffs_normalized(B, 3.0, 1e-4);
#else
  thread->set_coeffs_normalized(1.0, 3.0, 1e-4);
#endif
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
	//glEnable (GL_LIGHT0);		// uncomment this if you want another source of light
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

void glutMenu(int ID) {
	switch(ID) {
    case 0:
      exit(0);
      break;
  }
}

