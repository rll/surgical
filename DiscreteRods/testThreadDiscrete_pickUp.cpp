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
#include "SimpleEnv.h"
#include "drawutils.h"

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

void drawStuff();
void processInput();
void updateState(const Vector3d& proxy_pos, const Matrix3d& proxy_rot, Cursor* cursor);
bool closeEnough(Vector3d my_pos, Matrix3d my_rot, Vector3d pos, Matrix3d rot);
void mouseTransform(Vector3d &new_pos, Matrix3d &new_rot, vector<Vector3d> &positions, vector<Matrix3d> &rotations, int cvnum);
void initThread();
void glutMenu(int ID);
void initGL();

#define NUM_PTS 500
#define THREAD_RADII 1.0
#define MOVE_POS_CONST 1.0
#define MOVE_TAN_CONST 0.2
#define ROTATE_TAN_CONST 0.2
#define HAPTICS true
#define VIEW3D

enum key_code {NONE, MOVEPOS, MOVETAN, ROTATETAN};

float lastx_L=0;
float lasty_L=0;
float lastx_R=0;
float lasty_R=0;
float lastx_M=0;
float lasty_M=0;

float rotate_frame[2] = { 0.0, 0.0 };
float last_rotate_frame[2];
float translate_frame[3] = { 0.0, 0.0, -110.0 };
#ifdef VIEW3D
int main_window = 0;
int side_window = 0;
float eye_separation = 5.0;
float eye_focus_depth = -translate_frame[2];//-50.0;
#endif

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
SimpleTexturedSphere* textured_sphere;

SimpleEnv *env;


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
	} else if (glutGetModifiers() & GLUT_ACTIVE_CTRL){
		translate_frame[0] += 0.1*(x-lastx_L);
		translate_frame[1] += 0.1*(lasty_L-y);
	} else {
		rotate_frame[0] += x-lastx_L;
		rotate_frame[1] += lasty_L-y;
	}
	lastx_L = x;
	lasty_L = y;
}

/*void processRight(int x, int y) {
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
}*/

void processMiddle(int x, int y) {
	translate_frame[2] -= x-lastx_M;
	if (translate_frame[2] >= -(50.0 + 5.0))
		translate_frame[2] += x-lastx_M;
	eye_focus_depth = -translate_frame[2]; //-50.0;
	lastx_M = x;
	lasty_M = y;
}

void MouseMotion (int x, int y) {
  if (pressed_mouse_button == GLUT_LEFT_BUTTON) {
   	processLeft(x, y);
  //} else if (pressed_mouse_button == GLUT_RIGHT_BUTTON) {
  //  processRight(x,y);
  } else if (pressed_mouse_button == GLUT_MIDDLE_BUTTON) {
    processMiddle(x,y);
  }
  processInput();
	glutPostRedisplay ();
}

void processMouse(int button, int state, int x, int y) {
	if (state == GLUT_DOWN) {
    pressed_mouse_button = button;
    if (button == GLUT_LEFT_BUTTON) {
      lastx_L = x;
      lasty_L = y;
    }
    //if (button == GLUT_RIGHT_BUTTON) {
    //  lastx_R = x;
    //  lasty_R = y;
    //}
    if (button == GLUT_MIDDLE_BUTTON) {
      lastx_M = x;
      lasty_M = y;
    }
    processInput();
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
    processInput();
    //glutPostRedisplay();
  } else if (key == 'v' && choose_mode) {
    change_constraint = true;
    processInput();
    //glutPostRedisplay();
  } else if (key == 'b') {
  	toggle--;
  	processInput();
    //glutPostRedisplay();
  } else if (key == 'n') {
  	toggle++;
  	processInput();
    //glutPostRedisplay();
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
      printf("\n");  glPushMatrix ();  
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  /* set up some matrices so that the object spins with the mouse */
  glTranslatef (0.0,0.0,-110.0);
  glRotatef (rotate_frame[1], 1.0, 0.0, 0.0);
  glRotatef (rotate_frame[0], 0.0, 0.0, 1.0);
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
  } else if(key == 'e') {
  	thread->toggleExamineMode();
  	processInput();
  	//glutPostRedisplay ();
#ifdef VIEW3D
  } else if(key == '=') {
  	eye_separation += 0.5;
  	//glutPostRedisplay ();
  } else if(key == '-') {
  	if (eye_separation > 0.5)
	  	eye_separation -= 0.5;
  	//glutPostRedisplay ();
  /*
  } else if(key == '+') {
  	eye_focus_depth += 1.0;
  	//glutPostRedisplay ();
  } else if(key == '_') {
  	//if (eye_focus_depth > 1.0)
	  	eye_focus_depth -= 1.0;
  	//glutPostRedisplay ();
  */
#endif
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
	
	processInput();
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
    "  Phantom control input:\n"
    "    The red/green small cylinders (cursors) represent the true position and \n      orientation of the haptic device.\n"
    "    The cursor can be open or closed, and this is represented by a green or \n      red cursor respectively.\n"
    "    The cursor can be attached to an end effector, which inherits the \n      open/close status of the cursor that holds it.\n"
    "    To attach the cursor to an end effector, the cursor have to be near and \n      aligned (orientation) to the end effector's yellow cylinder.\n"
    "    Press the haptic's upper button to change between open and closed.\n"
    "    Press the haptic's lower button to attach or dettach the cursor to or from \n      the end effector.\n"
    "\n"
    "  Mouse and keyboard control input:\n"
    "    Only one end effector can be selected at a time. The selected end effector \n      has a darker color.\n"
    "    Press 'n' or 'b' to select the next or previous end effector, respectively.\n"
    "    Hold 'm' while holding down the left or right mouse button to move the \n      selected end effector.\n"
    "    Hold 't' while holding down the left or right mouse button to orientate \n      the tangent of the selected end effector.\n"
    "    Hold 'r' while holding down the left or right mouse button to roll the \n      tangent of the selected end effector.\n"
    "\n"
    "In choose mode:\n"
    "  Use this mode to add constraints (with end effectors) to the thread using \n    only the mouse and keyboard.\n"
    "  Any end effectors held by the cursor and not holding the thread are dropped \n    when changing into this mode\n"
    "  Constrained vertices are represented by red spheres. Free vertices where an \n    end effector can be attached are represented by green spheres.\n"
    "  There is one end effector that has a darker color. Use this end effector to \n    toggle between vertices.\n"
    "  If the darker end effector is open, there is not a constraint in the \n    selected vertex, and viceversa.\n"
    "  The orientation of the end effector _relative_ to the thread can be changed \n    using 't' and 'r' and the mouse in a similar way as in normal mode.\n"
    "  Press 'b' to close the end effector and thus constraint the vertex, or to \n    open the end effector and thus remove the constraint from the vertex.\n"
    "  Press 'n' or 'b' to select the next or previous vertex, respectively.\n"
    "\n"
    "Press 'w' to reset view.\n"
    "Press 'q' or 'ESC' to quit.\n");

	/* initialize glut */
	glutInit (&argc, argv); //can i do that?
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
#ifdef VIEW3D
	int screenWidth, screenHeight;
 	screenWidth = glutGet(GLUT_SCREEN_WIDTH);
	screenHeight = glutGet(GLUT_SCREEN_HEIGHT);
	glutInitWindowSize(screenWidth/2.0, screenHeight);
	main_window = glutCreateWindow ("Thread");
#else
	glutInitWindowSize(900,900);
	glutCreateWindow ("Thread");
#endif
  glutPositionWindow(0,0);
	glutDisplayFunc (drawStuff);
	glutMotionFunc (MouseMotion);
  glutMouseFunc (processMouse);
  glutKeyboardFunc(processNormalKeys);
  glutKeyboardUpFunc(processKeyUp);
  if (HAPTICS)
  	glutTimerFunc(100, processHapticDevice, 0);
	
	/* create popup menu */
	glutCreateMenu (glutMenu);
	glutAddMenuEntry ("Exit", 0);
	glutAttachMenu (GLUT_RIGHT_BUTTON);

	initGL();
	
#ifdef VIEW3D
	side_window = glutCreateWindow ("Thread");
	glutPositionWindow(screenWidth/2.0, 0);
	initGL();
	glutSetCursor(GLUT_CURSOR_NONE);
	glutSetWindow(main_window);
#endif
	
  initThread();
	thread->minimize_energy();
  thread->getConstrainedTransforms(positions, rotations);

  zero_location = Vector3d::Zero(); //points[0];
  zero_angle = 0.0;
	
	textured_sphere = new SimpleTexturedSphere(Vector3d::Zero(), 150.0, "../utils/wmap.bmp");
	env = new SimpleEnv();
	
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
	
	processInput();
  glutMainLoop ();
}

void drawStuff()
{
#ifdef VIEW3D
 	glutSetWindow(main_window);
#endif

  glPushMatrix ();
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  /* set up some matrices so that the object spins with the mouse */
  glTranslatef (translate_frame[0], translate_frame[1], translate_frame[2]);
  glRotatef (rotate_frame[1], 1.0, 0.0, 0.0);
  glRotatef (rotate_frame[0], 0.0, 0.0, 1.0);	
#ifdef VIEW3D
	glRotatef (+atan(eye_separation/(2.0*eye_focus_depth)) * 180.0/M_PI, 0.0, 1.0, 0.0);
#endif
  env->drawObjs();
  glPopMatrix();
  glutSwapBuffers ();
  
#ifdef VIEW3D
	glutSetWindow(side_window);
	glPushMatrix ();  
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	/* set up some matrices so that the object spins with the mouse */
	glTranslatef (translate_frame[0], translate_frame[1], translate_frame[2]);
  glRotatef (rotate_frame[1], 1.0, 0.0, 0.0);
  glRotatef (rotate_frame[0], 0.0, 0.0, 1.0);
	glRotatef (-atan(eye_separation/(2.0*eye_focus_depth)) * 180.0/M_PI, 0.0, 1.0, 0.0);
	env->drawObjs();
	glPopMatrix();
	glutSwapBuffers ();
		
	glutSetWindow(main_window);
	
	cout << "eye_separation: " << eye_separation << "\t" << "eye_focus_depth: " << eye_focus_depth << "\t" << "angle(degrees): " << atan(eye_separation/(2.0*eye_focus_depth)) * 180.0/M_PI << endl;
#endif
  
	env->clearObjs();
}

void processInput()
{
	glPushMatrix ();
  /* set up some matrices so that the object spins with the mouse */
  glTranslatef (translate_frame[0], translate_frame[1], translate_frame[2]);
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
		  	//drawSphere(all_positions[operable_vertices_num[i]], 1.2, 1.0, 0.0, 0.0);
		  	env->addObj(SimpleSphere(all_positions[operable_vertices_num[i]], 1.2, 1.0, 0.0, 0.0));
		  } else {
		  	//drawSphere(all_positions[operable_vertices_num[i]], 1.2, 0.0, 1.0, 0.0);
		  	env->addObj(SimpleSphere(all_positions[operable_vertices_num[i]], 1.2, 0.0, 1.0, 0.0));
		  }
		}
		for (int i=0; i<constrained_vertices_nums.size(); i++) {
			constrained_ee[i]->setTransform(all_positions[constrained_vertices_nums[i]], all_rotations[constrained_vertices_nums[i]]);
		}
	  int selected_vertex_ind = find(constrained_vertices_nums, selected_vertex_num);
		if (constrained_or_free[toggle]) {
	  	//drawSphere(all_positions[selected_vertex_num], 1.4, 0.5, 0.0, 0.0);
	  	env->addObj(SimpleSphere(all_positions[selected_vertex_num], 1.4, 0.5, 0.0, 0.0));
	  	if (selected_vertex_ind == -1)
	  		cout << "Internal error: drawStuff(): if (constrained_or_free[toggle]): selected vertex should be constrained by an end effector." << endl;
	  	constrained_ee[selected_vertex_ind]->highlight();
	  } else {
	  	//drawSphere(all_positions[selected_vertex_num], 1.4, 0.0, 0.5, 0.0);
	  	env->addObj(SimpleSphere(all_positions[selected_vertex_num], 1.4, 0.0, 0.5, 0.0));
	  	if (selected_vertex_ind != -1)
	  		cout << "Internal error: drawStuff(): if (constrained_or_free[toggle]): selected vertex should be constrained by an end effector." << endl;
  		//drawEndEffector(all_positions[selected_vertex_num], all_rotations[selected_vertex_num], 15, 0.4, 0.4, 0.4);
  		env->addObj(SimpleEndEffector(all_positions[selected_vertex_num], all_rotations[selected_vertex_num], 15, 0.4, 0.4, 0.4));
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
				
		for(int i=0; i<positions.size(); i++) {
			constrained_ee[i]->unhighlight();
			constrained_ee[i]->setTransform(positions[i], rotations[i]);
		}
		constrained_ee[toggle]->highlight();
			
		if (cursor0->isAttached() && cursor0->end_eff->constraint < 0) {
			//cursor0->end_eff->draw();
			env->addObj(cursor0->end_eff);
		}
		if (cursor1->isAttached() && cursor1->end_eff->constraint < 0) {
			//cursor1->end_eff->draw();
			env->addObj(cursor0->end_eff);
		}
		//cursor0->draw();
		//cursor1->draw();
		env->addObj(cursor0);
		env->addObj(cursor1);
	}
	
	//thread->adapt_links();
	
	//drawThread();
	//thread->draw();
	env->addObj(thread);
	
	//base->draw();
	env->addObj(base);
	
	//extra_end_effector->draw();
	env->addObj(extra_end_effector);
	
	for(int i=0; i<positions.size(); i++) {
		//constrained_ee[i]->draw();
		env->addObj(constrained_ee[i]);
	}
	
	env->addObj(textured_sphere);
	
	glPopMatrix();
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
			thread->getConstrainedVerticesNums(constrained_vertices_nums);
			if (cursor==cursor0 && cursor1->end_eff!=NULL)
				cursor1->end_eff->constraint_ind = find(constrained_vertices_nums, cursor1->end_eff->constraint);
			else if (cursor==cursor1 && cursor0->end_eff!=NULL)
				cursor0->end_eff->constraint_ind = find(constrained_vertices_nums, cursor0->end_eff->constraint);
			else if ((cursor != cursor0) && (cursor != cursor1))
				cout << "Internal error: updateState(): (cursor->attach_dettach_attempt): assumes there's only two cursors." << endl;
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

bool closeEnough(Vector3d my_pos, Matrix3d my_rot, Vector3d pos, Matrix3d rot) {
	double angle = 2*asin((my_rot.col(0) - rot.col(0)).norm()/2);
  return (((my_pos - pos).norm() < 4.0) && angle < 0.25*M_PI);
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
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);    
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
	
	glEnable(GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}


void glutMenu(int ID) {
	switch(ID) {
    case 0:
      exit(0);
      break;
  }
}

