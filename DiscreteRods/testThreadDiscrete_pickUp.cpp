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
#include <stdarg.h>

#include <boost/algorithm/string.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>

#include "IO/ControlBase.h"
#include "IO/Mouse.h"
#include "IO/Haptic.h"
#include "thread_socket_interface.h"

#include "thread_discrete.h"
#include "ThreadConstrained.h"
#include "EnvObjects/World.h"
#include "EnvObjects/EnvObject.h"
#include "EnvObjects/Capsule.h"
#include "EnvObjects/Cursor.h"
#include "EnvObjects/EndEffector.h"
#include "EnvObjects/InfinitePlane.h"
#include "EnvObjects/TexturedSphere.h"

#include "StateRecorder.h"
#include "StateReader.h"
#include "TrajectoryRecorder.h"
#include "TrajectoryReader.h"

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

void updateCursorFromButton(Cursor* cursor, ControlBase* control);
void updateCursorFromTransform(Cursor* cursor, ControlBase* control);
void processInput(ControlBase* control0, ControlBase* control1);
void updateObjectsFromCursor(Cursor* cursor);
void linkObjectsToWorldObjects();
void displayTextInScreen(const char* textline, ...);
void initThread();
void initLongerThread();
void initRestingThread();
void glutMenu(int ID);
void initGL();

#define NUM_PTS 500
#define THREAD_RADII 1.0
#define VIEW3D

float lastx_L=0;
float lasty_L=0;
float lastx_M=0;
float lasty_M=0;

int pressed_mouse_button;
float rotate_frame[2] = { 0.0, 0.0 };
float last_rotate_frame[2];
float translate_frame[3] = { 0.0, 0.0, -110.0 };
#ifdef VIEW3D
int main_window = 0;
int side_window = 0;
float eye_separation = 7.0;
float eye_focus_depth = 0.0; // distance from sphere center to focus point
#endif

GLdouble model_view[16];
GLdouble projection[16];
GLint viewport[4];
int window_width, window_height;

// currently not used. should they be used?
Vector3d zero_location;
double zero_angle;

vector<ThreadConstrained*> threads;

// interactive variables
bool limit_displacement = false;
bool haptics = false;

//IO
Haptic *haptic0, *haptic1;
Mouse *mouse0, *mouse1;

//Objects in environment
World *world;
Cursor *cursor0, *cursor1;
vector<EndEffector*> end_effectors;
InfinitePlane *plane;
TexturedSphere *textured_sphere;

//For recording and playing back trajectories
TrajectoryRecorder trajectory_recorder;
vector<World*> worlds;
int world_ind = 0;

void processLeft(int x, int y)
{
	if (mouse0->getKeyPressed() != NONE) {
		mouse0->add2DMove(x-lastx_L, lasty_L-y);
	} else if (mouse1->getKeyPressed() != NONE) {
		mouse1->add2DMove(x-lastx_L, lasty_L-y);
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

void processMiddle(int x, int y)
{
#ifdef VIEW3D
	if ((((x-lastx_M) < 0.0) && ((eye_focus_depth + 50.0 + 5.0) < (-translate_frame[2]))) ||
			((x-lastx_M) > 0.0))
		translate_frame[2] -= x-lastx_M;
#else
	translate_frame[2] -= x-lastx_M;
#endif
	lastx_M = x;
	lasty_M = y;
}

void mouseMotion (int x, int y)
{
  if (pressed_mouse_button == GLUT_LEFT_BUTTON) {
   	processLeft(x, y);
  } else if (pressed_mouse_button == GLUT_MIDDLE_BUTTON) {
    processMiddle(x,y);
  }
  
	if (!haptics) {
		mouse0->applyTransformFrom2DMove(model_view, projection, viewport);
		mouse1->applyTransformFrom2DMove(model_view, projection, viewport);
		updateCursorFromTransform(cursor0, mouse0);
		updateCursorFromTransform(cursor1, mouse1);
		processInput(mouse0, mouse1);
	}
	glutPostRedisplay ();
}

void processMouse(int button, int state, int x, int y)
{
	if (state == GLUT_DOWN) {
    pressed_mouse_button = button;
    if (button == GLUT_LEFT_BUTTON) {
      lastx_L = x;
      lasty_L = y;
    }
    if (button == GLUT_MIDDLE_BUTTON) {
      lastx_M = x;
      lasty_M = y;
    }
	}
}

void processNormalKeys(unsigned char key, int x, int y)
{
	if (key == 't')
	  mouse0->setKeyPressed(MOVETAN);
  else if (key == 'm')
    mouse0->setKeyPressed(MOVEPOS);
  else if (key == 'r')
    mouse0->setKeyPressed(ROTATETAN);
  else if (key == 'T')
	  mouse1->setKeyPressed(MOVETAN);
  else if (key == 'M')
    mouse1->setKeyPressed(MOVEPOS);
  else if (key == 'R')
    mouse1->setKeyPressed(ROTATETAN);
 	else if (key == 'u')
    mouse0->setButtonState(true, UP);
  else if (key == 'j')
    mouse0->setButtonState(true, DOWN);
  else if (key == 'U')
    mouse1->setButtonState(true, UP);
  else if (key == 'J')
    mouse1->setButtonState(true, DOWN);
  else if (key == 'w') {
    rotate_frame[0] = 0.0;
    rotate_frame[1] = 0.0;
	} else if(key == 's') {
    cout << "Saving...\n";
    cout << "Please enter destination file name (without extension): ";
    char *dstFileName = new char[256];
    cin >> dstFileName;
    char *fullPath = new char[256];
    sprintf(fullPath, "%s%s", "environmentFiles/", dstFileName);
		StateRecorder state_recorder(fullPath);
    state_recorder.writeObjectsToFile(world);
  } else if((key == 'a') || (key >= '0' && key <= '9')) {
  	cout << "Loading...\n";
  	StateReader state_reader;
  	char *fullPath = new char[256];
  	if (key == 'l') {
  		cout << "Please enter destination file name (without extension): ";
	  	char *dstFileName = new char[256];
    	cin >> dstFileName;
    	sprintf(fullPath, "%s%s", "environmentFiles/", dstFileName);
  	} else {
	    sprintf(fullPath, "%s%s%c", "environmentFiles/", "o", key);
	  }
    state_reader.setFileName(fullPath);
    if (state_reader.readObjectsFromFile(world)) {
			linkObjectsToWorldObjects();
			cout << "State loading was sucessful." << endl;
		}
	} else if(key == 'c') {
		cout << "Starting trajectory and saving...\n";
    cout << "Please enter destination file name (without extension): ";
    char *dstFileName = new char[256];
    cin >> dstFileName;
    char *fullPath = new char[256];
    sprintf(fullPath, "%s%s", "environmentFiles/", dstFileName);
		trajectory_recorder.setFileName(fullPath);
		trajectory_recorder.start();
	} else if(key == 'x') {
		cout << "Finished saving trajectory.\n";
		trajectory_recorder.stop();
	} else if((key == 'z') || 
						(key == '!') || (key == '@') || (key == '#') || (key == '$') || (key == '%') ||
						(key == '^') || (key == '&') || (key == '*') || (key == '(') || (key == ')')) {
		char map_key;
		if 			(key == '!') 		map_key = '1';
		else if (key == '@') 		map_key = '2';
		else if (key == '#')		map_key = '3';
		else if (key <= '$')		map_key = '4';
		else if (key <= '%')		map_key = '5';
		else if (key <= '^')		map_key = '6';
		else if (key <= '&')		map_key = '7';
		else if (key <= '*')		map_key = '8';
		else if (key <= '(')		map_key = '9';
		else if (key <= ')')		map_key = '0';
		cout << "Loading trajectory...\n";
  	char *fullPath = new char[256];
		if (key == 'z') {
  		cout << "Please enter destination file name (without extension): ";
	  	char *dstFileName = new char[256];
    	cin >> dstFileName;
    	sprintf(fullPath, "%s%s", "environmentFiles/", dstFileName);
  	} else {
	    sprintf(fullPath, "%s%s%c", "environmentFiles/", "t", map_key);
	  }
  	TrajectoryReader trajectory_reader(fullPath);
  	trajectory_reader.readStatesFromFile(worlds);
  	cout << "worlds size " << worlds.size() << endl;
  	world_ind = 0;
  	world = worlds[world_ind];
  	linkObjectsToWorldObjects();
	} else if(key == '[') {
		if (worlds.size() > 0) {
			world_ind = max(0, world_ind-1);
			world = worlds[world_ind];
			linkObjectsToWorldObjects();
			cout << "world " << world_ind << " / " << worlds.size() << endl;
		} else { cout << "There is no previous state. worlds is empty." << endl; }
	} else if(key == ']') {
		if (worlds.size() > 0) {
			world_ind = min((int) worlds.size()-1, world_ind+1);
			world = worlds[world_ind];
			linkObjectsToWorldObjects();
			cout << "world " << world_ind << " / " << worlds.size() << endl;
		} else { cout << "There is no next state. worlds is empty." << endl; }
	} else if(key == 'l') {
		limit_displacement = !limit_displacement;
		for (int ee_ind = 0; ee_ind < end_effectors.size(); ee_ind++)
			end_effectors[ee_ind]->setLimitDisplacement(limit_displacement);
  } else if(key == 'e') {
  	for (int thread_ind=0; thread_ind<threads.size(); thread_ind++)
  		threads[thread_ind]->toggleExamineMode();
  } else if(key == 'h') {
  	haptics = !haptics;
  	if (haptics) {
  		haptic0->resetPosition(mouse0->getPosition());
  		haptic1->resetPosition(mouse1->getPosition());
  	} else {
  		mouse0->setTransform(haptic0);
  		mouse1->setTransform(haptic1);
  	}

#ifdef VIEW3D
  } else if(key == '=') {
  	eye_separation += 0.5;
  } else if(key == '-') {
  	if (eye_separation > 0.5)
	  	eye_separation -= 0.5;
  } else if(key == '+') {
  	if ((eye_focus_depth + 50.0 + 5.0) < (-translate_frame[2]))
  		eye_focus_depth += 1.0;
  } else if(key == '_') {
	 	eye_focus_depth -= 1.0;
#endif
  }	else if (key == 'q' || key == 27) {
    exit(0);
  }
  glutPostRedisplay ();
}

void processKeyUp(unsigned char key, int x, int y)
{
  mouse0->setKeyPressed(NONE);
  mouse1->setKeyPressed(NONE);
  mouse0->add2DMove(0.0, 0.0);
  mouse1->add2DMove(0.0, 0.0);
  
  if (!haptics && (key == 'u') || (key == 'j') || (key == 'U') || (key == 'J')) {	
		mouse0->setButtonState(false, UP);
		mouse0->setButtonState(false, DOWN);
		mouse1->setButtonState(false, UP);
		mouse1->setButtonState(false, DOWN);
		updateCursorFromButton(cursor0, mouse0);
		updateCursorFromButton(cursor1, mouse1);
		processInput(mouse0, mouse1);
		glutPostRedisplay();
	}
}

void processSpecialKeys(int key, int x, int y) {
	switch(key) {
		case GLUT_KEY_LEFT :
				translate_frame[0] += 1;
				break;
		case GLUT_KEY_RIGHT :
				translate_frame[0] -= 1;
				break;
		case GLUT_KEY_UP :
				#ifdef VIEW3D
					if ((((x-lastx_M) < 0.0) && ((eye_focus_depth + 50.0 + 5.0) < (-translate_frame[2]))) ||
							((x-lastx_M) > 0.0))
						translate_frame[2] += 1;
				#else
					translate_frame[2] += 1;
				#endif
				break;
		case GLUT_KEY_DOWN :
				#ifdef VIEW3D
					if ((((x-lastx_M) < 0.0) && ((eye_focus_depth + 50.0 + 5.0) < (-translate_frame[2]))) ||
							((x-lastx_M) > 0.0))
						translate_frame[2] -= 1;
				#else
					translate_frame[2] -= 1;
				#endif
				break;
		case GLUT_KEY_PAGE_UP :
				translate_frame[1] -= 1;
				break;
		case GLUT_KEY_PAGE_DOWN :
				translate_frame[1] += 1;
				break;
	}
}

void processHapticDevice(int value)
{
	Vector3d start_proxy_pos, end_proxy_pos;
	Matrix3d start_proxy_rot, end_proxy_rot;
	bool start_proxybutton[2], end_proxybutton[2];
	
	if (getDeviceState(start_proxy_pos, start_proxy_rot, start_proxybutton, end_proxy_pos, end_proxy_rot, end_proxybutton)) {
		haptic0->setRelativeTransform(start_proxy_pos, start_proxy_rot);
		haptic0->setButtonState(start_proxybutton[UP], UP);
		haptic0->setButtonState(start_proxybutton[DOWN], DOWN);
		haptic1->setRelativeTransform(end_proxy_pos, end_proxy_rot);
		haptic1->setButtonState(end_proxybutton[UP], UP);
		haptic1->setButtonState(end_proxybutton[DOWN], DOWN);
	}

	if (haptics) {
		updateCursorFromButton(cursor0, haptic0);
		updateCursorFromButton(cursor1, haptic1);
		updateCursorFromTransform(cursor0, haptic0);		
		updateCursorFromTransform(cursor1, haptic1);
		processInput(haptic0, haptic1);
		glutPostRedisplay ();
	}
	glutTimerFunc(100, processHapticDevice, value);
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
#ifdef VIEW3D
	glTranslatef(0.0, 0.0, +eye_focus_depth);
  drawSphere(Vector3d::Zero(), 1, 0.8, 0.8, 0.8);
	// (-translate_frame[2]) is distance from camera to sphere center
	// (-translate_frame[2] - eye_focus_depth) is distance from camera to focus point
	glRotatef (+atan(eye_separation/(2.0*(-translate_frame[2]-eye_focus_depth))) * 180.0/M_PI, 0.0, 1.0, 0.0);
	glTranslatef(0.0, 0.0, -eye_focus_depth);
#endif
  glRotatef (rotate_frame[1], 1.0, 0.0, 0.0);
  glRotatef (rotate_frame[0], 0.0, 0.0, 1.0);
  glGetDoublev(GL_MODELVIEW_MATRIX, model_view);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);
  world->draw();
  glPopMatrix();
#ifdef VIEW3D
  displayTextInScreen("eye separation: %.2f\ncamera to focus point: %.2f\ncamera to sphere center: %.2f", eye_separation, (-translate_frame[2] - eye_focus_depth), (-translate_frame[2]));
#endif
  glutSwapBuffers ();
  
#ifdef VIEW3D
	glutSetWindow(side_window);
	glPushMatrix ();  
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	/* set up some matrices so that the object spins with the mouse */
	glTranslatef (translate_frame[0], translate_frame[1], translate_frame[2]);
	glTranslatef(0.0, 0.0, +eye_focus_depth);
  drawSphere(Vector3d::Zero(), 1, 0.8, 0.8, 0.8);
	glRotatef (-atan(eye_separation/(-2.0*translate_frame[2]-eye_focus_depth)) * 180.0/M_PI, 0.0, 1.0, 0.0);
	glTranslatef(0.0, 0.0, -eye_focus_depth);
  glRotatef (rotate_frame[1], 1.0, 0.0, 0.0);
  glRotatef (rotate_frame[0], 0.0, 0.0, 1.0);
	world->draw();
	glPopMatrix();
	displayTextInScreen("eye separation: %.2f\ncamera to focus point: %.2f\ncamera to sphere center: %.2f", eye_separation, (-translate_frame[2] - eye_focus_depth), (-translate_frame[2]));
	glutSwapBuffers ();
		
	glutSetWindow(main_window);
#endif
}

int main (int argc, char * argv[])
{
  srand(time(NULL));
  srand((unsigned int)time((time_t *)NULL));

  printf("Instructions:\n"
    "\n"
    "There are two types of control: haptic (Phantom) and normal (mouse and \nkeyboard).\n"
    "The 3D cursor (represented by a capsule) is controlled by either one of the \ncontrols.\n"
    "Initially, the 3D cursor is controlled by the normal device. To change the \ncontrol, press 'h'.\n"
    "\n"
    "The upper part of the 3D cursor is where the white ring is closer to.\n"
    "The cursor represent the true position and orientation of the control.\n"
    "The cursor can be open or closed, and this is represented by a green or red \ncolor on the upper part of it respectively.\n"
    "The cursor can be attached to an end effector, which inherits the open/close \nstatus of the cursor that holds it.\n"
    "To attach the cursor to an end effector, the cursor have to be NEAR and \nALIGNED (orientation) to the end effector's yellow cylinder.\n"
    "Press the control's upper button to change between open and closed.\n"
    "Press the control's lower button to attach or dettach the cursor to or from \nthe end effector.\n"
    "\n"
    "Manipule the cursor using the normal control (use SHIFT for controlling the \nother cursor):\n"
    "  Move:                    Left mouse button and 'm'.\n"
    "  Move tangent:            Left mouse button and 't'.\n"
    "  Rotate around tangent:   Left mouse button and 'r'.\n"
    "  Upper button:            'u'.\n"
    "  Lower button:            'j'.\n"
    "\n"
    "Change views:\n"
    "  Rotate:                  Left mouse button.\n"
    "  Move:                    Left mouse button and CTRL.\n"
    "  Zoom:                    Middle mouse button.\n"
    "  You can also translate using the arrows keys for x and z translation and \n  PAGE_UP/PAGE_DOWN for y translation.\n"
    "  Reset rotation:          'w'.\n"
    "\n"
    "Press 'q' or 'ESC' to quit.\n");

	/* initialize glut */
	glutInit (&argc, argv); //can i do that?
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
#ifdef VIEW3D	
	int screen_width, screen_height;
 	screen_width = glutGet(GLUT_SCREEN_WIDTH);
	screen_height = glutGet(GLUT_SCREEN_HEIGHT);
	glutInitWindowSize(screen_width/4.0, screen_height);
	window_width = screen_width/4.0;
	window_height = screen_height;
	
	side_window = glutCreateWindow ("Thread");
	glutPositionWindow(3.0*screen_width/4.0, 0);
	initGL();
	glutSetCursor(GLUT_CURSOR_NONE);
	
	main_window = glutCreateWindow ("Thread");
	glutPositionWindow(screen_width/2.0,0);
#else
	glutInitWindowSize(900,900);
	window_width = 900;
	window_height = 900;
	glutCreateWindow ("Thread");
	glutPositionWindow(0,0);
#endif
	glutDisplayFunc (drawStuff);
	glutMotionFunc (mouseMotion);
  glutMouseFunc (processMouse);
  glutKeyboardFunc(processNormalKeys);
  glutKeyboardUpFunc(processKeyUp);
  //TODO handle keyUps differently for u and j and not for all the keys
  //glutIgnoreKeyRepeat(1);
  glutSpecialFunc(processSpecialKeys);

 	glutTimerFunc(100, processHapticDevice, 0);
		
	/* create popup menu */
	glutCreateMenu (glutMenu);
	glutAddMenuEntry ("Exit", 0);
	glutAttachMenu (GLUT_RIGHT_BUTTON);
	
	initGL();
	
  //initThread();
  //initLongerThread();
  initRestingThread();

  zero_location = Vector3d::Zero();
  zero_angle = 0.0;

	//IO
	mouse0 = new Mouse();
	mouse1 = new Mouse();
	haptic0 = new Haptic();
	haptic1 = new Haptic();
	connectionInit();

	glGetDoublev(GL_MODELVIEW_MATRIX, model_view);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);
	
	//setting up objects in environment
	cursor0 = new Cursor(Vector3d::Zero(), Matrix3d::Identity());
	cursor1 = new Cursor(Vector3d::Zero(), Matrix3d::Identity());	
	plane = new InfinitePlane(Vector3d(0.0, -30.0, 0.0), Vector3d(0.0, 1.0, 0.0), "../utils/textures/checkerBoardSquare32.bmp");
	//plane = new InfinitePlane(Vector3d(0.0, -30.0, 0.0), Vector3d(0.0, 1.0, 0.0), 0.6, 0.6, 0.6);
	textured_sphere = new TexturedSphere(Vector3d::Zero(), 150.0, "../utils/textures/checkerBoardRect16.bmp");
	
	//setting up end effectors
	vector<Vector3d> positions;
	vector<Matrix3d> rotations;
	threads[0]->getConstrainedTransforms(positions, rotations);
	EndEffector* end_effector0 = new EndEffector(positions[0], rotations[0]);
	end_effector0->attachThread(threads[0]);
	end_effector0->constraint_ind = 0;
	end_effector0->updateConstraint();
	end_effectors.push_back(end_effector0);

	EndEffector* end_effector1 = new EndEffector(positions[1], rotations[1]);
	end_effector1->attachThread(threads[0]);
	end_effector1->constraint_ind = 1;
	end_effector1->updateConstraint();	
	end_effectors.push_back(end_effector1);
	
	threads[1]->getConstrainedTransforms(positions, rotations);
	EndEffector* end_effector2 = new EndEffector(positions[0], rotations[0]);
	end_effector2->attachThread(threads[1]);
	end_effector2->constraint_ind = 0;
	end_effector2->updateConstraint();
	end_effectors.push_back(end_effector2);
	
	EndEffector* end_effector3 = new EndEffector(positions[1], rotations[1]);
	end_effector3->attachThread(threads[1]);
	end_effector3->constraint_ind = 1;
	end_effector3->updateConstraint();
	end_effectors.push_back(end_effector3);
	
	EndEffector* end_effector4 = new EndEffector(plane->getPosition() + Vector3d(30.0, EndEffector::short_handle_r, 0.0), (Matrix3d) AngleAxisd(-M_PI/2.0, Vector3d::UnitY()) * AngleAxisd(M_PI/2.0, Vector3d::UnitX()));
	end_effectors.push_back(end_effector4);
	
	EndEffector* end_effector5 = new EndEffector(plane->getPosition() + Vector3d(35.0, EndEffector::short_handle_r, 0.0), (Matrix3d) AngleAxisd(-M_PI/2.0, Vector3d::UnitY()) * AngleAxisd(M_PI/2.0, Vector3d::UnitX()));
	end_effectors.push_back(end_effector5);

	// adding objects to environment
	world = new World();
	for (int thread_ind = 0; thread_ind < threads.size(); thread_ind++)
		world->addThread(threads[thread_ind]);
	world->addEnvObj(cursor0);
	world->addEnvObj(cursor1);
	for (int ee_ind = 0; ee_ind < end_effectors.size(); ee_ind++)
		world->addEnvObj(end_effectors[ee_ind]);
	world->addEnvObj(plane);
	world->addEnvObj(textured_sphere);
	world->initializeThreadsInEnvironment();
	
	if (haptics)
		processInput(haptic0, haptic1);
	else
		processInput(mouse0, mouse1);
  glutMainLoop ();
}

// This function updates the open/close and attach/dettach state of the 
// cursor based on the button state change of control. This should always 
// be called after a possible change of a control's button state.
// However, it should not be called if the control's button state might
// not have changed; i.e. don't call it from mouseMotion. bug?
void updateCursorFromButton(Cursor* cursor, ControlBase* control)
{
	cursor->saveLastOpen();
	if (control->hasButtonPressed(UP))
		cursor->openClose();
	cursor->attach_dettach_attempt = control->hasButtonPressed(DOWN);
}

void updateCursorFromTransform(Cursor* cursor, ControlBase* control)
{
	cursor->setTransform(control->getPosition(), control->getRotation());
}

void processInput(ControlBase* control0, ControlBase* control1)
{
	updateObjectsFromCursor(cursor0);
	updateObjectsFromCursor(cursor1);

	for (int thread_ind = 0; thread_ind < threads.size(); thread_ind++) {
		
		vector<EndEffector*> thread_end_effs;
		for (int ee_ind = 0; ee_ind < end_effectors.size(); ee_ind++)
			if (end_effectors[ee_ind]->getThread()==threads[thread_ind])
				thread_end_effs.push_back(end_effectors[ee_ind]);

		vector<Vector3d> positionConstraints;
		vector<Matrix3d> rotationConstraints;
		threads[thread_ind]->getConstrainedTransforms(positionConstraints, rotationConstraints);

		for (int ee_ind = 0; ee_ind < thread_end_effs.size(); ee_ind++) {
			EndEffector* ee = thread_end_effs[ee_ind];
			positionConstraints[ee->constraint_ind] = ee->getPosition();
			rotationConstraints[ee->constraint_ind] = ee->getRotation();
		}

		threads[thread_ind]->updateConstraints(positionConstraints, rotationConstraints);
		threads[thread_ind]->getConstrainedTransforms(positionConstraints, rotationConstraints);
		
		for (int ee_ind = 0; ee_ind < thread_end_effs.size(); ee_ind++) {
			EndEffector* ee = thread_end_effs[ee_ind];
			ee->forceSetTransform(positionConstraints[ee->constraint_ind], rotationConstraints[ee->constraint_ind]);
		}

		//threads[thread_ind]->adapt_links();
	}

	if (trajectory_recorder.hasStarted()) {
		trajectory_recorder.writeStateToFile(world);
	}

	glPopMatrix();
}

inline bool closeEnough(const Vector3d& my_pos, const Matrix3d& my_rot, const Vector3d& pos, const Matrix3d& rot)
{
	double angle = 2*asin((my_rot.col(0) - rot.col(0)).norm()/2);
  return (((my_pos - pos).norm() < 4.0) && angle < 0.25*M_PI);
}

void updateObjectsFromCursor(Cursor* cursor)
{
	const Vector3d tip_pos = cursor->getPosition() - EndEffector::grab_offset * cursor->getRotation().col(0);
	const Matrix3d tip_rot = cursor->getRotation();
	
	if (cursor->isAttached()) {																																				// cursor has an end effector
		EndEffector* ee = cursor->end_eff;
	  if (!ee->isThreadAttached()) {																																					// cursor has an end effector but it isn't holding the thread
	  	int nearest_vertex = threads[0]->nearestVertex(tip_pos);
	  	ThreadConstrained* thread = threads[0];
	  	for (int thread_ind = 1; thread_ind < threads.size(); thread_ind++) {
	  		int candidate_nearest_vertex = threads[thread_ind]->nearestVertex(tip_pos);
	  		if ( (threads[thread_ind]->position(candidate_nearest_vertex) - tip_pos).squaredNorm() <
	  				 (threads[thread_ind]->position(nearest_vertex) - tip_pos).squaredNorm() ) {	  		
	  			nearest_vertex = candidate_nearest_vertex;
	  			thread = threads[thread_ind];
	  		}
	  	}
	  	if (cursor->justClosed() && ((thread->position(nearest_vertex) - tip_pos).squaredNorm() < 32.0)) {	// cursor has an end effector which just started holding the thread
	  		ee->constraint = nearest_vertex;
		    thread->addConstraint(nearest_vertex);
		    ee->attachThread(thread);
		    for (int ee_ind = 0; ee_ind < end_effectors.size(); ee_ind++) {
					end_effectors[ee_ind]->updateConstraintIndex();
		    }
		    thread->setConstrainedTransforms(ee->constraint_ind, tip_pos, tip_rot);										// the end effector's orientation matters when it grips the thread. This updates the offset rotation.
		  }
	  } else {																																												// cursor has an end effector which is holding the thread
	  	if ((ee->constraint==0 || ee->constraint==(ee->getThread()->numVertices()-1)) && cursor->isOpen()) {		// cursor has an end effector which is holding the thread end and trying to be opened
				cout << "You cannot open the end effector holding the end constraint" << endl;
			 	cursor->forceClose();
			} else if (cursor->isOpen()) {																																// cursor has an end effector which is holding the thread and trying to be opened
			  ee->getThread()->removeConstraint(ee->constraint);
			 	ee->constraint = ee->constraint_ind = -1;
			 	ee->dettachThread();
			 	for (int ee_ind = 0; ee_ind < end_effectors.size(); ee_ind++)
					end_effectors[ee_ind]->updateConstraintIndex();
			}
		}
		ee->setTransform(tip_pos, tip_rot);
		if (cursor->attach_dettach_attempt) {
			cursor->dettach();
			for (int ee_ind = 0; ee_ind < end_effectors.size(); ee_ind++)
				end_effectors[ee_ind]->updateConstraintIndex();
		}
	} else {																				// cursor doesn't have an end effector, THUS it isn't holding the thread i.e. ee->constraint = ee->constraint_ind = -1;
  	for (int ee_ind = 0; ee_ind < end_effectors.size(); ee_ind++) {
  		EndEffector* ee = end_effectors[ee_ind];
  		if (cursor->attach_dettach_attempt && closeEnough(tip_pos, tip_rot, ee->getPosition(), ee->getRotation())) {
  			cursor->attach(ee);
  			if (ee->isThreadAttached()) {
					ee->updateConstraint();
  				if ((ee->constraint==0 || ee->constraint==(ee->getThread()->numVertices()-1)) && cursor->isOpen()) 
	  				cursor->forceClose();
  			}
  		}
  	}
 	}
	cursor->attach_dettach_attempt = false;
}

// This should be called everytime the pointer world is set to another
// world; i.e. after loading a world from a file.
void linkObjectsToWorldObjects()
{
	vector<ThreadConstrained*> world_thread = *(world->getThreads());
	for (int i = 0; i < threads.size(); i++) {
		threads[i] = world_thread[i];
	}

	vector<EnvObject*> world_cursors = world->getEnvObjs(CURSOR);
	cursor0 = dynamic_cast<Cursor*>(world_cursors[0]);
	cursor1 = dynamic_cast<Cursor*>(world_cursors[1]);

	vector<EnvObject*> world_end_effs = world->getEnvObjs(END_EFFECTOR);
	for (int i = 0; i < end_effectors.size(); i++) {
		end_effectors[i] = dynamic_cast<EndEffector*>(world_end_effs[i]);
	}

	vector<EnvObject*> world_planes = world->getEnvObjs(INFINITE_PLANE);
	plane = dynamic_cast<InfinitePlane*>(world_planes[0]);

	vector<EnvObject*> world_spheres = world->getEnvObjs(TEXTURED_SPHERE);
	textured_sphere = dynamic_cast<TexturedSphere*>(world_spheres[0]);
}

void displayTextInScreen(const char* textline, ...)
{
	glPushMatrix();
	glTranslatef(0.0, 0.0, -50.1 );
	va_list argList;
	char cbuffer[5000];
	va_start(argList, textline);
	vsnprintf(cbuffer, 5000, textline, argList);
	va_end(argList);
	vector<string> textvect;
	boost::split(textvect, cbuffer, boost::is_any_of("\n"));
	for (int i=0; i<textvect.size(); i++) {
		printText(-28, 28-i*1.5, textvect[i].c_str());
	}
  glPopMatrix();
}

void initThread()
{
  int numInit = 6;

	double first_length = 3.0; //FIRST_REST_LENGTH;
	double second_length = SECOND_REST_LENGTH;
	double middle_length = DEFAULT_REST_LENGTH;

  vector<Vector3d> vertices;
  vector<double> angles;
  vector<double> lengths;
  vector<Vector3d> directions;
  
	for (int i=0; i < 2*numInit + 5; i++)
		angles.push_back(0.0);
  
  lengths.push_back(first_length);
  lengths.push_back(second_length);
  for (int i=0; i < 2*numInit; i++)
  	lengths.push_back(middle_length);
  lengths.push_back(second_length);
  lengths.push_back(first_length);
  lengths.push_back(first_length);
  
  directions.push_back(Vector3d::UnitX());
  directions.push_back(Vector3d::UnitX());
  Vector3d direction = Vector3d(2.0, -1.0, 0.0);
  direction.normalize();
  for (int i=0; i < numInit; i++)
  	directions.push_back(direction);
  direction = Vector3d(1.0, -2.0, 0.0);
  direction.normalize();
  for (int i=0; i < numInit; i++)
  	directions.push_back(direction);
  directions.push_back(-Vector3d::UnitY());
  directions.push_back(-Vector3d::UnitY());
  
  vertices.push_back(Vector3d::Zero());
  for (int i=1; i < 2*numInit + 5; i++) {
  	Vector3d next_vertex;
  	next_vertex(0) = vertices[i-1](0) + directions[i-1](0) * lengths[i-1];
  	next_vertex(1) = vertices[i-1](1) + directions[i-1](1) * lengths[i-1];
  	next_vertex(2) = vertices[i-1](2) + directions[i-1](2) * lengths[i-1];
    vertices.push_back(next_vertex);
  }
  
	Vector3d last_pos = Vector3d(-10.0, -30.0, 0.0);
	for (int i=0; i<vertices.size(); i++)
		vertices[i] += -vertices.back() + last_pos;

  Matrix3d start_rotation0 = Matrix3d::Identity();
  Matrix3d end_rotation0 = (Matrix3d) AngleAxisd(-M_PI/2.0, Vector3d::UnitZ());

  ThreadConstrained* thread0 = new ThreadConstrained(vertices, angles, lengths, start_rotation0, end_rotation0);
  threads.push_back(thread0);
  
  for (int i=0; i<vertices.size(); i++)
		vertices[i](0) *= -1;
	Matrix3d start_rotation1 = (Matrix3d) AngleAxisd(M_PI, Vector3d::UnitZ());
  Matrix3d end_rotation1 = (Matrix3d) AngleAxisd(-M_PI/2.0, Vector3d::UnitZ());
  ThreadConstrained* thread1 = new ThreadConstrained(vertices, angles, lengths, start_rotation1, end_rotation1);
  threads.push_back(thread1);
  
	for (int thread_ind=0; thread_ind < threads.size(); thread_ind++) {
#ifndef ISOTROPIC
		Matrix2d B = Matrix2d::Zero();
		B(0,0) = 10.0;
		B(1,1) = 1.0;
		threads[thread_ind]->set_coeffs_normalized(B, 3.0, 1e-4);
#else
  	threads[thread_ind]->set_coeffs_normalized(1.0, 3.0, 1e-4);
#endif
	}
}

void initLongerThread()
{
  int numInit = 5;

	double first_length = 3.0; //FIRST_REST_LENGTH;
	double second_length = SECOND_REST_LENGTH;
	double middle_length = DEFAULT_REST_LENGTH;

  vector<Vector3d> vertices;
  vector<double> angles;
  vector<double> lengths;
  vector<Vector3d> directions;
  
	for (int i=0; i < 4*numInit + 5; i++)
		angles.push_back(0.0);
  
  lengths.push_back(first_length);
  lengths.push_back(second_length);
  for (int i=0; i < 4*numInit; i++)
  	lengths.push_back(middle_length);
  lengths.push_back(second_length);
  lengths.push_back(first_length);
  lengths.push_back(first_length);
  
  directions.push_back(Vector3d::UnitX());
  directions.push_back(Vector3d::UnitX());
  Vector3d direction = Vector3d(1.0, 1.0, 0.0);
  direction.normalize();
  for (int i=0; i < numInit; i++)
  	directions.push_back(direction);
  direction = Vector3d(1.0, -1.0, 0.4);
  direction.normalize();
  for (int i=0; i < numInit; i++)
  	directions.push_back(direction);
  direction = Vector3d(-1.0, -1.0, -0.4);
  direction.normalize();
  for (int i=0; i < numInit; i++)
  	directions.push_back(direction);
  direction = Vector3d(0.0, -1.0, 0.0);
  direction.normalize();
  for (int i=0; i < numInit; i++)
  	directions.push_back(direction);
  directions.push_back(-Vector3d::UnitY());
  directions.push_back(-Vector3d::UnitY());
  
  vertices.push_back(Vector3d::Zero());
  for (int i=1; i < 4*numInit + 5; i++) {
  	Vector3d next_vertex;
  	next_vertex(0) = vertices[i-1](0) + directions[i-1](0) * lengths[i-1];
  	next_vertex(1) = vertices[i-1](1) + directions[i-1](1) * lengths[i-1];
  	next_vertex(2) = vertices[i-1](2) + directions[i-1](2) * lengths[i-1];
    vertices.push_back(next_vertex);
  }
  
	Vector3d last_pos = Vector3d(-10.0, -30.0, 0.0);
	for (int i=0; i<vertices.size(); i++)
		vertices[i] += -vertices.back() + last_pos;

  Matrix3d start_rotation0 = Matrix3d::Identity();
  Matrix3d end_rotation0 = (Matrix3d) AngleAxisd(-M_PI/2.0, Vector3d::UnitZ());

  ThreadConstrained* thread0 = new ThreadConstrained(vertices, angles, lengths, start_rotation0, end_rotation0);
  threads.push_back(thread0);
  
  directions.clear();
  directions.push_back(Vector3d::UnitX());
  directions.push_back(Vector3d::UnitX());
  direction = Vector3d(1.0, 1.0, 0.0);
  direction.normalize();
  for (int i=0; i < numInit; i++)
  	directions.push_back(direction);
  direction = Vector3d(1.0, -1.0, -0.4);
  direction.normalize();
  for (int i=0; i < numInit; i++)
  	directions.push_back(direction);
  direction = Vector3d(-1.0, -1.0, 0.4);
  direction.normalize();
  for (int i=0; i < numInit; i++)
  	directions.push_back(direction);
  direction = Vector3d(0.0, -1.0, 0.0);
  direction.normalize();
  for (int i=0; i < numInit; i++)
  	directions.push_back(direction);
  directions.push_back(-Vector3d::UnitY());
  directions.push_back(-Vector3d::UnitY());
  
  vertices.clear();
  vertices.push_back(Vector3d::Zero());
  for (int i=1; i < 4*numInit + 5; i++) {
  	Vector3d next_vertex;
  	next_vertex(0) = vertices[i-1](0) + directions[i-1](0) * lengths[i-1];
  	next_vertex(1) = vertices[i-1](1) + directions[i-1](1) * lengths[i-1];
  	next_vertex(2) = vertices[i-1](2) + directions[i-1](2) * lengths[i-1];
    vertices.push_back(next_vertex);
  }
  
	last_pos = Vector3d(-10.0, -30.0, 0.0);
	for (int i=0; i<vertices.size(); i++)
		vertices[i] += -vertices.back() + last_pos;
  
  for (int i=0; i<vertices.size(); i++)
		vertices[i](0) *= -1;
	Matrix3d start_rotation1 = (Matrix3d) AngleAxisd(M_PI, Vector3d::UnitZ());
  Matrix3d end_rotation1 = (Matrix3d) AngleAxisd(-M_PI/2.0, Vector3d::UnitZ());
  ThreadConstrained* thread1 = new ThreadConstrained(vertices, angles, lengths, start_rotation1, end_rotation1);
  threads.push_back(thread1);
  
	for (int thread_ind=0; thread_ind < threads.size(); thread_ind++) {
#ifndef ISOTROPIC
		Matrix2d B = Matrix2d::Zero();
		B(0,0) = 10.0;
		B(1,1) = 1.0;
		threads[thread_ind]->set_coeffs_normalized(B, 3.0, 1e-4);
#else
  	threads[thread_ind]->set_coeffs_normalized(1.0, 3.0, 1e-4);
#endif
	}
}

void initRestingThread()
{
  int numInit = 5;

	double first_length = 3.0; //FIRST_REST_LENGTH;
	double second_length = SECOND_REST_LENGTH;
	double middle_length = DEFAULT_REST_LENGTH;

  vector<Vector3d> vertices;
  vector<double> angles;
  vector<double> lengths;
  vector<Vector3d> directions;
  
	for (int i=0; i < 4*numInit + 5; i++)
		angles.push_back(0.0);
  
  lengths.push_back(first_length);
  lengths.push_back(second_length);
  for (int i=0; i < 4*numInit; i++)
  	lengths.push_back(middle_length);
  lengths.push_back(second_length);
  lengths.push_back(first_length);
  lengths.push_back(first_length);
  
  directions.push_back(Vector3d::UnitX());
  directions.push_back(Vector3d::UnitX());
  Vector3d direction = Vector3d(1.0, 1.0, 0.0);
  direction.normalize();
  for (int i=0; i < numInit; i++)
  	directions.push_back(direction);
  direction = Vector3d(1.0, -1.0, 0.4);
  direction.normalize();
  for (int i=0; i < numInit; i++)
  	directions.push_back(direction);
  direction = Vector3d(-1.0, -1.0, -0.4);
  direction.normalize();
  for (int i=0; i < numInit; i++)
  	directions.push_back(direction);
  direction = Vector3d(0.0, -1.0, 0.0);
  direction.normalize();
  for (int i=0; i < numInit; i++)
  	directions.push_back(direction);
  directions.push_back(-Vector3d::UnitY());
  directions.push_back(-Vector3d::UnitY());
  
  vertices.push_back(Vector3d::Zero());
  for (int i=1; i < 4*numInit + 5; i++) {
  	Vector3d next_vertex;
  	next_vertex(0) = vertices[i-1](0) + directions[i-1](0) * lengths[i-1];
  	next_vertex(1) = vertices[i-1](1) + directions[i-1](1) * lengths[i-1];
  	next_vertex(2) = vertices[i-1](2) + directions[i-1](2) * lengths[i-1];
    vertices.push_back(next_vertex);
  }
  
	Vector3d last_pos = Vector3d(-10.0, -30.0, 0.0);
	for (int i=0; i<vertices.size(); i++)
		vertices[i] += -vertices.back() + last_pos;

  Matrix3d start_rotation0 = Matrix3d::Identity();
  Matrix3d end_rotation0 = (Matrix3d) AngleAxisd(-M_PI/2.0, Vector3d::UnitZ());

  ThreadConstrained* thread0 = new ThreadConstrained(vertices, angles, lengths, start_rotation0, end_rotation0);
  threads.push_back(thread0);
  
  int numInit1 = 2;
  
  angles.clear();
  for (int i=0; i < 16*numInit1 + 5; i++)
		angles.push_back(0.0);
  
  lengths.clear();
  lengths.push_back(first_length);
  lengths.push_back(second_length);
  for (int i=0; i < 16*numInit1; i++)
  	lengths.push_back(middle_length);
  lengths.push_back(second_length);
  lengths.push_back(first_length);
  lengths.push_back(first_length);
  
  directions.clear();
  directions.push_back(-Vector3d::UnitX());
  directions.push_back(-Vector3d::UnitX());
  direction = Vector3d(-1.0, 0.0, 1.0);
  direction.normalize();
  for (int i=0; i < 1.5*numInit1; i++)
  	directions.push_back(direction);
  direction = Vector3d(0.0, -1.0, 2.0);
  direction.normalize();
  for (int i=0; i < 1.5*numInit1; i++)
  	directions.push_back(direction);
  direction = Vector3d(-2.0, 1.0, 0.0);
  direction.normalize();
  for (int i=0; i < numInit1; i++)
  	directions.push_back(direction);
  direction = Vector3d(-2.0, -1.0, 0.0);
  direction.normalize();
  for (int i=0; i < numInit1; i++)
  	directions.push_back(direction);
  direction = Vector3d(0.0, 1.0, -2.0);
  direction.normalize();
  for (int i=0; i < 3*numInit1; i++)
  	directions.push_back(direction);
  direction = Vector3d(0.0, -1.0, -2.0);
  direction.normalize();
  for (int i=0; i < 3*numInit1; i++)
  	directions.push_back(direction);
  direction = Vector3d(2.0, 1.0, 0.0);
  direction.normalize();
  for (int i=0; i < numInit1; i++)
  	directions.push_back(direction);
  direction = Vector3d(2.0, -1.0, 0.0);
  direction.normalize();
  for (int i=0; i < numInit1; i++)
  	directions.push_back(direction);
  direction = Vector3d(0.0, 1.0, 2.0);
  direction.normalize();
  for (int i=0; i < 1.5*numInit1; i++)
  	directions.push_back(direction);
  direction = Vector3d(0.0, -1.0, 2.0);
  direction.normalize();
  for (int i=0; i < 1.5*numInit1; i++)
  	directions.push_back(direction);
  directions.push_back(-Vector3d::UnitY());
  directions.push_back(-Vector3d::UnitY());
  
  vertices.clear();
  vertices.push_back(Vector3d::Zero());
  for (int i=1; i < 16*numInit1 + 5; i++) {
  	Vector3d next_vertex;
  	next_vertex(0) = vertices[i-1](0) + directions[i-1](0) * lengths[i-1];
  	next_vertex(1) = vertices[i-1](1) + directions[i-1](1) * lengths[i-1];
  	next_vertex(2) = vertices[i-1](2) + directions[i-1](2) * lengths[i-1];
    vertices.push_back(next_vertex);
  }
  
	last_pos = Vector3d(10.0, -30.0, 0.0);
	for (int i=0; i<vertices.size(); i++)
		vertices[i] += -vertices.back() + last_pos;
  
	Matrix3d start_rotation1 = (Matrix3d) AngleAxisd(M_PI, Vector3d::UnitZ());
  Matrix3d end_rotation1 = (Matrix3d) AngleAxisd(M_PI/2.0, Vector3d::UnitZ());
  ThreadConstrained* thread1 = new ThreadConstrained(vertices, angles, lengths, start_rotation1);
  threads.push_back(thread1);
  
	for (int thread_ind=0; thread_ind < threads.size(); thread_ind++) {
#ifndef ISOTROPIC
		Matrix2d B = Matrix2d::Zero();
		B(0,0) = 10.0;
		B(1,1) = 1.0;
		threads[thread_ind]->set_coeffs_normalized(B, 3.0, 1e-4);
#else
  	threads[thread_ind]->set_coeffs_normalized(1.0, 3.0, 1e-4);
#endif
	}
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
