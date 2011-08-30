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
#include <signal.h>

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

#include "planner_lib.h"

//TODO//#include "StateRecorder.h"
//TODO//#include "StateReader.h"
//TODO//#include "TrajectoryRecorder.h"
//TODO//#include "TrajectoryReader.h"

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

void processInput(ControlBase* control0, ControlBase* control1);
void moveMouseToClosestEE(Mouse* mouse);
void displayTextInScreen(const char* textline, ...);
void glutMenu(int ID);
void initGL();
void interruptHandler(int sig);
void sqpSmoother(vector<World*>& trajectory_to_smooth);
void sqpPlanner();
//#define VIEW3D

float lastx_L=0;
float lasty_L=0;
float lastx_M=0;
float lasty_M=0;

int pressed_mouse_button;
float rotate_frame[2] = { 0.0, 0.0 };
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

// interactive variables
bool limit_displacement = false;
bool haptics = false;

//IO
Haptic *haptic0, *haptic1;
Mouse *mouse0, *mouse1;

//Environment
World *world;

//drawing SQP results
vector<World*> drawWorlds;
int drawInd = 0; 

World* start_world = NULL;
World* goal_world = NULL;
bool drawStartWorld = false;
bool drawGoalWorld = false; 
bool interruptEnabled = false;
bool smoothingEnabled = false; 

//For recording and playing back trajectories
//TODO//TrajectoryRecorder trajectory_recorder;
vector<World*> worlds;
int world_ind = 0;

void processLeft(int x, int y)
{
	if (mouse0->getKeyPressed() != NONE) {
		mouse0->add2DMove(x-lastx_L, lasty_L-y);
	} else if (mouse1->getKeyPressed() != NONE) {
		mouse1->add2DMove(x-lastx_L, lasty_L-y);
	} else if (/*glutGetModifiers()*/ false &  GLUT_ACTIVE_CTRL){

		translate_frame[0] += 0.1*(x-lastx_L);
		translate_frame[1] += 0.1*(lasty_L-y);
	} else {
		rotate_frame[0] += 0.2*(x-lastx_L);
		rotate_frame[1] += -0.2*(lasty_L-y);
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
 	else if (key == 'u') {
		glutIgnoreKeyRepeat(1);
  } else if (key == 'j') {
    if (!world->cursorAtIndex(0)->isAttached())
   		moveMouseToClosestEE(mouse0);
		glutIgnoreKeyRepeat(1);
  } else if (key == 'U') {
		glutIgnoreKeyRepeat(1);
  } else if (key == 'J') {
    if (!world->cursorAtIndex(1)->isAttached())
    	moveMouseToClosestEE(mouse1);
		glutIgnoreKeyRepeat(1);
	/*} else if(key == 's') {
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
  	char *fullPath = new char[256];
  	if (key == 'l') {
  		cout << "Please enter destination file name (without extension): ";
	  	char *dstFileName = new char[256];
    	cin >> dstFileName;
    	sprintf(fullPath, "%s%s", "environmentFiles/", dstFileName);
  	} else {
	    sprintf(fullPath, "%s%s%c", "environmentFiles/", "o", key);
	  }
    //TODO don't forget to unattach control
    StateReader state_reader(fullPath);
    if (state_reader.readObjectsFromFile(world)) {
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
  	//TODO don't forget to unattach control
  	trajectory_reader.readStatesFromFile(worlds);
  	cout << "worlds size " << worlds.size() << endl;
  	world_ind = 0;
  	world = worlds[world_ind];*/
	} else if(key == '[') {
		if (worlds.size() > 0) {
			world_ind = max(0, world_ind-1);
			world = worlds[world_ind];
			cout << "world " << world_ind << " / " << worlds.size() << endl;
		} else { cout << "There is no previous state. worlds is empty." << endl; }
	} else if(key == ']') {
		if (worlds.size() > 0) {
			world_ind = min((int) worlds.size()-1, world_ind+1);
			world = worlds[world_ind];
			cout << "world " << world_ind << " / " << worlds.size() << endl;
		} else { cout << "There is no next state. worlds is empty." << endl; }
	} else if(key == 'l') {
		limit_displacement = !limit_displacement;
  } else if(key == 'e') {
  	//for (int thread_ind=0; thread_ind<threads.size(); thread_ind++)
  	//	threads[thread_ind]->toggleExamineMode();
  } else if(key == 'h') {
  	haptics = !haptics;
  	if (haptics) {
  		haptic0->resetPosition(mouse0->getPosition());
  		haptic1->resetPosition(mouse1->getPosition());
  	} else {
  		mouse0->setTransform(haptic0);
  		mouse1->setTransform(haptic1);
  	}
  } else if(key == 'g') {
    sqpPlanner();
  } else if (key == 'G') {
    cout << "Changing smoothing enabled clears worlds" << endl;
    for (int i = 0; i < worlds.size(); i++) delete worlds[i];
    worlds.clear(); 
    smoothingEnabled = !smoothingEnabled; 
  } else if (key == '<') { 
    drawInd = max(0, drawInd - 1);
  } else if (key == '>') { 
    drawInd = min((int) drawWorlds.size()-1, drawInd + 1);
  } else if (key == ',') {
    drawStartWorld = !drawStartWorld;
  } else if (key == '.') { 
    drawGoalWorld = !drawGoalWorld;
  } else if (key == 'b') { 
    start_world = new World(*world);
  } else if (key == 'n') { 
    goal_world = new World(*world);
  } else if (key == 'c') {
    cout << "Deleting " << worlds.size() << " worlds" << endl;  
    for (int i = 0; i < worlds.size(); i++) delete worlds[i];
    worlds.clear();
  } else if(key == 'd') {
  	cout << "saving to backup" << endl;
  	world->backup();
  } else if(key == 'f') {
  	cout << "restoring from backup" << endl;
  	world->restore();
  }
#ifdef VIEW3D
  else if(key == '=') {
  	eye_separation += 0.5;
  } else if(key == '-') {
  	if (eye_separation > 0.5)
	  	eye_separation -= 0.5;
  } else if(key == '+') {
  	if ((eye_focus_depth + 50.0 + 5.0) < (-translate_frame[2]))
  		eye_focus_depth += 1.0;
  } else if(key == '_') {
	 	eye_focus_depth -= 1.0;
  }
#endif
	else if (key == 'q' || key == 27) {
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
  
  if ((key == 'u') || (key == 'j') || (key == 'U') || (key == 'J')) {
  	glutIgnoreKeyRepeat(0);
  	if (!haptics) {	
			if (key == 'u')
				mouse0->setPressButton(true, UP);
			if (key == 'j')
				mouse0->setPressButton(true, DOWN);
			if (key == 'U')
				mouse1->setPressButton(true, UP);
			if (key == 'J')
				mouse1->setPressButton(true, DOWN);
			processInput(mouse0, mouse1);
			glutPostRedisplay();
		}
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
	glutPostRedisplay ();
}

void processHapticDevice()
{
	Vector3d start_proxy_pos, end_proxy_pos;
	Matrix3d start_proxy_rot, end_proxy_rot;
	bool start_proxybutton[2], end_proxybutton[2];
	
	if (getDeviceState(start_proxy_pos, start_proxy_rot, start_proxybutton, end_proxy_pos, end_proxy_rot, end_proxybutton)) {
		haptic0->setRelativeTransform(start_proxy_pos, start_proxy_rot);
		haptic0->setHapticButton(start_proxybutton[UP], UP);
		haptic0->setHapticButton(start_proxybutton[DOWN], DOWN);
		haptic1->setRelativeTransform(end_proxy_pos, end_proxy_rot);
		haptic1->setHapticButton(end_proxybutton[UP], UP);
		haptic1->setHapticButton(end_proxybutton[DOWN], DOWN);
	}

	if (haptics) {
		processInput(haptic0, haptic1);
		glutPostRedisplay ();
	}
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
  glRotatef (rotate_frame[0], 0.0, 1.0, 0.0);
  glGetDoublev(GL_MODELVIEW_MATRIX, model_view);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);
  drawWorlds[drawInd]->draw();
  world->draw();
  if (start_world && drawStartWorld) start_world->draw();
  if (goal_world && drawGoalWorld) goal_world->draw(); 
  //world->draw();
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
	glutPositionWindow(1680-900, 0);
#endif
	glutDisplayFunc (drawStuff);
	glutMotionFunc (mouseMotion);
  glutMouseFunc (processMouse);
  glutKeyboardFunc(processNormalKeys);
  glutKeyboardUpFunc(processKeyUp);
  glutSpecialFunc(processSpecialKeys);
  //glutIdleFunc(processHapticDevice);
 
	
	/* create popup menu */
	glutCreateMenu (glutMenu);
	glutAddMenuEntry ("Exit", 0);
	glutAttachMenu (GLUT_RIGHT_BUTTON);
	
	initGL();
	
  zero_location = Vector3d::Zero();
  zero_angle = 0.0;

	//IO
	mouse0 = new Mouse();
	mouse1 = new Mouse();
	haptic0 = new Haptic();
	haptic1 = new Haptic();
	connectionInit();
	
	//Environment
	world = new World();

	glGetDoublev(GL_MODELVIEW_MATRIX, model_view);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);
	
  signal(SIGINT, &interruptHandler);

	if (haptics)
		processInput(haptic0, haptic1);
	else
		processInput(mouse0, mouse1);

  drawWorlds.push_back(world);
  worlds.push_back(new World(*world));
  glutMainLoop ();
}

void processInput(ControlBase* control0, ControlBase* control1)
{	
	vector<ControlBase*> controls;
	controls.push_back(control0);
	controls.push_back(control1);

	world->setTransform(controls);
  worlds.push_back(new World(*world));

  if (worlds.size() > 10 && smoothingEnabled) { 
    sqpSmoother(worlds);
    worlds.clear();
    worlds.push_back(drawWorlds.back());
  }
}

void moveMouseToClosestEE(Mouse* mouse) {
	const Vector3d tip_pos = mouse->getPosition() - EndEffector::grab_offset * mouse->getRotation().col(0);
	EndEffector* ee = world->closestEndEffector(tip_pos);
	mouse->setTransform(ee->getPosition() + EndEffector::grab_offset * ee->getRotation().col(0), ee->getRotation());
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

void interruptHandler(int sig) { 
  cout << "Signal " << sig << "caught..." << endl; 
  interruptEnabled = true; 
}

void sqpPlanner() { 

    int num_worlds = 5;
    string namestring = "world_sqp_debug";
    VectorXd du(12);
    double norm = 1e-1;
    if (!start_world || !goal_world) return;  

    World* initial_world = new World(*start_world);
    vector<World*> completeOpenLoopTrajectory;
    cout << "Planning over " << num_worlds << " worlds" << endl; 

    while (cost_metric(initial_world, goal_world) > 2 && !interruptEnabled) {
      // Generate initial trajectory
      vector<World*> initialization_worlds;
      
      initialization_worlds.push_back(new World(*initial_world)); 
      for (int i = 0; i < num_worlds; i++) {
        if (i % 1 == 0) {
          sample_on_sphere(du, norm); 
        }
        initial_world->applyRelativeControlJacobian(du);
        initialization_worlds.push_back(new World(*initial_world));
      }

      initialization_worlds.push_back(new World(*goal_world));

      vector<World*> sqpWorlds;
      vector<VectorXd> sqpControls;

      solveSQP(initialization_worlds, sqpWorlds, sqpControls, namestring.c_str());

      vector<World*> openLoopWorlds;
      openLoopController(initialization_worlds, sqpControls, openLoopWorlds);

      for (int i = 0; i < openLoopWorlds.size(); i++) {
        completeOpenLoopTrajectory.push_back(new World(*openLoopWorlds[i]));
      }
      initial_world = new World(*openLoopWorlds.back());
    }

    sqpSmoother(completeOpenLoopTrajectory);
    //drawWorlds.clear();
    //drawWorlds = worlds;
    //drawWorlds = openLoopWorlds;
    drawWorlds = completeOpenLoopTrajectory;

}

void sqpSmoother(vector<World*>& trajectory_to_smooth) {

  getTrajectoryStatistics(trajectory_to_smooth);

  if (trajectory_to_smooth.size() == 0) return;
  cout << "Smoothing over " << trajectory_to_smooth.size() << " worlds" << endl; 

  vector<World*> sqpWorlds;
  vector<VectorXd> sqpControls;

  string namestring = "world_sqp_debug";
  solveSQP(trajectory_to_smooth, sqpWorlds, sqpControls, namestring.c_str());

  vector<World*> openLoopWorlds;
  openLoopController(trajectory_to_smooth, sqpControls, openLoopWorlds);

  drawWorlds = openLoopWorlds;
}
