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
#include "thread_discrete.h"
#include "thread_socket_interface.h"
#include "ThreadConstrained.h"
#include "EnvObjects/World.h"
#include "EnvObjects/EnvObject.h"
#include "EnvObjects/Capsule.h"
#include "EnvObjects/Cursor.h"
#include "EnvObjects/EndEffector.h"
#include "EnvObjects/InfinitePlane.h"
#include "EnvObjects/TexturedSphere.h"

#include "ObjectTrajectoryRecorder.h"
#include "ObjectTrajectoryReader.h"

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

void drawStuff();
void processInput();
void updateState(const Vector3d& proxy_pos, const Matrix3d& proxy_rot, Cursor* cursor);
bool closeEnough(Vector3d my_pos, Matrix3d my_rot, Vector3d pos, Matrix3d rot);
void mouseTransform(Vector3d &new_pos, Matrix3d &new_rot, vector<Vector3d> &positions, vector<Matrix3d> &rotations, int cvnum);
void displayTextInScreen(const char* textline, ...);
void initThread();
void initLongerThread();
void glutMenu(int ID);
void initGL();

#define NUM_PTS 500
#define THREAD_RADII 1.0
#define MOVE_POS_CONST 1.0
#define MOVE_TAN_CONST 0.2
#define ROTATE_TAN_CONST 0.2
#define HAPTICS true
//#define VIEW3D

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
float eye_focus_depth = 0.0; // distance from sphere center to focus point
#endif

float move[2];
float tangent[2];
float tangent_rotation[2];

vector<Vector3d> points;
vector<double> twist_angles;

Vector3d zero_location;
double zero_angle;

vector<ThreadConstrained*> threads;

int pressed_mouse_button;

key_code key_pressed;

Vector3d start_proxy_pos(-40.0, -30.0, 0.0), end_proxy_pos(-40.0, -30.0, 0.0), start_tip_pos, end_tip_pos;
Matrix3d start_proxy_rot = Matrix3d::Identity(), end_proxy_rot = Matrix3d::Identity();
bool start_proxybutton[2] = {false, false}, end_proxybutton[2] = {false, false};
bool last_start_proxybutton[2] = {false, false}, last_end_proxybutton[2] = {false, false};
double start_feedback_pos[3], end_feedback_pos[3];
bool start_feedback_enabled = false, end_feedback_enabled = false;
bool change_constraint = false;
bool choose_mode = false;
int toggle = 1, selected_vertex_num = 0;
double grab_offset = EndEffector::grab_offset;
Vector3d plane_origin = Vector3d(0.0, -30.0, 0.0);

//Objects in environment
World *world;
Cursor *cursor0, *cursor1;
vector<EndEffector*> end_effectors;
InfinitePlane *plane;
TexturedSphere *textured_sphere;

int window_width, window_height;

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
	} else if(key == 's') {
    cout << "Saving...\n";
    cout << "Please enter destination file name (without extension): ";
    char *dstFileName = new char[256];
    cin >> dstFileName;
    char *fullPath = new char[256];
    sprintf(fullPath, "%s%s", "environmentFiles/", dstFileName);
		ObjectTrajectoryRecorder traj_recorder(fullPath);
    traj_recorder.writeObjectsToFile(world);
  } else if((key == 'l') ||
						(key == '1') || (key == '2') || (key == '3') || (key == '4') || (key == '5') || 
  					(key == '6') || (key == '7') || (key == '8') || (key == '9') || (key == '0')) {
  	cout << "Loading...\n";
  	ObjectTrajectoryReader traj_reader;
  	char *fullPath = new char[256];
  	if (key == '1')
	    sprintf(fullPath, "%s%s", "environmentFiles/", "o1");
	  else if (key == '2')
	  	sprintf(fullPath, "%s%s", "environmentFiles/", "o2");
	  else if (key == '3')
	  	sprintf(fullPath, "%s%s", "environmentFiles/", "o3");
	  else if (key == '4')
	  	sprintf(fullPath, "%s%s", "environmentFiles/", "o4");
	  else if (key == '5')
	  	sprintf(fullPath, "%s%s", "environmentFiles/", "o5");
	  else if (key == '6')
	  	sprintf(fullPath, "%s%s", "environmentFiles/", "o6");
	  else if (key == '7')
	  	sprintf(fullPath, "%s%s", "environmentFiles/", "o7");
	  else if (key == '8')
	  	sprintf(fullPath, "%s%s", "environmentFiles/", "o8");
	  else if (key == '9')
	  	sprintf(fullPath, "%s%s", "environmentFiles/", "o9");
	  else if (key == '0')
	  	sprintf(fullPath, "%s%s", "environmentFiles/", "o0");
	  else {
	  	cout << "Please enter destination file name (without extension): ";
	  	char *dstFileName = new char[256];
    	cin >> dstFileName;
    	sprintf(fullPath, "%s%s", "environmentFiles/", dstFileName);
	  }
    traj_reader.setFileName(fullPath);
    if (traj_reader.readObjectsFromFile(world) == 0) {

			vector<ThreadConstrained*> world_thread = *(world->getThreads());
			for (int i = 0; i < threads.size(); i++) {
				threads[i] = world_thread[i];
			}

			vector<EnvObject*> world_cursors = world->getEnvObjs(CURSOR);
			cursor0 = (Cursor*) world_cursors[0];
			cursor1 = (Cursor*) world_cursors[1];

			vector<EnvObject*> world_end_effs = world->getEnvObjs(END_EFFECTOR);
			for (int i = 0; i < end_effectors.size(); i++) {
				end_effectors[i] = (EndEffector*) world_end_effs[i];
			}

			vector<EnvObject*> world_planes = world->getEnvObjs(INFINITE_PLANE);
			plane = (InfinitePlane*) world_planes[0];

			vector<EnvObject*> world_spheres = world->getEnvObjs(TEXTURED_SPHERE);
			textured_sphere = (TexturedSphere*) world_spheres[0];
			
			cout << "Thread loading was sucessful." << endl;
			glutPostRedisplay();  
		}
  } else if(key == 'e') {
  	for (int thread_ind=0; thread_ind<threads.size(); thread_ind++)
  		threads[thread_ind]->toggleExamineMode();
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
  } else if(key == '+') {
  	if ((eye_focus_depth + 50.0 + 5.0) < (-translate_frame[2]))
  		eye_focus_depth += 1.0;
  	//glutPostRedisplay ();
  } else if(key == '_') {
	 	eye_focus_depth -= 1.0;
  	//glutPostRedisplay ();
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
	glutMotionFunc (MouseMotion);
  glutMouseFunc (processMouse);
  glutKeyboardFunc(processNormalKeys);
  glutKeyboardUpFunc(processKeyUp);

 	glutTimerFunc(100, processHapticDevice, 0);
	
	/* create popup menu */
	glutCreateMenu (glutMenu);
	glutAddMenuEntry ("Exit", 0);
	glutAttachMenu (GLUT_RIGHT_BUTTON);
	
	initGL();
	
  //initThread();
  initLongerThread();

  zero_location = Vector3d::Zero(); //points[0];
  zero_angle = 0.0;

	//setting up objects in environment
	cursor0 = new Cursor(Vector3d::Zero(), Matrix3d::Identity());
	cursor1 = new Cursor(Vector3d::Zero(), Matrix3d::Identity());	
	plane = new InfinitePlane(plane_origin, Vector3d(0.0, 1.0, 0.0), 0.8, 0.8, 0.8);
	textured_sphere = new TexturedSphere(Vector3d::Zero(), 150.0, "../utils/wmap.bmp");
	
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
	
	EndEffector* end_effector4 = new EndEffector(plane_origin + Vector3d(30.0, EndEffector::short_handle_r, 0.0), (Matrix3d) AngleAxisd(-M_PI/2.0, Vector3d::UnitY()) * AngleAxisd(M_PI/2.0, Vector3d::UnitX()));
	end_effectors.push_back(end_effector4);
	
	EndEffector* end_effector5 = new EndEffector(plane_origin + Vector3d(35.0, EndEffector::short_handle_r, 0.0), (Matrix3d) AngleAxisd(-M_PI/2.0, Vector3d::UnitY()) * AngleAxisd(M_PI/2.0, Vector3d::UnitX()));
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
	
	threads[0]->initializeThreadsInEnvironment();

	connectionInit();
	
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
#ifdef VIEW3D
	glTranslatef(0.0, 0.0, +eye_focus_depth);
  drawSphere(Vector3d::Zero(), 1, 0.8, 0.8, 0.8);
	//glRotatef (15.0, 0.0, 1.0, 0.0);
	// (-translate_frame[2]) is distance from camera to sphere center
	// (-translate_frame[2] - eye_focus_depth) is distance from camera to focus point
	glRotatef (+atan(eye_separation/(2.0*(-translate_frame[2]-eye_focus_depth))) * 180.0/M_PI, 0.0, 1.0, 0.0);
	glTranslatef(0.0, 0.0, -eye_focus_depth);
#endif
  glRotatef (rotate_frame[1], 1.0, 0.0, 0.0);
  glRotatef (rotate_frame[0], 0.0, 0.0, 1.0);
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
	//glRotatef (-15.0, 0.0, 1.0, 0.0);
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

void processInput()
{
	updateState(start_proxy_pos, start_proxy_rot, cursor0);
	updateState(end_proxy_pos, end_proxy_rot, cursor1);
		
	for (int thread_ind = 0; thread_ind < threads.size(); thread_ind++) {
		vector<Vector3d> positionConstraints;
		vector<Matrix3d> rotationConstraints;
		threads[thread_ind]->getConstrainedTransforms(positionConstraints, rotationConstraints);

		if (cursor0->isAttached() && cursor0->end_eff->getThread()==threads[thread_ind]) {
			positionConstraints[cursor0->end_eff->constraint_ind] = start_proxy_pos - grab_offset * start_proxy_rot.col(0);
			rotationConstraints[cursor0->end_eff->constraint_ind] = start_proxy_rot;
		}
		if (cursor1->isAttached() && cursor1->end_eff->getThread()==threads[thread_ind]) {
			positionConstraints[cursor1->end_eff->constraint_ind] = end_proxy_pos - grab_offset * end_proxy_rot.col(0);
			rotationConstraints[cursor1->end_eff->constraint_ind] = end_proxy_rot;
		}

		threads[thread_ind]->updateConstraints(positionConstraints, rotationConstraints);
	
		threads[thread_ind]->getConstrainedTransforms(positionConstraints, rotationConstraints);

		for (int ee_ind = 0; ee_ind < end_effectors.size(); ee_ind++) {
			EndEffector* ee = end_effectors[ee_ind];
			if (ee->getThread()==threads[thread_ind])
				ee->setTransform(positionConstraints[ee->constraint_ind], rotationConstraints[ee->constraint_ind]);
		}
	
		//threads[thread_ind]->adapt_links();
	}

	glPopMatrix();
}

void updateState(const Vector3d& proxy_pos, const Matrix3d& proxy_rot, Cursor* cursor) {
	Vector3d tip_pos = proxy_pos - grab_offset * proxy_rot.col(0);
	
	cursor->setTransform(proxy_pos, proxy_rot);
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
		    thread->setConstrainedTransforms(ee->constraint_ind, tip_pos, proxy_rot);										// the end effector's orientation matters when it grips the thread. This updates the offset rotation.
		  } else {																																											// cursor has an end effector which remains without holding the thread
	  		ee->setTransform(tip_pos, proxy_rot);
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
		if (cursor->attach_dettach_attempt) {
			cursor->dettach();
			for (int ee_ind = 0; ee_ind < end_effectors.size(); ee_ind++)
				end_effectors[ee_ind]->updateConstraintIndex();
		}
	} else {																				// cursor doesn't have an end effector, THUS it isn't holding the thread i.e. ee->constraint = ee->constraint_ind = -1;
  	for (int ee_ind = 0; ee_ind < end_effectors.size(); ee_ind++) {
  		EndEffector* ee = end_effectors[ee_ind];
  		if (cursor->attach_dettach_attempt && closeEnough(tip_pos, proxy_rot, ee->getPosition(), ee->getRotation())) {
  			cursor->attach(ee);
  			if (ee->isThreadAttached()) {
					ee->updateConstraint();
  				if ((ee->constraint==0 || ee->constraint==(ee->getThread()->numVertices()-1)) && cursor->isOpen()) 
	  				cursor->forceClose();
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
