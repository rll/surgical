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

#include "IO/ControllerBase.h"
#include "IO/Mouse.h"
#include "IO/Haptic.h"
#include "IO/Control.h"
#include "thread_socket_interface.h"

#include "thread_discrete.h"
#include "ThreadConstrained.h"
#include "EnvObjects/World.h"
#include "EnvObjects/Cursor.h"

#include "planner_lib.h"

#include "StateRecorder.h"
#include "StateReader.h"
#include "TrajectoryRecorder.h"
#include "TrajectoryReader.h"

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

void processInput(ControllerBase* controller0, ControllerBase* controller1);
void moveMouseToClosestEE(Mouse* mouse);
void displayTextInScreen(const char* textline, ...);
void bitmap_output(int x, int y, char *string, void *font);
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
bool limit_displacement = true;
bool haptics = true;
bool examine_mode = false;

//IO
Haptic *haptic0, *haptic1;
Mouse *mouse0, *mouse1;
Control *control0, *control1;

//Environment
World *world;

//drawing SQP results
vector<World*> drawWorlds;
int drawInd = 0; 

World* start_world = NULL;
World* goal_world = NULL;
bool drawStartWorld = false;
bool drawGoalWorld = false; 
bool drawVisualizationData = false;
bool drawInteractiveWorld = true; 

bool interruptEnabled = false;
bool smoothingEnabled = false;
Timer* interruptTimer; 

//For recording and playing back trajectories
TrajectoryRecorder trajectory_recorder;
TrajectoryRecorder trajectory_recorder_world;
vector<World*> worlds;
int world_ind = 0;

vector<string> samples_problems;
vector<string> samples_initial_conditions1;
vector<string> samples_problems1;

bool absoluteControl = false;

void setVisualizationData(vector<World*>& visualize_worlds)
{
	drawWorlds = visualize_worlds;
	drawInd = 0;
}

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
    if (!world->objectAtIndex<Cursor>(0)->isAttached())
   		moveMouseToClosestEE(mouse0);
		glutIgnoreKeyRepeat(1);
  } else if (key == 'U') {
		glutIgnoreKeyRepeat(1);
  } else if (key == 'J') {
    if (!world->objectAtIndex<Cursor>(1)->isAttached())
    	moveMouseToClosestEE(mouse1);
		glutIgnoreKeyRepeat(1);
	} else if(key == 's') {
    cout << "Saving...\n";
    cout << "Please enter destination file name (without extension): ";
    char *dstFileName = new char[256];
    cin >> dstFileName;
    char *fullPath = new char[256];
    sprintf(fullPath, "%s%s", "environmentFiles/", dstFileName);
		StateRecorder state_recorder(fullPath);
    state_recorder.writeWorldToFile(world);
  } else if((key == 'a') || (key >= '0' && key <= '9')) {
  	cout << "Loading...\n";
  	char *fullPath = new char[256];
  	if (key == 'a') {
  		cout << "Please enter destination file name (without extension): ";
	  	char *dstFileName = new char[256];
    	cin >> dstFileName;
    	sprintf(fullPath, "%s%s", "environmentFiles/", dstFileName);
  	} else {
	    sprintf(fullPath, "%s%s%c", "environmentFiles/", "s", key);
	  }
    StateReader state_reader(fullPath);
    if (state_reader.readWorldFromFile(world)) {
			cout << "State loading was sucessful." << endl;
			World* temp_world = new World(*world);
			vector<World*> vis_data;
			vis_data.push_back(temp_world);
			setVisualizationData(vis_data); 
		} else {
			//TODO safely handle this case
			cout << "State loading was unsucessful. Finishing program because this case is not safely handled (in terms of pointers)." << endl;
			assert(0);
		}
	} else if(key == '?') {
		TrajectoryReader traj(string(samples_problems1.back()).c_str());
		samples_problems1.pop_back();
		vector<World*> traj_out;
		if (traj.readWorldsFromFile(traj_out)) {
			cout << "Trajectory loading was sucessful. " << traj_out.size() << " worlds were loaded." << endl;
			setVisualizationData(traj_out);
		}
	} else if(key == 'c') { // records control trajectory and world trajectory
		if (trajectory_recorder.hasStarted()) {
			cout << "Trajectory was already started. Stop it first before starting to save a new trajectory." << endl;
		} else {
			cout << "Starting trajectory and saving...\n";
		  cout << "Please enter destination file name (without extension): ";
		  char *dstFileName = new char[256];
		  cin >> dstFileName;
		  char *fullPath = new char[256];
		  sprintf(fullPath, "%s%s", "environmentFiles/", dstFileName);
			trajectory_recorder.setFileName(fullPath);
			trajectory_recorder.start();
			char *fullPath_world  = new char[256];
			sprintf(fullPath_world, "%s%s%s", "environmentFiles/", dstFileName, "_world");
			trajectory_recorder_world.setFileName(fullPath_world);
			trajectory_recorder_world.start();
			start_world = new World(*world); 
		}
	} else if(key == 'x') { // stops recording control trajectory and world trajectory
		if (!trajectory_recorder.hasStarted()) {
			cout << "Trajectory have not been started. Start it first before stopping it." << endl;
		} else {
			cout << "Finished saving trajectory.\n";
			trajectory_recorder.stop();
			trajectory_recorder_world.stop();
		}
	} else if(key == 'k') { // loads a world trajectory and puts it in the visualize drawing
		cout << "Loading trajectory from Worlds file...\n";
		cout << "Please enter destination file name (without extension): ";
  	char *fullPath = new char[256];
		char *dstFileName = new char[256];
  	cin >> dstFileName;
  	sprintf(fullPath, "%s%s", "environmentFiles/", dstFileName);
		TrajectoryReader trajectory_reader(fullPath);
		vector<World*> vis_data;
		if (trajectory_reader.readWorldsFromFile(vis_data)) {
			cout << "Trajectory loading was sucessful. " << vis_data.size() << " worlds were loaded." << endl;
			setVisualizationData(vis_data);
		} else {
			cout << "Failed to load trajectory. Specified file might not exist." << endl;
		}
	} else if(key == '=') { // loads the initial conditions and puts it in the visualize drawing
		char *fullPath = new char[256];
		cout << "Please enter control file (i.e. c8): ";
  	char *dstFileName = new char[256];
  	cin >> dstFileName;
  	sprintf(fullPath, "%s%s", "environmentFiles/", dstFileName);
  	
  	vector<vector<double> > dof_perts;
  	vector<double> scale;
  	scale.push_back(-2.0);
  	scale.push_back(-4.0);
  	scale.push_back(2.0);
  	scale.push_back(4.0);
  	
  	for (int scale_ind = 0; scale_ind < scale.size(); scale_ind++) {
			for (int dof = 0; dof < 12; dof++) {
				vector<double > dof_pert(12,0);
				dof_pert[dof] = scale[scale_ind];
				dof_perts.push_back(dof_pert);
			}
		}
		
		
		vector<World*> vis_data;
		for (int p = 0; p < dof_perts.size(); p++)
		{
			char *ic_pert_path = new char[256];
  		sprintf(ic_pert_path, "%s%s%s%.0f%.0f%.0f%.0f%.0f%.0f%.0f%.0f%.0f%.0f%.0f%.0f", "environmentFiles/ic/ic_", dstFileName, "_dof_",
  													dof_perts[p][0], dof_perts[p][1], dof_perts[p][2], dof_perts[p][3], dof_perts[p][4], dof_perts[p][5],
  												  dof_perts[p][6], dof_perts[p][7], dof_perts[p][8], dof_perts[p][9], dof_perts[p][10], dof_perts[p][11]);
			//load initial condition from a worlds trajectory
			StateReader state_reader(ic_pert_path);
		  World* temp_world = new World();
		  if (state_reader.readWorldFromFile(temp_world)) {
		  	vis_data.push_back(temp_world);
		  	cout << "State loading was sucessful." << endl;
			} else {
				cout << "State loading was unsucessful." << endl;
				assert(0);
			}
		}		
		setVisualizationData(vis_data);

	} else if(key == '-') { // loads the last states of an icc_traj and puts it in the visualize drawing
		char *fullPath = new char[256];
		cout << "Please enter control file (i.e. c8): ";
  	char *dstFileName = new char[256];
  	cin >> dstFileName;
  	sprintf(fullPath, "%s%s", "environmentFiles/", dstFileName);
  	
  	vector<vector<double> > dof_perts;
  	vector<double> scale;
  	scale.push_back(-2.0);
  	scale.push_back(-4.0);
  	scale.push_back(2.0);
  	scale.push_back(4.0);
  	
  	for (int scale_ind = 0; scale_ind < scale.size(); scale_ind++) {
			for (int dof = 0; dof < 12; dof++) {
				vector<double > dof_pert(12,0);
				dof_pert[dof] = scale[scale_ind];
				dof_perts.push_back(dof_pert);
			}
		}
		
		
		vector<World*> vis_data;
		for (int p = 0; p < dof_perts.size(); p++)
		{
			char *icc_traj_pert_path = new char[256];
  		sprintf(icc_traj_pert_path, "%s%s%s%.0f%.0f%.0f%.0f%.0f%.0f%.0f%.0f%.0f%.0f%.0f%.0f", "environmentFiles/icc_traj/traj_", dstFileName, "_dof_",
  													dof_perts[p][0], dof_perts[p][1], dof_perts[p][2], dof_perts[p][3], dof_perts[p][4], dof_perts[p][5],
  												  dof_perts[p][6], dof_perts[p][7], dof_perts[p][8], dof_perts[p][9], dof_perts[p][10], dof_perts[p][11]);
			//load last state's many a worlds trajectory
			vector<World*> temp_worlds;
			TrajectoryReader read(icc_traj_pert_path);
			if (read.readWorldsFromFile(temp_worlds)) {
				vis_data.push_back(temp_worlds.back());
				cout << "Trajectory loading was sucessful. Putting last state into the trajectory." << endl;
			} else {
				assert(0);
			}
		}	
		setVisualizationData(vis_data);
		
		TrajectoryRecorder rec("c8_last_states");
		rec.start();
		for (int i=0; i<vis_data.size(); i++) {
			rec.writeWorldToFile(vis_data[i]);
		}
		rec.stop();

	} else if(key == '+') { // loads a world and puts it in the start drawing
		vector<World*> temp_worlds;
		TrajectoryReader read("environmentFiles/c2_world");
		if (read.readWorldsFromFile(temp_worlds)) {
			cout << "Trajectory loading was sucessful. Using first world as initial condition." << endl;
			start_world = temp_worlds[0];
		} else {
			assert(0);
		}
	} else if (key == 'z') { // generate a world trajectory from an initial condition and control trajectory
													 // puts the generated world trajectory in the visualiaze drawing
		
		cout << "Loading trajectory from Controls file...\n";
  	char *fullPath = new char[256];
  	char *fullPath_world = new char[256];
		cout << "Please enter control file (i.e. c8): ";
  	char *dstFileName = new char[256];
  	cin >> dstFileName;
  	sprintf(fullPath, "%s%s", "environmentFiles/", dstFileName);
  	sprintf(fullPath_world, "%s%s%s", "environmentFiles/", dstFileName, "_world");
  	
  	vector<vector<double> > dof_perts;
  	vector<double> scale;
  	scale.push_back(-2.0);
  	scale.push_back(-4.0);
  	scale.push_back(2.0);
  	scale.push_back(4.0);
  	
  	for (int scale_ind = 0; scale_ind < scale.size(); scale_ind++) {
			for (int dof = 0; dof < 12; dof++) {
				vector<double > dof_pert(12,0);
				dof_pert[dof] = scale[scale_ind];
				dof_perts.push_back(dof_pert);
			}
		}
  	
		//for (int p = 0; p < dof_perts.size(); p++)
		for (int p = 0; p < 1; p++)
		{
			//load initial condition from a worlds trajectory
			vector<World*> temp_worlds;
			TrajectoryReader read(fullPath_world); //TODO input world trajectory to extract initial condition
			if (read.readWorldsFromFile(temp_worlds)) {
				cout << "Trajectory loading was sucessful. Using first world as initial condition." << endl;
			} else {
				assert(0);
			}
			
			char *ic_pert_path = new char[256];
  		sprintf(ic_pert_path, "%s%s%s%.0f%.0f%.0f%.0f%.0f%.0f%.0f%.0f%.0f%.0f%.0f%.0f", "environmentFiles/ic/ic_", dstFileName, "_dof_",
  													dof_perts[p][0], dof_perts[p][1], dof_perts[p][2], dof_perts[p][3], dof_perts[p][4], dof_perts[p][5],
  												  dof_perts[p][6], dof_perts[p][7], dof_perts[p][8], dof_perts[p][9], dof_perts[p][10], dof_perts[p][11]);
  		char *icc_traj_pert_path = new char[256];
  		sprintf(icc_traj_pert_path, "%s%s%s%.0f%.0f%.0f%.0f%.0f%.0f%.0f%.0f%.0f%.0f%.0f%.0f", "environmentFiles/icc_traj/traj_", dstFileName, "_dof_",
  													dof_perts[p][0], dof_perts[p][1], dof_perts[p][2], dof_perts[p][3], dof_perts[p][4], dof_perts[p][5],
  												  dof_perts[p][6], dof_perts[p][7], dof_perts[p][8], dof_perts[p][9], dof_perts[p][10], dof_perts[p][11]);
			
			//perturb initial condition
			Control ctrl0(Vector3d::Zero(), Matrix3d::Identity());
			ctrl0.setTranslate(Vector3d(dof_perts[p][0],dof_perts[p][1],dof_perts[p][2]));
			ctrl0.setRotate((Matrix3d) (AngleAxisd(dof_perts[p][3]*2.0*M_PI/180.0, (temp_worlds[0]->objectAtIndex<Cursor>(0)->getRotation()).col(0))
											* AngleAxisd(dof_perts[p][4]*2.0*M_PI/180.0, (temp_worlds[0]->objectAtIndex<Cursor>(0)->getRotation()).col(1))
											* AngleAxisd(dof_perts[p][5]*2.0*M_PI/180.0, (temp_worlds[0]->objectAtIndex<Cursor>(0)->getRotation()).col(2))));
			
			Control ctrl1(Vector3d::Zero(), Matrix3d::Identity());
			ctrl1.setTranslate(Vector3d(dof_perts[p][6],dof_perts[p][7],dof_perts[p][8]));
			ctrl1.setRotate((Matrix3d) (AngleAxisd(dof_perts[p][9]*2.0*M_PI/180.0, (temp_worlds[0]->objectAtIndex<Cursor>(1)->getRotation()).col(0))
											* AngleAxisd(dof_perts[p][10]*2.0*M_PI/180.0, (temp_worlds[0]->objectAtIndex<Cursor>(1)->getRotation()).col(1))
											* AngleAxisd(dof_perts[p][11]*2.0*M_PI/180.0, (temp_worlds[0]->objectAtIndex<Cursor>(1)->getRotation()).col(2))));
						
			vector<Control*> ctrls;
			ctrls.push_back(&ctrl0);
			ctrls.push_back(&ctrl1);
			//temp_worlds[0]->applyRelativeControl(ctrls, false);
			
			cout << "Saving initial condition state in " << ic_pert_path << endl;
			StateRecorder ic_state_recorder(ic_pert_path);
		  ic_state_recorder.writeWorldToFile(temp_worlds[0]);
			
			

//			//load initial condition from a world state
//			World* initial_world = new World();
//			//char *icpath = new char[256];
//			//sprintf(icpath, "%s%s", "environmentFiles/", samples_initial_conditions1[p].c_str());
//			StateReader read(samples_initial_conditions1[p].c_str()); //TODO input initial condition
//			if (read.readWorldFromFile(initial_world)) {
//				cout << "State loading was sucessful. Using this state as initial condition." << endl;
//				start_world = initial_world;
//			} else {
//				assert(0);
//			}



			vector<vector<Control*> > controls;
			TrajectoryReader control_traj_reader(fullPath); //TODO input control trajectory
			if (control_traj_reader.readControlsFromFile(controls)) {
				cout << "Trajectory loading was sucessful. " << controls.size() << " controls were loaded." << endl;
				vector<World*> traj_in;
				traj_in.push_back(new World(*temp_worlds[0]));
				vector<World*> traj_out; 			
				openLoopController(temp_worlds[0], temp_worlds, controls, traj_out);
				setVisualizationData(traj_out);
				break;
				
//				VectorXd u(12);
//				u.setZero();
//				for(int i =0; i<temp_worlds.size(); i++) {
//					temp_worlds[i]->applyRelativeControlJacobian(u);
//				}
//				setVisualizationData(temp_worlds);
				
				//TrajectoryRecorder rec(icc_traj_pert_path); //TODO output world trajectory file generated from control trajectory
				TrajectoryRecorder rec("c2_test");
				rec.start();
				for (int i=0; i<traj_out.size(); i++) {
					rec.writeWorldToFile(traj_out[i]);
				}
				rec.stop();
			} else {
				cout << "Failed to load trajectory. Specified file might not exist." << endl;
				assert(0);
			}
			
		}
	} else if(key == 'l') {
		limit_displacement = !limit_displacement;
  } else if(key == 'h') {
  	haptics = !haptics;
  	/*if (haptics) {
  		haptic0->resetPosition(mouse0->getPosition());
  		haptic1->resetPosition(mouse1->getPosition());
  	} else {
  		mouse0->setTransform(haptic0);
  		mouse1->setTransform(haptic1);
  	}*/
  } else if(key == 'e') {
  	examine_mode = !examine_mode;
  } else if(key == 'w') {
  	rotate_frame[0] = rotate_frame[1] = 0.0;
		translate_frame[0] = translate_frame[1] = 0.0;
		translate_frame[2] = -110.0;
  } else if(key == 'g') {
    sqpPlanner();
  } else if (key == 'G') {
    cout << "Changing smoothing enabled clears worlds" << endl;
    for (int i = 0; i < worlds.size(); i++) delete worlds[i];
    worlds.clear(); 
    //smoothingEnabled = !smoothingEnabled;
  } else if (key == 'v') { 
    getTrajectoryStatistics(worlds);
  } else if (key == '<') { 
    drawInd = max(0, drawInd - 1);
    cout << drawInd << endl; 
  } else if (key == '>') { 
    drawInd = min((int) drawWorlds.size()-1, drawInd + 1);
    cout << drawInd << endl; 
  } else if (key == ',') {
    drawStartWorld = !drawStartWorld;
  } else if (key == '.') { 
    drawGoalWorld = !drawGoalWorld;
  } else if (key == ';') { 
  	drawInteractiveWorld = !drawInteractiveWorld;
  }	else if (key == '\'') {
  	drawVisualizationData = !drawVisualizationData;
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

void checkMouseUpdate()
{
	if (!haptics) {
		processInput(mouse0, mouse1);
		glutPostRedisplay();
	}
}

void processIdle()
{
	processHapticDevice();
	//checkMouseUpdate();
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
	if (drawInd < drawWorlds.size() && drawWorlds[drawInd] && drawVisualizationData) { 
    bitmap_output(50, 55, "Viz (')", GLUT_BITMAP_TIMES_ROMAN_24);
  }
  if (world && drawInteractiveWorld) {
    bitmap_output(50, 50, "Int (;)", GLUT_BITMAP_TIMES_ROMAN_24);
  }
  if (start_world && drawStartWorld) {
  	bitmap_output(50, 45, "Start (,)", GLUT_BITMAP_TIMES_ROMAN_24);
  }
  if (goal_world && drawGoalWorld) {
  	bitmap_output(50, 40, "Goal (.)", GLUT_BITMAP_TIMES_ROMAN_24);
  } 

#ifdef VIEW3D
	glTranslatef(0.0, 0.0, +eye_focus_depth);
	glColor3f(0.8, 0.8, 0.8);
  drawSphere(Vector3d::Zero(), 1);
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
  if (drawInd < drawWorlds.size() && drawWorlds[drawInd] && drawVisualizationData) { 
  	drawWorlds[drawInd]->draw(examine_mode);
  }
  if (world && drawInteractiveWorld) {
   	world->draw(examine_mode);
   	world->drawDebug();
  }
  if (start_world && drawStartWorld) {
  	start_world->draw();
  }
  if (goal_world && drawGoalWorld) {
  	goal_world->draw();
  } 
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
	glColor3f(0.8, 0.8, 0.8);
  drawSphere(Vector3d::Zero(), 1);
	glRotatef (-atan(eye_separation/(-2.0*translate_frame[2]-eye_focus_depth)) * 180.0/M_PI, 0.0, 1.0, 0.0);
	glTranslatef(0.0, 0.0, -eye_focus_depth);
  glRotatef (rotate_frame[1], 1.0, 0.0, 0.0);
  glRotatef (rotate_frame[0], 0.0, 1.0, 0.0);
	world->draw(examine_mode);
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
  glutIdleFunc(processIdle);
 
	
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
	
	//control0 = new Control(Vector3d::Zero(), Matrix3d::Identity());
	//control1 = new Control(Vector3d::Zero(), Matrix3d::Identity());
	control0 = new Control(world->objectAtIndex<Cursor>(0)->getPosition(), world->objectAtIndex<Cursor>(0)->getRotation());	
	control1 = new Control(world->objectAtIndex<Cursor>(1)->getPosition(), world->objectAtIndex<Cursor>(1)->getRotation());	

	glGetDoublev(GL_MODELVIEW_MATRIX, model_view);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);
	
  signal(SIGINT, &interruptHandler);

	if (haptics)
		processInput(haptic0, haptic1);
	else
		processInput(mouse0, mouse1);

  
  worlds.push_back(new World(*world));
  interruptTimer = new Timer();
  interruptTimer->restart();
  
//  samples_problems.push_back("environmentFiles/TWIST_COEFF = BEND_COEFF*2.9");
//	samples_problems.push_back("environmentFiles/TWIST_COEFF = BEND_COEFF*2.8");
//	samples_problems.push_back("environmentFiles/STRETCH_COEFF = 0.05 * BEND_COEFF");
//	samples_problems.push_back("environmentFiles/STRETCH_COEFF = 0.2 * BEND_COEFF");
//	samples_problems.push_back("environmentFiles/REPULSION_COEFF = 10.0 * BEND_COEFF");
//	samples_problems.push_back("environmentFiles/REPULSION_COEFF = 1.0 * BEND_COEFF");
//	samples_problems.push_back("environmentFiles/REPULSION_COEFF = 0.1 * BEND_COEFF");
//	samples_problems.push_back("environmentFiles/REPULSION_COEFF = 0.0 * BEND_COEFF");
//	samples_problems.push_back("environmentFiles/initial_condition_problem_leftee_right");
//	samples_problems.push_back("environmentFiles/initial_condition_problem_rightee_left");
//	samples_problems.push_back("environmentFiles/initial_condition_problem_rightee_down");
//	samples_problems.push_back("environmentFiles/initial_condition_problem_rightee_up");
//	samples_problems.push_back("environmentFiles/initial_condition_problem_rightee_mdeg");
//	samples_problems.push_back("environmentFiles/initial_condition_problem_rightee_pdeg");
//	samples_problems.push_back("environmentFiles/initial_condition_problem_rightee_pdegup");
//	
	samples_initial_conditions1.push_back("environmentFiles/ic_left_tan_m_large");
	samples_initial_conditions1.push_back("environmentFiles/ic_left_tan_m_small");
	samples_initial_conditions1.push_back("environmentFiles/ic_left_tan_p_large");
	samples_initial_conditions1.push_back("environmentFiles/ic_left_tan_p_small");
	samples_initial_conditions1.push_back("environmentFiles/ic_left_tran_left_small");
	samples_initial_conditions1.push_back("environmentFiles/ic_left_tran_right_small");
	samples_initial_conditions1.push_back("environmentFiles/ic_right_roll_med");
	samples_initial_conditions1.push_back("environmentFiles/ic_right_roll_opposite");
	samples_initial_conditions1.push_back("environmentFiles/ic_right_roll_small");
	samples_initial_conditions1.push_back("environmentFiles/ic_right_tan_m_large");
	samples_initial_conditions1.push_back("environmentFiles/ic_right_tan_m_small");
	samples_initial_conditions1.push_back("environmentFiles/ic_right_tan_p_large");
	samples_initial_conditions1.push_back("environmentFiles/ic_right_tan_p_small");
	samples_initial_conditions1.push_back("environmentFiles/ic_right_tran_down_small");
	samples_initial_conditions1.push_back("environmentFiles/ic_right_tran_left_large");
	samples_initial_conditions1.push_back("environmentFiles/ic_right_tran_left_small");
	samples_initial_conditions1.push_back("environmentFiles/ic_right_tran_right_large");
	samples_initial_conditions1.push_back("environmentFiles/ic_right_tran_right_small");
	samples_initial_conditions1.push_back("environmentFiles/ic_right_tran_up_large");
	samples_initial_conditions1.push_back("environmentFiles/ic_right_tran_up_small");
	samples_initial_conditions1.push_back("environmentFiles/ic_small");
	
	samples_problems1.push_back("environmentFiles/ic_left_tan_m_large_traj");
	samples_problems1.push_back("environmentFiles/ic_left_tan_m_small_traj");
	samples_problems1.push_back("environmentFiles/ic_left_tan_p_large_traj");
	samples_problems1.push_back("environmentFiles/ic_left_tan_p_small_traj");
	samples_problems1.push_back("environmentFiles/ic_left_tran_left_small_traj");
	samples_problems1.push_back("environmentFiles/ic_left_tran_right_small_traj");
	samples_problems1.push_back("environmentFiles/ic_right_roll_med_traj");
	samples_problems1.push_back("environmentFiles/ic_right_roll_opposite_traj");
	samples_problems1.push_back("environmentFiles/ic_right_roll_small_traj");
	samples_problems1.push_back("environmentFiles/ic_right_tan_m_large_traj");
	samples_problems1.push_back("environmentFiles/ic_right_tan_m_small_traj");
	samples_problems1.push_back("environmentFiles/ic_right_tan_p_large_traj");
	samples_problems1.push_back("environmentFiles/ic_right_tan_p_small_traj");
	samples_problems1.push_back("environmentFiles/ic_right_tran_down_small_traj");
	samples_problems1.push_back("environmentFiles/ic_right_tran_left_large_traj");
	samples_problems1.push_back("environmentFiles/ic_right_tran_left_small_traj");
	samples_problems1.push_back("environmentFiles/ic_right_tran_right_large_traj");
	samples_problems1.push_back("environmentFiles/ic_right_tran_right_small_traj");
	samples_problems1.push_back("environmentFiles/ic_right_tran_up_large_traj");
	samples_problems1.push_back("environmentFiles/ic_right_tran_up_small_traj");
	samples_problems1.push_back("environmentFiles/ic_small_traj");

  glutMainLoop ();
}

void processInput(ControllerBase* controller0, ControllerBase* controller1)
{	
	
  if (absoluteControl) {
		vector<ControllerBase*> controllers;
		controllers.push_back(controller0);
		controllers.push_back(controller1);
	
		world->setTransformFromController(controllers, limit_displacement);
		
		if (trajectory_recorder.hasStarted())
			trajectory_recorder.writeWorldToFile(world);
	} else {
		control0->setControl(controller0);
		control1->setControl(controller1);
		
		if (haptics) { //TODO fix this hack
			control0->setTranslate(haptic0->getPosition() - world->objectAtIndex<Cursor>(0)->getPosition());
			control0->setRotate(world->objectAtIndex<Cursor>(0)->getRotation().transpose() * haptic0->getRotation());
			control1->setTranslate(haptic1->getPosition() - world->objectAtIndex<Cursor>(1)->getPosition());
			control1->setRotate(world->objectAtIndex<Cursor>(1)->getRotation().transpose() * haptic1->getRotation());
		} else {
			control0->setTranslate(mouse0->getPosition() - world->objectAtIndex<Cursor>(0)->getPosition());
			control0->setRotate(world->objectAtIndex<Cursor>(0)->getRotation().transpose() * mouse0->getRotation());
			control1->setTranslate(mouse1->getPosition() - world->objectAtIndex<Cursor>(1)->getPosition());
			control1->setRotate(world->objectAtIndex<Cursor>(1)->getRotation().transpose() * mouse1->getRotation());
		}
	
		if (trajectory_recorder_world.hasStarted())
			trajectory_recorder_world.writeWorldToFile(world);
		if (trajectory_recorder.hasStarted())
			trajectory_recorder.writeControlToFile(control0, control1);
	
		vector<Control*> controls;
		controls.push_back(control0);
		controls.push_back(control1);
	
		world->applyRelativeControl(controls, limit_displacement);
		
	}

  //worlds.push_back(new World(*world));
  if (worlds.size() > 10 && smoothingEnabled) { 
    sqpSmoother(worlds);
    worlds.clear();
    worlds.push_back(drawWorlds.back());
  }
}

void moveMouseToClosestEE(Mouse* mouse)
{
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

void bitmap_output(int x, int y, char *string, void *font)
{
  int len, i;

  glRasterPos2f(x, y);
  len = (int) strlen(string);
  for (i = 0; i < len; i++) {
    glutBitmapCharacter(font, string[i]);
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

void interruptHandler(int sig) {
  cout << "Time since last interrupt: " << interruptTimer->elapsed() << endl; 
  if (interruptTimer->elapsed() < 10) exit(0);
  cout << "You need to hold ctrl-c to forcefully exit the program!" << endl;

  interruptTimer->restart();
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
    cout << "Initial SQP score: " <<
      cost_metric(initial_world, goal_world) << endl; 

    while (cost_metric(initial_world, goal_world) > SQP_BREAK_THRESHOLD && !interruptEnabled) {
      // Generate initial trajectory
      vector<World*> initialization_worlds;
      
      initialization_worlds.push_back(new World(*initial_world)); 
      for (int i = 0; i < num_worlds-2; i++) {
        if (i % 1 == 0) {
          sample_on_sphere(du, norm); 
        }
        initial_world->applyRelativeControlJacobian(du);
        initialization_worlds.push_back(new World(*initial_world));
      }

      initialization_worlds.push_back(new World(*goal_world));

      vector<World*> sqpWorlds;
      vector<VectorXd> sqpControls;

      cout << "calling SQP solver" << endl; 
      solveSQP(initialization_worlds, sqpWorlds, sqpControls, namestring.c_str());

      vector<World*> openLoopWorlds;
      openLoopController(initialization_worlds, sqpControls, openLoopWorlds);

      for (int i = 0; i < openLoopWorlds.size(); i++) {
        completeOpenLoopTrajectory.push_back(new World(*openLoopWorlds[i]));
      }
      initial_world = new World(*openLoopWorlds.back());
    }
    
    //for smoothing, put objective in there
    completeOpenLoopTrajectory.push_back(new World(*goal_world));
    sqpSmoother(completeOpenLoopTrajectory);
    //drawWorlds.clear();
    //drawWorlds = worlds;
    //drawWorlds = openLoopWorlds;
    setVisualizationData(completeOpenLoopTrajectory);
}

void sqpSmoother(vector<World*>& trajectory_to_smooth) {

  getTrajectoryStatistics(trajectory_to_smooth);

  if (trajectory_to_smooth.size() == 0) return;
  cout << "Smoothing over " << trajectory_to_smooth.size() << " worlds" << endl; 

  vector<World*> sqpWorlds;
  vector<VectorXd> sqpControls;

  string namestring = "world_sqp_debug";
  solveSQP(trajectory_to_smooth, sqpWorlds, sqpControls, namestring.c_str(), false);

  vector<World*> openLoopWorlds;
  openLoopController(trajectory_to_smooth, sqpControls, openLoopWorlds);

  setVisualizationData(openLoopWorlds);
}
