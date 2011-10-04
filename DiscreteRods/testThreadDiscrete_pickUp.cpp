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

#include <dirent.h>

#include "IO/ControllerBase.h"
#include "IO/Mouse.h"
#include "IO/Haptic.h"
#include "IO/Control.h"
#include "thread_socket_interface.h"

#include "thread_discrete.h"
#include "ThreadConstrained.h"
#include "EnvObjects/World.h"
#include "EnvObjects/WorldManager.h"
#include "EnvObjects/Cursor.h"

#include "planner_lib.h"

#include "StateRecorder.h"
#include "StateReader.h"
#include "TrajectoryRecorder.h"
#include "TrajectoryReader.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace cv;

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

void processInput(ControllerBase* controller0, ControllerBase* controller1);
void moveMouseToClosestEE(Mouse* mouse);
void drawStuff();
void displayTextInScreen(const char* textline, ...);
void bitmap_output(int x, int y, const char* str, void *font);
void glutMenu(int ID);
void initGL();
void save_opengl_image(char* filename);
void interruptHandler(int sig);
//void sqpSmoother(vector<World*>& trajectory_to_smooth, vector<World*>& smooth_trajectory);
//void sqpPlanner(World* start, World* goal, vector<World*>& trajectory);
void closedLoopSQPController(World* start, vector<World*>& target, vector<vector<Control*> >& ctrls);
VectorXd closedLoopSQPStepper(World* start, World* goal, WorldSQP* solver);
void chunkSmoother(vector<World*>& dirty, vector<vector<Control*> >& ctrl_in, vector<World*>& smooth, vector<vector<Control*> >& smooth_controls);

static const GLfloat light_model_ambient[] = {0.3f, 0.3f, 0.3f, 1.0f};    
static const GLfloat lightOnePosition[] = {140.0, 0.0, 200.0, 0.0};
static const GLfloat lightOneColor[] = {0.5, 0.5, 0.5, 1.0};
static const GLfloat lightTwoPosition[] = {-140.0, 0.0, 200.0, 0.0};
static const GLfloat lightTwoColor[] = {0.9, 0.9, 0.9, 1.0};
static const GLfloat lightThreePosition[] = {140.0, 0.0, -200.0, 0.0};
static const GLfloat lightThreeColor[] = {0.5, 0.5, 0.5, 1.0};
static const GLfloat lightFourPosition[] = {-140.0, 0.0, -200.0, 0.0};
static const GLfloat lightFourColor[] = {0.9, 0.9, 0.9, 1.0};

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
RenderMode examine_mode = NORMAL;
int downsample = 0;

//IO
Haptic *haptic0, *haptic1;
Mouse *mouse0, *mouse1;
Control *control0, *control1;

//Environment
World *world;
WorldManager* test_world_manager;

//drawing SQP results
vector<vector<World*> > drawWorlds;
int drawInd = 0;
int drawWorldInd = 0; 

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

#define IMAGE_BASE_NAME "./videoImages/"
int im_save_ind = 1;

//for collecting data
DIR *dir = NULL;
struct dirent *ent;
ofstream data_file;
char* data_raw_filename;
vector<vector<World*> > world_data;

void setVisualizationData(vector<vector<World*> > vis_data) {
  drawWorlds = vis_data;
  drawWorldInd = 0; 
  drawInd = 0; 
}

void setVisualizationData(vector<World*>& visualize_worlds)
{
  vector<vector<World*> > all_vis_data;
  all_vis_data.push_back(visualize_worlds);
  setVisualizationData(all_vis_data);
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

void loadNextTrajectory()
{

}

void processNormalKeys(unsigned char key, int x, int y)
{
	/*
	if (key == 'o') {
		cout << "Please enter problem name (without extension) (i.e. x1): ";
	  data_raw_filename = new char[256];
	  cin >> data_raw_filename;
	  char *directory = new char[256];
	  sprintf(directory, "environmentFiles/%s/video_trajs/", data_raw_filename);
	  //sprintf(directory, "environmentFiles/%s/", data_raw_filename);
		dir = opendir (directory);
		if (dir != NULL) {
			cout << "Sucessfully opened directory" << endl;
//		  char *fullPath = new char[256];
//		  sprintf(fullPath, "%s%s%s%s%s", "environmentFiles/", data_raw_filename, "/", data_raw_filename, "_data.txt");
//			cout << "Writing data to: " << fullPath << endl;
//			data_file.precision(20);
//			data_file.open(fullPath);
			
			processNormalKeys('b', x, y);
			return;
		} else {
			cout << "Failed to open directory" << endl;
		}
	} else if (key == 'b') {
		cout << "--------------------------------------------" << endl;
		if (dir != NULL) {
			if ((ent = readdir (dir)) != NULL) {
				char *filename = new char[256];
				sprintf(filename, "%s", ent->d_name);
				vector<string> parameters;
				boost::split(parameters, filename, boost::is_any_of("_"));
				if (parameters.size() != 8 || parameters[0]!=string(data_raw_filename) || parameters[4]!="1.0") {
				//if (parameters.size() != 7 || parameters[0]!=string(data_raw_filename) || parameters[4]!="0.6" || parameters[3] != "0") {
				//if (parameters.size() != 7 || parameters[0]!=string(data_raw_filename) || parameters[3] == "5") {
					cout << "Ignoring file " << filename << endl;
					processNormalKeys('b', x, y);
					return;
				}
				string base_name = "environmentFiles/";
				base_name.append(data_raw_filename);
				//base_name.append("/");
				base_name.append("/video_trajs/");
				vector<World*> temp_worlds;
				TrajectoryReader read(filename, base_name.c_str());
				if (read.readWorldsFromFile(temp_worlds)) {
					cout << "Displaying last world." << endl;
				} else {
					assert(0);
				}
				world_data.push_back(temp_worlds);
				setVisualizationData(world_data);
    		drawWorldInd = drawWorlds.size()-1;
    		drawInd = drawWorlds[drawWorldInd].size()-1;
				cout << "Press y or n." << endl;
			} else {
				closedir (dir);
				dir = NULL;
//				data_file.close();
			  cout << "Writing data file Done\n";
			}
		} else {
			cout << "No directory is open" << endl;
		}
	} else if (key == 'y') {
//		data_file << "1 \n";
		cout << "You answered yes" << endl;
		processNormalKeys('b', x, y);
		return;
	} else if (key == 'n') {
//		data_file << "0 \n";
		cout << "You answered no" << endl;
		processNormalKeys('b', x, y);
		return;
	} else if (key == 'i') {
		cout << "Please enter image destination file name (without extension): ";
    char *dstFileName = new char[256];
    cin >> dstFileName;
		save_opengl_image(dstFileName);
	} else if (key == 'I') {
		TrajectoryReader trajectory_reader();
		trajectory_reader.queryFileName();
		
		cout << "Please enter image destination file path (without extension): ";
    char *dstPath = new char[256];
    cin >> dstPath;
    cout << "Image destination file path is " << dstPath << endl;
    
    vector<string> srcFullFileName_vect;
    string srcFullFileName_string = string(srcFullFileName);
    boost::split(srcFullFileName_vect, srcFullFileName_string, boost::is_any_of("/"));
    string srcFileName = srcFullFileName_vect.back();

		TrajectoryReader trajectory_reader(srcRawFileName);
		vector<World*> worlds_to_save;
		if (trajectory_reader.readWorldsFromFile(worlds_to_save)) {
			for (int i = 0; i < worlds_to_save.size(); i++) {
				world_data.push_back(worlds_to_save);
				setVisualizationData(world_data);
    		drawWorldInd = drawWorlds.size()-1;
    		drawInd = i; //drawWorlds[drawWorldInd].size()-1;
    		
    		drawStuff();
    		
    		char *dstFullFileName = new char[256];
    		sprintf(dstFullFileName, "%s%s_%.4d", dstPath, srcFileName.c_str(), i);
    		
				save_opengl_image(dstFullFileName);
			}
		} else {
			cout << "Unable to save images" << endl;
		}
	}
	*/
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
	} else if (key == 's') {
    StateRecorder state_recorder;
    state_recorder.queryFileName();
    state_recorder.writeWorldToFile(world);
  } else if ((key == 'a') || (key >= '0' && key <= '9')) { // loads a world state and puts it in the visualize drawing
  	StateReader state_reader;
  	if (key == 'a') {
  		state_reader.queryFileName();
  	} else {
	    char *file_name = new char[256];
	    sprintf(file_name, "%s%c", "s", key);
	    state_reader.setFileName(file_name);
	  }    
    if (state_reader.readWorldFromFile(world)) {
			vector<World*> vis_data;
			vis_data.push_back(new World(*world));
			setVisualizationData(vis_data); 
		}
	} else if (key == 'c') { // records control trajectory and world trajectory
		if (trajectory_recorder.hasStarted()) {
			cout << "Trajectory has already started. Stop it first before starting it." << endl;
		} else {
			trajectory_recorder.queryFileName();
			trajectory_recorder.start(CONTROL);
			char *file_name_world  = new char[256];
			trajectory_recorder.getFileName(file_name_world);
			//strcat(file_name_world, "_world");
			trajectory_recorder_world.setFileName(file_name_world);
			trajectory_recorder_world.start(STATE);
		}
	} else if (key == 'x') { // stops recording control trajectory and world trajectory
		if (!trajectory_recorder.hasStarted()) {
			cout << "Trajectory has not been started. Start it first before stopping it." << endl;
		} else {
			trajectory_recorder.stop();
			trajectory_recorder_world.stop();
		}
	} else if (key == 'k') { // loads a world trajectory and puts it in the visualize drawing
		TrajectoryReader trajectory_reader;
		trajectory_reader.queryFileName();
		char* file_name = new char[256];
		trajectory_reader.getFileName(file_name);
		
		string file_name_string(file_name);
		size_t found = file_name_string.find_last_of(".");
		string root_file_name = file_name_string.substr(0,found);
		string ext_file_name = file_name_string.substr(found+1);

		if (ext_file_name == "twd" || found == string::npos) {
			vector<World*> vis_data;
			if (trajectory_reader.readWorldsFromFile(vis_data)) {
				setVisualizationData(vis_data);
			}
		} else if (ext_file_name == "tcl") {
			vector<vector<Control*> > controls;
			trajectory_reader.readControlsFromFile(controls);

			char* world_file_name = new char[256];
			strcpy(world_file_name, root_file_name.c_str());
			strcat(world_file_name, ".twd");
			TrajectoryReader trajectory_reader_world;
			trajectory_reader_world.setFileName(world_file_name);
			vector<World*> temp_worlds;
			trajectory_reader_world.readWorldsFromFile(temp_worlds);

			vector<World*> openLoopWorlds; 
			openLoopWorlds.push_back(new World(*temp_worlds[0]));
			World* OLcopy = new World(*temp_worlds[0], test_world_manager);
			cout << "Step " << 0 << " / " << controls.size()-1 << endl;
			for (int i = 1; i < controls.size(); i++) {
			  fputs("\033[A\033[2K",stdout);
				rewind(stdout);
				int dummy = ftruncate(1,0);
			  cout << "Step " << i << " / " << controls.size()-1 << endl;
			  
			  OLcopy->applyRelativeControl(controls[i-1], 0, true);
			  openLoopWorlds.push_back(new World(*OLcopy));
			}
			delete OLcopy; 

			setVisualizationData(openLoopWorlds);
		} else {
			cout << "Unable to open trajectory. Unknown extension " << ext_file_name << endl;
		}

	} else if(key == 'd') { // puts the current world in start_world
		start_world = new World(*world);
	} else if(key == 'f') { // puts the current world in goal_world
		goal_world = new World(*world);
	} else if(key == '[') {
		drawStartWorld = !drawStartWorld;
	} else if(key == ']') {
		drawGoalWorld = !drawGoalWorld;
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
  	examine_mode = (RenderMode) ((examine_mode+1)%4);
  } else if(key == 'w') {
  	rotate_frame[0] = rotate_frame[1] = 0.0;
		translate_frame[0] = translate_frame[1] = 0.0;
		translate_frame[2] = -110.0;
  } else if(key == 'g') {
    vector<World*> plan; 
    //sqpPlanner(start_world, goal_world, plan);
    setVisualizationData(plan);
  } else if (key == 'G') {
    vector<World*> traj_to_smooth;
    vector<World*> completeOpenLoopTrajectory;
    //completeOpenLoopTrajectory.push_back(new World(*worlds.front()));
    completeOpenLoopTrajectory.push_back(worlds[200]);
    for (int i = 10; i < 25; i++) {
      cout << "Running set: " << i << endl; 
      traj_to_smooth.clear();
      traj_to_smooth.push_back(completeOpenLoopTrajectory.back());
      for (int j = 0; j < 20; j++) { 
        traj_to_smooth.push_back(new World(*worlds[i*20+j]));
      }
      vector<World*> smooth_traj; 
      //sqpSmoother(traj_to_smooth, smooth_traj);
      for (int j = 0; j < smooth_traj.size(); j++) { 
        completeOpenLoopTrajectory.push_back(new World(*smooth_traj[j]));
      }
    }
    setVisualizationData(completeOpenLoopTrajectory);
    //smoothingEnabled = !smoothingEnabled;
  } else if (key == 'v') { 
    getTrajectoryStatistics(worlds);
    //drawWorlds.clear(); 
    vector<World*> temp_worlds;
    getWaypoints(worlds, temp_worlds);
    setVisualizationData(temp_worlds);
    //cout << "number of waypoints = " << drawWorlds.size() << endl; 
  } else if (key == 'V') {
    vector<World*> waypoints;
    getWaypoints(worlds, waypoints);
    vector<World*> completeOpenLoopTrajectory;
    completeOpenLoopTrajectory.push_back(worlds[0]);
    for (int i = 0; i < 10; i++) {
      World* start = completeOpenLoopTrajectory.back();
      World* goal = waypoints[i];
      //sqpPlanner(start, goal, completeOpenLoopTrajectory);
    }
    setVisualizationData(completeOpenLoopTrajectory);
  } else if (key == '<') {
    if (drawWorldInd < drawWorlds.size()) { 
      drawInd = max(0, drawInd - 1);
      cout << drawInd << "/" << drawWorlds[drawWorldInd].size()-1 << endl;
    }
  } else if (key == '>') {
    if (drawWorldInd < drawWorlds.size()) { 
      drawInd = min((int) drawWorlds[drawWorldInd].size()-1, drawInd + 1);
      cout << drawInd << "/" << drawWorlds[drawWorldInd].size()-1 << endl;
    }
  } else if (key == ',') {
    drawWorldInd = max(0, drawWorldInd - 1);
    cout << "drawWorldInd = " << drawWorldInd << endl; 
  } else if (key == '.') {
    drawWorldInd = min((int) drawWorlds.size()-1, drawWorldInd + 1);
    cout << "drawWorldInd = " << drawWorldInd << endl;
  } else if (key == ';') { 
  	drawInteractiveWorld = !drawInteractiveWorld;
  }	else if (key == '\'') {
  	drawVisualizationData = !drawVisualizationData;
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
	checkMouseUpdate();
}

void drawStuff()
{
#ifdef VIEW3D
 	glutSetWindow(main_window);
#endif
  
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glClearColor(0.4, 0.4, 0.4, 0.0);
  glColor3f(1.0, 1.0, 1.0);
	glPushMatrix ();
  /* set up some matrices so that the object spins with the mouse */
  glTranslatef (translate_frame[0], translate_frame[1], translate_frame[2]);

#ifndef PICTURE
  glDisable(GL_LIGHTING);
	if (drawWorldInd < drawWorlds.size() &&
      drawInd < drawWorlds[drawWorldInd].size() &&
      drawWorlds[drawWorldInd][drawInd] != NULL &&
      drawVisualizationData) { 
    bitmap_output(50, 55, "Viz (')", GLUT_BITMAP_TIMES_ROMAN_24);
  }
  if (world && drawInteractiveWorld) {
    bitmap_output(50, 50, "Int (;)", GLUT_BITMAP_TIMES_ROMAN_24);
  }
  if (start_world && drawStartWorld) {
    bitmap_output(50, 45, "Start ([)", GLUT_BITMAP_TIMES_ROMAN_24);
  }
  if (goal_world && drawGoalWorld) {
    bitmap_output(50, 40, "Goal (])", GLUT_BITMAP_TIMES_ROMAN_24);
  }
  glEnable(GL_LIGHTING);
#endif

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
  
  glLightfv (GL_LIGHT0, GL_POSITION, lightOnePosition);
	glLightfv (GL_LIGHT1, GL_POSITION, lightTwoPosition);
	glLightfv (GL_LIGHT2, GL_POSITION, lightThreePosition);
	glLightfv (GL_LIGHT3, GL_POSITION, lightFourPosition);
  
  glGetDoublev(GL_MODELVIEW_MATRIX, model_view);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);
  
	if (drawWorldInd < drawWorlds.size() &&
      drawInd < drawWorlds[drawWorldInd].size() &&
      drawWorlds[drawWorldInd][drawInd] != NULL &&
      drawVisualizationData) { 
  	drawWorlds[drawWorldInd][drawInd]->draw(examine_mode);
  }
  if (world && drawInteractiveWorld) {
   	world->draw(examine_mode);
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
  test_world_manager = new WorldManager();
	world = new World(test_world_manager);
	
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

  
  interruptTimer = new Timer();
  interruptTimer->restart();

  glutMainLoop ();
}

void processInput(ControllerBase* controller0, ControllerBase* controller1)
{	
	control0->setControl(controller0);
	control1->setControl(controller1);

	bool button_change = control0->getButton(UP) ||
											 control0->getButton(DOWN) ||
											 control1->getButton(UP) ||
											 control1->getButton(DOWN);

	if (button_change)
		downsample = 0;
	else
		downsample = (downsample+1) % 10;
		
	if (downsample == 0) {

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
//			if (trajectory_recorder.hasStarted())
//				trajectory_recorder.writeControlToFile(control0, control1);

		vector<Control*> controls;
		controls.push_back(control0);
		controls.push_back(control1);

		Control* c0;
		Control* c1;
		Vector3d old_pos0;
		Vector3d old_pos1;
		Matrix3d old_rot0;
		Matrix3d old_rot1;
		
		if (trajectory_recorder.hasStarted()) {
			c0 = new Control(Vector3d::Zero(), Matrix3d::Identity());
			c1 = new Control(Vector3d::Zero(), Matrix3d::Identity());
			old_pos0 = world->objectAtIndex<Cursor>(0)->end_eff->getPosition();
			old_pos1 = world->objectAtIndex<Cursor>(1)->end_eff->getPosition();
			old_rot0 = world->objectAtIndex<Cursor>(0)->end_eff->getRotation();
			old_rot1 = world->objectAtIndex<Cursor>(1)->end_eff->getRotation();
			c0->setButton(UP, control0->getButton(UP));
			c1->setButton(UP, control1->getButton(UP));
			c0->setButton(DOWN, control0->getButton(DOWN));
			c1->setButton(DOWN, control1->getButton(DOWN));
		}
		
		world->applyRelativeControl(controls, NOISE_THRESHOLD, limit_displacement);
		
		if (trajectory_recorder.hasStarted()) {
			c0->setTranslate(world->objectAtIndex<Cursor>(0)->end_eff->getPosition() - old_pos0);
			c1->setTranslate(world->objectAtIndex<Cursor>(1)->end_eff->getPosition() - old_pos1);
			
			c0->setRotate(old_rot0.transpose() * world->objectAtIndex<Cursor>(0)->end_eff->getRotation());
			c1->setRotate(old_rot1.transpose() * world->objectAtIndex<Cursor>(1)->end_eff->getRotation());
			trajectory_recorder.writeControlToFile(c0, c1);
		}
		
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

void bitmap_output(int x, int y, const char* str, void *font)
{
  int len, i;

  glRasterPos2f(x, y);
  len = (int) strlen(str);
  for (i = 0; i < len; i++) {
    glutBitmapCharacter(font, str[i]);
  }
}

void initGL()        
{
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
	glLoadIdentity();
  // initialize lighting
  glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_FALSE);
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);    
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, light_model_ambient);
  glLightfv (GL_LIGHT0, GL_POSITION, lightOnePosition);
	glLightfv (GL_LIGHT0, GL_DIFFUSE, lightOneColor);
	glEnable (GL_LIGHT0);		// uncomment this if you want another source of light
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
  if (interruptTimer->elapsed() < 0.1) exit(0);
  cout << "You need to hold ctrl-c to forcefully exit the program!" << endl;

  interruptTimer->restart();
  interruptEnabled = true;
}

/*void sqpPlanner(World* start, World* goal, vector<World*>& completeOpenLoopTrajectory) { 

    int num_worlds = 15;
    string namestring = "world_sqp_debug";
    VectorXd du(12);
    double norm = 1e-1;
    if (!start || !goal) return;  

    World* initial_world = new World(*start);
    cout << "Planning over " << num_worlds << " worlds" << endl;
    cout << "Initial SQP score: " <<
      cost_metric(initial_world, goal) << endl; 

    while (cost_metric(initial_world, goal) > SQP_BREAK_THRESHOLD && !interruptEnabled) {
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

      initialization_worlds.push_back(new World(*goal));

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
    //completeOpenLoopTrajectory.push_back(new World(*goal_world));
    //drawWorlds.clear();
    //drawWorlds = worlds;
    //drawWorlds = openLoopWorlds;
    //setVisualizationData(completeOpenLoopTrajectory);
}*/

/*void sqpSmoother(vector<World*>& trajectory_to_smooth, vector<World*>& smooth_trajectory) {

  //getTrajectoryStatistics(trajectory_to_smooth);

  if (trajectory_to_smooth.size() == 0) return;
  cout << "Smoothing over " << trajectory_to_smooth.size() << " worlds" << endl; 

  vector<vector<World*> > traj_in;
  traj_in.push_back(trajectory_to_smooth);

  vector<vector<World*> > sqpWorlds;
  vector<VectorXd> sqpControls;

  string namestring = "world_sqp_debug";
  solveSQP(traj_in, sqpWorlds, sqpControls, namestring.c_str(), false);

  smooth_trajectory.clear();
  openLoopController(trajectory_to_smooth, sqpControls, smooth_trajectory);

  //setVisualizationData(openLoopWorlds);
}*/

void save_opengl_image(char* filename)
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


    vector<int> p(3);

    p[0] = CV_IMWRITE_PNG_COMPRESSION;
    p[1] = 20;
    p[2] = 0;


    merge(img_planes, img);
    flip(img, img, 0);

    char im_name[256];
    sprintf(im_name, "%s%s.png", IMAGE_BASE_NAME, filename);
    cout << "Image written to " << im_name << endl;
    im_save_ind++;
    imwrite(im_name, img, p);
    waitKey(1);
  //sleep(0);
} 

VectorXd closedLoopSQPStepper(World* start, World* goal, WorldSQP* solver) { 
  if (solver == NULL) assert(false); //initialize your shit
  
  solver->popStart(); //pop the initial start 
  solver->popStart(); //pop the target start

  vector<World*> perturbations;
  perturbations.push_back(start);
  for (int i = 1; i < solver->num_traj(); i++) { 
   World* perturbed_world = new World(*start, test_world_manager);
   VectorXd du(12);
   sample_on_sphere(du, NOISE_THRESHOLD);
   perturbed_world->applyRelativeControlJacobian(du);
   World* input = new World(*perturbed_world);
   perturbations.push_back(input);
   delete perturbed_world;
  }

  solver->pushStart(perturbations); 
  solver->pushGoal(goal);

  for (int i = 0; i < perturbations.size(); i++) {
    //cout << &(*perturbations[i]) << endl; 
    //delete perturbations[i];
  }

  solver->solve();
  return solver->getStartControl();
}

void closedLoopSQPController(World* start, vector<World*>& target,
    vector<vector<Control*> >& ctrls) { 
  vector<int> horizon;
  //horizon.push_back(5);
  horizon.push_back(0);
  //horizon.push_back(30);
  //horizon.push_back(20);

  vector<vector<World*> > horizon_trajs;
  horizon_trajs.resize(horizon.size());


  for (int h = 0; h < horizon.size(); h++) { 
    cout << horizon[h] << endl; 

    int max_ind = target.size();
    if (horizon[h] == 0) { 
      vector<World*> openLoopWorlds; 
      openLoopWorlds.push_back(new World(*start));
      World* OLcopy = new World(*start, test_world_manager);
      for (int i = 1; i < max_ind; i++) { //state size change at 1088
        cout << "Step " << i << " / " << max_ind << endl;
        //if (interruptEnabled) break; 
        OLcopy->applyRelativeControl(ctrls[i-1], NOISE_THRESHOLD, true);
        openLoopWorlds.push_back(new World(*OLcopy));
      }
      delete OLcopy; 
      horizon_trajs[h] = openLoopWorlds;

    } else { 
      int T = horizon[h];
      vector<World*> init_worlds;
      for (int i = 0; i < T; i++) {
        init_worlds.push_back(target[i]); // solver will make copies
      }
      vector<vector<World*> > sqp_init;
      sqp_init.push_back(init_worlds);
      //sqp_init.push_back(init_worlds);
      //sqp_init.push_back(init_worlds);
      //sqp_init.push_back(init_worlds);

      WorldSQP* solver; 
      solver = new WorldSQP(0,0,0); /// HACK!!
      char namestring[128];
      sprintf(namestring, "clsqp_stepper_h_%d", horizon[h]);
      cout << "namestring = " << namestring << endl; 
      solver->set_namestring(namestring);
      solver->initializeClosedLoopStepper(start, sqp_init);
      solver->solve();

      vector<World*> closedLoopWorlds;
      closedLoopWorlds.push_back(new World(*start));
      VectorXd ctrl_cl_0 = solver->getStartControl();

      World* CLcopy = new World(*start, test_world_manager);
      //start_copy->applyRelativeControlJacobian(ctrl_cl_0);
      //closedLoopWorlds.push_back(new World(*start_copy));

      for (int i = 1; i < max_ind; i++) { //state size change at 1088
        cout << "Step " << i << " / " << max_ind << endl;
        //if (interruptEnabled) break; 
        if (i + T > target.size() - 1) T = target.size() - i - 1;
        CLcopy->applyRelativeControl(ctrls[i-1], NOISE_THRESHOLD, true);  
        VectorXd cl_ctrl = closedLoopSQPStepper(CLcopy, target[i+T], solver);
        CLcopy->applyRelativeControlJacobian(cl_ctrl, NOISE_THRESHOLD);
        closedLoopWorlds.push_back(new World(*CLcopy));
      }
      delete CLcopy;

      horizon_trajs[h] = closedLoopWorlds;
    }
    //if (interruptEnabled) break; 

  }


    //solver->getCurrentStates(closedLoopWorlds);
  
  vector< vector<World*> > visualization_data; 

  visualization_data.push_back(target);
  for (int h = 0; h < horizon_trajs.size(); h++) {
    visualization_data.push_back(horizon_trajs[h]);
  }

  setVisualizationData(visualization_data);

}

void chunkSmoother(vector<World*>& traj_in, vector<vector<Control*> >& controls_in, vector<World*>& traj_out, vector<vector<Control*> >& controls_out) {
  int size_each_chunk = 10;
	assert((traj_in.size()%size_each_chunk) == 0);
  int num_chunks = traj_in.size() / size_each_chunk;

  vector<vector<World*> > chunks;
  vector<vector<vector<Control*> > > chunk_ctrls; 

  for (int i = 0; i < num_chunks; i++) {
    vector<World*> individual_chunk;
    vector<vector<Control*> > individual_ctrls; 
    for (int j = 0; j < size_each_chunk; j++) {
      individual_chunk.push_back(traj_in[i*size_each_chunk + j]);
      individual_ctrls.push_back(controls_in[i*size_each_chunk +j]);
    }
    chunks.push_back(individual_chunk);
    chunk_ctrls.push_back(individual_ctrls);
  }

  vector<vector<World*> > smooth_chunks;
  vector<vector<vector<Control *> > > smooth_controls;

  smooth_chunks.resize(chunks.size());
  smooth_controls.resize(chunks.size());

  //#pragma omp parallel for
  for (int i = 0; i < chunks.size(); i++) {
    //smooth each chunk
    char namestring[128];
    sprintf(namestring, "sqp_smoother_chunk_%d", i);
    vector<vector<World*> > smooth_chunk;
    vector<vector<Control*> > smooth_control;
    vector<vector<World*> > sqp_init;
    sqp_init.push_back(chunks[i]);
    chunk_ctrls[i].pop_back();
    solveSQP(sqp_init, chunk_ctrls[i], smooth_chunk, smooth_control, namestring, false);
    vector<Control *>  du;
    for (int j = 0; j < 2; j++) {
      du.push_back(new Control(Vector3d::Zero(), Matrix3d::Identity()));
    }
    smooth_control.push_back(du);
    smooth_chunks[i] = smooth_chunk[0];
    smooth_controls[i] = smooth_control;
  }

  for (int i = 0; i < smooth_chunks.size(); i++) { 
    for (int j = 1; j < smooth_chunks[i].size()-1; j++) {
      traj_out.push_back(smooth_chunks[i][j]);
      controls_out.push_back(smooth_controls[i][j]);
    }
  }

  vector<vector<World *> > visualization_data;
  visualization_data.push_back(traj_in);
  visualization_data.push_back(traj_out);
  setVisualizationData(visualization_data);
}
