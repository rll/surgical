#include <stdio.h> 
#include <cstring> 
#include "DS0.h"
#include <iostream>
#include <fstream>
#include <string> 
#include <vector> 

#ifdef MAC
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#include <GL/gle.h>
#else
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/gle.h> 
#endif

#include <Eigen/Core>
#include <Eigen/Geometry>

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

using namespace std;

#define RAD2DEG *180.0/M_PI
#define MICRORADS_PER_RAD 1000000
#define MICROMETERS_PER_METER 1000000 

vector<vector<double> > data; 
char* parsedFile;
int data_i = 0;
GLuint gCursorDisplayList = 0;

#define MOVE_POS_CONST 1.0
#define MOVE_TAN_CONST 0.2
#define ROTATE_TAN_CONST 0.2

float lastx_L=0;
float lasty_L=0;
float lastx_R=0;
float lasty_R=0;

float rotate_frame[2];
float last_rotate_frame[2];

float move[2];

int pressed_mouse_button;

void processLeft(int x, int y) {
	rotate_frame[0] += x-lastx_L;
	rotate_frame[1] += lasty_L-y;

	lastx_L = x;
	lasty_L = y;
}

void processRight(int x, int y) {
	rotate_frame[0] += x-lastx_L;
	rotate_frame[1] += lasty_L-y;

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
	
	if (key == 'n') {
		if (data_i < data.size()-1) {
			data_i+=10;
      cout << data_i << endl; 
			glutPostRedisplay();
		} else {
			cout << "There is no next data" << endl;
		}
	} if (key == 'p') {
	 	if (data_i > 0) {
		 	data_i--;
			glutPostRedisplay();
		} else {
			cout << "There no previous data" << endl;
		}
	}
}
	  
void initGL() {
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

void drawCylinder(double transform[16], float color0, float color1, float color2) {
  static const double kCursorRadius = 1;
  static const double kCursorHeight = 1;
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
    gluCylinder(qobj, kCursorRadius, 0.0, kCursorHeight,
                kCursorTess, kCursorTess);

    gluDeleteQuadric(qobj);
    glEndList();
  }


  // Get the proxy transform in world coordinates
 	glMultMatrixd(transform);

  // Apply the local cursor scale factor.
  double gDiamondScale = 0.5;
  glScaled(gDiamondScale, gDiamondScale, gDiamondScale);

  glEnable(GL_COLOR_MATERIAL);
  glColor3f(color0, color1, color2);

  glCallList(gCursorDisplayList);
  
  glPopMatrix();
  glPopAttrib();
}

void drawCursor(double transform[16], float color0, float color1, float color2) {
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
 	glMultMatrixd(transform);

  // Apply the local cursor scale factor.
  double gCursorScale = 10;
  glScaled(gCursorScale, gCursorScale, gCursorScale);

  glEnable(GL_COLOR_MATERIAL);
  glColor3f(color0, color1, color2);

  glCallList(gCursorDisplayList);
  
  glPopMatrix();
  glPopAttrib();
}

void createTransform(vector<double>& pos_ori, double transform[16]) { 
  Matrix3d rot;
  rot = Eigen::AngleAxisd(pos_ori[3], Vector3d::UnitZ())
  			* Eigen::AngleAxisd(pos_ori[4], Vector3d::UnitY())
  			* Eigen::AngleAxisd(pos_ori[5], Vector3d::UnitX());
  transform[0] = rot(0,0);
  transform[1] = rot(1,0);
  transform[2] = rot(2,0);
  transform[3] = 0;
  transform[4] = rot(0,1);
  transform[5] = rot(1,1);
  transform[6] = rot(2,1);;
  transform[7] = 0;
  transform[8] = rot(0,2);
  transform[9] = rot(1,2);
  transform[10] = rot(2,2);
  transform[11] = 0;
  transform[12] = pos_ori[0]*200;
  transform[13] = pos_ori[1]*200;
  transform[14] = pos_ori[2]*200; 
  transform[15] = 1; 
}

void drawStuff() {
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glColor3f(0.8, 0.3, 0.6);
  glPushMatrix();
  glTranslatef (0.0,0.0,-110.0);
  glRotatef (rotate_frame[1], 1.0, 0.0, 0.0);
  glRotatef (rotate_frame[0], 0.0, 0.0, 1.0);
  
  cout << "rotate_frame: \t" << rotate_frame[0] << "\t" << rotate_frame[1] << endl;
  
  vector<double> pos_ori;
  for (int i=0; i<15; i++) {
  	pos_ori.push_back(data[data_i][i]);
  }
  
  Matrix3d other_rot, rot, rot2, rot3;
	other_rot = Eigen::AngleAxisd(pos_ori[3], Vector3d::UnitZ())
  			* Eigen::AngleAxisd(pos_ori[4], Vector3d::UnitY())
  			* Eigen::AngleAxisd(pos_ori[5], Vector3d::UnitX());
  
  rot = other_rot;

	double transformYPR[16] = { rot(0,0) 				, rot(1,0) 					, rot(2,0) 					, 0 ,
		 											 rot(0,1) 				, rot(1,1) 					, rot(2,1) 					, 0 ,
													 rot(0,2) 				, rot(1,2) 					, rot(2,2) 					, 0 ,
													 pos_ori[0]*200.0 , pos_ori[1]*200.0	, pos_ori[2]*200.0	, 1 };
	/*
	double transform[16] = { pos_ori[6], pos_ori[9], pos_ori[12], 0,
													 pos_ori[7], pos_ori[10], pos_ori[13], 0,
													 pos_ori[8], pos_ori[11], pos_ori[14], 0,
													 pos_ori[0]*200.0 , pos_ori[1]*200.0	, pos_ori[2]*200.0	, 1 };
	*/
  double transform[16];
  for (int i = 0; i < data_i; i++) { 
    createTransform(data[i], transform); 
    //drawCursor(transform, 0.0, 0.5, 1.0);
    drawCylinder(transform, 0.0, 0.5, 1.0); 
    //drawCursor(transformYPR, 0.0, 1.0, 0.5);
  }
	glPopMatrix ();
	glutSwapBuffers ();
}

unsigned int byteswap (unsigned int nLongNumber) {
	return (((nLongNumber&0x000000FF)<<24)+((nLongNumber&0x0000FF00)<<8)+
					((nLongNumber&0x00FF0000)>>8)+((nLongNumber&0xFF000000)>>24));
}
void loadParsedFile(const char* fileName, vector<vector<double> >& data) {
  ifstream parsedFile(fileName, ios::in);
  data.resize(0); 
  if (parsedFile.good()) { 
    cout << "Log file open success" << endl; 
    while(!parsedFile.eof()) {
      vector<double> point; 
      for (int i = 0; i < 15; i++) { 
        double value;
        parsedFile >> value; 
        point.push_back(value); 
      }
      data.push_back(point); 
    }
    // last point is bogus so pop it
    data.pop_back(); 
  }
}

int main(int argc, char* argv[]) {
	/* initialize glut */
	glutInit (&argc, argv); //can i do that?
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(900,900);
	glutCreateWindow ("Thread");	
	glutDisplayFunc (drawStuff);
	glutMotionFunc (MouseMotion);
  glutMouseFunc (processMouse);
  glutKeyboardFunc(processNormalKeys);

	initGL();
	
	parsedFile = argv[1]; 
  loadParsedFile(parsedFile, data);
	
  glutMainLoop ();

  return 0; 
}
