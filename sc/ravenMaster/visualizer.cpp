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

#define CHECKERS_PER_ROW 8.0
#define CHECKERS_PER_COL 6.0
#define SIZE_EACH_CHECKER 1.0

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

void estimatePointsInRobotHand(const Vector3d& startPt, const Matrix3d& startRot, const Vector3d& offsetGuess, const double offsetAngGuess, vector<Vector3d>& estimatedPoints);

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
			data_i++;
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
  Vector3d diff_pos = pos;
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

void drawStuff() {
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glColor3f (0.8, 0.3, 0.6);
  
  glPushMatrix ();
  
  glTranslatef (0.0,0.0,-110.0);
  glRotatef (rotate_frame[1], 1.0, 0.0, 0.0);
  glRotatef (rotate_frame[0], 0.0, 0.0, 1.0);
  
  cout << "rotate_frame: \t" << rotate_frame[0] << "\t" << rotate_frame[1] << endl;
  
  vector<double> pos_ori;
  for (int i=0; i<15; i++) {
  	pos_ori.push_back(data[data_i][i]);
  }
  
  Matrix3d rot, rot2, rot3;
	rot = Eigen::AngleAxisd(pos_ori[3], Vector3d::UnitZ())
  			* Eigen::AngleAxisd(pos_ori[4], Vector3d::UnitY())
  			* Eigen::AngleAxisd(pos_ori[5], Vector3d::UnitX());
	
	Vector3d current_point;
	current_point << pos_ori[0]*200.0, pos_ori[1]*200.0, pos_ori[2]*200.0; 
	
	vector<Vector3d> estimatedPoints; 
	estimatePointsInRobotHand(current_point, rot, Vector3d(0.0, 0.0, 0.0), 0.0 * M_PI/180.0, estimatedPoints);
	
	double transformYPR[16] = { rot(0,0) 				, rot(1,0) 					, rot(2,0) 					, 0 ,
		 											 rot(0,1) 				, rot(1,1) 					, rot(2,1) 					, 0 ,
													 rot(0,2) 				, rot(1,2) 					, rot(2,2) 					, 0 ,
													 pos_ori[0]*200.0 , pos_ori[1]*200.0	, pos_ori[2]*200.0	, 1 };
	
	double transform[16] = { pos_ori[6], pos_ori[9], pos_ori[12], 0,
													 pos_ori[7], pos_ori[10], pos_ori[13], 0,
													 pos_ori[8], pos_ori[11], pos_ori[14], 0,
													 pos_ori[0]*200.0 , pos_ori[1]*200.0	, pos_ori[2]*200.0	, 1 };
	
	//drawCursor(transformYPR, 0.0, 1.0, 0.5);
	
	drawCursor(transform, 0.0, 0.5, 1.0);
	
	drawAxes(current_point, rot);
	labelAxes(current_point, rot);
	
	for (int i=0; i<estimatedPoints.size(); i++) {
		drawSphere(estimatedPoints[i], 0.5, 0.0, 0.7, 0.7);
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

void estimatePointsInRobotHand(const Vector3d& startPt, const Matrix3d& startRot, const Vector3d& offsetGuess, const double offsetAngGuess, vector<Vector3d>& estimatedPoints)
{
  //rotate about x axis by offsetAng, then offsetGuess will lead to point in top-left corner
  Matrix3d rotatedAboutX = startRot * Eigen::AngleAxisd(offsetAngGuess, Vector3d::UnitX());
  Vector3d middlePt = startPt + rotatedAboutX*Vector3d(0.0, -(CHECKERS_PER_ROW-1.0)/2.0, 0.0);
  Vector3d offsetGuessRotated = rotatedAboutX*offsetGuess + middlePt;
  Vector3d yAxisRotated = rotatedAboutX*Vector3d::UnitY();
  Vector3d zAxisRotated = rotatedAboutX*Vector3d::UnitZ();

  estimatedPoints.resize(CHECKERS_PER_ROW*CHECKERS_PER_COL);
  for (int c = 0; c < CHECKERS_PER_COL; c++)
  {
    int cInd = c*CHECKERS_PER_ROW;
    for (int r = 0; r < CHECKERS_PER_ROW; r++)
    {
      estimatedPoints[cInd+r] = offsetGuessRotated + yAxisRotated*r*SIZE_EACH_CHECKER - zAxisRotated*c*SIZE_EACH_CHECKER;
    }
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
