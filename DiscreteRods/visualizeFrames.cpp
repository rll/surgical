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
#include "trajectory_reader.h"
#include "glThread_stripe.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include "text3d.h"

#define GL_WIDTH_PIX 1200
#define GL_HEIGHT_PIX 1200

#define IMG_SAVE_OPENGL "savedIms/opengl_"
#define IMG_SAVE_BENDING "savedIms/bending/opengl_"
#define IMG_SAVE_BENDING_MINIMIZE "savedIms/bending/minimize/opengl_"
#define IMG_SAVE_BENDING_SINGLES "savedIms/singles/opengl_"
#define IMG_SAVE_TWISTING_PROPOGATE "savedIms/twisting/propogateBishop/opengl_"
#define IMG_SAVE_TWISTING "savedIms/twisting/opengl_"
#define IMG_SAVE_MOVEMENT "savedIms/movement/opengl_"



// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN
using namespace cv;


void InitLights();
void InitGLUT(int argc, char * argv[]);
void InitStuff();
void DrawStuff();
void saveImages_setLoc();
void saveImages_init();
void saveImages_slowMini(const char* base_im_name);
void saveImages_gravity();
void saveImages_bending();
void saveImages_twist();
void saveImages_propogateBishop();
void saveImages_movement();
void saveImages_setDefaultColor();
void DrawAxes();
void DrawTextAtTop(string& str);
void InitThread(int argc, char* argv[]);
void save_opengl_image(const char* img_base_name = IMG_SAVE_OPENGL);
void update_all_threads();
void handleResize(int w, int h);




#define NUM_PTS 500
// #define THREAD_RADII 1.0
#define MOVE_POS_CONST 1.0
#define MOVE_TAN_CONST 0.2
#define ROTATE_TAN_CONST 0.2


enum key_code {NONE, MOVEPOS, MOVETAN, ROTATETAN};

#define NUMBER_THREADS 1
GLThread* glThreads[NUMBER_THREADS];
int curThread = 0;
int startThread = 0;

//params for each of the threads EXCEPT start thread
//these are set in InitThread
double twist_params[NUMBER_THREADS];
double grav_params[NUMBER_THREADS];

#ifdef ISOTROPIC
double bend_params[NUMBER_THREADS];
#else
Matrix2d bend_params[NUMBER_THREADS];
#endif



int thread_ind = 0;
int image_ind = 1;


float lastx_L=0;
float lasty_L=0;
float lastx_R=0;
float lasty_R=0;

float rotate_frame[2];
float move_end[2];
float tangent_end[2];
float tangent_rotation_end[2];


// double radii[NUM_PTS];
int pressed_mouse_button;

key_code key_pressed;



/* set up a light */
GLfloat lightOnePosition[] = {140.0, 0.0, 200.0, 0.0};
GLfloat lightOneColor[] = {0.99, 0.99, 0.99, 1.0};

GLfloat lightTwoPosition[] = {-140.0, 0.0, 200.0, 0.0};
GLfloat lightTwoColor[] = {0.99, 0.99, 0.99, 1.0};

GLfloat lightThreePosition[] = {140.0, 0.0, -200.0, 0.0};
GLfloat lightThreeColor[] = {0.99, 0.99, 0.99, 1.0};

GLfloat lightFourPosition[] = {-140.0, 0.0, -200.0, 0.0};
GLfloat lightFourColor[] = {0.99, 0.99, 0.99, 1.0};

GLfloat lightFivePosition[] = {-140.0, 0.0, 0.0, 0.0};
GLfloat lightFiveColor[] = {0.99, 0.99, 0.99, 1.0};

GLfloat lightSixPosition[] = {140.0, 0.0, 0.0, 0.0};
GLfloat lightSixColor[] = {0.99, 0.99, 0.99, 1.0};

GLfloat lightSevenPosition[] = {0.0, 0.0, -200.0, 0.0};
GLfloat lightSevenColor[] = {0.99, 0.99, 0.99, 1.0};

GLfloat lightEightPosition[] = {0.0, 0.0, 200.0, 0.0};
GLfloat lightEightColor[] = {0.99, 0.99, 0.99, 1.0};



void applyControl(Thread* start, const VectorXd& u, VectorXd* res) {
  int N = glThreads[startThread]->getThread()->num_pieces();
  res->setZero(3*N);

  Vector3d translation;
  translation << u(0), u(1), u(2);

  double dw = 1.0 - u(3)*u(3) - u(4)*u(4) - u(5)*u(5);
  if (dw < 0) {
    cout << "Huge differential quaternion: " << endl;
  }
  dw = (dw > 0) ? dw : 0.0;
  dw = sqrt(dw);
  Eigen::Quaterniond q(dw, u(3),u(4),u(5));
  Matrix3d rotation(q);

  // apply the control u to thread start, and return the new config in res
  Frame_Motion toMove(translation, rotation);

  Vector3d end_pos = start->end_pos();
  Matrix3d end_rot = start->end_rot();
  toMove.applyMotion(end_pos, end_rot);
  start->set_end_constraint(end_pos, end_rot);
  start->minimize_energy();

  start->toVector(res);
}

void computeDifference(Thread* a, Thread* b, VectorXd* res) {
  VectorXd avec;
  a->toVector(&avec);
  VectorXd bvec;
  b->toVector(&bvec);

  (*res) = avec - bvec;
}


void selectThread(int inc) {
  curThread = (curThread + inc) % NUMBER_THREADS;
  glutPostRedisplay();
}

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
    move_end[0] += (x-lastx_R)*MOVE_POS_CONST;
    move_end[1] += (lasty_R-y)*MOVE_POS_CONST;
  } else if (key_pressed == MOVETAN)
  {
    tangent_end[0] += (x-lastx_R)*MOVE_TAN_CONST;
    tangent_end[1] += (lasty_R-y)*MOVE_TAN_CONST;
  } else if (key_pressed == ROTATETAN)
  {
    tangent_rotation_end[0] += (x-lastx_R)*ROTATE_TAN_CONST;
    tangent_rotation_end[1] += (lasty_R-y)*ROTATE_TAN_CONST;
  }

  lastx_R = x;
  lasty_R = y;
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


void processSpecialKeys(int key, int x, int y) {
 /*if (key == GLUT_KEY_LEFT) {
   if(initialized) {
     if (curNode->prev != NULL) {
       curNode = curNode->prev;
       glThreads[7]->setThread(new Thread(curNode->x, VectorXd::Zero(curNode->N/3), Matrix3d::Identity()));
       glThreads[7]->minimize_energy();
       glutPostRedisplay();
     }
   }
 } else if (key == GLUT_KEY_RIGHT) {
   if(initialized) {
     if (curNode->next != NULL) {
       curNode = curNode->next;
       glThreads[7]->setThread(new Thread(curNode->x, VectorXd::Zero(curNode->N/3), Matrix3d::Identity()));
       glThreads[7]->minimize_energy();
       glutPostRedisplay();
     }
   }
 }
 */
}

void processNormalKeys(unsigned char key, int x, int y)
{
  if (key == 't') {
    key_pressed = MOVETAN;
  }
  else if (key == 'm') {
    key_pressed = MOVEPOS;
  }
  else if (key == 'r') {
    key_pressed = ROTATETAN;
  } else if (key == 's') {
		save_opengl_image();
	} else if (key == 27) {
    exit(0);
  }

  lastx_R = x;
  lasty_R = y;

}

void processKeyUp(unsigned char key, int x, int y)
{
  if (key == '+') {
    selectThread(1);
  } else if (key == '-') {
    selectThread(-1);
  }
  key_pressed = NONE;
  move_end[0] = move_end[1] = tangent_end[0] = tangent_end[1] = tangent_rotation_end[0] = tangent_rotation_end[1] = 0.0;
}



void JoinStyle (int msg)
{
  exit (0);
}


int main (int argc, char * argv[])
{

  srand(time(NULL));
  srand((unsigned int)time((time_t *)NULL));

  printf("Instructions:\n"
      "Hold down the left mouse button to rotate image: \n"
      "\n"
      "Hold 'm' while holding down the right mouse to move the end\n"
      "Hold 't' while holding down the right mouse to rotate the tangent \n"
      "\n"
      "Press 'Esc' to quit\n"
      );

  InitGLUT(argc, argv);
  InitLights();
  InitStuff ();



  InitThread(argc, argv);

	//glDisable(GL_LIGHTING);
	rotate_frame[0] = -52.0;
	rotate_frame[1] = -52.0;


	saveImages_init();
 	//saveImages_slowMini(IMG_SAVE_BENDING_MINIMIZE);
	saveImages_gravity();
	saveImages_bending();
	saveImages_twist();
	saveImages_propogateBishop();
	saveImages_movement();


  // for (int i=0; i < NUM_PTS; i++)
  // {
  //   radii[i]=THREAD_RADII;
  // }

  glutMainLoop ();
}


GLuint sphereList;
void InitStuff (void)
{
  //glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
  //glClearColor(.05f, 0.05f, 0.05f, 0.0f);
  glEnable(GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  rotate_frame[0] = 30.0;
  rotate_frame[1] = -75.0;
	glutReshapeFunc(handleResize);

  sphereList = glGenLists(1);
  glNewList(sphereList, GL_COMPILE);
  glutSolidSphere(0.5,16,16);

  glEndList();
}

/* draw the helix shape */
void DrawStuff (void)
{
	//saveImages_bending();
	//return;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  //glColor3f (0.8, 0.3, 0.6);

  glPushMatrix ();

  /* set up some matrices so that the object spins with the mouse */
	saveImages_setDefaultColor();
  glTranslatef (-17.0,18.0,-85.0);
  glRotatef (rotate_frame[1], 1.0, 0.0, 0.0);
  glRotatef (rotate_frame[0], 0.0, 0.0, 1.0);

  //change thread, if necessary
  if (move_end[0] != 0.0 || move_end[1] != 0.0 || tangent_end[0] != 0.0 || tangent_end[1] != 0.0 || tangent_rotation_end[0] != 0 || tangent_rotation_end[1] != 0)
  {
    glThreads[startThread]->ApplyUserInput(move_end, tangent_end, tangent_rotation_end);
	for (int i=0; i < NUMBER_THREADS; i++)
	{
		glThreads[i]->setThread((new Thread(*glThreads[startThread]->getThread())));
	}

  }


  //Draw Axes
  DrawAxes();

  glThreads[startThread]->updateThreadPoints();
  Vector3d display_start_pos = glThreads[startThread]->getStartPosition();


  for(int i = 0; i < NUMBER_THREADS; i++) {
    //Draw Thread
		//colors are R, G, B, Opacity
		glColor4f (1.0, 1.0, 1.0, 1.0);

    glThreads[i]->display_start_pos = display_start_pos;
    glThreads[i]->DrawThread();
    glThreads[i]->DrawSpheresAtLinks();
    glThreads[i]->DrawAxes();

  }

  glPopMatrix ();
  glutSwapBuffers ();
}


void DrawTextAtTop(string& str)
{
  glPushMatrix ();
	glColor3d(0.8, 0.1, 0.0);
	glTranslatef(20.0, 15.0, 5.0);
	glRotatef(50,0.0,0.0,1.0);
	glRotatef(75,1.0,0.0,0.0);
	glScalef(4,4,4);
	t3dDraw3D(str, 0.0, 0.0, 0.2); 
  glPopMatrix ();
}

void saveImages_init()
{
	image_ind = 0;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix ();

	saveImages_setLoc();
	saveImages_setDefaultColor();
	
	for (int i=0; i < 8; i++)
		glThreads[startThread]->getThread()->rotate_end_by(-M_PI/4.0);
	glThreads[startThread]->getThread()->minimize_energy_twist_angles();
	glThreads[startThread]->getThread()->minimize_energy();



	glThreads[startThread]->removeTexture_stripe();
	glThreads[startThread]->DrawThread();
	glThreads[startThread]->DrawAxes();
	glThreads[startThread]->DrawSpheresAtLinks();
	save_opengl_image(IMG_SAVE_BENDING_SINGLES);
	

  glPopMatrix ();
  glutSwapBuffers ();

}

void saveImages_slowMini(const char* base_im_name)
{
	image_ind = 0;
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix ();
	saveImages_setLoc();

	saveImages_setDefaultColor();
	
	while (!glThreads[startThread]->getThread()->minimize_energy(2, MIN_MOVEMENT_VERTICES*1e-1, MAX_MOVEMENT_VERTICES, 1e-6))
	{
		glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		saveImages_setDefaultColor();
		glThreads[startThread]->DrawThread();
		glThreads[startThread]->DrawAxes();
		glThreads[startThread]->DrawSpheresAtLinks();
		save_opengl_image(base_im_name);
	}
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	saveImages_setDefaultColor();
	glThreads[startThread]->DrawThread();
	glThreads[startThread]->DrawAxes();
	glThreads[startThread]->DrawSpheresAtLinks();
	save_opengl_image(base_im_name);

  glPopMatrix();
  glutSwapBuffers();
}

void handleResize(int w, int h) {
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, (double)w / (double)h, 1.0, 200.0);
}

void saveImages_gravity()
{
	t3dInit();
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix ();
	saveImages_setLoc();

	//glThreads[startThread]->minimize_energy();

	saveImages_setDefaultColor();
	
	/* plain thread, not minimized */
	glThreads[startThread]->removeTexture_stripe();
	glThreads[startThread]->DrawThread();
	glThreads[startThread]->DrawAxes();
	glThreads[startThread]->DrawSpheresAtLinks();
	for (int i=1; i < glThreads[startThread]->getThread()->num_pieces()-1; i++)
		glThreads[startThread]->DrawDownvecAtInd(i);

	string str = "Gravity";
	DrawTextAtTop(str);

	

	save_opengl_image(IMG_SAVE_BENDING_SINGLES);

  glPopMatrix ();
  glutSwapBuffers ();
}



void saveImages_bending()
{
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix ();
	saveImages_setLoc();

	//glThreads[startThread]->minimize_energy();

	saveImages_setDefaultColor();
	
	/* plain thread, not minimized */
	glThreads[startThread]->removeTexture_stripe();
	glThreads[startThread]->DrawThread();
	glThreads[startThread]->DrawAxes();
	for (int i=1; i < glThreads[startThread]->getThread()->num_pieces()-1; i++)
		glThreads[startThread]->DrawAngleArcAtPoint(i);
	glThreads[startThread]->DrawSpheresAtLinks();
	
	string str = "Bend";
	DrawTextAtTop(str);
	
	save_opengl_image(IMG_SAVE_BENDING_SINGLES);

  glPopMatrix ();
  glutSwapBuffers ();
}

void saveImages_twist()
{
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix ();
	saveImages_setLoc();

	//glThreads[startThread]->minimize_energy();

	saveImages_setDefaultColor();


	
	/* plain thread, not minimized */
	glThreads[startThread]->addTexture_stripe();
	glThreads[startThread]->DrawThread();
	glThreads[startThread]->DrawAxes();
	glThreads[startThread]->DrawSpheresAtLinks();
	save_opengl_image(IMG_SAVE_BENDING_SINGLES);
	int draw_each = 1;
	for (int i=draw_each; i < glThreads[startThread]->getThread()->num_pieces()-draw_each; i+=draw_each)
	{
		glThreads[startThread]->DrawAxesAtPoint(i, material, true); 
	}


	save_opengl_image(IMG_SAVE_BENDING_SINGLES);

  glPopMatrix ();
  glutSwapBuffers ();
}

void saveImages_propogateBishop()
{
	const int num_ims_each_seg = 30;
	const int num_ims_each_bishop_update = 22;
	const int num_ims_each_rot = 22;
	const double move_per_ind = 1.5;
	const int start_moving_at_ind = 2;

	const int pause_at_ind = 0;
	const int pause_frames_bishop = 22;
	const int pause_frames_angle = 100;

	const int sum_weights_move = num_ims_each_seg + num_ims_each_bishop_update + num_ims_each_rot;

	int savedInd = image_ind;
	image_ind = 0;
	vector<Matrix3d> saved_rotations;
	
	//calculate all start rotations
	saved_rotations.push_back(glThreads[startThread]->getThread()->start_rot());
	for (int i=1; i < glThreads[startThread]->getThread()->num_pieces(); i++)
	{
		Matrix3d rot = glThreads[startThread]->getThread()->bishop_at_ind(i);
		rot = Eigen::AngleAxisd(glThreads[startThread]->getThread()->angle_at_ind(i-1), rot.col(0))*rot;
		saved_rotations.push_back(rot);
	}




	for (int piece_ind=0; piece_ind < glThreads[startThread]->getThread()->num_pieces()-1; piece_ind++)
	{
		Matrix3d start_rotation = glThreads[startThread]->getThread()->material_at_ind(piece_ind);
		Eigen::Quaterniond start_quat(start_rotation);
		Vector3d start_loc = glThreads[startThread]->getThread()->vertex_at_ind(piece_ind);

		//double ang_at_ind = glThreads[startThread]->getThread()->angle_at_ind(piece_ind);
		//Vector3d edge_at_next_ind = glThreads[startThread]->getThread()->edge_at_ind(piece_ind+1).normalized();
		//Matrix3d end_rotation_noAng = Eigen::AngleAxisd(edge_at_next_ind, ang_at_ind) *glThreads[startThread]->getThread()->bishop_at_ind(piece_ind+1);
		
		Matrix3d end_rotation_noAng = saved_rotations[piece_ind+1];
		Eigen::Quaterniond end_quat_noAng(end_rotation_noAng);

		Matrix3d end_rotation = glThreads[startThread]->getThread()->material_at_ind(piece_ind+1);
		//Matrix3d end_rotation = saved_rotations[piece_ind+1];
		Eigen::Quaterniond end_quat(end_rotation);
		Vector3d end_loc = glThreads[startThread]->getThread()->vertex_at_ind(piece_ind+1);

		/* PROPOGATE FRAME ACROSS EDGE */

		for (int im_ind=0; im_ind < num_ims_each_seg; im_ind++)
		{

		Matrix3d curr_rotation = start_rotation;
		Vector3d curr_loc = start_loc + (end_loc-start_loc)*((double)im_ind/(double)(num_ims_each_seg-1));

		if (piece_ind> start_moving_at_ind)
		{
				rotate_frame[1] -= move_per_ind/(double)sum_weights_move;
  			glTranslatef (-17.0,18.0,-85.0);
				glRotatef (-move_per_ind/(double)sum_weights_move, 1.0, 0.0, 0.0);
  			glTranslatef (17.0,-18.0,85.0);
		}

		glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glPushMatrix ();
		saveImages_setLoc();

		//glThreads[startThread]->minimize_energy();

		saveImages_setDefaultColor();

		glThreads[startThread]->addTexture_stripe();
		glThreads[startThread]->DrawThread();
		glThreads[startThread]->DrawAxes();
		glThreads[startThread]->DrawSpheresAtLinks();
		int draw_each = 1;
		for (int i=draw_each; i < glThreads[startThread]->getThread()->num_pieces()-draw_each; i+=draw_each)
		{
			glThreads[startThread]->DrawAxesAtPoint(i, material, true); 
		}

		glThreads[startThread]->DrawAxesAtPoint(curr_loc, curr_rotation, bishop, false);

		if (piece_ind > 0)
		{
			string str = "Twist";
			DrawTextAtTop(str);
		}

		save_opengl_image(IMG_SAVE_TWISTING_PROPOGATE);

		glPopMatrix ();
		glutSwapBuffers ();

		}

		if( piece_ind == glThreads[startThread]->getThread()->num_pieces()-2)
			break;


		/*************************************
		 * Pause if we need to *
		 *************************************/ 
		if (piece_ind == pause_at_ind)
		{

		Matrix3d curr_rotation = start_rotation;
		Vector3d curr_loc = end_loc;

			for (int im_ind=0; im_ind < pause_frames_bishop; im_ind++)
			{

				if (piece_ind> start_moving_at_ind)
				{
					rotate_frame[1] -= move_per_ind/(double)sum_weights_move;
					glTranslatef (-17.0,18.0,-85.0);
					glRotatef (-move_per_ind/(double)sum_weights_move, 1.0, 0.0, 0.0);
					glTranslatef (17.0,-18.0,85.0);
				}

				glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
				glPushMatrix ();
				saveImages_setLoc();

				//glThreads[startThread]->minimize_energy();

				saveImages_setDefaultColor();

				glThreads[startThread]->addTexture_stripe();
				glThreads[startThread]->DrawThread();
				glThreads[startThread]->DrawAxes();
				glThreads[startThread]->DrawSpheresAtLinks();
				int draw_each = 1;
				for (int i=draw_each; i < glThreads[startThread]->getThread()->num_pieces()-draw_each; i+=draw_each)
				{
					glThreads[startThread]->DrawAxesAtPoint(i, material, true); 
				}

				glThreads[startThread]->DrawAxesAtPoint(curr_loc, curr_rotation, bishop, false);

				if (piece_ind > 0)
				{
					string str = "Twist";
					DrawTextAtTop(str);
				}
				save_opengl_image(IMG_SAVE_TWISTING_PROPOGATE);

				glPopMatrix ();
				glutSwapBuffers ();
			}

		}




		/* DO BISHOP FRAME UPDATE */
		for (int im_ind=0; im_ind < num_ims_each_bishop_update; im_ind++)
		{
		Eigen::Quaterniond curr_quat = start_quat.slerp(((double)im_ind/(double)(num_ims_each_bishop_update-1)), end_quat_noAng);
		Matrix3d curr_rotation(curr_quat);
		Vector3d curr_loc = end_loc;

		if (piece_ind>  start_moving_at_ind)
		{
				rotate_frame[1] -= move_per_ind/(double)sum_weights_move;
  			glTranslatef (-17.0,18.0,-85.0);
				glRotatef (-move_per_ind/(double)sum_weights_move, 1.0, 0.0, 0.0);
  			glTranslatef (17.0,-18.0,85.0);
		}

		glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glPushMatrix ();
		saveImages_setLoc();

		//glThreads[startThread]->minimize_energy();

		saveImages_setDefaultColor();

		glThreads[startThread]->addTexture_stripe();
		glThreads[startThread]->DrawThread();
		glThreads[startThread]->DrawAxes();
		glThreads[startThread]->DrawSpheresAtLinks();
		int draw_each = 1;
		for (int i=draw_each; i < glThreads[startThread]->getThread()->num_pieces()-draw_each; i+=draw_each)
		{
			glThreads[startThread]->DrawAxesAtPoint(i, material, true); 
		}

		glThreads[startThread]->DrawAxesAtPoint(curr_loc, curr_rotation, bishop, false);

		if (piece_ind > 0)
		{
			string str = "Twist";
			DrawTextAtTop(str);
		}

		save_opengl_image(IMG_SAVE_TWISTING_PROPOGATE);

		glPopMatrix ();
		glutSwapBuffers ();

		}


		/*************************************
		 * Pause if we need to *
		 *************************************/ 
		if (piece_ind == pause_at_ind)
		{

		Matrix3d curr_rotation = end_rotation_noAng;
		Vector3d curr_loc = end_loc;

			for (int im_ind=0; im_ind < pause_frames_angle; im_ind++)
			{

				if (piece_ind> start_moving_at_ind)
				{
					rotate_frame[1] -= move_per_ind/(double)sum_weights_move;
					glTranslatef (-17.0,18.0,-85.0);
					glRotatef (-move_per_ind/(double)sum_weights_move, 1.0, 0.0, 0.0);
					glTranslatef (17.0,-18.0,85.0);
				}

				glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
				glPushMatrix ();
				saveImages_setLoc();

				//glThreads[startThread]->minimize_energy();

				saveImages_setDefaultColor();

				glThreads[startThread]->addTexture_stripe();
				glThreads[startThread]->DrawThread();
				glThreads[startThread]->DrawAxes();
				glThreads[startThread]->DrawSpheresAtLinks();
				int draw_each = 1;
				for (int i=draw_each; i < glThreads[startThread]->getThread()->num_pieces()-draw_each; i+=draw_each)
				{
					glThreads[startThread]->DrawAxesAtPoint(i, material, true); 
				}

				glThreads[startThread]->DrawAxesAtPoint(curr_loc, curr_rotation, bishop, false);

				glColor3f(0.8, 0.1, 0.0);
				Vector3d dir1, dir2;
				dir1 = end_rotation_noAng.col(2).normalized();
				dir2 = end_rotation.col(2).normalized();
				glThreads[startThread]->DrawArcBetweenMats(end_loc, dir1, dir2, 3.5, 4.5);
				dir1 = -end_rotation_noAng.col(1).normalized();
				dir2 = -end_rotation.col(1).normalized();
				glThreads[startThread]->DrawArcBetweenMats(end_loc, dir1, dir2, 3.5, 4.5);


	
				string str = "Twist";
				DrawTextAtTop(str);


				save_opengl_image(IMG_SAVE_TWISTING_PROPOGATE);

				glPopMatrix ();
				glutSwapBuffers ();
			}

		}





		/* GET TO NEW TWIST*/

		for (int im_ind=0; im_ind < num_ims_each_rot; im_ind++)
		{
		Eigen::Quaterniond curr_quat = end_quat_noAng.slerp(((double)im_ind/(double)(num_ims_each_bishop_update-1)), end_quat);
		Matrix3d curr_rotation(curr_quat);
		Vector3d curr_loc = end_loc;

		if (piece_ind>  start_moving_at_ind)
		{
				rotate_frame[1] -= move_per_ind/(double)sum_weights_move;
  			glTranslatef (-17.0,18.0,-85.0);
				glRotatef (-move_per_ind/(double)sum_weights_move, 1.0, 0.0, 0.0);
  			glTranslatef (17.0,-18.0,85.0);
		}

		glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glPushMatrix ();
		saveImages_setLoc();

		//glThreads[startThread]->minimize_energy();

		saveImages_setDefaultColor();

		glThreads[startThread]->addTexture_stripe();
		glThreads[startThread]->DrawThread();
		glThreads[startThread]->DrawAxes();
		glThreads[startThread]->DrawSpheresAtLinks();
		int draw_each = 1;
		for (int i=draw_each; i < glThreads[startThread]->getThread()->num_pieces()-draw_each; i+=draw_each)
		{
			glThreads[startThread]->DrawAxesAtPoint(i, material, true); 
		}

		glThreads[startThread]->DrawAxesAtPoint(curr_loc, curr_rotation, bishop, false);

		//if (piece_ind == pause_at_ind && im_ind != num_ims_each_rot-1)
		if (im_ind != num_ims_each_rot-1)
		{
			glColor3f(0.8, 0.1, 0.0);
			Vector3d dir1, dir2;
			dir1 = curr_rotation.col(2).normalized();
			dir2 = end_rotation.col(2).normalized();
			glThreads[startThread]->DrawArcBetweenMats(end_loc, dir1, dir2, 3.5, 4.5);
			dir1 = -curr_rotation.col(1).normalized();
			dir2 = -end_rotation.col(1).normalized();
			glThreads[startThread]->DrawArcBetweenMats(end_loc, dir1, dir2, 3.5, 4.5);

		}


			string str = "Twist";
			DrawTextAtTop(str);



		save_opengl_image(IMG_SAVE_TWISTING_PROPOGATE);

		glPopMatrix ();
		glutSwapBuffers ();

		}

	}

	for (int i=0; i < 10; i++)
	{
		save_opengl_image(IMG_SAVE_TWISTING_PROPOGATE);
		glutSwapBuffers ();
	}


	image_ind = savedInd;
}

void saveImages_movement()
{
	int savedInd = image_ind;
	image_ind = 0;

	vector<Frame_Motion> motions;
	vector<int> num_ims_per;

	Frame_Motion tocpy;
	tocpy._pos_movement << -0.15, -0.05, 0.0;
	tocpy._frame_rotation = Matrix3d::Identity();
	motions.push_back(tocpy);
	num_ims_per.push_back(60);


	tocpy._pos_movement.setZero();
	tocpy._frame_rotation = Eigen::AngleAxisd(0.11, Vector3d::UnitX());
	motions.push_back(tocpy);
	num_ims_per.push_back(60);


	for (int mot_ind=0; mot_ind < motions.size(); mot_ind++)
	{
		for (int im_ind = 0;im_ind < num_ims_per[mot_ind]; im_ind++)
		{

			glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			glPushMatrix ();
			saveImages_setLoc();


			saveImages_setDefaultColor();

			Matrix3d curr_end_rot = glThreads[startThread]->getThread()->end_rot();
			Vector3d curr_end_pos = glThreads[startThread]->getThread()->end_pos();
			motions[mot_ind].applyMotion(curr_end_pos, curr_end_rot);

			glThreads[startThread]->getThread()->set_end_constraint(curr_end_pos, curr_end_rot);
			glThreads[startThread]->getThread()->minimize_energy(80000, 1e-8, MAX_MOVEMENT_VERTICES, 1e-10);


			/* plain thread, not minimized */
			glThreads[startThread]->addTexture_stripe();
			glThreads[startThread]->DrawThread();
			glThreads[startThread]->DrawAxes();
			glThreads[startThread]->DrawSpheresAtLinks();
			int draw_each = 1;
			for (int i=draw_each; i < glThreads[startThread]->getThread()->num_pieces()-draw_each; i+=draw_each)
			{
				glThreads[startThread]->DrawAxesAtPoint(i, material, true); 
			}


			save_opengl_image(IMG_SAVE_MOVEMENT);

			glPopMatrix ();
			glutSwapBuffers ();


		}
	}

	for (int i=0; i < 10; i++)
	{
		save_opengl_image(IMG_SAVE_MOVEMENT);
		glutSwapBuffers ();
	}




	image_ind = savedInd;
}


void saveImages_setLoc()
{
	/*
  glTranslatef (-22.0,18.0,-85.0);
	rotate_frame[0] = -45.0;
	rotate_frame[1] = -75.0;
	*/
  glTranslatef (-17.0,18.0,-85.0);
  glRotatef (rotate_frame[1], 1.0, 0.0, 0.0);
  glRotatef (rotate_frame[0], 0.0, 0.0, 1.0);
}

void saveImages_setDefaultColor()
{
  glThreads[startThread]->updateThreadPoints();
  Vector3d display_start_pos = glThreads[startThread]->getStartPosition();

	glThreads[startThread]->display_start_pos = display_start_pos;
	glColor4f (1.0, 1.0, 1.0, 0.9);
}


void InitThread(int argc, char* argv[])
{


  for (int i=0; i < NUMBER_THREADS; i++)
  {
    glThreads[i] = new GLThread();
    glThreads[i]->updateThreadPoints();
  }

/*
	for (int i=1; i < NUMBER_THREADS; i++)
	{
		glThreads[i]->to_set_twist = twist_params[i];
		glThreads[i]->to_set_grav = grav_params[i];
#ifdef ISOTROPIC
		glThreads[i]->to_set_bend = bend_params[i];
#else
		glThreads[i]->to_set_B = bend_params[i];
#endif
	}

*/
}


void DrawAxes()
{
	return;

  //Draw Axes
  glBegin(GL_LINES);
  glEnable(GL_LINE_SMOOTH);
  glColor3d(1.0, 0.0, 0.0); //red
  glVertex3f(0.0f, 0.0f, 0.0f); //x
  glVertex3f(10.0f, 0.0f, 0.0f);
  glColor3d(0.0, 1.0, 0.0); //green
  glVertex3f(0.0f, 0.0f, 0.0f); //y
  glVertex3f(0.0f, 10.0f, 0.0f);
  glColor3d(0.0, 0.0, 1.0); //blue
  glVertex3f(0.0f, 0.0f, 0.0f); //z
  glVertex3f(0.0f, 0.0f, 10.0f);

  //label axes
  void * font = GLUT_BITMAP_HELVETICA_18;
  glColor3d(1.0, 0.0, 0.0); //red
  glRasterPos3i(20.0, 0.0, -1.0);
  glutBitmapCharacter(font, 'X');
  glColor3d(0.0, 1.0, 0.0); //red
  glRasterPos3i(0.0, 20.0, -1.0);
  glutBitmapCharacter(font, 'Y');
  glColor3d(0.0, 0.0, 1.0); //red
  glRasterPos3i(-1.0, 0.0, 20.0);
  glutBitmapCharacter(font, 'Z');
  glEnd();
	
}


void InitGLUT(int argc, char * argv[]) {
  /* initialize glut */
  glutInit (&argc, argv); //can i do that?
  glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(GL_WIDTH_PIX,GL_HEIGHT_PIX);
  glutCreateWindow ("Thread");
  glutDisplayFunc (DrawStuff);
  glutMotionFunc (MouseMotion);
  glutMouseFunc (processMouse);
  glutKeyboardFunc(processNormalKeys);
  glutSpecialFunc(processSpecialKeys);
  glutKeyboardUpFunc(processKeyUp);

  /* create popup menu */
  glutCreateMenu (JoinStyle);
  glutAddMenuEntry ("Exit", 99);
  glutAttachMenu (GLUT_MIDDLE_BUTTON);

  /* initialize GL */
  glClearDepth (1.0);
  glEnable (GL_DEPTH_TEST);
  glClearColor (0.0, 0.0, 0.0, 0.0);
  glShadeModel (GL_SMOOTH);

  glMatrixMode (GL_PROJECTION);
  /* roughly, measured in centimeters */
  glFrustum (-10.0, 10.0, -10.0, 10.0, 30.0, 500.0);
  glMatrixMode(GL_MODELVIEW);

	glLineWidth (1.5);

}


void InitLights() {
  /* initialize lighting */
  glLightfv (GL_LIGHT0, GL_POSITION, lightOnePosition);
  glLightfv (GL_LIGHT0, GL_DIFFUSE, lightOneColor);
  glEnable (GL_LIGHT0);
  glLightfv (GL_LIGHT1, GL_POSITION, lightTwoPosition);
  glLightfv (GL_LIGHT1, GL_DIFFUSE, lightTwoColor);
  glEnable (GL_LIGHT1);
  glLightfv (GL_LIGHT2, GL_POSITION, lightThreePosition);
  glLightfv (GL_LIGHT2, GL_DIFFUSE, lightThreeColor);
  glEnable (GL_LIGHT2);
  glLightfv (GL_LIGHT3, GL_POSITION, lightFourPosition);
  glLightfv (GL_LIGHT3, GL_DIFFUSE, lightFourColor);
  glEnable (GL_LIGHT3);
  glLightfv (GL_LIGHT4, GL_POSITION, lightFivePosition);
  glLightfv (GL_LIGHT4, GL_DIFFUSE, lightFiveColor);
  //glEnable (GL_LIGHT4);
  glLightfv (GL_LIGHT5, GL_POSITION, lightSixPosition);
  glLightfv (GL_LIGHT5, GL_DIFFUSE, lightSixColor);
  //glEnable (GL_LIGHT5);
  glLightfv (GL_LIGHT6, GL_POSITION, lightSevenPosition);
  glLightfv (GL_LIGHT6, GL_DIFFUSE, lightSevenColor);
  //glEnable (GL_LIGHT6);
  glLightfv (GL_LIGHT7, GL_POSITION, lightEightPosition);
  glLightfv (GL_LIGHT7, GL_DIFFUSE, lightEightColor);
  //glEnable (GL_LIGHT7);
  glEnable (GL_LIGHTING);
  glColorMaterial (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
  glEnable (GL_COLOR_MATERIAL);
}




void save_opengl_image(const char* img_base_name)
{
  //playback and save images
  Mat img(GL_HEIGHT_PIX, GL_WIDTH_PIX, CV_8UC3);
  vector<Mat> img_planes;
  split(img, img_planes);

  uchar tmp_data[3][GL_WIDTH_PIX*GL_HEIGHT_PIX];

  GLenum read_formats[3];
  read_formats[0] = GL_BLUE;
  read_formats[1] = GL_GREEN;
  read_formats[2] = GL_RED;

  for (int i=0; i < 3; i++)
  {
    glReadPixels(0, 0, GL_WIDTH_PIX, GL_HEIGHT_PIX, read_formats[i], GL_UNSIGNED_BYTE, tmp_data[i]);
    img_planes[i].data = tmp_data[i];
  }


  merge(img_planes, img);
  flip(img, img, 0);

  char im_name[256];
  sprintf(im_name, "%s%d.png", img_base_name, image_ind+1);
	image_ind++;
  imwrite(im_name, img);
  waitKey(1);
  //sleep(0);
}




void update_all_threads()
{
	/*
	for (int i=0; i < NUMBER_THREADS; i++)
	{
		glThreads[i]->setThread((new Thread(all_threads[thread_ind])));
	}*/


	/*
  vector<Vector3d> pts;
  vector<double> angls;
	glThreads[0]->getThread()->get_thread_data(pts, angls);

	glThreads[1]->getThread()->set_twist_and_minimize(-2*M_PI, pts);
	glThreads[2]->getThread()->set_twist_and_minimize(0, pts);
	glThreads[3]->getThread()->set_twist_and_minimize(2*M_PI, pts);
*/

}
