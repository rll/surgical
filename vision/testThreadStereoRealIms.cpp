#include <stdlib.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/gle.h>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry> 
#include <math.h>
#include "ThreadStereo_Optimization.h"
#include "../threadenergy/thread_minenergy.h"
#include "../threadenergy/globals_thread_param_estimation.h"
#include "../threadenergy/trajectory_recorder.h"



// import most common Eigen types 
USING_PART_OF_NAMESPACE_EIGEN


void InitStuff();
void DrawStuff();
void updateIms(Point2i& start_pt_center);
int num_ims_seen=0;
//int num_ims_before_write=136;

#define NUM_PTS 200
#define THREAD_RADII 1.0
#define MOVE_POS_CONST 5.0

enum key_code {NONE, MOVEPOS, MOVETAN};


float lastx_L=0;
float lasty_L=0;
float lastx_R=0;
float lasty_R=0;

float rotate_frame[2];
float move_end[2];
float tangent_end[2];


Trajectory_Recorder traj_recorder;

Thread* thread;
MatrixXd points(NUM_PTS,3);

ThreadStereo* threadStereo;
int curr_im_ind = 1;
vector<glline_draw_params> vision_draw;


double radii[NUM_PTS];
int pressed_mouse_button;

Vector3d positions[2];
Vector3d tangents[2];

key_code key_pressed;


int num=2;
double curvature [2] = {1.0, 2.0};
double torsion [2] = {0.5, 0.0};
double length [2] = {100.0, 100.0};


/* set up a light */
GLfloat lightOnePosition[] = {140.0, 0.0, 200.0, 0.0};
GLfloat lightOneColor[] = {0.99, 0.99, 0.99, 1.0}; 

GLfloat lightTwoPosition[] = {-140.0, 0.0, 200.0, 0.0};
GLfloat lightTwoColor[] = {0.99, 0.99, 0.99, 1.0}; 

GLfloat lightThreePosition[] = {140.0, 0.0, -200.0, 0.0};
GLfloat lightThreeColor[] = {0.99, 0.99, 0.99, 1.0}; 

GLfloat lightFourPosition[] = {-140.0, 0.0, -200.0, 0.0};
GLfloat lightFourColor[] = {0.99, 0.99, 0.99, 1.0}; 



void processLeft(int x, int y)
{
  rotate_frame[0] += x-lastx_L;
  rotate_frame[1] += lasty_L-y;

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
    tangent_end[0] += x-lastx_R;
    tangent_end[1] += lasty_R-y;
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


void processNormalKeys(unsigned char key, int x, int y)
{
	if (key == 't') 
	  key_pressed = MOVETAN;
  else if (key == 'm')
    key_pressed = MOVEPOS;
  else if (key == 'q')
  {
    delete thread;
    thread = new Thread(curvature, torsion, length, 2, positions, tangents);
    thread->minimize_energy();
    glutPostRedisplay ();
  } else if (key == '1'){
    delete thread;
    length[0] = length[1] = 100.0;
    positions[0](0) = 50.0;
    positions[0](1) = 0.0;
    positions[0](2) = 0.0;
    positions[1](0) = -40.0;
    positions[1](1) = 80.0;
    positions[1](2) = 80.0;
    
    tangents[0](0) = 0.0;
    tangents[0](1) = 0.0;
    tangents[0](2) = 1.0;
    tangents[1](0) = 1.0;
    tangents[1](1) = 0.0;
    tangents[1](2) = 1.0;
    thread = new Thread(curvature, torsion, length, 2, positions, tangents);
    thread->minimize_energy();
   // updateIms();
    glutPostRedisplay ();
  } else if (key == '2') {
    length[0] = length[1] = 150.0;
    positions[0](0) = -10.0;
    positions[0](1) = 140.0;
    positions[0](2) = 40.0;
    positions[1](0) = 70.0;
    positions[1](1) = 0.0;
    positions[1](2) = -20.0;

    tangents[0](0) = 0.0;
    tangents[0](1) = 0.0;
    tangents[0](2) = 1.0;
    tangents[1](0) = 1.0;
    tangents[1](1) = 0.0;
    tangents[1](2) = 0.5;
    thread = new Thread(curvature, torsion, length, 2, positions, tangents);
    thread->minimize_energy();
   // updateIms();
    glutPostRedisplay ();
  } else if (key == '3') {
    length[0] = length[1] = 190.0;
    positions[0](0) = 0.0;
    positions[0](1) = 120.0;
    positions[0](2) = -40.0;
    positions[1](0) = 80.0;
    positions[1](1) = -30.0;
    positions[1](2) = 20.0;

    tangents[0](0) = 0.0;
    tangents[0](1) = 1.0;
    tangents[0](2) = 0.0;
    tangents[1](0) = 1.0;
    tangents[1](1) = 0.0;
    tangents[1](2) = -1.0;
    thread = new Thread(curvature, torsion, length, 2, positions, tangents);
    thread->minimize_energy();
   // updateIms();
    glutPostRedisplay ();
  } else if (key == 27)
    exit(0);

  lastx_R = x;
  lasty_R = y;

}

void processKeyUp(unsigned char key, int x, int y)
{
  key_pressed = NONE;
  move_end[0] = move_end[1] = tangent_end[0] = tangent_end[1] = 0.0;
}



 void JoinStyle (int msg) 
{
	exit (0);
}



int main (int argc, char * argv[]) 
{
/*  int me, numprocs;
  MPI_Init(&argc, &argv);
  MPI_Comm_size(MPI_COMM_WORLD, &numprocs);
  std::cout << "num processors: " << numprocs << std::endl;
  MPI_Comm_rank(MPI_COMM_WORLD, &me);*/

  printf("Instructions:\n"
      "Hold down the left mouse button to rotate image: \n"
      "\n"
      "Hold 'm' while holding down the right mouse to move the end\n"
      "Hold 't' while holding down the right mouse to rotate the tangent \n"
      "\n"
      "Press 'q' to recalculate the thread from initial\n"
      );


	/* initialize glut */
	glutInit (&argc, argv); //can i do that?
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(900,900);
	glutCreateWindow ("Thread");
	glutDisplayFunc (DrawStuff);
	glutMotionFunc (MouseMotion);
  glutMouseFunc (processMouse);
  glutKeyboardFunc(processNormalKeys);
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
	glFrustum (-30.0, 30.0, -30.0, 30.0, 50.0, 500.0);
	glMatrixMode(GL_MODELVIEW);


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
	glEnable (GL_LIGHTING);
	glColorMaterial (GL_FRONT_AND_BACK, GL_DIFFUSE);
	glEnable (GL_COLOR_MATERIAL);

	InitStuff ();

  positions[0](0) = 50.0;
	positions[0](1) = 0.0;
	positions[0](2) = 0.0;
	positions[1](0) = -40.0;
	positions[1](1) = 80.0;
	positions[1](2) = 80.0;
	tangents[0](0) = 0.0;
	tangents[0](1) = 0.0;
	tangents[0](2) = 1.0;
	tangents[1](0) = 1.0;
	tangents[1](1) = 0.0;
	tangents[1](2) = 1.0;
	
	for (int i=0; i < NUM_PTS; i++)
	{
		radii[i]=THREAD_RADII;
	}

  //initialize images
  threadStereo = new ThreadStereo();
  threadStereo->initializeOnClicks();
  //threadStereo->setInitPtCenterImFromClicks();
  threadStereo->setInitPtFromClicks();
  //Point3f initPtSaved(60.9077, 33.2329, 60.1416); //nylon
  //Point3f initPtSaved(58.1533, 32.9361, 60.5692); //longer
  //Point3f initPtSaved(61.1455, 33.3064, 59.7723); //purple
  //threadStereo->setInitPt(initPtSaved);

  //Point2i start;
  //updateIms(start);
  vision_draw.resize(0);

  Vector3d newPositions[2];
  Vector3d newTangents[2];

  if (threadStereo->optimizeThread(vision_draw))
    threadStereo->getEndsAndTans(newPositions, newTangents);
  else {
    std::cout << "vision failed. there is no hope..." << std::endl;
    exit(0);
  }
  num_ims_seen++;

  double length_tot = threadStereo->getLength();
  if (image_record_mode == SAVE_POINTS)
  {
    MatrixXd visPoints(NUM_PTS_TO_SAVE,3);
    threadStereo->getThreadPoints(visPoints);

    traj_recorder.setLength(length_tot);
    traj_recorder.add_thread_to_list(visPoints, newPositions, newTangents);
  }

  positions[0] = newPositions[0];
  positions[1] = newPositions[1];
  tangents[0] = newTangents[0];
  tangents[1] = newTangents[1];
  
  length[0] = length[1] = length_tot/2.0;


  thread = new Thread(curvature, torsion, length, num, positions, tangents); 
  

  //thread = threadStereo->equivalentThreadMinEnergy();

//  thread->minimize_energy();
  MatrixXd initPoints(NUM_PTS_TO_SAVE,3);
  threadStereo->getThreadPoints(initPoints);
  thread->optimizeManyPoints_startAndEnd_MyParams(initPoints, 2, positions, tangents);
  thread->minimize_energy_fixedPieces();
  MatrixXd threadPoints(NUM_PTS_TO_SAVE,3);
  thread->getPoints(threadPoints);

  Vector3d posSimulator[2];
  thread->getWantedStartPosition(posSimulator[0]);
  thread->getWantedEndPosition(posSimulator[1]);

  //std::cout << "vision wanted: \n" << positions[0] << "\n" << positions[1] << std::endl;
  //std::cout << "simulator wanted: \n" << posSimulator[0] << "\n" << posSimulator[1] << std::endl;


  std::cout << "points orig     points new      norm\n";
  for (int i=0; i < initPoints.rows(); i++)
  {
    std::cout << (initPoints.block(i,0,1,3)) << "               " << (threadPoints.block(i,0,1,3)) << "      " << (initPoints.block(i,0,1,3) - threadPoints.block(i,0,1,3)).squaredNorm() << std::endl;
  }

 //     std::cout << "points orig \n" << savedIntermediateThreads[currIndIntermediateThread+1] << std::endl;
  //    std::cout << "points new \n" << ptsNewPiece << std::endl;
  double dist = avgDistBetweenPoints(initPoints, threadPoints);
  std::cout << "this dist: " << dist << std::endl;

 


  //add thread to display on images
  MatrixXd pointsForIms(32,3);
  thread->getPoints(pointsForIms);
  Scalar colorThreadMinimizerOnIms(127,127,0);
  //threadStereo->addThreadPointsToDebugImages(pointsForIms, colorThreadMinimizerOnIms);


  //display
  threadStereo->display();





  glutMainLoop ();
	//   return 0;             /* ANSI C requires main to return int. */
}


void InitStuff (void) 
{
	glEnable(GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	gleSetJoinStyle (TUBE_NORM_PATH_EDGE | TUBE_JN_ANGLE );
  rotate_frame[0] = 0.0;
  rotate_frame[1] = -111.0;
}

/* draw the helix shape */
void DrawStuff (void) 
{
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glColor3f (0.8, 0.3, 0.6);

	glPushMatrix ();

	/* set up some matrices so that the object spins with the mouse */
	glTranslatef (0.0,0.0,-300.0);
  glRotatef (rotate_frame[1], 1.0, 0.0, 0.0);
  glRotatef (rotate_frame[0], 0.0, 0.0, 1.0);
  


  //change thread, if necessary
  if (move_end[0] != 0.0 || move_end[1] != 0.0 || tangent_end[0] != 0.0 || tangent_end[1] != 0.0)
  {
    GLdouble model_view[16];
    glGetDoublev(GL_MODELVIEW_MATRIX, model_view);

    GLdouble projection[16];
    glGetDoublev(GL_PROJECTION_MATRIX, projection);

    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

    
    double winX, winY, winZ;

    //change end positions
    //gluProject(positions[1](0), positions[1](1), positions[1](2), model_view, projection, viewport, &winX, &winY, &winZ);
    winX += move_end[0];
    winY += move_end[1];
    move_end[0] = 0.0;
    move_end[1] = 0.0;
    //gluUnProject(winX, winY, winZ, model_view, projection, viewport, &positions[1](0), &positions[1](1), &positions[1](2));
//    std::cout << "X: " << positions[1](0) << " Y: " << positions[1](1) << " Z: " << positions[1](2) << std::endl; 

    //change tangents
    //gluProject(tangents[1](0), tangents[1](1), tangents[1](2), model_view, projection, viewport, &winX, &winY, &winZ);
    winX += tangent_end[0];
    winY += tangent_end[1];
    tangent_end[0] = 0.0;
    tangent_end[1] = 0.0;
    //gluUnProject(winX, winY, winZ, model_view, projection, viewport, &tangents[1](0), &tangents[1](1), &tangents[1](2));
    tangents[1].normalize();

    //Point2i start;
    //updateIms(start);
    vision_draw.resize(0);
    //threadStereo->setInitPtCenterImFromClicks();

    Vector3d newPositions[2];
    Vector3d newTangents[2];

    if (threadStereo->optimizeThread(vision_draw))
      threadStereo->getEndsAndTans(newPositions, newTangents);

    num_ims_seen++;
    if (image_record_mode == SAVE_POINTS)
    {
      MatrixXd visPoints(NUM_PTS_TO_SAVE,3);
      threadStereo->getThreadPoints(visPoints);

      double length_tot = threadStereo->getLength();
      traj_recorder.add_thread_to_list(visPoints, newPositions, newTangents);
      if (num_ims_seen == num_ims_before_write)
        traj_recorder.write_threads_to_file();
    }
    if (traj_mode == RECORD)
    {
      traj_recorder.add_motion_to_list(positions[1], newPositions[1], tangents[1], newTangents[1]);
      if (num_ims_seen == num_ims_before_write)
        traj_recorder.write_motions_to_file();
    }

    positions[0] = newPositions[0];
    positions[1] = newPositions[1];
    tangents[0] = newTangents[0];
    tangents[1] = newTangents[1];

    

    //change thread
    thread->setConstraints(positions, tangents);
    //thread->upsampleAndOptimize_minLength(0.065);
    //thread->minimize_energy_fixedPieces();

    MatrixXd initPoints(NUM_PTS_TO_SAVE,3);
    threadStereo->getThreadPoints(initPoints);
    //thread->optimizeManyPoints_startAndEnd_MyParams(initPoints, 2, positions, tangents);
    //thread->minimize_energy_fixedPieces();
    MatrixXd threadPoints(NUM_PTS_TO_SAVE,3);
    thread->getPoints(threadPoints);

    Vector3d posSimulator[2];
    thread->getWantedStartPosition(posSimulator[0]);
    thread->getWantedEndPosition(posSimulator[1]);

    std::cout << "vision wanted: \n" << positions[0] << "\n" << positions[1] << std::endl;
    std::cout << "simulator wanted: \n" << posSimulator[0] << "\n" << posSimulator[1] << std::endl;



    std::cout << "points orig     points new      norm\n";
    for (int i=0; i < initPoints.rows(); i++)
    {
      std::cout << (initPoints.block(i,0,1,3)) << "               " << (threadPoints.block(i,0,1,3)) << "      " << (initPoints.block(i,0,1,3) - threadPoints.block(i,0,1,3)).squaredNorm() << std::endl;
    }

    //     std::cout << "points orig \n" << savedIntermediateThreads[currIndIntermediateThread+1] << std::endl;
    //    std::cout << "points new \n" << ptsNewPiece << std::endl;
    double dist = avgDistBetweenPoints(initPoints, threadPoints);
    std::cout << "this dist: " << dist << std::endl;


    
    //delete thread;
    //thread = threadStereo->equivalentThreadMinEnergy();


    //add thread to display on images
    MatrixXd pointsForIms(33,3);
    thread->getPoints(pointsForIms);
    Scalar colorThreadMinimizerOnIms(127,127,0);
    //threadStereo->addThreadPointsToDebugImages(pointsForIms, colorThreadMinimizerOnIms);


    //display
    threadStereo->display();


    //std::cout << "objX: " <<objX << " objY: " << objY << " objZ: " << objZ << " winX: " << winX << " winY: " << winY << " winZ: " << winZ << std::endl;
  


  }


  //Draw Axes
	glBegin(GL_LINES);
	glEnable(GL_LINE_SMOOTH);
	glColor3d(1.0, 0.0, 0.0); //red
	glVertex3f(0.0f, 0.0f, 0.0f); //x
	glVertex3f(20.0f, 0.0f, 0.0f);
	glColor3d(0.0, 1.0, 0.0); //green
	glVertex3f(0.0f, 0.0f, 0.0f); //y
	glVertex3f(0.0f, 20.0f, 0.0f);
	glColor3d(0.0, 0.0, 1.0); //blue
	glVertex3f(0.0f, 0.0f, 0.0f); //z
	glVertex3f(0.0f, 0.0f, 20.0f);

	glColor3d(0.5, 0.5, 1.0); 
	glVertex3f(positions[1](0)-positions[0](0), positions[1](1)-positions[0](1), positions[1](2)-positions[0](2)); //z
	glVertex3f(positions[1](0)-positions[0](0)+tangents[1](0)*4.0, positions[1](1)-positions[0](1)+tangents[1](1)*4.0, positions[1](2)-positions[0](2)+tangents[1](2)*4.0); //z


	glEnd( );



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





  
  //Draw lines from vision
  for (int i=0; i < vision_draw.size(); i++)
  {
    glline_draw_params& currP = vision_draw[i];
    glBegin(GL_LINE_STRIP);
    glColor4f(currP.color[0], currP.color[1], currP.color[2], 1.0);
    for (int j=0; j < currP.size; j++)
    {
      glVertex3f(currP.vertices[j].x-positions[0](0), currP.vertices[j].y-positions[0](1), currP.vertices[j].z-positions[0](2));
    }
    glEnd();
    
  }



  //Draw Thread
  glColor4f (0.5, 0.5, 0.2, 0.5);

	thread->getPoints(points);
	double pts_cpy[NUM_PTS][3];

	for (int i=0; i < NUM_PTS; i++)
	{
		pts_cpy[i][0] = points(i,0)-(double)positions[0](0);
		pts_cpy[i][1] = points(i,1)-(double)positions[0](1);
		pts_cpy[i][2] = points(i,2)-(double)positions[0](2);
	}

	glePolyCone_c4f (NUM_PTS, pts_cpy, 0x0, radii);





	glPopMatrix ();

	glutSwapBuffers ();


  move_end[0] = 1;
  DrawStuff();

}








void updateIms(Point2i& start_pt_center)
{
  int num_pts = 20000;
  MatrixXd points_for_ims(num_pts,3);
	thread->getPoints(points_for_ims);

  Mat ims[3];
  for (int i=0; i < 3; i++){
    ims[i] = cv::Mat::zeros(threadStereo->myThread._frames[i].size(), CV_8UC3);
  }

  Point2i points2d[NUMCAMS];
  for (int i=0; i < num_pts; i++)
  {
    cv::Point3f currP(points_for_ims(i,0), points_for_ims(i,1), points_for_ims(i,2));
    threadStereo->myThread._cams->project3dPoint(currP, points2d);
    for (int j=0; j < NUMCAMS; j++)
    {
      if (threadStereo->myThread._captures[j]->inRange(points2d[j].y, points2d[j].x))
      {
        ims[j].at<Vec3b>(points2d[j].y, points2d[j].x)[0] = (unsigned char)255;
        ims[j].at<Vec3b>(points2d[j].y, points2d[j].x)[1] = (unsigned char)255;
        ims[j].at<Vec3b>(points2d[j].y, points2d[j].x)[2] = (unsigned char)255;
      }
    }

  }

  if (num_pts > 0)
  {
    cv::Point3f currP(points_for_ims(0,0), points_for_ims(0,1), points_for_ims(0,2));
    threadStereo->myThread._cams->project3dPoint(currP, points2d);
    start_pt_center = points2d[CENTER_IM_IND];
    std::cout << "start pt center: " << start_pt_center << std::endl;
  }



 /* imshow("1", ims[0]);
  imshow("2", ims[1]);
  imshow("3", ims[2]);
*/

  char im_name[256];
  for (int cam_ind = 0; cam_ind < NUMCAMS; cam_ind++)
  {
    sprintf(im_name, "./stereo_test/stereo_test%d-%d.tif", cam_ind+1, curr_im_ind);
    imwrite(im_name, ims[cam_ind]);
  }
  curr_im_ind++;


}

