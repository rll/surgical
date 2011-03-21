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
#include "../DiscreteRods/thread_discrete.h"
#include "thread_vision_discrete.h"

#define TRAJ_BASE_NAME "../DiscreteRods/LearnParams/config/suturenylon_processed_projected"
#define READ_THREADS true     /*comment this out to record threads */

#ifdef READ_THREADS
  #include "../DiscreteRods/trajectory_reader.h"
#else
  #include "../DiscreteRods/trajectory_recorder.h"
#endif



// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN


    void InitStuff();
void DrawStuff();
void addThreadDebugInfo();
void updateThreadPoints();
void initThread();
void initThread_closedPolygon();
void updateIms(cv::Point3f& start_pt);
void findThreadInIms();


#define NUM_PTS 500
#define THREAD_RADII 1.0
#define MOVE_POS_CONST 1.0
#define MOVE_TAN_CONST 0.2
#define ROTATE_TAN_CONST 0.2


enum key_code {NONE, MOVEPOS, MOVETAN, ROTATETAN};


float lastx_L=0;
float lasty_L=0;
float lastx_R=0;
float lasty_R=0;

float rotate_frame[2];
float move_end[2];
float tangent_end[2];
float tangent_rotation_end[2];

Thread* thread;
vector<Vector3d> points;
vector<double> twist_angles;
vector<Matrix3d> material_frames;



double radii[NUM_PTS];
int pressed_mouse_button;

Vector3d positions[2];
Vector3d tangents[2];
Matrix3d rotations[3];

cv::Point3f _start_pt;
int curr_im_ind = 1;
Thread_Vision thread_vision;

#ifdef READ_THREADS
Trajectory_Reader traj_reader(TRAJ_BASE_NAME);
vector<Thread> all_threads;
int thread_ind = 0;
#else
Trajectory_Recorder traj_recorder(TRAJ_BASE_NAME);
#endif

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



//CONTOUR STUFF
#define SCALE 1.0
#define CONTOUR(x,y) {					\
double ax, ay, alen;					\
    contour[i][0] = SCALE * (x);				\
    contour[i][1] = SCALE * (y);				\
if (i!=0) {						\
    ax = contour[i][0] - contour[i-1][0];		\
    ay = contour[i][1] - contour[i-1][1];		\
    alen = 1.0 / sqrt (ax*ax + ay*ay);		\
    ax *= alen;   ay *= alen;				\
    contour_norms [i-1][0] = ay;				\
    contour_norms [i-1][1] = -ax;				\
    }							\
    i++;							\
}

#define NUM_PTS_CONTOUR (25)

double contour[NUM_PTS_CONTOUR][2];
double contour_norms[NUM_PTS_CONTOUR][2];


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


                void processNormalKeys(unsigned char key, int x, int y)
                {
                    if (key == 't')
                        key_pressed = MOVETAN;
                    else if (key == 'm')
                        key_pressed = MOVEPOS;
                    else if (key == 'r')
                        key_pressed = ROTATETAN;
                    else if (key == 'q')
                    {
                        delete thread;
                        glutPostRedisplay ();
                        }  else if (key == 's') {
#ifdef READ_THREADS
                            if (thread_ind < all_threads.size()-1)
                            {
                                thread = &all_threads[thread_ind];
                                thread_ind++;
                                glutPostRedisplay();
                            }
#else
                            traj_recorder.add_thread_to_list(*thread);
#endif
                            glutPostRedisplay ();
                        } else if (key == 27) {
#ifndef READ_THREADS
                            traj_recorder.write_threads_to_file();
#endif
                            exit(0);
                        }

                        lastx_R = x;
                        lasty_R = y;

                    }

                    void processKeyUp(unsigned char key, int x, int y)
                    {
                        key_pressed = NONE;
                        move_end[0] = move_end[1] = tangent_end[0] = tangent_end[1] = tangent_rotation_end[0] = tangent_rotation_end[1] = 0.0;
                    }



                    void JoinStyle (int msg)
                    {
                        exit (0);
                    }

                    void init_contour (void)
                    {
                        int i;
                        double contour_scale_factor = 0.3;

#ifdef ISOTROPIC
   // outline of extrusion 
                        i=0;
                        CONTOUR (1.0 *contour_scale_factor, 1.0 *contour_scale_factor);
                        CONTOUR (1.0 *contour_scale_factor, 2.9 *contour_scale_factor);
                        CONTOUR (0.9 *contour_scale_factor, 3.0 *contour_scale_factor);
                        CONTOUR (-0.9*contour_scale_factor, 3.0 *contour_scale_factor);
                        CONTOUR (-1.0*contour_scale_factor, 2.9 *contour_scale_factor);

                        CONTOUR (-1.0*contour_scale_factor, 1.0 *contour_scale_factor);
                        CONTOUR (-2.9*contour_scale_factor, 1.0 *contour_scale_factor);
                        CONTOUR (-3.0*contour_scale_factor, 0.9 *contour_scale_factor);
                        CONTOUR (-3.0*contour_scale_factor, -0.9*contour_scale_factor);
                        CONTOUR (-2.9*contour_scale_factor, -1.0*contour_scale_factor);

                        CONTOUR (-1.0*contour_scale_factor, -1.0*contour_scale_factor);
                        CONTOUR (-1.0*contour_scale_factor, -2.9*contour_scale_factor);
                        CONTOUR (-0.9*contour_scale_factor, -3.0*contour_scale_factor);
                        CONTOUR (0.9 *contour_scale_factor, -3.0*contour_scale_factor);
                        CONTOUR (1.0 *contour_scale_factor, -2.9*contour_scale_factor);

                        CONTOUR (1.0 *contour_scale_factor, -1.0*contour_scale_factor);
                        CONTOUR (2.9 *contour_scale_factor, -1.0*contour_scale_factor);
                        CONTOUR (3.0 *contour_scale_factor, -0.9*contour_scale_factor);
                        CONTOUR (3.0 *contour_scale_factor, 0.9 *contour_scale_factor);
                        CONTOUR (2.9 *contour_scale_factor, 1.0 *contour_scale_factor);

                        CONTOUR (1.0 *contour_scale_factor, 1.0 *contour_scale_factor);   // repeat so that last normal is computed 

#else
   // outline of extrusion 
                        i=0;
                        CONTOUR (1.0, 0.0);
                        CONTOUR (1.0, 0.5);
                        CONTOUR (1.0, 1.0);
                        CONTOUR (1.0, 2.0);
                        CONTOUR (1.0, 2.9);
                        CONTOUR (0.9, 3.0);
                        CONTOUR (0.0, 3.0);
                        CONTOUR (-0.9, 3.0);
                        CONTOUR (-1.0, 2.9);

                        CONTOUR (-1.0, 2.0);
                        CONTOUR (-1.0, 1.0);
                        CONTOUR (-1.0, 0.5);
                        CONTOUR (-1.0, 0.0);
                        CONTOUR (-1.0, -0.5);
                        CONTOUR (-1.0, -1.0);
                        CONTOUR (-1.0, -2.0);
                        CONTOUR (-1.0, -2.9);
                        CONTOUR (-0.9, -3.0);
                        CONTOUR (0.0, -3.0);
                        CONTOUR (0.9, -3.0);
                        CONTOUR (1.0, -2.9);
                        CONTOUR (1.0, -2.0);
                        CONTOUR (1.0, -1.0);
                        CONTOUR (1.0, -0.5);

                        CONTOUR (1.0, 0.0);   // repeat so that last normal is computed 
#endif

                    }


                    int main (int argc, char * argv[])
                    {

                        srand(time(NULL));
                        srand((unsigned int)time((time_t *)NULL));

#ifdef READ_THREADS
                        traj_reader.read_threads_from_file();
                        all_threads = traj_reader.get_all_threads();
#endif



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
    "Press 'q' to quit\n"
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

#ifdef READ_THREADS
thread = &all_threads[thread_ind];
#else
initThread();
thread->minimize_energy();
#endif
updateThreadPoints();


updateIms(_start_pt);
thread_vision.set_max_length(98);
thread_vision.setInitPt(_start_pt);
thread_vision.optimizeThread();




for (int i=0; i < NUM_PTS; i++)
{
    radii[i]=THREAD_RADII;
}


glutMainLoop ();
    //   return 0;             /* ANSI C requires main to return int. */
}


void InitStuff (void)
{
    glEnable(GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //gleSetJoinStyle (TUBE_NORM_PATH_EDGE | TUBE_JN_ANGLE );
    rotate_frame[0] = 0.0;
    rotate_frame[1] = -111.0;

    int style;

  /* pick model-vertex-cylinder coords for texture mapping */
  //TextureStyle (509);

  /* configure the pipeline */
    style = TUBE_JN_CAP;
    style |= TUBE_CONTOUR_CLOSED;
    style |= TUBE_NORM_FACET;
    style |= TUBE_JN_ANGLE;
    gleSetJoinStyle (style);

//  lastx = 121.0;
//  lasty = 121.0;

    init_contour();

}

/* draw the helix shape */
void DrawStuff (void)
{
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor3f (0.8, 0.3, 0.6);

    glPushMatrix ();

  /* set up some matrices so that the object spins with the mouse */
    glTranslatef (0.0,0.0,-150.0);
    glRotatef (rotate_frame[1], 1.0, 0.0, 0.0);
    glRotatef (rotate_frame[0], 0.0, 0.0, 1.0);


  //change thread, if necessary
    if (move_end[0] != 0.0 || move_end[1] != 0.0 || tangent_end[0] != 0.0 || tangent_end[1] != 0.0 || tangent_rotation_end[0] != 0 || tangent_rotation_end[1] != 0)
    {
        GLdouble model_view[16];
        glGetDoublev(GL_MODELVIEW_MATRIX, model_view);

        GLdouble projection[16];
        glGetDoublev(GL_PROJECTION_MATRIX, projection);

        GLint viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);


        double winX, winY, winZ;


    //change end positions
        Vector3d new_end_pos;
        gluProject(positions[1](0), positions[1](1), positions[1](2), model_view, projection, viewport, &winX, &winY, &winZ);
        winX += move_end[0];
        winY += move_end[1];
        move_end[0] = 0.0;
        move_end[1] = 0.0;
        gluUnProject(winX, winY, winZ, model_view, projection, viewport, &new_end_pos(0), &new_end_pos(1), &new_end_pos(2));
    //    std::cout << "X: " << positions[1](0) << " Y: " << positions[1](1) << " Z: " << positions[1](2) << std::endl;

    //change tangents
        Vector3d new_end_tan;
        gluProject(positions[1](0)+tangents[1](0),positions[1](1)+tangents[1](1), positions[1](2)+tangents[1](2), model_view, projection, viewport, &winX, &winY, &winZ);
        winX += tangent_end[0];
        winY += tangent_end[1];
        tangent_end[0] = 0.0;
        tangent_end[1] = 0.0;
        gluUnProject(winX, winY, winZ, model_view, projection, viewport, &new_end_tan(0), &new_end_tan(1), &new_end_tan(2));
        new_end_tan -= positions[1];
        new_end_tan.normalize();

        positions[1] = new_end_pos;

        Matrix3d rotation_new_tan;
        rotate_between_tangents(tangents[1], new_end_tan, rotation_new_tan);
        rotations[1] = rotation_new_tan*rotations[1];


    //check rotation around tangent
        Vector3d tangent_normal_rotate_around = rotations[1].col(1);
        Vector3d new_end_tan_normal;
        gluProject(positions[1](0)+tangent_normal_rotate_around(0), positions[1](1)+tangent_normal_rotate_around(1), positions[1](2)+tangent_normal_rotate_around(2), model_view, projection, viewport, &winX, &winY, &winZ);
        winX += tangent_rotation_end[0];
        winY += tangent_rotation_end[1];
        tangent_rotation_end[0] = 0.0;
        tangent_rotation_end[1] = 0.0;
        gluUnProject(winX, winY, winZ, model_view, projection, viewport, &new_end_tan_normal(0), &new_end_tan_normal(1), &new_end_tan_normal(2));
        new_end_tan_normal -= positions[1];
    //project this normal onto the plane normal to X (to ensure Y stays normal to X)
        new_end_tan_normal -= new_end_tan_normal.dot(rotations[1].col(0))*rotations[1].col(0);
        new_end_tan_normal.normalize();


        rotations[1].col(1) = new_end_tan_normal;
        rotations[1].col(2) = rotations[1].col(0).cross(new_end_tan_normal);

    //change thread
        thread->set_constraints(positions[0], rotations[0], positions[1], rotations[1]);
    //thread->set_end_constraint(positions[1], rotations[1]);

    //std::cout <<"CONSTRAINT END:\n" << rotations[1] << std::endl;
        thread->minimize_energy();
        updateThreadPoints();
        findThreadInIms();


    //std::cout <<"ACTUAL END:\n" << thread->end_rot() << std::endl;

    //std::cout << "objX: " <<objX << " objY: " << objY << " objZ: " << objZ << " winX: " << winX << " winY: " << winY << " winZ: " << winZ << std::endl;



    }


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

/*  glColor3d(0.5, 0.5, 1.0);
glVertex3f(positions[1](0)-positions[0](0), positions[1](1)-positions[0](1), positions[1](2)-positions[0](2)); //z
glVertex3f(positions[1](0)-positions[0](0)+tangents[1](0)*4.0, positions[1](1)-positions[0](1)+tangents[1](1)*4.0, positions[1](2)-positions[0](2)+tangents[1](2)*4.0); //z
*/


  //Draw Axes at End
Vector3d diff_pos = positions[1]-positions[0];
double rotation_scale_factor = 10.0;
Matrix3d rotations_project = rotations[1]*rotation_scale_factor;
glBegin(GL_LINES);
glEnable(GL_LINE_SMOOTH);
glColor3d(1.0, 0.0, 0.0); //red
glVertex3f((float)diff_pos(0), (float)diff_pos(1), (float)diff_pos(2)); //x
glVertex3f((float)(diff_pos(0)+rotations_project(0,0)), (float)(diff_pos(1)+rotations_project(1,0)), (float)(diff_pos(2)+rotations_project(2,0)));
glColor3d(0.0, 1.0, 0.0); //green
glVertex3f((float)diff_pos(0), (float)diff_pos(1), (float)diff_pos(2)); //y
glVertex3f((float)(diff_pos(0)+rotations_project(0,1)), (float)(diff_pos(1)+rotations_project(1,1)), (float)(diff_pos(2)+rotations_project(2,1)));
glColor3d(0.0, 0.0, 1.0); //blue
glVertex3f((float)diff_pos(0), (float)diff_pos(1), (float)diff_pos(2)); //z
glVertex3f((float)(diff_pos(0)+rotations_project(0,2)), (float)(diff_pos(1)+rotations_project(1,2)), (float)(diff_pos(2)+rotations_project(2,2)));


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


addThreadDebugInfo();



  //Draw Thread
glColor4f (0.5, 0.5, 0.2, 0.5);

  /*
  double pts_cpy[points.size()][3];
  double twist_cpy[points.size()];
  pts_cpy[0][0] = 0.0;
  pts_cpy[0][1] = 0.0;
  pts_cpy[0][2] = 0.0;
  twist_cpy[0] = -(360.0/(2.0*M_PI))*twist_angles[0];
  //std::cout << twist_cpy[0] << std::endl;

for (int i=1; i < points.size(); i++)
{
pts_cpy[i][0] = points[i](0)-(double)positions[0](0);
pts_cpy[i][1] = points[i](1)-(double)positions[0](1);
pts_cpy[i][2] = points[i](2)-(double)positions[0](2);
twist_cpy[i] = -(360.0/(2.0*M_PI))*twist_angles[i];
//std::cout << twist_cpy[i] << std::endl;
}

gleTwistExtrusion(20,
contour,
contour_norms,
NULL,
points.size(),
pts_cpy,
0x0,
twist_cpy);
*/


double pts_cpy[points.size()+2][3];
double twist_cpy[points.size()+2];
pts_cpy[1][0] = 0.0;
pts_cpy[1][1] = 0.0;
pts_cpy[1][2] = 0.0;
twist_cpy[1] = -(360.0/(2.0*M_PI))*twist_angles[0];
  //std::cout << twist_cpy[0] << std::endl;

for (int i=1; i < points.size()-1; i++)
{
    pts_cpy[i+1][0] = points[i](0)-(double)positions[0](0);
    pts_cpy[i+1][1] = points[i](1)-(double)positions[0](1);
    pts_cpy[i+1][2] = points[i](2)-(double)positions[0](2);
    twist_cpy[i+1] = -(360.0/(2.0*M_PI))*twist_angles[i];
    //std::cout << twist_cpy[i+1] - twist_cpy[i] << std::endl;
}

  //add first and last point
pts_cpy[0][0] = -rotations[0](0,0);
pts_cpy[0][1] = -rotations[0](1,0);
pts_cpy[0][2] = -rotations[0](2,0);
twist_cpy[0] = -(360.0/(2.0*M_PI))*twist_angles[0];

pts_cpy[points.size()][0] = (double)positions[1](0)-(double)positions[0](0);
pts_cpy[points.size()][1] = (double)positions[1](1)-(double)positions[0](1);
pts_cpy[points.size()][2] = (double)positions[1](2)-(double)positions[0](2);
twist_cpy[points.size()] = twist_cpy[points.size()-1];


pts_cpy[points.size()+1][0] = pts_cpy[points.size()][0]+rotations[1](0,0);
pts_cpy[points.size()+1][1] = pts_cpy[points.size()][1]+rotations[1](1,0);
pts_cpy[points.size()+1][2] = pts_cpy[points.size()][2]+rotations[1](2,0);
twist_cpy[points.size()+1] = twist_cpy[points.size()];

gleTwistExtrusion_c4f(20,
    contour,
    contour_norms,
    NULL,
    points.size()+2,
    pts_cpy,
    0x0,
    twist_cpy);



glPopMatrix ();

glutSwapBuffers ();

thread_vision.display();

}

void addThreadDebugInfo()
{
    for (int i=0; i < thread_vision.gl_display_for_debug.size(); i++)
    {
        glline_draw_params& currP = thread_vision.gl_display_for_debug[i];;
        glBegin(GL_LINE_STRIP);
        glColor4f(currP.color[0], currP.color[1], currP.color[2], 1.0);
        for (int j=0; j < currP.size; j++)
        {
            glVertex3f(currP.vertices[j].x-positions[0](0), currP.vertices[j].y-positions[0](1), currP.vertices[j].z-positions[0](2));
        }
        glEnd();

    }


}


void updateThreadPoints()
{
    thread->get_thread_data(points, twist_angles, material_frames);
    positions[0] = points.front();
    positions[1] = points.back();

    tangents[0] = points[1] - points[0];
    tangents[0].normalize();
    tangents[1] = points[points.size()-1] - points[points.size()-2];
    tangents[1].normalize();

    rotations[0] = thread->start_rot();
    rotations[1] = thread->end_rot();
}


void initThread()
{
    int numInit = 10;
    double noise_factor = 0.0;

    vector<Vector3d> vertices;
    vector<double> angles;

    Vector3d start_pt(18, -50, 125);

    vertices.push_back(start_pt);
    angles.push_back(0.0);
  //push back unitx so first tangent matches start_frame
    vertices.push_back(start_pt + Vector3d::UnitX()*_rest_length);
    angles.push_back(0.0);

    Vector3d direction;
    direction(0) = 1.0;
    direction(1) = 0.0;
    direction(2) = -1.0;
    direction.normalize();
    for (int i=0; i < numInit; i++)
    {
        Vector3d noise( ((double)(rand()%10000)) / 10000.0, ((double)(rand()%10000)) / 10000.0, ((double)(rand()%10000)) / 10000.0);
        noise *= noise_factor;
        Vector3d next_Vec = vertices.back()+(direction+noise).normalized()*_rest_length;
        vertices.push_back(next_Vec);
        angles.push_back(0.0);

    //std::cout << positions[i] << std::endl << std::endl;
    }



  //change direction
    direction(0) = 1.0;
    direction(1) = 0.0;
    direction(2) = 1.0;
    direction.normalize();

    for (int i=0; i < numInit; i++)
    {
        Vector3d noise( ((double)(rand()%10000)) / 10000.0, ((double)(rand()%10000)) / 10000.0, ((double)(rand()%10000)) / 10000.0);
        noise *= noise_factor;
        Vector3d next_Vec = vertices.back()+(direction+noise).normalized()*_rest_length;
        vertices.push_back(next_Vec);
        angles.push_back(0.0);

    }


  //push back unitx so last tangent matches end_frame
    vertices.push_back(vertices.back()+Vector3d::UnitX()*_rest_length);
    angles.push_back(0.0);


    angles.resize(vertices.size());

    rotations[0] = Matrix3d::Identity();
    rotations[1] = Matrix3d::Identity();

  /*
  Vector3d tan = (_thread_pieces[1].vertex() - _thread_pieces[0].vertex()).normalized();
  Vector3d toRotAxis;
  Matrix3d rot_for_frame;
  if ( (tan -Vector3d::UnitX()).norm() < 0.01 )
  {
  rot_for_frame = Matrix3d::Identity();
  }
  else 
  {
  toRotAxis = Vector3d::UnitX().cross(tan);
  double Axisnorm = toRotAxis.norm();
  double toRotAng = asin(Axisnorm);

toRotAxis /= Axisnorm;
// std::cout << "axis: " << toRotAxis << std::endl;
// std::cout << "ang: " << toRotAng << std::endl;

rot_for_frame = (Eigen::AngleAxisd(toRotAng, toRotAxis));
}
//std::cout << "rot for frame: " << rot_for_frame << std::endl;

_start_rot = rot_for_frame*Matrix3d::Identity();

//std::cout << "start frame: " << _start_rot << std::endl;

std::cout << "energy: " << calculate_energy() << std::endl;
*/


thread = new Thread(vertices, angles, rotations[0], rotations[1]);
updateThreadPoints();
}




void updateIms(Point3f& start_pt)
{
    updateThreadPoints();
    int num_pts = 300;

    vector<cv::Point3f> points_to_proj;

    for (int i=0; i < points.size()-1; i++)
    {
        Vector3d vec_between = points[i+1]-points[i];
        for (int j=0; j <= num_pts; j++)
        {
            Vector3d nextPoint = points[i]+( ((double)j)/((double)num_pts))*vec_between;
            cv::Point3f toAdd((float)nextPoint(0), (float)nextPoint(1), (float)nextPoint(2));
            points_to_proj.push_back(toAdd);
        }
    }

    Mat ims[3];
    for (int i=0; i < 3; i++){
        ims[i] = cv::Mat::zeros(thread_vision._frames[i].size(), CV_8UC3);
    }

    Point2i points2d[NUMCAMS];
    for (int i=0; i < points_to_proj.size(); i++)
    {
        thread_vision._cams->project3dPoint(points_to_proj[i], points2d);
        for (int j=0; j < NUMCAMS; j++)
        {
            if (thread_vision._captures[j]->inRange(points2d[j].y, points2d[j].x))
            {
                ims[j].at<Vec3b>(points2d[j].y, points2d[j].x)[0] = (unsigned char)255;
                ims[j].at<Vec3b>(points2d[j].y, points2d[j].x)[1] = (unsigned char)255;
                ims[j].at<Vec3b>(points2d[j].y, points2d[j].x)[2] = (unsigned char)255;
            }
        }

    }

    start_pt = points_to_proj[0];

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

void findThreadInIms()
{
    updateIms(_start_pt);
    thread_vision.setInitPt(_start_pt);
    thread_vision.optimizeThread();
}
