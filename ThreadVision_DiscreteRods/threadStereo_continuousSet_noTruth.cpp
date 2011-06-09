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
#include "../DiscreteRods/glThread.h"
#include "thread_vision_discrete.h"
#include "../DiscreteRods/trajectory_reader.h"
#include "../DiscreteRods/trajectory_recorder.h"


#define MAX_LENGTH_VIS 67


#define FAKEIMS_FILE "saved_threads/test_trajectories"
#define IMAGE_SAVE_BASE_OPENGL "saved_threads/saved_opengl"

//#define TRY_ALL false

vector<Thread> saved_trajectory;


#define NUM_THREADS_DISPLAY 2
#define NUM_THREADS 60
enum DisplayedThreads {optimizedThread, truthThread};

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

void InitLights();
void InitGLUT(int argc, char * argv[]);
void InitStuff();
void DrawStuff();
void DrawAxes();
void InitThread(int argc, char* argv[]);
void updateIms(cv::Point3f& start_pt, Vector3d& start_tan, cv::Point3f& end_pt, Vector3d& end_tan);
void findThreadInIms();
void findThreadInIms_next();
void showThread(Thread *aThread, DisplayedThreads type = optimizedThread);
void addThreadDebugInfo();
void save_opengl_image();
double thread_difference(Thread* t1, Thread* t2);





#define NUM_PTS 500
// #define THREAD_RADII 1.0
#define MOVE_POS_CONST 1.0
#define MOVE_TAN_CONST 0.2
#define ROTATE_TAN_CONST 0.2

#define WINDOW_HEIGHT 900
#define WINDOW_WIDTH 900


enum key_code {NONE, MOVEPOS, MOVETAN, ROTATETAN};
enum thread_type {silk, nylon, vicryl};

float lastx_L=0;
float lasty_L=0;
float lastx_R=0;
float lasty_R=0;

float rotate_frame[2];
float move_end[2];
float tangent_end[2];
float tangent_rotation_end[2];

GLThread* glThreads[NUM_THREADS_DISPLAY];
bool show_threads[NUM_THREADS_DISPLAY];

thread_type _thread_type = silk;

cv::Point3f _start_pt;
cv::Point3f _end_pt;
Vector3d _start_tan;
Vector3d _end_tan;
int curr_im_ind = 1;
Thread_Vision thread_vision;
bool thread_vision_searched = false;

int thread_ind = 0;
double twistAngle_correct;
double twistAngle_best;
double score_correct_twist;
double score_best_twist;

/* Keep track of difference between generated thread and actual for
 * each thread read from trajectory file */
 double thread_differences[NUM_THREADS];

 /* Keep track of runtimes with different metrics as well */
 double generation_times[NUM_THREADS];


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

/* Do comparison of runtime */

clock_t start_time, end_time;
void wait(int seconds)
{
        clock_t endwait;
            endwait = clock() + seconds*CLOCKS_PER_SEC;
                while (clock() < endwait) {};
}
void start_timer()
{
    start_time = clock();
}
void stop_timer()
{
    end_time = clock();
}
double seconds_elapsed()
{
    return ((double) end_time - start_time)/((double) CLOCKS_PER_SEC);
}

/* Difference between 2 threads based on l2 norm of distance between vertices.
 * Requires that 2 threads be the same length.
 */
double thread_difference(Thread* t1, Thread* t2)
{
    double difference = 0;
    int length = t1->_thread_pieces.size();
    for (int i = 0; i < length; i++)
    {
        difference += (t1->_thread_pieces[i]->vertex()
                      - t2->_thread_pieces[i]->vertex()).norm();
    }
    return difference/((double) length);
}


void selectThread(int inc) {
    int i = 0;
//    curThread = (curThread + inc) % totalThreads;
//    glutPostRedisplay();
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

}

void next_thread()
{
    thread_ind = (thread_ind + NUM_THREADS + 1)%NUM_THREADS;
    cout << "displaying thread " << thread_ind << std::endl;
    glThreads[truthThread]->setThread(new Thread(saved_trajectory[thread_ind]));
    glThreads[truthThread]->updateThreadPoints();
    glutPostRedisplay();
}

void prev_thread()
{
    thread_ind = (thread_ind + NUM_THREADS - 1)%NUM_THREADS;
    cout << "displaying thread " << thread_ind << std::endl;
    glThreads[truthThread]->setThread(new Thread(saved_trajectory[thread_ind]));
    glThreads[truthThread]->updateThreadPoints();
    glutPostRedisplay();
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
    }
    else if (key == 'n') {
        next_thread();
    } else if (key == 'b') {
        prev_thread();
    } else if (key == 'v') {
        findThreadInIms();
        glutPostRedisplay();
    } else if (key == 'z') {
        /* Step */
        if (thread_vision.hasInit) {
            if (!thread_vision.isDone()) {
                thread_vision.generateNextSetOfHypoths();
                showThread(thread_vision.curr_thread());
            }
            else {
                cout << "Search already finished" << endl;
            }
        }
        else {
            // Taken from start of findThreadInIms();
            thread_vision_searched = true;
            thread_vision.set_max_length(MAX_LENGTH_VIS);
            thread_vision.clear_display();
            updateIms(_start_pt, _start_tan, _end_pt, _end_tan);
            thread_vision.clearStartData();
            thread_vision.addStartData(_start_pt, _start_tan);
            thread_vision.initThreadSearch();
            //cout << "Thread Vision not initialized. Press 'v' to init" << endl;
        }
    } else if (key == 'q') {
        cout << "Flip to prev hypoth" << endl;
        thread_vision.prev_hypoth();
        showThread(thread_vision.curr_thread());
    } else if (key == 'w') {
        cout << "Flip to next hypoth" << endl;
        thread_vision.next_hypoth();
        showThread(thread_vision.curr_thread());
    } else if (key == '1' && key <= '1' + NUM_THREADS_DISPLAY)
    {
    show_threads[((int)key-'1')] = !show_threads[((int)key-'1')];
//    } else if (key == 's') {
//        /* Save current trajectory */
//        cout << "Saving...\n";
//        cout << "Please enter destination file name (without extension): ";
//        char *dstFileName = new char[256];
//        cin >> dstFileName;
//        char *fullPath = new char[256];
//        sprintf(fullPath, "%s%s", "saved_threads/", dstFileName);
//
//        traj_recorder.setFileName(fullPath);
//
//        Thread *newThread = glThreads[currentThread]->getThread();
//        Thread copiedThread(*newThread);
//        traj_recorder.add_thread_to_list(copiedThread);
//        traj_recorder.write_threads_to_file();
    }
    /* Run through all threads, generate guess, save image in test_images */
    else if (key == 'p')
    {
        thread_ind = 0;
        while (true) {
            next_thread();
            DrawStuff();
            cout << endl << "testing thread " << thread_ind << endl << endl;
            start_timer();
            findThreadInIms();
            glutPostRedisplay();
            stop_timer();
            wait(2);
            thread_differences[thread_ind] = thread_difference(glThreads[truthThread]->getThread(), thread_vision.curr_thread());
            generation_times[thread_ind] = seconds_elapsed();
            printf("thread %2d |\t runtime (s): %05.3f | difference: %05.3f\n", thread_ind, generation_times[thread_ind], thread_differences[thread_ind]);
            DrawStuff();
            save_opengl_image();
            if (thread_ind == 59) break;
        }
        cout << endl << "-------------------" << endl;
        cout << "PERFORMANCE SUMMARY" << endl;
        cout << "-------------------" << endl;
        for (int i = 1; i < NUM_THREADS; i++)
        {
            printf("thread %2d |\t runtime (s): %05.3f | difference: %05.3f\n", i, generation_times[i], thread_differences[i]);
        }
    }
    else if (key == 27) {
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


    glutMainLoop ();
}


GLuint sphereList;
void InitStuff (void)
{
    //glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
    glEnable(GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //gleSetJoinStyle (TUBE_NORM_PATH_EDGE | TUBE_JN_ANGLE );
    rotate_frame[0] = 30.0;
    rotate_frame[1] = -75.0;

    sphereList = glGenLists(1);
    glNewList(sphereList, GL_COMPILE);
    glutSolidSphere(0.5,16,16);
    glEndList();
}

/* draw the helix shape */
void DrawStuff (void)
{
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor3f (0.8, 0.3, 0.6);

    glPushMatrix ();

    /* set up some matrices so that the object spins with the mouse */
    glTranslatef (0.0,0.0,-120.0);
    glRotatef (rotate_frame[1], 1.0, 0.0, 0.0);
    glRotatef (rotate_frame[0], 0.0, 0.0, 1.0);

    //change thread, if necessary
    if (move_end[0] != 0.0 || move_end[1] != 0.0 || tangent_end[0] != 0.0 || tangent_end[1] != 0.0 || tangent_rotation_end[0] != 0 || tangent_rotation_end[1] != 0)
    {
        glThreads[truthThread]->ApplyUserInput(move_end, tangent_end, tangent_rotation_end);
    }


    //Draw Axes
    DrawAxes();
    addThreadDebugInfo();

    if (thread_vision_searched)
    {
      glThreads[optimizedThread]->updateThreadPoints();
    }
    Vector3d display_start_pos = glThreads[optimizedThread]->getStartPosition();


    for(int i = 0; i < NUM_THREADS_DISPLAY; i++) {
        //Draw Thread
        if (!show_threads[i])
            continue;
        if (i==optimizedThread) {
            glColor4f (0.8, 0.5, 0.0, 1.0);
        } else if (i==truthThread) {
            glColor4f (0.8, 0.0, 0.0, 1.0);
        } else {
            glColor4f (0.5, 0.5, 0.5, 1.0);
        }
        glThreads[i]->display_start_pos = display_start_pos;
        glThreads[i]->DrawThread();
    }

    if (thread_vision_searched)
    {
        thread_vision.display();
    }


    glPopMatrix ();
    glutSwapBuffers ();
}



void InitThread(int argc, char* argv[])
{

#ifdef FAKEIMS
  Trajectory_Reader for_stereo_fakeims(FAKEIMS_FILE);
  for_stereo_fakeims.read_threads_from_file();
  saved_trajectory = for_stereo_fakeims.get_all_threads();
#endif


  for (int i=0; i < NUM_THREADS_DISPLAY; i++)
  {
    glThreads[i] = new GLThread();
#ifdef FAKEIMS
    glThreads[i]->setThread(new Thread(saved_trajectory[0]));
#endif
    glThreads[i]->updateThreadPoints();
    show_threads[i] = true;

    if (_thread_type == nylon)
    {
      glThreads[i]->to_set_bend = 232.0000;
      glThreads[i]->to_set_twist = 847.000;
      glThreads[i]->to_set_grav = 1e-4;
    } else if (_thread_type == vicryl) {
      glThreads[i]->to_set_bend = 17.0000;
      glThreads[i]->to_set_twist = 54.6250;
      glThreads[i]->to_set_grav = 1e-4;
    } else if (_thread_type == silk) {
      glThreads[i]->to_set_bend = 123.5000;
      glThreads[i]->to_set_twist = 209.250;
      glThreads[i]->to_set_grav = 1e-4;
    }
  }


}


void DrawAxes()
{
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
    glutInit (&argc, argv);
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(900,900);
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
    glFrustum (-30.0, 30.0, -30.0, 30.0, 50.0, 500.0);
    glMatrixMode(GL_MODELVIEW);
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
    glEnable (GL_LIGHTING);
    glColorMaterial (GL_FRONT_AND_BACK, GL_DIFFUSE);
    glEnable (GL_COLOR_MATERIAL);
}



//New version for search from both ends
void updateIms(Point3f& start_pt, Vector3d& start_tan, Point3f& end_pt, Vector3d& end_tan)
{
  
#ifdef FAKEIMS
    glThreads[truthThread]->updateThreadPoints();
    vector<Vector3d> points = glThreads[truthThread]->points;
    int num_pts = 300;

    vector<cv::Point3f> points_to_proj;

    /* Interpolate between points linearly for the generated image */
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


    char im_name[256];
    for (int cam_ind = 0; cam_ind < NUMCAMS; cam_ind++)
    {
        sprintf(im_name, "./stereo_test/stereo_test%d-%d.tif", cam_ind+1, curr_im_ind);
        imwrite(im_name, ims[cam_ind]);
    }
    curr_im_ind++;

    EigenToOpencv(points.front(), start_pt);
    EigenToOpencv(points.back(), end_pt);

    start_tan = (points[1]-points[0]).normalized();
    end_tan = (points[points.size() - 2] - points[points.size() - 1]).normalized();
#else
    thread_vision._cams->setImageNumber(thread_ind+1);
    start_pt = _start_pt;
    end_pt = _end_pt;
    start_tan = _start_tan;
    end_tan = _end_tan;
#endif


      
}

vector<Vector3d> points_real;
vector<double> angle_real;

void findThreadInIms()
{
    thread_vision_searched = true;

    //will run search until this length
    thread_vision.set_max_length(MAX_LENGTH_VIS);
    //clear debugging displays
    thread_vision.clear_display();

    updateIms(_start_pt, _start_tan, _end_pt, _end_tan);
    thread_vision.clearStartData();
    thread_vision.addStartData(_start_pt, _start_tan);
    //thread_vision.addStartData(_end_pt, _end_tan);


    //updates images in thread_vision class, runs canny
    //Also generates initial hypotheses
    thread_vision.initThreadSearch();

    
    if (thread_vision.runThreadSearch())
    {
        std::cout << "Found thread full opt" << std::endl;
        showThread(thread_vision.curr_thread());
    }
}

void showThread(Thread *aThread, DisplayedThreads type)
{
    /* Will make a copy of the thread */
    glThreads[type]->setThread(new Thread(*aThread));
    glThreads[type]->updateThreadPoints();

    if (thread_vision.isDone()) {
        /* May move debug images to when 5 thread pieces have been added */
        thread_vision.addThreadPointsToDebugImages(Scalar(200,0,0));
        thread_vision.add_debug_points_to_ims();

        //thread_vision.saveImages(image_save_base, thread_ind+1);

        vector<Vector3d> points_im;
        vector<double> angle_im;

        //thread_vision.get_thread_data(points_im, angle_im);
        //err_fullopt = calculate_vector_norm_avg(points_im, points_real)/points_im.size();
        //std::cout << "Error: " << err_fullopt << std::endl;

        //twistAngle_correct = glThreads[currentThread]->getThread()->end_angle();
        //twistAngle_best = thread_vision.end_angle();
    }

    glutPostRedisplay();

}


/* NOT WORKING, runThreadSearch_nextIms() just returns */
//assumes one thread already found, just tries to find the next one
void findThreadInIms_next()
{
  thread_ind++;
  thread_vision.clear_display();
#ifdef FAKEIMS
  showThread(&saved_trajectory[thread_ind], truthThread);
#endif

  updateIms(_start_pt, _start_tan, _end_pt, _end_tan);
  //thread_vision.clearStartData();
  //thread_vision.addStartData(_start_pt, _start_tan);


  //    for (int i = 0; i < 11; i++)
  //        thread_vision.generateNextSetOfHypoths();

  thread_vision.runThreadSearch_nextIms();
  showThread(thread_vision.curr_thread());


}



void addThreadDebugInfo()
{
    Vector3d display_start_pos = glThreads[optimizedThread]->getStartPosition();
    for (int i=0; i < thread_vision.gl_display_for_debug.size(); i++)
    {
        glline_draw_params& currP = thread_vision.gl_display_for_debug[i];;
        glBegin(GL_LINE_STRIP);
        glColor4f(currP.color[0], currP.color[1], currP.color[2], 1.0);
        for (int j=0; j < currP.size; j++)
        {
            glVertex3f(currP.vertices[j].x-display_start_pos(0), currP.vertices[j].y-display_start_pos(1), currP.vertices[j].z-display_start_pos(2));
        }
        glEnd();

    }


}


void save_opengl_image()
{
    const int IMG_COLS_TOTAL = WINDOW_WIDTH;
    const int IMG_ROWS_TOTAL = WINDOW_HEIGHT;
    //playback and save images
    Mat img(WINDOW_WIDTH, WINDOW_HEIGHT, CV_8UC3);
    vector<Mat> img_planes;
    split(img, img_planes);

    uchar tmp_data[3][WINDOW_WIDTH*WINDOW_HEIGHT];

    GLenum read_formats[3];
    read_formats[0] = GL_BLUE;
    read_formats[1] = GL_GREEN;
    read_formats[2] = GL_RED;

    for (int i=0; i < 3; i++)
    {
        glReadPixels(0, 0, IMG_COLS_TOTAL, IMG_ROWS_TOTAL, read_formats[i], GL_UNSIGNED_BYTE, tmp_data[i]);
        img_planes[i].data = tmp_data[i];
    }

    merge(img_planes, img);
    flip(img, img, 0);

    char im_name[256];
    const char*prefix = USE_DISTANCE_METRIC ? "distance" : "energy";
    sprintf(im_name, "test_images/%s_thread%d_%d.jpg", prefix, NUM_HYPOTHS_MAX, thread_ind);
    cout << "saving " << im_name << "...\n";
    imwrite(im_name, img);
    waitKey(1);
}

