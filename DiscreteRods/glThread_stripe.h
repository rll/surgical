#ifndef _gl_thread_stripe_h
#define _gl_thread_stripe_h

#include "thread_discrete.h"

#ifdef MAC
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#include <GL/gle.h>
#else
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/gle.h>
#endif

using namespace std;

//CONTOUR STUFF
#define SCALE 1.0
#define CONTOUR(x,y) {                          \
    double ax, ay, alen;                        \
    contour[i][0] = SCALE * (x);                \
    contour[i][1] = SCALE * (y);                \
    if (i!=0) {                                 \
      ax = contour[i][0] - contour[i-1][0];     \
      ay = contour[i][1] - contour[i-1][1];     \
      alen = 1.0 / sqrt (ax*ax + ay*ay);        \
      ax *= alen;   ay *= alen;                 \
      contour_norms [i-1][0] = ay;              \
      contour_norms [i-1][1] = -ax;             \
    }                                           \
    i++;                                        \
  }

#define NUM_PTS_CONTOUR (100)

enum ColorCode{basic, material, bishop};


class GLThread {
 public:
  GLThread();
  GLThread(Thread* _thread);
  //void GetConfiguration(int* size, double* pts_cpy[][], double* twist_cpy[]);
  void DrawThread(bool disableTexture = false);
  void DrawAxes();
	void DrawAxesAtPoint(int ind, ColorCode = basic, bool skipX = false);
	void DrawAxesAtPoint(const Vector3d& pt, const Matrix3d& rot, ColorCode = basic, bool skipX = false);
	void DrawDownvecAtInd(int ind);
	void DrawSpheresAtLinks();
  void DrawAngleArcAtPoint(int i);
	void DrawArcBetweenMats(Vector3d& center, Vector3d& dir1, Vector3d& dir2, double inner_radius, double outer_radius);
	void DrawArcBetweenMats(Vector3d& center, Vector3d& dir1, Vector3d& dir2, double inner_radius, double outer_radius, string& str);
  void minimize_energy();
  void updateThreadPoints();
  void ApplyUserInput(float move_end[], float tangent_end[], float tangent_rotation_end[]);
  void InitContour();
  void set_end_constraint(Vector3d pos, Matrix3d rot);

  Thread* getThread() { return _thread; }
  void setThread(Thread* thread) {
    delete _thread;
    _thread = thread;
#ifdef ISOTROPIC
    _thread->set_coeffs_normalized(to_set_bend, to_set_twist, to_set_grav);
#else
    _thread->set_coeffs_normalized(to_set_B, to_set_twist, to_set_grav);
#endif
  }

  void setThreadCoeffs()
  {
#ifdef ISOTROPIC
    _thread->set_coeffs_normalized(to_set_bend, to_set_twist, to_set_grav);
#else
    _thread->set_coeffs_normalized(to_set_B, to_set_twist, to_set_grav);
#endif
  }

  void printThreadData();

  //Vector3d getEndPosition() { return positions[1]; }
  const Vector3d& getEndPosition() { return positions[1]; }
  const Vector3d& getStartPosition() { return positions[0]; }
  Vector3d getEndTangent()  { return tangents[1]; }
  Matrix3d getEndRotation() { return rotations[1]; }
  Matrix3d getStartRotation() { return rotations[0]; }


	void addTexture_stripe();
	void removeTexture_stripe();
	void setTexture_truth();
	int thread_color[3];
	int stripe_color[3];
	int truththread_color[3];
	float thread_color_float[3];
	float stripe_color_float[3];


#ifdef ISOTROPIC
  double to_set_bend;
#else
  Matrix2d to_set_B;
#endif
  double to_set_twist;
  double to_set_grav;



// protected:
  Thread* _thread;

  Vector3d positions[2];
  Vector3d tangents[2];
  Matrix3d rotations[3];

  vector<Vector3d> points;
  vector<double> twist_angles;

  double contour[NUM_PTS_CONTOUR][2];
  double contour_norms[NUM_PTS_CONTOUR][2];

  Vector3d display_start_pos;

};

#endif
