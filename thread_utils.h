#ifndef _thread_utils_h
#define _thread_utils_h

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include "NLF.h"
#include "OptQNewton.h"
#include "NLP.h"
#include "GenSet.h"
#include "OptGSS.h"
//#include "OptPDS.h"
 //  #include "cblas.h"
  // #include "ioformat.h"



// import most common Eigen types 
USING_PART_OF_NAMESPACE_EIGEN

void getTransform(double curvature, double torsion, double length, Matrix4d& T);
void inverseTransform(Matrix4d& orig, Matrix4d& inv);

void getPoint(Matrix4d& transform, double length, double curvature, double torsion, Vector3d point);
void getPoint(Matrix4d& transform, double length, double curvature, double torsion, double rho, double c, Vector3d point);

void transformFromEulerAngles(Matrix4d& transform, double angZ, double angY, double angX=0.0, const Vector3d& start_pts=Vector3d::Zero());
void eulerAnglesFramTransform(const Matrix3d& transform, double& angZ, double& angY, double& angX);

double randomNumUnit();
double randomMaxAbsValue(double max_abs);

void rotationBetweenTans(const Vector3d& tan_start, const Vector3d& tan_end, Matrix3d& rotation_between);

double avgDistBetweenPoints(MatrixXd& a, MatrixXd& b);


void optimize_GSS(int numParams, OPTPP::USERFCN0 eval, OPTPP::INITFCN init, NEWMAT::ColumnVector& solution, double fcnTol=1e-7);
void optimize_FDNLF(int numParams, OPTPP::USERFCN0 eval, OPTPP::INITFCN init, NEWMAT::ColumnVector& solution, double fcnTol=1e-7, int numIters=5000);
//void optimize_PDS(int numParams, OPTPP::USERFCN0 eval, OPTPP::INITFCN init, NEWMAT::ColumnVector& solution, double fcnTol=1e-7);


#endif
