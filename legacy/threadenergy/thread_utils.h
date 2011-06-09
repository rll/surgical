#ifndef _thread_utils_h
#define _thread_utils_h

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include "NLF.h"
#include "OptQNewton.h"
#include "NLP.h"
#include "GenSet.h"
#ifdef PDS
  #include "OptPDS.h"
#else
  #include "OptGSS.h"
#endif



// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

void getTransform(double curvature, double torsion, double length, Matrix4d& T);
void inverseTransform(Matrix4d& orig, Matrix4d& inv);

void getPoint(Matrix4d& transform, double length, double curvature, double torsion, Vector3d point);
void getPoint(Matrix4d& transform, double length, double curvature, double torsion, double rho, double c, Vector3d point);

void transformFromEulerAngles(Matrix4d& transform, double angZ, double angY, double angX=0.0, const Vector3d& start_pts=Vector3d::Zero());
void eulerAnglesFromTransform(const Matrix3d& transform, double& angZ, double& angY, double& angX);

double randomNumUnit();
double randomMaxAbsValue(double max_abs);

void rotationBetweenTans(const Vector3d& tan_start, const Vector3d& tan_end, Matrix3d& rotation_between);

double avgDistBetweenPoints(MatrixXd& a, MatrixXd& b);


#ifdef PDS
void optimize_PDS(int numParams, OPTPP::USERFCN0 eval, OPTPP::INITFCN init, NEWMAT::ColumnVector& solution, double fcnTol=1e-7);
#else
void optimize_GSS(int numParams, OPTPP::USERFCN0 eval, OPTPP::INITFCN init, NEWMAT::ColumnVector& solution, double fcnTol=1e-7);
#endif
void optimize_FDNLF(int numParams, OPTPP::USERFCN0 eval, OPTPP::INITFCN init, NEWMAT::ColumnVector& solution, double fcnTol=1e-7, int numIters=5000, double maxStep=-1);

void readMatrix4d(Matrix4d&, ifstream& in);
void writeMatrix4d(const Matrix4d&, ofstream& out);
void readVector3d(Vector3d& , ifstream& in);
void writeVector3d(const Vector3d&, ofstream& out);
void readParams(std::string file, double* out);
void writeParams(std::string file, double* out);


#endif
