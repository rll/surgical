#include <iostream>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <Eigen/Core>
#include <Eigen/Geometry>



using namespace cv;
USING_PART_OF_NAMESPACE_EIGEN



std::istream & operator>>(std::istream& in, Mat &A);

std::ostream & operator<<(std::ostream& out, Mat& A);
std::ostream & operator<<(std::ostream& out, Point3f& A);
std::ostream & operator<<(std::ostream& out, Point3d& A);
std::ostream & operator<<(std::ostream& out, Point2i& A);

void ReadCvReal2DMatrix(std::istream& in, Mat& A);
void ReadCvReal2DMatrix(std::istream& in, CvMat* A);

void PrintCvReal2DMatrix(std::ostream& out, const Mat& A, const char* name);
void PrintPoint(std::ostream& out, const Point3f& A, const char* name);
void PrintPoint(std::ostream& out, const Point3d& A, const char* name);
void PrintPoint(std::ostream& out, const Point2i& A, const char* name);
void PrintCvReal2DMatrix(std::ostream& out, CvMat* A, const char* name);

void LeastSquares(const Mat& A, const Mat& b, Mat& LSsol);
void LeastSquares3Cam(const Mat& ray1, const  Mat& ray2, const  Mat& ray3, const  Mat& camPos1, const  Mat& camPos2, const Mat& camPos3, Mat& estimatedPoint);
void TwoPointsToLine(const Point2f&  point1, const Point2f& point2, Mat& lineParams); //in ax + by + c = 0 form
void PointsForSquare(const Point2i pt, const int dist, vector<Point2i>& pts);

//void LeastSquares(CvMat* const A, CvMat* const b, CvMat* LSsol, CvMat* residuals);


//char GetYesOrNo(const char* question);

double norm2(double* x, double* y, int n);

//bool operator <(const Point2i& a, const Point2i& b);

void EulerZYX(const double Z, const double Y, const double X, Matrix3d& rot);
void RotToEulerZYX(double& Z, double& Y, double& X, const Matrix3d& rot);
void EigenToOpencv(const Matrix3d& eig, Mat& cv);
void EigenToOpencv(const Vector3d& eig, Mat& cv);
void EigenToOpencv(const Vector3d& eig, Point3f& cv);
void OpencvToEigen(const Point3f& cv, Vector3d& eig);
