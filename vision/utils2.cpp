#include "util2.h"
#include <math.h>
#include <limits>


///////////////// cv matrix class helpers  //////////////////////////////////////////////////////////////
using namespace cv;

void
ReadCvReal2DMatrix(std::istream& in, Mat &A)
{
	double tmp;
	for (int i=0; i<A.rows; ++i){
		for (int j=0; j<A.cols; ++j){
			in >> tmp;
			if(in.fail())
				break;
			A.at<float>(i, j) = tmp;
		}
		if(in.fail())
			break;
	}
	if(in.fail()){
		std::cerr << "Error in pieter_util.cpp: ReadCvReal2DMatrix()." << std::endl;
		std::cerr << "Returning anyway, matrix not completely filled ..." << std::endl;
		return;
	}
}

void
PrintCvReal2DMatrix(std::ostream& out, const Mat &A, const char* name){
	if(name!=0){
		out << name << std::endl;
	}
	if(A.empty()){
		std::cerr << "Error in PrintCvReal2DMatrix: argument CvMat* == 0, can't print it." << std::endl;
		return;
	}
	for (int i=0; i<A.rows; ++i){
		for (int j=0; j<A.cols; ++j){
			out << A.at<float>(i, j) << "\t";
		}
		out << std::endl;
	}
}

void
PrintPoint(std::ostream& out, const Point3f& A, const char* name){
	if(name!=0){
		out << name << std::endl;
	}
    out << A.x << "    " << A.y << "    " << A.z;
}

void
PrintPoint(std::ostream& out, const Point3d& A, const char* name){
	if(name!=0){
		out << name << std::endl;
	}
    out << A.x << "    " << A.y << "    " << A.z;
}

void PrintPoint(std::ostream& out, const Point2i& A, const char* name){
	if(name!=0){
		out << name << std::endl;
	}
    out << A.x << "    " << A.y;
}




std::istream & operator>>(std::istream& in, Mat &A)
{
	ReadCvReal2DMatrix(in, A);
	return in;
}

std::ostream & operator<<(std::ostream& out, Mat &A)
{
	PrintCvReal2DMatrix(out, A,0);
	return out;
}

std::ostream & operator<<(std::ostream& out, Point3f &A)
{
	PrintPoint(out, A,0);
	return out;
}

std::ostream & operator<<(std::ostream& out, Point3d &A)
{
	PrintPoint(out, A,0);
	return out;
}

std::ostream & operator<<(std::ostream& out, Point2i &A)
{
	PrintPoint(out, A,0);
	return out;
}


void LeastSquares(const Mat& A, const Mat& b, Mat &LSsol)
{
    solve(A, b, LSsol, DECOMP_SVD);
}




void LeastSquares3Cam(const Mat& ray1, const  Mat& ray2, const  Mat& ray3, const  Mat& camPos1, const  Mat& camPos2, const Mat& camPos3, Mat& estimatedPoint)
{
    Mat A = Mat::zeros(9,6,CV_32FC1);
    A.at<float>(0,0)=1; 
    A.at<float>(1,1)=1; 
    A.at<float>(2,2)=1; 
    A.at<float>(3,0)=1; 
    A.at<float>(4,1)=1; 
    A.at<float>(5,2)=1; 
    A.at<float>(6,0)=1; 
    A.at<float>(7,1)=1; 
    A.at<float>(8,2)=1; 
    A.at<float>(0,3)=ray1.at<float>(0,0);
    A.at<float>(1,3)=ray1.at<float>(1,0);
    A.at<float>(2,3)=ray1.at<float>(2,0);
    A.at<float>(3,4)=ray2.at<float>(0,0);
    A.at<float>(4,4)=ray2.at<float>(1,0);
    A.at<float>(5,4)=ray2.at<float>(2,0);
    A.at<float>(6,5)=ray3.at<float>(0,0);
    A.at<float>(7,5)=ray3.at<float>(1,0);
    A.at<float>(8,5)=ray3.at<float>(2,0);

    Mat b = Mat(9,1,CV_32FC1);
    b.at<float>(0,0) = camPos1.at<float>(0,0);
    b.at<float>(1,0) = camPos1.at<float>(1,0);
    b.at<float>(2,0) = camPos1.at<float>(2,0);
    b.at<float>(3,0) = camPos2.at<float>(0,0);
    b.at<float>(4,0) = camPos2.at<float>(1,0);
    b.at<float>(5,0) = camPos2.at<float>(2,0);
    b.at<float>(6,0) = camPos3.at<float>(0,0);
    b.at<float>(7,0) = camPos3.at<float>(1,0);
    b.at<float>(8,0) = camPos3.at<float>(2,0);

    solve(A,b,estimatedPoint,DECOMP_SVD);
}

void TwoPointsToLine(const Point2f&  point1, const Point2f& point2, Mat& lineParams)
{
    double m,a,b,c;

    if (point2.x == point1.x)
    {
        a = 1;
        b = 0;
        c = -point1.x;
    } 
    else if (point2.y == point1.y) 
    {
        a = 0;
        b = 1;
        c = -point1.y;
    } 
    else
    {
        m = (double)(point2.y - point1.y) / (double)(point2.x - point1.x);
        b = 1./sqrt(1+m*m);
        a = -m*b;
        c = -(double)point1.x * a - (double)point1.y * b;
    }
    double yInt = point1.y - m*point1.x;

    lineParams.at<float>(0,0) = (float)a;
    lineParams.at<float>(1,0) = (float)b;
    lineParams.at<float>(2,0) = (float)c;

}

void PointsForSquare(const Point2i pt, const int dist, vector<Point2i>& pts)
{
  if (dist == 0)
  {
    pts.resize(1);
    pts[0] = pt;
    return;
  } 

  int r,c;
  r = pt.y - dist;
  int currInd = 0;
  pts.resize(8*dist);
  for (c = pt.x-dist; c <= pt.x+dist; c++)
  {
    pts[currInd].x = c;
    pts[currInd].y = r;
    currInd++;
  }

  c = pt.x+dist;
  for (r=pt.y-dist+1; r <= pt.y+dist; r++)
  {
    pts[currInd].x = c;
    pts[currInd].y = r;
    currInd++;
  }

  r = pt.y+dist;
  for (c = pt.x+dist-1; c >= pt.x-dist; c--)
  {
    pts[currInd].x = c;
    pts[currInd].y = r;
    currInd++;
  }
  c = pt.x-dist;
  for (r=pt.y+dist-1; r >= pt.y-dist+1; r--)
  {
    pts[currInd].x = c;
    pts[currInd].y = r;
    currInd++;
  }

}


/*void LeastSquares(CvMat* const A, CvMat* const b, CvMat* LSsol, CvMat* residuals)
{
	// Performs Singular Value Decomposition of a matrix 
	//CVAPI(void)   cvSVD( CvArr* A, CvArr* W, CvArr* U CV_DEFAULT(NULL),
	//                 CvArr* V CV_DEFAULT(NULL), int flags CV_DEFAULT(0));

//	PrintCvReal2DMatrix(std::cout, A, "A=\n");
//	PrintCvReal2DMatrix(std::cout, b, "b=\n");
	CvMat* W = cvCreateMat(A->height,A->width,CV_32FC1);
	CvMat* Wt = cvCreateMat(A->width,A->height,CV_32FC1);
	CvMat* Ut = cvCreateMat(A->height,A->height,CV_32FC1);
	CvMat* V = cvCreateMat(A->width, A->width,CV_32FC1);

	CvMat* tmp1 = cvCreateMat(A->height,1,CV_32FC1);
	CvMat* tmp2 = cvCreateMat(A->width,1,CV_32FC1);


//	PrintCvReal2DMatrix(std::cout, A, "A=\n");	
	cvSVD(A, W, Ut, V, CV_SVD_U_T);
//	PrintCvReal2DMatrix(std::cout, W, "W=\n");
//	PrintCvReal2DMatrix(std::cout, Ut, "Ut=\n");
//	PrintCvReal2DMatrix(std::cout, V, "V=\n");

	cvTranspose(W, Wt);

	// find the least squares solution from the SVD results
	const double my_eps = 1e-6;
	double W11 = cvGetReal2D(Wt,1,1);
	cvMatMul(Ut, b, tmp1);
	for (int i=0; i<Wt->height; ++i){
		for (int j=0; j<Wt->width; ++j){
			if(i==j){
				double Wii = cvGetReal2D(Wt,i,i);
				if(Wii/W11 > my_eps)
					cvSetReal2D(Wt,i,i,1.0/Wii);
			}
		}
	}
	cvMatMul(Wt, tmp1, tmp2);
	cvMatMul(V, tmp2, LSsol);

        CvMat* tmp3 = cvCreateMat(A->height,1,CV_32FC1);
        cvMatMul(A, LSsol, tmp3);
        cvSub(tmp3, b, residuals);
	
	cvReleaseMat(&W);
	cvReleaseMat(&Wt);
	cvReleaseMat(&Ut);
	cvReleaseMat(&V);
	cvReleaseMat(&tmp1);
	cvReleaseMat(&tmp2);

}



char GetYesOrNo(const char* question)
{
	char c;
	while(1){
		std::cout << question << " (y/n)\n";
		std::cin >> c;
		if(c=='y' || c=='Y' || c=='n' || c=='N')
			return(c);
	}
}

*/
double norm2(double* x, double* y, int n)
{
	double ans = 0.0;
	for (int i=0; i<n; ++i){
		ans += (x[i]-y[i])*(x[i]-y[i]);
	}
	return (sqrt(ans));
}



void EulerZYX(const double Z, const double Y, const double X, Matrix3d& rot)
{
	rot(0,0) = cos(Z)*cos(Y);
	rot(1,0) = sin(Z)*cos(Y);
	rot(2,0) = -sin(Y);

	rot(0,1) = cos(Z)*sin(Y)*sin(X)-sin(Z)*cos(X);
	rot(1,1) = sin(Z)*sin(Y)*sin(X)+cos(Z)*cos(X);
	rot(2,1) = cos(Y)*sin(X);

	rot(0,2) = cos(Z)*sin(Y)*cos(X)+sin(Z)*sin(X);
	rot(1,2) = sin(Z)*sin(Y)*cos(X)-cos(Z)*sin(X);
	rot(2,2) = cos(Y)*cos(X);
}

void RotToEulerZYX(double& Z, double& Y, double& X, const Matrix3d& rot)
{
	Y = atan2(-rot(2,0), sqrt( pow(rot(0,0),2) + pow(rot(1,0),2)));
	Z = atan2(rot(1,0), rot(0,0));
	X = atan2(rot(2,1), rot(2,2));
}

/*bool operator <(const Point2i& a, const Point2i& b)
{
  return a.y < b.y || (a.y == b.y && a.x < b.x);
}*/


void EigenToOpencv(const Matrix3d& eig, Mat& cv)
{
	cv.create(eig.rows(),eig.cols(),CV_32FC1);
	for (int r=0; r < eig.rows(); r++)
	{
		for (int c=0; c < eig.cols(); c++)
		{
			cv.at<float>(r,c) = eig(r,c);
		}
	}
}

void EigenToOpencv(const Vector3d& eig, Mat& cv)
{
	cv.create(eig.rows(),eig.cols(),CV_32FC1);
	for (int r=0; r < eig.rows(); r++)
	{
		for (int c=0; c < eig.cols(); c++)
		{
			cv.at<float>(r,c) = eig(r,c);
		}
	}
}

void EigenToOpencv(const Vector3d& eig, Point3f& cv)
{
	cv.x = (float)eig(0);
	cv.y = (float)eig(1);
	cv.z = (float)eig(2);
}


void OpencvToEigen(const Point3f& cv, Vector3d& eig)
{
	eig(0) = (double)cv.x;
	eig(1) = (double)cv.y;
	eig(2) = (double)cv.z;
}
