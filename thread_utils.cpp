#include "thread_utils.h"


void getTransform(double curvature, double torsion, double length, Matrix4d& T)
{

	double rho = sqrt(curvature*curvature + torsion*torsion);
	double r = rho*length;
	double c = (1.0/(rho*rho));


	T(0,0) = c*(curvature*curvature*cos(r)+torsion*torsion);
	T(1,1) = c*rho*rho*cos(r);
	T(2,2) = c*(curvature*curvature + torsion*torsion*cos(r));

	T(1,0) = c*curvature*rho*sin(r);
	T(0,1) = -T(1,0);
	T(0,2) = T(2,0) = c*curvature*torsion*(1.0-cos(r));
	T(2,1) = c*torsion*rho*sin(r);
	T(1,2) = -T(2,1);

	c /= rho;

	T(0,3) = c*(curvature*curvature*sin(r) + torsion*torsion*r);
	T(1,3) = c*(curvature*rho*(1.0-cos(r)));
	T(2,3) = c*(curvature*torsion*(r-sin(r)));

	T(3,0) = T(3,1) = T(3,2) = 0.0;
	T(3,3) = 1.0;


}

void inverseTransform(Matrix4d& orig, Matrix4d& inv)
{
	inv.corner(Eigen::TopLeft,3,3) = orig.corner(Eigen::TopLeft,3,3).transpose();
	inv.corner(Eigen::TopRight,3,1) = -1*(inv.corner(Eigen::TopLeft,3,3)*orig.corner(Eigen::TopRight,3,1));

	inv(3,0) = inv(3,1) = inv(3,2) = 0.0;
	inv(3,3) = 1.0;

}


void getPoint(Matrix4d& transform, double length, double curvature, double torsion, Vector3d point)
{
	
	double rho = sqrt(curvature*curvature + torsion*torsion);
	double c = (1.0/(rho*rho*rho));
  getPoint(transform, length, curvature, torsion, rho, c, point);
}


void getPoint(Matrix4d& transform, double length, double curvature, double torsion, double rho, double c, Vector3d point)
{
  double r = rho*length;
  
  Vector4d p;
  p(0) = c*(curvature*curvature*sin(r) + torsion*torsion*r);
  p(1) = c*(curvature*rho*(1.0-cos(r)));
  p(2) = c*(curvature*torsion*(r-sin(r)));
  p(3) = 1.0;
  point = (transform*p).start(3);

}

void transformFromEulerAngles(Matrix4d& transform, double angZ, double angY, double angX, const Vector3d& start_pts)
{
  Matrix3d rot1(Eigen::AngleAxisd(angZ, Vector3d::UnitZ()));
  Vector3d axis2 = rot1*(Vector3d::UnitY());
  Matrix3d rot2 = Eigen::AngleAxisd(angY, axis2)*rot1;
  Vector3d axis3 = rot2*(Vector3d::UnitX());
  transform.corner(Eigen::TopLeft,3,3) = Eigen::AngleAxisd(angX, axis3)*rot2;
  transform.corner(Eigen::TopRight,3,1) = start_pts;
  transform(3,0) = transform(3,1) = transform(3,2) = 0.0;
  transform(3,3) = 1.0;


  transform.corner(Eigen::TopRight,3,1) = start_pts;
}

void eulerAnglesFramTransform(const Matrix3d& transform, double& angZ, double& angY, double& angX)
{
  angZ = atan2(transform(1,0), transform(0,0));
  angY = atan2(-transform(2,0), sqrt(transform(2,1)*transform(2,1) + transform(2,2)*transform(2,2)));
  angX = atan2(transform(2,1), transform(2,2));
}


double randomNumUnit()
{
  return ((double)rand()) / ((double)RAND_MAX);
}

double randomMaxAbsValue(double max_abs)
{
  return 2.0*max_abs*randomNumUnit() - max_abs;
}


void rotationBetweenTans(const Vector3d& tan_start, const Vector3d& tan_end, Matrix3d& rotation_between)
{
	Vector3d axis = tan_start.cross(tan_end);
	if (axis.norm() <= 1e-7)
  {
    rotation_between.setIdentity();
    return;
  }
  axis.normalize();
  rotation_between = (Eigen::AngleAxisd(acos(tan_start.dot(tan_end)),axis));
}


//assumes these matrices are the same size
double avgDistBetweenPoints(MatrixXd& a, MatrixXd& b)
{
  double toRtn = 0.0;
  int numCols = a.cols();
  for (int i=0; i < a.rows(); i++)
  {
    toRtn += (a.block(i,0,1,numCols) - b.block(i,0,1,numCols)).norm();
  }
  toRtn /= ((double)(a.rows()));

  return toRtn;
}


void optimize_GSS(int numParams, OPTPP::USERFCN0 eval, OPTPP::INITFCN init, NEWMAT::ColumnVector& solution, double fcnTol)
{
  OPTPP::NLF0 nlp(numParams, eval, init); 
  OPTPP::GenSetStd gs(numParams); 
  OPTPP::OptGSS optobj(&nlp, &gs);
  optobj.setMaxIter(5000); 
  optobj.setMaxFeval(50000); 
  optobj.setFcnTol(fcnTol);  
  optobj.setFullSearch(true);
  optobj.optimize();

  solution = nlp.getXc();

}


void optimize_FDNLF(int numParams, OPTPP::USERFCN0 eval, OPTPP::INITFCN init, NEWMAT::ColumnVector& solution, double fcnTol, int numIters)
{
  OPTPP::FDNLF1 nlp(numParams, eval, init);

  OPTPP::OptQNewton objfcn(&nlp);

  objfcn.setSearchStrategy(OPTPP::TrustRegion);
  objfcn.setMaxFeval(20000);
  objfcn.setMaxIter(numIters);
  objfcn.setFcnTol(fcnTol);

  objfcn.optimize();

  solution = nlp.getXc();
}

/*
void optimize_PDS(int numParams, OPTPP::USERFCN0 eval, OPTPP::INITFCN init, NEWMAT::ColumnVector& solution, double fcnTol)
{
  OPTPP::NLF0 nlp(numParams, eval, init); 
  OPTPP::OptPDS objfcn(&nlp);
  objfcn.setFcnTol(fcnTol);
  objfcn.setMaxIter(5000);
  objfcn.setMaxFeval(10000);
  objfcn.setMaxStep(1.e-1);

  objfcn.optimize();
  
  solution = nlp.getXc();

}*/
