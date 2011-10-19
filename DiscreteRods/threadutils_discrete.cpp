#include "threadutils_discrete.h"
#include <iostream>
#include <fstream>

using namespace Eigen;

double Normal(double mu, double sigma) {
  double res = 0.0;
  for(int i = 0; i < 24; i++) {
    res += drand48();
  }
  res -= 12;
  return res*sigma + mu;
}


//assumes the tangents are already normalized
void rotate_between_tangents(const Vector3d& start_tan, const Vector3d& end_tan, Matrix3d& rotation)
{
  if ( (start_tan-end_tan).norm() < 0.01 )
  {
    rotation = Matrix3d::Identity();
  }
  else
  {
    Vector3d toRotAxis;
    toRotAxis = start_tan.cross(end_tan);
    double Axisnorm = toRotAxis.norm();
    double toRotAng = asin(Axisnorm);

    toRotAxis /= Axisnorm;
   // std::cout << "axis: " << toRotAxis << std::endl;
   // std::cout << "ang: " << toRotAng << std::endl;

    rotation = (Eigen::AngleAxisd(toRotAng, toRotAxis));
  }

}

void make_vectors_perpendicular(const Vector3d& start, Vector3d& to_be_perp)
{
	to_be_perp = -(to_be_perp - (to_be_perp.dot(start.normalized()))*start.normalized()).normalized();
}

double angle_mismatch(const Matrix3d& Q, const Matrix3d& R)
{
  //std::cout << "Q\n" << Q << "\nR\n" << R << std::endl;
  Matrix3d S = Q.transpose()*R;
  //std::cout << "S\n: " << S << std::endl;
  double thecos = (S.trace()-1.0)/2.0;
  thecos = std::max( std::min ( thecos, 1.0), -1.0);
  //std::cout << "thecos: " << thecos << " leads to angle: " << acos(thecos) << " * " << ( S(1,2) < 0.0 ? -1.0 : 1.0) << std::endl;

  return (acos(thecos) * ( S(1,2) < 0.0 ? -1.0 : 1.0));
}


double angle_diff(const double goal_angle, const double start_angle)
{
  return atan2( sin(goal_angle - start_angle), cos(goal_angle - start_angle));
}

bool compare_vector_norms(Vector3d& first, Vector3d& second)
{
  return (first.norm() < second.norm());
}


void skew_symmetric_for_cross(const Vector3d& vec, Matrix3d& skew_mat)
{
  skew_mat.setZero();
  skew_mat(0,1) = -vec(2);
  skew_mat(1,0) = vec(2);
  skew_mat(0,2) = vec(1);
  skew_mat(2,0) = -vec(1);
  skew_mat(1,2) = -vec(0);
  skew_mat(2,1) = vec(0);
}

//for speed, doesn't set everything to zero (assumes it's set) - only modifies other indices
void skew_symmetric_for_cross_fast(const Vector3d& vec, Matrix3d& skew_mat)
{
  skew_mat(0,1) = -vec(2);
  skew_mat(1,0) = vec(2);
  skew_mat(0,2) = vec(1);
  skew_mat(2,0) = -vec(1);
  skew_mat(1,2) = -vec(0);
  skew_mat(2,1) = vec(0);
}


double distance_between_points(Vector3d point1, Vector3d point2){
	return (point1 - point2).squaredNorm();
}

double calculate_magnitude_squared(vector<Vector3d>& to_mag)
{
	double mag = 0.0;
	for (int i=0; i < to_mag.size(); i++)
	{
		mag += to_mag[i].squaredNorm();
	}

	return mag;
}



double calculate_vector_diff_norm(vector<Vector3d>& pts1, vector<Vector3d>& pts2)
{
	double mag = 0.0;
	for (int i=0; i < pts1.size(); i++)
	{
		mag += (pts1[i]-pts2[i]).squaredNorm();
	}

	return sqrt(mag);
}

double calculate_vector_norm_avg(vector<Vector3d>& pts1, vector<Vector3d>& pts2)
{
	double mag = 0.0;
	for (int i=0; i < pts1.size(); i++)
	{
		mag += (pts1[i]-pts2[i]).norm();
	}

	return mag / ((double)pts1.size());
}


double angle_between(const Vector3d& tan1, const Vector3d& tan2)
{
	double for_ang = (tan1.dot(tan2))/(tan1.norm()*tan2.norm());
	for_ang = max( min ( for_ang, 1.0), -1.0);
	return acos(for_ang);
}


void rotation_from_euler_angles(Matrix3d& rotation, double angZ, double angY, double angX)
{
  Matrix3d rot1(Eigen::AngleAxisd(angZ, Vector3d::UnitZ()));
  Vector3d axis2 = rot1*(Vector3d::UnitY());
  Matrix3d rot2 = Eigen::AngleAxisd(angY, axis2)*rot1;
  Vector3d axis3 = rot2*(Vector3d::UnitX());
  rotation = Eigen::AngleAxisd(angX, axis3)*rot2;
}

void euler_angles_from_rotation(const Matrix3d& rotation, double& angZ, double& angY, double& angX)
{
  angZ = atan2(rotation(1,0), rotation(0,0));
  angY = atan2(-rotation(2,0), sqrt(rotation(2,1)*rotation(2,1) + rotation(2,2)*rotation(2,2)));
  angX = atan2(rotation(2,1), rotation(2,2));
}

void intermediate_rotation(Matrix3d &inter_rot, const Matrix3d& end_rot, const Matrix3d& start_rot) {
	Quaterniond start_q(start_rot);
  Quaterniond end_q(end_rot);
  Quaterniond interp_q = start_q.slerp(0.5, end_q);
  inter_rot = interp_q.toRotationMatrix();
}

// assumes tan is normalized
void rotation_from_tangent(const Vector3d& tan, Matrix3d& rot) {
  rot.col(0) = tan;
  int max_coor = 0, min_coor = 0;
  for (int i = 1; i < 3; i++) {
    if (tan(i) > tan(max_coor))
      max_coor = i;
    if (tan(i) <= tan(min_coor))
      min_coor = i;
  }
  Vector3d arb = tan;
  arb(max_coor) *= 0.1;
  arb(min_coor) *= 10.0;
  rot.col(1) = arb - arb.dot(tan) * tan;
  rot.col(1).normalize();
  rot.col(2) = rot.col(0).cross(rot.col(1));
  rot.col(2).normalize();
}

bool almost_equal(const Vector3d &a, const Vector3d &b) {
	Vector3d diff = a-b;
	double eps = 0.00001;
	return ((abs(diff(0)) < eps) && (abs(diff(1)) < eps) && (abs(diff(2)) < eps));
}

//Finds element e in vector v. If found, makes the element equal to NULL and returns the position where it was found. If not found, returns -1
template<typename T>
int findInvalidate(vector<T* > v, T* e) {
	int i;
	for (i=0; i<v.size() && v[i]!=e; i++) {}
	if (i==v.size())
		return -1;
	v[i] = NULL;
	return i;
}


Frame_Motion::Frame_Motion(const Vector3d& pos_movement, const Matrix3d& frame_rotation)
  :_pos_movement(pos_movement), _frame_rotation(frame_rotation)
{
}


Frame_Motion::Frame_Motion(const Vector3d& pos_movement, const Vector3d& rotation_axis, const double rotation_ang)
  :_pos_movement(pos_movement)
{
  _frame_rotation = Eigen::AngleAxisd(rotation_ang, rotation_axis);
}

void Frame_Motion::applyMotion(Vector3d& pos, Matrix3d& frame)
{
  pos += _pos_movement;
  frame = _frame_rotation*frame;
}

Frame_Motion operator+(const Frame_Motion& lhs, const Frame_Motion& rhs)
{
  Frame_Motion copy(rhs);
  copy._pos_movement += lhs._pos_movement;
  copy._frame_rotation *= lhs._frame_rotation;
  return copy;
}

Frame_Motion& Frame_Motion::operator=(const Frame_Motion& rhs)
{
  _pos_movement = rhs._pos_movement;
  _frame_rotation = rhs._frame_rotation;
}

void readParams(std::string file, double* out) {
	std::ifstream in;
  in.open(file.c_str());
  if (!in)
  {
    std::cout << "file does not exist: " << file << std::endl;
    exit(0);
  }

  std::cout << file.c_str() << std::endl;
  char str[80];
  double a;
  int counter = 0;
  while(!in.eof()) {
    in >> str;
    in >> out[counter];
    counter++;
  }
  in.close();
}

void writeParams(std::string file, double* towrite) {
	std::ofstream out;
  out.open(file.c_str());
  if (!out)
  {
    std::cout << "file could not be opened: " << file << std::endl;
    exit(0);
  }
  std::cout << file.c_str() << std::endl;
  out << "BEND_COEFF " << towrite[0] << std::endl
      << "TWIST_COEFF " << towrite[1] << std::endl
      << "GRAV_COEFF " << towrite[2] << std::endl;
  out.close();
}


Two_Motions::Two_Motions(const Vector3d& pos_movement_start, const Matrix3d& frame_rotation_start,const Vector3d& pos_movement_end, const Matrix3d& frame_rotation_end)
{
  _start._pos_movement = pos_movement_start;
  _start._frame_rotation = frame_rotation_start;
  _end._pos_movement = pos_movement_end;
  _end._frame_rotation = frame_rotation_end;
}

Two_Motions& Two_Motions::operator=(const Two_Motions& rhs)
{
  _start = rhs._start;
  _end = rhs._end;
}


Two_Motions operator+(const Two_Motions& lhs, const Two_Motions& rhs)
{
  Two_Motions copy(lhs);
  copy._start = copy._start + rhs._start;
  copy._end = copy._end + rhs._end;
  return copy;
}
