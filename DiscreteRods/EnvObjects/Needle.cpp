#include "Needle.h"

Needle::Needle(const Vector3d& pos, const Matrix3d& rot, double degrees, double r, float c0, float c1, float c2, World* w)
	: EnvObject(c0, c1, c2, NEEDLE)
	, angle(degrees)
	, radius(r)
	, world(w)
{
	assert(degrees > MIN_ANGLE);
	assert(r > MIN_RADIUS);

	double arc_length = radius * angle * M_PI/180.0;
	int pieces = ceil(arc_length/2.0);
  for (int piece=0; piece<pieces; piece++) {
 		Intersection_Object* obj = new Intersection_Object();
 		obj->_radius = radius/8.0;
  	i_objs.push_back(obj);
	}	
	setTransform(pos, rot);
}

Needle::Needle(const Vector3d& pos, const Vector3d& start_pos, const Vector3d& end_pos, float c0, float c1, float c2, World* w)
	: EnvObject(c0, c1, c2, NEEDLE)
	, world(w)
{
	double center_start_length = (start_pos - pos).norm();
	double center_end_length = (end_pos - pos).norm();
	assert(center_start_length > MIN_RADIUS);
	assert((center_start_length - center_end_length) < 1e-10);
	radius = center_start_length;
	rotation.col(2) = (start_pos - pos).normalized();
	rotation.col(1) = ((end_pos - pos).normalized()).cross(rotation.col(2));
	rotation.col(0) = rotation.col(1).cross(rotation.col(2));
	angle = (180.0/M_PI) * acos((start_pos - pos).dot(end_pos - pos) / (center_start_length * center_end_length));
	assert(angle > MIN_ANGLE);
	
	double arc_length = radius * angle * M_PI/180.0;
	int pieces = ceil(arc_length/2.0);
  for (int piece=0; piece<pieces; piece++) {
 		Intersection_Object* obj = new Intersection_Object();
 		obj->_radius = radius/8.0;
  	i_objs.push_back(obj);
	}	
	setTransform(pos, rotation);
}

Needle::Needle(const Vector3d& pos, const Vector3d& center_start, double degrees, const Vector3d& axis, float c0, float c1, float c2, World* w)
	: EnvObject(pos, Matrix3d::Identity(), c0, c1, c2, NEEDLE)
	, angle(degrees)
	, radius(center_start.norm())
	, world(w)
{
	assert(degrees > MIN_ANGLE);
	assert(radius > MIN_RADIUS);
	rotation.col(2) = center_start.normalized();
	rotation.col(1) = axis.normalized();
	rotation.col(0) = rotation.col(1).cross(rotation.col(2));
	
	double arc_length = radius * angle * M_PI/180.0;
	int pieces = ceil(arc_length/2.0);
  for (int piece=0; piece<pieces; piece++) {
 		Intersection_Object* obj = new Intersection_Object();
 		obj->_radius = radius/8.0;
  	i_objs.push_back(obj);
	}	
	setTransform(pos, rotation);
}

Needle::Needle(const Needle& rhs, World* w)
	: EnvObject(rhs.color0, rhs.color1, rhs.color2, rhs.type)
	, angle(rhs.angle)
	, radius(rhs.radius)
	, world(w)
{
	assert(type == NEEDLE);
  
	double arc_length = radius * angle * M_PI/180.0;
	int pieces = ceil(arc_length/2.0);
  for (int piece=0; piece<pieces; piece++) {
 		Intersection_Object* obj = new Intersection_Object();
 		obj->_radius = radius/8.0;
  	i_objs.push_back(obj);
	}	
	setTransform(rhs.position, rhs.rotation);
}
		
Needle::~Needle()
{}

void Needle::writeToFile(ofstream& file)
{
	assert(type == NEEDLE);
	file << type << " ";
	for (int i=0; i<3; i++)
		file << position(i) << " ";
	for (int r=0; r < 3; r++)
    for (int c=0; c < 3; c++)
      file << rotation(r,c) << " ";
  file << angle << " " << radius << " " << color0 << " " << color1 << " " << color2 << " ";
  file << "\n";
}

Needle::Needle(ifstream& file, World* w)
{
  world = w;
  type = NEEDLE;
  
	for (int i=0; i<3; i++)
		file >> position(i);
	for (int r=0; r < 3; r++)
    for (int c=0; c < 3; c++)
      file >> rotation(r,c);

  file >> angle >> radius >> color0 >> color1 >> color2;
  
  double arc_length = radius * angle * M_PI/180.0;
	int pieces = ceil(arc_length/2.0);
  for (int piece=0; piece<pieces; piece++) {
 		Intersection_Object* obj = new Intersection_Object();
 		obj->_radius = radius/8.0;
  	i_objs.push_back(obj);
	}
	setTransform(position, rotation);
}

void Needle::setTransform(const Vector3d& pos, const Matrix3d& rot)
{
	position = pos;
	rotation = rot;
	
	double angle_per_link = angle/i_objs.size();
	
	i_objs[0]->_start_pos = position + radius * rotation.col(2);
	i_objs[0]->_end_pos = position + radius * (rotation * AngleAxisd(-angle_per_link * M_PI/180.0, rotation.col(1))).col(2);
	for (int piece=1; piece<i_objs.size(); piece++) {
 		i_objs[piece]->_start_pos = i_objs[piece-1]->_end_pos;
		i_objs[piece]->_end_pos = position + radius * (rotation * AngleAxisd(-angle_per_link * (piece+1) * M_PI/180.0, rotation.col(1))).col(2);
	}
}

Vector3d Needle::getStartPosition()
{
	return position + radius * rotation.col(2);
}

Vector3d Needle::getEndPosition()
{
	return position + radius * (rotation * AngleAxisd(-angle * M_PI/180.0, rotation.col(1))).col(2);
}

double Needle::getAngle()
{
	return angle;
}

double Needle::getRadius()
{
	return radius;
}

Vector3d Needle::getAxis()
{
	return rotation.col(1);
}

void Needle::rotateAboutAxis(double degrees)
{
	rotation = rotation * AngleAxisd(degrees * M_PI/180.0, rotation.col(1));
}

void Needle::draw()
{
	glColor3f(color0, color1, color2);
 	int obj_ind;
  for (obj_ind = 0; obj_ind<i_objs.size(); obj_ind++) {
 		drawCylinder(i_objs[obj_ind]->_start_pos, i_objs[obj_ind]->_end_pos, i_objs[obj_ind]->_radius);
		drawSphere(i_objs[obj_ind]->_start_pos, i_objs[obj_ind]->_radius);
	}	
	drawSphere(i_objs[obj_ind-1]->_end_pos, i_objs[obj_ind-1]->_radius);
}

void Needle::backup()
{
	backup_position = position;
	backup_rotation = rotation;
}

// caller is responsible for having backedup before restoring
void Needle::restore()
{
	position = backup_position;;
	rotation = backup_rotation;
}

bool Needle::capsuleIntersection(int capsule_ind, const Vector3d& start, const Vector3d& end, const double radius, vector<Intersection>& intersections)
{
	return false;
}

double Needle::capsuleRepulsionEnergy(const Vector3d& start, const Vector3d& end, const double radius)
{
	return 0.0;
}

void Needle::capsuleRepulsionEnergyGradient(const Vector3d& start, const Vector3d& end, const double radius, Vector3d& gradient)
{
	return;
}
