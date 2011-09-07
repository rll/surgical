#include "Box.h"
#include "World.h"
#include "../ThreadConstrained.h"

Box::Box(const Vector3d& pos, const Matrix3d& rot, const Vector3d& half_length_xyz, float c0, float c1, float c2, World* w, Needle* n, ThreadConstrained* t, int constrained_vertex_num0, int constrained_vertex_num1)
	: EnvObject(pos, rot, c0, c1, c2, BOX)
	, half_length(half_length_xyz)
	, thread(t)
	, constraint0(constrained_vertex_num0)
	, constraint1(constrained_vertex_num1)
	, needle(n)
	, world(w)
{
  assert(((thread == NULL) && (constrained_vertex_num0 == -1) && (constrained_vertex_num1 == -1)) || ((thread != NULL) && (constrained_vertex_num0 != -1)));
  setTransform(position, rotation);
}

Box::Box(const Box& rhs, World* w)
	: EnvObject(rhs.position, rhs.rotation, rhs.color0, rhs.color1, rhs.color2, rhs.type)
	, half_length(rhs.half_length)
	, thread(rhs.thread)
	, constraint0(rhs.constraint0)
	, constraint1(rhs.constraint1)
	, needle(rhs.needle)
	, world(w)
{
	assert(type == BOX);
	
	if (thread == NULL) {
		assert(constraint0 == -1);
		assert(constraint1 == -1);
	} else {
		assert(constraint0 != -1);
		//box have only one constraint of the thread if the thread hasn't crossed the box
		thread = world->objectAtIndex<ThreadConstrained>(rhs.world->objectIndex<ThreadConstrained>(rhs.thread));
	}
	if (needle != NULL) {
		needle = world->objectAtIndex<Needle>(rhs.world->objectIndex<Needle>(rhs.needle));
	}
	
  setTransform(position, rotation);
}
		
Box::~Box()
{}

void Box::writeToFile(ofstream& file)
{
	assert(type == BOX);
	file << type << " ";
	for (int r=0; r < 3; r++)
    for (int c=0; c < 3; c++)
      file << rotation(r,c) << " ";
  file << half_length(0) << " " << half_length(1) << " " << half_length(2) << " " << color0 << " " << color1 << " " << color2 << " ";
  file << constraint0 << " " << constraint1 << " " << world->objectIndex<ThreadConstrained>(thread) << " " << world->objectIndex<Needle>(needle) << " ";
  file << "\n";
}

Box::Box(ifstream& file, World* w)
	: thread(NULL)
	, needle(NULL)
	, world(w)
{
  type = BOX;
  
	for (int i=0; i<3; i++)
		file >> position(i);
	for (int r=0; r < 3; r++)
    for (int c=0; c < 3; c++)
      file >> rotation(r,c);

  file >> half_length(0) >> half_length(1) >> half_length(2) >> color0 >> color1 >> color2;
  int world_thread_ind, world_needle_ind;
  file >> constraint0 >> constraint1 >> world_thread_ind >> world_needle_ind;
 	thread = world->objectAtIndex<ThreadConstrained>(world_thread_ind);
 	needle = world->objectAtIndex<Needle>(world_needle_ind);
  
  assert(((thread == NULL) && (constraint0 == -1) && (constraint1 == -1)) || ((thread != NULL) && (constraint0 != -1)));

  setTransform(position, rotation);
}

void Box::setTransform(const Vector3d& pos, const Matrix3d& rot)
{
	position = pos;
	rotation = rot;

	normals.resize(6);
	vertex_positions.resize(6);
	for (int axis_sign_ind = 0; axis_sign_ind < 2; axis_sign_ind++) {
		for (int axis = 0; axis < 3; axis++) {
			float axis_sign = (axis_sign_ind) ? -1 : 1;
			Vector3d vertex_sign[4];
			vertex_sign[0][axis] = vertex_sign[1][axis] = vertex_sign[2][axis] = vertex_sign[3][axis] = axis_sign;
			vertex_sign[0][(axis+1)%3] = vertex_sign[0][(axis+2)%3] = 1;
			vertex_sign[2][(axis+1)%3] = vertex_sign[2][(axis+2)%3] = -1;			
			vertex_sign[1][(axis+2)%3] = vertex_sign[3][(axis+1)%3] = axis_sign * 1;
			vertex_sign[1][(axis+1)%3] = vertex_sign[3][(axis+2)%3] = axis_sign * (-1);

			normals[axis_sign_ind*3 + axis] = axis_sign * rotation.col(axis);
			vertex_positions[axis_sign_ind*3 + axis].resize(4);
			for (int vertex_ind = 0; vertex_ind < 4; vertex_ind++) {
				vertex_positions[axis_sign_ind*3 + axis][vertex_ind] = position + 
																															 vertex_sign[vertex_ind][0]*half_length(0)*rotation.col(0) + 
																															 vertex_sign[vertex_ind][1]*half_length(1)*rotation.col(1) + 
																															 vertex_sign[vertex_ind][2]*half_length(0)*rotation.col(2);
			}
		}
	}
}

void Box::draw()
{
	glPushMatrix();
	glColor3f(color0, color1, color2);
	for (int face_ind = 0; face_ind < 6; face_ind++) {
		for (int vertex_ind = 0; vertex_ind < 4; vertex_ind++) {
			glBegin(GL_QUADS);
			glNormal3f(normals[face_ind](0), normals[face_ind](1), normals[face_ind](2));
			for (int vertex_ind = 0; vertex_ind < 4; vertex_ind++)
			{
				glVertex3f(vertex_positions[face_ind][vertex_ind](0), vertex_positions[face_ind][vertex_ind](1), vertex_positions[face_ind][vertex_ind](2));
			}
			glEnd();
		}
	}
	glPopMatrix();
}

void insertNeedle(Needle* n)
{

}

void stepThread()
{

}

void Box::backup()
{
	backup_position = position;
	backup_rotation = rotation;
	backup_constraint0 = constraint0;
	backup_constraint1 = constraint1;
	backup_thread_ind = world->objectIndex<ThreadConstrained>(thread);
	backup_needle_ind = world->objectIndex<Needle>(needle);
}

// caller is responsible for having backedup before restoring
void Box::restore()
{
	position = backup_position;;
	rotation = backup_rotation;
	constraint0 = backup_constraint0;
	constraint1 = backup_constraint1;
	thread = world->objectAtIndex<ThreadConstrained>(backup_thread_ind);
	needle = world->objectAtIndex<Needle>(backup_needle_ind);
}

bool Box::capsuleIntersection(int capsule_ind, const Vector3d& start, const Vector3d& end, const double radius, vector<Intersection>& intersections)
{
	return false;
}

double Box::capsuleRepulsionEnergy(const Vector3d& start, const Vector3d& end, const double radius)
{
	return 0.0;
}

void Box::capsuleRepulsionEnergyGradient(const Vector3d& start, const Vector3d& end, const double radius, Vector3d& gradient)
{
	return;
}
