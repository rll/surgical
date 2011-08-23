#include "Cursor.h"
#include "../threadpiece_discrete.h"

Cursor::Cursor(const Vector3d& pos, const Matrix3d& rot)
	: EnvObject(pos, rot, 0.0, 0.0, 0.0, CURSOR)
	, end_eff(NULL)
	, height(3)
	, radius(2)
	,	attach_dettach_attempt(false)
	, open(false)
	, last_open(false)
{
	i_obj = new Intersection_Object();
	i_obj->_radius = radius;
  i_obj->_start_pos = pos;
  i_obj->_end_pos = pos - height * rot.col(0);
}

Cursor::~Cursor()
{
	delete i_obj;
	i_obj = NULL;
}

//updateIndFromPointers(World* world) should have been called before calling this
void Cursor::writeToFile(ofstream& file)
{
	file << type << " ";
	for (int i=0; i<3; i++)
		file << position(i) << " ";
	for (int r=0; r < 3; r++)
  {
    for (int c=0; c < 3; c++)
    {
      file << rotation(r,c) << " ";
    }
  }
  file << (double)end_eff_ind << " ";
  file << "\n";
}

//linkPointersFromInd(World* world) shouled be called after calling this
Cursor::Cursor(ifstream& file)
	: end_eff(NULL)
	, height(3)
	, radius(2)
	,	attach_dettach_attempt(false)
	, open(false)
	, last_open(false)
{
	
  color0 = color1 = color2 = 0.0;
  type = CURSOR;
  
	for (int i=0; i<3; i++) {
		file >> position(i);
	}
	for (int r=0; r < 3; r++)
  {
    for (int c=0; c < 3; c++)
    {
      file >> rotation(r,c);
    }
  }
  
  file >> end_eff_ind;

	i_obj = new Intersection_Object();
	i_obj->_radius = radius;
  i_obj->_start_pos = position;
  i_obj->_end_pos = position - height * rotation.col(0);
}

void Cursor::updateIndFromPointers(World* world)
{
	if (end_eff == NULL) {
		end_eff_ind = -1;
	} else {
		vector<EnvObject*> end_effs = world->getEnvObjs(END_EFFECTOR);
		for (end_eff_ind = 0; end_eff_ind<end_effs.size(); end_eff_ind++) {
			EndEffector* ee = dynamic_cast<EndEffector*>(end_effs[end_eff_ind]);
			if (ee == end_eff)
				break;
		}
		if (end_eff_ind == end_effs.size())
			cout << "Internal error: Cursor::updateIndFromPointers: end_eff was not in world->getEnvObjs(END_EFFECTOR)." << endl;
	}
}

void Cursor::linkPointersFromInd(World* world)
{
	if (end_eff_ind == -1) {
		end_eff = NULL;
	} else {
		vector<EnvObject*> end_effs = world->getEnvObjs(END_EFFECTOR);
		if (end_eff_ind < 0 || end_eff_ind >= end_effs.size())
			cout << "Internal error: Cursor::linkPointersFromInd: end_eff_ind " << end_eff_ind << " is out of bounds." << endl;
		end_eff = dynamic_cast<EndEffector*>(end_effs[end_eff_ind]);
	}
}

void Cursor::recomputeFromTransform(const Vector3d& pos, const Matrix3d& rot)
{
	i_obj->_start_pos = pos;
	i_obj->_end_pos = pos - height * rot.col(0);
}

void Cursor::draw()
{
	Vector3d before_mid_point = i_obj->_start_pos + (2.0/5.0)*(i_obj->_end_pos - i_obj->_start_pos);
	Vector3d after_mid_point = i_obj->_start_pos + (3.0/5.0)*(i_obj->_end_pos - i_obj->_start_pos);
	drawCylinder(i_obj->_start_pos, before_mid_point, i_obj->_radius, isAttached()?0.0:0.5, isAttached()?0.5:0.0, 0.0);
	drawCylinder(before_mid_point, after_mid_point, i_obj->_radius, 0.0, 0.0, 0.4);
	drawCylinder(after_mid_point, i_obj->_end_pos, i_obj->_radius, open?0.0:0.5, open?0.5:0.0, 0.0);
	drawSphere(i_obj->_start_pos, i_obj->_radius, isAttached()?0.0:0.5, isAttached()?0.5:0.0, 0.0);
	drawSphere(i_obj->_end_pos, i_obj->_radius, open?0.0:0.5, open?0.5:0.0, 0.0);
}

bool Cursor::capsuleIntersection(int capsule_ind, const Vector3d& start, const Vector3d& end, const double radius, vector<Intersection>& intersections)
{
  return false;
  Vector3d direction;
  double intersection_dist = capsuleCapsuleDistance(start, end, radius, i_obj->_start_pos, i_obj->_end_pos, i_obj->_radius, direction);
  if(intersection_dist < 0) {
    intersections.push_back(Intersection(capsule_ind, -intersection_dist, direction));
    return true;
  }
	return false;
}

double Cursor::capsuleRepulsionEnergy(const Vector3d& start, const Vector3d& end, const double radius)
{
	return 0.0;
	if (REPULSION_COEFF <= 0.0) { return 0.0; }
	Vector3d direction;
	double dist = capsuleCapsuleDistance(start, end, radius, i_obj->_start_pos, i_obj->_end_pos, i_obj->_radius, direction);
	if (dist < 0 || dist > radius)
		return 0.0;
	return REPULSION_COEFF/2.0 * pow(dist-radius,2);
}

void Cursor::capsuleRepulsionEnergyGradient(const Vector3d& start, const Vector3d& end, const double radius, Vector3d& gradient)
{
	return;
	if (REPULSION_COEFF <= 0.0) { return; }
	Vector3d direction;
	double dist = capsuleCapsuleDistance(start, end, radius, i_obj->_start_pos, i_obj->_end_pos, i_obj->_radius, direction);
	if (dist < 0 || dist > radius)
		return;
	gradient -= REPULSION_COEFF * (radius - dist) * direction.normalized();
}

void Cursor::attach(EndEffector* ee)
{
	end_eff = ee;
	end_eff->attach(this);
}

// Dettaches the cursor from the end effector it is holding. It has to be holding an end effector.
void Cursor::dettach()
{
	if (end_eff == NULL)
		cout << "Internal errror: Cursor::dettach(): cursor cannot dettach since it does't have an end effector attached" << endl;
	end_eff->dettach();
	end_eff = NULL;
}
