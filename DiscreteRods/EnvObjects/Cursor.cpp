#include "Cursor.h"
#include "../threadpiece_discrete.h"
#include "World.h"
#include "../ThreadConstrained.h"

Cursor::Cursor(const Vector3d& pos, const Matrix3d& rot)
	: EnvObject(pos, rot, 0.0, 0.0, 0.0, CURSOR)
	, end_eff(NULL)
	, height(3)
	, radius(2)
	,	attach_dettach_attempt(false)
	, open(false)
	, last_open(false)
	, backup(NULL)
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

// TODO use EnvObject::copydata(rhs) ; i.e. a base clase copy data
Cursor::Cursor(const Cursor& rhs)
	: EnvObject(rhs.position, rhs.rotation, rhs.color0, rhs.color0, rhs.color0, rhs.type)
	, end_eff(NULL)
	, end_eff_ind(rhs.end_eff_ind)
	, height(rhs.height)
	, radius(rhs.radius)
	,	attach_dettach_attempt(rhs.attach_dettach_attempt)
	, open(rhs.open)
	, last_open(rhs.last_open)
	, backup(NULL)
{
	type = rhs.type;
	if (type != CURSOR)
		cerr << " it is not cursor type" << endl;
	i_obj = new Intersection_Object();
	i_obj->_radius = radius;
  i_obj->_start_pos = position;
  i_obj->_end_pos = position - height * rotation.col(0);
	if (rhs.backup != NULL)
		backup = new CursorState(*(rhs.backup));
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
  file << (double)end_eff_ind << " " << open << " ";
  file << "\n";
}

//linkPointersFromInd(World* world) shouled be called after calling this
Cursor::Cursor(ifstream& file)
	: end_eff(NULL)
	, height(3)
	, radius(2)
	,	attach_dettach_attempt(false)
	, backup(NULL)
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
  
  file >> end_eff_ind >> open;
  last_open = open;

	i_obj = new Intersection_Object();
	i_obj->_radius = radius;
  i_obj->_start_pos = position;
  i_obj->_end_pos = position - height * rotation.col(0);
}

void Cursor::recomputeFromTransform(const Vector3d& pos, const Matrix3d& rot)
{
	if (isAttached()) {
		end_eff->setTransform(position - EndEffector::grab_offset * rotation.col(0), rotation);
	}
	i_obj->_start_pos = pos;
	i_obj->_end_pos = pos - height * rot.col(0);
}

void Cursor::draw()
{
	const Vector3d before_mid_point = i_obj->_start_pos + (2.0/6.0)*(i_obj->_end_pos - i_obj->_start_pos);
	const Vector3d after_mid_point = i_obj->_start_pos + (4.0/6.0)*(i_obj->_end_pos - i_obj->_start_pos);
	drawCylinder(i_obj->_start_pos, before_mid_point, i_obj->_radius, isAttached()?0.0:0.5, isAttached()?0.5:0.0, 0.0);
	drawCylinder(before_mid_point, (before_mid_point+after_mid_point)/2.0, i_obj->_radius, 0.0, 0.0, 0.0);
	drawCylinder((before_mid_point+after_mid_point)/2.0, after_mid_point, i_obj->_radius, 1.0, 1.0, 1.0);
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

inline bool closeEnough(const Vector3d& my_pos, const Matrix3d& my_rot, const Vector3d& pos, const Matrix3d& rot)
{
	double angle = 2*asin((my_rot.col(0) - rot.col(0)).norm()/2);
  return (((my_pos - pos).norm() < 4.0) && angle < 0.25*M_PI);
}

void Cursor::setWorld(World* w) {
	world = w;
}

void Cursor::attachDettach()
{
	if (isAttached())
		dettach();
	else
		attach();
}

void Cursor::attach()
{
	assert(!isAttached());
	const Vector3d tip_pos = position - EndEffector::grab_offset * rotation.col(0);
	vector<EnvObject*> world_end_effs = world->getEnvObjs(END_EFFECTOR);
	for (int ee_ind = 0; ee_ind < world_end_effs.size(); ee_ind++) {
		EndEffector* ee = dynamic_cast<EndEffector*>(world_end_effs[ee_ind]);
		if (closeEnough(tip_pos, rotation, ee->getPosition(), ee->getRotation())) {
			end_eff = ee;
			if (ee->isThreadAttached()) {
				ee->updateConstraint();
				if ((ee->constraint==0 || ee->constraint==(ee->getThread()->numVertices()-1)) && isOpen()) 
  				forceClose();
			}
			end_eff->attach(this); //TODO delete this line
			return;
		}
	}
	assert(!isAttached());
}

// Dettaches the cursor from the end effector it is holding. It has to be holding an end effector.
void Cursor::dettach()
{
	assert(isAttached());
	end_eff->dettach(); // TODO delete this line
	end_eff = NULL;
	vector<EnvObject*> world_end_effs = world->getEnvObjs(END_EFFECTOR);
	for (int ee_ind = 0; ee_ind < world_end_effs.size(); ee_ind++) {
		(dynamic_cast<EndEffector*>(world_end_effs[ee_ind]))->updateConstraintIndex();
	}
}

void Cursor::openClose()
{
	if (open) {
		closeCursor();
	} else {
		openCursor();
	}
}

void Cursor::closeCursor()
{
	assert(open);
	const Vector3d tip_pos = position - EndEffector::grab_offset * rotation.col(0);
	if (isAttached()) {
	  if (!end_eff->isThreadAttached()) {
	  	vector<ThreadConstrained*> threads = *(world->getThreads());
	  	int nearest_vertex = threads[0]->nearestVertex(tip_pos);
	  	ThreadConstrained* thread = threads[0];
	  	for (int thread_ind = 1; thread_ind < threads.size(); thread_ind++) {
	  		int candidate_nearest_vertex = threads[thread_ind]->nearestVertex(tip_pos);
	  		if ( (threads[thread_ind]->position(candidate_nearest_vertex) - tip_pos).squaredNorm() <
	  				 (threads[thread_ind]->position(nearest_vertex) - tip_pos).squaredNorm() ) {	  		
	  			nearest_vertex = candidate_nearest_vertex;
	  			thread = threads[thread_ind];
	  		}
	  	}
	  	if ((thread->position(nearest_vertex) - tip_pos).squaredNorm() < 32.0) {												// cursor has an end effector which just started holding the thread
	  		end_eff->constraint = nearest_vertex;
		    thread->addConstraint(nearest_vertex);
		    end_eff->attachThread(thread);
		    vector<EnvObject*> world_end_effs = world->getEnvObjs(END_EFFECTOR);
				for (int ee_ind = 0; ee_ind < world_end_effs.size(); ee_ind++) {
					(dynamic_cast<EndEffector*>(world_end_effs[ee_ind]))->updateConstraintIndex();
				}
		    thread->setConstrainedTransforms(end_eff->constraint_ind, tip_pos, rotation);										// the end effector's orientation matters when it grips the thread. This updates the offset rotation.
		    end_eff->setTransform(tip_pos, rotation); //TODO should I use the updated transform from previous line?
		  }
		} else {
			assert(0); //impossible to close cursor when the end effector is already holding the thread
		}
		last_open = open; //TODO should no longer be needed
		open = false;
		end_eff->setTransform(tip_pos, rotation);
	} else {
		last_open = open; //TODO should no longer be needed
		open = false;
	}
}

void Cursor::openCursor()
{
	assert(!open);
	if (isAttached()) {																																													// cursor has an end effector
	  if (end_eff->isThreadAttached()) {																																							// cursor has an end effector which is holding the thread
	  	if (end_eff->constraint==0 || end_eff->constraint==(end_eff->getThread()->numVertices()-1)) {		// cursor has an end effector which is holding the thread end and trying to be opened
			 	last_open = open = false;
			} else {																																// cursor has an end effector which is holding the thread and trying to be opened
			  end_eff->getThread()->removeConstraint(end_eff->constraint);
			 	end_eff->constraint = end_eff->constraint_ind = -1;
			 	end_eff->dettachThread();
			 	vector<EnvObject*> world_end_effs = world->getEnvObjs(END_EFFECTOR);
				for (int ee_ind = 0; ee_ind < world_end_effs.size(); ee_ind++) {
					(dynamic_cast<EndEffector*>(world_end_effs[ee_ind]))->updateConstraintIndex();
				}
				last_open = open; //TODO should no longer be needed
				open = true;
			}
		} else {
			last_open = open; //TODO should no longer be needed
			open = true;
		}
		end_eff->setTransform(position - EndEffector::grab_offset * rotation.col(0), rotation);
	} else {
		last_open = open; //TODO should no longer be needed
		open = true;
	}
}

void Cursor::saveToBackup()
{
	if (backup != NULL)
		delete backup;
	backup = new CursorState(*this);
}

void Cursor::restoreFromBackup()
{
	if (backup == NULL)
		cerr << "Internal Error: EndEffector::restoreFromBackup(): unable to restore because end effector has not been saved." << endl;
	setTransform(backup->position, backup->rotation);
	end_eff = backup->end_eff;
	height = backup->height;
	radius = backup->radius;
	attach_dettach_attempt = backup->attach_dettach_attempt;
	open = backup->open;
	last_open = backup->last_open;
}

CursorState::CursorState(const Cursor& rhs)
{
	position = rhs.position;
	rotation = rhs.rotation;
	end_eff = rhs.end_eff;
	height = rhs.height;
	radius = rhs.radius;
	attach_dettach_attempt = rhs.attach_dettach_attempt;
	open = rhs.open;
	last_open = rhs.last_open;
}
	
CursorState::CursorState(const CursorState& rhs)
{
	position = rhs.position;
	rotation = rhs.rotation;
	end_eff = rhs.end_eff;
	height = rhs.height;
	radius = rhs.radius;
	attach_dettach_attempt = rhs.attach_dettach_attempt;
	open = rhs.open;
	last_open = rhs.last_open;
}
