#include "Cursor.h"
#include "../threadpiece_discrete.h"
#include "World.h"
#include "../ThreadConstrained.h"

Cursor::Cursor(const Vector3d& pos, const Matrix3d& rot, World* w, EndEffector* ee)
	: position(pos)
	, rotation(rot)
	, world(w)
	, end_eff(ee)
{
	if (ee!=NULL) {
		open = ee->isOpen();
	}
	//TODO should i call this->setOpen() instead?

}

Cursor::Cursor(const Cursor& rhs, World* w)
	: position(rhs.position)
	, rotation(rhs.rotation)
	, world(w)
	, end_eff(rhs.end_eff)
	, open(false)
{
	if (end_eff != NULL) {
		end_eff = dynamic_cast<EndEffector*>(world->envObjAtIndex(rhs.world->envObjIndex(rhs.end_eff)));
		open = end_eff->isOpen();
	}
	//TODO should i call this->setOpen() instead?
}

Cursor::~Cursor()
{}

void Cursor::setTransform(const Vector3d& pos, const Matrix3d& rot, bool limit_displacement)
{
	position = pos;
	rotation = rot;
	if (isAttached()) {
		end_eff->setTransform(position - EndEffector::grab_offset * rotation.col(0), rotation, limit_displacement);
	}
}

void Cursor::draw()
{
	const Vector3d end_pos = position - height * rotation.col(0);
	
	const Vector3d before_mid_point = position + (2.0/6.0)*(end_pos - position);
	const Vector3d after_mid_point = position + (4.0/6.0)*(end_pos - position);
	glColor3f(isAttached()?0.0:0.5, isAttached()?0.5:0.0, 0.0);
	drawCylinder(position, before_mid_point, radius);
	glColor3f(0.0, 0.0, 0.0);
	drawCylinder(before_mid_point, (before_mid_point+after_mid_point)/2.0, radius);
	glColor3f(1.0, 1.0, 1.0);
	drawCylinder((before_mid_point+after_mid_point)/2.0, after_mid_point, radius);
	glColor3f(open?0.0:0.5, open?0.5:0.0, 0.0);
	drawCylinder(after_mid_point, end_pos, radius);
	glColor3f(isAttached()?0.0:0.5, isAttached()?0.5:0.0, 0.0);
	drawSphere(position, radius);
	glColor3f(open?0.0:0.5, open?0.5:0.0, 0.0);
	drawSphere(end_pos, radius);
}

inline bool closeEnough(const Vector3d& my_pos, const Matrix3d& my_rot, const Vector3d& pos, const Matrix3d& rot)
{
	double angle = 2*asin((my_rot.col(0) - rot.col(0)).norm()/2);
  return (((my_pos - pos).norm() < 4.0) && angle < 0.25*M_PI);
}

void Cursor::attachDettach()
{
	assert(world!=NULL);
	if (isAttached())
		dettach();
	else
		attach();
}

void Cursor::attach()
{
	assert(!isAttached());
	const Vector3d tip_pos = position - EndEffector::grab_offset * rotation.col(0);
	vector<EnvObject*> world_end_effs;
	world->getEnvObjs(world_end_effs, END_EFFECTOR);
	for (int ee_ind = 0; ee_ind < world_end_effs.size(); ee_ind++) {
		EndEffector* ee = dynamic_cast<EndEffector*>(world_end_effs[ee_ind]);
		if (closeEnough(tip_pos, rotation, ee->getPosition(), ee->getRotation())) {
			end_eff = ee;
			if (ee->isAttached()) {
				ee->updateConstraint();
				if ((ee->constraint==0 || ee->constraint==(ee->getThread()->numVertices()-1)) && isOpen()) {
  				open = false;
  				assert(!ee->isOpen());
  			}
  		}
			//to update the open/close feature display of the end effector
			if (isOpen() != ee->isOpen()) {
				if (isOpen()) {
					ee->setOpen();
				} else {
					ee->setClose();
				}
				ee->setTransform(tip_pos, rotation);
			}
			return;
		}
	}
	assert(!isAttached());
}

// Dettaches the cursor from the end effector it is holding. It has to be holding an end effector.
void Cursor::dettach()
{
	assert(isAttached());
	end_eff = NULL;
	vector<EnvObject*> world_end_effs;
	world->getEnvObjs(world_end_effs, END_EFFECTOR);
	for (int ee_ind = 0; ee_ind < world_end_effs.size(); ee_ind++) {
		(dynamic_cast<EndEffector*>(world_end_effs[ee_ind]))->updateConstraintIndex();
	}
}

void Cursor::openClose()
{
	assert(world!=NULL);
	if (open) {
		setClose();
	} else {
		setOpen();
	}
}

void Cursor::setOpen()
{
	assert(!open);
	if (isAttached()) {																																													// cursor has an end effector
	  if (end_eff->isAttached()) {																																							// cursor has an end effector which is holding the thread
	  	if (end_eff->constraint==0 || end_eff->constraint==(end_eff->getThread()->numVertices()-1)) {		// cursor has an end effector which is holding the thread end and trying to be opened
			 	open = false;
			 	assert(!end_eff->isOpen());
			} else {																																// cursor has an end effector which is holding the thread and trying to be opened
			  end_eff->getThread()->removeConstraint(end_eff->constraint);
			 	end_eff->constraint = end_eff->constraint_ind = -1;
			 	end_eff->dettach();
			 	vector<EnvObject*> world_end_effs;
			 	world->getEnvObjs(world_end_effs, END_EFFECTOR);
				for (int ee_ind = 0; ee_ind < world_end_effs.size(); ee_ind++) {
					(dynamic_cast<EndEffector*>(world_end_effs[ee_ind]))->updateConstraintIndex();
				}
				open = true;
				end_eff->setOpen();
			}
		} else {
			open = true;
			end_eff->setOpen();
		}
		end_eff->setTransform(position - EndEffector::grab_offset * rotation.col(0), rotation);
	} else {
		open = true;
	}
}

void Cursor::setClose()
{
	assert(open);
	if (isAttached()) {
		const Vector3d tip_pos = position - EndEffector::grab_offset * rotation.col(0);
	  if (!end_eff->isAttached()) {
	  	vector<ThreadConstrained*> threads;
	  	world->getThreads(threads);
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
		    end_eff->attach(thread);
		    vector<EnvObject*> world_end_effs;
		    world->getEnvObjs(world_end_effs, END_EFFECTOR);
				for (int ee_ind = 0; ee_ind < world_end_effs.size(); ee_ind++) {
					(dynamic_cast<EndEffector*>(world_end_effs[ee_ind]))->updateConstraintIndex();
				}
		    thread->setConstrainedTransforms(end_eff->constraint_ind, tip_pos, rotation);										// the end effector's orientation matters when it grips the thread. This updates the offset rotation.
		    end_eff->setTransform(tip_pos, rotation); //TODO should I use the updated transform from previous line?
		  }
		} else {
			assert(0); //impossible to close cursor when the end effector is already holding the thread
		}
		open = false;
		end_eff->setClose();
		end_eff->setTransform(tip_pos, rotation);
	} else {
		open = false;
	}
}
