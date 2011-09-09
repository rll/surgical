#include "Cursor.h"
#include "World.h"
#include "../threadpiece_discrete.h"
#include "../ThreadConstrained.h"

Cursor::Cursor(const Vector3d& pos, const Matrix3d& rot, World* w, EndEffector* ee)
	: position(pos)
	, rotation(rot)
	, world(w)
	, end_eff(ee)
	, type(CURSOR)
{
	if (isAttached() && ee->isOpen()) {
		open = false;			//need this in order to satisfy assertion
		setOpen();
	} else {
		open = true;
		setClose();
	}
}

Cursor::Cursor(const Cursor& rhs, World* w)
	: position(rhs.position)
	, rotation(rhs.rotation)
	, world(w)
	, end_eff(rhs.end_eff)
	, type(CURSOR)
{
	if (isAttached()) {
		end_eff = world->objectAtIndex<EndEffector>(rhs.world->objectIndex<EndEffector>(rhs.end_eff));
		if (end_eff->isOpen()) {
			//open = false;
			//setOpen();
			open = true;
		} else {
			//open = true;
			//setClose();
			open = false;
		}
	} else {
		//open = true;
		//setClose();
		open = false;
	}
}

Cursor::~Cursor()
{}

void Cursor::writeToFile(ofstream& file)
{
	assert(type == CURSOR);
	file << type << " ";
	for (int i=0; i<3; i++)
		file << position(i) << " ";
	for (int r=0; r < 3; r++)
    for (int c=0; c < 3; c++)
      file << rotation(r,c) << " ";
  file << " " << world->objectIndex<EndEffector>(end_eff) << " " << open << " ";
  file << "\n";
}

Cursor::Cursor(ifstream& file, World* w)
	: world(w)
	, end_eff(NULL)
	, type(CURSOR)
{
	for (int i=0; i<3; i++)
		file >> position(i);
	for (int r=0; r < 3; r++)
    for (int c=0; c < 3; c++)
      file >> rotation(r,c);
      
	int end_eff_ind;
	file >> end_eff_ind >> open;
	end_eff = world->objectAtIndex<EndEffector>(end_eff_ind);
}

void Cursor::setTransform(const Vector3d& pos, const Matrix3d& rot, bool limit_displacement)
{
	position = pos - EndEffector::grab_offset * rot.col(0) ;
	rotation = rot;
	if (isAttached()) {
		//end effector is holding needle, which is going through the box
//    bool needle_box_constrained = false;
//    if (end_eff->isNeedleAttached()) {
//    	cout << "needle attached" << endl;
//    	vector<Box*> boxes;
//    	world->getObjects<Box>(boxes);
//    	Box* box = NULL;
//    	Vector3d direction;
//    	for (int i = 0; i < boxes.size(); i++) {
//    		double dist = sphereBoxDistance(end_eff->needle->getStartPosition(), end_eff->needle->getThicknessRadius(), boxes[i]->getPosition(), boxes[i]->getHalfLength(), direction);
//    		if (dist < 0) {
//    			box = boxes[i];
//    			break;
//    		}
//    	}
//    	if (box) {
//    		needle_box_constrained = true;
//    		cout << "box party" << endl;
//    		//end_eff->needle->rotateAboutAxis(1.0);
//    		//box->attachNeedle();
//    		
//    		
//    	}
//    }
//    
//    if (!needle_box_constrained) {
//    	end_eff->setTransform(position - EndEffector::grab_offset * rotation.col(0), rotation, limit_displacement);
//    }
    
    if (end_eff->isNeedleAttached()) {
    	vector<Box*> boxes;
    	world->getObjects<Box>(boxes);
    	Vector3d direction;
    	for (int i = 0; i < boxes.size(); i++) {
    		if (boxes[i]->isNeedleAttached()) {
    			//thread is not in box yet, but starts to be in the box
		  		if (!boxes[i]->isThreadAttached() && (sphereBoxDistance(end_eff->needle->getEndPosition(), end_eff->needle->getThicknessRadius(), boxes[i]->getPosition(), boxes[i]->getHalfLength(), direction) < 0)) {
		  			boxes[i]->attach(end_eff->needle->getThread());
		  			boxes[i]->constraint0 = 3;
		  			boxes[i]->c0_pos = end_eff->needle->getEndPosition();
		  			boxes[i]->c0_rot = end_eff->needle->getEndRotation();
		  			cout << "constrained first" << endl;
		  		}
		  		//thread is only coming in to the box, but starts to come out of the box
		  		if (boxes[i]->isThreadAttached() && (sphereBoxDistance(end_eff->needle->getStartPosition(), end_eff->needle->getThicknessRadius(), boxes[i]->getPosition(), boxes[i]->getHalfLength(), direction) >= 0)) {
		  			boxes[i]->attach(end_eff->needle->getThread());
		  			boxes[i]->constraint0 = 7;
		  			boxes[i]->c0_pos = end_eff->needle->getEndPosition();
		  			boxes[i]->c0_rot = end_eff->needle->getEndRotation();
		  			boxes[i]->constraint1 = 3;
		  			boxes[i]->c1_pos = end_eff->needle->getStartPosition();
		  			boxes[i]->c1_rot = end_eff->needle->getStartRotation();
		  			cout << "constrained second" << endl;
		  		}
		  		//the needle gets out of the box
		  		if (!end_eff->needle->boxCollision(boxes[i]->getPosition(), boxes[i]->getHalfLength())) {
		  			boxes[i]->dettachNeedle();
		  			cout << "needle gets out of the box" << endl;
		  			break;
		  		}
		  	} else {
		  		double dist = sphereBoxDistance(end_eff->needle->getStartPosition(), end_eff->needle->getThicknessRadius(), boxes[i]->getPosition(), boxes[i]->getHalfLength(), direction);
		  		if (dist < 0) {
		  			boxes[i]->attach(end_eff->needle);
		  			cout << "needle gets into the box" << endl;
		  			break;
		  		}
		  	}
    	}
    }
    
    end_eff->setTransform(position - EndEffector::grab_offset * rotation.col(0), rotation, limit_displacement);
    
    
    
	}
}

const Vector3d& Cursor::getPosition() const
{
	return position;
}

const Matrix3d& Cursor::getRotation() const
{
	return rotation;
}

void Cursor::draw()
{
	Vector3d start_pos;
	Vector3d end_pos;
	Vector3d new_pos;
	Matrix3d new_rot;
	Matrix3d open_rot = (Matrix3d) AngleAxisd(-(open?15.0:0.0)*M_PI/180, rotation*Vector3d::UnitZ());
	double r = 0.9;
	
	new_rot = open_rot * rotation;
	new_pos = open_rot * rotation * Vector3d(-EndEffector::end, 0.0, 0.0) + rotation * Vector3d(EndEffector::end, 0.0, 0.0) + position;
	start_pos = new_rot * Vector3d(EndEffector::start, r, 0.0) + new_pos;
	end_pos = new_rot * Vector3d(EndEffector::start+EndEffector::h, r, 0.0) + new_pos;
	drawColoredCapsule(start_pos, end_pos);


	new_rot = open_rot.transpose() * rotation;
	new_pos = open_rot.transpose() * rotation * Vector3d(-EndEffector::end, 0.0, 0.0) + rotation * Vector3d(EndEffector::end, 0.0, 0.0) + position;
	start_pos = new_rot * Vector3d(EndEffector::start, -r, 0.0) + new_pos;
	end_pos = new_rot * Vector3d(EndEffector::start+EndEffector::h, -r, 0.0) + new_pos;
	drawColoredCapsule(start_pos, end_pos);
	
	drawAxes(position, rotation);
}

void Cursor::drawColoredCapsule(const Vector3d& start_pos, const Vector3d& end_pos)
{
	Vector3d before_mid_point = start_pos + (2.0/6.0)*(end_pos - start_pos);
	Vector3d after_mid_point = start_pos + (4.0/6.0)*(end_pos - start_pos);
	glColor3f(isAttached()?0.0:0.5, isAttached()?0.5:0.0, 0.0);
	drawCylinder(start_pos, before_mid_point, radius);
	glColor3f(0.0, 0.0, 0.0);
	drawCylinder(before_mid_point, (before_mid_point+after_mid_point)/2.0, radius);
	glColor3f(1.0, 1.0, 1.0);
	drawCylinder((before_mid_point+after_mid_point)/2.0, after_mid_point, radius);
	glColor3f(open?0.0:0.5, open?0.5:0.0, 0.0);
	drawCylinder(after_mid_point, end_pos, radius);
	glColor3f(isAttached()?0.0:0.5, isAttached()?0.5:0.0, 0.0);
	drawSphere(start_pos, radius);
	glColor3f(open?0.0:0.5, open?0.5:0.0, 0.0);
	drawSphere(end_pos, radius);
}

inline bool closeEnough(const Vector3d& my_pos, const Matrix3d& my_rot, const Vector3d& pos, const Matrix3d& rot)
{
	double angle = 2*asin((my_rot.col(0) - rot.col(0)).norm()/2);
  return (((my_pos - pos).norm() < 4.0) && angle < 0.25*M_PI);
}

void Cursor::attachDettach(bool limit_displacement)
{
	assert(world!=NULL);
	if (isAttached())
		dettach(limit_displacement);
	else
		attach(limit_displacement);
}

void Cursor::attach(bool limit_displacement)
{
	assert(!isAttached());
	const Vector3d tip_pos = position - EndEffector::grab_offset * rotation.col(0);
	vector<EndEffector*> world_end_effs;
	world->getObjects<EndEffector>(world_end_effs);
	for (int ee_ind = 0; ee_ind < world_end_effs.size(); ee_ind++) {
		EndEffector* ee = world_end_effs[ee_ind];
		if (closeEnough(tip_pos, rotation, ee->getPosition(), ee->getRotation())) {
			end_eff = ee;
			if (ee->isThreadAttached()) {
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
				ee->setTransform(tip_pos, rotation, limit_displacement);
			}
			return;
		}
	}
	assert(!isAttached());
}

// Dettaches the cursor from the end effector it is holding. It has to be holding an end effector.
void Cursor::dettach(bool limit_displacement)
{
	assert(isAttached());
	end_eff = NULL;
	vector<EndEffector*> world_end_effs;
	world->getObjects<EndEffector>(world_end_effs);
	for (int ee_ind = 0; ee_ind < world_end_effs.size(); ee_ind++) {
		world_end_effs[ee_ind]->updateConstraintIndex();
	}
}

void Cursor::openClose(bool limit_displacement)
{
	assert(world!=NULL);
	if (open) {
		setClose(limit_displacement);
	} else {
		setOpen(limit_displacement);
	}
}

void Cursor::setOpen(bool limit_displacement)
{
	assert(!open);
	if (isAttached()) {																																													// cursor has an end effector
	  if (end_eff->isThreadAttached()) {																																							// cursor has an end effector which is holding the thread
	  	if (end_eff->constraint==0 || end_eff->constraint==(end_eff->getThread()->numVertices()-1)) {		// cursor has an end effector which is holding the thread end and trying to be opened
			 	open = false;
			 	assert(!end_eff->isOpen());
			} else {																																// cursor has an end effector which is holding the thread and trying to be opened
			  end_eff->getThread()->removeConstraint(end_eff->constraint);
			 	end_eff->constraint = end_eff->constraint_ind = -1;
			 	end_eff->dettachThread();	
				vector<EndEffector*> world_end_effs;
				world->getObjects<EndEffector>(world_end_effs);
				for (int ee_ind = 0; ee_ind < world_end_effs.size(); ee_ind++) {
					world_end_effs[ee_ind]->updateConstraintIndex();
				}
				open = true;
				end_eff->setOpen();
			}
		} else if (end_eff->isNeedleAttached()) {
			end_eff->dettachNeedle();
			open = true;
			end_eff->setOpen();
		}	else {
			open = true;
			end_eff->setOpen();
		}
		end_eff->setTransform(position - EndEffector::grab_offset * rotation.col(0), rotation, limit_displacement);
	} else {
		open = true;
	}
}

void Cursor::setClose(bool limit_displacement)
{
	assert(open);
	if (isAttached()) {
		const Vector3d tip_pos = position - EndEffector::grab_offset * rotation.col(0);
	  if (!end_eff->isThreadAttached() && !end_eff->isNeedleAttached()) {
	  	vector<ThreadConstrained*> threads;
	  	world->getObjects<ThreadConstrained>(threads);
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
	  	
	  	vector<Needle*> needles;
	  	world->getObjects<Needle>(needles);
	  	Vector3d nearest_position;
	  	Needle* needle;
	  	if (needles.size() > 0) {
				nearest_position = needles[0]->nearestPosition(tip_pos);
				needle = needles[0];
				for (int needle_ind; needle_ind < needles.size(); needle_ind++) {
					if ( (needles[needle_ind]->nearestPosition(tip_pos) - tip_pos).squaredNorm() <
							 (nearest_position - tip_pos).squaredNorm() ) {	  		
						nearest_position = needles[needle_ind]->nearestPosition(tip_pos);
						needle = needles[needle_ind];
					}
				}
	  	}
	  	
	  	if ((needles.size() <= 0) || (thread->position(nearest_vertex) - tip_pos).squaredNorm() < (nearest_position - tip_pos).squaredNorm()) {
				if ((thread->position(nearest_vertex) - tip_pos).squaredNorm() < 32.0) {												// cursor has an end effector which just started holding the thread
					end_eff->constraint = nearest_vertex;
				  end_eff->constraint_ind = thread->addConstraint(nearest_vertex);
				  end_eff->attach(thread);
				  
				  vector<EndEffector*> world_end_effs;
					world->getObjects<EndEffector>(world_end_effs);
				  for (int ee_ind = 0; ee_ind < world_end_effs.size(); ee_ind++) {
						if ((world_end_effs[ee_ind]->thread == thread) && (world_end_effs[ee_ind]->constraint_ind >= end_eff->constraint_ind) && (world_end_effs[ee_ind]!=end_eff)) {
							world_end_effs[ee_ind]->constraint_ind++;
						}
						world_end_effs[ee_ind]->updateConstraint();
					}

					thread->updateRotationOffset(end_eff->constraint_ind, rotation);	// the end effector's orientation matters when it grips the thread. This updates the offset rotation.
				  end_eff->setTransform(tip_pos, rotation, limit_displacement);
				}
			} else {
				if ((nearest_position - tip_pos).squaredNorm() < 8.0) {												// cursor has an end effector which just started holding the needle
					end_eff->attach(needle);
					needle->setTransformOffsetFromEndEffector(nearest_position, rotation);
					end_eff->setTransform(tip_pos, rotation, limit_displacement);
				}
			}
		}	else {
			//impossible to close cursor when the end effector is already holding the thread.
			//however, this happens when a world is copyied and probably in other ocasions too.
		}
		open = false;
		end_eff->setClose();
		end_eff->setTransform(tip_pos, rotation, limit_displacement);
	} else {
		open = false;
	}
}
