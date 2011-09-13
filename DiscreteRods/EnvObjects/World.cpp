#include "World.h"
#include "../thread_discrete.h"
#include "../ThreadConstrained.h"
#include "../../utils/clock2.h"

World::World(WorldManager* wm)
{
	world_manager = wm;
	if (world_manager != NULL)
		collision_world = world_manager->allocateWorld(this);
	else
		collision_world = NULL;

	//any of these pushes two threads into threads.
	//initThread();
  //initLongerThread();
  initRestingThread(0);
	
	//setting up control handles
	cursors.push_back(new Cursor(Vector3d::Zero(), Matrix3d::Identity(), this, NULL));
	cursors.push_back(new Cursor(Vector3d::Zero(), Matrix3d::Identity(), this, NULL));	
	
	//setting up objects in environment
	//InfinitePlane* plane = new InfinitePlane(Vector3d(0.0, -30.0, 0.0), Vector3d(0.0, 1.0, 0.0), "../utils/textures/checkerBoardSquare32.bmp", this);
	InfinitePlane* plane = new InfinitePlane(Vector3d(0.0, -30.0, 0.0), Vector3d(0.0, 1.0, 0.0), 0.6, 0.6, 0.6, this);
	objs.push_back(plane);
	//objs.push_back(new TexturedSphere(Vector3d::Zero(), 150.0, "../utils/textures/checkerBoardRect16.bmp", this));
	
//	objs.push_back(new Box(plane->getPosition() + Vector3d(-40.0, 10.0, 0.0), Matrix3d::Identity(), Vector3d(10,10,10), 0.0, 0.5, 0.7, this));
//	
//	objs.push_back(new Needle(plane->getPosition() + Vector3d(0.0, 50.0, 0.0), Matrix3d::Identity(), 120.0, 10.0, 0.3, 0.3, 0.3, this));
//	objs.push_back(new Needle(threads[0]->positionAtConstraint(0), threads[0]->rotationAtConstraint(0), 120.0, 10.0, 0.3, 0.3, 0.3, this, threads[0], 0));
	
	//setting up end effectors
	objs.push_back(new EndEffector(threads[0]->positionAtConstraint(0), threads[0]->rotationAtConstraint(0), this, threads[0], 0));
	assert((TYPE_CAST<EndEffector*>(objs.back()))->constraint_ind == 0);

	objs.push_back(new EndEffector(threads[0]->positionAtConstraint(1), threads[0]->rotationAtConstraint(1), this, threads[0], threads[0]->numVertices()-1));
	assert((TYPE_CAST<EndEffector*>(objs.back()))->constraint_ind == 1);

	objs.push_back(new EndEffector(threads[1]->positionAtConstraint(0), threads[1]->rotationAtConstraint(0), this, threads[1], 0));
	assert((TYPE_CAST<EndEffector*>(objs.back()))->constraint_ind == 0);
	
	objs.push_back(new EndEffector(threads[1]->positionAtConstraint(1), threads[1]->rotationAtConstraint(1), this, threads[1], threads[1]->numVertices()-1));
	assert((TYPE_CAST<EndEffector*>(objs.back()))->constraint_ind == 1);
	
	objs.push_back(new EndEffector(plane->getPosition() + Vector3d(30.0, EndEffector::short_handle_r, 0.0), (Matrix3d) AngleAxisd(-M_PI/2.0, Vector3d::UnitY()) * AngleAxisd(M_PI/2.0, Vector3d::UnitX()), this));
	assert((TYPE_CAST<EndEffector*>(objs.back()))->constraint_ind == -1);
	assert((TYPE_CAST<EndEffector*>(objs.back()))->constraint == -1);
	
	objs.push_back(new EndEffector(plane->getPosition() + Vector3d(35.0, EndEffector::short_handle_r, 0.0), (Matrix3d) AngleAxisd(-M_PI/2.0, Vector3d::UnitY()) * AngleAxisd(M_PI/2.0, Vector3d::UnitX()), this));
	assert((TYPE_CAST<EndEffector*>(objs.back()))->constraint_ind == -1);
	assert((TYPE_CAST<EndEffector*>(objs.back()))->constraint == -1);
}

World::World(const World& rhs, WorldManager* wm)
{
	world_manager = wm;
	if (world_manager != NULL)
		collision_world = world_manager->allocateWorld(this);
	else
		collision_world = NULL;
	
	threads.clear();
	for (int i = 0; i<rhs.threads.size(); i++) {
		threads.push_back(new ThreadConstrained(*(rhs.threads[i]), this));
	}
	objs.clear();
	for (int i = 0; i<rhs.objs.size(); i++) {
		switch (rhs.objs[i]->getType())
    {
      case END_EFFECTOR:
        {
          objs.push_back(new EndEffector(*(TYPE_CAST<EndEffector*>(rhs.objs[i])), this));
          break;
        }
      case INFINITE_PLANE:
        {
          objs.push_back(new InfinitePlane(*(TYPE_CAST<InfinitePlane*>(rhs.objs[i])), this));
          break;
        }
      case TEXTURED_SPHERE:
        {
          objs.push_back(new TexturedSphere(*(TYPE_CAST<TexturedSphere*>(rhs.objs[i])), this));
          break;
        }
      case BOX:
        {
          objs.push_back(new Box(*(TYPE_CAST<Box*>(rhs.objs[i])), this));
          break;
        }
      case NEEDLE:
        {
          objs.push_back(new Needle(*(TYPE_CAST<Needle*>(rhs.objs[i])), this));
          break;
        }
      default:
      	{
      		assert(0);
      		break;
      	}
    }
  }
  cursors.clear();
	for (int i = 0; i<rhs.cursors.size(); i++) {
		cursors.push_back(new Cursor(*(rhs.cursors[i]), this));
	}
}

World::~World()
{
	for (int i = 0; i<cursors.size(); i++) {
		assert(cursors[i]!=NULL);
		delete cursors[i];
		cursors[i] = NULL;
	}
	for (int i = 0; i<threads.size(); i++) {
		assert(threads[i]!=NULL);
		delete threads[i];
		threads[i] = NULL;
	}
	for (int i = 0; i<objs.size(); i++) {
		assert(objs[i]!=NULL);
		delete objs[i];
		objs[i] = NULL;
	}
	cursors.clear();
	threads.clear();
	objs.clear();
	if (world_manager != NULL)
		world_manager->freeWorld(this);
}

void World::writeToFile(ofstream& file)
{
	for (int i = 0; i < threads.size(); i++)
    threads[i]->writeToFile(file);
  
  for (int i = 0; i < objs.size(); i++)
    objs[i]->writeToFile(file);
    
  for (int i = 0; i < cursors.size(); i++)
    cursors[i]->writeToFile(file);
  
  file << NO_OBJECT << " ";
  file << "\n";
}

World::World(ifstream& file, WorldManager* wm)
{
	world_manager = wm;
	if (world_manager != NULL)
		collision_world = world_manager->allocateWorld(this);
	else
		collision_world = NULL;
	
  int type;
  while (!file.eof()) {
    file >> type;
    switch (type)
    {
      case THREAD_CONSTRAINED:
        {
          threads.push_back(new ThreadConstrained(file, this));
          break;
        }
      case END_EFFECTOR:
        {
          objs.push_back(new EndEffector(file, this));
          break;
        }
      case INFINITE_PLANE:
        {
          objs.push_back(new InfinitePlane(file, this));
          break;
        }
      case TEXTURED_SPHERE:
        {
          objs.push_back(new TexturedSphere(file, this));
          break;
        }
      case BOX:
        {
          objs.push_back(new Box(file, this));
          break;
        }
      case NEEDLE:
        {
          objs.push_back(new Needle(file, this));
          break;
        }
      case CURSOR:
      	{
      		cursors.push_back(new Cursor(file, this));
          break;
      	}
      case NO_OBJECT:
      	{
      		break;
      	}
      default:
      	{
      		assert(0);
      		break;
      	}
    }
    if (type == NO_OBJECT) { break; }
  }
  //cout << endl;
  
}


EndEffector* World::closestEndEffector(Vector3d tip_pos)
{
	vector<EndEffector*> end_effectors;
	getObjects<EndEffector>(end_effectors);
	int min_ee_ind = 0;
	double min_squared_dist = (tip_pos - end_effectors[min_ee_ind]->getPosition()).squaredNorm();
	for (int ee_ind = 1; ee_ind < end_effectors.size(); ee_ind++) {
		double squared_dist = (tip_pos - end_effectors[ee_ind]->getPosition()).squaredNorm(); 
		if (squared_dist < min_squared_dist) {
			min_ee_ind = ee_ind;
			min_squared_dist = squared_dist;
		}
	}
	return end_effectors[min_ee_ind];
	
	//TODO don't return an end effector that already has cursor attached to it	
//	vector<EndEffector*> end_effectors;
//	getObjects<EndEffector>(end_effectors);
//	int min_ee_ind;
//	for (min_ee_ind = 0; min_ee_ind < end_effectors.size(); min_ee_ind++) {
//		bool isCursorNotAttachedToEE = cursors[0]->end_eff!=end_effectors[min_ee_ind];
//		for (int cursor_ind = 1; cursor_ind < cursors.size(); cursor_ind++) {
//			isCursorNotAttachedToEE = isCursorNotAttachedToEE && cursors[cursor_ind]->end_eff!=end_effectors[min_ee_ind];
//		}		
//		if (isCursorNotAttachedToEE)
//			break;
//	}	
//	assert(min_ee_ind != end_effectors.size()); //There is no any end effectors that is not being holded by a cursor
//	float min_squared_dist = (tip_pos - end_effectors[min_ee_ind]->getPosition()).squaredNorm();
//	for (int ee_ind = 1; ee_ind < end_effectors.size(); ee_ind++) {
//		float squared_dist = (tip_pos - end_effectors[ee_ind]->getPosition()).squaredNorm();
//		if (squared_dist < min_squared_dist) {
//			bool isCursorNotAttachedToEE = cursors[0]->end_eff!=end_effectors[min_ee_ind];
//			for (int cursor_ind = 1; cursor_ind < cursors.size(); cursor_ind++) {
//				isCursorNotAttachedToEE = isCursorNotAttachedToEE && cursors[cursor_ind]->end_eff!=end_effectors[min_ee_ind];
//			}
//			if (isCursorNotAttachedToEE) {
//				min_squared_dist = squared_dist;
//				min_ee_ind = ee_ind;
//			}
//		}				
//	}
//	return end_effectors[min_ee_ind];
}

void World::draw(RenderMode render_mode)
{
	if (render_mode == NORMAL) {
		for (int i = 0; i<cursors.size(); i++)
			cursors[i]->draw();
		for (int i = 0; i<objs.size(); i++)
			objs[i]->draw();
		for (int i = 0; i<threads.size(); i++)
			threads[i]->draw(false);
	} else if (render_mode == EXAMINE) {
		for (int i = 0; i<cursors.size(); i++)
			cursors[i]->draw();
		for (int i = 0; i<objs.size(); i++)
			objs[i]->draw();
		for (int i = 0; i<threads.size(); i++)
			threads[i]->draw(true);
	} else if (render_mode == DEBUG) {
		for (int i = 0; i<cursors.size(); i++)
			cursors[i]->draw();
		for (int i = 0; i<threads.size(); i++)
			threads[i]->drawDebug();
		for (int i = 0; i<objs.size(); i++)
			objs[i]->drawDebug();
	} else {
		if (collision_world != NULL) {
			btCollisionObjectArray col_objs = collision_world->collision_world->getCollisionObjectArray();
			glColor3f(0,0.8,0.6);
			for (int i = 0; i < col_objs.size(); i++) {
				drawCapsule(col_objs[i], true);
			}
		}
	}
	if (collision_world != NULL)
		collision_world->drawAllCollisions();
}

void World::setTransformFromController(const vector<ControllerBase*>& controllers, bool limit_displacement)
{
	assert(cursors.size() == controllers.size());
	for (int i = 0; i < cursors.size(); i++) {
		Cursor* cursor = cursors[i];
		cursor->setTransform(controllers[i]->getPosition(), controllers[i]->getRotation(), limit_displacement);
		if (controllers[i]->hasButtonPressedAndReset(UP))
			cursor->openClose(limit_displacement);
		if (controllers[i]->hasButtonPressedAndReset(DOWN))
			cursor->attachDettach(limit_displacement);
	}
	
	for (int thread_ind = 0; thread_ind < threads.size(); thread_ind++) {
		threads[thread_ind]->minimize_energy();
	}
	vector<EndEffector*> end_effs;
	getObjects<EndEffector>(end_effs);
	for (int ee_ind = 0; ee_ind < end_effs.size(); ee_ind++) {
		end_effs[ee_ind]->updateTransformFromAttachment();
	}
}

double uniformRand()
{
  return ((drand48() - 0.5) * 2); 
}

boost::variate_generator<boost::mt19937, boost::normal_distribution<> > normRand(boost::mt19937(time(0)), boost::normal_distribution<>(0.0, 1.0));

void World::applyRelativeControl(const vector<Control*>& controls, double thresh, bool limit_displacement)
{
	assert(cursors.size() == controls.size());
	for (int i = 0; i < cursors.size(); i++) {
		Cursor* cursor = cursors[i];
    Matrix3d rotate(controls[i]->getRotate());
    
    
    AngleAxisd noise_rot = AngleAxisd(thresh * normRand() * M_PI/180.0,
        Vector3d(normRand(), normRand(), normRand()).normalized());
		const Matrix3d cursor_rot = cursor->rotation * rotate * noise_rot;



    const Vector3d noise_vec = Vector3d(thresh * normRand(),
                                        thresh * normRand(),
                                        thresh * normRand());
		const Vector3d cursor_pos = cursor->position + controls[i]->getTranslate() + EndEffector::grab_offset * cursor_rot.col(0) + noise_vec;


		cursor->setTransform(cursor_pos, cursor_rot, limit_displacement);
		
		if (controls[i]->getButton(UP))
			cursor->openClose(limit_displacement);
		if (controls[i]->getButton(DOWN))
			cursor->attachDettach(limit_displacement);
	}
	
	for (int thread_ind = 0; thread_ind < threads.size(); thread_ind++) {
		threads[thread_ind]->minimize_energy();
	}
	vector<EndEffector*> end_effs;
	getObjects<EndEffector>(end_effs);
	for (int ee_ind = 0; ee_ind < end_effs.size(); ee_ind++) {
		end_effs[ee_ind]->updateTransformFromAttachment();
	}
}

//The control is effectively applied to the tip of the end effector
void World::applyRelativeControl(const VectorXd& relative_control, double thresh, bool limit_displacement)
{
  vector<Control*> all_u;
  VectorXdToControl(relative_control, all_u);
  applyRelativeControl(all_u, thresh, limit_displacement);

	/*for (int i = 0; i < cursors.size(); i++) {
		Cursor* cursor = cursors[i];
		Matrix3d rotation;
		rotation_from_euler_angles(rotation, relative_control(8*i+3), relative_control(8*i+4), relative_control(8*i+5));

    AngleAxisd noise_rot = AngleAxisd(thresh * normRand() * M_PI/180.0,
        Vector3d(normRand(), normRand(), normRand()).normalized());
    const Vector3d noise_vec = Vector3d(thresh * normRand(),
                                        thresh * normRand(),
                                        thresh * normRand());


		const Matrix3d cursor_rot = cursor->rotation * rotation * noise_rot;
		const Vector3d cursor_pos = cursor->position + relative_control.segment(8*i+0, 3) + EndEffector::grab_offset * cursor_rot.col(0) + noise_vec;
		cursor->setTransform(cursor_pos, cursor_rot, limit_displacement);
		
		if (relative_control(8*i+6))
			cursor->openClose(limit_displacement);
		if (relative_control(8*i+7))
			cursor->attachDettach(limit_displacement);
	}
	
	for (int thread_ind = 0; thread_ind < threads.size(); thread_ind++) {
		threads[thread_ind]->minimize_energy();
	}
	vector<EndEffector*> end_effs;
	getObjects<EndEffector>(end_effs);
	for (int ee_ind = 0; ee_ind < end_effs.size(); ee_ind++) {
		end_effs[ee_ind]->updateTransformFromAttachment();
	}*/
}

void World::applyRelativeControlJacobian(const VectorXd& relative_control, double thresh) 
{
  assert(cursors.size()*6 == relative_control.size());
  VectorXd wrapper_control = JacobianControlWrapper(relative_control);
  /*wrapper_control.setZero(); 
  wrapper_control.segment(0, 6) = relative_control.segment(0,6);
  wrapper_control.segment(8, 6) = relative_control.segment(6,6); */

  applyRelativeControl(wrapper_control, thresh, true);

}

void World::getStates(vector<VectorXd>& states)
{
	states.clear();
	for (int i = 0; i<threads.size(); i++) {
		VectorXd state;
		threads[i]->getState(state);
		states.push_back(state);
	}
	for (int i = 0; i<objs.size(); i++) {
		VectorXd state;
		objs[i]->getState(state);
		states.push_back(state);
	}
}

void World::getStateForJacobian(VectorXd& world_state) { 
  vector<VectorXd> states;
  int state_size = 0; 
  for (int i = 0; i < threads.size(); i++) { 
    VectorXd state;
    threads[i]->getState(state);
    states.push_back(state); 
    state_size += state.size();
    
    // to verify state.size is correct, require state.size() == state(0)
    assert(state(0) == state.size());
  }

  
  for (int i = 0; i < cursors.size(); i++) { 
    if (cursors[i]->isAttached()) {
      VectorXd state;
      //if (!cursors[i]->end_eff->isAttached()) {
        //cout << "WARNING: End Effector is not attached to a thread" << endl;
      //}
      cursors[i]->end_eff->getState(state);
      //cursors[i]->getState(state);
      //for (int i = 0; i < 3; i++) state(i) *= 5;
      //for (int i = 3; i < 6; i++) state(i) *= 50;
      states.push_back(state); 
      state_size += state.size();

      // to verify state.size is correct, require state.size() == state(0)
      assert(state(0) == state.size());
    }
  }  
  
  //flatten vector<VectorXd> into one long VectorXd
  world_state.resize(state_size);
  int start_ind = 0;
  for (int i = 0; i < states.size(); i++) { 
    world_state.segment(start_ind, states[i].size()) = states[i];
    start_ind += states[i].size(); 
  }

}

void World::setStateForJacobian(VectorXd& world_state) {
  int ind = 0; 
  for (int i = 0; i < threads.size(); i++) {
    VectorXd state;
    state = world_state.segment(ind, world_state(ind));
    threads[i]->setState(state);
    ind += state.size();
  }

  for (int i = 0; i < cursors.size(); i++) {
    if(cursors[i]->isAttached()) {
      VectorXd state;
      state = world_state.segment(ind, world_state(ind));
      cursors[i]->setState(state);
      cursors[i]->end_eff->setState(state);
      ind += state.size(); 
    }
  }

  assert(ind == world_state.size());

}

void World::projectLegalState() {
  for (int i = 0; i < threads.size(); i++) {
    vector<Thread*> ind_threads;
    threads[i]->getThreads(ind_threads); 
    for (int t = 0; t < ind_threads.size(); t++) {
      //ind_threads[t]->unviolate_total_length_constraint();
      
      ind_threads[t]->project_length_constraint();
      ind_threads[t]->minimize_energy();
      ind_threads[t]->minimize_energy_twist_angles();
    }
  }
}


void World::computeJacobian(MatrixXd* J) {
  double t0 = GetClock();

  World* world_copy = new World(*this, test_world_manager);

  //cout << "copying took = " << GetClock() - t0 << endl;  

  VectorXd world_state;
  //getStateForJacobian(world_state);
  world_copy->getStateForJacobian(world_state);
  int size_each_state = world_state.size();
  int size_each_control = 12; 
  J->resize(world_state.size(), size_each_control);
  J->setZero();
  double eps = 1e-1;

  t0 = GetClock();

  boost::thread_group group;

  /*for (int i = 0; i < 12; i++) {
    VectorXd du(12);
    du.setZero(); 
    du(i) = eps;
    if (i >= 3 && i <= 5) du(i) = 0.5 * eps; 
    if (i >= 9 && i <= 11) du(i) = 0.5 * eps;

    group.create_thread( boost::bind(computeJacCord,
          this, i, size_each_state, &du, &J) ); 

  }

  group.join_all();
  */
  
  for (int i = 0 ; i < 12; i++) { 
    VectorXd du(12);
    du.setZero(); 
    du(i) = eps;
    if (i >= 3 && i <= 5) du(i) = 0.5 * eps; 
    if (i >= 9 && i <= 11) du(i) = 0.5 * eps; 
    world_copy->applyRelativeControlJacobian(du); 
    VectorXd new_state;
    world_copy->getStateForJacobian(new_state);
    J->block(0,i, size_each_state, 1) = new_state;
    world_copy->setStateForJacobian(world_state);

    du(i) = -eps;
    if (i >= 3 && i <= 5) du(i) = 0.5 * -eps; 
    if (i >= 9 && i <= 11) du(i) = 0.5 * -eps; 
    world_copy->applyRelativeControlJacobian(du); 
    world_copy->getStateForJacobian(new_state);
    J->block(0,i, size_each_state, 1) -= new_state;
    world_copy->setStateForJacobian(world_state);

  }
  //cout << GetClock() - t0 << endl; 

  delete world_copy;
  
  (*J) /= (2 * eps); 
  //J /= eps; 

}

void computeJacCord(World* w, int i, int size_each_state, VectorXd* du, MatrixXd* J) {

    World* world_copy = new World(*w, test_world_manager); 
    world_copy->applyRelativeControlJacobian(*du); 
    VectorXd new_state;
    world_copy->getStateForJacobian(new_state);
    J->block(0,i, size_each_state, 1) = new_state;
    delete world_copy;
    
    world_copy = new World(*w, test_world_manager); 
    world_copy->applyRelativeControlJacobian(-1*(*du)); 
    world_copy->getStateForJacobian(new_state);
    J->block(0,i, size_each_state, 1) -= new_state;
    delete world_copy;
}


void World::printStates() { 
  VectorXd world_state;
  getStateForJacobian(world_state);
  if (world_state.size() > 0) 
    cout << world_state.transpose() << endl; 
}

void World::backup()
{
	for (int i = 0; i < threads.size(); i++)
		threads[i]->backup();
	for (int i = 0; i<objs.size(); i++)
		objs[i]->backup();
}

void World::restore()
{
	for (int i = 0; i < threads.size(); i++)
		threads[i]->restore();
	for (int i = 0; i<objs.size(); i++)
		objs[i]->restore();
	for (int i = 0; i<cursors.size(); i++) {
		if(cursors[i]->isAttached())
			cursors[i]->dettach();
	}
}

void World::initThread()
{
  int numInit = 6;

	double first_length = 3.0; //FIRST_REST_LENGTH;
	double second_length = SECOND_REST_LENGTH;
	double middle_length = DEFAULT_REST_LENGTH;

  vector<Vector3d> vertices;
  vector<double> angles;
  vector<double> lengths;
  vector<Vector3d> directions;
  
	for (int i=0; i < 2*numInit + 5; i++)
		angles.push_back(0.0);
  
  lengths.push_back(first_length);
  lengths.push_back(second_length);
  for (int i=0; i < 2*numInit; i++)
  	lengths.push_back(middle_length);
  lengths.push_back(second_length);
  lengths.push_back(first_length);
  lengths.push_back(first_length);
  
  directions.push_back(Vector3d::UnitX());
  directions.push_back(Vector3d::UnitX());
  Vector3d direction = Vector3d(2.0, -1.0, 0.0);
  direction.normalize();
  for (int i=0; i < numInit; i++)
  	directions.push_back(direction);
  direction = Vector3d(1.0, -2.0, 0.0);
  direction.normalize();
  for (int i=0; i < numInit; i++)
  	directions.push_back(direction);
  directions.push_back(-Vector3d::UnitY());
  directions.push_back(-Vector3d::UnitY());
  
  vertices.push_back(Vector3d::Zero());
  for (int i=1; i < 2*numInit + 5; i++) {
  	Vector3d next_vertex;
  	next_vertex(0) = vertices[i-1](0) + directions[i-1](0) * lengths[i-1];
  	next_vertex(1) = vertices[i-1](1) + directions[i-1](1) * lengths[i-1];
  	next_vertex(2) = vertices[i-1](2) + directions[i-1](2) * lengths[i-1];
    vertices.push_back(next_vertex);
  }
  
	Vector3d last_pos = Vector3d(-10.0, -30.0, 0.0);
	for (int i=0; i<vertices.size(); i++)
		vertices[i] += -vertices.back() + last_pos;

  Matrix3d start_rotation0 = Matrix3d::Identity();
  Matrix3d end_rotation0 = (Matrix3d) AngleAxisd(-M_PI/2.0, Vector3d::UnitZ());

  ThreadConstrained* thread0 = new ThreadConstrained(vertices, angles, lengths, start_rotation0, end_rotation0, this);
  threads.push_back(thread0);
  
  for (int i=0; i<vertices.size(); i++)
		vertices[i](0) *= -1;
	Matrix3d start_rotation1 = (Matrix3d) AngleAxisd(M_PI, Vector3d::UnitZ());
  Matrix3d end_rotation1 = (Matrix3d) AngleAxisd(-M_PI/2.0, Vector3d::UnitZ());
  ThreadConstrained* thread1 = new ThreadConstrained(vertices, angles, lengths, start_rotation1, end_rotation1, this);
  threads.push_back(thread1);
  
	for (int thread_ind=0; thread_ind < threads.size(); thread_ind++) {
#ifndef ISOTROPIC
		Matrix2d B = Matrix2d::Zero();
		B(0,0) = 10.0;
		B(1,1) = 1.0;
		threads[thread_ind]->set_coeffs_normalized(B, 3.0, 1e-4);
#else
  	threads[thread_ind]->set_coeffs_normalized(1.0, 3.0, 1e-4);
#endif
	}
}

void World::initLongerThread()
{
  int numInit = 5;

	double first_length = 3.0; //FIRST_REST_LENGTH;
	double second_length = SECOND_REST_LENGTH;
	double middle_length = DEFAULT_REST_LENGTH;

  vector<Vector3d> vertices;
  vector<double> angles;
  vector<double> lengths;
  vector<Vector3d> directions;
  
	for (int i=0; i < 4*numInit + 5; i++)
		angles.push_back(0.0);
  
  lengths.push_back(first_length);
  lengths.push_back(second_length);
  for (int i=0; i < 4*numInit; i++)
  	lengths.push_back(middle_length);
  lengths.push_back(second_length);
  lengths.push_back(first_length);
  lengths.push_back(first_length);
  
  directions.push_back(Vector3d::UnitX());
  directions.push_back(Vector3d::UnitX());
  Vector3d direction = Vector3d(1.0, 1.0, 0.0);
  direction.normalize();
  for (int i=0; i < numInit; i++)
  	directions.push_back(direction);
  direction = Vector3d(1.0, -1.0, 0.4);
  direction.normalize();
  for (int i=0; i < numInit; i++)
  	directions.push_back(direction);
  direction = Vector3d(-1.0, -1.0, -0.4);
  direction.normalize();
  for (int i=0; i < numInit; i++)
  	directions.push_back(direction);
  direction = Vector3d(0.0, -1.0, 0.0);
  direction.normalize();
  for (int i=0; i < numInit; i++)
  	directions.push_back(direction);
  directions.push_back(-Vector3d::UnitY());
  directions.push_back(-Vector3d::UnitY());
  
  vertices.push_back(Vector3d::Zero());
  for (int i=1; i < 4*numInit + 5; i++) {
  	Vector3d next_vertex;
  	next_vertex(0) = vertices[i-1](0) + directions[i-1](0) * lengths[i-1];
  	next_vertex(1) = vertices[i-1](1) + directions[i-1](1) * lengths[i-1];
  	next_vertex(2) = vertices[i-1](2) + directions[i-1](2) * lengths[i-1];
    vertices.push_back(next_vertex);
  }
  
	Vector3d last_pos = Vector3d(-10.0, -30.0, 0.0);
	for (int i=0; i<vertices.size(); i++)
		vertices[i] += -vertices.back() + last_pos;

  Matrix3d start_rotation0 = Matrix3d::Identity();
  Matrix3d end_rotation0 = (Matrix3d) AngleAxisd(-M_PI/2.0, Vector3d::UnitZ());

  ThreadConstrained* thread0 = new ThreadConstrained(vertices, angles, lengths, start_rotation0, end_rotation0, this);
  threads.push_back(thread0);
  
  directions.clear();
  directions.push_back(Vector3d::UnitX());
  directions.push_back(Vector3d::UnitX());
  direction = Vector3d(1.0, 1.0, 0.0);
  direction.normalize();
  for (int i=0; i < numInit; i++)
  	directions.push_back(direction);
  direction = Vector3d(1.0, -1.0, -0.4);
  direction.normalize();
  for (int i=0; i < numInit; i++)
  	directions.push_back(direction);
  direction = Vector3d(-1.0, -1.0, 0.4);
  direction.normalize();
  for (int i=0; i < numInit; i++)
  	directions.push_back(direction);
  direction = Vector3d(0.0, -1.0, 0.0);
  direction.normalize();
  for (int i=0; i < numInit; i++)
  	directions.push_back(direction);
  directions.push_back(-Vector3d::UnitY());
  directions.push_back(-Vector3d::UnitY());
  
  vertices.clear();
  vertices.push_back(Vector3d::Zero());
  for (int i=1; i < 4*numInit + 5; i++) {
  	Vector3d next_vertex;
  	next_vertex(0) = vertices[i-1](0) + directions[i-1](0) * lengths[i-1];
  	next_vertex(1) = vertices[i-1](1) + directions[i-1](1) * lengths[i-1];
  	next_vertex(2) = vertices[i-1](2) + directions[i-1](2) * lengths[i-1];
    vertices.push_back(next_vertex);
  }
  
	last_pos = Vector3d(-10.0, -30.0, 0.0);
	for (int i=0; i<vertices.size(); i++)
		vertices[i] += -vertices.back() + last_pos;
  
  for (int i=0; i<vertices.size(); i++)
		vertices[i](0) *= -1;
	Matrix3d start_rotation1 = (Matrix3d) AngleAxisd(M_PI, Vector3d::UnitZ());
  Matrix3d end_rotation1 = (Matrix3d) AngleAxisd(-M_PI/2.0, Vector3d::UnitZ());
  ThreadConstrained* thread1 = new ThreadConstrained(vertices, angles, lengths, start_rotation1, end_rotation1, this);
  threads.push_back(thread1);
  
	for (int thread_ind=0; thread_ind < threads.size(); thread_ind++) {
#ifndef ISOTROPIC
		Matrix2d B = Matrix2d::Zero();
		B(0,0) = 10.0;
		B(1,1) = 1.0;
		threads[thread_ind]->set_coeffs_normalized(B, 3.0, 1e-4);
#else
  	threads[thread_ind]->set_coeffs_normalized(1.0, 3.0, 1e-4);
#endif
	}
}

void World::initRestingThread(int opt)
{
  int numInit = 5;

	double first_length = 3.0; //FIRST_REST_LENGTH;
	double second_length = SECOND_REST_LENGTH;
	double middle_length = DEFAULT_REST_LENGTH;

  vector<Vector3d> vertices;
  vector<double> angles;
  vector<double> lengths;
  vector<Vector3d> directions;
  
  if (opt == 0 || opt == 1) {
		for (int i=0; i < 4*numInit + 5; i++)
			angles.push_back(0.0);
		
		lengths.push_back(first_length);
		lengths.push_back(second_length);
		for (int i=0; i < 4*numInit; i++)
			lengths.push_back(middle_length);
		lengths.push_back(second_length);
		lengths.push_back(first_length);
		lengths.push_back(first_length);
		
		directions.push_back(Vector3d::UnitX());
		directions.push_back(Vector3d::UnitX());
		Vector3d direction = Vector3d(1.0, 1.0, 0.0);
		direction.normalize();
		for (int i=0; i < numInit; i++)
			directions.push_back(direction);
		direction = Vector3d(1.0, -1.0, 0.4);
		direction.normalize();
		for (int i=0; i < numInit; i++)
			directions.push_back(direction);
		direction = Vector3d(-1.0, -1.0, -0.4);
		direction.normalize();
		for (int i=0; i < numInit; i++)
			directions.push_back(direction);
		direction = Vector3d(0.0, -1.0, 0.0);
		direction.normalize();
		for (int i=0; i < numInit; i++)
			directions.push_back(direction);
		directions.push_back(-Vector3d::UnitY());
		directions.push_back(-Vector3d::UnitY());
		
		vertices.push_back(Vector3d::Zero());
		for (int i=1; i < 4*numInit + 5; i++) {
			Vector3d next_vertex;
			next_vertex(0) = vertices[i-1](0) + directions[i-1](0) * lengths[i-1];
			next_vertex(1) = vertices[i-1](1) + directions[i-1](1) * lengths[i-1];
			next_vertex(2) = vertices[i-1](2) + directions[i-1](2) * lengths[i-1];
		  vertices.push_back(next_vertex);
		}
		
		Vector3d last_pos = Vector3d(-10.0, -30.0, 0.0);
		for (int i=0; i<vertices.size(); i++)
			vertices[i] += -vertices.back() + last_pos;

		Matrix3d start_rotation0 = Matrix3d::Identity();
		Matrix3d end_rotation0 = (Matrix3d) AngleAxisd(-M_PI/2.0, Vector3d::UnitZ());

		ThreadConstrained* thread0 = new ThreadConstrained(vertices, angles, lengths, start_rotation0, end_rotation0, this);
		threads.push_back(thread0);
  }
  
  if (opt == 0 || opt == 2) {
		int numInit1 = 2;
		
		angles.clear();
		for (int i=0; i < 16*numInit1 + 5; i++)
			angles.push_back(0.0);
		
		lengths.clear();
		lengths.push_back(first_length);
		lengths.push_back(second_length);
		for (int i=0; i < 16*numInit1; i++)
			lengths.push_back(middle_length);
		lengths.push_back(second_length);
		lengths.push_back(first_length);
		lengths.push_back(first_length);
		
		directions.clear();
		directions.push_back(-Vector3d::UnitX());
		directions.push_back(-Vector3d::UnitX());
		Vector3d direction = Vector3d(-1.0, 0.0, 1.0);
		direction.normalize();
		for (int i=0; i < 1.5*numInit1; i++)
			directions.push_back(direction);
		direction = Vector3d(0.0, -1.0, 2.0);
		direction.normalize();
		for (int i=0; i < 1.5*numInit1; i++)
			directions.push_back(direction);
		direction = Vector3d(-2.0, 1.0, 0.0);
		direction.normalize();
		for (int i=0; i < numInit1; i++)
			directions.push_back(direction);
		direction = Vector3d(-2.0, -1.0, 0.0);
		direction.normalize();
		for (int i=0; i < numInit1; i++)
			directions.push_back(direction);
		direction = Vector3d(0.0, 1.0, -2.0);
		direction.normalize();
		for (int i=0; i < 3*numInit1; i++)
			directions.push_back(direction);
		direction = Vector3d(0.0, -1.0, -2.0);
		direction.normalize();
		for (int i=0; i < 3*numInit1; i++)
			directions.push_back(direction);
		direction = Vector3d(2.0, 1.0, 0.0);
		direction.normalize();
		for (int i=0; i < numInit1; i++)
			directions.push_back(direction);
		direction = Vector3d(2.0, -1.0, 0.0);
		direction.normalize();
		for (int i=0; i < numInit1; i++)
			directions.push_back(direction);
		direction = Vector3d(0.0, 1.0, 2.0);
		direction.normalize();
		for (int i=0; i < 1.5*numInit1; i++)
			directions.push_back(direction);
		direction = Vector3d(0.0, -1.0, 2.0);
		direction.normalize();
		for (int i=0; i < 1.5*numInit1; i++)
			directions.push_back(direction);
		directions.push_back(-Vector3d::UnitY());
		directions.push_back(-Vector3d::UnitY());
		
		vertices.clear();
		vertices.push_back(Vector3d::Zero());
		for (int i=1; i < 16*numInit1 + 5; i++) {
			Vector3d next_vertex;
			next_vertex(0) = vertices[i-1](0) + directions[i-1](0) * lengths[i-1];
			next_vertex(1) = vertices[i-1](1) + directions[i-1](1) * lengths[i-1];
			next_vertex(2) = vertices[i-1](2) + directions[i-1](2) * lengths[i-1];
		  vertices.push_back(next_vertex);
		}
		
		Vector3d last_pos = Vector3d(10.0, -30.0, 0.0);
		for (int i=0; i<vertices.size(); i++)
			vertices[i] += -vertices.back() + last_pos;
		
		Matrix3d start_rotation1 = (Matrix3d) AngleAxisd(M_PI, Vector3d::UnitZ());
		Matrix3d end_rotation1 = (Matrix3d) AngleAxisd(M_PI/2.0, Vector3d::UnitZ());
		ThreadConstrained* thread1 = new ThreadConstrained(vertices, angles, lengths, start_rotation1, this);
		threads.push_back(thread1);
	}
  
	for (int thread_ind=0; thread_ind < threads.size(); thread_ind++) {
#ifndef ISOTROPIC
		Matrix2d B = Matrix2d::Zero();
		B(0,0) = 10.0;
		B(1,1) = 1.0;
		threads[thread_ind]->set_coeffs_normalized(B, 3.0, 1e-4);
#else
  	threads[thread_ind]->set_coeffs_normalized(1.0, 3.0, 1e-4);
#endif
	}
}

