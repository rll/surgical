#include "World.h"
#include "../thread_discrete.h"
#include "../ThreadConstrained.h"

World::World()
{
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
	
//	objs.push_back(new Box(plane->getPosition() + Vector3d(15.0, 10.0, 0.0), Matrix3d::Identity(), Vector3d(10,10,10), 0.0, 0.5, 0.7, this));
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

	initializeThreadsInEnvironment();
}

World::World(const World& rhs)
{
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

  initializeThreadsInEnvironment();
}

World::~World()
{
	clearObjs();
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

World::World(ifstream& file)
{
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
  cout << endl;
  
  initializeThreadsInEnvironment();
}

// Updates the threads_in_env variable of every Thread object in the world (i.e. every Thread of every ThreadConstrained in the world). This variable is used for thread-thread collisions.
void World::initializeThreadsInEnvironment()
{
	vector<ThreadConstrained*> all_thread_constrained = threads;	
	vector<Thread*> all_threads;
	for (int k=0; k<all_thread_constrained.size(); k++) {
		ThreadConstrained* thread_constrained = all_thread_constrained[k];
		vector<Thread*> threads;
		thread_constrained->getThreads(threads);
		for (int i=0; i<threads.size(); i++)
			all_threads.push_back(threads[i]);
	}
	for (int i=0; i<all_threads.size(); i++) {
		all_threads[i]->clear_threads_in_env();
		for (int j=0; j<all_threads.size(); j++) {
			if (i!=j) 
				all_threads[i]->add_thread_to_env(all_threads[j]);
		}
	}
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

void World::clearObjs()
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
	threads.clear();
	objs.clear();
}

void World::draw(bool examine_mode)
{
//	for (int i = 0; i<cursors.size(); i++)
//		cursors[i]->draw();
//	for (int i = 0; i<threads.size(); i++)
//		threads[i]->draw(examine_mode);
//	for (int i = 0; i<objs.size(); i++)
//		objs[i]->draw();
	
	if (examine_mode) {
		for (int i = 0; i<threads.size(); i++)
			threads[i]->draw(examine_mode);
	} else {
		for (int i = 0; i<cursors.size(); i++)
			cursors[i]->draw();
		for (int i = 0; i<threads.size(); i++)
			threads[i]->draw(examine_mode);
		for (int i = 0; i<objs.size(); i++)
			objs[i]->draw();
	}	
}

void World::drawDebug()
{
//	for (int i = 0; i<cursors.size(); i++)
//		cursors[i]->drawDebug();
//	for (int i = 0; i<threads.size(); i++)
//		threads[i]->drawDebug();
	for (int i = 0; i<objs.size(); i++)
		objs[i]->drawDebug();
	
//	vector<Box*> boxes;
//	getObjects<Box>(boxes);
//	vector<Needle*> needles;
//	getObjects<Needle>(needles);
//	vector<EndEffector*> end_effs;
//	getObjects<EndEffector>(end_effs);
//	if (boxes.size() > 0 && needles.size() > 0) {
//		Vector3d direction;
//		Vector3d positionWorldOnA;
//		Vector3d positionWorldOnB;
//		capsuleBoxDistance(end_effs[0]->getStartPosition(), end_effs[0]->getEndPosition(), 5.0/8.0, boxes[0]->getPosition(), boxes[0]->getHalfLength(), direction, positionWorldOnA, positionWorldOnB);
//		//capsuleBoxDistance(needles[0]->getStartPosition(), needles[0]->getEndPosition(), 5.0/8.0, boxes[0]->getPosition(), boxes[0]->getHalfLength(), direction, positionWorldOnA, positionWorldOnB);
//		//sphereBoxDistance(needles[0]->getStartPosition(), 5.0/8.0, boxes[0]->getPosition(), boxes[0]->getHalfLength(), direction, positionWorldOnA, positionWorldOnB);
//		drawSphere(positionWorldOnA, 2.0);		
//		drawSphere(positionWorldOnB, 2.0);
//		drawArrow(positionWorldOnB, direction);
//	}
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

void World::applyRelativeControl(const vector<Control*>& controls, bool limit_displacement)
{
	assert(cursors.size() == controls.size());
	for (int i = 0; i < cursors.size(); i++) {
		Cursor* cursor = cursors[i];
		Matrix3d rotate(controls[i]->getRotate());		
		const Matrix3d cursor_rot = cursor->rotation * rotate;
		const Vector3d cursor_pos = cursor->position + controls[i]->getTranslate() + EndEffector::grab_offset * cursor_rot.col(0);
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
void World::applyRelativeControl(const VectorXd& relative_control, bool limit_displacement)
{
	assert(cursors.size()*8 == relative_control.size());
	for (int i = 0; i < cursors.size(); i++) {
		Cursor* cursor = cursors[i];
		Matrix3d rotation;
		rotation_from_euler_angles(rotation, relative_control(8*i+3), relative_control(8*i+4), relative_control(8*i+5));
		const Matrix3d cursor_rot = cursor->rotation * rotation;
		const Vector3d cursor_pos = cursor->position + relative_control.segment(8*i+0, 3) + EndEffector::grab_offset * cursor_rot.col(0);
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
	}
}

void World::applyRelativeControlJacobian(const VectorXd& relative_control) 
{
  assert(cursors.size()*6 == relative_control.size());
  VectorXd wrapper_control(16);
  wrapper_control.setZero(); 
  wrapper_control.segment(0, 6) = relative_control.segment(0,6);
  wrapper_control.segment(8, 6) = relative_control.segment(6,6);

  applyRelativeControl(wrapper_control, true);

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
  }
 
  
  /*for (int i = 0; i < cursors.size(); i++) { 
    if (cursors[i]->isAttached()) {
      VectorXd state;
      if (!cursors[i]->end_eff->isAttached()) {
        cout << "WARNING: End Effector is not attached to a thread" << endl;
      }
      cursors[i]->end_eff->getState(state);
      //cursors[i]->getState(state); 
      states.push_back(state); 
      state_size += state.size();
    }
  }*/
  
  
  //flatten vector<VectorXd> into one long VectorXd
  world_state.resize(state_size);
  int start_ind = 0;
  for (int i = 0; i < states.size(); i++) { 
    world_state.segment(start_ind, states[i].size()) = states[i];
    start_ind += states[i].size(); 
  }

}

void World::computeJacobian(MatrixXd& J) { 
  VectorXd world_state;
  getStateForJacobian(world_state);
  int size_each_state = world_state.size();
  int size_each_control = 12; 
  J.resize(world_state.size(), size_each_control);
  J.setZero();
  double eps = 1e-1;
   
  #pragma omp parallel for
  for (int i = 0 ; i < 12; i++) { 
    VectorXd du(12);
    du.setZero(); 
    du(i) = eps;
    World* world_copy = new World(*this); 
    world_copy->applyRelativeControlJacobian(du); 
    VectorXd new_state;
    world_copy->getStateForJacobian(new_state);
    J.block(0,i, size_each_state, 1) = new_state; 
    delete world_copy;

    du(i) = -eps;
    world_copy = new World(*this); 
    world_copy->applyRelativeControlJacobian(du); 
    world_copy->getStateForJacobian(new_state);
    J.block(0,i, size_each_state, 1) -= new_state; 
    delete world_copy;
  }
  
  J /= (2 * eps); 
  //J /= eps; 

}


void World::printStates() { 
  cout << endl << endl << endl << endl << endl << endl << endl << endl;
  cout << endl << endl << endl << endl << endl << endl << endl << endl;
  cout << endl << endl << endl << endl << endl << endl << endl << endl;
  cout << endl << endl << endl << endl << endl << endl << endl << endl;
  cout << endl << endl << endl << endl << endl << endl << endl << endl;
  cout << endl << endl << endl << endl << endl << endl << endl << endl;
  cout << endl << endl << endl << endl << endl << endl << endl << endl;
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
	initializeThreadsInEnvironment();
}

bool World::capsuleObjectIntersection(int capsule_ind, const Vector3d& start, const Vector3d& end, const double radius, vector<Intersection>& intersections)
{
	bool found = false;
	for (int obj_ind = 0; obj_ind < objs.size(); obj_ind++) {
		found = objs[obj_ind]->capsuleIntersection(capsule_ind, start, end, radius, intersections) || found;
	}
	return found;
}

double World::capsuleObjectRepulsionEnergy(const Vector3d& start, const Vector3d& end, const double radius)
{
	double energy = 0.0;
	for (int obj_ind = 0; obj_ind < objs.size(); obj_ind++) {
		energy += objs[obj_ind]->capsuleRepulsionEnergy(start, end, radius);
	}
	return energy;
}

void World::capsuleObjectRepulsionEnergyGradient(const Vector3d& start, const Vector3d& end, const double radius, Vector3d& gradient)
{
	for (int obj_ind = 0; obj_ind < objs.size(); obj_ind++) {
		objs[obj_ind]->capsuleRepulsionEnergyGradient(start, end, radius, gradient);
	}
}

#if 0

const int maxNumObjects = 4;
const int numObjects = 2;

GL_Simplex1to4 simplex;

btCollisionObject	objects[maxNumObjects];
btCollisionWorld*	collisionWorld = 0;

GLDebugDrawer debugDrawer;


void	CollisionInterfaceDemo::initPhysics()
{
			
//init
	btMatrix3x3 basisA;
	basisA.setIdentity();

	btMatrix3x3 basisB;
	basisB.setIdentity();

	objects[0].getWorldTransform().setBasis(basisA);
	objects[1].getWorldTransform().setBasis(basisB);

	btCapsuleShape* capsuleA = new btCapsuleShape(btScalar(1), btScalar(4));
	capsuleA->setMargin(0.f);
	
	btCapsuleShape* capsuleB = new btCapsuleShape(btScalar(1), btScalar(4));
	capsuleB->setMargin(0.f);
	
	objects[0].setCollisionShape(capsuleA);
	objects[1].setCollisionShape(capsuleB);

	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
	btVector3	worldAabbMin(-1000,-1000,-1000);
	btVector3	worldAabbMax(1000,1000,1000);

	btAxisSweep3*	broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax);
	
	//SimpleBroadphase is a brute force alternative, performing N^2 aabb overlap tests
	//SimpleBroadphase*	broadphase = new btSimpleBroadphase;

	collisionWorld = new btCollisionWorld(dispatcher,broadphase,collisionConfiguration);

	collisionWorld->addCollisionObject(&objects[0]);
	collisionWorld->addCollisionObject(&objects[1]);



//in every iter
	btVector3	worldBoundsMin,worldBoundsMax;
	collisionWorld->getBroadphase()->getBroadphaseAabb(worldBoundsMin,worldBoundsMax);
	
	if (collisionWorld)
		collisionWorld->performDiscreteCollisionDetection();

	
	//Assume collisionWorld->stepSimulation or collisionWorld->performDiscreteCollisionDetection has been called
	int numManifolds = collisionWorld->getDispatcher()->getNumManifolds();
	for (i=0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold = collisionWorld->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
		btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
	
		int numContacts = contactManifold->getNumContacts();
		for (int j=0;j<numContacts;j++)
		{
			btManifoldPoint& pt = contactManifold->getContactPoint(j);

			if (pt.getDistance()<0.f)
			{
				const btVector3& ptA = pt.getPositionWorldOnA();
				const btVector3& ptB = pt.getPositionWorldOnB();
				const btVector3& normalOnB = pt.m_normalWorldOnB;
			}

			glBegin(GL_LINES);
			glColor3f(0, 0, 0);
			
			btVector3 ptA = pt.getPositionWorldOnA();
			btVector3 ptB = pt.getPositionWorldOnB();

			glVertex3d(ptA.x(),ptA.y(),ptA.z());
			glVertex3d(ptB.x(),ptB.y(),ptB.z());
			glEnd();
		}

		//you can un-comment out this line, and then all points are removed
		//contactManifold->clearManifold();	
	}

	//for update transform
	if ((objects[1].getWorldTransform().getOrigin()).y() > -2)
			objects[1].getWorldTransform().setOrigin(objects[1].getWorldTransform().getOrigin()+btVector3(0,-0.0005*timeInSeconds,0));

bool World::check_for_intersection(vector<Self_Intersection>& self_intersections, vector<Thread_Intersection>& thread_intersections, vector<Intersection>& intersections)
{   	
  double found = false; // count of number of intersections
  Vector3d direction;
  self_intersections.clear();
  thread_intersections.clear();
  intersections.clear();

  //self intersections
  for(int i = 0; i < _thread_pieces.size() - 3; i++) {
    //+2 so you don't check the adjacent piece - bug?
    for(int j = i + 2; j <= _thread_pieces.size() - 2; j++) { //check. it was < instead of <=
      //skip if both ends, since these are constraints
      if(i == 0 && j == _thread_pieces.size() - 2) 
        continue;
      double intersection_dist = self_intersection(i,j,THREAD_RADIUS,direction);
      if(intersection_dist < 0) {
        found = true;

        self_intersections.push_back(Self_Intersection(i,j,-intersection_dist,direction));
      }
    }
  }
	
	//intersections between threads
	for (int k=0; k < threads_in_env.size(); k++) {
		//if (this == threads_in_env[k]) //check
			//continue;
		for(int i = 0; i < _thread_pieces.size()-1; i++) {
	    for(int j = 0; j < threads_in_env[k]->_thread_pieces.size()-1; j++) {
	      //skip if both ends, since these are constraints
	      if((i==0 && j==0) ||
	      	 (i==0 && j==threads_in_env[k]->_thread_pieces.size()-2) ||
	      	 (i==_thread_pieces.size()-2 && j==0) ||
	      	 (i==_thread_pieces.size()-2 && j==threads_in_env[k]->_thread_pieces.size()-2))
	        continue;
	      double intersection_dist = thread_intersection(i,j,k,THREAD_RADIUS,direction);		//asumes other threads have same radius
	      if(intersection_dist < 0) {
	        found = true;
	        thread_intersections.push_back(Thread_Intersection(i,j,k,-intersection_dist,direction));
	      }
	    }
	  }
	}
	bool obj_intersection = false;
	//object intersections
  if (world != NULL) {
		for(int i = 2; i < _thread_pieces.size() - 3; i++) {
			//found = world->capsuleObjectIntersection(i, _thread_pieces[i]->vertex(), _thread_pieces[i+1]->vertex(), THREAD_RADIUS, intersections) || found;
			bool temp_obj_intersection = world->capsuleObjectIntersection(i, _thread_pieces[i]->vertex(), _thread_pieces[i+1]->vertex(), THREAD_RADIUS, intersections);
			if (temp_obj_intersection) obj_intersection = true;
			found = temp_obj_intersection || found;
		}
  }
  
  //if (found || obj_intersection)
  	//cout << "intersections. obj_intersection = " << obj_intersection << endl;

  return found;
}
#endif
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

