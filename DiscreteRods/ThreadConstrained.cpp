#include "ThreadConstrained.h"
#include "thread_discrete.h"

ThreadConstrained::ThreadConstrained(vector<Vector3d>& vertices, vector<double>& twist_angles, vector<double>& rest_lengths, Matrix3d& start_rot, Matrix3d& end_rot) {
	num_vertices = vertices.size();
	Thread* thread = new Thread(vertices, twist_angles, rest_lengths, start_rot, end_rot);
	threads.push_back(thread);

	thread->minimize_energy();
	
	zero_angle = 0.0;
	rot_diff.push_back(Matrix3d::Identity());
	rot_diff.push_back(Matrix3d::Identity());
	rot_offset.push_back((Matrix3d) (AngleAxisd(M_PI, Vector3d::UnitZ())));
	for (int i=0; i<num_vertices-2; i++)
		rot_offset.push_back((Matrix3d) (AngleAxisd(0.5*M_PI, Vector3d::UnitZ())));
	rot_offset.push_back(Matrix3d::Identity());
	if (LIMITED_DISPLACEMENT) {
		last_pos.push_back(vertices.front());
		last_pos.push_back(vertices.back());
		last_rot.push_back(threads[0]->start_rot() * rot_offset.front().transpose());
		last_rot.push_back(threads[0]->end_rot() * rot_offset.back().transpose());
	}
	constrained_vertices_nums.push_back(0);
	constrained_vertices_nums.push_back(num_vertices-1);
	
	examine_mode = false;
	initContour();
}

ThreadConstrained::ThreadConstrained(vector<Vector3d>& vertices, vector<double>& twist_angles, Matrix3d& start_rot, Matrix3d& end_rot) {
	num_vertices = vertices.size();
	Thread* thread = new Thread(vertices, twist_angles, start_rot, end_rot);
	threads.push_back(thread);

	thread->minimize_energy();

	zero_angle = 0.0;
	rot_diff.push_back(Matrix3d::Identity());
	rot_diff.push_back(Matrix3d::Identity());
	rot_offset.push_back((Matrix3d) (AngleAxisd(M_PI, Vector3d::UnitZ())));
	for (int i=0; i<num_vertices-2; i++)
		rot_offset.push_back((Matrix3d) (AngleAxisd(0.5*M_PI, Vector3d::UnitZ())));
	rot_offset.push_back(Matrix3d::Identity());
	if (LIMITED_DISPLACEMENT) {
		last_pos.push_back(vertices.front());
		last_pos.push_back(vertices.back());
		last_rot.push_back(threads[0]->start_rot() * rot_offset.front().transpose());
		last_rot.push_back(threads[0]->end_rot() * rot_offset.back().transpose());
	}
	constrained_vertices_nums.push_back(0);
	constrained_vertices_nums.push_back(num_vertices-1);
	
	examine_mode = false;
	initContour();
}

ThreadConstrained::ThreadConstrained(int num_vertices_init) {
	num_vertices = num_vertices_init;
	if (num_vertices < 5)
		cout << "Internal error: ThreadConstrained: too few number of vertices";
	int numInit = (num_vertices-3)/2;
	double noise_factor = 0.0;
	vector<Vector3d> vertices;
	vector<double> angles;
	Vector3d direction;
	double start_pos_x;
	if (num_vertices%2 == 0)
		start_pos_x = -((3.0 + (1.0/sqrt(1.0*1.0 + 2.0*2.0))*((double) (num_vertices-4)))/2.0)*DEFAULT_REST_LENGTH;
	else
		start_pos_x = -((2.0 + (1.0/sqrt(1.0*1.0 + 2.0*2.0))*((double) (num_vertices-3)))/2.0)*DEFAULT_REST_LENGTH;
	vertices.push_back(Vector3d(start_pos_x,0,0));
	angles.push_back(0.0);
	//push back unitx so first tangent matches start_frame
	vertices.push_back(vertices.back()+Vector3d::UnitX()*DEFAULT_REST_LENGTH);
	angles.push_back(0.0);
	//specify direction
	direction(0) = 1.0;
	direction(1) = -2.0;
	direction(2) = 0.0;
	direction.normalize();
	for (int i=0; i < numInit; i++) {
		Vector3d noise( ((double)(rand()%10000)) / 10000.0, ((double)(rand()%10000)) / 10000.0, ((double)(rand()%10000)) / 10000.0);
		noise *= noise_factor;
		Vector3d next_Vec = vertices.back()+(direction+noise).normalized()*DEFAULT_REST_LENGTH;
		vertices.push_back(next_Vec);
		angles.push_back(0.0);
	}
	//change direction
	direction(0) = 1.0;
	direction(1) = 2.0;
	direction(2) = 0.0;
	direction.normalize();
	for (int i=0; i < numInit; i++)	{
		Vector3d noise( ((double)(rand()%10000)) / 10000.0, ((double)(rand()%10000)) / 10000.0, ((double)(rand()%10000)) / 10000.0);
		noise *= noise_factor;
		Vector3d next_Vec = vertices.back()+(direction+noise).normalized()*DEFAULT_REST_LENGTH;
		vertices.push_back(next_Vec);
		angles.push_back(0.0);
	}
	//push back unitx so last tangent matches end_frame
	for (int i =0; i < (num_vertices - numInit*2 - 2); i++) {
		vertices.push_back(vertices.back()+Vector3d::UnitX()*DEFAULT_REST_LENGTH);
		angles.push_back(0.0);
	}
	angles.resize(vertices.size());

  vector<Matrix3d> rotations;
	rotations.push_back(Matrix3d::Identity());
	rotations.push_back(Matrix3d::Identity());

	Thread* thread = new Thread(vertices, angles, rotations[0], rotations[1]);
	threads.push_back(thread);

	thread->minimize_energy();

	zero_angle = 0.0;
	rot_diff.push_back(Matrix3d::Identity());
	rot_diff.push_back(Matrix3d::Identity());
	rot_offset.push_back((Matrix3d) (AngleAxisd(M_PI, Vector3d::UnitZ())));
	for (int i=0; i<num_vertices-2; i++)
		rot_offset.push_back((Matrix3d) (AngleAxisd(0.5*M_PI, Vector3d::UnitZ())));
	rot_offset.push_back(Matrix3d::Identity());
	if (LIMITED_DISPLACEMENT) {
		last_pos.push_back(vertices.front());
		last_pos.push_back(vertices.back());
		last_rot.push_back(threads[0]->start_rot() * rot_offset.front().transpose());
		last_rot.push_back(threads[0]->end_rot() * rot_offset.back().transpose());
	}
	constrained_vertices_nums.push_back(0);
	constrained_vertices_nums.push_back(num_vertices_init-1);
	
	examine_mode = false;
	initContour();
}

void ThreadConstrained::get_thread_data(vector<Vector3d> &absolute_points) {
	vector<vector<Vector3d> > points(threads.size());
	for (int thread_num=0; thread_num<threads.size(); thread_num++)
		threads[thread_num]->get_thread_data(points[thread_num]);
	mergeMultipleVector(absolute_points, points);
}

void ThreadConstrained::get_thread_data(vector<Vector3d> &absolute_points, vector<double> &absolute_twist_angles) {
	vector<vector<Vector3d> > points(threads.size());
	vector<vector<double> > twist_angles(threads.size());
	for (int thread_num=0; thread_num<threads.size(); thread_num++) {
		threads[thread_num]->get_thread_data(points[thread_num], twist_angles[thread_num]);
		twist_angles[thread_num].back() = 2.0*twist_angles[thread_num][twist_angles[thread_num].size()-2] - twist_angles[thread_num][twist_angles[thread_num].size()-3];
		if (thread_num==0)
			mapAdd(twist_angles[thread_num], zero_angle);
		else
			mapAdd(twist_angles[thread_num], twist_angles[thread_num-1].back());
 	}
	mergeMultipleVector(absolute_points, points);
	mergeMultipleVector(absolute_twist_angles, twist_angles);
}

void ThreadConstrained::get_thread_data(vector<Vector3d> &absolute_points, vector<double> &absolute_twist_angles, vector<Matrix3d> &absolute_material_frames) {
	vector<vector<Vector3d> > points(threads.size());
	vector<vector<double> > twist_angles(threads.size());
	vector<vector<Matrix3d> > material_frames(threads.size());
	for (int thread_num=0; thread_num<threads.size(); thread_num++) {
		threads[thread_num]->get_thread_data(points[thread_num], twist_angles[thread_num], material_frames[thread_num]);
		twist_angles[thread_num].back() = 2.0*twist_angles[thread_num][twist_angles[thread_num].size()-2] - twist_angles[thread_num][twist_angles[thread_num].size()-3];
	 	if (thread_num==0)
			mapAdd(twist_angles[thread_num], zero_angle);
		else
			mapAdd(twist_angles[thread_num], twist_angles[thread_num-1].back());
  }
	mergeMultipleVector(absolute_points, points);
	mergeMultipleVector(absolute_twist_angles, twist_angles);
	mergeMultipleVector(absolute_material_frames, material_frames);
}

void ThreadConstrained::get_thread_data(vector<Vector3d> &absolute_points, vector<Matrix3d> &absolute_material_frames) {
	vector<vector<Vector3d> > points(threads.size());
	vector<vector<Matrix3d> > material_frames(threads.size());
	for (int thread_num=0; thread_num<threads.size(); thread_num++) {
		threads[thread_num]->get_thread_data(points[thread_num], material_frames[thread_num]);
	}
	mergeMultipleVector(absolute_points, points);
	mergeMultipleVector(absolute_material_frames, material_frames);
}

// parameters have to be of the right size, i.e. threads.size()+1
void ThreadConstrained::getConstrainedTransforms(vector<Vector3d> &positions, vector<Matrix3d> &rotations) {
	int threads_size = threads.size();
	positions.resize(threads_size+1);
	rotations.resize(threads_size+1);
	vector<vector<Vector3d> > points(threads_size);	
	int thread_num = 0;
	threads[thread_num]->get_thread_data(points[thread_num]);
	positions[thread_num] = (points[thread_num]).front();
	rotations[thread_num] = (threads[thread_num]->start_rot()) * rot_offset[constrained_vertices_nums[thread_num]].transpose();
	for (thread_num=1; thread_num<threads_size; thread_num++) {
		threads[thread_num]->get_thread_data(points[thread_num]);
		positions[thread_num] = (points[thread_num]).front();
		rotations[thread_num] = (threads[thread_num]->start_rot()) * rot_offset[constrained_vertices_nums[thread_num]].transpose();
	}
	positions[thread_num] = (points[thread_num-1]).back();
	rotations[thread_num] = (threads[thread_num-1])->end_rot() * rot_offset[constrained_vertices_nums[thread_num]].transpose();
}

void ThreadConstrained::setConstrainedTransforms(vector<Vector3d> positions, vector<Matrix3d> rotations) {
	int threads_size = threads.size();
	if (positions.size()!=(threads_size+1) || rotations.size()!=(threads_size+1))
    cout << "Internal error: getConstrainedTransforms: at least one of the vector parameters is of the wrong size" << endl;
	vector<Matrix3d> material_frames;
	get_thread_data(positions, material_frames);
	for (int i=0; i<constrained_vertices_nums.size(); i++) {
		if (constrained_vertices_nums[i]==0) {
			rot_offset[0] = rotations[i].transpose() * start_rot();
			rot_offset[0] = (Matrix3d) AngleAxisd(rot_offset[0]); // to fix numerical errors and rot_offset stays orthonormal
		} else if (constrained_vertices_nums[i]==num_vertices-1) {
			rot_offset[num_vertices-1] = rotations[i].transpose() * end_rot();
			rot_offset[num_vertices-1] = (Matrix3d) AngleAxisd(rot_offset[num_vertices-1]); // to fix numerical errors and rot_offset stays orthonormal
		} else {
			Matrix3d inter_rot;
			int vertex_num = constrained_vertices_nums[i];
			intermediateRotation(inter_rot, material_frames[vertex_num-1], material_frames[vertex_num]);
			rot_offset[vertex_num] = rotations[i].transpose() * material_frames[vertex_num];
			rot_offset[vertex_num] = (Matrix3d) AngleAxisd(rot_offset[vertex_num]); // to fix numerical errors and rot_offset stays orthonormal
		}
		if (LIMITED_DISPLACEMENT)
			last_rot[i] = rotations[i];
	}
}

void ThreadConstrained::setConstrainedTransforms(int constraint_ind, Vector3d position, Matrix3d rotation) {
	if (constraint_ind < 0 || constraint_ind >= threads.size()+1)
		cout << "Internal error: getConstrainedTransforms: constraint_ind is out of bounds" << endl;
	vector<Vector3d> positions;
	vector<Matrix3d> material_frames;
	get_thread_data(positions, material_frames);	
	if (constrained_vertices_nums[constraint_ind]==0) {
		rot_offset[0] = rotation.transpose() * start_rot();
		rot_offset[0] = (Matrix3d) AngleAxisd(rot_offset[0]); // to fix numerical errors and rot_offset stays orthonormal
	} else if (constrained_vertices_nums[constraint_ind]==num_vertices-1) {
		rot_offset[num_vertices-1] = rotation.transpose() * end_rot();
		rot_offset[num_vertices-1] = (Matrix3d) AngleAxisd(rot_offset[num_vertices-1]); // to fix numerical errors and rot_offset stays orthonormal
	} else {
		Matrix3d inter_rot;
		int vertex_num = constrained_vertices_nums[constraint_ind];
		intermediateRotation(inter_rot, material_frames[vertex_num-1], material_frames[vertex_num]);
		rot_offset[vertex_num] = rotation.transpose() * material_frames[vertex_num];
		rot_offset[vertex_num] = (Matrix3d) AngleAxisd(rot_offset[vertex_num]); // to fix numerical errors and rot_offset stays orthonormal
	}
	if (LIMITED_DISPLACEMENT)
		last_rot[constraint_ind] = rotation;
}

void ThreadConstrained::getAllTransforms(vector<Vector3d> &positions, vector<Matrix3d> &rotations) {
	vector<Matrix3d> material_frames;
	get_thread_data(positions, material_frames);
	rotations[0] = start_rot() * rot_offset[0].transpose();
	for(int vertex_num=1; vertex_num<num_vertices-1; vertex_num++) {
		rotations[vertex_num] = material_frames[vertex_num] * rot_offset[vertex_num].transpose();
	}
	rotations[num_vertices-1] = end_rot() * rot_offset[num_vertices-1].transpose();
}

void ThreadConstrained::setAllTransforms(vector<Vector3d> positions, vector<Matrix3d> rotations) {
	vector<Matrix3d> material_frames;
	get_thread_data(positions, material_frames);
	rot_offset[0] = rotations[0].transpose() * start_rot();
	rot_offset[0] = (Matrix3d) AngleAxisd(rot_offset[0]); // to fix numerical errors and rot_offset stays orthonormal
	for(int vertex_num=1; vertex_num<num_vertices-1; vertex_num++) {
		Matrix3d inter_rot;
		intermediateRotation(inter_rot, material_frames[vertex_num-1], material_frames[vertex_num]);
		rot_offset[vertex_num] = rotations[vertex_num].transpose() * material_frames[vertex_num];
		rot_offset[vertex_num] = (Matrix3d) AngleAxisd(rot_offset[vertex_num]); // to fix numerical errors and rot_offset stays orthonormal
	}
	rot_offset[num_vertices-1] = rotations[num_vertices-1].transpose() * end_rot();
	rot_offset[num_vertices-1] = (Matrix3d) AngleAxisd(rot_offset[num_vertices-1]); // to fix numerical errors and rot_offset stays orthonormal
	if (LIMITED_DISPLACEMENT)
		for (int i=0; i<constrained_vertices_nums.size(); i++)
			last_rot[i] = rotations[constrained_vertices_nums[i]];
}

void ThreadConstrained::getConstrainedVerticesNums(vector<int> &vertices_nums) {
	vertices_nums = constrained_vertices_nums;
}

void ThreadConstrained::getConstrainedVertices(vector<Vector3d> &constrained_vertices) {
	vector<Vector3d> vertices;
	this->get_thread_data(vertices);
	constrained_vertices.clear();
	for (int i=0; i<constrained_vertices_nums.size(); i++) {
		constrained_vertices.push_back(vertices[constrained_vertices_nums[i]]);
	}
}

void ThreadConstrained::getFreeVerticesNums(vector<int> &free_vertices_nums) {
	free_vertices_nums.resize(num_vertices);
	for (int i=0; i<free_vertices_nums.size(); i++)
		free_vertices_nums[i] = i;
  vector<int> ind_vect(num_vertices);
  invalidateConstraintsNums(ind_vect, constrained_vertices_nums);
	for(int i=num_vertices-1; i>=0; i--)
		if (ind_vect[i]==-1)
			free_vertices_nums.erase(free_vertices_nums.begin()+i);
}

void ThreadConstrained::getFreeVertices(vector<Vector3d> &free_vertices_num) {
  this->get_thread_data(free_vertices_num);
  vector<int> ind_vect(num_vertices);
  invalidateConstraintsNums(ind_vect, constrained_vertices_nums);
	for(int i=num_vertices-1; i>=0; i--)
		if (ind_vect[i]==-1)
			free_vertices_num.erase(free_vertices_num.begin()+i);
}

void ThreadConstrained::getOperableFreeVertices(vector<int> &free_vertices_num) {
	free_vertices_num.resize(num_vertices);
	for(int i=0; i<num_vertices; i++)
		free_vertices_num[i] = i;
	vector<int> ind_vect(num_vertices);
	invalidateAroundConstraintsNums(ind_vect, constrained_vertices_nums);
	for(int i=0; i<constrained_vertices_nums.size(); i++)
		ind_vect[constrained_vertices_nums[i]] = -1;
	for(int i=num_vertices-1; i>=0; i--)
		if (ind_vect[i]==-1)
			free_vertices_num.erase(free_vertices_num.begin()+i);
}

void ThreadConstrained::getOperableVertices(vector<int> &operable_vertices_num, vector<bool> &constrained_or_free) {
	operable_vertices_num.resize(num_vertices);
	constrained_or_free.resize(num_vertices);
	for(int i=0; i<num_vertices; i++) {
		operable_vertices_num[i] = i;
		constrained_or_free[i] = false;
	}
	for(int i=0; i<constrained_vertices_nums.size(); i++)
		constrained_or_free[constrained_vertices_nums[i]] = true;
	vector<int> ind_vect(num_vertices);
	invalidateAroundConstraintsNums(ind_vect, constrained_vertices_nums);
	for(int i=num_vertices-1; i>=0; i--) {
		if (ind_vect[i]==-1) {
			operable_vertices_num.erase(operable_vertices_num.begin()+i);
			constrained_or_free.erase(constrained_or_free.begin()+i);
		}
	}
}

Vector3d ThreadConstrained::start_pos() {
	return (threads.front())->start_pos();
}

Vector3d ThreadConstrained::end_pos() {
	return (threads.back())->end_pos();
}

Matrix3d ThreadConstrained::start_rot() {
	return (threads.front())->start_rot();
}

Matrix3d ThreadConstrained::end_rot() {
	return (threads.back())->end_rot();
}

void ThreadConstrained::set_coeffs_normalized(double bend_coeff, double twist_coeff, double grav_coeff) {
	for (int thread_num=0; thread_num<threads.size(); thread_num++)
		threads[thread_num]->set_coeffs_normalized(bend_coeff, twist_coeff, grav_coeff);
}

void ThreadConstrained::set_coeffs_normalized(const Matrix2d& bend_coeff, double twist_coeff, double grav_coeff) {
	for (int thread_num=0; thread_num<threads.size(); thread_num++)
		threads[thread_num]->set_coeffs_normalized(bend_coeff, twist_coeff, grav_coeff);
}

void ThreadConstrained::minimize_energy() {
	for (int thread_num=0; thread_num<threads.size(); thread_num++) {
		threads[thread_num]->minimize_energy();
	}
}

void ThreadConstrained::adapt_links() {
	for (int thread_num=0; thread_num<threads.size(); thread_num++) {
		threads[thread_num]->adapt_links();
	}
}

void ThreadConstrained::updateConstraints (vector<Vector3d> poss, vector<Matrix3d> rots) {
	if (poss.size() != rots.size())
		cout << "Internal Error: setConstraints: different number of positions and rotations" << endl;
	if (poss.size() != threads.size()+1)
		cout << "Internal Error: setConstraints: not the right number of transforms for current threads. addConstraint or removeConstraint first." << endl;
	
	//the following loop is to synchronize the end positions of two threads held by the same end-effector
	bool wrong_positions = true;
	int iters = 0;
	while (wrong_positions) {
		wrong_positions = false;
		iters++;
		for (int i=0; i<threads.size(); i++) {
			Matrix3d new_start_rot = rots[i]*rot_offset[constrained_vertices_nums[i]];
			Matrix3d new_end_rot 	 = rots[i+1]*rot_offset[constrained_vertices_nums[i+1]]*rot_diff[i+1];
			wrong_positions = wrong_positions || threads[i]->check_fix_positions(poss[i], new_start_rot, poss[i+1], new_end_rot);
		}
		if (iters == 10) { break; }
	}
	
	for (int i=0; i<threads.size(); i++) {
		Vector3d new_start_pos;
		Vector3d new_end_pos;
		Matrix3d new_start_rot;
		Matrix3d new_end_rot;
			
		if (LIMITED_DISPLACEMENT) {
			double start_displacement = (poss[i]-last_pos[i]).norm();
			double end_displacement 	= (poss[i+1]-last_pos[i+1]).norm();
			double start_angle_change	= 2*asin((rots[i].col(0) - last_rot[i].col(0)).norm()/2);
			double end_angle_change		= 2*asin((rots[i+1].col(0) - last_rot[i+1].col(0)).norm()/2);
			Quaterniond new_start_q(rots[i]);
	 		Quaterniond last_start_q(last_rot[i]);
			Quaterniond new_end_q(rots[i+1]);
			Quaterniond last_end_q(last_rot[i+1]);		
			if (start_displacement > MAX_DISPLACEMENT)
				poss[i] = last_pos[i] + (MAX_DISPLACEMENT/start_displacement) * (poss[i]-last_pos[i]);
			if (end_displacement > MAX_DISPLACEMENT)
				poss[i+1] = last_pos[i+1] + (MAX_DISPLACEMENT/end_displacement) * (poss[i+1]-last_pos[i+1]);
			if (start_angle_change > MAX_ANGLE_CHANGE) {
				Quaterniond start_interp_q = last_start_q.slerp(MAX_ANGLE_CHANGE/start_angle_change, new_start_q);
				rots[i] = start_interp_q.toRotationMatrix(); 
			}
			if (end_angle_change > MAX_ANGLE_CHANGE) {
				Quaterniond end_interp_q = last_end_q.slerp(MAX_ANGLE_CHANGE/end_angle_change, new_end_q);
				rots[i+1] = end_interp_q.toRotationMatrix();
			}
		}
		new_start_pos = poss[i];
		new_end_pos 	= poss[i+1];
		new_start_rot = rots[i]*rot_offset[constrained_vertices_nums[i]];
		new_end_rot 	= rots[i+1]*rot_offset[constrained_vertices_nums[i+1]]*rot_diff[i+1];
				
		double angle_change = angle_mismatch(new_start_rot, threads[i]->start_rot());
		if (i==0)
			zero_angle += angle_change - M_PI*((int) (angle_change/M_PI));
	  threads[i]->set_constraints_check(new_start_pos, new_start_rot, new_end_pos, new_end_rot);
	  if (LIMITED_DISPLACEMENT) {
			last_pos[i] 	= new_start_pos;
			last_pos[i+1] = new_end_pos;
			last_rot[i]		= rots[i];
			last_rot[i+1] = rots[i+1];
	  }
	}
}

void ThreadConstrained::addConstraint (int absolute_vertex_num) {
	int thread_num = insertSorted(constrained_vertices_nums, absolute_vertex_num)-1;
	splitThread(thread_num, localVertex(absolute_vertex_num));
}

void ThreadConstrained::removeConstraint (int absolute_vertex_num) {
	int thread_num = removeSorted(constrained_vertices_nums, absolute_vertex_num) - 1;
	mergeThread(thread_num);
}

// Returns the number of the vertex that is nearest to pos. The chosen vertex have to be in vertices but not in vertex_exception.
int ThreadConstrained::nearestVertex(Vector3d pos) {
	vector<Vector3d> vertices;
	get_thread_data(vertices);
	vector<int> free_vertices_num;
	getOperableFreeVertices(free_vertices_num);
	vector<double> distances;
	for (int i=0; i<free_vertices_num.size(); i++) {
		distances.push_back((vertices[free_vertices_num[i]]-pos).squaredNorm());
	}
	int min_ind = 0;
	for (int i=1; i<free_vertices_num.size(); i++)
		if (distances[i]<distances[min_ind])
			min_ind = i;
	return free_vertices_num[min_ind];
}

void ThreadConstrained::initializeThreadsInEnvironment() {
	vector<ThreadConstrained*>* all_thread_constrained = world->getThreads();
	vector<Thread*> all_threads;
	for (int k=0; k<(*all_thread_constrained).size(); k++) {
		ThreadConstrained* thread_constrained = (*all_thread_constrained)[k];
		for (int i=0; i<thread_constrained->threads.size(); i++)
			all_threads.push_back(thread_constrained->threads[i]);
	}
	for (int i=0; i<all_threads.size(); i++) {
		all_threads[i]->clear_threads_in_env();
		for (int j=0; j<all_threads.size(); j++) {
			if (i!=j) 
				all_threads[i]->add_thread_to_env(all_threads[j]);
		}
	}
}

Vector3d ThreadConstrained::position(int absolute_vertex_num) {
	vector<Vector3d> positions;
	get_thread_data(positions);
	return positions[absolute_vertex_num];
}

Matrix3d ThreadConstrained::rotation(int absolute_vertex_num) {
	vector<Vector3d> positions;
	vector<Matrix3d> material_frames;
	get_thread_data(positions, material_frames);	
	Matrix3d vertex_start_rot;
	if (absolute_vertex_num == 0) {
		vertex_start_rot = start_rot();
	} else if (absolute_vertex_num ==num_vertices-1) {
		vertex_start_rot = end_rot();
	} else {
		vertex_start_rot = material_frames[absolute_vertex_num];
	}
	return vertex_start_rot * rot_offset[absolute_vertex_num].transpose();
}

void ThreadConstrained::draw() {
	vector<Vector3d> points;
	vector<double> twist_angles;	
	get_thread_data(points, twist_angles);

  glPushMatrix();
  glColor3f (0.5, 0.5, 0.2);
  double pts_cpy[points.size()+2][3];
  double twist_cpy[points.size()+2]; 
  for (int i=0; i < points.size(); i++)
  {
    pts_cpy[i+1][0] = points[i](0);
    pts_cpy[i+1][1] = points[i](1);
    pts_cpy[i+1][2] = points[i](2);
   	twist_cpy[i+1] = -(360.0/(2.0*M_PI))*(twist_angles[i]);
  }
  //add first and last point
  pts_cpy[0][0] = 2*pts_cpy[1][0] - pts_cpy[2][0];
  pts_cpy[0][1] = 2*pts_cpy[1][1] - pts_cpy[2][1];
  pts_cpy[0][2] = 2*pts_cpy[1][2] - pts_cpy[2][2];
  twist_cpy[0] = twist_cpy[1];

  pts_cpy[points.size()+1][0] = 2*pts_cpy[points.size()][0] - pts_cpy[points.size()-1][0];
  pts_cpy[points.size()+1][1] = 2*pts_cpy[points.size()][1] - pts_cpy[points.size()-1][1];
  pts_cpy[points.size()+1][2] = 2*pts_cpy[points.size()][2] - pts_cpy[points.size()-1][2];
  twist_cpy[points.size()+1] = twist_cpy[points.size()];

  gleTwistExtrusion(20,
      contour,
      contour_norms,
      NULL,
      points.size()+2,
      pts_cpy,
      0x0,
      twist_cpy);
      
  if (examine_mode)
		for (int i=0; i<points.size(); i++)
			drawSphere(points[i], 0.7, 0.0, 0.5, 0.5);
			
  glPopMatrix ();
}

void ThreadConstrained::initContour()
{
  int style;

  /* pick model-vertex-cylinder coords for texture mapping */
  //TextureStyle (509);

  /* configure the pipeline */
  style = TUBE_JN_CAP;
  style |= TUBE_CONTOUR_CLOSED;
  style |= TUBE_NORM_FACET;
  style |= TUBE_JN_ANGLE;
  gleSetJoinStyle (style);

  int i;
  double contour_scale_factor= 0.3;
   if (examine_mode)
    contour_scale_factor = 0.05;

#ifdef ISOTROPIC
  // outline of extrusion
  i=0;
  CONTOUR (1.0 *contour_scale_factor, 1.0 *contour_scale_factor);
  CONTOUR (1.0 *contour_scale_factor, 2.9 *contour_scale_factor);
  CONTOUR (0.9 *contour_scale_factor, 3.0 *contour_scale_factor);
  CONTOUR (-0.9*contour_scale_factor, 3.0 *contour_scale_factor);
  CONTOUR (-1.0*contour_scale_factor, 2.9 *contour_scale_factor);

  CONTOUR (-1.0*contour_scale_factor, 1.0 *contour_scale_factor);
  CONTOUR (-2.9*contour_scale_factor, 1.0 *contour_scale_factor);
  CONTOUR (-3.0*contour_scale_factor, 0.9 *contour_scale_factor);
  CONTOUR (-3.0*contour_scale_factor, -0.9*contour_scale_factor);
  CONTOUR (-2.9*contour_scale_factor, -1.0*contour_scale_factor);

  CONTOUR (-1.0*contour_scale_factor, -1.0*contour_scale_factor);
  CONTOUR (-1.0*contour_scale_factor, -2.9*contour_scale_factor);
  CONTOUR (-0.9*contour_scale_factor, -3.0*contour_scale_factor);
  CONTOUR (0.9 *contour_scale_factor, -3.0*contour_scale_factor);
  CONTOUR (1.0 *contour_scale_factor, -2.9*contour_scale_factor);

  CONTOUR (1.0 *contour_scale_factor, -1.0*contour_scale_factor);
  CONTOUR (2.9 *contour_scale_factor, -1.0*contour_scale_factor);
  CONTOUR (3.0 *contour_scale_factor, -0.9*contour_scale_factor);
  CONTOUR (3.0 *contour_scale_factor, 0.9 *contour_scale_factor);
  CONTOUR (2.9 *contour_scale_factor, 1.0 *contour_scale_factor);

  CONTOUR (1.0 *contour_scale_factor, 1.0 *contour_scale_factor);   // repeat so that last normal is computed
#else
  // outline of extrusion
  i=0;
  CONTOUR (1.0*contour_scale_factor, 0.0*contour_scale_factor);
  CONTOUR (1.0*contour_scale_factor, 0.5*contour_scale_factor);
  CONTOUR (1.0*contour_scale_factor, 1.0*contour_scale_factor);
  CONTOUR (1.0*contour_scale_factor, 2.0*contour_scale_factor);
  CONTOUR (1.0*contour_scale_factor, 2.9*contour_scale_factor);
  CONTOUR (0.9*contour_scale_factor, 3.0*contour_scale_factor);
  CONTOUR (0.0*contour_scale_factor, 3.0*contour_scale_factor);
  CONTOUR (-0.9*contour_scale_factor, 3.0*contour_scale_factor);
  CONTOUR (-1.0*contour_scale_factor, 2.9*contour_scale_factor);

  CONTOUR (-1.0*contour_scale_factor, 2.0*contour_scale_factor);
  CONTOUR (-1.0*contour_scale_factor, 1.0*contour_scale_factor);
  CONTOUR (-1.0*contour_scale_factor, 0.5*contour_scale_factor);
  CONTOUR (-1.0*contour_scale_factor, 0.0*contour_scale_factor);
  CONTOUR (-1.0*contour_scale_factor, -0.5*contour_scale_factor);
  CONTOUR (-1.0*contour_scale_factor, -1.0*contour_scale_factor);
  CONTOUR (-1.0*contour_scale_factor, -2.0*contour_scale_factor);
  CONTOUR (-1.0*contour_scale_factor, -2.9*contour_scale_factor);
  CONTOUR (-0.9*contour_scale_factor, -3.0*contour_scale_factor);
  CONTOUR (0.0*contour_scale_factor, -3.0*contour_scale_factor);
  CONTOUR (0.9*contour_scale_factor, -3.0*contour_scale_factor);
  CONTOUR (1.0*contour_scale_factor, -2.9*contour_scale_factor);
  CONTOUR (1.0*contour_scale_factor, -2.0*contour_scale_factor);
  CONTOUR (1.0*contour_scale_factor, -1.0*contour_scale_factor);
  CONTOUR (1.0*contour_scale_factor, -0.5*contour_scale_factor);

  CONTOUR (1.0*contour_scale_factor, 0.0*contour_scale_factor);   // repeat so that last normal is computed
#endif
}

void ThreadConstrained::toggleExamineMode() {
	examine_mode = !examine_mode;
  initContour();
}

void ThreadConstrained::setWorld(World* w) {
	world = w;
	for(int i=0; i<threads.size(); i++)
		threads[i]->setWorld(world);
}

void ThreadConstrained::intermediateRotation(Matrix3d &inter_rot, Matrix3d end_rot, Matrix3d start_rot) {
	Quaterniond start_q(start_rot);
  Quaterniond end_q(end_rot);
  Quaterniond interp_q = start_q.slerp(0.5, end_q);
  inter_rot = interp_q.toRotationMatrix();
}

// Splits the thread threads[thread_num] into two threads, which are stored at threads[thread_num] and threads[thread_num+1].  Threads in threads that are stored after thread_num now have a new thread_num which is one unit more than before. The split is done at vertex vertex of thread[thread_num]
void ThreadConstrained::splitThread(int thread_num, int vertex_num) {
	vector<Vector3d> point0;
	vector<Vector3d> point1;
	vector<Vector3d> points;
	vector<double> twist_angle0;
	vector<double> twist_angle1;
	vector<double> twist_angles;
	threads[thread_num]->get_thread_data(points, twist_angles);

	splitVector(point0, point1, points, vertex_num);
	splitVector(twist_angle0, twist_angle1, twist_angles, vertex_num);

  Matrix3d vertex_end_rot = threads[thread_num]->material_at_ind(vertex_num-1);
  Matrix3d vertex_start_rot = threads[thread_num]->material_at_ind(vertex_num);
  Matrix3d rot_diff_matrix = vertex_start_rot.transpose()*vertex_end_rot;
  rot_diff.insert(rot_diff.begin()+thread_num+1, rot_diff_matrix);
 	if (LIMITED_DISPLACEMENT) {
 		last_pos.insert(last_pos.begin()+thread_num+1, points[vertex_num]);
 		last_rot.insert(last_rot.begin()+thread_num+1, (Matrix3d) (vertex_start_rot * rot_offset[constrained_vertices_nums[thread_num+1]].transpose()));
 	}
	
	mapAdd(twist_angle1, -twist_angle1.front());
	twist_angle0.back() = 2.0*twist_angle0[twist_angle0.size()-2] - twist_angle0[twist_angle0.size()-3];
	twist_angle1.back() = 2.0*twist_angle1[twist_angle1.size()-2] - twist_angle1[twist_angle1.size()-3];
	
	Thread* thread0 = new Thread(point0, twist_angle0, (Matrix3d&) (threads[thread_num])->start_rot(), vertex_end_rot);
	Thread* thread1 = new Thread(point1, twist_angle1, vertex_start_rot, (Matrix3d&) (threads[thread_num])->end_rot());
	thread0->setWorld(world);
	thread1->setWorld(world);
	delete threads[thread_num];
	threads[thread_num] = thread0;
	threads.insert(threads.begin()+thread_num+1, thread1);
	
	initializeThreadsInEnvironment();
	
	threads[thread_num]->minimize_energy();
	threads[thread_num+1]->minimize_energy();
	threads[thread_num]->get_thread_data(point0, twist_angle0);
	threads[thread_num+1]->get_thread_data(point1, twist_angle1);
}

// Merges the threads threads[thread_num] and threads[thread_num+1] into one thread, which is stored at threads[thread_num]. Threads in threads that are stored after thread_num+1 now have a new thread_num which is one unit less than before.
void ThreadConstrained::mergeThread(int thread_num) {
	vector<Vector3d> points0;
	vector<Vector3d> points1;
	vector<Vector3d> point;
	vector<double> twist_angles0;
	vector<double> twist_angles1;
	threads[thread_num]->get_thread_data(points0, twist_angles0);
	threads[thread_num+1]->get_thread_data(points1, twist_angles1);
	vector<double> twist_angle(twist_angles0.size()+twist_angles1.size()-1);

	mergeVector(point, points0, points1);
	mergeVector(twist_angle, twist_angles0, twist_angles1);
	
	rot_diff.erase(rot_diff.begin() + thread_num+1);
	if (LIMITED_DISPLACEMENT) {
		last_pos.erase(last_pos.begin() + thread_num+1);
		last_rot.erase(last_rot.begin() + thread_num+1);
	}

	Thread* thread = new Thread(point, twist_angle, (Matrix3d&) (threads[thread_num])->start_rot(), (Matrix3d&) (threads[thread_num+1])->end_rot());
	thread->setWorld(world);
	delete threads[thread_num];
	delete threads[thread_num+1];
	threads[thread_num] = thread;
	threads.erase(threads.begin()+thread_num+1);
	
	initializeThreadsInEnvironment();
	
	threads[thread_num]->minimize_energy();
}

// Returns the thread number that owns the absolute_vertex_num. absolute_vertex_num must not be a constrained vertex. Might be buggy?
int ThreadConstrained::threadOwner(int absolute_vertex_num) {
	int thread_num;
	int num_absolute_vertices = 1;
	vector<vector<Vector3d> > points(threads.size());
	for (thread_num=0; thread_num<threads.size(); thread_num++) {
		threads[thread_num]->get_thread_data(points[thread_num]);
		num_absolute_vertices += (points[thread_num]).size() - 1;
		if (absolute_vertex_num < num_absolute_vertices) { break; }
	}
	if (thread_num==threads.size()) {
		cout << "Internal error: threadOwner: absolute_vertex_num is greater than total number of vertices" << endl;
		return 0;
	} else {
		return thread_num;
	}
}

// Returns the local vertex number (i.e. vertex number within a thread), given the absolute vertex number (i.e. vertex number within all vertices).
int ThreadConstrained::localVertex(int absolute_vertex_num) {
	int thread_num;
	int num_absolute_vertices = 1;
	vector<vector<Vector3d> > points(threads.size());
	for (thread_num=0; thread_num<threads.size(); thread_num++) {
		threads[thread_num]->get_thread_data(points[thread_num]);
		num_absolute_vertices += (points[thread_num]).size() - 1;
		if (absolute_vertex_num < num_absolute_vertices) { break; }
	}
	if (thread_num==threads.size()) {
		cout << "Internal error: localVertex: absolute_vertex_num is greater than total number of vertices" << endl;
		return 0;
	} else {
		return absolute_vertex_num + (points[thread_num]).size() - num_absolute_vertices;
	}
}

// Invalidates (sets to -1) the elements of v at indices around i. Indices i are specified by constraintsNums. Indices in constraintsNums have to be in the range of v.
void invalidateAroundConstraintsNums(vector<int> &v, vector<int> constraintsNums) {
	if (v.size()<7)
		cout << "Internal error: invalidateAroundConstraintsNums: vector v is too small" << endl;
	for (int i=0; i<v.size(); i++)
		v[i] = i;
	for (int i=0; i<constraintsNums.size(); i++) {
		if (constraintsNums[i] == 0) {
			v[constraintsNums[i]+1] = v[constraintsNums[i]+2] = v[constraintsNums[i]+3] = -1;
		} else if (constraintsNums[i] == 1) {
			v[constraintsNums[i]-1] = v[constraintsNums[i]+1] = v[constraintsNums[i]+2] = v[constraintsNums[i]+3] = -1;
		} else if (constraintsNums[i] == 2) {
			v[constraintsNums[i]-2] = v[constraintsNums[i]-1] = v[constraintsNums[i]+1] = v[constraintsNums[i]+2] = v[constraintsNums[i]+3] = -1;
		} else if (constraintsNums[i] == v.size()-1) {
			v[constraintsNums[i]-3] = v[constraintsNums[i]-2] = v[constraintsNums[i]-1] = -1;
		} else if (constraintsNums[i] == v.size()-2) {
			v[constraintsNums[i]-3] = v[constraintsNums[i]-2] = v[constraintsNums[i]-1] = v[constraintsNums[i]+1] = -1;
		} else if (constraintsNums[i] == v.size()-3) {
			v[constraintsNums[i]-3] = v[constraintsNums[i]-2] = v[constraintsNums[i]-1] = v[constraintsNums[i]+1] = v[constraintsNums[i]+2] = -1;
		} else {
			v[constraintsNums[i]-3] = v[constraintsNums[i]-2] = v[constraintsNums[i]-1] = v[constraintsNums[i]+1] = v[constraintsNums[i]+2] = v[constraintsNums[i]+3] = -1;
		}
	}
}

// Invalidates (sets to -1) the elements of v at indices i. Indices i are specified by constraintsNums. Indices in constraintsNums have to be in the range of v.
void invalidateConstraintsNums(vector<int> &v, vector<int> constraintsNums) {
	for (int i=0; i<v.size(); i++)
		v[i] = i;
	for (int i=0; i<constraintsNums.size(); i++) {
		v[constraintsNums[i]] = -1;
	}
}

template<typename T>
void mapAdd (vector<T> &v, T num) {
	for (int i=0; i<v.size(); i++)
		v[i] = v[i] + num;
}

// Last element of v1 and first element of v2 are equal to v[index].
template<typename T>
void splitVector (vector<T> &v1, vector<T> &v2, vector<T> v, int index) {
	v1.clear();
	v2.clear();
	v1.insert(v1.begin(), v.begin(), v.begin() + index + 1);
	v2.insert(v2.begin(), v.begin() + index, v.end());
}
// Last element of v1 is discarded since it is assumed to be the same as the first element of v2.
template<typename T>
void mergeVector (vector<T> &v, vector<T> v1, vector<T> v2) {
	v = v1;
	v.pop_back();
	v.insert(v.end(), v2.begin(), v2.end());
}

// Last element of a vector in vectors is discarded since it is assumed to be the same as the first element of the consecutive vector in vectors. The last vector in vectors is the exception.
template<typename T>
void mergeMultipleVector(vector<T> &v, vector<vector<T> > vectors) {
	if (vectors.size()==1) {
		v = vectors[0];
	} else {
		mergeVector(v,vectors[0],vectors[1]);
		for (int i=2; i < vectors.size(); i++)
			mergeVector(v,v,vectors[i]);
	}
}

// Returns the position where the element was inserted.
int insertSorted (vector<int> &v, int e) {
	if (!v.size()) {
		v.push_back(e);
		return 0;
	}
	int last = v.size() - 1;
	v.push_back(v[last]);
	int i;
	for (i=last; i>=0 && v[i]>e; i--)
		v[i] = v[i - 1];
	if (v[i]==e)
		cout << "Internal error: insertSorted: element to insert is already in vector" << endl;
	v[i+1] = e;
	return i+1;
}

//Returns the position where the element was removed. Element to remove has to be in vector.
int removeSorted (vector<int> &v, int e) {
	int i;
	for (i=0; i<v.size() && v[i]!=e; i++) {}
	if (i==v.size())
		cout << "Internal error: remove: element to remove is not in vector" << endl;
	v.erase(v.begin()+i);
	return i;
}

//Returns the position of the element to be found.
int find(vector<int> v, int e) {
	int i;
	for (i=0; i<v.size() && v[i]!=e; i++) {}
	if (i==v.size())
		return -1;
	return i;
}
