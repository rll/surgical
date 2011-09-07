#include "ThreadConstrained.h"
#include "thread_discrete.h"

/*ThreadConstrained::ThreadConstrained() {
	type = THREAD_CONSTRAINED;
	world = NULL;
}*/

ThreadConstrained::ThreadConstrained(vector<Vector3d>& vertices, vector<double>& twist_angles, vector<double>& rest_lengths, Matrix3d& start_rot, Matrix3d& end_rot, World* w) {
	num_vertices = vertices.size();
	Thread* thread = new Thread(vertices, twist_angles, rest_lengths, start_rot, end_rot);
	thread->setWorld(w);
	threads.push_back(thread);

	thread->minimize_energy();
	
	zero_angle = 0.0;
	rot_diff.push_back(Matrix3d::Identity());
	rot_diff.push_back(Matrix3d::Identity());
	rot_offset.push_back((Matrix3d) (AngleAxisd(M_PI, Vector3d::UnitZ())));
	rot_offset.push_back(Matrix3d::Identity());

	constrained_vertices_nums.push_back(0);
	constrained_vertices_nums.push_back(num_vertices-1);
	
	type = THREAD_CONSTRAINED;
	
	world = w;
	examine_mode = false;
	initContour();
}

ThreadConstrained::ThreadConstrained(vector<Vector3d>& vertices, vector<double>& twist_angles, vector<double>& rest_lengths, Matrix3d& start_rot, World* w) {
	num_vertices = vertices.size();
	Thread* thread = new Thread(vertices, twist_angles, rest_lengths, start_rot);
	thread->setWorld(w);
	threads.push_back(thread);

	thread->minimize_energy();
	
	zero_angle = 0.0;
	rot_diff.push_back(Matrix3d::Identity());
	rot_diff.push_back(Matrix3d::Identity());
	rot_offset.push_back((Matrix3d) (AngleAxisd(M_PI, Vector3d::UnitZ())));
	rot_offset.push_back(Matrix3d::Identity());

	constrained_vertices_nums.push_back(0);
	constrained_vertices_nums.push_back(num_vertices-1);
	
	type = THREAD_CONSTRAINED;
	
	world = w;
	examine_mode = false;
	initContour();
}

ThreadConstrained::ThreadConstrained(vector<Vector3d>& vertices, vector<double>& twist_angles, Matrix3d& start_rot, Matrix3d& end_rot, World* w) {
	num_vertices = vertices.size();
	Thread* thread = new Thread(vertices, twist_angles, start_rot, end_rot);
	thread->setWorld(w);
	threads.push_back(thread);

	thread->minimize_energy();

	zero_angle = 0.0;
	rot_diff.push_back(Matrix3d::Identity());
	rot_diff.push_back(Matrix3d::Identity());
	rot_offset.push_back((Matrix3d) (AngleAxisd(M_PI, Vector3d::UnitZ())));
	rot_offset.push_back(Matrix3d::Identity());

	constrained_vertices_nums.push_back(0);
	constrained_vertices_nums.push_back(num_vertices-1);
	
	type = THREAD_CONSTRAINED;
	
	world = w;
	examine_mode = false;
	initContour();
}

ThreadConstrained::ThreadConstrained(int num_vertices_init, World* w) {
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
	thread->setWorld(w);
	threads.push_back(thread);

	thread->minimize_energy();

	zero_angle = 0.0;
	rot_diff.push_back(Matrix3d::Identity());
	rot_diff.push_back(Matrix3d::Identity());
	rot_offset.push_back((Matrix3d) (AngleAxisd(M_PI, Vector3d::UnitZ())));
	rot_offset.push_back(Matrix3d::Identity());

	constrained_vertices_nums.push_back(0);
	constrained_vertices_nums.push_back(num_vertices_init-1);
	
	type = THREAD_CONSTRAINED;
	
	world = w;
	examine_mode = false;
	initContour();
}

ThreadConstrained::ThreadConstrained(const ThreadConstrained& rhs, World* w)
{
	if (rhs.type != THREAD_CONSTRAINED)
		cerr << "Internal error: ThreadConstrained(const ThreadConstrained& rhs): rhs.type is not THREAD_CONSTRAINED." << endl;
	type = rhs.type;
	
	num_vertices = rhs.num_vertices;
	zero_angle = rhs.zero_angle;
	world = w;

	threads.resize(rhs.threads.size());	 
  for (int thread_ind = 0; thread_ind < rhs.threads.size(); thread_ind++) {
    threads[thread_ind] = new Thread(*rhs.threads[thread_ind]);
		threads[thread_ind]->setWorld(w);
	}
	
	rot_diff = rhs.rot_diff;
	rot_offset = rhs.rot_offset;
	constrained_vertices_nums = rhs.constrained_vertices_nums;

	examine_mode = false;
	initContour();
}

void ThreadConstrained::backup()
{
	backup_num_vertices = num_vertices;
	
	for (int thread_ind = 0; thread_ind < backup_threads.size(); thread_ind++)
    delete backup_threads[thread_ind];
	
	backup_threads.resize(threads.size());	 
  for (int thread_ind = 0; thread_ind < threads.size(); thread_ind++) {
    backup_threads[thread_ind] = new Thread(*threads[thread_ind]);
    backup_threads[thread_ind]->setWorld(world);
  }
	
	backup_zero_angle = zero_angle;

	backup_rot_diff = rot_diff;	
	backup_rot_offset = rot_offset;
	backup_constrained_vertices_nums = constrained_vertices_nums;
}

void ThreadConstrained::restore()
{
	num_vertices = backup_num_vertices;
	
	for (int thread_ind = 0; thread_ind < threads.size(); thread_ind++)
    delete threads[thread_ind];
	
	int computed_num_vertices = 1;
	threads.resize(backup_threads.size());	 
  for (int thread_ind = 0; thread_ind < backup_threads.size(); thread_ind++)
  {
    threads[thread_ind] = new Thread(*backup_threads[thread_ind]);
    threads[thread_ind]->setWorld(world);
    computed_num_vertices += (int)threads[thread_ind]->num_pieces()-1;
  }
  assert(computed_num_vertices == num_vertices);
	
	zero_angle = backup_zero_angle;

	rot_diff = backup_rot_diff;	
	rot_offset = backup_rot_offset;
	constrained_vertices_nums = backup_constrained_vertices_nums;
	
	examine_mode = false;
	initContour();
}

void ThreadConstrained::writeToFile(ofstream& file)
{
	file << type << " ";
	file << num_vertices << " ";

	file << threads.size() << " ";
	//write each point for each thread
  for (int i=0; i < threads.size(); i++)
  {
    vector<Vector3d> points;
		vector<double> twist_angles;
		vector<double> rest_lengths;
		threads[i]->get_thread_data(points, twist_angles, rest_lengths);
    Matrix3d start_rot = threads[i]->start_rot();
    Matrix3d end_rot = threads[i]->end_rot();

		file << points.size() << " ";

    for (int r=0; r < 3; r++)
      for (int c=0; c < 3; c++)
        file << start_rot (r,c) << " ";

    for (int r=0; r < 3; r++)
      for (int c=0; c < 3; c++)
        file << end_rot (r,c) << " ";

    for (int j=0; j < points.size(); j++)
      file << points[j](0) << " " << points[j](1) << " " << points[j](2) << " " << twist_angles[j] << " " << rest_lengths[j] << " ";
  }
	
	file << zero_angle << " ";
	
	file << rot_diff.size() << " ";
	for (int i=0; i<rot_diff.size(); i++)
		for (int r=0; r < 3; r++)
		  for (int c=0; c < 3; c++)
		    file << rot_diff[i](r,c) << " ";

	file << rot_offset.size() << " ";
	for (int i=0; i<rot_offset.size(); i++)
		for (int r=0; r < 3; r++)
		  for (int c=0; c < 3; c++)
		    file << rot_offset[i](r,c) << " ";

	file << constrained_vertices_nums.size() << " ";
	for (int k=0; k<constrained_vertices_nums.size(); k++)
		file << constrained_vertices_nums[k] << " ";

  file << "\n";
}

// world need to be set after creating this
ThreadConstrained::ThreadConstrained(ifstream& file, World* w)
{
	type = THREAD_CONSTRAINED;
	file >> num_vertices;

	int size;
	file >> size;

	threads.resize(size);
	//write each point for each thread
  for (int i=0; i < threads.size(); i++)
  {
    file >> size;
    
    vector<Vector3d> points(size);
		vector<double> twist_angles(size);
		vector<double> rest_lengths(size);
    Matrix3d start_rot;
    Matrix3d end_rot;

    for (int r=0; r < 3; r++)
      for (int c=0; c < 3; c++)
        file >> start_rot (r,c);

    for (int r=0; r < 3; r++)
      for (int c=0; c < 3; c++)
        file >> end_rot (r,c);

    for (int j=0; j < points.size(); j++)
      file >> points[j](0) >> points[j](1) >> points[j](2) >> twist_angles[j] >> rest_lengths[j];
    
    threads[i] = new Thread(points, twist_angles, rest_lengths, start_rot, end_rot);
    threads[i]->setWorld(w);
  }
	
	file >> zero_angle;
	
	file >> size;
	rot_diff.resize(size);
	for (int i=0; i<rot_diff.size(); i++)
		for (int r=0; r < 3; r++)
		  for (int c=0; c < 3; c++)
		    file >> rot_diff[i](r,c);

	file >> size;
	rot_offset.resize(size);
	for (int i=0; i<rot_offset.size(); i++)
		for (int r=0; r < 3; r++)
		  for (int c=0; c < 3; c++)
		    file >> rot_offset[i](r,c);

	file >> size;
	constrained_vertices_nums.resize(size);
	for (int k=0; k<constrained_vertices_nums.size(); k++)
		file >> constrained_vertices_nums[k];

	world = w;
	examine_mode = false;
	initContour();
}

int ThreadConstrained::numVertices()
{
	checkNumVertices();
	return num_vertices;
}

void ThreadConstrained::checkNumVertices()
{
#ifndef NDEBUG
	vector<Vector3d> vertices;
	get_thread_data(vertices);
	assert(num_vertices == vertices.size());
#endif
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
		//TODO
		//if (thread_num==0)
		//	mapAdd(twist_angles[thread_num], zero_angle);
		//else
		//	mapAdd(twist_angles[thread_num], twist_angles[thread_num-1].back());
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
	 	//TODO
	 	//if (thread_num==0)
		//	mapAdd(twist_angles[thread_num], zero_angle);
		//else
		//	mapAdd(twist_angles[thread_num], twist_angles[thread_num-1].back());
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
	rotations[thread_num] = (threads[thread_num]->start_rot()) * rot_offset[thread_num].transpose();
	for (thread_num=1; thread_num<threads_size; thread_num++) {
		threads[thread_num]->get_thread_data(points[thread_num]);
		positions[thread_num] = (points[thread_num]).front();
		rotations[thread_num] = (threads[thread_num]->start_rot()) * rot_offset[thread_num].transpose();
	}
	positions[thread_num] = (points[thread_num-1]).back();
	rotations[thread_num] = (threads[thread_num-1])->end_rot() * rot_offset[thread_num].transpose();
}

void ThreadConstrained::setConstrainedTransforms(vector<Vector3d> positions, vector<Matrix3d> rotations) {
	int threads_size = threads.size();
	if (positions.size()!=(threads_size+1) || rotations.size()!=(threads_size+1))
    cout << "Internal error: getConstrainedTransforms: at least one of the vector parameters is of the wrong size" << endl;
	vector<Matrix3d> material_frames;
	get_thread_data(positions, material_frames);
	for (int i=0; i<constrained_vertices_nums.size(); i++) {
		if (constrained_vertices_nums[i]==0) {
			rot_offset[i] = rotations[i].transpose() * start_rot();
			rot_offset[i] = (Matrix3d) AngleAxisd(rot_offset[i]); // to fix numerical errors and rot_offset stays orthonormal
		} else if (constrained_vertices_nums[i]==num_vertices-1) {
			rot_offset[i] = rotations[i].transpose() * end_rot();
			rot_offset[i] = (Matrix3d) AngleAxisd(rot_offset[i]); // to fix numerical errors and rot_offset stays orthonormal
		} else {
			Matrix3d inter_rot;
			int vertex_num = constrained_vertices_nums[i];
			intermediateRotation(inter_rot, material_frames[vertex_num-1], material_frames[vertex_num]);
			rot_offset[i] = rotations[i].transpose() * material_frames[vertex_num];
			rot_offset[i] = (Matrix3d) AngleAxisd(rot_offset[i]); // to fix numerical errors and rot_offset stays orthonormal
		}
	}
}

void ThreadConstrained::updateRotationOffset(int constraint_ind, Matrix3d rotation) {
	assert(!(constraint_ind < 0 || constraint_ind >= threads.size()+1)); // constraint_ind is out of bounds
	vector<Vector3d> positions;
	vector<Matrix3d> material_frames;
	get_thread_data(positions, material_frames);
	if (constrained_vertices_nums[constraint_ind]==0) {
		rot_offset[constraint_ind] = rotation.transpose() * start_rot();
		rot_offset[constraint_ind] = (Matrix3d) AngleAxisd(rot_offset[constraint_ind]); // to fix numerical errors and rot_offset stays orthonormal
	} else if (constrained_vertices_nums[constraint_ind]==num_vertices-1) {
		rot_offset[constraint_ind] = rotation.transpose() * end_rot();
		rot_offset[constraint_ind] = (Matrix3d) AngleAxisd(rot_offset[constraint_ind]); // to fix numerical errors and rot_offset stays orthonormal
	} else {
		Matrix3d inter_rot;
		int vertex_num = constrained_vertices_nums[constraint_ind];
		intermediateRotation(inter_rot, material_frames[vertex_num-1], material_frames[vertex_num]);
		rot_offset[constraint_ind] = rotation.transpose() * material_frames[vertex_num];
		rot_offset[constraint_ind] = (Matrix3d) AngleAxisd(rot_offset[constraint_ind]); // to fix numerical errors and rot_offset stays orthonormal
	}
}

//void ThreadConstrained::getAllTransforms(vector<Vector3d> &positions, vector<Matrix3d> &rotations) {
//	vector<Matrix3d> material_frames;
//	get_thread_data(positions, material_frames);
//	rotations[0] = start_rot() * rot_offset[0].transpose();
//	for(int vertex_num=1; vertex_num<num_vertices-1; vertex_num++) {
//		rotations[vertex_num] = material_frames[vertex_num] * rot_offset[vertex_num].transpose();
//	}
//	rotations[num_vertices-1] = end_rot() * rot_offset[num_vertices-1].transpose();
//}

//void ThreadConstrained::setAllTransforms(vector<Vector3d> positions, vector<Matrix3d> rotations) {
//	vector<Matrix3d> material_frames;
//	get_thread_data(positions, material_frames);
//	rot_offset[0] = rotations[0].transpose() * start_rot();
//	rot_offset[0] = (Matrix3d) AngleAxisd(rot_offset[0]); // to fix numerical errors and rot_offset stays orthonormal
//	for(int vertex_num=1; vertex_num<num_vertices-1; vertex_num++) {
//		Matrix3d inter_rot;
//		intermediateRotation(inter_rot, material_frames[vertex_num-1], material_frames[vertex_num]);
//		rot_offset[vertex_num] = rotations[vertex_num].transpose() * material_frames[vertex_num];
//		rot_offset[vertex_num] = (Matrix3d) AngleAxisd(rot_offset[vertex_num]); // to fix numerical errors and rot_offset stays orthonormal
//	}
//	rot_offset[num_vertices-1] = rotations[num_vertices-1].transpose() * end_rot();
//	rot_offset[num_vertices-1] = (Matrix3d) AngleAxisd(rot_offset[num_vertices-1]); // to fix numerical errors and rot_offset stays orthonormal
//}

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
	assert(poss.size() == rots.size()); // different number of positions and rotations
	assert(poss.size() == (threads.size()+1)); //	not the right number of transforms for current threads. addConstraint or removeConstraint first.
	
	//the following loop is to synchronize the end positions of two threads held by the same end-effector
//	bool wrong_positions = true;
//	int iters = 0;
//	while (wrong_positions) {
//		wrong_positions = false;
//		iters++;
//		for (int i=0; i<threads.size(); i++) {
//			Matrix3d new_start_rot = rots[i]*rot_offset[i];
//			Matrix3d new_end_rot 	 = rots[i+1]*rot_offset[i+1]*rot_diff[i+1];
//			wrong_positions = wrong_positions || threads[i]->check_fix_positions(poss[i], new_start_rot, poss[i+1], new_end_rot);
//		}
//		if (iters == 10) { break; }
//	}
//	
//	for (int i=0; i<threads.size(); i++) {
//		Vector3d new_start_pos;
//		Vector3d new_end_pos;
//		Matrix3d new_start_rot;
//		Matrix3d new_end_rot;
//			
//		new_start_pos = poss[i];
//		new_end_pos 	= poss[i+1];
//		new_start_rot = rots[i]*rot_offset[i];
//		new_end_rot 	= rots[i+1]*rot_offset[i+1]*rot_diff[i+1];
//				
//		double angle_change = angle_mismatch(new_start_rot, threads[i]->start_rot());
//		if (i==0)
//			zero_angle += angle_change - M_PI*((int) (angle_change/M_PI));
//	  threads[i]->set_constraints_check(new_start_pos, new_start_rot, new_end_pos, new_end_rot);
//	}
	
	for (int constraint_ind = 0; constraint_ind < constrained_vertices_nums.size(); constraint_ind++) {
		updateConstrainedTransform(constraint_ind, poss[constraint_ind], rots[constraint_ind]);
	}
	minimize_energy();
}

// doesn't minimize energy
void ThreadConstrained::updateConstrainedTransform(int constraint_ind, const Vector3d& pos, const Matrix3d& rot)
{
	Vector3d position(pos);
	Matrix3d rotation(rot);
	bool wrong_positions = true;
	int iters = 0;
	while (wrong_positions) {
		wrong_positions = false;
		iters++;
		for (int i=0; i<threads.size(); i++) {
			if (constrained_vertices_nums[constraint_ind] != 0) {
				Vector3d temp_start_pos = threads[constraint_ind-1]->start_pos();
				Matrix3d temp_start_rot = threads[constraint_ind-1]->start_rot();
				wrong_positions = wrong_positions || threads[constraint_ind-1]->check_fix_positions(temp_start_pos, temp_start_rot, position, rotation);
			}
			
			if (constrained_vertices_nums[constraint_ind] != num_vertices-1) {
				Vector3d temp_end_pos = threads[constraint_ind]->end_pos();
				Matrix3d temp_end_rot = threads[constraint_ind]->end_rot();
				wrong_positions = wrong_positions || threads[constraint_ind]->check_fix_positions(position, rotation, temp_end_pos, temp_end_rot);
			}
		}
		if (iters == 10) { break; }
	}
	
	if (constrained_vertices_nums[constraint_ind] != 0)
		threads[constraint_ind-1]->set_end_constraint(position, rotation*rot_offset[constraint_ind]*rot_diff[constraint_ind]);
	if (constrained_vertices_nums[constraint_ind] != num_vertices-1)
		threads[constraint_ind]->set_start_constraint(position, rotation*rot_offset[constraint_ind]);
}

void ThreadConstrained::getConstrainedTransform(int constraint_ind, Vector3d& pos, Matrix3d& rot)
{
	pos = positionAtConstraint(constraint_ind);
	rot = rotationAtConstraint(constraint_ind);
}

//void ThreadConstrained::getConstrainedTransforms(vector<Vector3d> &positions, vector<Matrix3d> &rotations) {
//	int threads_size = threads.size();
//	positions.resize(threads_size+1);
//	rotations.resize(threads_size+1);
//	vector<vector<Vector3d> > points(threads_size);	
//	int thread_num = 0;
//	threads[thread_num]->get_thread_data(points[thread_num]);
//	positions[thread_num] = (points[thread_num]).front();
//	rotations[thread_num] = (threads[thread_num]->start_rot()) * rot_offset[thread_num].transpose();
//	for (thread_num=1; thread_num<threads_size; thread_num++) {
//		threads[thread_num]->get_thread_data(points[thread_num]);
//		positions[thread_num] = (points[thread_num]).front();
//		rotations[thread_num] = (threads[thread_num]->start_rot()) * rot_offset[thread_num].transpose();
//	}
//	positions[thread_num] = (points[thread_num-1]).back();
//	rotations[thread_num] = (threads[thread_num-1])->end_rot() * rot_offset[thread_num].transpose();
//}


const Vector3d& ThreadConstrained::positionAtConstraint(int constraint_ind) const
{
	if (constrained_vertices_nums[constraint_ind] != num_vertices-1)
		return threads[constraint_ind]->start_pos();
	else
		return threads[constraint_ind-1]->end_pos();
}

Matrix3d ThreadConstrained::rotationAtConstraint(int constraint_ind)
{
	#ifndef NDEBUG
	vector<Vector3d> positions;
	vector<Matrix3d> rotations;
	getConstrainedTransforms(positions, rotations);
	
	if (constrained_vertices_nums[constraint_ind] != num_vertices-1) {
		Matrix3d result = threads[constraint_ind]->start_rot() * rot_offset[constraint_ind].transpose();
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				assert ((result(i,j) - rotations[constraint_ind](i,j)) < 0.00001);
			}
		}
		return result;
	} else {
		Matrix3d result = threads[constraint_ind-1]->end_rot() * rot_offset[constraint_ind].transpose();
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				assert ((result(i,j) - rotations[constraint_ind](i,j)) < 0.00001);
			}
		}
		return result;
	}
	#else
	if (constrained_vertices_nums[constraint_ind] != num_vertices-1)
		return threads[constraint_ind]->start_rot() * rot_offset[constraint_ind].transpose();
	else
		return threads[constraint_ind-1]->end_rot() * rot_offset[constraint_ind].transpose();
	#endif
}

void ThreadConstrained::applyMotionAtConstraints(vector<Vector3d> translations, vector<Matrix3d> rotations)
{
	vector<Vector3d> new_positions(constrained_vertices_nums.size());
	vector<Matrix3d> new_rotations(constrained_vertices_nums.size());
	getConstrainedTransforms(new_positions, new_rotations);
	for (int i = 0; i < constrained_vertices_nums.size(); i++) {
		new_positions[i] += translations[i];
		new_rotations[i] = new_rotations[i] * rotations[i];
	}
	updateConstraints(new_positions, new_rotations);
}

void ThreadConstrained::applyControl(const VectorXd& u)
{
	if ((u.size() % 3) != 0)
		cout << "Internal error: ThreadConstrained::applyControl(): control u is not a multiple of 3." << endl;
	if ((u.size()/3) != constrained_vertices_nums.size())
		cout << "Internal error: ThreadConstrained::applyControl(): control doesn't match the number of thread constraints." << endl;
  
  double max_ang = 0.0;
  for (int i = 0; i < constrained_vertices_nums.size(); i++) {
  	double constraint_max_ang = max( max(abs(u(3*i+3)), abs(u(3*i+4))), abs(u(3*i+5)));
  	if (constraint_max_ang > max_ang) max_ang = constraint_max_ang;
	}
	
  int number_steps = max ((int)ceil(max_ang / (M_PI/4.0)), 1);
  VectorXd u_for_translation = u/((double)number_steps);

	vector<Vector3d> translations;
	vector<Matrix3d> rotations;
	for (int i = 0; i < constrained_vertices_nums.size(); i++) {	
		translations.push_back(Vector3d(u_for_translation(3*i+0), u_for_translation(3*i+1), u_for_translation(3*i+2)));

		Matrix3d rotation;
		rotation_from_euler_angles(rotation, u(3*i+3), u(3*i+4), u(3*i+5));
		Quaterniond quat_rotation(rotation);
		rotations.push_back((Matrix3d) Quaterniond::Identity().slerp(1.0/(double)number_steps, quat_rotation));
	}

	for (int j=0; j < number_steps; j++)
	{
		applyMotionAtConstraints(translations, rotations);
	}
}

void ThreadConstrained::getState(VectorXd& state) {
	vector<VectorXd> thread_states;
	int thread_states_size = 0;
	for (int thread_ind = 0; thread_ind < threads.size(); thread_ind++) {
		VectorXd thread_state;
		threads[thread_ind]->getState(thread_state);
		thread_states.push_back(thread_state);
		thread_states_size += thread_state.size();
	}
	state.resize(thread_states_size);
	int vector_start = 0;
	for (int i = 0; i < thread_states.size(); i++) {
		state.segment(vector_start, thread_states[i].size()) = thread_states[i];
		vector_start += thread_states[i].size();
	}
}

// returns the constraint_ind of the new added constraint.
int ThreadConstrained::addConstraint (int absolute_vertex_num) {
	int thread_num = insertSorted(constrained_vertices_nums, absolute_vertex_num)-1;
	splitThread(thread_num, localVertex(absolute_vertex_num));
	return thread_num+1;
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
	assert(0);
	return vertex_start_rot * rot_offset[absolute_vertex_num].transpose();
}

void ThreadConstrained::draw(bool mode) {
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

	if (mode != examine_mode) {
  	examine_mode = mode;
  	initContour();
	}

  gleTwistExtrusion(20,
      contour,
      contour_norms,
      NULL,
      points.size()+2,
      pts_cpy,
      0x0,
      twist_cpy);
      
  if (examine_mode) {
		glColor3f (0.0, 0.5, 0.5);
		for (int i=0; i<points.size(); i++)
			drawSphere(points[i], 0.7);
		drawAxes(positionAtConstraint(0), rotationAtConstraint(0));
		drawAxes(positionAtConstraint(constrained_vertices_nums.size()-1), rotationAtConstraint(constrained_vertices_nums.size()-1));
	}
			
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

void ThreadConstrained::setWorld(World* w) {
	world = w;
	for(int i=0; i<threads.size(); i++)
		threads[i]->setWorld(world);
}

void ThreadConstrained::getThreads(vector<Thread*>& ths) {
	ths.clear();
	for (int i = 0; i < threads.size(); i++)
		ths.push_back(threads[i]);
}

void ThreadConstrained::intermediateRotation(Matrix3d &inter_rot, Matrix3d end_rot, Matrix3d start_rot) {
	Quaterniond start_q(start_rot);
  Quaterniond end_q(end_rot);
  Quaterniond interp_q = start_q.slerp(0.5, end_q);
  inter_rot = interp_q.toRotationMatrix();
}

// Splits the thread threads[thread_num] into two threads, which are stored at threads[thread_num] and threads[thread_num+1].  Threads in threads that are stored after thread_num now have a new thread_num which is one unit more than before. The split is done at vertex vertex of thread[thread_num]
void ThreadConstrained::splitThread(int thread_num, int vertex_num) {
	checkNumVertices();
	
	vector<Vector3d> point0;
	vector<Vector3d> point1;
	vector<Vector3d> points;
	vector<double> twist_angle0;
	vector<double> twist_angle1;
	vector<double> twist_angles;
	vector<double> length0;
	vector<double> length1;
	vector<double> lengths;
	threads[thread_num]->get_thread_data(points, twist_angles, lengths);

	splitVector(point0, point1, points, vertex_num);
	splitVector(twist_angle0, twist_angle1, twist_angles, vertex_num);
	splitVector(length0, length1, lengths, vertex_num);

  Matrix3d vertex_end_rot = threads[thread_num]->material_at_ind(vertex_num-1);
  Matrix3d vertex_start_rot = threads[thread_num]->material_at_ind(vertex_num);
  Matrix3d rot_diff_matrix = vertex_start_rot.transpose()*vertex_end_rot;
  rot_diff.insert(rot_diff.begin()+thread_num+1, rot_diff_matrix);
  rot_offset.insert(rot_offset.begin()+thread_num+1, (Matrix3d) (AngleAxisd(M_PI/2.0, Vector3d::UnitZ())));

	mapAdd(twist_angle1, -twist_angle1.front());
	twist_angle0.back() = 2.0*twist_angle0[twist_angle0.size()-2] - twist_angle0[twist_angle0.size()-3];
	twist_angle1.back() = 2.0*twist_angle1[twist_angle1.size()-2] - twist_angle1[twist_angle1.size()-3];
	
	double first_length = FIRST_REST_LENGTH;
	double second_length = SECOND_REST_LENGTH;
	
	if (length0[length0.size()-2] > (first_length + second_length)) {
		length0.push_back(0.0);
		length0.push_back(0.0);
		length0[length0.size()-1] = length0[length0.size()-4];
		length0[length0.size()-2] = first_length;
		length0[length0.size()-3] = second_length;
		length0[length0.size()-4] = length0[length0.size()-4] - first_length - second_length;
	
		Vector3d sl = point0[point0.size()-2];
		Vector3d l_sl = (sl - point0.back()).normalized();
		point0.push_back(Vector3d::Zero());
		point0.push_back(Vector3d::Zero());
		point0[point0.size()-1] = point0[point0.size()-3];
		point0[point0.size()-2] = point0[point0.size()-1] + (first_length) * l_sl;
		point0[point0.size()-3] = point0[point0.size()-2] + (second_length) * l_sl;
		
		num_vertices += 2;
		for (int i = thread_num+1; i < constrained_vertices_nums.size(); i++) {
			constrained_vertices_nums[i] += 2;
		}
	} else if (length0[length0.size()-2] > first_length) {
		length0.push_back(0.0);
		length0[length0.size()-1] = length0[length0.size()-3];
		length0[length0.size()-2] = first_length;
		length0[length0.size()-3] = length0[length0.size()-3] - first_length;
	
		Vector3d sl = point0[point0.size()-2];
		Vector3d l_sl = (sl - point0.back()).normalized();
		point0.push_back(Vector3d::Zero());
		point0[point0.size()-1] = point0[point0.size()-2];
		point0[point0.size()-2] = point0[point0.size()-1] + (first_length) * l_sl;
		
		num_vertices++;
		for (int i = thread_num+1; i < constrained_vertices_nums.size(); i++) {
			constrained_vertices_nums[i] += 1;
		}
	}
		
	if (length1[0] > (first_length + second_length)) {
		length1.push_back(0.0);
		length1.push_back(0.0);
		for (int i = length1.size()-1; i >= 2; i--) {
			length1[i] = length1[i-2];
		}
		length1[0] = first_length;
		length1[1] = second_length;
		length1[2] = length1[2] - first_length - second_length;
	
		Vector3d s = point1[1];
		Vector3d f_s = (s - point1.front()).normalized();
		point1.push_back(Vector3d::Zero());
		point1.push_back(Vector3d::Zero());
		for (int i = point1.size()-1; i >= 2; i--) {
			point1[i] = point1[i-2];
		}
		point1[0] = point1[2];
		point1[1] = point1[0] + (first_length) * f_s;
		point1[2] = point1[1] + (second_length) * f_s;
		
		num_vertices += 2;
		for (int i = thread_num+2; i < constrained_vertices_nums.size(); i++) {
			constrained_vertices_nums[i] += 2;
		}
	} else if (length1[0] > first_length) {
		length1.push_back(0.0);
		for (int i = length1.size()-1; i >= 1; i--) {
			length1[i] = length1[i-1];
		}
		length1[0] = first_length;
		length1[1] = length1[1] - first_length;
	
		Vector3d s = point1[1];
		Vector3d f_s = (s - point1.front()).normalized();
		point1.push_back(Vector3d::Zero());
		for (int i = point1.size()-1; i >= 1; i--) {
			point1[i] = point1[i-1];
		}
		point1[0] = point1[1];
		point1[1] = point1[0] + (first_length) * f_s;
		
		num_vertices++;
		for (int i = thread_num+2; i < constrained_vertices_nums.size(); i++) {
			constrained_vertices_nums[i] += 1;
		}
	}
	
	Thread* thread0 = new Thread(point0, twist_angle0, length0, (Matrix3d&) (threads[thread_num])->start_rot(), vertex_end_rot);
	Thread* thread1 = new Thread(point1, twist_angle1, length1, vertex_start_rot, (Matrix3d&) (threads[thread_num])->end_rot());
	thread0->setWorld(world);
	thread1->setWorld(world);
	delete threads[thread_num];
	threads[thread_num] = thread0;
	threads.insert(threads.begin()+thread_num+1, thread1);
	
	if (world!=NULL)
		world->initializeThreadsInEnvironment();
	
	threads[thread_num]->minimize_energy();
	threads[thread_num+1]->minimize_energy();
	threads[thread_num]->get_thread_data(point0, twist_angle0);
	threads[thread_num+1]->get_thread_data(point1, twist_angle1);
	
	checkNumVertices();
}

// Merges the threads threads[thread_num] and threads[thread_num+1] into one thread, which is stored at threads[thread_num]. Threads in threads that are stored after thread_num+1 now have a new thread_num which is one unit less than before.
void ThreadConstrained::mergeThread(int thread_num) {
	vector<Vector3d> points0;
	vector<Vector3d> points1;
	vector<Vector3d> point;
	vector<double> twist_angles0;
	vector<double> twist_angles1;
	vector<double> lengths0;
	vector<double> lengths1;
	vector<double> length;
	threads[thread_num]->get_thread_data(points0, twist_angles0, lengths0);
	threads[thread_num+1]->get_thread_data(points1, twist_angles1, lengths1);
	vector<double> twist_angle(twist_angles0.size()+twist_angles1.size()-1);

	mergeVector(point, points0, points1);
	mergeVector(twist_angle, twist_angles0, twist_angles1);
	mergeVector(length, lengths0, lengths1);
	
	rot_diff.erase(rot_diff.begin() + thread_num+1);
	rot_offset.erase(rot_offset.begin() + thread_num+1);

	Thread* thread = new Thread(point, twist_angle, length, (Matrix3d&) (threads[thread_num])->start_rot(), (Matrix3d&) (threads[thread_num+1])->end_rot());
	thread->setWorld(world);
	delete threads[thread_num];
	delete threads[thread_num+1];
	threads[thread_num] = thread;
	threads.erase(threads.begin()+thread_num+1);
	
	if (world!=NULL)
		world->initializeThreadsInEnvironment();
	
	threads[thread_num]->minimize_energy();
}

// Returns the thread number that owns the absolute_vertex_num. Example:
// constrained absolute_vertex_num:			0               8               14                            24
// absolute_vertex_num: 								0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24
// threadOwner(absolute_vertex_num):		0 0 0 0 0 0 0 0 0 1  1  1  1  1  1  2  2  2  2  2  2  2  2  2  2
int ThreadConstrained::threadOwner(int absolute_vertex_num)
{
	int thread_num;
	int num_absolute_vertices = 1;
	vector<vector<Vector3d> > points(threads.size());
	for (thread_num=0; thread_num<threads.size(); thread_num++) {
		threads[thread_num]->get_thread_data(points[thread_num]);
		num_absolute_vertices += (points[thread_num]).size() - 1;
		if (absolute_vertex_num < num_absolute_vertices) { break; }
	}
	assert(thread_num!=threads.size()); //absolute_vertex_num is greater than total number of vertices
	return thread_num;
}

// Returns the local vertex number (i.e. vertex number within a thread), given the absolute vertex number (i.e. vertex number within all vertices). Example:
// constrained absolute_vertex_num:			0               8               14                            24
// absolute_vertex_num: 								0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24
// localVertex(absolute_vertex_num):		0 1 2 3 4 5 6 7 8 1  2  3  4  5  6  1  2  3  4  5  6  7  8  9 10
int ThreadConstrained::localVertex(int absolute_vertex_num)
{
	int thread_num;
	int num_absolute_vertices = 1;
	vector<vector<Vector3d> > points(threads.size());
	for (thread_num=0; thread_num<threads.size(); thread_num++) {
		threads[thread_num]->get_thread_data(points[thread_num]);
		num_absolute_vertices += (points[thread_num]).size() - 1;
		if (absolute_vertex_num < num_absolute_vertices) { break; }
	}
	assert(thread_num!=threads.size()); // absolute_vertex_num is greater than total number of vertices
	return absolute_vertex_num + (points[thread_num]).size() - num_absolute_vertices;
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
