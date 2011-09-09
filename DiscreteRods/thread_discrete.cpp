#include "thread_discrete.h"

Thread::Thread()
{
  world = NULL;
  _thread_pieces.resize(0);
	_thread_pieces_backup.resize(0);
}


Thread::Thread(const VectorXd& vertices, const VectorXd& twists, const Matrix3d& start_rot)
{
  world = NULL;
  _thread_pieces.resize(twists.size());
  _thread_pieces_backup.resize(twists.size());
  _angle_twist_backup.resize(twists.size());
  for (int i=0; i < twists.size(); i++)
  {
    _thread_pieces[i] = new ThreadPiece(vertices.segment<3>(3*i), twists(i), DEFAULT_REST_LENGTH, this);
  }

  for (int i=1; i < twists.size(); i++)
  {
    _thread_pieces[i]->set_prev(_thread_pieces[i-1]);
  }
	_thread_pieces[0]->set_prev(NULL);
	for (int i=0; i < twists.size()-1; i++)
  {
    _thread_pieces[i]->set_next(_thread_pieces[i+1]);
  }
  _thread_pieces[twists.size()-1]->set_next(NULL);
	
	_total_length = 0;
	for (int piece_ind = 0; piece_ind < _thread_pieces.size()-1; piece_ind++)
	{
		_total_length += _thread_pieces[piece_ind]->rest_length();
	}
	
  //setup backups
  for (int i=0; i < twists.size(); i++)
  {
  	_thread_pieces_backup[i] = new ThreadPiece(vertices.segment<3>(3*i), twists(i), DEFAULT_REST_LENGTH, this);
  }

  for (int i=1; i < twists.size(); i++)
  {
  	_thread_pieces_backup[i]->set_prev(_thread_pieces_backup[i-1]);
  }
	_thread_pieces_backup[0]->set_prev(NULL);
  for (int i=0; i < twists.size()-1; i++)
  {
  	_thread_pieces_backup[i]->set_next(_thread_pieces_backup[i+1]);
  }
  _thread_pieces_backup[twists.size()-1]->set_next(NULL);


  _thread_pieces.front()->set_bishop_frame(start_rot);
  _thread_pieces.front()->set_material_frame(start_rot);

  _thread_pieces.front()->initializeFrames();

  set_start_constraint(vertices.segment<3>(0), start_rot);

  Matrix3d end_bishop = _thread_pieces[_thread_pieces.size()-2]->bishop_frame();
  Matrix3d end_rot = Eigen::AngleAxisd(twists[twists.size()-2], end_bishop.col(0).normalized())*end_bishop;

  set_end_constraint(vertices.segment<3>(vertices.size()-3), end_rot);
}

//Create a Thread. start_rot is the first bishop frame. the last material frame is calculated from twist_angles
Thread::Thread(vector<Vector3d>& vertices, vector<double>& twist_angles, Matrix3d& start_rot)
{
  world = NULL;
  _thread_pieces.resize(vertices.size());
  _thread_pieces_backup.resize(vertices.size());
  _angle_twist_backup.resize(vertices.size());
  for (int i=0; i < vertices.size(); i++)
  {
    _thread_pieces[i] = new ThreadPiece(vertices[i], twist_angles[i], DEFAULT_REST_LENGTH, this);
//    _thread_pieces.push_back(ThreadPiece(vertices[i], twist_angles[i]));
  }

  for (int i=1; i < vertices.size(); i++)
  {
    _thread_pieces[i]->set_prev(_thread_pieces[i-1]);
  }
	_thread_pieces[0]->set_prev(NULL);
  for (int i=0; i < vertices.size()-1; i++)
  {
    _thread_pieces[i]->set_next(_thread_pieces[i+1]);
  }
  _thread_pieces[vertices.size()-1]->set_next(NULL);

	_total_length = 0;
	for (int piece_ind = 0; piece_ind < _thread_pieces.size()-1; piece_ind++)
	{
		_total_length += _thread_pieces[piece_ind]->rest_length();
	}
	
	//setup backups
  for (int i=0; i < vertices.size(); i++)
  {
    _thread_pieces_backup[i] = new ThreadPiece(vertices[i], twist_angles[i], DEFAULT_REST_LENGTH, this);
  }

  for (int i=1; i < vertices.size(); i++)
  {
    _thread_pieces_backup[i]->set_prev(_thread_pieces_backup[i-1]);
  }
	_thread_pieces_backup[0]->set_prev(NULL);
  for (int i=0; i < vertices.size()-1; i++)
  {
    _thread_pieces_backup[i]->set_next(_thread_pieces_backup[i+1]);
  }
  _thread_pieces_backup[vertices.size()-1]->set_next(NULL);


  _thread_pieces.front()->set_bishop_frame(start_rot);
  _thread_pieces.front()->set_material_frame(start_rot);

  _thread_pieces.front()->initializeFrames();


 // _saved_last_thetas.resize(_thread_pieces.size());
  //_saved_last_theta_changes.resize(_thread_pieces.size());


  set_start_constraint(vertices.front(), start_rot);

  Matrix3d end_bishop = _thread_pieces[_thread_pieces.size()-2]->bishop_frame();
  Matrix3d end_rot = Eigen::AngleAxisd(twist_angles[twist_angles.size()-2], end_bishop.col(0).normalized())*end_bishop;

  set_end_constraint(vertices.back(), end_rot);

}

//As above, with a specific rest length
Thread::Thread(vector<Vector3d>& vertices, vector<double>& twist_angles, Matrix3d& start_rot, const double rest_length)
{
  world = NULL;
  _thread_pieces.resize(vertices.size());
  _thread_pieces_backup.resize(vertices.size());
  _angle_twist_backup.resize(vertices.size());
  for (int i=0; i < vertices.size(); i++)
  {
    _thread_pieces[i] = new ThreadPiece(vertices[i], twist_angles[i], rest_length, this);
//    _thread_pieces.push_back(ThreadPiece(vertices[i], twist_angles[i]));
  }

  for (int i=1; i < vertices.size(); i++)
  {
    _thread_pieces[i]->set_prev(_thread_pieces[i-1]);
  }
	_thread_pieces[0]->set_prev(NULL);
  for (int i=0; i < vertices.size()-1; i++)
  {
    _thread_pieces[i]->set_next(_thread_pieces[i+1]);
  }
  _thread_pieces[vertices.size()-1]->set_next(NULL);
  
  _total_length = 0;
	for (int piece_ind = 0; piece_ind < _thread_pieces.size()-1; piece_ind++)
	{
		_total_length += _thread_pieces[piece_ind]->rest_length();
	}

	//setup backups
  for (int i=0; i < vertices.size(); i++)
  {
    _thread_pieces_backup[i] = new ThreadPiece(vertices[i], twist_angles[i], rest_length, this);
  }

  for (int i=1; i < vertices.size(); i++)
  {
    _thread_pieces_backup[i]->set_prev(_thread_pieces_backup[i-1]);
  }
	_thread_pieces_backup[0]->set_prev(NULL);
  for (int i=0; i < vertices.size()-1; i++)
  {
    _thread_pieces_backup[i]->set_next(_thread_pieces_backup[i+1]);
  }
	_thread_pieces_backup[vertices.size()-1]->set_next(NULL);

  _thread_pieces.front()->set_bishop_frame(start_rot);
  _thread_pieces.front()->set_material_frame(start_rot);

  _thread_pieces.front()->initializeFrames();


 // _saved_last_thetas.resize(_thread_pieces.size());
  //_saved_last_theta_changes.resize(_thread_pieces.size());


  set_start_constraint(vertices.front(), start_rot);

  Matrix3d end_bishop = _thread_pieces[_thread_pieces.size()-2]->bishop_frame();
  Matrix3d end_rot = Eigen::AngleAxisd(twist_angles[twist_angles.size()-2], end_bishop.col(0).normalized())*end_bishop;

  set_end_constraint(vertices.back(), end_rot);
}


//As above, with a specific rest length for each threadpiece.
Thread::Thread(vector<Vector3d>& vertices, vector<double>& twist_angles, vector<double>& rest_lengths, Matrix3d& start_rot)
{
  world = NULL;
  _thread_pieces.resize(vertices.size());
  _thread_pieces_backup.resize(vertices.size());
  _angle_twist_backup.resize(vertices.size());
  for (int i=0; i < vertices.size(); i++)
  {
    _thread_pieces[i] = new ThreadPiece(vertices[i], twist_angles[i], rest_lengths[i], this);
//    _thread_pieces.push_back(ThreadPiece(vertices[i], twist_angles[i]));
  }
	
  for (int i=1; i < vertices.size(); i++)
  {
    _thread_pieces[i]->set_prev(_thread_pieces[i-1]);
  }
	_thread_pieces[0]->set_prev(NULL);
  for (int i=0; i < vertices.size()-1; i++)
  {
    _thread_pieces[i]->set_next(_thread_pieces[i+1]);
  }
  _thread_pieces[vertices.size()-1]->set_next(NULL);
  
  _total_length = 0;
	for (int piece_ind = 0; piece_ind < _thread_pieces.size()-1; piece_ind++)
	{
		_total_length += _thread_pieces[piece_ind]->rest_length();
	}

	//setup backups
  for (int i=0; i < vertices.size(); i++)
  {
    _thread_pieces_backup[i] = new ThreadPiece(vertices[i], twist_angles[i], rest_lengths[i], this);
  }

  for (int i=1; i < vertices.size(); i++)
  {
    _thread_pieces_backup[i]->set_prev(_thread_pieces_backup[i-1]);
  }
	_thread_pieces_backup[vertices.size()-1]->set_next(NULL);
  for (int i=0; i < vertices.size()-1; i++)
  {
    _thread_pieces_backup[i]->set_next(_thread_pieces_backup[i+1]);
  }
	_thread_pieces_backup[vertices.size()-1]->set_next(NULL);

  _thread_pieces.front()->set_bishop_frame(start_rot);
  _thread_pieces.front()->set_material_frame(start_rot);

  _thread_pieces.front()->initializeFrames();


 // _saved_last_thetas.resize(_thread_pieces.size());
  //_saved_last_theta_changes.resize(_thread_pieces.size());


  set_start_constraint(vertices.front(), start_rot);

  Matrix3d end_bishop = _thread_pieces[_thread_pieces.size()-2]->bishop_frame();
  Matrix3d end_rot = Eigen::AngleAxisd(twist_angles[twist_angles.size()-2], end_bishop.col(0).normalized())*end_bishop;

  set_end_constraint(vertices.back(), end_rot);
}

//Create a Thread. start_rot is the first bishop frame. end_rot is used to calculate the last twist angle
Thread::Thread(vector<Vector3d>& vertices, vector<double>& twist_angles, Matrix3d& start_rot, Matrix3d& end_rot)
{
  world = NULL;
  //_thread_pieces.resize(vertices.size());
  _thread_pieces_backup.resize(vertices.size());
  _angle_twist_backup.resize(vertices.size());
  _thread_pieces.resize(vertices.size());
  for (int i=0; i < vertices.size(); i++)
  {
    _thread_pieces[i] = new ThreadPiece(vertices[i], twist_angles[i], DEFAULT_REST_LENGTH, this);
  }

  for (int i=1; i < vertices.size(); i++)
  {
    _thread_pieces[i]->set_prev(_thread_pieces[i-1]);
  }
	_thread_pieces[0]->set_prev(NULL);
  for (int i=0; i < vertices.size()-1; i++)
  {
    _thread_pieces[i]->set_next(_thread_pieces[i+1]);
  }
  _thread_pieces[vertices.size()-1]->set_next(NULL);
  
  _total_length = 0;
	for (int piece_ind = 0; piece_ind < _thread_pieces.size()-1; piece_ind++)
	{
		_total_length += _thread_pieces[piece_ind]->rest_length();
	}

	//setup backups
  for (int i=0; i < vertices.size(); i++)
  {
    _thread_pieces_backup[i] = new ThreadPiece(vertices[i], twist_angles[i], DEFAULT_REST_LENGTH, this);
    //_thread_pieces.push_back(ThreadPiece(vertices[i], twist_angles[i]));
  }

  for (int i=1; i < vertices.size(); i++)
  {
    _thread_pieces_backup[i]->set_prev(_thread_pieces_backup[i-1]);
  }
	_thread_pieces_backup[0]->set_prev(NULL);
  for (int i=0; i < vertices.size()-1; i++)
  {
    _thread_pieces_backup[i]->set_next(_thread_pieces_backup[i+1]);
  }
  _thread_pieces_backup[vertices.size()-1]->set_next(NULL);

  _thread_pieces.front()->set_bishop_frame(start_rot);
  _thread_pieces.front()->set_material_frame(start_rot);

  _thread_pieces.front()->initializeFrames();

 // _saved_last_thetas.resize(_thread_pieces.size());
  //_saved_last_theta_changes.resize(_thread_pieces.size());

  set_constraints(vertices.front(), start_rot, vertices.back(), end_rot);

  //set_end_constraint(vertices.back(), end_rot);
  //project_length_constraint();



/*
  vector<Vector3d> vertices_actual;
  vector<Matrix3d> material_frames;

  get_thread_data(vertices_actual, material_frames);

  std::cout << "init data:\n";
  for (int i=0; i < _thread_pieces.size(); i++)
  {
    if (i < _thread_pieces.size()-2)
    _thread_pieces[i]->update_material_frame();
    std::cout << "vertex:\n" << vertices_actual[i] << "\nmaterial frame:\n" << material_frames[i] << std::endl;
  }


  Matrix3d last_bishop = _thread_pieces[_thread_pieces.size()-2]->bishop_frame();
  std::cout << "last bishop:\n" << last_bishop << std::endl;
  std::cout << "last material:\n" << _thread_pieces.back()->material_frame() << std::endl;
  std::cout << "last angle:\n" << _thread_pieces[_thread_pieces.size()-2]->angle_twist() << std::endl;


  Vector3d offset(2.0, 2.0, 0.0);
 // set_start_constraint(offset, start_rot);
  _thread_pieces[3]->offset_vertex(offset);

*/
  /*
  int numInit = 50;
  Vector3d positions[numInit];
  double angles[numInit];
  for (int i=0; i < numInit; i++)
  {
    positions[i](0) = 10*(int)((i)/2.0)%2;
    positions[i](1) = 10*floor((i+1)/2);
    positions[i](2) = i*10;
    angles[i] =0;
    _thread_pieces.push_back(ThreadPiece(positions[i], angles[i]));
    //std::cout << positions[i] << std::endl << std::endl;
  }


  Vector3d tan = (_thread_pieces[1]->vertex() - _thread_pieces[0]->vertex()).normalized();
  Vector3d toRotAxis;
  Matrix3d rot_for_frame;
  if ( (tan -Vector3d::UnitX()).norm() < 0.01 )
  {
    rot_for_frame = Matrix3d::Identity();
  }
  else
  {
    toRotAxis = Vector3d::UnitX().cross(tan);
    double Axisnorm = toRotAxis.norm();
    double toRotAng = asin(Axisnorm);

    toRotAxis /= Axisnorm;
   // std::cout << "axis: " << toRotAxis << std::endl;
   // std::cout << "ang: " << toRotAng << std::endl;

    rot_for_frame = (Eigen::AngleAxisd(toRotAng, toRotAxis));
  }
  //std::cout << "rot for frame: " << rot_for_frame << std::endl;

  _start_rot = rot_for_frame*Matrix3d::Identity();

  //std::cout << "start frame: " << _start_rot << std::endl;

  std::cout << "energy: " << calculate_energy() << std::endl;
*/

}


Thread::Thread(vector<Vector3d>& vertices, vector<double>& twist_angles, vector<double>& rest_lengths, Matrix3d& start_rot, Matrix3d& end_rot)
{
  world = NULL;
  _thread_pieces_backup.resize(vertices.size());
  _angle_twist_backup.resize(vertices.size());
  _thread_pieces.resize(vertices.size());
  for (int i=0; i < vertices.size(); i++)
  {
    _thread_pieces[i] = new ThreadPiece(vertices[i], twist_angles[i], rest_lengths[i], this);
  }

  for (int i=1; i < vertices.size(); i++)
  {
    _thread_pieces[i]->set_prev(_thread_pieces[i-1]);
  }
	_thread_pieces[0]->set_prev(NULL);
  for (int i=0; i < vertices.size()-1; i++)
  {
    _thread_pieces[i]->set_next(_thread_pieces[i+1]);
  }
  _thread_pieces[vertices.size()-1]->set_next(NULL);
  
  _total_length = 0;
	for (int piece_ind = 0; piece_ind < _thread_pieces.size()-1; piece_ind++)
	{
		_total_length += _thread_pieces[piece_ind]->rest_length();
	}

	//setup backups
  for (int i=0; i < vertices.size(); i++)
  {
    _thread_pieces_backup[i] = new ThreadPiece(vertices[i], twist_angles[i], rest_lengths[i], this);
  }

  for (int i=1; i < vertices.size(); i++)
  {
    _thread_pieces_backup[i]->set_prev(_thread_pieces_backup[i-1]);
  }
	_thread_pieces_backup[0]->set_prev(NULL);
  for (int i=0; i < vertices.size()-1; i++)
  {
    _thread_pieces_backup[i]->set_next(_thread_pieces_backup[i+1]);
  }
  _thread_pieces_backup[vertices.size()-1]->set_next(NULL);

  _thread_pieces.front()->set_bishop_frame(start_rot);
  _thread_pieces.front()->set_material_frame(start_rot);

  _thread_pieces.front()->initializeFrames();

  set_constraints(vertices.front(), start_rot, vertices.back(), end_rot);
}


Thread::Thread(const Thread& rhs)
{
  world = rhs.world;
  _thread_pieces.resize(rhs._thread_pieces.size());
  _thread_pieces_backup.resize(rhs._thread_pieces_backup.size());
  _angle_twist_backup.resize(rhs._thread_pieces.size());

  for (int piece_ind =0; piece_ind < rhs._thread_pieces.size(); piece_ind++)
  {
    //_thread_pieces.push_back(rhs._thread_pieces[piece_ind]);
    _thread_pieces[piece_ind] = new ThreadPiece(*rhs._thread_pieces[piece_ind], this);
  }

  for (int i=1; i < _thread_pieces.size(); i++)
  {
    _thread_pieces[i]->set_prev(_thread_pieces[i-1]);
  }
	_thread_pieces[0]->set_prev(NULL);
  for (int i=0; i < _thread_pieces.size()-1; i++)
  {
    _thread_pieces[i]->set_next(_thread_pieces[i+1]);
  }
  _thread_pieces[_thread_pieces.size()-1]->set_next(NULL);

	_total_length = rhs._total_length;
	
	//setup backups
  for (int i=0; i < rhs._thread_pieces_backup.size(); i++)
  {
    _thread_pieces_backup[i] = new ThreadPiece(*rhs._thread_pieces_backup[i], this);
    //_thread_pieces.push_back(ThreadPiece(vertices[i], twist_angles[i]));
  }

  for (int i=1; i < _thread_pieces_backup.size(); i++)
  {
    _thread_pieces_backup[i]->set_prev(_thread_pieces_backup[i-1]);
  }
	_thread_pieces_backup[0]->set_prev(NULL);
  for (int i=0; i < _thread_pieces_backup.size()-1; i++)
  {
    _thread_pieces_backup[i]->set_next(_thread_pieces_backup[i+1]);
  }
	_thread_pieces_backup[_thread_pieces_backup.size()-1]->set_next(NULL);


  _thread_pieces.front()->set_bishop_frame(rhs.start_rot());
  _thread_pieces.front()->set_material_frame(rhs.start_rot());

  _thread_pieces.front()->initializeFrames();

  //set_constraints(start_pos, start_rot, end_pos, end_rot);
  set_start_constraint(rhs.start_pos(), rhs.start_rot());

  if (this->num_pieces() > 4)
    set_end_constraint(rhs.end_pos(), rhs.end_rot());
  //project_length_constraint();
  
  set_all_pieces_mythread();
}

Thread::Thread(ifstream& file)
{
  int num_pts_each;
  file >> num_pts_each;

  vector<Vector3d> points;
  points.resize(num_pts_each);
  vector<double> twist_angles;
  twist_angles.resize(num_pts_each);
  vector<double> rest_lengths;
  rest_lengths.resize(num_pts_each);
  Matrix3d start_rot;
  Matrix3d end_rot;

  for (int r=0; r < 3; r++)
  {
    for (int c=0; c < 3; c++)
    {
      file >> start_rot (r,c);
    }
  }

  for (int r=0; r < 3; r++)
  {
    for (int c=0; c < 3; c++)
    {
      file >> end_rot (r,c);
    }
  }

  for (int i=0; i < points.size(); i++)
  {
    file >> points[i](0) >> points[i](1) >> points[i](2) >> twist_angles[i] >> rest_lengths[i];
  }

  world = NULL;
  _thread_pieces_backup.resize(points.size());
  _angle_twist_backup.resize(points.size());
  _thread_pieces.resize(points.size());
  for (int i=0; i < points.size(); i++)
  {
    _thread_pieces[i] = new ThreadPiece(points[i], twist_angles[i], rest_lengths[i], this);
  }

  for (int i=1; i < points.size(); i++)
  {
    _thread_pieces[i]->set_prev(_thread_pieces[i-1]);
  }
	_thread_pieces[0]->set_prev(NULL);
  for (int i=0; i < points.size()-1; i++)
  {
    _thread_pieces[i]->set_next(_thread_pieces[i+1]);
  }
  _thread_pieces[points.size()-1]->set_next(NULL);
  
  _total_length = 0;
	for (int piece_ind = 0; piece_ind < _thread_pieces.size()-1; piece_ind++)
	{
		_total_length += _thread_pieces[piece_ind]->rest_length();
	}

	//setup backups
  for (int i=0; i < points.size(); i++)
  {
    _thread_pieces_backup[i] = new ThreadPiece(points[i], twist_angles[i], rest_lengths[i], this);
  }

  for (int i=1; i < points.size(); i++)
  {
    _thread_pieces_backup[i]->set_prev(_thread_pieces_backup[i-1]);
  }
	_thread_pieces_backup[0]->set_prev(NULL);
  for (int i=0; i < points.size()-1; i++)
  {
    _thread_pieces_backup[i]->set_next(_thread_pieces_backup[i+1]);
  }
  _thread_pieces_backup[points.size()-1]->set_next(NULL);

  _thread_pieces.front()->set_bishop_frame(start_rot);
  _thread_pieces.front()->set_material_frame(start_rot);

  _thread_pieces.front()->initializeFrames();

  set_constraints(points.front(), start_rot, points.back(), end_rot);
}

void Thread::writeToFile(ofstream& file)
{
  vector<Vector3d> points;
  vector<double> twist_angles;
  vector<double> rest_lengths;
  Matrix3d start_rot = this->start_rot();
  Matrix3d end_rot = this->end_rot();
  get_thread_data(points, twist_angles, rest_lengths);

	file << (double)points.size() << "\n";

  for (int r=0; r < 3; r++)
  {
    for (int c=0; c < 3; c++)
    {
      file << start_rot(r,c) << " ";
    }
  }

  for (int r=0; r < 3; r++)
  {
    for (int c=0; c < 3; c++)
    {
      file << end_rot(r,c) << " ";
    }
  }

  for (int j=0; j < points.size(); j++)
  {
    file << points[j](0) << " " << points[j](1) << " " << points[j](2) << " " << twist_angles[j] << " " << rest_lengths[j] << " ";
  }
  file << "\n";
}

Thread::~Thread()
{
  delete_current_threadpieces();
}

void Thread::delete_current_threadpieces()
{
  for (int piece_ind=0; piece_ind < _thread_pieces.size(); piece_ind++)
	{
		delete _thread_pieces[piece_ind];
	}
	_thread_pieces.clear();

	for (int piece_ind=0; piece_ind < _thread_pieces_backup.size(); piece_ind++)
	{
		delete _thread_pieces_backup[piece_ind];
	}
	_thread_pieces_backup.clear();
}

void Thread::delete_threadpieces(vector<ThreadPiece*> thread_pieces)
{
  for (int piece_ind=0; piece_ind < thread_pieces.size(); piece_ind++)
	{
		delete thread_pieces[piece_ind];
	}
	thread_pieces.clear();

}


/* Calculate energy for the entire thread
 * Needs at least two thread pieces to work */
double Thread::calculate_energy()
{

#ifdef ISOTROPIC
  //double energy = _thread_pieces[2]->get_twist_coeff() * (pow(_thread_pieces[_thread_pieces.size() - 2]->angle_twist() - _thread_pieces.front()->angle_twist(),2)) / (2.0 * _rest_length * (_thread_pieces.size() - 2));
	double energy = _thread_pieces[2]->get_twist_coeff() * (pow(end_angle() - start_angle(),2)) / (2.0*total_length() - start_rest_length() - end_rest_length());
	
	//std::cout << _thread_pieces[2]->get_twist_coeff() << " " << _thread_pieces.size() << " " << _thread_pieces[_thread_pieces.size()-2]->angle_twist() << " " << _thread_pieces.front()->angle_twist() << " " << 2.0*_rest_length*(_thread_pieces.size()-2) << std::endl;

  if (REPULSION_COEFF>0) {
	  Vector3d direction;
	  
	  //self repulsion
		for(int i = 0; i < _thread_pieces.size() - 3; i++) {
			//+2 so you don't check the adjacent piece - bug?
			for(int j = i + 2; j <= _thread_pieces.size() - 2; j++) { //check. it was < instead of <=
			  //skip if both ends, since these are constraints
			  if(i == 0 && j == _thread_pieces.size() - 2) 
			    continue;
			  const double dist = self_intersection(i,j,THREAD_RADIUS,direction);
				energy += 2.0 * repulsionEnergy(dist, THREAD_RADIUS, direction);	//it's multiplied by two because this ammount of energy corresponds to each vertex of the edge
			}
		}

		//repulsions between threads
		for (int k=0; k < threads_in_env.size(); k++) {
			for(int i = 0; i < _thread_pieces.size()-1; i++) {
				for(int j = 0; j < threads_in_env[k]->_thread_pieces.size()-1; j++) {
				  //skip if both ends, since these are constraints
				  if((i==0 && j==0) ||
				  	 (i==0 && j==threads_in_env[k]->_thread_pieces.size()-2) ||
				  	 (i==_thread_pieces.size()-2 && j==0) ||
				  	 (i==_thread_pieces.size()-2 && j==threads_in_env[k]->_thread_pieces.size()-2))
				    continue;
				  const double dist = thread_intersection(i,j,k,THREAD_RADIUS,direction);		//asumes other threads have same radius
					energy += 2.0 * repulsionEnergy(dist, THREAD_RADIUS, direction);	//it's multiplied by two because this ammount of energy corresponds to each vertex of the edge
				}
			}
		}
		
		//object repulsions
		if (world != NULL)
			for(int i = 2; i < _thread_pieces.size() - 3; i++)
				energy += world->capsuleObjectRepulsionEnergy(_thread_pieces[i]->vertex(), _thread_pieces[i+1]->vertex(), THREAD_RADIUS);
	}
	
//#pragma omp parallel for num_threads(NUM_CPU_THREADS) reduction(+ : energy)
  for (int piece_ind = 0; piece_ind < _thread_pieces.size(); piece_ind++)
  {
    energy += _thread_pieces[piece_ind]->energy_curvature() + _thread_pieces[piece_ind]->energy_grav() + _thread_pieces[piece_ind]->energy_stretch();
  }
  return energy;
#else
  double energy = 0.0;
  for (int piece_ind = 0; piece_ind < _thread_pieces.size(); piece_ind++)
  {
    energy += _thread_pieces[piece_ind]->energy();
  }
  return energy;
#endif
}

void Thread::dynamic_step_until_convergence(double step_size, double mass, int max_steps) {
  VectorXd last_position;
  VectorXd current_position;

  int current_step = 0; 
  do {
    toVector(&last_position);
    //dynamic_step(step_size, mass, 1);
    minimize_energy();
    toVector(&current_position); 
    current_step += 1;
    //current_step += max_steps;

  } while ((last_position - current_position).norm() > 1e-4 && current_step < max_steps);

  
  if ((last_position - current_position).norm() > 1e-4) {
    cout << "WARNING: Did not converge in " << current_step << " steps. Current norm = " << (last_position - current_position).norm() << endl;
    minimize_energy();
  }

  //cout << (last_position - current_position).norm() << endl; ;
  //cout << current_step << endl; 

}


void Thread::dynamic_step(double step_size, double mass, int steps) { 
  vector<Vector3d> vertex_gradients(num_pieces());
  vector<Vector3d> position_offsets(num_pieces());
  vector<Vector3d> velocity(num_pieces());

  bool initialize_velocity = false; 
  if (last_velocity.size() == 0)  {
    initialize_velocity = true; 
    last_velocity.resize(num_pieces());
  }

    
  for (int t = 0; t < steps; t++) { 
    for (int i = 0; i < vertex_gradients.size(); i++) {
      velocity[i].setZero();
      if (initialize_velocity) { 
        last_velocity[i].setZero();
        initialize_velocity = false; 
      }
      vertex_gradients[i].setZero();
      position_offsets[i].setZero();
    }
    calculate_gradient_vertices(vertex_gradients);

    for (int i = 0; i < velocity.size(); i++) { 
      last_velocity[i] = 0.10*last_velocity[i] + (-vertex_gradients[i] / mass) * step_size;
      velocity[i] = last_velocity[i];
      position_offsets[i] = velocity[i] * step_size;  
    }

    //
    double max_move = 0;
    bool move_too_far = false;
    for(int i = 2; i < position_offsets.size() - 2; i++) {
      if(position_offsets[i].norm() > MAX_MOVEMENT_VERTICES) {
        move_too_far = true;
        if(position_offsets[i].norm() > max_move) 
          max_move = position_offsets[i].norm();
          //cout << "Max move = " << max_move << endl;
      }
    }
    if(move_too_far) {
      for(int i = 2; i < position_offsets.size()-2; i++) {
        position_offsets[i] = position_offsets[i] * MAX_MOVEMENT_VERTICES / 2;
      }
    }
    apply_vertex_offsets(position_offsets);

    /*if (COLLISION_CHECKING) {
      bool project_length_constraint_pass = true;
      vector<Self_Intersection> self_intersections;
      vector<Thread_Intersection> thread_intersections;
      vector<Intersection> intersections;
      int intersection_iters = 0; 
      while(check_for_intersection(self_intersections, thread_intersections, intersections) && project_length_constraint_pass) {
        fix_intersections();
        //project_length_constraint_pass &= project_length_constraint();
        intersection_iters++;
        //cout << "fixing for " << intersection_iters << " iterations." << endl;
      }
    }*/
    minimize_energy_twist_angles();
    project_length_constraint();
  }
}


double Thread::calculate_energy_inefficient()
{
  double energy = 0.0;
  for (int piece_ind = 0; piece_ind < _thread_pieces.size(); piece_ind++)
  {
    energy += _thread_pieces[piece_ind]->energy();
  }
  return energy;
}

bool debugflag = false;
bool Thread::minimize_energy(int num_opt_iters, double min_move_vert, double max_move_vert, double energy_error_for_convergence) 
{
  //backup pieces in case collision checking infinite loops due to unsatisfiable system
  //cout << "backing up pieces" << endl; 
	
	/*cout << "_thread_pieces.size(): " << _thread_pieces.size() << endl;
	for (int i=0; i<_thread_pieces.size(); i++) {
		if (_thread_pieces[i]->_my_thread == NULL)
			cout << "_thread_pieces[" << i << "]->_my_thread is NULL" << endl;
		else if (_thread_pieces[i]->_my_thread == this)
			cout << "_thread_pieces[" << i << "]->_my_thread is this" << endl;
	}*/
	
  if (COLLISION_CHECKING) { 
    save_thread_pieces_and_resize(_thread_pieces_collision_backup);
    for (int i=0; i<threads_in_env.size(); i++) {
    	//if (this == threads_in_env[i])
    		//continue;
    	threads_in_env[i]->save_thread_pieces_and_resize(threads_in_env[i]->_thread_pieces_collision_backup);
    }
  }
  bool project_length_constraint_pass = true; 

  double step_in_grad_dir_vertices = 1.0;

  vector<Vector3d> vertex_gradients(_thread_pieces.size());
  for (int piece_ind=0; piece_ind < vertex_gradients.size(); piece_ind++)
  {
    vertex_gradients[piece_ind].setZero();
  }


  double max_movement_vertices = max_move_vert;
  project_length_constraint_pass &= project_length_constraint();


  bool recalc_vertex_grad = true;

  double curr_energy = calculate_energy();
  double next_energy = 0.0;

  int opt_iter;
  for (opt_iter = 0; opt_iter < num_opt_iters; opt_iter++)
  {
   

    double curr_energy = calculate_energy();
    double next_energy = 0.0;

    if (recalc_vertex_grad)
    {
      save_thread_pieces();
      calculate_gradient_vertices(vertex_gradients);
      make_max_norm_one(vertex_gradients);
      curr_energy = calculate_energy();
    }



    
#ifndef ISOTROPIC
    save_angle_twists();
#endif
    step_in_grad_dir_vertices = max_movement_vertices/2.0;
    apply_vertex_offsets(vertex_gradients, true, -(step_in_grad_dir_vertices));
#ifndef ISOTROPIC
    minimize_energy_twist_angles();
#endif
    double search_step = max_movement_vertices/4.0;
    while (search_step > min_move_vert)
    {
      //energy for adding search_step to current step size
      apply_vertex_offsets(vertex_gradients, true, -search_step);
#ifndef ISOTROPIC
      restore_angle_twists();
      minimize_energy_twist_angles();
#endif
      double energy_add = calculate_energy();
      if (energy_add + energy_error_for_convergence < curr_energy)
      {
        step_in_grad_dir_vertices += search_step;
        break;
      }

      //energy for subtracting search_step to current step size
      apply_vertex_offsets(vertex_gradients, true, 2.0*search_step);
#ifndef ISOTROPIC
      restore_angle_twists();
      minimize_energy_twist_angles();
#endif
      double energy_subtract = calculate_energy();

      //std::cout << "energy add: " << energy_add << "  energy_subtract: " << energy_subtract << "  search step: " << search_step << " curr step: " << step_in_grad_dir_vertices << std::endl;

      if (energy_add < energy_subtract) {
        apply_vertex_offsets(vertex_gradients, true, -2.0*search_step);
        step_in_grad_dir_vertices += search_step;
      } else {
        step_in_grad_dir_vertices -= search_step;
      }
      search_step /= 2.0;

#ifndef ISOTROPIC
      minimize_energy_twist_angles();
#endif

    }

#ifndef ISOTROPIC
    minimize_energy_twist_angles();
#endif
    double energy_before_projection = calculate_energy();
    project_length_constraint_pass &= project_length_constraint();

    next_energy = calculate_energy();


   //std::cout << "curr energy: " << curr_energy << "   next energy: " << next_energy << "  before projection: " << energy_before_projection << "  last step: " << step_in_grad_dir_vertices <<  std::endl;

    recalc_vertex_grad = true;
    if (next_energy + energy_error_for_convergence > curr_energy)
    {
			opt_iter--;
      if (next_energy >= curr_energy) {
        restore_thread_pieces();
        recalc_vertex_grad = false;
      }

      if (max_movement_vertices <= min_move_vert)
      {
        break;
      }

      max_movement_vertices = step_in_grad_dir_vertices / 2.0;
    } else {
      max_movement_vertices = min(step_in_grad_dir_vertices*2.0, max_move_vert);
      //max_movement_vertices = max_move_vert;
    }

    if (COLLISION_CHECKING) {
      vector<Self_Intersection> self_intersections;
      vector<Thread_Intersection> thread_intersections;
      vector<Intersection> intersections;
      int intersection_iters = 0; 
      while(check_for_intersection(self_intersections, thread_intersections, intersections) && project_length_constraint_pass) {
        fix_intersections();
        project_length_constraint_pass &= project_length_constraint();
        intersection_iters++;
        //cout << "fixing for " << intersection_iters << " iterations." << endl;
      }
      /*if (opt_iter < 200) { 
      // reset thread_pieces and restore prior end constraints
      restore_thread_pieces_and_resize(_thread_pieces_collision_backup);
      restore_constraints(_start_pos_backup, _start_rot_backup, 
          _end_pos_backup, _end_rot_backup); 
      }*/
    } else { 

      project_length_constraint_pass &= project_length_constraint();

    }

  }

  //cout << "Number of loops needed in minimize energy: " << opt_iter << endl;
  curr_energy = calculate_energy();

  project_length_constraint_pass &= project_length_constraint();
  minimize_energy_twist_angles();

  next_energy = calculate_energy();

  if (!project_length_constraint_pass && COLLISION_CHECKING) {
    cout << "reverting thread to prior state" << endl; 
    restore_thread_pieces_and_resize(_thread_pieces_collision_backup);
    restore_constraints(_start_pos_backup, _start_rot_backup, _end_pos_backup, _end_rot_backup);
    for (int i=0; i<threads_in_env.size(); i++) {
    	//if (this == threads_in_env[i])
    		//continue;
    	threads_in_env[i]->restore_thread_pieces_and_resize(threads_in_env[i]->_thread_pieces_collision_backup);
    }
  }
  /*
     cout << "Edges: " << endl; 
     for (int i = 0; i < num_pieces()-1; i++) {
     cout << edge_at_ind(i).norm() << endl;; 
     }
  */
	
	//if (opt_iter!=-1)
		//std::cout << "num iters: " << opt_iter << " curr energy final: " << curr_energy << "   next energy final: " << next_energy <<  std::endl;

	return (opt_iter != num_opt_iters);
} // end minimize_energy


void Thread::fix_intersections() {
	vector<Self_Intersection> self_intersections;
	vector<Thread_Intersection> thread_intersections;
	vector<Intersection> intersections;
	while(check_for_intersection(self_intersections, thread_intersections, intersections)) {
	  //restore_thread_pieces();
		//cout << "INTERSECTION FOUND, total current intersections is: " << (intersections.size() + self_intersections.size())<< endl;
		debugflag = true;

		for (int i=0; i < self_intersections.size(); i++)
		{
			int ind_a = self_intersections[i]._piece_ind_a;
			int ind_b = self_intersections[i]._piece_ind_b;
			Vector3d dir = self_intersections[i]._direction;
			dir.normalize();

			double dist_a, dist_b;
			if (ind_a <= 1)
			{
				dist_a = 0.0;
				dist_b = self_intersections[i]._dist + INTERSECTION_PUSHBACK_EPS;
			} else if (ind_b >= _thread_pieces.size()-3) {
				dist_a = self_intersections[i]._dist + INTERSECTION_PUSHBACK_EPS;
				dist_b = 0.0;
			} else {
				dist_a = dist_b = self_intersections[i]._dist/2.0 + INTERSECTION_PUSHBACK_EPS;
			}

			_thread_pieces[ind_a]->offset_vertex(dir*dist_a); 
			_thread_pieces[ind_a + 1]->offset_vertex(dir*dist_a);
			_thread_pieces[ind_b]->offset_vertex(-dir*dist_b); 
			_thread_pieces[ind_b + 1]->offset_vertex(-dir*dist_b);
		}
				            
		for (int i=0; i < thread_intersections.size(); i++)
		{
			int ind_a = thread_intersections[i]._piece_ind_a;
			int ind_b = thread_intersections[i]._piece_ind_b;
			int ind_t = thread_intersections[i]._thread_ind;
			Vector3d dir = thread_intersections[i]._direction;
			dir.normalize();

			double dist_a, dist_b;
			if (ind_a <= 1 || ind_a >= _thread_pieces.size()-3) {
				dist_a = 0.0;
				dist_b = thread_intersections[i]._dist + INTERSECTION_PUSHBACK_EPS;
			} else if (ind_b <= 1 || ind_b >= threads_in_env[ind_t]->_thread_pieces.size()-3) {
				dist_a = thread_intersections[i]._dist + INTERSECTION_PUSHBACK_EPS;
				dist_b = 0.0;
			} else {
				dist_a = dist_b = thread_intersections[i]._dist/2.0 + INTERSECTION_PUSHBACK_EPS;
			}

			_thread_pieces[ind_a]->offset_vertex(dir*dist_a); 
			_thread_pieces[ind_a + 1]->offset_vertex(dir*dist_a);
			threads_in_env[ind_t]->_thread_pieces[ind_b]->offset_vertex(-dir*dist_b); 
			threads_in_env[ind_t]->_thread_pieces[ind_b + 1]->offset_vertex(-dir*dist_b);
		}

		for (int i=0; i < intersections.size(); i++)
		{
			int ind_piece = intersections[i]._piece_ind;
			Vector3d dir = intersections[i]._direction;
			dir.normalize();

			double dist = intersections[i]._dist + INTERSECTION_PUSHBACK_EPS;

			_thread_pieces[ind_piece]->offset_vertex(dir*dist); 
			_thread_pieces[ind_piece + 1]->offset_vertex(dir*dist);
			
			//glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			//glPushMatrix();
			//glTranslatef (0.0, 0.0, -110.0);
			//drawArrow(_thread_pieces[ind_piece]->vertex(), 100.0*dir*dist, 0, 1, 0);
			//drawArrow(_thread_pieces[ind_piece + 1]->vertex(), 100.0*dir*dist, 0, 1, 0);
			//glPopMatrix();
			//glutSwapBuffers ();
			//glutPostRedisplay ();
		}
	}
}

bool Thread::check_for_intersection(vector<Self_Intersection>& self_intersections, vector<Thread_Intersection>& thread_intersections, vector<Intersection>& intersections)
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

double Thread::self_intersection(int i, int j, double radius, Vector3d& direction)
{
	return capsuleCapsuleDistance(_thread_pieces[i]->vertex(), _thread_pieces[i+1]->vertex(), radius, _thread_pieces[j]->vertex(), _thread_pieces[j+1]->vertex(), radius, direction);
}

double Thread::thread_intersection(int i, int j, int k, double radius, Vector3d& direction)
{
	return capsuleCapsuleDistance(_thread_pieces[i]->vertex(), _thread_pieces[i+1]->vertex(), radius, threads_in_env[k]->_thread_pieces[j]->vertex(), threads_in_env[k]->_thread_pieces[j+1]->vertex(), radius, direction);
}

/*********************************************************************************/
//variable-length thread_pieces

void Thread::split_thread_piece(ThreadPiece* this_piece)
{
	ThreadPiece* new_piece = new ThreadPiece();
	this_piece->splitPiece(new_piece);
	
	int piece_ind;
	for (piece_ind=0; piece_ind<_thread_pieces.size() && _thread_pieces[piece_ind]!=this_piece; piece_ind++) {}
	if (piece_ind==_thread_pieces.size())
		cout << "Internal error: Thread::split_thread_piece: this_piece is not in _thread_pieces." << endl;
	
	_thread_pieces.insert(_thread_pieces.begin() + piece_ind + 1, new_piece);
}

// Merges the edges adjacent to this_piece->_vertex.
void Thread::merge_thread_piece(ThreadPiece* this_piece)
{	
	this_piece->mergePiece();
	
	int piece_ind;
	for (piece_ind=0; piece_ind<_thread_pieces.size() && _thread_pieces[piece_ind]!=this_piece; piece_ind++) {}
	if (piece_ind==_thread_pieces.size())
		cout << "Internal error: Thread::merge_thread_piece: this piece is not in _thread_pieces." << endl; // debugerror
		
	delete _thread_pieces[piece_ind-1];
	_thread_pieces[piece_ind-1] = NULL;
	_thread_pieces.erase(_thread_pieces.begin() + piece_ind - 1);
}

void Thread::adapt_links()
{
	unrefine_links();
	refine_links_geometrical();
	//if (COLLISION_CHECKING)
	//	refine_links_mechanical();
}

void Thread::refine_links_geometrical() {
	vector<ThreadPiece*> to_refine;
	priority_queue <ThreadPiece*, vector<ThreadPiece*>, angleLess<ThreadPiece*> > tp_queue;
	
	for (int i=1; i<_thread_pieces.size()-2; i++) {
		tp_queue.push(_thread_pieces[i]);
	}
	
	while((tp_queue.size() > 0) && needs_refine_geometrical(tp_queue.top())) {
		to_refine.push_back(tp_queue.top());
		tp_queue.pop();
	}

	for (int piece_ind=0; piece_ind < to_refine.size(); piece_ind++) {
		if (to_refine[piece_ind]->_prev_piece!=NULL && 
				to_refine[piece_ind]->_prev_piece->_prev_piece!=NULL && 
				to_refine[piece_ind]->_prev_piece->rest_length()>MIN_REST_LENGTH &&
				!almost_equal(to_refine[piece_ind]->_prev_piece->edge(), to_refine[piece_ind]->_prev_piece->_prev_piece->edge())) {
			split_concatenation_left(to_refine[piece_ind]->_prev_piece);
		}
		if (to_refine[piece_ind]->_next_piece!=NULL &&
				to_refine[piece_ind]->_next_piece->_next_piece!=NULL &&
				to_refine[piece_ind]->rest_length()>MIN_REST_LENGTH &&
				!almost_equal(to_refine[piece_ind]->edge(), to_refine[piece_ind]->_next_piece->edge())) {
			split_concatenation_right(to_refine[piece_ind]);
		}
	}
	save_thread_pieces_and_resize(_thread_pieces_backup);
	//minimize_energy();
}

void Thread::refine_links_mechanical() {
	vector<ThreadPiece*> to_refine;

	for (int i=1; i<_thread_pieces.size()-2; i++) {
		if (needs_refine_mechanical(_thread_pieces[i])) {
			to_refine.push_back(_thread_pieces[i]);
		}
	}

	for (int piece_ind=0; piece_ind < to_refine.size(); piece_ind++) {
		if (to_refine[piece_ind]->_prev_piece!=NULL && 
				to_refine[piece_ind]->_prev_piece->_prev_piece!=NULL && 
				to_refine[piece_ind]->_prev_piece->rest_length()>MIN_REST_LENGTH &&
				!almost_equal(to_refine[piece_ind]->_prev_piece->edge(), to_refine[piece_ind]->_prev_piece->_prev_piece->edge())) {
			split_concatenation_left(to_refine[piece_ind]->_prev_piece);
		}
		if (to_refine[piece_ind]->_next_piece!=NULL &&
				to_refine[piece_ind]->_next_piece->_next_piece!=NULL &&
				to_refine[piece_ind]->rest_length()>MIN_REST_LENGTH &&
				!almost_equal(to_refine[piece_ind]->edge(), to_refine[piece_ind]->_next_piece->edge())) {
			split_concatenation_right(to_refine[piece_ind]);
		}
	}
	save_thread_pieces_and_resize(_thread_pieces_backup);
	//minimize_energy();
}

void Thread::unrefine_links() {
	vector<ThreadPiece*> to_unrefine, tp_heap;
	priority_queue <ThreadPiece*, vector<ThreadPiece*>, angleGreater<ThreadPiece*> > tp_queue;
	
	for (int i=2; i<_thread_pieces.size()-2; i++) {
		tp_queue.push(_thread_pieces[i]);
	}
	
	while((tp_queue.size() > 0) && needs_unrefine(tp_queue.top())) {
		to_unrefine.push_back(tp_queue.top());
		tp_queue.pop();
	}

	for (int piece_ind=0; piece_ind < to_unrefine.size(); piece_ind++) {
	
		if (to_unrefine[piece_ind] == NULL) {	// this edge was already merged and invalidated with findInvalidate()
			continue;
		}
	
		if (to_unrefine[piece_ind] == NULL || to_unrefine[piece_ind]->_next_piece == NULL ||
				to_unrefine[piece_ind]->_prev_piece == NULL || to_unrefine[piece_ind]->_prev_piece->_prev_piece == NULL)
		cout << "Internal error: unrefine_links: to_unrefine[piece_ind] or one of its neighbors is NULL." << endl;
		
		if (to_unrefine[piece_ind] == NULL)
		cout << "Internal error: unrefine_links: 0" << endl;
		if (to_unrefine[piece_ind]->_next_piece == NULL)
		cout << "Internal error: unrefine_links: 1" << endl;
		if (to_unrefine[piece_ind]->_prev_piece == NULL)
		cout << "Internal error: unrefine_links: 2" << endl;
		if (to_unrefine[piece_ind]->_prev_piece->_prev_piece == NULL)
		cout << "Internal error: unrefine_links: 3" << endl;
		
		double r0 = to_unrefine[piece_ind]->rest_length();
		double r1 = to_unrefine[piece_ind]->_next_piece->rest_length();
		double l0 = to_unrefine[piece_ind]->_prev_piece->rest_length();		
		double l1 = to_unrefine[piece_ind]->_prev_piece->_prev_piece->rest_length();
		if (((l0+r0) <= (GRADING_FACTOR+GRADING_FACTOR_EPS)*r1) && ((l0+r0) <= (GRADING_FACTOR+GRADING_FACTOR_EPS)*l1)) { // merge only if the new rest length after merging is not longer by more than a factor of 2 with respect to the rest length of the new neighbors
			//remove to_unrefine[piece_ind]->_prev_piece and to_unrefine[piece_ind] from to_unrefine;
			ThreadPiece* temp = to_unrefine[piece_ind];
			findInvalidate(to_unrefine, to_unrefine[piece_ind]->_prev_piece);
			findInvalidate(to_unrefine, to_unrefine[piece_ind]);
			merge_thread_piece(temp);
		}
	}
	save_thread_pieces_and_resize(_thread_pieces_backup);
	//minimize_energy();
}

void Thread::split_concatenation_left(ThreadPiece* piece) {
	if (piece->_prev_piece != NULL) {
		double adj0 = piece->rest_length();
		double adj1 = piece->_prev_piece->rest_length();
		split_thread_piece(piece);
		if ((GRADING_FACTOR-GRADING_FACTOR_EPS)*adj0 <= adj1) {	// split only if the new rest length after splitting is not smaller by more than a factor of 2 with respect to the rest length of the new neighbors
			split_concatenation_left(piece->_prev_piece);
		}
	}
}

void Thread::split_concatenation_right(ThreadPiece* piece) {
	if (piece->_next_piece == NULL) {
		cout << "Internal error: split_concatenation_right: piece->_vertex should not be vertex _threads_pieces.size()-1." << endl;
	}
	if (piece->_next_piece != NULL && piece->_next_piece->_next_piece != NULL) {
		double adj0 = piece->rest_length();
		double adj1 = piece->_next_piece->rest_length();
		if ((GRADING_FACTOR-GRADING_FACTOR_EPS)*adj0 <= adj1) {	// split only if the new rest length after splitting is not smaller by more than a factor of 2 with respect to the rest length of the new neighbors
			split_concatenation_right(piece->_next_piece);
		}
		split_thread_piece(piece);
	}
}

// Does the edges adyacent to piece->_vertex need to be refined?		
bool Thread::needs_refine_geometrical(ThreadPiece* piece)
{
	if (piece==NULL)
		cout << "Internal error: needs_unrefine: piece is NULL" << endl;
	return (piece->angle() < REFINE_THRESHHOLD * M_PI/180.0);
}

bool Thread::needs_refine_mechanical(ThreadPiece* piece)
{
	if (piece->_prev_piece==NULL || piece->_next_piece->_next_piece==NULL)
		cout << "Internal error: needs_refine_mechanical: piece cannot be the first or the last one." << endl;
	
  if ((closest_intersection_dist(piece) < (2.0*THREAD_RADIUS+REFINE_MECHANICAL_DIST)) &&
    	(piece->rest_length() > TARGET_REST_LENGTH_REFINE_MECHANICAL)) {
    return true;
  }
  return false;
}

// Does the vertex especified by piece->_vertex need to be unrefined?
bool Thread::needs_unrefine(ThreadPiece* piece)
{
	if (piece==NULL)
		cout << "Internal error: needs_unrefine: piece is NULL" << endl;
	return ((piece->angle() > UNREFINE_THRESHHOLD * M_PI/180.0) && 
					(!COLLISION_CHECKING || (closest_intersection_dist(piece) > (2.0*THREAD_RADIUS+UNREFINE_MECHANICAL_DIST))));
}

double Thread::closest_intersection_dist(ThreadPiece* piece)
{
	if (piece==NULL)
		cout << "Internal error: closest_intersection_dist: piece is NULL" << endl;
	if (piece->_next_piece==NULL)
		cout << "Internal error: closest_intersection_dist: piece cannot be the piece after the last one." << endl;
	
	double min_dist = numeric_limits<double>::max();
	double dist;
	Vector3d direction;
	ThreadPiece* other_piece = _thread_pieces.front();
	while (other_piece!=NULL && other_piece->_next_piece != NULL) {
		if (((piece->_prev_piece!=NULL) && (other_piece == piece->_prev_piece->_prev_piece)) ||
				(other_piece == piece->_prev_piece) ||
				(other_piece == piece) ||
				(other_piece == piece->_next_piece) ||
				((piece->_next_piece!=NULL) && (other_piece == piece->_next_piece->_next_piece))) {
			other_piece = other_piece->_next_piece;
		} else {
			min_dist = min(min_dist, capsuleCapsuleDistance(piece->vertex(), piece->_next_piece->vertex(), THREAD_RADIUS, other_piece->vertex(), other_piece->_next_piece->vertex(), THREAD_RADIUS, direction));
		  other_piece = other_piece->_next_piece;
		}
	}
	return min_dist;
}

//end variable-length thread_pieces
/*********************************************************************************/

void Thread::minimize_energy_hessian(int num_opt_iters, double min_move_vert, double max_move_vert, double energy_error_for_convergence) 
{
  double max_movement_vertices = max_move_vert;
  project_length_constraint();
  bool recalc_vertex_grad = true;

  double curr_energy = calculate_energy();
  double next_energy = 0.0;

  int opt_iter;
  MatrixXd Hessian(3*(int)_thread_pieces.size(), 3*(int)_thread_pieces.size());
  VectorXd Gradients(3*(int)_thread_pieces.size());
  VectorXd offsets_vectorized(3*(int)_thread_pieces.size());
  vector<Vector3d> offsets(_thread_pieces.size());
  Gradients.setZero();
  for (opt_iter = 0; opt_iter < num_opt_iters; opt_iter++)
  {
    double curr_energy = calculate_energy();
    double next_energy = 0.0;

    if (recalc_vertex_grad)
    {
      save_thread_pieces();
      calculate_hessian_vertices(Hessian);
      calculate_gradient_vertices_vectorized(&Gradients);
      curr_energy = calculate_energy();
    }
    

    //std::cout << "hessian:\n" << Inv_Hessian << "\nGrads:\n" << Gradients << std::endl;



    Hessian.lu().solve(Gradients, &offsets_vectorized);
    //offsets_vectorized = Inv_Hessian*Gradients;
    //std::cout << "wanted step: " << offsets_vectorized.transpose() << std::endl;
    for (int piece_ind=0; piece_ind < _thread_pieces.size(); piece_ind++)
    {
      offsets[piece_ind] = offsets_vectorized.segment<3>(3*piece_ind);
    }
    make_max_norm_one(offsets);

//    apply_vertex_offsets_vectorized(-max_movement_vertices*Inv_Hessian*Gradients, true);
    apply_vertex_offsets(offsets, true, -max_movement_vertices);
      
    double energy_before_projection = calculate_energy();
    project_length_constraint();


    next_energy = calculate_energy();


    std::cout << "curr energy: " << curr_energy << "   next energy: " << next_energy << "  before projection: " << energy_before_projection << "  last step: " << max_movement_vertices <<  std::endl;

    recalc_vertex_grad = true;
    if (next_energy + energy_error_for_convergence > curr_energy)
    {
      if (next_energy >= curr_energy) {
        restore_thread_pieces();
        recalc_vertex_grad = false;
      }

      if (max_movement_vertices <= min_move_vert)
      {
        break;
      }

      max_movement_vertices = max_movement_vertices / 2.0;
    } else {
      max_movement_vertices = min(max_movement_vertices*2.0, max_move_vert);
      //max_movement_vertices = max_move_vert;
    }

  }

  curr_energy = calculate_energy();

  project_length_constraint();
  //minimize_energy_twist_angles();

  next_energy = calculate_energy();

  //std::cout << "num iters: " << opt_iter << " curr energy final: " << curr_energy << "   next energy final: " << next_energy <<  std::endl;
}



void Thread::one_step_project(double step_size, bool normalize_gradient)
{


  vector<Vector3d> vertex_gradients(_thread_pieces.size());
  for (int piece_ind=0; piece_ind < vertex_gradients.size(); piece_ind++)
  {
    vertex_gradients[piece_ind].setZero();
  }
  

  calculate_gradient_vertices(vertex_gradients);

  double norm_vecs = 1;
  if (normalize_gradient)
  {
    norm_vecs = 0.0;
    for (int piece_ind=2; piece_ind < _thread_pieces.size()-2; piece_ind++)
    {
      norm_vecs += vertex_gradients[piece_ind].squaredNorm();
      //norm_vecs += vertex_gradients[piece_ind].norm();
    }
    norm_vecs = sqrt(norm_vecs);
  }
  //std::cout << "norm vecs: " << norm_vecs << std::endl;


 // make_max_norm_one(vertex_gradients);
  apply_vertex_offsets(vertex_gradients, true, -(step_size/norm_vecs), true);
  project_length_constraint();

}


double Thread::one_step_grad_change(double step_size)
{
  vector<Vector3d> vertex_gradients(_thread_pieces.size());
  for (int piece_ind=0; piece_ind < vertex_gradients.size(); piece_ind++)
  {
    vertex_gradients[piece_ind].setZero();
  }
  

  calculate_gradient_vertices(vertex_gradients);
  double norm_vecs = 0.0;
  for (int piece_ind=2; piece_ind < _thread_pieces.size()-2; piece_ind++)
  {
    norm_vecs += vertex_gradients[piece_ind].squaredNorm();
    //norm_vecs += vertex_gradients[piece_ind].norm();
  }
  norm_vecs = sqrt(norm_vecs);

  //std::cout << "norm vecs: " << norm_vecs << std::endl;

	vector<Vector3d> points;
	vector<double> angles;

	get_thread_data(points, angles);

 // make_max_norm_one(vertex_gradients);
  apply_vertex_offsets(vertex_gradients, true, -(step_size/norm_vecs), true);
  project_length_constraint();


	vector<Vector3d> points_new;
	vector<double> angles_new;

	get_thread_data(points_new, angles_new);
	double dist = 0.0;
  for (int piece_ind=2; piece_ind < _thread_pieces.size()-2; piece_ind++)
	{
		Vector3d point_movement = points_new[piece_ind]-points[piece_ind];
		double dot_prod = (-vertex_gradients[piece_ind]).dot(point_movement);
		if ((vertex_gradients[piece_ind].norm() > 1e-7) && (point_movement.norm() > 1e-7))
		{
			double thing = max (min (1.0, dot_prod/(vertex_gradients[piece_ind].norm() * point_movement.norm())), -1.0);
			dist += acos(thing);
		}
	}
	dist /= ((double)(_thread_pieces.size()-4));

	return dist;


}




void Thread::minimize_energy_twist_angles()
{
#ifdef ISOTROPIC
  double accum_rest_length = rest_length_at_ind(0);
  double angle_per_length = end_angle() / (_total_length - end_rest_length());
  
  //#pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
  for (int piece_ind = 1; piece_ind < _thread_pieces.size()-2; piece_ind++)
 	{
 		_thread_pieces[piece_ind]->set_angle_twist(angle_per_length*accum_rest_length);
    _thread_pieces[piece_ind]->updateFrames_twistOnly();
 		accum_rest_length += rest_length_at_ind(piece_ind);
 	}

#else
  double step_in_grad_dir_twist = 1.0;
  double max_rotation_twist = MAX_ROTATION_TWIST;

  const int num_opt_iters = 1500;
  const double energy_error_for_convergence = 1e-40;

  vector<double> angle_twist_gradients;
  angle_twist_gradients.resize(_thread_pieces.size());



  //vector<double> angles_before(_thread_pieces.size());
  //vector<double> angles_after(_thread_pieces.size());
  //save_angle_twists(angles_before);
  int opt_iter;
  for (opt_iter = 0; opt_iter < num_opt_iters; opt_iter++)
  {
    calculate_gradient_twist(angle_twist_gradients);
    double max_grad_norm = (*(std::max_element(angle_twist_gradients.begin()+1, angle_twist_gradients.end()-2)));
    double mult_const = 1.0/max_grad_norm;

    for (int piece_ind=1; piece_ind < _thread_pieces.size()-1; piece_ind++)
    {
      angle_twist_gradients[piece_ind] *= mult_const;
    }


    double curr_energy = calculate_energy();
    double next_energy = 0.0;

    step_in_grad_dir_twist = max_rotation_twist/2.0;
    apply_angle_twist_offsets(angle_twist_gradients, true, -(step_in_grad_dir_twist));
    double search_step = max_rotation_twist/4.0;
    while (search_step > MIN_ROTATION_TWIST)
    {
      //energy for adding search_step to current step size
      apply_angle_twist_offsets(angle_twist_gradients, true, -search_step);
      double energy_add = calculate_energy();

      apply_angle_twist_offsets(angle_twist_gradients, true, 2.0*search_step);
      double energy_subtract = calculate_energy();

      if (energy_add < energy_subtract) {
        apply_angle_twist_offsets(angle_twist_gradients, true, -2.0*search_step);
        step_in_grad_dir_twist += search_step;
      } else {
        step_in_grad_dir_twist -= search_step;
      }
      search_step /= 2.0;
    }


    next_energy = calculate_energy();


    //std::cout << "curr energy: " << curr_energy << "   next energy: " << next_energy << "  last step: " << step_in_grad_dir_twist <<  std::endl;

    if (next_energy + energy_error_for_convergence > curr_energy)
    {
        break;
    } else {
      max_rotation_twist = min(step_in_grad_dir_twist*2.0, MAX_ROTATION_TWIST);
    }

    //std::cout << "max: " << max_movement_vertices << std::endl;

  }
  double curr_energy, next_energy;
  curr_energy = calculate_energy();
  next_energy = calculate_energy();

  //std::cout << "num iters: " << opt_iter << " curr energy final: " << curr_energy << "   next energy final: " << next_energy <<  std::endl;

#endif

}

void Thread::match_start_and_end_constraints(Thread& to_match, int num_steps, int num_steps_break)
{

  Vector3d start_pos = this->start_pos();
  Vector3d start_goal_pos = to_match.start_pos();

  Vector3d end_pos = this->end_pos();
  Vector3d end_goal_pos = to_match.end_pos();

  Vector3d start_translation = start_goal_pos - start_pos;
  Vector3d end_translation = end_goal_pos - end_pos;
  for (int step_num=0; step_num < num_steps-1; step_num++)
  {
    if (step_num > num_steps_break)
      return;
    // use quaternion interpolation to move closer to end rot
    Matrix3d end_rot = this->end_rot();
    start_pos = this->start_pos();

    Matrix3d end_goal_rot = to_match.end_rot();
    Eigen::Quaterniond endq(end_rot);
    Eigen::Quaterniond end_goalq(end_goal_rot);



    double for_ang, angle;
    angle = DBL_MIN;
    Vector3d after_end_goal;
    Vector3d after_end;
    for (int col_num=0; col_num < 3; col_num++)
    {
        after_end_goal = end_goal_rot.col(col_num);
        after_end= end_rot.col(col_num);

        for_ang = after_end_goal.dot(after_end);
        for_ang = max (min (for_ang, 1.0), -1.0);
        angle = max(acos(for_ang), angle);
    }
    //cout << "Angle = " << angle << endl; 
    double t = angle* ((double)(step_num+1))/((double)num_steps);
  

    /*double for_ang = after_end_goal.dot(after_end);
    for_ang = max (min (for_ang, 1.0), -1.0);
    double angle = acos(for_ang);
    cout << "Angle = " << angle << endl; 
    double t = angle* ((double)step_num)/((double)num_steps);
    */
    Eigen::Quaterniond finalq = endq.slerp(t, end_goalq).normalized();

    Matrix3d end_rotation(finalq*endq.inverse());
    end_pos = this->end_pos();


    Matrix3d start_rot = this->start_rot();

    Matrix3d start_goal_rot = to_match.start_rot();
    Eigen::Quaterniond startq(start_rot);
    Eigen::Quaterniond start_goalq(start_goal_rot);

    angle = DBL_MIN;
    Vector3d after_start_goal;
    Vector3d after_start;
    for (int col_num=0; col_num < 3; col_num++)
    {
        after_start_goal = start_goal_rot.col(col_num);
        after_start= start_rot.col(col_num);

        for_ang = after_start_goal.dot(after_start);
        for_ang = max (min (for_ang, 1.0), -1.0);
        angle = max(acos(for_ang), angle);
    }
    //cout << "Angle = " << angle << endl; 
    t = angle* ((double)(step_num+1))/((double)num_steps);


    finalq = startq.slerp(t, start_goalq).normalized();

    Matrix3d start_rotation(finalq*startq.inverse());


    // use linear interpolation to move closer to end pos

    // apply the motion
    Frame_Motion toMove_start(start_translation/((double)num_steps), start_rotation);
    Frame_Motion toMove_end(end_translation/((double)num_steps), end_rotation);
    toMove_start.applyMotion(start_pos, start_rot);
    toMove_end.applyMotion(end_pos, end_rot);

    set_constraints(start_pos, start_rot, end_pos, end_rot);
    minimize_energy();
  }

  //last step, just match constraints
  set_constraints(to_match.start_pos(), to_match.start_rot(), to_match.end_pos(), to_match.end_rot());
  minimize_energy(30000, 1e-6, 0.2, 1e-7);
  //minimize_energy();

}


void Thread::match_start_and_end_constraints(Thread& to_match, int num_steps, int num_steps_break, vector<Thread*>& intermediates)
{

    minimize_energy(1000, 1e-6, 0.2, 1e-7);
    intermediates.push_back(new Thread(*this));

  Vector3d start_pos = this->start_pos();
  Vector3d start_goal_pos = to_match.start_pos();

  Vector3d end_pos = this->end_pos();
  Vector3d end_goal_pos = to_match.end_pos();

  Vector3d start_translation = start_goal_pos - start_pos;
  Vector3d end_translation = end_goal_pos - end_pos;
  for (int step_num=0; step_num < num_steps-1; step_num++)
  {
    if (step_num > num_steps_break)
      return;
    // use quaternion interpolation to move closer to end rot
    Matrix3d end_rot = this->end_rot();
    start_pos = this->start_pos();

    Matrix3d end_goal_rot = to_match.end_rot();
    Eigen::Quaterniond endq(end_rot);
    Eigen::Quaterniond end_goalq(end_goal_rot);



    double for_ang, angle;
    angle = DBL_MIN;
    Vector3d after_end_goal;
    Vector3d after_end;
    for (int col_num=0; col_num < 3; col_num++)
    {
        after_end_goal = end_goal_rot.col(col_num);
        after_end= end_rot.col(col_num);

        for_ang = after_end_goal.dot(after_end);
        for_ang = max (min (for_ang, 1.0), -1.0);
        angle = max(acos(for_ang), angle);
    }
    //cout << "Angle = " << angle << endl; 
    double t = angle* ((double)(step_num+1))/((double)num_steps);
  

    /*double for_ang = after_end_goal.dot(after_end);
    for_ang = max (min (for_ang, 1.0), -1.0);
    double angle = acos(for_ang);
    cout << "Angle = " << angle << endl; 
    double t = angle* ((double)step_num)/((double)num_steps);
    */
    Eigen::Quaterniond finalq = endq.slerp(t, end_goalq).normalized();

    Matrix3d end_rotation(finalq*endq.inverse());
    end_pos = this->end_pos();


    Matrix3d start_rot = this->start_rot();

    Matrix3d start_goal_rot = to_match.start_rot();
    Eigen::Quaterniond startq(start_rot);
    Eigen::Quaterniond start_goalq(start_goal_rot);

    angle = DBL_MIN;
    Vector3d after_start_goal;
    Vector3d after_start;
    for (int col_num=0; col_num < 3; col_num++)
    {
        after_start_goal = start_goal_rot.col(col_num);
        after_start= start_rot.col(col_num);

        for_ang = after_start_goal.dot(after_start);
        for_ang = max (min (for_ang, 1.0), -1.0);
        angle = max(acos(for_ang), angle);
    }
    //cout << "Angle = " << angle << endl; 
    t = angle* ((double)(step_num+1))/((double)num_steps);


    finalq = startq.slerp(t, start_goalq).normalized();

    Matrix3d start_rotation(finalq*startq.inverse());


    // use linear interpolation to move closer to end pos

    // apply the motion
    Frame_Motion toMove_start(start_translation/((double)num_steps), start_rotation);
    Frame_Motion toMove_end(end_translation/((double)num_steps), end_rotation);
    toMove_start.applyMotion(start_pos, start_rot);
    toMove_end.applyMotion(end_pos, end_rot);

    set_constraints(start_pos, start_rot, end_pos, end_rot);
    minimize_energy(30000, 1e-6, 0.2, 1e-7);
    intermediates.push_back(new Thread(*this));
  }

  //last step, just match constraints
  set_constraints(to_match.start_pos(), to_match.start_rot(), to_match.end_pos(), to_match.end_rot());
  minimize_energy(30000, 1e-6, 0.2, 1e-7);
  intermediates.push_back(new Thread(*this));
  //minimize_energy();

}


void Thread::calculate_gradient(vector<Vector3d>& vertex_gradients, vector<double>& angle_twist_gradients)
{
  calculate_gradient_vertices(vertex_gradients);
  calculate_gradient_twist(angle_twist_gradients);
}

void Thread::calculate_gradient_vertices_vectorized(VectorXd* gradient) {
  Vector3d tmp;
  for (int piece_ind = 2; piece_ind < _thread_pieces.size()-2; piece_ind++)
  {
    _thread_pieces[piece_ind]->gradient_vertex(tmp);
    gradient->segment<3>(3*piece_ind) = tmp;
  }
}

void Thread::calculate_gradient_vertices(vector<Vector3d>& vertex_gradients)
{
  //#pragma omp parallel for num_threads(NUM_CPU_THREADS)
  for (int piece_ind = 2; piece_ind < _thread_pieces.size()-2; piece_ind++)
  {
    //_thread_pieces[piece_ind]->gradient_vertex_numeric(vertex_gradients[piece_ind]);
    //std::cout << "piece ind " << piece_ind << " old grad: " << vertex_gradients[piece_ind].transpose() << std::endl;
    _thread_pieces[piece_ind]->gradient_vertex(vertex_gradients[piece_ind]);
    //std::cout << "piece ind " << piece_ind << " new grad: " << vertex_gradients[piece_ind].transpose() << std::endl;

  }
  
  if (REPULSION_COEFF>0) {
	  Vector3d direction;

	  //self repulsion
		for(int i = 0; i < _thread_pieces.size() - 3; i++) {
			//+2 so you don't check the adjacent piece - bug?
			for(int j = i + 2; j <= _thread_pieces.size() - 2; j++) { //check. it was < instead of <=
			  //skip if both ends, since these are constraints
			  if(i == 0 && j == _thread_pieces.size() - 2) 
			    continue;
			  const double dist = self_intersection(i,j,THREAD_RADIUS,direction);
				const Vector3d grad = repulsionEnergyGradient(dist, THREAD_RADIUS, direction);
				vertex_gradients[i] -= grad;
				vertex_gradients[i+1] -= grad;
				vertex_gradients[j] += grad;
				vertex_gradients[j+1] += grad;
			}
		}

		//repulsions between threads
		for (int k=0; k < threads_in_env.size(); k++) {
			for(int i = 0; i < _thread_pieces.size()-1; i++) {
				for(int j = 0; j < threads_in_env[k]->_thread_pieces.size()-1; j++) {
				  //skip if both ends, since these are constraints
				  if((i==0 && j==0) ||
				  	 (i==0 && j==threads_in_env[k]->_thread_pieces.size()-2) ||
				  	 (i==_thread_pieces.size()-2 && j==0) ||
				  	 (i==_thread_pieces.size()-2 && j==threads_in_env[k]->_thread_pieces.size()-2))
				    continue;
				  const double dist = thread_intersection(i,j,k,THREAD_RADIUS,direction);		//asumes other threads have same radius
				  const Vector3d grad = repulsionEnergyGradient(dist, THREAD_RADIUS, direction);
				  vertex_gradients[i] -= grad;
					vertex_gradients[i+1] -= grad;
				}
			}
		}
		
		//object repulsions
		if (world != NULL)
			for(int i = 2; i < _thread_pieces.size() - 3; i++) {
				Vector3d grad = Vector3d::Zero();
				world->capsuleObjectRepulsionEnergyGradient(_thread_pieces[i]->vertex(), _thread_pieces[i+1]->vertex(), THREAD_RADIUS, grad);
				vertex_gradients[i] += grad;
				vertex_gradients[i+1] += grad;
			}
	}
  
}


void Thread::calculate_hessian_vertices(MatrixXd& hessian)
{
  //ignore first and last 2 pieces
  hessian.setZero();
  const double eps = 1e-3;
  Vector3d grad_offets[3];
  grad_offsets[0] = Vector3d(eps, 0, 0);
  grad_offsets[1] = Vector3d(0, eps, 0);
  grad_offsets[2] = Vector3d(0, 0, eps);

  Vector3d gradient;
  //vector<ThreadPiece*> backup_pieces;
  for (int r=2; r < _thread_pieces.size()-2; r++)
  {
    int r_ind = r-2;
    for (int grad_ind = 0; grad_ind < 3; grad_ind++)
    {
      //save_thread_pieces_and_resize(backup_pieces);
      _thread_pieces[r]->offset_and_update(grad_offsets[grad_ind]);
      for (int c=max(2,r-2); c < min((int)_thread_pieces.size()-2,r+3); c++)
      {
        int c_ind = c-2;
        _thread_pieces[c]->gradient_vertex(gradient);
        hessian.block(6+3*r_ind+grad_ind ,3*c_ind+6,1,3) = gradient.transpose();
      }

      //restore_thread_pieces(backup_pieces);
      _thread_pieces[r]->offset_and_update(-2.0*grad_offsets[grad_ind]);

      for (int c=max(2,r-2); c < min((int)_thread_pieces.size()-2,r+3); c++)
      {
        int c_ind = c-2;
        _thread_pieces[c]->gradient_vertex(gradient);
        hessian.block(6+3*r_ind+grad_ind, 3*c_ind+6,1,3) -= gradient.transpose();
        hessian.block(6+3*r_ind+grad_ind, 3*c_ind+6,1,3) /= 2.0*eps;
      }
      _thread_pieces[r]->offset_and_update(grad_offsets[grad_ind]);
      //restore_thread_pieces_and_resize(backup_pieces);
    } 
  }



  MatrixXd check_transposes(hessian.cols(), hessian.rows());
  for (int r=0; r < hessian.rows(); r++)
  {
    for (int c=0; c < hessian.cols(); c++)
    {
      if (abs(hessian(r,c) - hessian(c,r)) < 1e-2)
        check_transposes(r,c) = 0;
      else
        check_transposes(r,c) = (abs(hessian(r,c) - hessian(c,r)));

      
      if (abs(hessian(r,c)) < 1e-5)
        hessian(r,c) = 0;
    }

  }
/*
  //calculate pseudoinverse with svd
  Eigen::SVD<Eigen::MatrixXd> hessian_svd;
  hessian_svd.compute(hessian);


  MatrixXd sing_vals_mat(hessian_svd.singularValues().rows(), hessian_svd.singularValues().rows());
  sing_vals_mat.setZero();
  for (int i=0; i < hessian_svd.singularValues().rows(); i++)
  {
    if (hessian_svd.singularValues()(i) != 0)
      sing_vals_mat(i,i) = 1.0/(hessian_svd.singularValues()(i));
  }
  inv_hessian.block(6,6,hessian.cols(), hessian.rows()) = hessian_svd.matrixV()*sing_vals_mat*hessian_svd.matrixU().transpose();
  //inv_hessian.block(6,6,hessian.cols(), hessian.rows()) = hessian;

  MatrixXd hess_times_inv(hessian.rows(), hessian.cols());
  hess_times_inv = hessian*inv_hessian.block(6,6,hessian.cols(),hessian.rows());
  MatrixXd inv_times_hess(hessian.rows(), hessian.cols());
  inv_times_hess = inv_hessian.block(6,6,hessian.cols(),hessian.rows())*hessian;

  for (int r=0; r < hessian.rows(); r++)
  {
    for (int c=0; c < hessian.cols(); c++)
    {
      if (abs(hess_times_inv(r,c)) < 1e-4)
        hess_times_inv(r,c) = 0;

      if (abs(inv_times_hess(r,c)) < 1e-4)
        inv_times_hess(r,c) = 0;
    }
  }
  */
 
  //skip first 2 pieces
  //inv_hessian.block(6,6,hessian.cols(), hessian.rows()) = hessian.inverse();
  //std::cout << hessian << "\n\n\n";
  //std::cout << hessian_svd.matrixU()*sing_vals_mat*hessian_svd.matrixV().transpose() << "\n\n\n";
  //std::cout << check_transposes << "\n\n\n";
  //std::cout << inv_hessian.block(6,6,hessian.rows(), hessian.cols()) << "\n\n\n";
  //std::cout << hess_times_inv << "\n\n\n";
  //std::cout << inv_times_hess << "\n\n\n";
  //exit(0);


}

void Thread::make_max_norm_one(vector<Vector3d>& to_normalize)
{
  double max_grad_norm = (*(std::max_element(to_normalize.begin()+2, to_normalize.end()-2, compare_vector_norms))).norm();


//  #pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
  for (int pieceInd = 2; pieceInd < _thread_pieces.size()-2; pieceInd++)
  {
    to_normalize[pieceInd] /= max_grad_norm;
  }
  

}

void Thread::make_max_norm_one_allPieces(vector<Vector3d>& to_normalize)
{
  double max_grad_norm = (*(std::max_element(to_normalize.begin(), to_normalize.end(), compare_vector_norms))).norm();

//  #pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
  for (int pieceInd = 0; pieceInd < _thread_pieces.size(); pieceInd++)
  {
    to_normalize[pieceInd] /= max_grad_norm;
  }

}

void Thread::add_momentum_to_gradient(vector<Vector3d>& vertex_gradients, vector<Vector3d>& new_gradients, double last_step_size)
{
  double max_grad_norm = (*(std::max_element(new_gradients.begin()+2, new_gradients.end()-2, compare_vector_norms))).norm();
  double mult_const = 1.0/max_grad_norm;
  double last_vertex_mult_const = MOMENTUM_CONSTANT*last_step_size/MAX_MOVEMENT_VERTICES;

  //#pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
  for (int piece_ind = 2; piece_ind < new_gradients.size()-2; piece_ind++)
  {
    vertex_gradients[piece_ind] = new_gradients[piece_ind]*mult_const + last_vertex_mult_const*vertex_gradients[piece_ind];
  }

}


void Thread::calculate_gradient_twist(vector<double>& angle_twist_gradients)
{
  //#pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
  for (int piece_ind = 1; piece_ind < _thread_pieces.size()-2; piece_ind++)
  {
    _thread_pieces[piece_ind]->gradient_twist(angle_twist_gradients[piece_ind]);
  }

}

/*void Thread::project_length_constraint_old()
{
  const int num_iters_project = 5000;
  const double projection_scale_factor = 0.75;
  const double max_norm_to_break = 1e-5;


  vector<Vector3d> vertex_offsets(_thread_pieces.size());
  for (int i=0; i < _thread_pieces.size(); i++)
  {
    vertex_offsets[i].setZero();
  }


  int iter_num;
  for (iter_num=0; iter_num < num_iters_project; iter_num++)
  {
    //first and last 2 pieces are special cases - assumed length is correct
    //so 2nd edge and 2nd to last edge are treated as special cases
    Vector3d curr_edge = _thread_pieces[2]->vertex() - _thread_pieces[1]->vertex();
    double edge_norm = curr_edge.norm();
    bool projected_enough = abs(edge_norm-_rest_length) < max_norm_to_break;
    vertex_offsets[2] = (curr_edge/edge_norm)*((_rest_length-edge_norm));
    for (int edge_ind = 2; edge_ind < _thread_pieces.size()-3; edge_ind++)
    {
      curr_edge = _thread_pieces[edge_ind+1]->vertex() - _thread_pieces[edge_ind]->vertex();
      edge_norm = curr_edge.norm();
      curr_edge = curr_edge/edge_norm;
      //this already set by previous iteration, so just add this offset
      vertex_offsets[edge_ind] += (curr_edge)*((edge_norm - _rest_length)/2.0);
      //initialize this vertex
      vertex_offsets[edge_ind+1] = (curr_edge)*((_rest_length - edge_norm)/2.0);
      projected_enough &= (abs(edge_norm-_rest_length) < max_norm_to_break);
    }

    curr_edge = _thread_pieces[_thread_pieces.size()-2]->vertex() - _thread_pieces[_thread_pieces.size()-3]->vertex();
    edge_norm = curr_edge.norm();
    vertex_offsets[_thread_pieces.size()-3] += (curr_edge/edge_norm)*((edge_norm - _rest_length));
    projected_enough &= (abs(edge_norm-_rest_length) < max_norm_to_break);
*//*

    Vector3d edge_before = _thread_pieces[2]->vertex() - _thread_pieces[1]->vertex();
    double edge_before_norm = edge_before.norm();
    Vector3d edge_after = _thread_pieces[2]->vertex() - _thread_pieces[3]->vertex();
    double edge_after_norm = edge_after.norm();
    vertex_offsets[2] = (edge_before/edge_before_norm) * (_rest_length-edge_before_norm) + 0.5*(edge_after/edge_after_norm) * (_rest_length-edge_after_norm);

    bool projected_enough = abs(edge_before_norm-_rest_length) < max_norm_to_break;

    edge_before = _thread_pieces[_thread_pieces.size()-3]->vertex() - _thread_pieces[_thread_pieces.size()-4].vertex();
    edge_before_norm = edge_before.norm();
    edge_after = _thread_pieces[_thread_pieces.size()-3]->vertex() - _thread_pieces[_thread_pieces.size()-2]->vertex();
    edge_after_norm = edge_after.norm();;
    vertex_offsets[_thread_pieces.size()-3] = 0.5*(edge_before/edge_before_norm)*(_rest_length-edge_before_norm) + (edge_after/edge_after_norm) * (_rest_length-edge_after_norm);

    //projected_enough &= abs(edge_after_norm-_rest_length) < max_norm_to_break;

    #pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
    for (int edge_ind = 3; edge_ind < _thread_pieces.size()-4; edge_ind++)
    {
      Vector3d edge_before_curr = _thread_pieces[edge_ind]->vertex() - _thread_pieces[edge_ind-1].vertex();
      double edge_before_norm_curr = edge_before_curr.norm();
      Vector3d edge_after_curr = _thread_pieces[edge_ind]->vertex() - _thread_pieces[edge_ind+1]->vertex();
      double edge_after_norm_curr = edge_after_curr.norm();
      vertex_offsets[edge_ind] = 0.5* ((edge_before_curr/edge_before_norm_curr) * (_rest_length-edge_before_norm_curr) + (edge_after_curr/edge_after_norm_curr) * (_rest_length-edge_after_norm_curr));

      if (abs(edge_before_norm_curr-_rest_length) > max_norm_to_break)
        projected_enough = false;
    }

*//*     
			if (vertex_offsets[_thread_pieces.size()-3].norm() > 3)
      {
        std::cout << "iter " << iter_num << "piece " << (_thread_pieces.size()-3) << " projection offset big " << vertex_offsets[_thread_pieces.size()-3]<< std::endl;
      }
      *//*
    //apply the actual offsets
    apply_vertex_offsets(vertex_offsets, true, projection_scale_factor, iter_num == (num_iters_project-1) || projected_enough);

    if (projected_enough)
      break;

  }


  //std::cout << "num projs: " << iter_num << std::endl;

}*/


//this function might introduce intersections. fix_intersections should be called (with caution) afterwards. 
bool Thread::project_length_constraint(int recursive_depth)
{

  if (recursive_depth <= 0) {
    //cout << "project length constraint recursively called too much!" << endl;
    return false; 
  }

  const int num_iters_project = 50;
  const double projection_scale_factor = 1.0;
  const double max_norm_to_break = DEFAULT_REST_LENGTH * 1e-2; //1e-5;


  //quick check to see if we can avoid calling this function
  bool all_norms_within_thresh = true;
  for (int piece_ind = 0; piece_ind < _thread_pieces.size()-1; piece_ind++)
  {
    all_norms_within_thresh &= abs(_thread_pieces[piece_ind]->edge_norm() - _thread_pieces[piece_ind]->rest_length()) < max_norm_to_break;
  }
  if (all_norms_within_thresh)
    return true;

  // set up the augmented lagrangian system: need C, grad C, mass mat.
  // y is the coordinates of all vertices
  // dy is what we are solving for
  // optimize dy ' * dy + C lambda
  // what is h?
  // \nabla C(x) nabla C(x)^T lambda = C,
  // dy = nabla C(x_j) lambda
  // dy = -nabla C(x_j)^T (nabla C(x) nabla C(x)^T)^-1 C(x)

  int N = _thread_pieces.size();
  VectorXd C(N-3);
  MatrixXd gradC(N-3, 3*(N-4));

  vector<Vector3d> vertex_offsets(_thread_pieces.size());

  int iter_num;

  Vector3d next_point;
  Vector3d cur_point;
  double normerror;
  VectorXd dy(3*(N-4));
  for (iter_num=0; iter_num < num_iters_project; iter_num++)
  {
    C.setZero();
    gradC.setZero();
    dy.setZero();

    for(int i = 0; i < N-3; i++) {
      next_point = _thread_pieces[i+2]->vertex();
      cur_point = _thread_pieces[i+1]->vertex();

      C[i] = (next_point - cur_point).squaredNorm() - pow(_thread_pieces[i+1]->rest_length(),2);
    }

    gradC.block<1,3>(0,0)         = (_thread_pieces[2]->vertex() - _thread_pieces[1]->vertex())*2;
    for(int i = 1; i < N-4; i++) {
      gradC.block<1,3>(i,3*(i-1)) = (_thread_pieces[i+2]->vertex() - _thread_pieces[i+1]->vertex())*-2;
      gradC.block<1,3>(i,3*i)     = (_thread_pieces[i+2]->vertex() - _thread_pieces[i+1]->vertex())*2;
    }
    gradC.block<1,3>(N-4,3*(N-5)) = (_thread_pieces[N-2]->vertex() - _thread_pieces[N-3]->vertex())*-2;

    // slower lu decomposition
    // VectorXd dl(N-5);
    // (gradC*gradC.transpose()).lu().solve(C,&dl);
    // VectorXd dy = -gradC.transpose()*dl;

    //gradC*gradC.transpose();
    (gradC*gradC.transpose()).ldlt().solveInPlace(C);
    dy = -gradC.transpose()*C;

    for(int i = 2; i < N-2; i++) {
      for(int j = 0; j < 3; j++) {
        vertex_offsets[i](j) = dy(3*(i-2)+j);
      }
    }


    bool projected_enough = true;
    normerror = 0.0;
    for(int i = 0; i < N-5; i++) {
      projected_enough &= (abs(C[i]) < max_norm_to_break);
      normerror += abs(C[i]);
    }
    // find max norm and store in vertex_offsets (for now just announce, max movement too large)
    // if greater than .2, scale vertex_offsets by max norm
    // check for and fix intersections
    // call project_length constraints if needed (don't let fall off end without fixing length constrainsts and intersections)

    double max_move = 0;
    bool move_too_far = false;
    for(int i = 2; i < vertex_offsets.size() - 2; i++) {
        if(vertex_offsets[i].norm() > MAX_MOVEMENT_VERTICES) {
          //cout << "MOVED MORE THAN MAX_MOVE_VERTICES IN PROJECT_LENGTH CONSTRAINTS, CANNOT GUARUNTEE LACK OF SELF INTERSECTION)" << endl;
          //cout << "vertex " << i << "moved: " << vertex_offsets[i].norm() << " vector3d " << vertex_offsets[i] << endl;
          move_too_far = true;
          if(vertex_offsets[i].norm() > max_move) max_move = vertex_offsets[i].norm();
          }
    }
    
    if(move_too_far) {
        for(int i = 2; i < vertex_offsets.size()-2; i++) {
           vertex_offsets[i] = vertex_offsets[i] * MAX_MOVEMENT_VERTICES / max_move;
        }
    }


    //apply the actual offsets
    apply_vertex_offsets(vertex_offsets, true, projection_scale_factor,true /*iter_num == (num_iters_project-1) || projected_enough*/);
    

    int intersection_iters = 0; 
    if (COLLISION_CHECKING) { 
      vector<Self_Intersection> self_intersections;
      vector<Thread_Intersection> thread_intersections;
      vector<Intersection> intersections;
      while(check_for_intersection(self_intersections, thread_intersections, intersections)) {
        fix_intersections();
        intersection_iters++;
        //cout << "PLC: fixing for " << intersection_iters << " iterations."
        //  << endl;
      }
    }

    if(move_too_far) {
      //if (intersection_iters == 0) { 
      //cout << "plc: calling plc again" << endl; 
      return project_length_constraint(recursive_depth-1);
      //}
    }

    
    if (projected_enough)
      break;

  }
  //cout << iter_num << " " << normerror << endl;

  return true; 

}

/*void Thread::project_length_constraint_slow()
{
  const int num_iters_project = 900000;
  const double projection_scale_factor = 0.33;
  const double max_norm_to_break = 1e-8;


  vector<Vector3d> vertex_offsets(_thread_pieces.size());
  for (int i=0; i < _thread_pieces.size(); i++)
  {
    vertex_offsets[i] = Vector3d::Zero();
  }


  int iter_num;
  for (iter_num=0; iter_num < num_iters_project; iter_num++)
  {
    //first and last 2 pieces are special cases - assumed length is correct
    //so 2nd edge and 2nd to last edge are treated as special cases
    Vector3d curr_edge = _thread_pieces[2]->vertex() - _thread_pieces[1]->vertex();
    double edge_norm = curr_edge.norm();
    bool projected_enough = abs(edge_norm-_rest_length) < max_norm_to_break;
    vertex_offsets[2] = (curr_edge/edge_norm)*((_rest_length-edge_norm));
    for (int edge_ind = 2; edge_ind < _thread_pieces.size()-3; edge_ind++)
    {
      curr_edge = _thread_pieces[edge_ind+1]->vertex() - _thread_pieces[edge_ind]->vertex();
      edge_norm = curr_edge.norm();
      curr_edge = curr_edge/edge_norm;
      //this already set by previous iteration, so just add this offset
      vertex_offsets[edge_ind] += (curr_edge)*((edge_norm - _rest_length)/2.0);
      //initialize this vertex
      vertex_offsets[edge_ind+1] = (curr_edge)*((_rest_length - edge_norm)/2.0);
      projected_enough &= (abs(edge_norm-_rest_length) < max_norm_to_break);
    }

    curr_edge = _thread_pieces[_thread_pieces.size()-2]->vertex() - _thread_pieces[_thread_pieces.size()-3]->vertex();
    edge_norm = curr_edge.norm();
    vertex_offsets[_thread_pieces.size()-3] += (curr_edge/edge_norm)*((edge_norm - _rest_length));
    projected_enough &= (abs(edge_norm-_rest_length) < max_norm_to_break);
    *//*

    Vector3d edge_before = _thread_pieces[2]->vertex() - _thread_pieces[1]->vertex();
    double edge_before_norm = edge_before.norm();
    Vector3d edge_after = _thread_pieces[2]->vertex() - _thread_pieces[3]->vertex();
    double edge_after_norm = edge_after.norm();
    vertex_offsets[2] = (edge_before/edge_before_norm) * (_rest_length-edge_before_norm) + 0.5*(edge_after/edge_after_norm) * (_rest_length-edge_after_norm);

    bool projected_enough = abs(edge_before_norm-_rest_length) < max_norm_to_break;

    edge_before = _thread_pieces[_thread_pieces.size()-3]->vertex() - _thread_pieces[_thread_pieces.size()-4].vertex();
    edge_before_norm = edge_before.norm();
    edge_after = _thread_pieces[_thread_pieces.size()-3]->vertex() - _thread_pieces[_thread_pieces.size()-2]->vertex();
    edge_after_norm = edge_after.norm();
    vertex_offsets[_thread_pieces.size()-3] = 0.5*(edge_before/edge_before_norm)*(_rest_length-edge_before_norm) + (edge_after/edge_after_norm) * (_rest_length-edge_after_norm);

    //projected_enough &= abs(edge_after_norm-_rest_length) < max_norm_to_break;

    #pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
    for (int edge_ind = 3; edge_ind < _thread_pieces.size()-4; edge_ind++)
    {
      Vector3d edge_before_curr = _thread_pieces[edge_ind]->vertex() - _thread_pieces[edge_ind-1].vertex();
      double edge_before_norm_curr = edge_before_curr.norm();
      Vector3d edge_after_curr = _thread_pieces[edge_ind]->vertex() - _thread_pieces[edge_ind+1]->vertex();
      double edge_after_norm_curr = edge_after_curr.norm();
      vertex_offsets[edge_ind] = 0.5* ((edge_before_curr/edge_before_norm_curr) * (_rest_length-edge_before_norm_curr) + (edge_after_curr/edge_after_norm_curr) * (_rest_length-edge_after_norm_curr));

      if (abs(edge_before_norm_curr-_rest_length) > max_norm_to_break)
        projected_enough = false;
    }

*/
 /*     if (vertex_offsets[_thread_pieces.size()-3].norm() > 3)
      {
        std::cout << "iter " << iter_num << "piece " << (_thread_pieces.size()-3) << " projection offset big " << vertex_offsets[_thread_pieces.size()-3]<< std::endl;
      }
      *//*
    //apply the actual offsets
    apply_vertex_offsets(vertex_offsets, true, projection_scale_factor, iter_num == (num_iters_project-1) || projected_enough);

    if (projected_enough)
      break;

  }


  //std::cout << "num projs: " << iter_num << std::endl;

}*/


void Thread::apply_vertex_offsets(vector<Vector3d>& offsets, bool skip_edge_cases, double step_size, bool update_frames)
{

//std::cout << "last angle before: " << _thread_pieces[_thread_pieces.size()-2]->angle_twist() << std::endl;
  if (!skip_edge_cases)
  {
    _thread_pieces[0]->offset_vertex(step_size*offsets[0]);
    _thread_pieces[1]->offset_vertex(step_size*offsets[1]);
    _thread_pieces[_thread_pieces.size()-2]->offset_vertex(step_size*offsets[_thread_pieces.size()-2]);
    _thread_pieces[_thread_pieces.size()-1]->offset_vertex(step_size*offsets[_thread_pieces.size()-1]);
  }



 // #pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
  for (int piece_ind = 2; piece_ind < (_thread_pieces.size()-2); piece_ind++)
  {
    _thread_pieces[piece_ind]->offset_vertex(step_size*offsets[piece_ind]);
  }



  if (update_frames)
  {
    _thread_pieces.front()->updateFrames_all();
  }

}

void Thread::apply_vertex_offsets_vectorized(const VectorXd& offsets, bool skip_edge_cases, double step_size, bool update_frames)
{
  if (!skip_edge_cases)
    {
      exit(0);
      /*
      _thread_pieces[0]->offset_vertex(step_size*offsets[0]);
      _thread_pieces[1]->offset_vertex(step_size*offsets[1]);
      _thread_pieces[_thread_pieces.size()-2]->offset_vertex(step_size*offsets[_thread_pieces.size()-2]);
      _thread_pieces[_thread_pieces.size()-1]->offset_vertex(step_size*offsets[_thread_pieces.size()-1]);
      */
    }

  // #pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
  for (int piece_ind = 2; piece_ind < (_thread_pieces.size()-2); piece_ind++)
    {
      Vector3d offset = offsets.segment<3>(3*piece_ind);
      _thread_pieces[piece_ind]->offset_vertex(step_size*offset);
    }

  if (update_frames)
    {
      _thread_pieces.front()->updateFrames_all();
    }
}


void Thread::apply_angle_twist_offsets(vector<double>& offsets, bool skip_edge_cases, double step_size)
{
 if (!skip_edge_cases)
  {
    exit(0);
    _thread_pieces[_thread_pieces.size()-2]->offset_angle_twist(step_size*offsets[_thread_pieces.size()-2]);
    _thread_pieces[_thread_pieces.size()-1]->offset_angle_twist(step_size*offsets[_thread_pieces.size()-1]);
  }

//  #pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
 for (int piece_ind = 1; piece_ind < _thread_pieces.size()-2; piece_ind++)
  {
    _thread_pieces[piece_ind]->offset_angle_twist(step_size*offsets[piece_ind]);
    _thread_pieces[piece_ind]->updateFrames_twistOnly();
  }

}

/*void Thread::save_angle_twists(vector<double>& angles)
{
  #pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
  for (int piece_ind=1; piece_ind < _thread_pieces.size()-2; piece_ind++)
  {
    angles[piece_ind] = _thread_pieces[piece_ind]->angle_twist() - _thread_pieces[piece_ind-1].angle_twist();
  }
}


void Thread::save_angle_twist_changes(vector<double>& start_angles, vector<double>& end_angles)
{
 // std::cout << "\nsaved angles\n";
  #pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
  for (int piece_ind=0; piece_ind < _thread_pieces.size()-2; piece_ind++)
  {
    _saved_last_theta_changes[piece_ind] = end_angles[piece_ind] - start_angles[piece_ind];
//    std::cout << _saved_last_theta_changes[piece_ind] << std::endl;
  }
//  std::cout << "\n";
}
*/

void Thread::save_thread_pieces()
{
  for (int piece_ind=0; piece_ind < _thread_pieces.size(); piece_ind++)
  {
    _thread_pieces_backup[piece_ind]->copyData(*_thread_pieces[piece_ind]);
  }

}

void Thread::restore_thread_pieces()
{
  for (int piece_ind=0; piece_ind < _thread_pieces_backup.size(); piece_ind++)
  {
    _thread_pieces[piece_ind]->copyData(*_thread_pieces_backup[piece_ind]);
  }
}


void Thread::save_thread_pieces(vector<ThreadPiece*>& to_save)
{
  for (int piece_ind=0; piece_ind < _thread_pieces.size(); piece_ind++)
  {
    to_save[piece_ind]->copyData(*_thread_pieces[piece_ind]);
  }

}

void Thread::save_thread_pieces_and_resize(vector<ThreadPiece*>& to_save)
{
  while (to_save.size() < _thread_pieces.size())
  {
    to_save.push_back(new ThreadPiece());
  }

  for (int i = to_save.size()-1; i >= (int)_thread_pieces.size(); i--)
  {
    delete to_save[i];
    to_save.pop_back();
  }
  
  save_thread_pieces(to_save);

}

void Thread::restore_thread_pieces(vector<ThreadPiece*>& to_restore)
{
  for (int piece_ind=0; piece_ind < to_restore.size(); piece_ind++)
  {
    _thread_pieces[piece_ind]->copyData(*to_restore[piece_ind]);
  }
  for (int piece_ind=to_restore.size()-1; piece_ind >= 0; piece_ind--)
  {
      delete to_restore[piece_ind];
      to_restore.pop_back();
  }
}

void Thread::restore_thread_pieces_and_resize(vector<ThreadPiece*>& to_restore)
{
  while (_thread_pieces.size() < to_restore.size())
  {
    _thread_pieces.push_back(new ThreadPiece());
    _thread_pieces_backup.push_back(new ThreadPiece());
  }

  for (int i = _thread_pieces.size()-1; i >= (int)to_restore.size(); i--)
  {
    delete _thread_pieces[i];
    delete _thread_pieces_backup[i];
    _thread_pieces.pop_back();
    _thread_pieces_backup.pop_back();
  }
  
  set_prev_next_pointers(_thread_pieces);
  set_prev_next_pointers(_thread_pieces_backup);

  restore_thread_pieces(to_restore);

}

void Thread::set_prev_next_pointers(vector<ThreadPiece*> pieces)
{
  for (int i=1; i < (int)pieces.size(); i++)
  {
    pieces[i]->set_prev(pieces[i-1]);
  }

  for (int i=0; i < (int)(pieces.size()-1); i++)
  {
    pieces[i]->set_next(pieces[i+1]);
  }



}


void Thread::save_angle_twists()
{
  for (int piece_ind=0; piece_ind < _thread_pieces.size()-1; piece_ind++)
  {
    _angle_twist_backup[piece_ind] = _thread_pieces[piece_ind]->angle_twist();
  }

}

void Thread::restore_angle_twists()
{
  for (int piece_ind=0; piece_ind < _angle_twist_backup.size()-1; piece_ind++)
  {
    _thread_pieces[piece_ind]->set_angle_twist(_angle_twist_backup[piece_ind]);
    _thread_pieces[piece_ind]->update_material_frame();
  }

}


void Thread::get_thread_data(vector<Vector3d>& points)
{
  points.resize(_thread_pieces.size());
  for (int piece_ind=0; piece_ind < _thread_pieces.size(); piece_ind++)
  {
    points[piece_ind] = _thread_pieces[piece_ind]->vertex();
  }
}

void Thread::get_thread_data(vector<Vector3d>& points, vector<double>& twist_angles)
{
  points.resize(_thread_pieces.size());
  twist_angles.resize(_thread_pieces.size());
  for (int piece_ind=0; piece_ind < _thread_pieces.size(); piece_ind++)
  {
    points[piece_ind] = _thread_pieces[piece_ind]->vertex();
    twist_angles[piece_ind] = _thread_pieces[piece_ind]->angle_twist();
  }
#ifdef ISOTROPIC
  twist_angles.back() = twist_angles[twist_angles.size()-2] * (_total_length/(_total_length-end_rest_length()));
#else
	cout << "Internal error: Thread::get_thread_data() : wrong end twist angle for anisotropic case." << endl;
#endif
}

void Thread::get_thread_data(vector<Vector3d>& points, vector<double>& twist_angles, vector<double>& rest_lengths)
{
  points.resize(_thread_pieces.size());
  twist_angles.resize(_thread_pieces.size());
  rest_lengths.resize(_thread_pieces.size());
  for (int piece_ind=0; piece_ind < _thread_pieces.size(); piece_ind++)
  {
    points[piece_ind] = _thread_pieces[piece_ind]->vertex();
    twist_angles[piece_ind] = _thread_pieces[piece_ind]->angle_twist();
    rest_lengths[piece_ind] = _thread_pieces[piece_ind]->rest_length();
  }
#ifdef ISOTROPIC
  twist_angles.back() = twist_angles[twist_angles.size()-2] * (_total_length/(_total_length-end_rest_length()));
#else
	cout << "Internal error: Thread::get_thread_data() : wrong end twist angle for anisotropic case." << endl;
#endif
}

void Thread::get_thread_data(vector<Vector3d>& points, vector<Matrix3d>& material_frames)
{
  points.resize(_thread_pieces.size());
  material_frames.resize(_thread_pieces.size());
  for (int piece_ind=0; piece_ind < _thread_pieces.size(); piece_ind++)
  {
    points[piece_ind] = _thread_pieces[piece_ind]->vertex();
    material_frames[piece_ind] = _thread_pieces[piece_ind]->material_frame();
  }
}

void Thread::get_thread_data(vector<Matrix3d>& bishop_frames)
{
  bishop_frames.resize(_thread_pieces.size());
  for (int piece_ind=0; piece_ind < _thread_pieces.size(); piece_ind++)
  {
    bishop_frames[piece_ind] = _thread_pieces[piece_ind]->bishop_frame();
  }
}

void Thread::get_thread_data(vector<Vector3d>& points, vector<double>& twist_angles, vector<Matrix3d>& material_frames)
{
  points.resize(_thread_pieces.size());
  twist_angles.resize(_thread_pieces.size());
  material_frames.resize(_thread_pieces.size());
  for (int piece_ind=0; piece_ind < _thread_pieces.size(); piece_ind++)
  {
    points[piece_ind] = _thread_pieces[piece_ind]->vertex();
    twist_angles[piece_ind] = _thread_pieces[piece_ind]->angle_twist();
    material_frames[piece_ind] = _thread_pieces[piece_ind]->material_frame();
  }
#ifdef ISOTROPIC
  twist_angles.back() = twist_angles[twist_angles.size()-2] * (_total_length/(_total_length-end_rest_length()));
#else
	cout << "Internal error: Thread::get_thread_data() : wrong end twist angle for anisotropic case." << endl;
#endif
}

void Thread::get_thread_data(vector<double>& lengths, vector<double>& edge_norms)
{
  lengths.resize(_thread_pieces.size());
  edge_norms.resize(_thread_pieces.size());
  for (int piece_ind=0; piece_ind < _thread_pieces.size(); piece_ind++)
  {
    lengths[piece_ind] = _thread_pieces[piece_ind]->rest_length();
    edge_norms[piece_ind] = _thread_pieces[piece_ind]->edge_norm();
  }
}

void Thread::set_all_angles_zero()
{
    for (int piece_ind=0; piece_ind < _thread_pieces.size(); piece_ind++)
    {
        _thread_pieces[piece_ind]->set_angle_twist(0);
    }
} 

void Thread::set_all_pieces_mythread()
{
  for (int piece_ind=0; piece_ind < _thread_pieces.size(); piece_ind++)
  {
    _thread_pieces[piece_ind]->set_my_thread(this);
  }

  for (int piece_ind=0; piece_ind < _thread_pieces_backup.size(); piece_ind++)
  {
    _thread_pieces_backup[piece_ind]->set_my_thread(this);
  }
}

bool Thread::is_consistent()
{
  bool toRtn = true;
  for (int piece_ind=0; piece_ind < _thread_pieces.size()-1; piece_ind++)
  {
    toRtn &= _thread_pieces[piece_ind]->is_material_frame_consistent();
  }

  return toRtn;
}

double Thread::calculate_holonomy()
{
/*
  vector<ThreadPiece> pieces_for_calc(4);
  pieces_for_calc[0] = ThreadPiece(_thread_pieces[0]->vertex(), 0.0);
  pieces_for_calc[1] = ThreadPiece(_thread_pieces[1]->vertex(), 0.0);
  //Vector3d edge_first2 = pieces_for_calc[1]->vertex() - pieces_for_calc[0]->vertex();
  //Vector3d edge_last2 = _thread_pieces[_thread_pieces.size()-1]->vertex() - _thread_pieces[_thread_pieces.size()-2]->vertex();
  pieces_for_calc[2] = ThreadPiece(pieces_for_calc[1]->vertex()+edge_first2, 0.0);
  pieces_for_calc[3] = ThreadPiece(_thread_pieces[_thread_pieces.size()-1]->vertex()-pieces_for_calc[2].vertex(), 0.0);
  for (int i=1; i < pieces_for_calc.size(); i++)
  {
    pieces_for_calc[i].set_prev(&pieces_for_calc[i-1]);
  }
  for (int i=0; i < pieces_for_calc.size()-1; i++)
  {
    pieces_for_calc[i].set_next(&pieces_for_calc[i+1]);
  }

  pieces_for_calc.front().set_bishop_frame(_thread_pieces.front()->bishop_frame());
  pieces_for_calc.front().set_material_frame(_thread_pieces.front()->bishop_frame());

  Matrix3d start_rot = pieces_for_calc.front().bishop_frame();
  Vector3d start_pos = pieces_for_calc.front().vertex();


  Matrix3d end_rot = _thread_pieces[_thread_pieces.size()-2]->bishop_frame();

  pieces_for_calc.front().initializeFrames();

  //to fix tangent, set 2nd point as well
  pieces_for_calc.front().updateFrames_firstpiece();
  //pieces_for_calc.back().set_material_frame(end_rot);

  //pieces_for_calc.back().updateFrames_lastpiece();

*/
  Vector3d edge_first2 = _thread_pieces[1]->vertex() - _thread_pieces.front()->vertex();
  Vector3d edge_last2 = _thread_pieces[_thread_pieces.size()-1]->vertex() - _thread_pieces[_thread_pieces.size()-2]->vertex();

  Matrix3d no_twist_bishop;

  Vector3d to_twist_around = edge_first2.cross(edge_last2)/((_thread_pieces.front()->rest_length())*(_thread_pieces[_thread_pieces.size()-2]->rest_length()) + edge_first2.dot(edge_last2));	// check
  to_twist_around.normalize();
  if (isnan(to_twist_around(0)))
  {
    no_twist_bishop = _thread_pieces.front()->bishop_frame();
  } else {
  //double ang_to_twist = asin(to_twist_around.norm());
  double ang_to_twist = edge_first2.dot(edge_last2)/(edge_first2.norm()*edge_last2.norm());
  ang_to_twist = max( min ( ang_to_twist, 1.0), -1.0);
  ang_to_twist = acos(ang_to_twist);


  no_twist_bishop = Eigen::AngleAxisd(ang_to_twist, to_twist_around.normalized())*_thread_pieces.front()->bishop_frame();
  }

  double holonomy = angle_mismatch(_thread_pieces[_thread_pieces.size()-2]->bishop_frame(), no_twist_bishop);
//  double holonomy = angle_mismatch(_thread_pieces[_thread_pieces.size()-2]->bishop_frame(), pieces_for_calc[pieces_for_calc.size()-2].bishop_frame());

  return holonomy;
}

void Thread::print_vertices()
{
  std::cout << "vertices:\n";
  for (int piece_ind=0; piece_ind < _thread_pieces.size(); piece_ind++)
  {
    std::cout << _thread_pieces[piece_ind]->vertex().transpose() << std::endl;
  }
  std::cout << std::endl;
}

void Thread::restore_constraints(const Vector3d& start_pos, const Matrix3d& start_rot, const Vector3d& end_pos, const Matrix3d& end_rot) 
{
  set_start_constraint(start_pos, start_rot, false);
  set_end_constraint(end_pos, end_rot, false);

}
void Thread::set_constraints(const Vector3d& start_pos, const Matrix3d& start_rot, const Vector3d& end_pos, const Matrix3d& end_rot)
{
  set_start_constraint(start_pos, start_rot);
  set_end_constraint(end_pos, end_rot);
}

void Thread::set_start_constraint(const Vector3d& start_pos, const Matrix3d& start_rot, bool backup)
{
  if (backup) { 
    _start_pos_backup = this->start_pos();
    _start_rot_backup = this->start_rot();
  }

  _thread_pieces.front()->set_vertex(start_pos);
  //_start_rot = start_rot;
  _thread_pieces.front()->set_bishop_frame(start_rot);
  _thread_pieces.front()->set_material_frame(start_rot);

  //to fix tangent, set 2nd point as well
  Vector3d fixed_point = start_pos + start_rot.col(0)*start_rest_length();
  _thread_pieces[1]->set_vertex(fixed_point);
  _thread_pieces.front()->updateFrames_firstpiece();
}


//assumes bishop frame for end is already calculated
void Thread::set_end_constraint(const Vector3d& end_pos, const Matrix3d& end_rot, bool backup)
{
  if (backup) { 
  _end_pos_backup = this->end_pos();
  _end_rot_backup = this->end_rot();
  }

  _thread_pieces.back()->set_vertex(end_pos);
  //_end_rot = end_rot;
  _thread_pieces.back()->set_material_frame(end_rot);

  //to fix tangent, set 2nd to last point as well
  Vector3d fixed_point = end_pos - end_rot.col(0)*end_rest_length();
  _thread_pieces[_thread_pieces.size()-2]->set_vertex(fixed_point);

  _thread_pieces.back()->updateFrames_lastpiece();
}

void Thread::set_start_constraint_nearEnd(Vector3d& start_pos, Matrix3d& start_rot)
{
  Vector3d start_pos_movement = start_pos - this->start_pos();
  start_pos += (vertex_at_ind(1) - this->start_pos()) - (start_rot.col(0)*start_rest_length());
  set_start_constraint(start_pos, start_rot);
  //minimize_energy();
}

void Thread::set_end_constraint_nearEnd(Vector3d& end_pos, Matrix3d& end_rot)
{
  Vector3d end_pos_movement = end_pos - this->end_pos();
  end_pos += (vertex_at_ind(_thread_pieces.size()-2) - this->end_pos()) + (end_rot.col(0)*end_rest_length());
  set_end_constraint(end_pos, end_rot);
  //minimize_energy();
}

void Thread::set_constraints_nearEnds(Vector3d& start_pos, Matrix3d& start_rot, Vector3d& end_pos, Matrix3d& end_rot)
{
  Vector3d start_pos_movement = start_pos - this->start_pos();
  Vector3d end_pos_movement = end_pos - this->end_pos();

  //make it so the position is at 2nd and 2nd to last vertices
  //thus, account for how the frame rotation affected position of those vertices
  start_pos += (vertex_at_ind(1) - this->start_pos()) - (start_rot.col(0)*start_rest_length());
  end_pos += (vertex_at_ind(_thread_pieces.size()-2) - this->end_pos()) + (end_rot.col(0)*end_rest_length());

  //check to make sure we aren't trying to go beyond the max length
  Vector3d pointA = start_pos+start_rot.col(0)*start_rest_length();
  Vector3d pointB = end_pos-end_rot.col(0)*end_rest_length();

  //if so, correct by moving back into acceptable region
  Vector3d entire_length_vector = pointB - pointA;
  double too_long_by = (entire_length_vector).norm() - (total_length() - start_rest_length() - end_rest_length()) + LENGTH_THRESHHOLD;
  if (too_long_by > 0)
  {
    entire_length_vector.normalize();
    Vector3d start_motion_indir = start_pos_movement.dot(entire_length_vector)*entire_length_vector;
    Vector3d end_motion_indir = end_pos_movement.dot(entire_length_vector)*entire_length_vector;

    double total_movement = start_motion_indir.norm() + end_motion_indir.norm()+1e-5;

    start_pos -= start_motion_indir*(too_long_by/total_movement);
    end_pos -= end_motion_indir*(too_long_by/total_movement);

    //due to numerical reasons, this sometimes seems to fail. if so, just move the end back in
    unviolate_total_length_constraint();
  }
  set_constraints(start_pos, start_rot, end_pos, end_rot);
  minimize_energy();
}

// same as set_constraints_nearEnds except that the constraints is set at the end and not at the near end.
void Thread::set_constraints_check(Vector3d& start_pos, Matrix3d& start_rot, Vector3d& end_pos, Matrix3d& end_rot, bool minimize)
{
  Vector3d start_pos_movement = start_pos - this->start_pos();
  Vector3d end_pos_movement = end_pos - this->end_pos();

  //check to make sure we aren't trying to go beyond the max length
  Vector3d pointA = start_pos+start_rot.col(0)*start_rest_length();
  Vector3d pointB = end_pos-end_rot.col(0)*end_rest_length();

  //if so, correct by moving back into acceptable region
  Vector3d entire_length_vector = pointB - pointA;
  double too_long_by = (entire_length_vector).norm() - (total_length() - start_rest_length() - end_rest_length()) + LENGTH_THRESHHOLD;
  if (too_long_by > 0)
  {
    entire_length_vector.normalize();
    Vector3d start_motion_indir = start_pos_movement.dot(entire_length_vector)*entire_length_vector;
    Vector3d end_motion_indir = end_pos_movement.dot(entire_length_vector)*entire_length_vector;

    double total_movement = start_motion_indir.norm() + end_motion_indir.norm()+1e-5;

    start_pos -= start_motion_indir*(too_long_by/total_movement);
    end_pos -= end_motion_indir*(too_long_by/total_movement);

    //due to numerical reasons, this sometimes seems to fail. if so, just move the end back in
    unviolate_total_length_constraint();
  }
  set_constraints(start_pos, start_rot, end_pos, end_rot);
  if (minimize)
  	minimize_energy();
}

bool Thread::check_fix_positions(Vector3d& start_pos, Matrix3d& start_rot, Vector3d& end_pos, Matrix3d& end_rot)
{
  Vector3d start_pos_movement = start_pos - this->start_pos();
  Vector3d end_pos_movement = end_pos - this->end_pos();

  //check to make sure we aren't trying to go beyond the max length
  Vector3d pointA = start_pos+start_rot.col(0)*start_rest_length();
  Vector3d pointB = end_pos-end_rot.col(0)*end_rest_length();

  //if so, correct by moving back into acceptable region
  Vector3d entire_length_vector = pointB - pointA;
  double too_long_by = (entire_length_vector).norm() - (total_length() - start_rest_length() - end_rest_length()) + LENGTH_THRESHHOLD;
  if (too_long_by > 0)
  {
    entire_length_vector.normalize();
    Vector3d start_motion_indir = start_pos_movement.dot(entire_length_vector)*entire_length_vector;
    Vector3d end_motion_indir = end_pos_movement.dot(entire_length_vector)*entire_length_vector;

    double total_movement = start_motion_indir.norm() + end_motion_indir.norm()+1e-5;

    start_pos -= start_motion_indir*(too_long_by/total_movement);
    end_pos -= end_motion_indir*(too_long_by/total_movement);

    //due to numerical reasons, this sometimes seems to fail. if so, just move the end back in
    unviolate_total_length_constraint();
  }
	entire_length_vector = (end_pos-end_rot.col(0)*end_rest_length()) - (start_pos+start_rot.col(0)*start_rest_length());
  too_long_by = (entire_length_vector).norm() - (total_length() - start_rest_length() - end_rest_length()) + LENGTH_THRESHHOLD;
  
  return (too_long_by > 1e-3);
}

//bool Thread::check_fix_positions(double start_fix_fraction)
//{
//  //check to make sure we aren't trying to go beyond the max length
//  Vector3d pointA = this->start_pos()+this->start_rot().col(0)*start_rest_length();
//  Vector3d pointB= this->end_pos()-this->end_rot().col(0)*end_rest_length();

//  //if so, correct by moving back into acceptable region
//  Vector3d entire_length_vector = pointB - pointA;
//  double too_long_by = (entire_length_vector).norm() - (total_length() - start_rest_length() - end_rest_length()) + LENGTH_THRESHHOLD;
//  if (too_long_by > 0)
//  {
//    entire_length_vector.normalize();
//		set_constraints(this->start_pos() + start_fix_fraction*entire_length_vector*too_long_by, this->start_rot(),
//										this->end_pos() - (1.0-start_fix_fraction)*entire_length_vector*too_long_by, this->end_rot());
//  }  
//  pointA = this->start_pos()+this->start_rot().col(0)*start_rest_length();
//  pointB= this->end_pos()-this->end_rot().col(0)*end_rest_length();
//  entire_length_vector = pointB - pointA;
//  too_long_by = (entire_length_vector).norm() - (total_length() - start_rest_length() - end_rest_length()) + LENGTH_THRESHHOLD;
//  
//  return (too_long_by > 1e-3);
//}

void Thread::rotate_end_by(double degrees)
{
#ifdef ISOTROPIC
  _thread_pieces[_thread_pieces.size()-2]->offset_angle_twist(degrees);
  _thread_pieces[_thread_pieces.size()-2]->update_material_frame();
	set_end_constraint(this->end_pos(), this->end_rot());
 // _thread_pieces.front()->initializeFrames();
#else
  Matrix3d new_end_rot = (Eigen::AngleAxisd(degrees, this->end_rot().col(0).normalized()))*this->end_rot();
  set_end_constraint(this->end_pos(), new_end_rot);
#endif
}


//applies motion to END
void Thread::apply_motion(Frame_Motion& motion)
{
  Vector3d end_pos = this->end_pos();;
  Matrix3d end_rot = this->end_rot();
  motion.applyMotion(end_pos, end_rot);
  set_end_constraint(end_pos, end_rot);
  //minimize_energy();
}

//applies motion to start and end
void Thread::apply_motion(Two_Motions& motion)
{
  Vector3d start_pos = this->start_pos();
  Matrix3d start_rot = this->start_rot();
  Vector3d end_pos = this->end_pos();
  Matrix3d end_rot = this->end_rot();
  motion._start.applyMotion(start_pos, start_rot);
  motion._end.applyMotion(end_pos, end_rot);

  set_constraints(start_pos, start_rot, end_pos, end_rot);
  //minimize_energy();
}

//applies motion to END
void Thread::apply_motion_nearEnds(Frame_Motion& motion, bool mini_energy)
{
  Vector3d end_pos = this->end_pos();
  Matrix3d end_rot = this->end_rot();
  motion.applyMotion(end_pos, end_rot);
  //now account for translation that occurs because of rotation
  end_pos += (vertex_at_ind(_thread_pieces.size()-2)+motion._pos_movement) - (end_pos-end_rot.col(0).normalized()*end_rest_length());
  set_end_constraint(end_pos, end_rot);
  unviolate_total_length_constraint();
  //if (mini_energy)
  //  minimize_energy();
}

void Thread::apply_motion_nearEnds(Two_Motions& motion, bool mini_energy)
{
  Vector3d start_pos = this->start_pos();
  Matrix3d start_rot = this->start_rot();
  Vector3d end_pos = this->end_pos();
  Matrix3d end_rot = this->end_rot();

  motion._start.applyMotion(start_pos, start_rot);
  motion._end.applyMotion(end_pos, end_rot);

  //make it so the motion is at 2nd and 2nd to last vertices
  //thus, account for how the frame rotation affected position of those vertices
  start_pos += (vertex_at_ind(1)+motion._start._pos_movement) - (start_pos+start_rot.col(0)*start_rest_length());
  end_pos += (vertex_at_ind(_thread_pieces.size()-2)+motion._end._pos_movement) - (end_pos-end_rot.col(0)*end_rest_length());

  //check to make sure we aren't trying to go beyond the max length
  Vector3d pointA = start_pos+start_rot.col(0)*start_rest_length();
  Vector3d pointB= end_pos-end_rot.col(0)*end_rest_length();

  //if so, correct by moving back into acceptable region
  Vector3d entire_length_vector = pointB - pointA;
  double too_long_by = (entire_length_vector).norm() - (total_length() - start_rest_length() - end_rest_length()) + LENGTH_THRESHHOLD;
  if (too_long_by > 0)
  {
    entire_length_vector.normalize();
    Vector3d start_motion_indir = motion._start._pos_movement.dot(entire_length_vector)*entire_length_vector;
    Vector3d end_motion_indir = motion._end._pos_movement.dot(entire_length_vector)*entire_length_vector;

    double total_movement = start_motion_indir.norm() + end_motion_indir.norm()+1e-5;

    start_pos -= start_motion_indir*(too_long_by/total_movement);
    end_pos -= end_motion_indir*(too_long_by/total_movement);

    //due to numerical reasons, this sometimes seems to fail. if so, just move the end back in
    unviolate_total_length_constraint();
  }

  /*
  pointA = start_pos+start_rot.col(0)*_rest_length;
  pointB= end_pos-end_rot.col(0)*_rest_length;
  entire_length_vector = pointB - pointA;
  too_long_by = (entire_length_vector).norm() - (total_length() - 2.0*rest_length()) + LENGTH_THRESHHOLD;
  std::cout << "too long by " << too_long_by << std::endl;
  */
  // create a backup of the current thread


  set_constraints(start_pos, start_rot, end_pos, end_rot);
  //if (mini_energy)
  //  minimize_energy();


}


void Thread::unviolate_total_length_constraint()
{
  Vector3d pointA = this->start_pos()+this->start_rot().col(0)*start_rest_length();
  Vector3d pointB= this->end_pos()-this->end_rot().col(0)*end_rest_length();
  Vector3d entire_length_vector = pointB - pointA;
  double too_long_by = (entire_length_vector).norm() - (total_length() - start_rest_length() - end_rest_length()) + LENGTH_THRESHHOLD;

  if (too_long_by > 0)
  {
    entire_length_vector.normalize();
    set_end_constraint(this->end_pos() -entire_length_vector*too_long_by, this->end_rot());
  }


  pointA = this->start_pos()+this->start_rot().col(0)*start_rest_length();
  pointB= this->end_pos()-this->end_rot().col(0)*end_rest_length();
  entire_length_vector = pointB - pointA;
  too_long_by = (entire_length_vector).norm() - (total_length() - start_rest_length() - end_rest_length()) + LENGTH_THRESHHOLD;

  //std::cout << "total length: " << entire_length_vector.norm() << std::endl;
  

}


void Thread::copy_data_from_vector(VectorXd& toCopy)
{
  
  for (int piece_ind = 0; piece_ind < _thread_pieces.size(); piece_ind++)
  {
    //std::cout << "before vertex: " << _thread_pieces[piece_ind]->vertex().transpose() << "\t\t";
    _thread_pieces[piece_ind]->set_vertex(toCopy.segment(piece_ind*3,3));
    //std::cout << "after vertex: " << _thread_pieces[piece_ind]->vertex().transpose() << std::endl;
  }

  _thread_pieces[0]->update_edge();
  _thread_pieces[1]->update_edge();
  _thread_pieces[0]->update_bishop_frame_firstPiece();
  _thread_pieces[0]->update_material_frame();
  _thread_pieces[0]->initializeFrames();
  set_start_constraint(_thread_pieces.front()->vertex(), this->start_rot());
  
  set_end_constraint(_thread_pieces.back()->vertex(), this->end_rot());
  _thread_pieces[_thread_pieces.size()-2]->set_angle_twist(toCopy(toCopy.rows()-1));
  _thread_pieces[_thread_pieces.size()-2]->update_material_frame();
  set_end_constraint(_thread_pieces.back()->vertex(), this->end_rot());
}

void Thread::applyControl(const VectorXd& u)
{
  double max_ang_start = max( max(abs(u(3)), abs(u(4))), abs(u(5)));
  double max_ang_end = max( max(abs(u(9)), abs(u(10))), abs(u(11)));
  double max_ang = max(max_ang_start, max_ang_end);

  int number_steps = max ((int)ceil(max_ang / (M_PI/4.0)), 1);
  VectorXd u_for_translation = u/((double)number_steps);

	Two_Motions* to_Move = new Two_Motions();

  Vector3d translation_start;
  translation_start << u_for_translation(0), u_for_translation(1), u_for_translation(2);
  Vector3d translation_end;
  translation_end << u_for_translation(6), u_for_translation(7), u_for_translation(8);
	to_Move->_start._pos_movement = translation_start;
  to_Move->_end._pos_movement = translation_end;

  Matrix3d rotation_start;
  rotation_from_euler_angles(rotation_start, u(3), u(4), u(5));
  Matrix3d rotation_end;
  rotation_from_euler_angles(rotation_end, u(9), u(10), u(11));

  Quaterniond quat_rotation_start(rotation_start);
  Quaterniond quat_rotation_end(rotation_end);

  to_Move->_start._frame_rotation = Quaterniond::Identity().slerp(1.0/(double)number_steps, quat_rotation_start);
  to_Move->_end._frame_rotation = Quaterniond::Identity().slerp(1.0/(double)number_steps, quat_rotation_end);

  for (int i=0; i < number_steps; i++)
  {
    apply_motion_nearEnds(*to_Move);
  }
}

void Thread::getState(VectorXd& state)
{
  const int _num_pieces = num_pieces();
  state.resize(6*_num_pieces-3+1);
  for (int piece_ind=0; piece_ind < _num_pieces; piece_ind++)
  {
    state.segment(piece_ind*3, 3) = vertex_at_ind(piece_ind);
  }
  for (int piece_ind=0; piece_ind < _num_pieces-1; piece_ind++)
  {
    state.segment(3*_num_pieces + piece_ind*3, 3) = edge_at_ind(piece_ind);
  }
  state(6*_num_pieces-3) = end_angle();
}

void Thread::set_coeffs_normalized(double bend_coeff, double twist_coeff, double grav_coeff)
{
  double norm_factor = 1.0/sqrt(pow(bend_coeff,2) + pow(twist_coeff,2) + pow(grav_coeff,2));
  _thread_pieces.front()->set_bend_coeff(bend_coeff*norm_factor);
  _thread_pieces.front()->set_twist_coeff(twist_coeff*norm_factor);
  _thread_pieces.front()->set_grav_coeff(grav_coeff*norm_factor);
}

void Thread::set_coeffs_normalized(const Matrix2d& bend_matrix, double twist_coeff, double grav_coeff)
{
  double norm_factor = 1.0/sqrt(bend_matrix.determinant() + pow(twist_coeff,2) + pow(grav_coeff,2));
  _thread_pieces.front()->set_bend_matrix(bend_matrix*norm_factor);
  _thread_pieces.front()->set_twist_coeff(twist_coeff*norm_factor);
  _thread_pieces.front()->set_grav_coeff(grav_coeff*norm_factor);
}



void Thread::evaluate_twist_scores(vector<double>& twist_to_try, vector<double>& twist_scores, vector<double>& angle_start, vector<double>& angle_end)
{
  twist_scores.resize(twist_to_try.size());
  angle_start.resize(twist_to_try.size());
  angle_end.resize(twist_to_try.size());

  vector<Vector3d> orig_pts;
  vector<double> orig_angls;
  vector<Vector3d> new_pts;
  vector<double> new_angls;
  
  double holonomy = calculate_holonomy();


  get_thread_data(orig_pts, orig_angls);

  vector<ThreadPiece*> backups;

  save_thread_pieces_and_resize(backups);

  for (int i=0; i < twist_to_try.size(); i++)
  {
    _thread_pieces[_thread_pieces.size()-2]->set_angle_twist(twist_to_try[i]-holonomy);
    angle_start[i] = twist_to_try[i]-holonomy;
    minimize_energy_twist_angles();
    _thread_pieces.front()->initializeFrames();
   

    project_length_constraint();
  for (int k=0; k < 200; k++)
  {
  minimize_energy();
  }
    //one_step_project(1.0, false);
    get_thread_data(new_pts, new_angls);
    double this_score = calculate_vector_norm_avg(orig_pts, new_pts);
    
    twist_scores[i] = this_score;
    angle_end[i] = this->end_angle();

    restore_thread_pieces(backups);
  }

  //minimize_energy_twist_angles();
  //minimize_energy();
}


void Thread::set_twist_and_minimize(double twist, vector<Vector3d>& orig_pts)
{
  vector<Vector3d> new_pts;
  vector<double> new_angls;
  double holonomy = calculate_holonomy();
  _thread_pieces[_thread_pieces.size()-2]->set_angle_twist(twist-holonomy);
  minimize_energy_twist_angles();
  _thread_pieces.front()->initializeFrames();

  project_length_constraint();
  for (int i=0; i < 200; i++)
  {
  minimize_energy();
  }
    get_thread_data(new_pts, new_angls);
    double this_score = calculate_vector_norm_avg(orig_pts, new_pts);
    std::cout << "score: " << this_score << std::endl;
}



Thread& Thread::operator=(const Thread& rhs)
{
  world = rhs.world;
  //_thread_pieces.resize(rhs._thread_pieces.size());

	for (int piece_ind=0; piece_ind < _thread_pieces.size(); piece_ind++)
	{
		delete _thread_pieces[piece_ind];
	}
	_thread_pieces.clear();

	for (int piece_ind=0; piece_ind < _thread_pieces_backup.size(); piece_ind++)
	{
		delete _thread_pieces_backup[piece_ind];
	}
	_thread_pieces_backup.clear();

  _thread_pieces.resize(rhs._thread_pieces.size());
 _thread_pieces_backup.resize(rhs._thread_pieces_backup.size());
  _angle_twist_backup.resize(rhs._thread_pieces.size());
  for (int piece_ind =0; piece_ind < rhs._thread_pieces.size(); piece_ind++)
  {
    //_thread_pieces.push_back(rhs._thread_pieces[piece_ind]);
    _thread_pieces[piece_ind] = new ThreadPiece(*rhs._thread_pieces[piece_ind], this);
  }

  for (int i=1; i < _thread_pieces.size(); i++)
  {
    _thread_pieces[i]->set_prev(_thread_pieces[i-1]);
  }

  for (int i=0; i < _thread_pieces.size()-1; i++)
  {
    _thread_pieces[i]->set_next(_thread_pieces[i+1]);
  }

	//setup backups
  for (int piece_ind=0; piece_ind < rhs._thread_pieces_backup.size(); piece_ind++)
  {
    _thread_pieces_backup[piece_ind] = new ThreadPiece(*rhs._thread_pieces_backup[piece_ind], this);
  }

  for (int i=1; i < _thread_pieces_backup.size(); i++)
  {
    _thread_pieces_backup[i]->set_prev(_thread_pieces_backup[i-1]);
  }

  for (int i=0; i < _thread_pieces_backup.size()-1; i++)
  {
    _thread_pieces_backup[i]->set_next(_thread_pieces_backup[i+1]);
  }



  _thread_pieces.front()->set_bishop_frame(rhs.start_rot());
  _thread_pieces.front()->set_material_frame(rhs.start_rot());

  _thread_pieces.front()->initializeFrames();

  //_saved_last_theta_changes.resize(_thread_pieces.size());

  Matrix3d start_rot = rhs.start_rot();
  Matrix3d end_rot = rhs.end_rot();
  Vector3d start_pos = rhs.start_pos();
  Vector3d end_pos = rhs.end_pos();

  set_constraints(start_pos, start_rot, end_pos, end_rot);

  _total_length = rhs._total_length;
  //project_length_constraint();

/*
  Matrix3d end_bishop = rhs._thread_pieces[_thread_pieces.size()-2]->bishop_frame();
  double end_angle = rhs._thread_pieces[_thread_pieces.size()-2]->angle_twist();
  Matrix3d end_rot_calculated = Eigen::AngleAxisd(end_angle, end_bishop.col(0).normalized())*end_bishop;
  set_end_constraint(end_pos, end_rot_calculated);
 */

  //set_end_constraint(vertices.back(), end_rot);

  set_all_pieces_mythread();

  return *this;

}

void Thread::setWorld(World* w) {
	world = w;
}

void Thread::add_thread_to_env(Thread* thread) {  // check
	threads_in_env.push_back(thread);
}

void Thread::clear_threads_in_env() {
	threads_in_env.clear();
}

int findInvalidate(vector<ThreadPiece*> &v, ThreadPiece* e) {
	int i;
	for (i=0; i<v.size() && v[i]!=e; i++) {}
	if (i==v.size())
		return -1;
	v[i] = NULL;
	return i;
}
