#include "thread_minenergy.h"


Thread::Thread(double* curvature, double* torsion, double* length, int numPieces, Vector3d* positions, Vector3d* tangents)
{

	if (numPieces < 1)
	{
		std::cerr << "requires at least one piece" << std::endl;
		return;
	}

	_length = length[0];
	for (int i=1; i < numPieces; i++)
	{
		_length += length[i];
	}

	if ( (positions[1] - positions[0]).norm() > _length)
	{
		std::cerr << "the length of the thread is less than the distance between positions" << std::endl;
    exit(0);
		return;
	}


	//we want to normalize the length of the Thread, so each piece is in "canonical" form
	threadList = new ThreadPiece(curvature[0], torsion[0], length[0]/_length);
	ThreadPiece* threadCur = threadList;
	for (int i=1; i < numPieces; i++)
	{
		ThreadPiece* threadNext = new ThreadPiece(curvature[i], torsion[i], length[i]/_length);
		threadCur->addSegmentAfter(threadNext);
		threadCur = threadNext;
	}


  setConstraints(positions, tangents);
	
	_angle_first_rot = 0.0;

  turnOffNoise();
  initEnergyParams();

	opt_params.orig_params_each_piece = NULL;
}

//generate random thread with a start position and tangent, and total length
Thread::Thread(double length, Vector3d& startPos, Vector3d& startTan)
{
  int numPieces = 2; 
  double sampledRadius = DBL_MAX;
  double randX, randY, randZ;
  while (sampledRadius >= (length*length))
  {
    randX = 2.0*randomNumUnit()*length - length;
    randY = 2.0*randomNumUnit()*length - length;
    randZ = 2.0*randomNumUnit()*length - length;
    sampledRadius = randX*randX + randY*randY + randZ*randZ;
  }
  Vector3d positions[2];
  positions[0] = startPos;
  positions[1](0) = randX+startPos(0);
  positions[1](1) = randY+startPos(1);
  positions[1](2) = randZ+startPos(2);
   

  //sample tan randomly
  Vector3d tangents[2];
  tangents[0] = startTan;
  tangents[1](0) = randomNumUnit() - 0.5;
  tangents[1](1) = randomNumUnit() - 0.5;
  tangents[1](2) = randomNumUnit() - 0.5;
  tangents[1].normalize();



  //random params for init - needed to seed the energy minimizer
	double curvature[2] = {randomNumUnit()*RANDOM_THREAD_MAX_CURVATURE_INIT, randomNumUnit()*RANDOM_THREAD_MAX_CURVATURE_INIT};
	double torsion[2] = {randomNumUnit()*RANDOM_THREAD_MAX_TORSION_INIT, randomNumUnit()*RANDOM_THREAD_MAX_TORSION_INIT};
	double lengths[2] = {length/2.0, length/2.0};


	_length = length;

	//we want to normalize the length of the Thread, so each piece is in "canonical" form
	threadList = new ThreadPiece(curvature[0], torsion[0], lengths[0]/_length);
	ThreadPiece* threadCur = threadList;
	for (int i=1; i < numPieces; i++)
	{
		ThreadPiece* threadNext = new ThreadPiece(curvature[i], torsion[i], lengths[i]/_length);
		threadCur->addSegmentAfter(threadNext);
		threadCur = threadNext;
	}


  setConstraints(positions, tangents);
	
	_angle_first_rot = 0.0;

	opt_params.orig_params_each_piece = NULL;

  turnOffNoise();
  initEnergyParams();
}


Thread::Thread(const Thread* toCopy)
{
  _angle_first_rot = toCopy->_angle_first_rot;
  _length = toCopy->_length;
  _positions[0] = toCopy->_positions[0];
  _positions[1] = toCopy->_positions[1];
  _tangents[0] = toCopy->_tangents[0];
  _tangents[1] = toCopy->_tangents[1];
  _transform_to_start = toCopy->_transform_to_start;
  _translate_to_start = toCopy->_translate_to_start;
  _transform_to_unit = toCopy->_transform_to_unit;

  ThreadPiece* toCopyPiece = toCopy->threadList->_next_segment;
  threadList = new ThreadPiece(toCopy->threadList);
  ThreadPiece* copiedPiece = threadList;
  while (toCopyPiece != NULL)
  {
    copiedPiece->addSegmentAfter(new ThreadPiece(toCopyPiece));
    copiedPiece = copiedPiece->_next_segment;
    toCopyPiece = toCopyPiece->_next_segment;
  }

  turnOffNoise();

  //_noiseAmount = toCopy->_noiseAmount;
  copyEnergyParams(toCopy);
}

Thread::Thread(Thread* toGetPoints, int num_constraints, int num_pieces_between)
{
  _angle_first_rot = toGetPoints->_angle_first_rot;
  _length = toGetPoints->_length;
  optimizeManyPoints(toGetPoints, num_constraints, num_pieces_between);

  turnOffNoise();
  copyEnergyParams(toGetPoints);
}


Thread::Thread(MatrixXd& constraints, int num_pieces_between, double length_thread)
{
  Matrix3d init_rot = Matrix3d::Identity();
  optimizeManyPointsAndParams(constraints, num_pieces_between, length_thread, init_rot);
  turnOffNoise();
  initEnergyParams();
}

Thread::Thread(MatrixXd& constraints, int num_pieces_between, double length_thread, Matrix3d& init_rot)
{
  optimizeManyPointsAndParams(constraints, num_pieces_between, length_thread, init_rot);

  turnOffNoise();
  initEnergyParams();
}

Thread::~Thread()
{
  delete threadList;
}

void Thread::setConstraints(Vector3d* positions, Vector3d* tangents)
{
  for (int i = 0; i < 2; i++)
  {
    _positions[i] = positions[i];

    _tangents[i] = tangents[i];
    _tangents[i].normalize();
  }

	
	Vector3d axis = Vector3d::UnitX().cross(_tangents[0]);
	if (axis.norm() != 0.0)
		axis.normalize();
  Matrix3d rot_to_start(Eigen::AngleAxisd(acos(Vector3d::UnitX().dot(_tangents[0])),axis));

	_transform_to_start = Matrix4d::Zero();
	_transform_to_start.corner(Eigen::TopLeft,3,3) = rot_to_start;
	//_transform_to_start.corner(Eigen::TopRight,3,1) = _positions[0];
	_transform_to_start(3,3) = 1.0;
	_translate_to_start = Matrix4d::Identity();
  _translate_to_start.corner(Eigen::TopRight,3,1) = _positions[0];

	inverseTransform(_transform_to_start, _transform_to_unit);
	
  //add scaling factors to make it so we can solve in canonical form, with thread length 1
/*	_transform_to_start.corner(Eigen::TopLeft,3,3) = _length*_transform_to_start.corner(Eigen::TopLeft,3,3);
	_transform_to_unit.corner(Eigen::TopLeft,3,3) = (1.0/_length)*_transform_to_unit.corner(Eigen::TopLeft,3,3);
	_transform_to_unit.corner(Eigen::TopRight,3,1) = (1.0/_length)*_transform_to_unit.corner(Eigen::TopRight,3,1);
*/
//	std::cout << "transform to unit vector: " << _transform_to_unit << std::endl;
//	std::cout << "transform to start vector: " << _transform_to_start << std::endl;
	
}

void Thread::initEnergyParams()
{
  rotation_error_penalty = CONSTANT_ROTATION_ERROR_PENALTY;
  position_error_penalty = CONSTANT_POSITION_ERROR_PENALTY;
  thread_curvature_error_penalty = CONSTANT_THREAD_CURVATURE_ERROR_PENALTY;
  thread_torsion_error_penalty = CONSTANT_THREAD_TORSION_ERROR_PENALTY;  
  thread_diff_curvature_error_penalty = CONSTANT_THREAD_DIFF_CURVATURE_ERROR_PENALTY;
  thread_diff_torsion_error_penalty = CONSTANT_THREAD_DIFF_TORSION_ERROR_PENALTY; 
  total_error_penalty = CONSTANT_TOTAL_ERROR_PENALTY;
  gravity_penalty = GRAVITY_CONSTANT;
}

void Thread::copyEnergyParams(const Thread* toCopy)
{
  rotation_error_penalty = toCopy->rotation_error_penalty;
  position_error_penalty = toCopy->position_error_penalty;
  thread_curvature_error_penalty = toCopy->thread_curvature_error_penalty;
  thread_torsion_error_penalty = toCopy->thread_torsion_error_penalty;
  thread_diff_curvature_error_penalty = toCopy->thread_diff_curvature_error_penalty;
  thread_diff_torsion_error_penalty = toCopy->thread_diff_torsion_error_penalty;
  total_error_penalty = toCopy->total_error_penalty;
  gravity_penalty = toCopy->gravity_penalty;
}


void Thread::setEndConstraint(Vector3d& end_position, Vector3d& end_tangent)
{
  _positions[1] = end_position;
  _tangents[1] = end_tangent;
  _tangents[1].normalize();
}

void Thread::moveEndConstraint(Vector3d& move_end_pos, Matrix3d& rotate_end_tangent)
{
  _positions[1] += move_end_pos;
  _tangents[1] = rotate_end_tangent*_tangents[1];
  _tangents[1].normalize();
}


void Thread::getPoints(MatrixXd& points)
{
	Eigen::Transform3d first_rotation_transform(Eigen::AngleAxisd(_angle_first_rot, _tangents[0]));
	Matrix4d trans = _translate_to_start*first_rotation_transform*_transform_to_start;
  trans.corner(Eigen::TopLeft,3,3) *= _length;

	threadList->getPoints(points, 0.0, 1.0/((double)(points.rows()-1)), 0, trans); 
}


double Thread::findMaxDistance(ThreadPiece** maxDistPiece)
{
	if (threadList->_next_segment == NULL)
		return 0.0;

	ThreadPiece* currPiece = threadList;
	double dist = -1;

	while (currPiece->_next_segment != NULL)
	{
		ThreadPiece* next = currPiece->_next_segment;
		double currDist = (pow( (next->_curvature - currPiece->_curvature),2) + pow( (next->_torsion - currPiece->_torsion),2) )* max(next->_length, currPiece->_length);
		
		if (currDist > dist && min(currPiece->_length, next->_length) > MIN_SEG_LENGTH)
		{
			dist = currDist;
			*maxDistPiece = currPiece;
		}

		currPiece = next;
	}
	
	return dist;
}


void Thread::getStartTransform(Matrix4d& toStart)
{
	Eigen::Transform3d first_rotation_transform(Eigen::AngleAxisd(_angle_first_rot, _tangents[0]));
	toStart = _translate_to_start*first_rotation_transform*_transform_to_start;
}


void Thread::getWantedEndPosition(Vector3d& endPos)
{
  endPos = _positions[1];
}

void Thread::getWantedEndTangent(Vector3d& endTan)
{
  endTan = _tangents[1];
}


void Thread::getActualEndPosition(Vector3d& endPos)
{
  Matrix4d currTrans;
  getStartTransform(currTrans);
  currTrans.corner(Eigen::TopLeft,3,3) *= _length;
  ThreadPiece* currPiece = threadList;
  while (currPiece != NULL)
  {
    currTrans *= currPiece->_transform;
    currPiece = currPiece->_next_segment;
  }

  endPos = currTrans.corner(Eigen::TopRight,3,1);
}

void Thread::getActualEndTangent(Vector3d& endTan)
{
  Matrix4d currTrans;
  getStartTransform(currTrans);
  currTrans.corner(Eigen::TopLeft,3,3) *= _length;
  ThreadPiece* currPiece = threadList;
  while (currPiece != NULL)
  {
    currTrans *= currPiece->_transform;
    currPiece = currPiece->_next_segment;
  }

  endTan = currTrans.corner(Eigen::TopLeft,3,1);
}





void Thread::minimize_energy()
{
	//Set up optimization parameters
  setStationaryOptParams();	


/*
  std::cout << "start: " << std::endl;

  std::cout << thread_curvature_error_penalty << std::endl;
  std::cout << thread_torsion_error_penalty  << std::endl;
  std::cout << thread_diff_curvature_error_penalty  << std::endl;
  std::cout << thread_diff_torsion_error_penalty << std::endl;
  std::cout << position_error_penalty  << std::endl;
  std::cout << rotation_error_penalty  << std::endl;
  std::cout << total_error_penalty  << std::endl;
  std::cout << gravity_penalty  << std::endl;
*/

	ThreadPiece* maxDistPiece;
	double maxDist = findMaxDistance(&maxDistPiece);
	while (maxDist > DISTANCE_THRESH)
	{
		//decide how many pieces each piece must be split into
		int numPiecesLeft, numPiecesRight;
		if (maxDistPiece->_length == maxDistPiece->_next_segment->_length)
		{
			numPiecesLeft = 2;
			numPiecesRight = 2;
		} else if (maxDistPiece->_length > maxDistPiece->_next_segment->_length) {
			numPiecesLeft = (int)(maxDistPiece->_length / maxDistPiece->_next_segment->_length);
			numPiecesRight = 1;
		} else if (maxDistPiece->_length < maxDistPiece->_next_segment->_length) {
			numPiecesLeft = 1;
			numPiecesRight = (int)(maxDistPiece->_next_segment->_length / maxDistPiece->_length);
		}
		opt_params.length_per_segment = (maxDistPiece->_length + maxDistPiece->_next_segment->_length)/((double)(numPiecesLeft+numPiecesRight));
		opt_params.num_segments = numPiecesLeft+numPiecesRight;
		//std::cout << "splitting into " << opt_params.num_segments << "   length per segment: " << opt_params.length_per_segment << std::endl;


//		opt_params.transform_back = Matrix4d::Identity();
		opt_params.transform_back = _transform_to_start;
    opt_params.length_back = 0.0;
		opt_params.transform_front = Matrix4d::Identity();
    opt_params.length_front = 0.0;
	
		//opt_params.twist_total_other_segments = 0.0;

		ThreadPiece* currPiece = threadList;
		while (currPiece != maxDistPiece)
		{
			opt_params.transform_back *= currPiece->_transform;
      opt_params.length_back += currPiece->_length;
			//opt_params.twist_total_other_segments += currPiece->_length* currPiece->_torsion;
      opt_params.curvature_back = currPiece->_curvature;
      opt_params.torsion_back = currPiece->_torsion;
			currPiece = currPiece->_next_segment;
		}
		currPiece = maxDistPiece->_next_segment->_next_segment;
    if (currPiece != NULL)
    {
      opt_params.curvature_front = currPiece->_curvature;
      opt_params.torsion_front = currPiece->_torsion;
    }
		while (currPiece != NULL)
		{
			opt_params.transform_front *= currPiece->_transform;
      opt_params.length_front += currPiece->_length;
			//opt_params.twist_total_other_segments += currPiece->_length* currPiece->_torsion;
			currPiece = currPiece->_next_segment;
		}
		
		//std::cout << "twist other seg: " << opt_params.twist_total_other_segments << std::endl;
		
		//std::cout << "transform start: " << opt_params.transform_back << std::endl;
		//std::cout << "transform end: " <<   opt_params.transform_front << std::endl;



		//set initial parameters
		if (opt_params.orig_params_each_piece->nrows() < 2*opt_params.num_segments+1)
		{
			delete(opt_params.orig_params_each_piece);
			opt_params.orig_params_each_piece = new NEWMAT::ColumnVector(2*opt_params.num_segments+1);
		}

		int ind = 0;
		for (; ind < numPiecesLeft; ind++)
		{
			opt_params.orig_params_each_piece->element(2*ind) = maxDistPiece->_curvature;
			opt_params.orig_params_each_piece->element(2*ind+1) = maxDistPiece->_torsion;
		}
		for (; ind < numPiecesLeft + numPiecesRight; ind++)
		{
			opt_params.orig_params_each_piece->element(2*ind) = maxDistPiece->_next_segment->_curvature;
			opt_params.orig_params_each_piece->element(2*ind+1) = maxDistPiece->_next_segment->_torsion;
		}
		opt_params.orig_params_each_piece->element(2*opt_params.num_segments) = _angle_first_rot;


		//setup OPT++ problem
    NEWMAT::ColumnVector x_sol;
		optimize_FDNLF(2*opt_params.num_segments+1, energyEvalFunction, energyEvalFunction_init, x_sol, 1.e-6);

	  //split into pieces with new parameters
		double curvature[numPiecesLeft+numPiecesRight];
		double torsion[numPiecesLeft+numPiecesRight];

		for (int ind=0; ind < opt_params.num_segments; ind++)
		{
			curvature[ind] = x_sol(2*ind +1);
			torsion[ind] = x_sol(2*ind+2);
		}
		_angle_first_rot = x_sol(opt_params.num_segments*2+1);
	//	std::cout << curvature[0] << " " << curvature[1] <<  " " << curvature[2] <<  " " << curvature[3] <<  " " << std::endl;

		maxDistPiece->_next_segment->splitIntoSegments(&curvature[numPiecesLeft], &torsion[numPiecesLeft], numPiecesRight);
		maxDistPiece->splitIntoSegments(curvature, torsion, numPiecesLeft);

    

		//get next piece to be split
		maxDist = findMaxDistance(&maxDistPiece);
		//std::cout << "maxDist: " << maxDist << std::endl;
	}
  //printThreadInfo();
	
}

void Thread::upsample_minLength(double minLength)
{
  //normalize length for our canonical form
  //minLength /= _length;
  //make length some negative power of 2
  double minLengthShifted = minLength;
  while (minLengthShifted <= 1.0)
  {
    minLengthShifted *= 2.0;
  }
  minLength *= (2.0/minLengthShifted); //without the small number subtracted, it crashed...rounding error?


  ThreadPiece* currHead = threadList;
  ThreadPiece* currPiece = threadList;
  double currLength = 0.0;
  while (currPiece != NULL)
  {
    currLength += currPiece->_length;
    if (currLength >= minLength-0.00001)
    {
      if (currHead == currPiece)
      {
        int numPiecesHead = (int)(currPiece->_length / minLength);
        double curvatureHead[numPiecesHead];
        double torsionHead[numPiecesHead];
        for (int i=0; i < numPiecesHead; i++)
        {
          curvatureHead[i] = currHead->_curvature;
          torsionHead[i] = currHead->_torsion;
        }
        currHead->splitIntoSegments(curvatureHead, torsionHead, numPiecesHead);
      } else {
        currHead->combineSegments(currPiece);
      }
      currHead = currHead->_next_segment; //this has been reset by combine segments
      currPiece = currHead;
      currLength = 0.0;
    } else {
      currPiece = currPiece->_next_segment;
    }
  }

  //combine everything left
  if (currHead != NULL)
  {
    currHead->combineSegments(NULL);
  }
}


void Thread::upsampleAndOptimize_minLength(double minLength)
{
  //normalize length for our canonical form
  //minLength /= _length;
  //make length some negative power of 2
  double minLengthShifted = minLength;
  while (minLengthShifted <= 1.0)
  {
    minLengthShifted *= 2.0;
  }
  minLength *= (2.0/minLengthShifted);

  ThreadPiece* currPiece;

  upsample_minLength(minLength);

  //optimize new thread
  setStationaryOptParams(false);

  int numPieces = (int)(1.0/minLength);

	opt_params.length_per_segment = minLength; 
	opt_params.num_segments = numPieces;

  //opt_params.transform_back = Matrix4d::Identity();
  opt_params.transform_back = _transform_to_start;
  opt_params.transform_front = Matrix4d::Identity();
  opt_params.length_back = 0.0;
  opt_params.length_front = 0.0;
  

  //set initial parameters
  if (opt_params.orig_params_each_piece->nrows() < 2*opt_params.num_segments+1)
  {
    delete(opt_params.orig_params_each_piece);
    opt_params.orig_params_each_piece = new NEWMAT::ColumnVector(2*opt_params.num_segments+1);
  }

  currPiece = threadList;

  for (int i=0; i < opt_params.num_segments; i++)
  {
    
    opt_params.orig_params_each_piece->element(2*i) = currPiece->_curvature;
    opt_params.orig_params_each_piece->element(2*i+1) = currPiece->_torsion;
    currPiece = currPiece->_next_segment;
  }
  opt_params.orig_params_each_piece->element(2*opt_params.num_segments) = _angle_first_rot;

  //setup OPT++ problem
  NEWMAT::ColumnVector x_sol;
  optimize_FDNLF(2*opt_params.num_segments+1, energyEvalFunction, energyEvalFunction_init, x_sol, 1.e-4, 7);


  //set new parameters

  currPiece = threadList;
  for (int i=0; i < opt_params.num_segments; i++)
  {
    currPiece->setParams(x_sol.element(2*i), x_sol.element(2*i+1), minLength); 
    currPiece = currPiece->_next_segment;
  }
  _angle_first_rot = x_sol(opt_params.num_segments*2+1);
  

}


void Thread::turnOnNoise(double noiseVal)
{
  _noiseAmount = noiseVal;
}

void Thread::turnOffNoise()
{
  _noiseAmount = 0.0;

  _currNoisePoints[0].setZero();
  _currNoisePoints[1].setZero();
  _currNoiseTans[0].setZero();
  _currNoiseTans[1].setZero();
  _currNoiseCurveCurvature = 0.0;
  _currNoiseCurveTorsion = 0.0;
  _currNoiseCurveDiffCurvature = 0.0;
  _currNoiseCurveDiffTorsion = 0.0;
  _currNoiseGrav = 0.0;
}


void Thread::setStationaryOptParams(bool addNoise)
{
  if (_noiseAmount != 0.0 && addNoise)
  {
    _currNoiseCurveCurvature += randomMaxAbsValue(_noiseAmount*NOISE_EACH_ITER_CURVATURE*thread_curvature_error_penalty);
    _currNoiseCurveTorsion += randomMaxAbsValue(_noiseAmount*NOISE_EACH_ITER_TORSION*thread_torsion_error_penalty);
    _currNoiseCurveDiffCurvature += randomMaxAbsValue(_noiseAmount*NOISE_EACH_ITER_DIFF_CURVATURE*thread_diff_curvature_error_penalty);
    _currNoiseCurveDiffTorsion += randomMaxAbsValue(_noiseAmount*NOISE_EACH_ITER_DIFF_TORSION*thread_diff_torsion_error_penalty);
    _currNoiseGrav += randomMaxAbsValue(_noiseAmount*NOISE_EACH_ITER_GRAV*gravity_penalty);
    for (int i=0; i < 2; i++)
    {
      for (int j=0; j < 3; j++)
      {
        _currNoisePoints[i](j) += randomMaxAbsValue(_noiseAmount*NOISE_EACH_ITER_POS);
        _currNoiseTans[i](j) += randomMaxAbsValue(_noiseAmount*NOISE_EACH_ITER_TAN);
      }
    }

  }

	opt_params.position_end = (1/_length)*(_positions[1]+_currNoisePoints[1] - _positions[0]+_currNoisePoints[0]);
  if (opt_params.position_end.norm() > 0.9999999999)
  {
    opt_params.position_end *= 0.9999999999/opt_params.position_end.norm();
  }
	opt_params.tangent_start = _tangents[0]+_currNoiseTans[0];
	opt_params.tangent_end = _tangents[1]+_currNoiseTans[1];

	opt_params.tangent_start.normalize();
	opt_params.tangent_end.normalize();

  opt_params.gravity_multipler = (gravity_penalty+_currNoiseGrav)*_length;
  opt_params.total_length = _length;

  opt_params.thread_curvature_error_penalty = thread_curvature_error_penalty+_currNoiseCurveCurvature;
  opt_params.thread_torsion_error_penalty = thread_torsion_error_penalty+_currNoiseCurveTorsion;
  opt_params.thread_diff_curvature_error_penalty = thread_diff_curvature_error_penalty+_currNoiseCurveDiffCurvature;
  opt_params.thread_diff_torsion_error_penalty = thread_diff_torsion_error_penalty+_currNoiseCurveDiffTorsion;

  opt_params.thread_position_error_penalty = position_error_penalty;
  opt_params.thread_rotation_error_penalty = rotation_error_penalty;
  opt_params.thread_total_error_penalty = total_error_penalty;

  if (opt_params.orig_params_each_piece != NULL)
    delete(opt_params.orig_params_each_piece); 
	opt_params.orig_params_each_piece = new NEWMAT::ColumnVector(50);


}


void Thread::printThreadInfo()
{
  //print out some information
	Eigen::Transform3d first_rotation_transform(Eigen::AngleAxisd(_angle_first_rot, _tangents[0]));
  int numPieces = 0;
  double energy = 0.0;
	double twist = 0.0;
  double length = 0.0;
  ThreadPiece* currPiece = threadList;
	Matrix4d trans = _translate_to_start*first_rotation_transform*_transform_to_start;
  trans.corner(Eigen::TopLeft,3,3) *= _length;
  while (currPiece != NULL)
  {
    numPieces++;
    energy += currPiece->_length*(currPiece->_torsion*currPiece->_torsion + currPiece->_curvature*currPiece->_curvature); 
		//twist += currPiece->_length*currPiece->_torsion;
    length += currPiece->_length;
    std::cout << "length:" << currPiece->_length << std::endl;
    std::cout << "curvature: " << currPiece->_curvature/_length << "  torsion: " << currPiece->_torsion/_length << std::endl;
    trans *= currPiece->_transform;
    currPiece = currPiece->_next_segment;
  }

  std::cout << "Optimal thread has " << numPieces << " pieces, and a calculated energy of " << energy*_length << std::endl;
  std::cout << "Final Position: \n" << trans.corner(Eigen::TopRight, 3,1) << std::endl;
  std::cout << "Final Position Wanted: \n" << _positions[1] << std::endl;
  std::cout << "Final Tangent: \n" << trans.corner(Eigen::TopLeft, 3, 1)/_length << std::endl;
}


//sets constraints as points along thread, with num_pts_between each constraint, and sets this thread to be optimized version
//assumes start transform already set
bool Thread::optimizeManyPoints(Thread* orig_thread, int num_constraints, int num_pts_between)
{
  _length = orig_thread->length();
  MatrixXd resampled_points(num_constraints+1,3);
  vector<double> curvatures_resampled;
  vector<double> torsions_resampled;
  orig_thread->resamplePointsAndParams(resampled_points, curvatures_resampled, torsions_resampled);

  Matrix4d start_transform;
  orig_thread->getStartTransform(start_transform);

  optimizeManyPoints(resampled_points, num_pts_between, curvatures_resampled, torsions_resampled, start_transform);
}


void Thread::optimizeManyPoints_MyParams(MatrixXd& constraints, int num_pts_between)
{
  MatrixXd myPoints(constraints.rows(), 3); //really just going to throw this out...but this gives us curve params
  vector<double> curvatures_resampled;
  vector<double> torsions_resampled;
  resamplePointsAndParams(myPoints, curvatures_resampled, torsions_resampled);
  Matrix4d start_transform;
  getStartTransform(start_transform);

  optimizeManyPoints(constraints, num_pts_between, curvatures_resampled, torsions_resampled, start_transform);
}

void Thread::optimizeManyPoints(MatrixXd& constraints, int num_pts_between, vector<double>&curvatures_resampled, vector<double>& torsions_resampled, Matrix4d& start_transform)
{
  int num_constraints = constraints.rows()-1;
  int num_pieces_reopt = num_pts_between*num_constraints;
  double length_per_piece = 1.0/( num_pieces_reopt);
  
  //opt_params_many_points.transform_back = _transform_to_start;
  opt_params_many_points.num_segments = num_pieces_reopt;
  opt_params_many_points.length_per_segment = length_per_piece;


  opt_params_many_points.points.resize(num_constraints);
  opt_params_many_points.orig_params_each_piece = new NEWMAT::ColumnVector(2*num_pieces_reopt+3);
  opt_params_many_points.num_pts_between = num_pts_between;

  opt_params_many_points.gravity_multipler = (GRAVITY_CONSTANT)*_length;
  opt_params_many_points.total_length = _length;


  for (int i=0; i < num_constraints; i++)
  {
    for (int j=0; j < num_pts_between; j++)
    {
      int piece_num = i*num_pts_between+j;
      opt_params_many_points.orig_params_each_piece->element(2*piece_num) = curvatures_resampled[i];
      opt_params_many_points.orig_params_each_piece->element(2*piece_num+1) = torsions_resampled[i];
    }
  }

  

  double eulerZ, eulerY, eulerX;
  eulerAnglesFramTransform(start_transform.corner(Eigen::TopLeft,3,3), eulerZ, eulerY, eulerX);
  opt_params_many_points.orig_params_each_piece->element(2*num_pieces_reopt) = eulerZ;
  opt_params_many_points.orig_params_each_piece->element(2*num_pieces_reopt+1) = eulerY;
  opt_params_many_points.orig_params_each_piece->element(2*num_pieces_reopt+2) = eulerX;

  for (int i=0; i < num_constraints; i++)
  {
    opt_params_many_points.points[i] = ((constraints.block((i+1),0,1,3) - constraints.block(0,0,1,3))/_length).transpose();
  }


  NEWMAT::ColumnVector x_sol;

  optimize_FDNLF(2*opt_params_many_points.num_segments+3, energyEvalFunctionManyPoints, energyEvalFunctionManyPoints_init, x_sol);


  std::cout << "done re-optimizing" << std::endl;
  delete opt_params_many_points.orig_params_each_piece;

 
  //set thread params
  _positions[0] = constraints.block(0,0,1,3).transpose();
  transformFromEulerAngles(_transform_to_start, x_sol(2*num_pieces_reopt+1), x_sol(2*num_pieces_reopt+2));
	_translate_to_start = Matrix4d::Identity();
  _translate_to_start.corner(Eigen::TopRight,3,1) = _positions[0];

  _tangents[0] = _transform_to_start.corner(Eigen::TopLeft,3,1);
  //_tangents[0] = orig_thread->_tangents[0];

  _angle_first_rot = x_sol(2*num_pieces_reopt+3);

	inverseTransform(_transform_to_start, _transform_to_unit);



  Matrix4d transform_back;
  getStartTransform(transform_back);
  transform_back.corner(Eigen::TopLeft,3,3) *= _length;
  threadList = new ThreadPiece(x_sol(1), x_sol(2), length_per_piece);
  ThreadPiece* currPiece = threadList;
  for (int i=1; i < num_pieces_reopt; i++)
  {
    currPiece->addSegmentAfter(new ThreadPiece(x_sol(2*i+1), x_sol(2*i+2), length_per_piece));
    transform_back *= currPiece->_transform;
    currPiece = currPiece->_next_segment;
  }

  //_positions[1] = transform_back.corner(Eigen::TopRight,3,1);
 // _positions[1] = orig_thread->_positions[1];
 // _tangents[1] = orig_thread->_tangents[1];
 // _tangents[1].normalize();
 // _tangents[0].normalize();

  _positions[1] = transform_back.corner(Eigen::TopRight,3,1);
  _tangents[1] = transform_back.corner(Eigen::TopLeft,3,1);
  _tangents[1].normalize();

  //printThreadInfo();



}



bool Thread::resamplePointsAndParams(MatrixXd& resampled_points, vector<double>& curvatures, vector<double>& torsions)
{
  curvatures.resize(resampled_points.rows());
  torsions.resize(resampled_points.rows());
  for (int i=0; i <curvatures.size(); i++)
  {
    curvatures[i] = torsions[i] = 0.0;
  }


	Matrix4d first_trans;
  getStartTransform(first_trans);
  first_trans.corner(Eigen::TopLeft,3,3)*= _length;
	threadList->getPointsAndParams(resampled_points, 0.0, 1.0/((double)resampled_points.rows()-1), 0, first_trans, curvatures, torsions); 

/*
  for (int i=0; i < resampled_points.rows(); i++)
  {
    std::cout << "curvature: " << curvatures[i] << "      torsion: " << torsions[i] << std::endl;
  }
*/
}

void Thread::optimizeManyPointsAndParams(MatrixXd& constraints, int num_pieces_between, double length_thread, Matrix3d& init_rot)
{
  _length = length_thread;
  vector<double> curvatures; curvatures.resize(constraints.size()-1);
  vector<double> torsions; torsions.resize(constraints.size()-1);
  for (int i=0; i < constraints.size()-1; i++)
  {
    curvatures[i] = 1.0;
    torsions[i] = 0.5;
  }
  Matrix4d start_transform;
  start_transform.corner(Eigen::TopLeft,3,3) = init_rot;
  optimizeManyPoints(constraints, num_pieces_between, curvatures, torsions, start_transform);


}

/*

void Thread::optimize_GSS(int numParams, OPTPP::USERFCN0 eval, OPTPP::INITFCN init, NEWMAT::ColumnVector& solution, double fcnTol)
{
  OPTPP::NLF0 nlp(numParams, eval, init); 
  OPTPP::GenSetStd gs(numParams); 
  OPTPP::OptGSS optobj(&nlp, &gs);
  optobj.setMaxIter(5000); 
  optobj.setMaxFeval(50000); 
  optobj.setFcnTol(fcnTol);  
  optobj.setFullSearch(true);
  optobj.optimize();
  //optobj.printStatus("Solution Vision GSS");

  solution = nlp.getXc();



}


void Thread::optimize_FDNLF(int numParams, OPTPP::USERFCN0 eval, OPTPP::INITFCN init, NEWMAT::ColumnVector& solution, double fcnTol)
{
  OPTPP::FDNLF1 nlp(numParams, eval, init);

  OPTPP::OptQNewton objfcn(&nlp);

  objfcn.setSearchStrategy(OPTPP::TrustRegion);
  objfcn.setMaxFeval(20000);
  objfcn.setMaxIter(5000);
  objfcn.setFcnTol(fcnTol);

  objfcn.optimize();
//  objfcn.printStatus("Solution VISION");

  solution = nlp.getXc();
}


*/







void energyEvalFunction_init(int ndim, NEWMAT::ColumnVector& x)
{
	for (int i = 0; i < ndim; i++)
	{
		x.element(i) = opt_params.orig_params_each_piece->element(i);
	}
}


void energyEvalFunction(int ndim, const NEWMAT::ColumnVector& x, double& fx, int& result)
{
  //(ndim-1)/2 pieces of thread, where x(2*i) and x(2*i+1) are the curvature and torsion of piece i
  //x(ndim) represents the angle of rotation of the first piece

	//error from curvature
	fx = 0.0;
	for (int i=1; i < ndim; i+=2)
	{
		fx += x(i)*x(i);
	}
	fx *= opt_params.thread_curvature_error_penalty*opt_params.length_per_segment*opt_params.total_length;

  //error from torsion
  double torsion_energy = 0.0;
	for (int i=2; i < ndim; i+=2)
	{
		torsion_energy += x(i)*x(i);
	}
	fx += torsion_energy*opt_params.thread_torsion_error_penalty*opt_params.length_per_segment*opt_params.total_length;

  //std::cout << "thread energy: " << fx << std::endl;

  //energy from diff curvature
  double diff_curvature_energy = 0.0;
  if (opt_params.length_back > 0.0)
    diff_curvature_energy += pow(opt_params.curvature_back - x(1),2);
  for (int i=3; i < ndim; i+=2)
  {
    diff_curvature_energy += pow(x(i)-x(i-2),2);
  }
  if (opt_params.length_front > 0.0)
    diff_curvature_energy += pow(opt_params.curvature_front - x(ndim-2),2);

  //energy from diff torsion
  double diff_torsion_energy = 0.0;
  if (opt_params.length_back > 0.0)
    diff_torsion_energy += pow(opt_params.torsion_back - x(2),2);
  for (int i=4; i < ndim; i+=2)
  {
    diff_torsion_energy += pow(x(i)-x(i-2),2);
  }
  if (opt_params.length_front > 0.0)
    diff_torsion_energy += pow(opt_params.torsion_front - x(ndim-1),2);
  
  fx += (opt_params.thread_diff_curvature_error_penalty*diff_curvature_energy
         +opt_params.thread_diff_torsion_error_penalty*diff_torsion_energy)*opt_params.length_per_segment*opt_params.total_length;

  double grav_energy = 0.0;
	//calculate new end transform
	Eigen::Transform3d first_rotation_transform(Eigen::AngleAxisd(x(ndim), opt_params.tangent_start));
	Matrix4d transform_to_end = first_rotation_transform*opt_params.transform_back;

  double curr_z = transform_to_end(2,3);
  grav_energy += (curr_z/2.0)*opt_params.length_back*opt_params.total_length;
 
	Matrix4d nextTransform;
	for (int i=1; i <= (ndim-1)/2; i++)
	{
			getTransform(x(2*i-1), x(2*i), opt_params.length_per_segment, nextTransform);
			//twist_total += opt_params.length_per_segment*x(2*i);
			transform_to_end *= nextTransform;
      grav_energy += ((transform_to_end(2,3)-curr_z)/2.0 + curr_z)*opt_params.length_per_segment*opt_params.total_length;
      curr_z = transform_to_end(2,3);
	}
 
  transform_to_end *= opt_params.transform_front;
  grav_energy += ((transform_to_end(2,3)-curr_z)/2.0 + curr_z)*opt_params.length_per_segment*opt_params.total_length*opt_params.total_length;

 // std::cout << "grav energy " << grav_energy*opt_params.gravity_multipler << std::endl;
  fx += grav_energy*opt_params.gravity_multipler;


	//error from new transform
	Vector3d new_end_position = transform_to_end.corner(Eigen::TopRight,3,1);
	Vector3d new_end_tangent = transform_to_end.corner(Eigen::TopLeft,3,1);

	//error in position
	double pos_err = opt_params.thread_position_error_penalty*(pow( ((new_end_position - opt_params.position_end).norm()),2));

	//std::cout << "position error: " << new_end_position << " - " << opt_params.position_end << " ->" << pos_err << std::endl;

	//error in rotation
	double rot_err = exp(opt_params.thread_rotation_error_penalty*((1.0-opt_params.tangent_end.dot(new_end_tangent))) );
	//double rot_err = exp(CONSTANT_ROTATION_ERROR_PENALTY*((1.0-opt_params.tangent_start.dot(opt_params.tangent_start)) + (1.0-opt_params.tangent_end.dot(new_end_tangent))) );

	fx += opt_params.total_length*opt_params.thread_total_error_penalty*(exp(pos_err + rot_err) );

	//double twist_err = CONSTANT_TWIST_ERROR_PENALTY*(abs(twist_total-opt_params.twist_total));
	//fx += twist_err;
  


  result = OPTPP::NLPFunction;
	
}





void energyEvalFunctionManyPoints_init(int ndim, NEWMAT::ColumnVector& x)
{
	for (int i = 0; i < ndim; i++)
	{
		x.element(i) = opt_params_many_points.orig_params_each_piece->element(i);
	}

}


//CONSIDER ADDING TANGENT AT EACH POINT AS WELL
void energyEvalFunctionManyPoints(int ndim, const NEWMAT::ColumnVector& x, double& fx, int& result)
{
  //We have (ndim-3)/2 curve segments in x, where x(2*i-1) and x(2*i) are curvature and torsion of segment i
  //last 3 pieces are euler angles

	//error from energy
	fx = 0.0;
	for (int i=1; i <= ndim-3; i++)
	{
		fx += x(i)*x(i);
	}
	fx *= MANYPOINTS_COEFF_ENERGY*opt_params_many_points.length_per_segment*opt_params_many_points.total_length;
  //std::cout << "thread energy: " << fx << std::endl;
  
  //error from difference in curve params
  double diff_params = 0.0;
	for (int i=3; i <= ndim-3; i++)
	{
		diff_params += pow(x(i)-x(i-2),2);
	}
  fx += diff_params*MANYPOINTS_COEFF_DIFF_PARAMS*opt_params_many_points.length_per_segment*opt_params_many_points.total_length;
  //std::cout << "diff thread params: " << diff_params*OPTIMIZATION_COEFF_DIFF_PARAMS << std::endl;



  Matrix4d transform_to_end;
  transformFromEulerAngles(transform_to_end, x(ndim-2), x(ndim-1), x(ndim));

  //std::cout << "transform: " << transform_to_end;
 // std::cout << "angles: " << x(ndim-2) << " " << x(ndim-1) << " " << x(ndim) << std::endl;
  
  double curr_z = transform_to_end(2,3);

  //energy from gravity and vision
  double grav_energy = 0.0;
  double point_error_energy = 0.0;

	Matrix4d nextTransform;
	for (int i=1; i <= (ndim-3)/2; i++)
	{
			getTransform(x(2*i-1), x(2*i), opt_params_many_points.length_per_segment, nextTransform);
			transform_to_end *= nextTransform;
      grav_energy += ((transform_to_end(2,3)-curr_z)/2.0 + curr_z)*opt_params_many_points.length_per_segment;
      curr_z = transform_to_end(2,3);

      //point
      //error in position
      if (i%opt_params_many_points.num_pts_between == 0)
      {
        Vector3d new_end_position = transform_to_end.corner(Eigen::TopRight,3,1);

          
        point_error_energy += MANYPOINTS_COEFF_POINT_EXPONENTIAL*pow(  ((new_end_position - opt_params_many_points.points.at(i/opt_params_many_points.num_pts_between-1)).norm()),2);
      }
	}
  //add extra weight for last point
//  Vector3d final_end_position = transform_to_end.corner(Eigen::TopRight,3,1);
//  point_error_energy += 1000.0*pow((final_end_position - opt_params_many_points.points.back()).norm(),2);

  fx += grav_energy*opt_params_many_points.gravity_multipler + point_error_energy*MANYPOINTS_COEFF_POINT;


  result = OPTPP::NLPFunction;


}










Thread* Thread_Motion::applyMotion(const Thread* start)
{
  Thread* end = new Thread(start);
  end->moveEndConstraint(pos_movement, tan_rotation);
  end->upsampleAndOptimize_minLength(0.065);
  end->minimize_energy();
  return end;
}

void Thread_Motion::setRotationMatrixFromAngs(double ang1, double ang2)
{
  //ang1 is a rotation about Z, ang2 is a rotation about Y
  tan_rotation = Eigen::AngleAxisd(ang1, Vector3d::UnitZ()) *
                 Eigen::AngleAxisd(ang2, Vector3d::UnitY());
}

Thread_Motion& Thread_Motion::operator=(const Thread_Motion& rhs)
{
  pos_movement = rhs.pos_movement;
  tan_rotation = rhs.tan_rotation;
}

