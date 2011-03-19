#include "threadpiece_minenergy.h"

ThreadPiece::ThreadPiece() :
  _next_segment(NULL), _last_segment(NULL)
{
}


ThreadPiece::ThreadPiece(double curvature, double torsion, double length) :
  _next_segment(NULL), _last_segment(NULL)
{
	setParams(curvature, torsion, length);
}

ThreadPiece::ThreadPiece(const ThreadPiece* toCopyPiece) :
  _next_segment(NULL), _last_segment(NULL)
{
  _curvature = toCopyPiece->_curvature;
  _torsion = toCopyPiece->_torsion;
  _length = toCopyPiece->_length;
  _transform = toCopyPiece->_transform;
}

//this is untested
/*ThreadPiece::ThreadPiece(double curvature, double torsion, double length, ThreadPiece* prev, ThreadPiece* next)
{
	setParams(curvature, torsion, length);

	_next_segment = next;
	if (next != NULL)
		next->_last_segment = this;
	_last_segment = prev;
	if (prev != NULL)
		prev->_next_segment = this;
}*/


ThreadPiece::~ThreadPiece()
{
  _last_segment = NULL;
  delete _next_segment;
}


void ThreadPiece::setParams(double curvature, double torsion, double length)
{
	_curvature = curvature;
	_torsion = torsion;
	_length = length;

	getTransform(curvature, torsion, length, _transform);

}


void ThreadPiece::addSegmentAfter(ThreadPiece* new_segment)
{
	//have new segment point to next, and next to new
	if (_next_segment != NULL)
	{
		_next_segment->_last_segment = new_segment;
	}
	new_segment->_next_segment = _next_segment;

	//point to new, and have new point to me
	_next_segment = new_segment;
	new_segment->_last_segment = this;
}

void ThreadPiece::splitIntoSegments(double* curvature, double* torsion, int numSegments)
{
		if (numSegments < 1)
			return;

		double length = _length/((double)numSegments);

		//this current segment is the first in the chain, so remember final
		setParams(curvature[0], torsion[0], length);
		ThreadPiece* last_segment = this;
		for (int i=1; i < numSegments; i++)
		{
			last_segment->addSegmentAfter(new ThreadPiece(curvature[i], torsion[i], length) );
			last_segment = last_segment->_next_segment;
		}
}

void ThreadPiece::combineSegments(ThreadPiece* lastPiece)
{
  
  if (this == lastPiece)
  {
    return;
  }

  ThreadPiece* currPiece = this;
  double curvature = 0.0;
  double torsion = 0.0;
  double length = 0.0;
//  std::cout << "combining " << std::endl;
//  std::cout << "last length" << lastLength << std::endl;

  if (lastPiece != NULL)
  {
    lastPiece = lastPiece->_next_segment; //really, this is now the next piece
  }
  while (currPiece != lastPiece)
  {
    curvature += currPiece->_curvature*currPiece->_length;
    torsion += currPiece->_torsion*currPiece->_length;
    length += currPiece->_length;
    currPiece = currPiece->_next_segment; 
  }
 
  curvature /= length;
  torsion /= length;

  replaceSegment(curvature, torsion, length, lastPiece);

}



void ThreadPiece::replaceSegment(double curvature, double torsion, double length, ThreadPiece* next)
{
  setParams(curvature, torsion, length);

  if (next != NULL)
  {
    next->_last_segment->_next_segment = NULL;
    next->_last_segment = this;
  }
  
    delete _next_segment;
    _next_segment = next;
}



void ThreadPiece::getPoints(MatrixXd& points, double currLength, double increment, int index, Matrix4d& currTransform)
{
	
	double rho = sqrt(_curvature*_curvature + _torsion*_torsion);
	double r;
	double c = (1.0/(rho*rho*rho));
	Vector4d p;
	p(3) = 1.0;

//int orig_ind = index;

	while (currLength <= _length && index < points.rows())
	{
		r = rho*currLength;

		p(0) = c*(_curvature*_curvature*sin(r) + _torsion*_torsion*r);
		p(1) = c*(_curvature*rho*(1.0-cos(r)));
		p(2) = c*(_curvature*_torsion*(r-sin(r)));

		points.block(index,0,1,3) = ((currTransform*p).start(3)).transpose();

		currLength = currLength + increment;
		index++;
	}

	currTransform *= _transform;
	if (_next_segment != NULL)
		_next_segment->getPoints(points, currLength-_length, increment, index, currTransform);
  else {
    for ( ; index < points.rows(); index++)
    {
      points.block(index,0,1,3) = (currTransform.corner(Eigen::TopRight,3,1)).transpose();
    }
  }

}

void ThreadPiece::getPoints(vector<Vector3d>& points, double currLength, double increment, int index, Matrix4d& currTransform)
{
	double rho = sqrt(_curvature*_curvature + _torsion*_torsion);
	double r;
	double c = (1.0/(rho*rho*rho));
	Vector4d p;
	p(3) = 1.0;

//int orig_ind = index;

	while (currLength <= _length && index < points.size())
	{
		r = rho*currLength;

		p(0) = c*(_curvature*_curvature*sin(r) + _torsion*_torsion*r);
		p(1) = c*(_curvature*rho*(1.0-cos(r)));
		p(2) = c*(_curvature*_torsion*(r-sin(r)));

		points[index] = ((currTransform*p).start(3));

		currLength = currLength + increment;
		index++;
	}
//	std::cout << "params: " << _curvature << "  " << _torsion << "  " << _length << " " << std::endl;

	currTransform *= _transform;
	if (_next_segment != NULL)
		_next_segment->getPoints(points, currLength-_length, increment, index, currTransform);
  else {
    for ( ; index < points.size(); index++)
    {
      points[index] = (currTransform.corner(Eigen::TopRight,3,1)).transpose();
    }
  }

}



void ThreadPiece::getPointsAndParams(MatrixXd& points, double currLength, double increment, int index, Matrix4d& currTransform, std::vector<double>& curvatures, std::vector<double>& torsions)
{
	
	double rho = sqrt(_curvature*_curvature + _torsion*_torsion);
	double r;
	double c = (1.0/(rho*rho*rho));
	Vector4d p;
	p(3) = 1.0;

//int orig_ind = index;

  double length_used_for_avg = 0.0;
	while (currLength <= _length && index < points.rows())
	{
		r = rho*currLength;

		p(0) = c*(_curvature*_curvature*sin(r) + _torsion*_torsion*r);
		p(1) = c*(_curvature*rho*(1.0-cos(r)));
		p(2) = c*(_curvature*_torsion*(r-sin(r)));

		points.block(index,0,1,3) = ((currTransform*p).start(3)).transpose();



    if (index > 0)
    {
      curvatures[index-1] += _curvature*(currLength-length_used_for_avg);
      curvatures[index-1] /= increment;
      torsions[index-1] += _torsion*(currLength-length_used_for_avg); 
      torsions[index-1] /= increment;
      length_used_for_avg = currLength;
    }
    
		currLength = currLength + increment;
		index++;
	}
  if (index > 0)
  {
    curvatures[index-1] += _curvature*(_length-length_used_for_avg);
    torsions[index-1] += _torsion*(_length-length_used_for_avg);
  }
//	std::cout << "params: " << _curvature << "  " << _torsion << "  " << _length << " " << std::endl;

/*if (orig_ind < 500)
{
std::cout << "first point: " << points.block(orig_ind,0,1,3) << std::endl;
std::cout << "last point: " << points.block(index-1,0,1,3) << std::endl;
}
*/
	currTransform *= _transform;
	if (_next_segment != NULL)
		_next_segment->getPointsAndParams(points, currLength-_length, increment, index, currTransform, curvatures, torsions);
  else {
    for (; index < points.rows(); index++)
    {
      points.block(index,0,1,3) = (currTransform.corner(Eigen::TopRight,3,1)).transpose();
    }
  }
}

void ThreadPiece::getParams(double currLength, double increment, int index, std::vector<double>& curvatures, std::vector<double>& torsions)
{
//int orig_ind = index;

  double length_used_for_avg = 0.0;
	while (currLength <= _length && index < curvatures.size()+1)
	{
    if (index > 0)
    {
      curvatures[index-1] += _curvature*(currLength-length_used_for_avg);
      curvatures[index-1] /= increment;
      torsions[index-1] += _torsion*(currLength-length_used_for_avg); 
      torsions[index-1] /= increment;
      length_used_for_avg = currLength;
    }
    
		currLength = currLength + increment;
		index++;
	}

  if (index > 0)
  {
    curvatures[index-1] += _curvature*(_length-length_used_for_avg);
    torsions[index-1] += _torsion*(_length-length_used_for_avg);
  }


	if (_next_segment != NULL)
		_next_segment->getParams(currLength-_length, increment, index, curvatures, torsions);
  else {
    for (; index < curvatures.size(); index++)
    {
      curvatures[index] = _curvature;
      torsions[index] = _torsion;
    }
  }
}

