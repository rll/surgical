#include "threadpiece_vision.h"

ThreadPiece_Vision::ThreadPiece_Vision()
  :ThreadPiece(), 
    _numPointingToMe(0)
{

}

ThreadPiece_Vision::ThreadPiece_Vision(double curvature, double torsion, double length)
  :ThreadPiece(curvature, torsion, length), _numPointingToMe(0)
{

}


ThreadPiece_Vision::ThreadPiece_Vision(double curvature, double torsion, double length, ThreadPiece_Vision* prev)
  :ThreadPiece(curvature, torsion, length), _numPointingToMe(0)
{
 _last_segment = prev;
  prev->_numPointingToMe++;
  _numPieces = prev->_numPieces + 1;
  _transform_after = prev->_transform_after * _transform;
}


ThreadPiece_Vision::ThreadPiece_Vision(ThreadPiece_Vision* toCopy, ThreadPiece_Vision* prev)
{
  setParams(toCopy->_curvature, toCopy->_torsion, toCopy->_length);

  if (prev == NULL)
  {
    Matrix4d transform;
    toCopy->getTransformBefore(transform);
    setPrevTransform(transform);
    _numPieces = 0;
  } else {
    connectPrevSegment(prev);
  }

}



ThreadPiece_Vision::~ThreadPiece_Vision()
{
  //UNTESTED
  if (_last_segment != NULL)
  {
    if (((ThreadPiece_Vision*)_last_segment)->_numPointingToMe == 1)
    {
      delete _last_segment;
    } else {
      ((ThreadPiece_Vision*)_last_segment)->_numPointingToMe--;
    }
  }
}

void ThreadPiece_Vision::removeLastBeforeDelete()
{
  if (_last_segment != NULL)
  {
    ((ThreadPiece_Vision*)_last_segment)->_numPointingToMe--;
    _last_segment = NULL;
  }
}


void ThreadPiece_Vision::setParams(double curvature, double torsion, double length)
{
  ThreadPiece::setParams(curvature, torsion, length);

  if (_last_segment != NULL)
  {
    _transform_after = ((ThreadPiece_Vision*)_last_segment)->_transform_after*_transform;
  }

}


void ThreadPiece_Vision::setPrevTransform(Matrix4d& trans)
{
  _transform_after = trans*_transform;
}


void ThreadPiece_Vision::connectPrevSegment(ThreadPiece_Vision* prev)
{
  if (_last_segment != NULL)
    ((ThreadPiece_Vision*)_last_segment)->_numPointingToMe--;
  _last_segment = prev;
  prev->_numPointingToMe++;
  _numPieces = prev->_numPieces + 1;
  _transform_after = prev->_transform_after * _transform;
}

void ThreadPiece_Vision::getLastPoint(Point3f& endPoint)
{
  //assumes we already have the end transform set
  endPoint.x = (float)_transform_after(0,3);
  endPoint.y = (float)_transform_after(1,3);
  endPoint.z = (float)_transform_after(2,3);
}

void ThreadPiece_Vision::getFirstPoint(Point3f& startPoint)
{
  Matrix4d transformInv;
  inverseTransform(_transform, transformInv);
  Vector3d point = (_transform_after*transformInv).corner(Eigen::TopRight,3,1);
  startPoint.x = (float)point(0,0);
  startPoint.y = (float)point(1,0);
  startPoint.z = (float)point(2,0);
}

void ThreadPiece_Vision::getLastPoint(Vector3d& endPoint)
{
  //assumes we already have the end transform set
  endPoint = _transform_after.corner(Eigen::TopRight,3,1);
}

void ThreadPiece_Vision::getFirstPoint(Vector3d& startPoint)
{
  Matrix4d transformInv;
  inverseTransform(_transform, transformInv);
  startPoint = (_transform_after*transformInv).corner(Eigen::TopRight,3,1);
}


void ThreadPiece_Vision::getLastTan(Vector3d& tan)
{
  //assumes we already have the end transform set
  tan = _transform_after.corner(Eigen::TopLeft,3,1);
}

void ThreadPiece_Vision::getFirstTan(Vector3d& tan)
{
  Matrix4d transformInv;
  inverseTransform(_transform, transformInv);
  tan = (_transform_after*transformInv).corner(Eigen::TopLeft,3,1);
}



void ThreadPiece_Vision::getTransformBefore(Matrix4d& transform)
{
  Matrix4d transformInv;
  inverseTransform(_transform, transformInv);
  transform = _transform_after*transformInv;
}



void ThreadPiece_Vision::setScore(double scoreFromVis)
{
  _score = _length*(SCORE_COEFF_ENERGY_CURVATURE*pow(_curvature,2) + SCORE_COEFF_ENERGY_TORSION*pow(_torsion,2)) + SCORE_COEFF_VISION*(scoreFromVis);
  if (_last_segment != NULL)
  {
    _score += ((ThreadPiece_Vision*)_last_segment)->_score ;
   /* _score -= SCORE_COEFF_DISTANCE*
                    (pow(_transform_after(0,3)-((ThreadPiece_Vision*)_last_segment)->_transform_after(0,3),4) +
                    pow(_transform_after(1,3)-((ThreadPiece_Vision*)_last_segment)->_transform_after(1,3),4) + 
                    pow(_transform_after(2,3)-((ThreadPiece_Vision*)_last_segment)->_transform_after(2,3),4) );
    */
    _score += SCORE_COEFF_DIF_PARAMS_CURVATURE*pow(_curvature - _last_segment->_curvature,2) +
              SCORE_COEFF_DIF_PARAMS_TORSION*pow(_torsion - _last_segment->_torsion,2);
  }
}



bool operator <(const ThreadPiece_Vision& a, const ThreadPiece_Vision& b)
{
  return a._score < b._score;
}

/*
bool operator < (const ThreadPiece_Vision* a, const ThreadPiece_Vision* b)
{
  return a->_score < b->_score;
}
*/

bool lessThanThreadPiecePointer (const ThreadPiece_Vision* a, const ThreadPiece_Vision* b)
{
  return a->_score < b->_score;
}

