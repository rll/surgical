#ifndef _Haptic_h
#define _Haptic_h

#include "ControlBase.h"

class Haptic : public ControlBase
{
public:

  Haptic();
  ~Haptic();
  
	void setRelativeTransform(const Vector3d& pos, const Matrix3d& rot);
	void resetPosition(const Vector3d& reset_position);
  
protected:
	Vector3d zero_position;
};

#endif // _Haptic_h
