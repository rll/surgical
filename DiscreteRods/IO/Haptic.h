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
	void setHapticButton(bool bttn, button_type bttn_type);
	bool hasButtonPressedAndReset(button_type bttn_type);
  
protected:
	Vector3d zero_position;
	bool haptic_button[2];
  bool last_haptic_button[2];
};

#endif // _Haptic_h
