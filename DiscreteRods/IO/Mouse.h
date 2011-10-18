#ifndef _Mouse_h
#define _Mouse_h

#include "ControllerBase.h"
#include "../threadutils_discrete.h"

enum key_code {NONE, MOVEPOS, MOVETAN, ROTATETAN};

class Mouse : public ControllerBase
{
public:

  Mouse(const Vector3d& pos, const Matrix3d& rot);
  Mouse();
  ~Mouse();
  
  void setPressButton(bool bttn, button_type bttn_type);
	bool hasButtonPressed(button_type bttn_type);
	bool hasButtonPressedAndReset(button_type bttn_type);
  void setKeyPressed(key_code key);
  key_code getKeyPressed();
  void add2DMove(float m0, float m1);
  void applyTransformFrom2DMove(GLdouble model_view[], GLdouble projection[], GLint viewport[]);
  
protected:
  key_code key_pressed;
  Vector2f move;
  bool button_state[2];
};

#endif // _Mouse_h
