#ifndef _Mouse_h
#define _Mouse_h

#include "ControlBase.h"
#include "../threadutils_discrete.h"

enum key_code {NONE, MOVEPOS, MOVETAN, ROTATETAN};

class Mouse : public ControlBase
{
public:

  Mouse();
  ~Mouse();
  
  void setKeyPressed(key_code key);
  key_code getKeyPressed();
  void add2DMove(float m0, float m1);
  void applyTransformFrom2DMove(GLdouble model_view[], GLdouble projection[], GLint viewport[]);
  
protected:
  key_code key_pressed;
  Vector2f move;
};

#endif // _Mouse_h
