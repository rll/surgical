#include "Mouse.h"

#define MOVE_POS_CONST 1.0
#define MOVE_TAN_CONST 0.05
#define ROTATE_TAN_CONST 0.2

Mouse::Mouse(const Vector3d& pos, const Matrix3d& rot)
	: ControllerBase(pos, rot)
	, key_pressed(NONE)
	, move(0.0, 0.0)
{
	button_state[UP] = button_state[DOWN] = false;
}

Mouse::Mouse()
	: ControllerBase()
	, key_pressed(NONE)
	, move(0.0, 0.0)
{
	button_state[UP] = button_state[DOWN] = false;
}

Mouse::~Mouse() {}

void Mouse::setPressButton(bool bttn, button_type bttn_type)
{
	button_state[bttn_type] = bttn;
}

bool Mouse::hasButtonPressed(button_type bttn_type)
{
	return  button_state[bttn_type];
}

bool Mouse::hasButtonPressedAndReset(button_type bttn_type)
{
	bool result = button_state[bttn_type];
	button_state[bttn_type] = false;
	return result;
}

void Mouse::setKeyPressed(key_code key)
{
	key_pressed = key;
}

key_code Mouse::getKeyPressed()
{
	return key_pressed;
}

void Mouse::add2DMove(float m0, float m1)
{
	move(0) += m0;
	move(1) += m1;
}

void Mouse::applyTransformFrom2DMove(GLdouble model_view[], GLdouble projection[], GLint viewport[])
{
	switch (key_pressed)
    {
      case NONE:
        {
          break;
        }
      case MOVEPOS:
        {
					double winX, winY, winZ;
					gluProject(position(0), position(1), position(2), model_view, projection, viewport, &winX, &winY, &winZ);
					winX += move(0)*MOVE_POS_CONST;
					winY += move(1)*MOVE_POS_CONST;
					move(0) = 0.0;
					move(1) = 0.0;
					gluUnProject(winX, winY, winZ, model_view, projection, viewport, &position(0), &position(1), &position(2));
          break;
        }
      case MOVETAN:
        {
					double winX, winY, winZ;
					const Vector3d tangent = rotation.col(0);
					Vector3d new_tangent;
					gluProject(position(0) + tangent(0), position(1) + tangent(1), position(2) + tangent(2), model_view, projection, viewport, &winX, &winY, &winZ);
					winX += move(0)*MOVE_TAN_CONST;
					winY += move(1)*MOVE_TAN_CONST;
					move(0) = 0.0;
					move(1) = 0.0;
					gluUnProject(winX, winY, winZ, model_view, projection, viewport, &new_tangent(0), &new_tangent(1), &new_tangent(2));
					new_tangent -= position;
					new_tangent.normalize();
					Matrix3d rot_diff;
					rotate_between_tangents(tangent, new_tangent, rot_diff);
					rotation = rot_diff * rotation;
          break;
        }
      case ROTATETAN:
        {
					double winX, winY, winZ;
					//Matrix3d old_rot = rot;
					const Vector3d tangent_normal = rotation.col(1);
					Vector3d new_tangent_normal;
					gluProject(position(0) + tangent_normal(0), position(1) + tangent_normal(1), position(2) + tangent_normal(2), model_view, projection, viewport, &winX, &winY, &winZ);
					winX += move(0)*ROTATE_TAN_CONST;
					winY += move(1)*ROTATE_TAN_CONST;
					move(0) = 0.0;
					move(1) = 0.0;
					gluUnProject(winX, winY, winZ, model_view, projection, viewport, &new_tangent_normal(0), &new_tangent_normal(1), &new_tangent_normal(2));
					new_tangent_normal -= position;
					//project this normal onto the plane normal to X (to ensure Y stays normal to X)
					new_tangent_normal -= new_tangent_normal.dot(rotation.col(0))*rotation.col(0);
					new_tangent_normal.normalize();
					rotation.col(1) = new_tangent_normal;
					rotation.col(2) = rotation.col(0).cross(new_tangent_normal);
					//new_rot = Eigen::AngleAxisd(angle_mismatch(rot, old_rot), rot.col(0).normalized())*old_rot;
          break;
        }
		}
}
