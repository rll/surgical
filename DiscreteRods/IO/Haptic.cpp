#include "Haptic.h"

Haptic::Haptic()
	: ControllerBase()
	, zero_position(0.0, 0.0, 0.0)
{
	haptic_button[UP] = haptic_button[DOWN] = false;
	last_haptic_button[UP] = last_haptic_button[DOWN] = false;
}

Haptic::~Haptic() {}

void Haptic::setRelativeTransform(const Vector3d& pos, const Matrix3d& rot)
{
	position = zero_position + pos;
	rotation = rot;
}

void Haptic::resetPosition(const Vector3d& reset_position)
{
	zero_position += reset_position - position;
}

void Haptic::setHapticButton(bool bttn, button_type bttn_type)
{
	last_haptic_button[bttn_type] = haptic_button[bttn_type];
	haptic_button[bttn_type] = bttn;
}

bool Haptic::hasButtonPressed(button_type bttn_type)
{
	bool result = (last_haptic_button[bttn_type] && !haptic_button[bttn_type]);
	if (result) 
		last_haptic_button[bttn_type] = haptic_button[bttn_type] = false;
	return result;
}

bool Haptic::hasButtonPressedAndReset(button_type bttn_type)
{
	bool result = (last_haptic_button[bttn_type] && !haptic_button[bttn_type]);
	if (result) 
		last_haptic_button[bttn_type] = haptic_button[bttn_type] = false;
	return result;
}
