#include "Haptic.h"

Haptic::Haptic()
	: ControlBase()
	, zero_position(0.0, 0.0, 0.0)
{}

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
