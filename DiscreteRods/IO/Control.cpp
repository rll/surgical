#include "Control.h"
#include "../TrajectoryReader.h"

Control::Control(Vector3d start_position, Matrix3d start_rotation)
  : position(start_position)
  , rotation(start_rotation)
  , translate(0.0, 0.0, 0.0)
	, rotate(Matrix3d::Identity())
{
	button[0] = false;
	button[1] = false;
}

Control::~Control() {}

void Control::setControl(ControllerBase* controller)
{
  translate = controller->getPosition() - position;
  rotate = Quaterniond(controller->getRotation().transpose() * rotation);
  position = controller->getPosition();
  rotation = controller->getRotation();
  button[UP] = controller->hasButtonPressedAndReset(UP);
  button[DOWN] = controller->hasButtonPressedAndReset(DOWN);
}

void Control::setTranslate(const Vector3d& t)
{
	translate = t;
}

void Control::setRotate(const Matrix3d& r)
{
	rotate = r;
}

void Control::setInitialTransform(const Vector3d& pos, const Matrix3d& rot)
{
	position = pos;
	rotation = rot;
	setNoMotion();
}

void Control::setNoMotion()
{
	translate = Vector3d::Zero();
  rotate = Quaterniond(Matrix3d::Identity());
}

void Control::getControlVector(VectorXd& control)
{
  control.resize(9);
  control.segment(0, 3) = translate;
  control(3) = rotate.w();
  control(4) = rotate.x();
  control(5) = rotate.y();
  control(6) = rotate.z();
  control(7) = button[0];
  control(8) = button[1];
}

const Vector3d& Control::getTranslate() const
{
	return translate;
}

const Quaterniond& Control::getRotate() const
{
	return rotate;
}

const Vector3d& Control::getPosition() const
{
	return position;
}

const Matrix3d& Control::getRotation() const
{
	return rotation;
}

bool Control::getButton(button_type bttn_type)
{
	return button[bttn_type];
}

void Control::writeToFile(ofstream& file)
{
	for (int i=0; i<3; i++)
		file << position(i) << " ";
	for (int r=0; r < 3; r++)
	  for (int c=0; c < 3; c++)
	    file << rotation(r,c) << " ";
	for (int i=0; i<3; i++)
		file << translate(i) << " ";
	file << rotate.w() << " " << rotate.x() << " " << rotate.y() << " " << rotate.z() << " ";
	file << button[0] << " " << button[1] << " ";		
}

Control::Control(ifstream& file)
{
	for (int i=0; i<3; i++)
		file >> position(i);
	for (int r=0; r < 3; r++)
	  for (int c=0; c < 3; c++)
	    file >> rotation(r,c);
	for (int i=0; i<3; i++)
		file >> translate(i);
	double w, x, y, z;
	file >> w >> x >> y >> z;
	rotate = Quaterniond(w, x, y, z);
	file >> button[0] >> button[1];	
}
