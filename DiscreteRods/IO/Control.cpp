#include "Control.h"
#include "../TrajectoryReader.h"

Control::Control()
  : translate(0.0, 0.0, 0.0)
	, rotate(Matrix3d::Identity())
{
	button[0] = false;
	button[1] = false;
}

Control::Control(const Vector3d& p_translate, const Quaterniond& p_rotate)
  : translate(p_translate)
	, rotate(p_rotate)
{
	button[0] = false;
	button[1] = false;
}


Control::Control(const Vector3d& p_translate, const Matrix3d& p_rotate)
  : translate(p_translate)
	, rotate(p_rotate)
{
	button[0] = false;
	button[1] = false;
}
Control::Control(const Control& rhs)  
  : translate(rhs.translate)
  , rotate(rhs.rotate)
{
  button[0] = rhs.button[0];
  button[1] = rhs.button[1];
}

Control::~Control() {}

void Control::setTranslate(const Vector3d& t)
{
	translate = t;
}

void Control::setRotate(const Quaterniond& r)
{
	rotate = r;
}

void Control::setRotate(const Matrix3d& r)
{
	rotate = r;
}

void Control::setButton(button_type bttn_type, bool value)
{
	button[bttn_type] = value;
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

bool Control::getButton(button_type bttn_type)
{
	return button[bttn_type];
}

void Control::writeToFile(ofstream& file)
{
	cout << "Control::writeToFile(ofstream& file): position and rotation members are deprecated. They are no longer used." << endl;
	Vector3d position;
	Matrix3d rotation;	
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
	cout << "Control::Control(ifstream& file): position and rotation members are deprecated. They are no longer used." << endl;
	Vector3d position;
	Matrix3d rotation;
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
