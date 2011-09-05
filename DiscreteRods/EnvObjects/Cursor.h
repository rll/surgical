#ifndef _Cursor_h
#define _Cursor_h

#include "EnvObject.h"

class EndEffector;

class Cursor
{
	public:
		friend class World;
		friend class EndEffector;
		
		Cursor(const Vector3d& pos, const Matrix3d& rot, World* w = NULL, EndEffector* ee = NULL);
		Cursor(const Cursor& rhs, World* w);
		~Cursor();
		
		//saving and loading from and to file
		void writeToFile(ofstream& file);
		Cursor(ifstream& file, World* w);

		void setTransform(const Vector3d& pos, const Matrix3d& rot, bool limit_displacement = false);
		const Vector3d& getPosition() const;
		const Matrix3d& getRotation() const;

		void draw();		
		
		void attachDettach(bool limit_displacement = false);
		void attach(bool limit_displacement = false);
		void dettach(bool limit_displacement = false);
		bool isAttached() { return (end_eff!=NULL); }
		
		void openClose(bool limit_displacement = false);
		void setOpen(bool limit_displacement = false);
		void setClose(bool limit_displacement = false);		
		bool isOpen() { return open; }

    void getState(VectorXd& state)
    {
      state.resize(6);
      double angZ, angY, angX;
      euler_angles_from_rotation(rotation, angZ, angY, angX);
      for (int i = 0; i < 3; i++) {
        state(i) = position(i);
      }
      state(3) = angZ;
      state(4) = angY;
      state(5) = angX;
    }  
		
		static const double height = 3.0;
		static const double radius = 1.2;
		
	protected:
		void drawColoredCapsule(const Vector3d& start_pos, const Vector3d& end_pos);
	
		Vector3d position;
		Matrix3d rotation;
		World* world; //TODO ensure it is initialized properly
		EndEffector* end_eff;
		bool open;
		object_type type;
};

#endif
