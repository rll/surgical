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
      state.resize(8);
      double angZ, angY, angX;
      int ind = 0; 
      state(ind) = 8;
      ind += 1;
      euler_angles_from_rotation(rotation, angZ, angY, angX);
      for (int i = 0; i < 3; i++) {
        state(i+ind) = position(i);
      }
      ind += 3;
      Quaterniond quat_rotation(rotation);
      state(ind+0) = quat_rotation.w();
      state(ind+1) = quat_rotation.x();
      state(ind+2) = quat_rotation.y();
      state(ind+3) = quat_rotation.z(); 

      //state.segment(ind, 3) = 50 * rotation.col(0);
      //state(ind+0) = angZ;
      //state(ind+1) = angY;
      //state(ind+2) = angX;
    }

    void setState(VectorXd& state) 
    {
      assert(state(0) == state.size());
      double angZ, angY, angX;
      Matrix3d rot;
      Vector3d pos; 
      pos(0) = state(1);
      pos(1) = state(2);
      pos(2) = state(3);

      Quaterniond quat_rotation(state(4), state(5), state(6), state(7));
      rot = quat_rotation.toRotationMatrix(); 

      setTransform(pos, rot); 
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
