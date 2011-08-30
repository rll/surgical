#ifndef _Cursor_h
#define _Cursor_h

#include "EndEffector.h"

class EndEffector;

class World;

class Cursor
{
	public:
		friend class World;
		friend class EndEffector;
		
		Cursor(const Vector3d& pos, const Matrix3d& rot, World* w = NULL, EndEffector* ee = NULL);
		Cursor(const Cursor& rhs, World* w);
		~Cursor();

		void setTransform(const Vector3d& pos, const Matrix3d& rot, bool limit_displacement = false);

		void draw();		
		
		void attachDettach(bool limit_displacement = false);
		void attach(bool limit_displacement = false);
		void dettach(bool limit_displacement = false);
		bool isAttached() { return (end_eff!=NULL); }
		
		void openClose(bool limit_displacement = false);
		void setOpen(bool limit_displacement = false);
		void setClose(bool limit_displacement = false);		
		bool isOpen() { return open; }
		
		static const double height = 3.0;
		static const double radius = 2.0;
		
	protected:
		Vector3d position;
		Matrix3d rotation;
		World* world; //TODO ensure it is initialized properly
		EndEffector* end_eff;
		bool open;
};

#endif
