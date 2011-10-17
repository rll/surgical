#ifndef _InfinitePlane_h
#define _InfinitePlane_h

#include <btBulletDynamicsCommon.h>

#include <IL/ilut.h>
#include "imageloader.h"

#include "EnvObject.h"

class InfinitePlane : public EnvObject
{
	public:
		InfinitePlane(const Vector3d& pos, const Vector3d& norm, float c0, float c1, float c2, World* w);
		InfinitePlane(const Vector3d& pos, const Vector3d& norm, string filename, World* w);
		InfinitePlane(const InfinitePlane& rhs, World* w);
		~InfinitePlane();
		
		//saving and loading from and to file
		void writeToFile(ofstream& file);
		InfinitePlane(ifstream& file, World* w);
		
		void setTransform(const Vector3d& pos, const Matrix3d& rot);
		
		void draw();
		
		//backup
		void backup();
		void restore();
		
		//collision
		btCollisionObject* col_obj;
	
	protected:
		Vector3d normal;
		World* world;
		static const double side = 1000.0;
		string file_name;
		
		//backup
		Vector3d backup_normal;
		
		//needs to be backup
		//position
		//normal
		
		//needs to be restored
		//position
		//rotation
		//normal
		
		struct TextureHandle {
			ILubyte *p;  /* pointer to image data loaded into memory */
			ILuint id;   /* unique DevIL id of image */
			ILint w;     /* image width */
			ILint h;     /* image height */
		};
		TextureHandle texture;
		ILuint LoadImageDevIL (char *szFileName, struct TextureHandle *T);
};

#endif
