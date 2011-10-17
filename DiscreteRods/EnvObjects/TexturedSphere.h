#ifndef _TexturedSphere_h
#define _TexturedSphere_h

#include <IL/ilut.h>
#include "imageloader.h"

#include "EnvObject.h"


/** This class implements a sphere that can have an image on it. The 
	* sphere is meant to be used as a background enhancement, so collisions
	* are ignored for this object
	*/
class TexturedSphere : public EnvObject
{
	public:
		TexturedSphere(const Vector3d& pos, double r, string filename, World* w);
		TexturedSphere(const TexturedSphere& rhs, World* w);
		~TexturedSphere();
		
		//saving and loading from and to file
		void writeToFile(ofstream& file);
		TexturedSphere(ifstream& file, World* w);
		
		void setTransform(const Vector3d& pos, const Matrix3d& rot) {}
		
		void draw();
		
		//backup
		void backup();
		void restore();
		
		//collision
		bool capsuleIntersection(int capsule_ind, const Vector3d& start, const Vector3d& end, const double radius, vector<Intersection>& intersections) { return false; }
  	double capsuleRepulsionEnergy(const Vector3d& start, const Vector3d& end, const double radius) { return 0.0; }
  	void capsuleRepulsionEnergyGradient(const Vector3d& start, const Vector3d& end, const double radius, Vector3d& gradient) { return; }
		
	protected:
		double radius;
		World* world;
		GLUquadric *earth;
		string file_name;
		
		//backup
		double backup_radius;
		
		//needs to be backup
		//position
		//radius
		
		//needs to be restored
		//position
		//rotation
		//radius
		
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
