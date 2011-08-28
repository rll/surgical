#ifdef NEVERDEFINED
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
		TexturedSphere(const Vector3d& pos, double r, string filename);
		TexturedSphere(const TexturedSphere& rhs);
		~TexturedSphere();
		
		// For saving and loading objects to and from files
		void writeToFile(ofstream& file);
		TexturedSphere(ifstream& file);
		void updateIndFromPointers(World* world) {}
		void linkPointersFromInd(World* world) {}
		
		void recomputeFromTransform(const Vector3d& pos, const Matrix3d& rot) {}
		void draw();
		bool capsuleIntersection(int capsule_ind, const Vector3d& start, const Vector3d& end, const double radius, vector<Intersection>& intersections) { return false; }
  	double capsuleRepulsionEnergy(const Vector3d& start, const Vector3d& end, const double radius) { return 0.0; }
  	void capsuleRepulsionEnergyGradient(const Vector3d& start, const Vector3d& end, const double radius, Vector3d& gradient) { return; }
		
	protected:
		double radius;
		GLUquadric *earth;
		string file_name;
		
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
#endif
