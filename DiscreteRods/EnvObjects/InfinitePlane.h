#ifndef _InfinitePlane_h
#define _InfinitePlane_h

#include <IL/ilut.h>
#include "imageloader.h"

#include "EnvObject.h"

class InfinitePlane : public EnvObject
{
	public:
		InfinitePlane(const Vector3d& pos, const Vector3d& norm, float c0, float c1, float c2);
		InfinitePlane(const Vector3d& pos, const Vector3d& norm, string filename);
		~InfinitePlane();
		
		// For saving and loading objects to and from files
		void writeToFile(ofstream& file);
		InfinitePlane(ifstream& file);
		void updateIndFromPointers(World* world) {}
		void linkPointersFromInd(World* world) {}
		
		void recomputeFromTransform(const Vector3d& pos, const Matrix3d& rot);
		void draw();
		bool capsuleIntersection(int capsule_ind, const Vector3d& start, const Vector3d& end, const double radius, vector<Intersection>& intersections);
  	double capsuleRepulsionEnergy(const Vector3d& start, const Vector3d& end, const double radius);
  	void capsuleRepulsionEnergyGradient(const Vector3d& start, const Vector3d& end, const double radius, Vector3d& gradient);
	
	protected:
		Vector3d normal;
		double side;
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
