/* 	3D Vector Class
	wan, for knot simulation
 */

#ifndef __VEC3D__
#define __VEC3D__
#include <iostream>
#include <math.h>

class Vec3d {
public:
	double x, y, z;

	Vec3d() {
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
	}

	Vec3d(double nx, double ny, double nz) {
		x = nx;
		y = ny;
		z = nz;
	}

	Vec3d(const Vec3d& v) {
		x = v.x;
		y = v.y;
		z = v.z;
	}

	// assignment operators
	Vec3d &operator = (const Vec3d& v) {
		x = v.x;
		y = v.y;
		z = v.z;
		return(*this);
	}

	Vec3d &operator += (const Vec3d& v) {
		x += v.x;
		y += v.y;
		z += v.z;
		return(*this);
	}

	Vec3d &operator -= (const Vec3d& v) {
		x -= v.x;
		y -= v.y;
		z -= v.z;
		return(*this);
	}

	Vec3d &operator *= (const double d) {
		x *= d;
		y *= d;
		z *= d;
		return(*this);
	}

	Vec3d &operator /= (const double d) {
		x /= d;
		y /= d;
		z /= d;
		return(*this);
	}
	
	// arithmetic operators
	friend Vec3d operator + (const Vec3d& v1, const Vec3d& v2) {
		return(Vec3d(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z));
	}

	friend Vec3d operator - (const Vec3d& v1, const Vec3d& v2) {
		return(Vec3d(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z));
	}

	friend Vec3d operator / (const Vec3d& v, const double d) {
		return(Vec3d(v.x / d, v.y / d, v.z / d));
	}

	friend Vec3d operator * (const Vec3d& v, const double d) {
		return(Vec3d(v.x * d, v.y * d, v.z * d));
	}
	
	friend Vec3d operator - (const Vec3d& v1) {
		return(Vec3d(-v1.x, -v1.y, -v1.z));
	}
	
	// equality operators
	friend bool operator == (const Vec3d& v1, const Vec3d& v2) {
		return( v1.x == v2.x &&
			v1.y == v2.y &&
			v1.z == v2.z);
	}

	friend bool operator != (const Vec3d& v1, const Vec3d& v2) {
		return !(v1==v2);
	}

	// i/o
	friend std::ostream& operator<<(std::ostream &os, const Vec3d& v) {
		return(os << v.x << "," << v.y << "," << v.z);
	}
	
	double length() {
		return(sqrt(x*x + y*y + z*z));
	}

	void normalize() {
		double l = length();
		x /= l;
		y /= l;
		z /= l;
	}
};

typedef Vec3d Point;
#endif
