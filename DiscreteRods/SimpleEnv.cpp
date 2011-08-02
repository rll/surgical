#include "SimpleEnv.h"

SimpleEndEffector::SimpleEndEffector(Vector3d pos, Matrix3d rot, double deg, float c0, float c1, float c2)
: position(pos), rotation(rot), degrees(deg), color0(c0), color1(c1), color2(c2) {}

void SimpleEndEffector::draw() {
	glPushMatrix();
	double transform[16] = { rotation(0,0) , rotation(1,0) , rotation(2,0) , 0 ,
													 rotation(0,1) , rotation(1,1) , rotation(2,1) , 0 ,
													 rotation(0,2) , rotation(1,2) , rotation(2,2) , 0 ,
													 position(0)   , position(1)   , position(2)   , 1 };
	glMultMatrixd(transform);
	
	glColor3f(0.3, 0.3, 0.0);
	double cylinder[4][3] = { {grab_offset-4.0, 0.0, 0.0} , {grab_offset-3.0, 0.0, 0.0} , {grab_offset, 0.0, 0.0} ,
														{grab_offset+1.0, 0.0, 0.0} };
	glePolyCylinder(4, cylinder, NULL, 1.6);
	
	glColor3f(color0, color1, color2);
	double grip_handle[4][3] = { {end-1.0, 0.0, 0.0} , {end, 0.0, 0.0} , {end+30.0, 0.0, 0.0} ,
															 {end+31.0, 0.0, 0.0} };
	glePolyCylinder(4, grip_handle, NULL, handle_r);

	glTranslatef(end, 0.0, 0.0);
	glRotatef(-degrees, 0.0, 0.0, 1.0);
	glTranslatef(-end, 0.0, 0.0);
	Matrix3d open_rot = (Matrix3d) AngleAxisd(-degrees*M_PI/180, rotation*Vector3d::UnitZ());
	Vector3d new_pos;
	Matrix3d new_rot;
	for (int piece=0; piece<pieces; piece++) {
		double r = 0.5+((double) piece)*((0.8*handle_r)-0.5)/((double) pieces-1);
		gleDouble grip_tip[4][3] = { {start+((double) piece)*h-1.0, r, 0.0} , {start+((double) piece)*h, r, 0.0} , {start+((double) piece+1)*h, r, 0.0} , {start+((double) piece+1)*h+1.0, r, 0.0} };
 		glePolyCylinder(4, grip_tip, NULL, r); 		
		new_rot = open_rot * rotation;
		new_pos = open_rot * rotation * Vector3d(-end, 0.0, 0.0) + rotation * Vector3d(end, 0.0, 0.0) + position;
	}
	
	glTranslatef(end, 0.0, 0.0);
	glRotatef(2*degrees, 0.0, 0.0, 1.0);
	glTranslatef(-end, 0.0, 0.0);
	for (int piece=0; piece<pieces; piece++) {
		double r = 0.5+((double) piece)*((0.8*handle_r)-0.5)/((double) pieces-1);
		gleDouble grip_tip[4][3] = { {start+((double) piece)*h-1.0, -r, 0.0} , {start+((double) piece)*h, -r, 0.0} , {start+((double) piece+1)*h, -r, 0.0} , {start+((double) piece+1)*h+1.0, -r, 0.0} };
 		glePolyCylinder(4, grip_tip, NULL, r);
 		new_rot = open_rot.transpose() * rotation;
		new_pos = open_rot.transpose() * rotation * Vector3d(-end, 0.0, 0.0) + rotation * Vector3d(end, 0.0, 0.0) + position;
	}
	glPopMatrix();
}



SimpleSphere::SimpleSphere(Vector3d pos, float r, float c0, float c1, float c2)
: position(pos), radius(r), color0(c0), color1(c1), color2(c2) {}

void SimpleSphere::draw() {
	glPushMatrix();
	double transform[16] = {1,0,0,0,
													0,1,0,0,
													0,0,1,0,
													position(0), position(1), position(2), 1};
	glMultMatrixd(transform);
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_COLOR_MATERIAL);
  glColor3f(color0, color1, color2);
  glutSolidSphere(radius, 20, 16);
  //glFlush ();
  glPopMatrix();
}



SimpleEnv::SimpleEnv() {
	clearObjs();
}

void SimpleEnv::addObj(ThreadConstrained* thread) {	threads.push_back(thread); }

void SimpleEnv::addObj(Cylinder* cyl) {	cyls.push_back(cyl); }

void SimpleEnv::addObj(EndEffector* end_eff) {	end_effs.push_back(end_eff); }

void SimpleEnv::addObj(Cursor* cursor) { cursors.push_back(cursor); }
		
void SimpleEnv::addObj(SimpleEndEffector end_eff) {	simple_end_effs.push_back(end_eff); }

void SimpleEnv::addObj(SimpleSphere sphere) {	simple_spheres.push_back(sphere); }

void SimpleEnv::drawObjs() {
	int i;
	for (i=0; i<threads.size(); i++)
		threads[i]->draw();
	for (i=0; i<cyls.size(); i++)
		cyls[i]->draw();
	for (i=0; i<end_effs.size(); i++)
		end_effs[i]->draw();
	for (i=0; i<cursors.size(); i++)
		cursors[i]->draw();
	for (i=0; i<simple_end_effs.size(); i++)
		simple_end_effs[i].draw();
	for (i=0; i<simple_spheres.size(); i++)
		simple_spheres[i].draw();
}

void SimpleEnv::clearObjs() {
	threads.clear();
	cyls.clear();
	end_effs.clear();
	cursors.clear();
	simple_end_effs.clear();
	simple_spheres.clear();
}

