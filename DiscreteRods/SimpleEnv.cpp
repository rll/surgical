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

//textured sphere based on http://www.mfwweb.com/OpenGL/Loading_Textures/
SimpleTexturedSphere::SimpleTexturedSphere(Vector3d pos, float r, string filename)
: position(pos), radius(r) {
	earth = 0;
	ilInit();
  assert( LoadImageDevIL ((char*) filename.c_str(), &texture) );
  earth = gluNewQuadric();
}

SimpleTexturedSphere::~SimpleTexturedSphere() {
  //  Clear out the memory used by loading image files.
  printf("cleanup image memory:{");
  if (texture.id) {
    printf(" %d",texture.id); 
    ilDeleteImages(1, &texture.id);
  }
  printf(" }\n");

	//  Clear out the memory created by gluNewQuadric() calls.
  printf("cleanup gluQuadric memory:{");
  if (earth) {
    printf(" %p",earth); 
    gluDeleteQuadric(earth);
  } 
  printf(" }\n");
}

void SimpleTexturedSphere::draw() {
	glPushMatrix();
	glEnable(GL_COLOR_MATERIAL);
	//glColor3f(0.7, 0.7, 0.7);
	glColor3f(1.0, 1.0, 1.0);
	
	double transform[16] = {-0.5,  0, 0.866,     0,
													0.866, 0,   0.5,     0,
													0,     1,     0,     0,
													position(0), position(1), position(2), 1};

	// //Equivalent to the following
	// Matrix3d rotation = (Matrix3d) AngleAxisd(-M_PI/2.0, Vector3d(1.0,0.0,0.0));
	// rotation = AngleAxisd(240.0*M_PI/180.0, Vector3d(0.0,1.0,0.0)) * rotation;
	// 
	// double transform[16] = {rotation(0,0) , rotation(1,0) , rotation(2,0) , 0 ,
	// 												rotation(0,1) , rotation(1,1) , rotation(2,1) , 0 ,
	// 												rotation(0,2) , rotation(1,2) , rotation(2,2) , 0 ,
	// 												position(0), position(1), position(2), 1};

	glMultMatrixd(transform);
		
	glDisable(GL_CULL_FACE);
  glEnable (GL_TEXTURE_2D);

  gluQuadricTexture (earth, GL_TRUE);

  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

  glTexImage2D (GL_TEXTURE_2D, 0, 3, texture.w, texture.h, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, texture.p);

  gluSphere (earth, radius, 36, 72);
  
  glDisable (GL_TEXTURE_2D);
  glEnable(GL_CULL_FACE);
  
  glPopMatrix();
}

ILuint SimpleTexturedSphere::LoadImageDevIL (char *szFileName, struct TextureHandle *T)
{
    //When IL_ORIGIN_SET enabled, the origin is specified at an absolute 
    //position, and all images loaded or saved adhere to this set origin.
    ilEnable(IL_ORIGIN_SET);
    //sets the origin to be IL_ORIGIN_LOWER_LEFT when loading all images, so 
    //that any image with a different origin will be flipped to have the set 
    //origin.
    ilOriginFunc(IL_ORIGIN_LOWER_LEFT);

    //Now load the image file
    ILuint ImageNameID;
    ilGenImages(1, &ImageNameID);
    ilBindImage(ImageNameID);
    if (!ilLoadImage(szFileName)) return 0; // failure 

    T->id = ImageNameID;
    T->p = ilGetData(); 
    T->w = ilGetInteger(IL_IMAGE_WIDTH);
    T->h = ilGetInteger(IL_IMAGE_HEIGHT);
    
    printf("%s %d %d %d\n",szFileName,T->id,T->w,T->h);
    return 1; // success
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

void SimpleEnv::addObj(SimpleTexturedSphere* textured_sphere) { simple_textured_spheres.push_back(textured_sphere); }

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
	for (i=0; i<simple_textured_spheres.size(); i++)
		simple_textured_spheres[i]->draw();
}

void SimpleEnv::clearObjs() {
	threads.clear();
	cyls.clear();
	end_effs.clear();
	cursors.clear();
	simple_end_effs.clear();
	simple_spheres.clear();
	simple_textured_spheres.clear();
}

