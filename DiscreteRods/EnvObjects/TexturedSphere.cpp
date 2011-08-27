#include "TexturedSphere.h"
#include "../threadpiece_discrete.h"

//textured sphere based on http://www.mfwweb.com/OpenGL/Loading_Textures/
//Image should be saved as BMP 24 bits R8 G8 B8, RGB mode
TexturedSphere::TexturedSphere(const Vector3d& pos, double r, string filename)
	: EnvObject(pos, Matrix3d::Identity(), 0.6, 0.6, 0.6, TEXTURED_SPHERE)
	, radius(r)
	, file_name(filename)
{
	earth = 0;
	ilInit();
  if (! LoadImageDevIL ((char*) filename.c_str(), &texture) )
  	cerr << "Failed to load texture from filename " << filename << endl;
  else
  	earth = gluNewQuadric();
}

TexturedSphere::TexturedSphere(const TexturedSphere& rhs)
	: EnvObject(rhs.position, rhs.rotation, rhs.color0, rhs.color1, rhs.color2, rhs.type)
	, radius(rhs.radius)
	, file_name(rhs.file_name)
{
	if (type != TEXTURED_SPHERE)
		cerr << "it is not textrures sphere" << endl; //TODO
	earth = 0;
	ilInit();
  if (! LoadImageDevIL ((char*) file_name.c_str(), &texture) )
  	cerr << "Failed to load texture from filename " << file_name << endl;
  else
  	earth = gluNewQuadric();
}

TexturedSphere::~TexturedSphere()
{
  //  Clear out the memory used by loading image files.
  if (texture.id)
    ilDeleteImages(1, &texture.id);

	//  Clear out the memory created by gluNewQuadric() calls.
  if (earth)
    gluDeleteQuadric(earth); 
}

void TexturedSphere::writeToFile(ofstream& file)
{
	file << type << " ";
	for (int i=0; i<3; i++)
		file << position(i) << " ";

  file << radius << " " << file_name << " ";

  file << "\n";
}

TexturedSphere::TexturedSphere(ifstream& file)
{
	rotation = Matrix3d::Identity();
	color0 = color1 = color2 = 0.6;
	type = TEXTURED_SPHERE;
  
	for (int i=0; i<3; i++)
		file >> position(i);

  file >> radius >> file_name;
  
  earth = 0;
	ilInit();
  if (! LoadImageDevIL ((char*) file_name.c_str(), &texture) )
  	cerr << "Failed to load texture from file_name " << file_name << endl;
  else
  	earth = gluNewQuadric();
}

void TexturedSphere::draw()
{
	if (!earth) {
		glDisable(GL_CULL_FACE);
		drawSphere(position, radius, color0, color1, color2);
		glEnable(GL_CULL_FACE);
	} else {	
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
}

ILuint TexturedSphere::LoadImageDevIL (char *szFileName, struct TextureHandle *T)
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
    
    //printf("%s %d %d %d\n",szFileName,T->id,T->w,T->h);
    return 1; // success
}
