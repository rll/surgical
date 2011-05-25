#include "glThread_stripe.h"
#include "text3d.h"

GLThread::GLThread() {
/*
	vector<Vector3d> directions;
	directions.push_back(Vector3d::UnitX());
	Vector3d next;
	
	next << 1.0 , 0.0 , -1.0;
	directions.push_back(next.normalized());

	next << 1.0 , 0.0 , 0.0;
	directions.push_back(next.normalized());


	next << 1.0 , 0.0 , 1.0;
	directions.push_back(next.normalized());
	
	next << 1.0 , -1.0 , 0.0;
	directions.push_back(next.normalized());


	next << 1.0 , 0.0 , 0.0;
	directions.push_back(next.normalized());
	

	next << 1.0 , 1.0 , 0.0;
	directions.push_back(next.normalized());


	directions.push_back(Vector3d::UnitX());

  vector<Vector3d> vertices;
  vector<double> angles;

	vertices.push_back(Vector3d::Zero());
	angles.push_back(0);
	for (int i=0; i < directions.size(); i++)
	{
		vertices.push_back(vertices.back()+_rest_length*directions[i]);
		angles.push_back(0);
	}
	*/

	
  int numInit = 3;
  double noise_factor = 0.0;

  vector<Vector3d> vertices;
  vector<double> angles;

  vertices.push_back(Vector3d::Zero());
  angles.push_back(0.0);
  //push back unitx so first tangent matches start_frame
  vertices.push_back(Vector3d::UnitX()*DEFAULT_REST_LENGTH);
  angles.push_back(0.0);

  Vector3d direction;
  direction(0) = 1.0;
  direction(1) = 0.0;
  direction(2) = -1.0;
  direction.normalize();
  for (int i=0; i < numInit; i++)
    {
      Vector3d noise( ((double)(rand()%10000)) / 10000.0, ((double)(rand()%10000)) / 10000.0, ((double)(rand()%10000)) / 10000.0);
      noise *= noise_factor;
      Vector3d next_Vec = vertices.back()+(direction+noise).normalized()*DEFAULT_REST_LENGTH;
      vertices.push_back(next_Vec);
      angles.push_back(0.0);

      //std::cout << positions[i] << std::endl << std::endl;
    }



  //change direction
  direction(0) = 1.0;
  direction(1) = 0.0;
  direction(2) = 1.0;
  direction.normalize();

  for (int i=0; i < numInit; i++)
    {
      Vector3d noise( ((double)(rand()%10000)) / 10000.0, ((double)(rand()%10000)) / 10000.0, ((double)(rand()%10000)) / 10000.0);
      noise *= noise_factor;
      Vector3d next_Vec = vertices.back()+(direction+noise).normalized()*DEFAULT_REST_LENGTH;
      vertices.push_back(next_Vec);
      angles.push_back(0.0);

    }

  //push back unitx so last tangent matches end_frame
  vertices.push_back(vertices.back()+Vector3d::UnitX()*DEFAULT_REST_LENGTH);
  angles.push_back(0.0);


  angles.resize(vertices.size());

	

  rotations[0] = Matrix3d::Identity();
  rotations[1] = Matrix3d::Identity();


  _thread = new Thread(vertices, angles, rotations[0], rotations[1]);

#ifdef ISOTROPIC
  to_set_bend = _thread->get_bend_coeff();
#else
  to_set_B = _thread->get_bend_matrix();
#endif
  to_set_twist = _thread->get_twist_coeff();
  to_set_grav = _thread->get_grav_coeff();


#ifdef ISOTROPIC
    _thread->set_coeffs_normalized(to_set_bend, to_set_twist, to_set_grav);
#else
    _thread->set_coeffs_normalized(to_set_B, to_set_twist, to_set_grav);
#endif
	_thread->minimize_energy();

  InitContour();
}

GLThread::GLThread(Thread* _thread) {

  // pass in transformed vertices, the same twists, identity start_rot

  // normalize the positions, rotations so that we are at 0,0, identity rot
  Vector3d start = _thread->start_pos();
  Matrix3d start_inv = _thread->start_rot().inverse();

  vector<Vector3d> vertices;
  vector<double> twists;
  _thread->get_thread_data(vertices, twists);
  for (int i=0; i < vertices.size(); i++)
  {
    Vector3d vert = vertices[i] - start;
    vertices[i] = start_inv*vert;
  }

  Matrix3d rot = Matrix3d::Identity();
  this->_thread = new Thread(vertices, twists, rot);

#ifdef ISOTROPIC
  to_set_bend = _thread->get_bend_coeff();
#else
  to_set_B = _thread->get_bend_matrix();
#endif
  to_set_twist = _thread->get_twist_coeff();
  to_set_grav = _thread->get_grav_coeff();




  InitContour();
}


//void GLThread::GetConfiguration(int* size, double* pts_cpy[][], double* twist_cpy[]) {
void GLThread::DrawThread(bool disable_texture) {
if (disable_texture)
	glDisable(GL_TEXTURE_2D);

  updateThreadPoints();

  double pts_cpy[points.size()+2][3];
  double twist_cpy[points.size()+2];

  for (int i=0; i < points.size()-1; i++)
    {
      pts_cpy[i+1][0] = points[i](0)-display_start_pos(0);
      pts_cpy[i+1][1] = points[i](1)-display_start_pos(1);
      pts_cpy[i+1][2] = points[i](2)-display_start_pos(2);
      twist_cpy[i+1] = -(360.0/(2.0*M_PI))*twist_angles[i];
    }

  //std::cout << (points.back() - points.front()).norm() << std::endl;

  //add first and last point
  pts_cpy[0][0] = pts_cpy[1][0] -rotations[0](0,0);
  pts_cpy[0][1] = pts_cpy[1][1] -rotations[0](1,0);
  pts_cpy[0][2] = pts_cpy[1][2] -rotations[0](2,0);
  twist_cpy[0] = -(360.0/(2.0*M_PI))*twist_angles[0];

  pts_cpy[points.size()][0] = (double)positions[1](0)-display_start_pos(0);
  pts_cpy[points.size()][1] = (double)positions[1](1)-display_start_pos(1);
  pts_cpy[points.size()][2] = (double)positions[1](2)-display_start_pos(2);
  twist_cpy[points.size()] = twist_cpy[points.size()-1];


  pts_cpy[points.size()+1][0] = pts_cpy[points.size()][0]+rotations[1](0,0);
  pts_cpy[points.size()+1][1] = pts_cpy[points.size()][1]+rotations[1](1,0);
  pts_cpy[points.size()+1][2] = pts_cpy[points.size()][2]+rotations[1](2,0);
  twist_cpy[points.size()+1] = twist_cpy[points.size()];

  //(*size) = points.size()+2;
	
	//gleTextureMode (GLE_TEXTURE_ENABLE | GLE_TEXTURE_VERTEX_MODEL_CYL);

  gleTwistExtrusion(NUM_PTS_CONTOUR,
                    contour,
                    contour_norms,
                    NULL,
                    points.size()+2,
                    pts_cpy,
                    0x0,
                    twist_cpy);
	

if (disable_texture)
	glEnable(GL_TEXTURE_2D);
	
	/*
	gleSetNumSides(500);
  glePolyCylinder_c4f(points.size()+2,
                    pts_cpy,
                    0x0,
                    1.0);
*/


/*
#ifdef ISOTROPIC
    _thread->set_coeffs_normalized(to_set_bend, to_set_twist, to_set_grav);
#else
    _thread->set_coeffs_normalized(to_set_B, to_set_twist, to_set_grav);
    std::cout << "coeffs " << to_set_B << " " << to_set_twist << " " << to_set_grav << std::endl;
#endif
*/
}

void GLThread::minimize_energy() {
  _thread->minimize_energy();
  updateThreadPoints();
}



void GLThread::updateThreadPoints()
{
  _thread->get_thread_data(points, twist_angles);
  positions[0] = points.front();
  positions[1] = points.back();

  tangents[0] = points[1] - points[0];
  tangents[0].normalize();
  tangents[1] = points[points.size()-1] - points[points.size()-2];
  tangents[1].normalize();

  rotations[0] = _thread->start_rot();
	rotations[0] = Eigen::AngleAxisd(M_PI/2.0, Vector3d::UnitX())*rotations[0];
  rotations[1] = _thread->end_rot();
	rotations[1] = Eigen::AngleAxisd(M_PI/2.0, Vector3d::UnitX())*rotations[1];
}


void GLThread::ApplyUserInput(float move_end[], float tangent_end[], float tangent_rotation_end[])
{
  GLdouble model_view[16];
  glGetDoublev(GL_MODELVIEW_MATRIX, model_view);

  GLdouble projection[16];
  glGetDoublev(GL_PROJECTION_MATRIX, projection);

  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);


  double winX, winY, winZ;

  //change end positions
  Vector3d new_end_pos;
  gluProject(positions[1](0), positions[1](1), positions[1](2), model_view, projection, viewport, &winX, &winY, &winZ);
  winX += move_end[0];
  winY += move_end[1];
  move_end[0] = 0.0;
  move_end[1] = 0.0;
  gluUnProject(winX, winY, winZ, model_view, projection, viewport, &new_end_pos(0), &new_end_pos(1), &new_end_pos(2));
  //    std::cout << "X: " << positions[1](0) << " Y: " << positions[1](1) << " Z: " << positions[1](2) << std::endl;

  //change tangents
  Vector3d new_end_tan;
  gluProject(positions[1](0)+tangents[1](0),positions[1](1)+tangents[1](1), positions[1](2)+tangents[1](2), model_view, projection, viewport, &winX, &winY, &winZ);
  winX += tangent_end[0];
  winY += tangent_end[1];
  tangent_end[0] = 0.0;
  tangent_end[1] = 0.0;
  gluUnProject(winX, winY, winZ, model_view, projection, viewport, &new_end_tan(0), &new_end_tan(1), &new_end_tan(2));
  new_end_tan -= positions[1];
  new_end_tan.normalize();

  positions[1] = new_end_pos;

  Matrix3d rotation_new_tan;
  rotate_between_tangents(tangents[1], new_end_tan, rotation_new_tan);
  rotations[1] = rotation_new_tan*rotations[1];


  //check rotation around tangent
  Vector3d tangent_normal_rotate_around = rotations[1].col(1);
  Vector3d new_end_tan_normal;
  gluProject(positions[1](0)+tangent_normal_rotate_around(0), positions[1](1)+tangent_normal_rotate_around(1), positions[1](2)+tangent_normal_rotate_around(2), model_view, projection, viewport, &winX, &winY, &winZ);
  winX += tangent_rotation_end[0];
  winY += tangent_rotation_end[1];
  tangent_rotation_end[0] = 0.0;
  tangent_rotation_end[1] = 0.0;
  gluUnProject(winX, winY, winZ, model_view, projection, viewport, &new_end_tan_normal(0), &new_end_tan_normal(1), &new_end_tan_normal(2));
  new_end_tan_normal -= positions[1];
  //project this normal onto the plane normal to X (to ensure Y stays normal to X)
  new_end_tan_normal -= new_end_tan_normal.dot(rotations[1].col(0))*rotations[1].col(0);
  new_end_tan_normal.normalize();


  rotations[1].col(1) = new_end_tan_normal;
  rotations[1].col(2) = rotations[1].col(0).cross(new_end_tan_normal);


	rotations[0] = Eigen::AngleAxisd(-M_PI/2.0, Vector3d::UnitX())*rotations[0];
	rotations[1] = Eigen::AngleAxisd(-M_PI/2.0, Vector3d::UnitX())*rotations[1];

  //change thread
  _thread->set_constraints(positions[0], rotations[0], positions[1], rotations[1]);
  //thread->set_end_constraint(positions[1], rotations[1]);

  // std::cout <<"CONSTRAINT END:\n" << rotations[1] << std::endl;
  _thread->minimize_energy();
  updateThreadPoints();
  // std::cout <<"minimized END:\n" << rotations[1] << std::endl;
}


void GLThread::DrawAxes()
{
	//DrawAxesAtPoint(positions[0], rotations[0], material);
	//DrawAxesAtPoint(positions[1], rotations[1], material);
	DrawAxesAtPoint(0, material, false);
	DrawAxesAtPoint(_thread->num_pieces()-1, material, false);
}


void GLThread::DrawAxesAtPoint(int ind, ColorCode color_axis, bool skipX)
{
	if (color_axis == bishop)
		DrawAxesAtPoint(_thread->vertex_at_ind(ind), _thread->bishop_at_ind(ind), color_axis, skipX);
	else
		DrawAxesAtPoint(_thread->vertex_at_ind(ind), _thread->material_at_ind(ind), color_axis, skipX);
}


void GLThread::DrawAxesAtPoint(const Vector3d& pt, const Matrix3d& rot_in, ColorCode color_axis, bool skipX)
{
	glDisable(GL_TEXTURE_2D);
	glPushMatrix();
	//Matrix3d rot = Eigen::AngleAxisd(M_PI/2.0, Vector3d::UnitX())*rot_in;
	
	Matrix3d rot = Eigen::AngleAxisd(M_PI/2.0, rot_in.col(0))*rot_in;


	const double radius = 0.15;
	const double radius_cone_max = 0.4;


  Vector3d diff_pos = pt-display_start_pos;
  double rotation_scale_factor = 6.0;
	double cone_scale_factor = 0.07;
  Matrix3d rotations_project = rot*rotation_scale_factor;
	double pts[4][3];
	double pts_cone[4][3];

	double radius_cone[4];
	radius_cone[0] = radius_cone_max*2;
	radius_cone[1] = radius_cone_max;
	radius_cone[2] = 1e-3;
	radius_cone[3] = 0;

	


	for (int i=0; i < 3; i++)
	{
		pts[0][i] = diff_pos(i)-rotations_project(i,0);
		pts[1][i] = diff_pos(i);
	}


	for (int coord=0; coord < 3; coord++)
	{
		if (coord == 0 && skipX)
			continue;
		for (int i=0; i < 3; i++)
		{
			pts[2][i] = diff_pos(i) + rotations_project(i,coord);
			pts[3][i] = diff_pos(i) + 2*rotations_project(i,coord);

			pts_cone[0][i] = diff_pos(i) + rotations_project(i,coord)*(1.0-cone_scale_factor);
			pts_cone[1][i] = diff_pos(i) + rotations_project(i,coord);
			pts_cone[2][i] = diff_pos(i) + rotations_project(i,coord)*(1.0+cone_scale_factor);
			pts_cone[3][i] = diff_pos(i) + rotations_project(i,coord)*(1.0+cone_scale_factor*2);

		}

		float thread_color_array[4][3];
		float stripe_color_array[4][3];
		for (int i=0; i < 4; i++)
		{
			for (int j=0; j < 3; j++)
			{
				thread_color_array[i][j] = thread_color_float[j];
				stripe_color_array[i][j] = stripe_color_float[j];
			}
		}

		const float alpha = 0.85;

			
	// gleTextureMode (GLE_TEXTURE_VERTEX_MODEL_CYL);
		if (color_axis == material)
		{
			if (coord == 0)
				glColor4f(0.7, 0.2, 0, alpha);
			else if (coord == 1)
				glColor4f((thread_color_float[0])*0.8, (thread_color_float[1])*0.8, (thread_color_float[2])*0.8, alpha);
			else
				glColor4f((stripe_color_float[0])*0.8, (stripe_color_float[1])*0.8, (stripe_color_float[2])*0.8, alpha);
		} else if (color_axis == bishop) {
			if (coord == 0)
				glColor4f(0.7, 0.2, 0, alpha);
			else if (coord == 1)
				glColor3d(0.5, 0.5, 0.5);
			else
				glColor3d(0.17, 0.17, 0.17);
		} else {
			if (coord == 0)
				glColor3d(1.0, 0.0, 0.0);
			else if (coord == 1)
				glColor3d(0.0, 1.0, 0.0);
			else
				glColor3d(0.0, 0.0, 1.0);
		}

		glePolyCylinder(4, pts, 0x0,radius);
		glePolyCone(4, pts_cone, 0x0, radius_cone);

	}
	glPopMatrix();
	glEnable(GL_TEXTURE_2D);
}

void GLThread::DrawDownvecAtInd(int ind)
{
	glDisable(GL_TEXTURE_2D);
	glPushMatrix();
	glColor3d(0.8, 0.1, 0.0);


	const double radius = 0.20;
	const double radius_cone_max = 0.45;

  Vector3d diff_pos = _thread->vertex_at_ind(ind)-display_start_pos;
  double length_factor = 8.0;
	double cone_scale_factor = 0.07;
  Vector3d down_vec = -Vector3d::UnitZ()*length_factor;
	double pts[4][3];
	double pts_cone[4][3];

	double radius_cone[4];
	radius_cone[0] = radius_cone_max*2;
	radius_cone[1] = radius_cone_max;
	radius_cone[2] = 1e-3;
	radius_cone[3] = 0;


	for (int i=0; i < 3; i++)
	{
		pts[0][i] = diff_pos(i)-down_vec(i);
		pts[1][i] = diff_pos(i);

		pts[2][i] = diff_pos(i) + down_vec(i);
		pts[3][i] = diff_pos(i) + 2*down_vec(i);

		pts_cone[0][i] = diff_pos(i) + down_vec(i)*(1.0-cone_scale_factor);
		pts_cone[1][i] = diff_pos(i) + down_vec(i);
		pts_cone[2][i] = diff_pos(i) + down_vec(i)*(1.0+cone_scale_factor);
		pts_cone[3][i] = diff_pos(i) + down_vec(i)*(1.0+cone_scale_factor*2);

	}

	glePolyCylinder(4, pts, 0x0,radius);
	glePolyCone(4, pts_cone, 0x0, radius_cone);

	glPopMatrix();
	glEnable(GL_TEXTURE_2D);
}




void GLThread::DrawSpheresAtLinks()
{
	updateThreadPoints();
	double radius = 1.15;
	GLUquadricObj *quadric=gluNewQuadric();
  gluQuadricNormals(quadric, GLU_SMOOTH);


	glDisable(GL_TEXTURE_2D);
	float pt_loc[3];
	for (int i=1; i < points.size()-1; i++)
	{
		for (int j=0; j < 3; j++)
		{
			pt_loc[j] = (float)points[i](j)-display_start_pos(j);
		}
		
		glPushMatrix();
		glColor3d(0.3, 0.3, 0.3);
		glTranslatef(pt_loc[0], pt_loc[1], pt_loc[2]);
		gluSphere(quadric, radius, 100, 100);
			
		
		glPopMatrix();


		
	}

	gluDeleteQuadric(quadric);


	glEnable(GL_TEXTURE_2D);

}


void GLThread::DrawArcBetweenMats(Vector3d& center, Vector3d& dir1, Vector3d& dir2, double inner_radius, double outer_radius)
{
	string s;
	DrawArcBetweenMats(center, dir1, dir2, inner_radius, outer_radius, s);
}

void GLThread::DrawArcBetweenMats(Vector3d& center, Vector3d& dir1, Vector3d& dir2, double inner_radius, double outer_radius, string& str)
{
	updateThreadPoints();
	GLUquadricObj *quadric=gluNewQuadric();
  gluQuadricNormals(quadric, GLU_SMOOTH);

	glDisable(GL_TEXTURE_2D);

	glPushMatrix();
	float pt_loc[3];
	for (int j=0; j < 3; j++)
	{
		pt_loc[j] = (float)center(j)-display_start_pos(j);
	}

	glTranslatef(pt_loc[0], pt_loc[1], pt_loc[2]);


	Matrix3d rot_mat;
	rot_mat.col(0) = dir1;
	Vector3d newy = dir2;
	make_vectors_perpendicular(rot_mat.col(0), newy);
	rot_mat.col(1) = newy.normalized();
	rot_mat.col(2) = rot_mat.col(0).cross(rot_mat.col(1)).normalized();
	//rot_mat = Eigen::AngleAxisd(M_PI/2.0, rot_mat.col(0))*rot_mat;


	double angx, angy, angz;
	euler_angles_from_rotation(rot_mat, angz, angy, angx);
	glRotatef(angz*180.0/M_PI, 0.0, 0.0, 1.0);
	glRotatef(angy*180.0/M_PI, 0.0, 1.0, 0.0);
	glRotatef(angx*180.0/M_PI, 1.0, 0.0, 0.0);


	double max_ang = abs(acos(dir1.dot(dir2)))*180.0/M_PI;

	gluPartialDisk(quadric, inner_radius, outer_radius, 20, 20, 90, max_ang);

	glRasterPos3f (0,-3.0,3.0);
	for (string::iterator s = str.begin(); s != str.end(); ++s)
	{
		char c = *s;
		glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, c);
	}




	glPopMatrix();


	glEnable(GL_TEXTURE_2D);



}

void GLThread::DrawAngleArcAtPoint(int i)
{
	glColor3d(0.8, 0.1, 0.0);
	Vector3d dir2 = -1.0*_thread->edge_at_ind(i-1);
	Vector3d dir1 = _thread->edge_at_ind(i);
	dir1.normalize();
	dir2.normalize();
	Vector3d center = _thread->vertex_at_ind(i);

	//string s = "Bend Angle";
	DrawArcBetweenMats(center, dir1, dir2, 2.2, 2.7);


}



void GLThread::InitContour ()
{
  int style;
  /* configure the pipeline */
  style = TUBE_JN_CAP;
  style |= TUBE_CONTOUR_CLOSED;
  style |= TUBE_NORM_FACET;
  style |= TUBE_JN_ANGLE;
  gleSetJoinStyle (style);

  int i;
  double contour_scale_factor = 0.3;



#ifdef ISOTROPIC
	// outline of extrusion
	i=0;
	double radius = 1.0;
	while (i < NUM_PTS_CONTOUR)
	{
		double x = radius*cos(2*M_PI*((double)i/(double) (NUM_PTS_CONTOUR-1)));
		double y = radius*sin(2*M_PI* ((double)i/(double)(NUM_PTS_CONTOUR-1)));
		CONTOUR(x, y);
	}




#else
   // outline of extrusion
   i=0;
   CONTOUR (1.0*contour_scale_factor, 0.0*contour_scale_factor);
   CONTOUR (1.0*contour_scale_factor, 0.5*contour_scale_factor);
   CONTOUR (1.0*contour_scale_factor, 1.0*contour_scale_factor);
   CONTOUR (1.0*contour_scale_factor, 2.0*contour_scale_factor);
   CONTOUR (1.0*contour_scale_factor, 2.9*contour_scale_factor);
   CONTOUR (0.9*contour_scale_factor, 3.0*contour_scale_factor);
   CONTOUR (0.0*contour_scale_factor, 3.0*contour_scale_factor);
   CONTOUR (-0.9*contour_scale_factor, 3.0*contour_scale_factor);
   CONTOUR (-1.0*contour_scale_factor, 2.9*contour_scale_factor);

   CONTOUR (-1.0*contour_scale_factor, 2.0*contour_scale_factor);
   CONTOUR (-1.0*contour_scale_factor, 1.0*contour_scale_factor);
   CONTOUR (-1.0*contour_scale_factor, 0.5*contour_scale_factor);
   CONTOUR (-1.0*contour_scale_factor, 0.0*contour_scale_factor);
   CONTOUR (-1.0*contour_scale_factor, -0.5*contour_scale_factor);
   CONTOUR (-1.0*contour_scale_factor, -1.0*contour_scale_factor);
   CONTOUR (-1.0*contour_scale_factor, -2.0*contour_scale_factor);
   CONTOUR (-1.0*contour_scale_factor, -2.9*contour_scale_factor);
   CONTOUR (-0.9*contour_scale_factor, -3.0*contour_scale_factor);
   CONTOUR (0.0*contour_scale_factor, -3.0*contour_scale_factor);
   CONTOUR (0.9*contour_scale_factor, -3.0*contour_scale_factor);
   CONTOUR (1.0*contour_scale_factor, -2.9*contour_scale_factor);
   CONTOUR (1.0*contour_scale_factor, -2.0*contour_scale_factor);
   CONTOUR (1.0*contour_scale_factor, -1.0*contour_scale_factor);
   CONTOUR (1.0*contour_scale_factor, -0.5*contour_scale_factor);

   CONTOUR (1.0*contour_scale_factor, 0.0*contour_scale_factor);   // repeat so that last normal is computed

#endif

	thread_color[0] = 240;
	thread_color[1] = 180;
	thread_color[2] = 10;
	stripe_color[0] = 2;
	stripe_color[1] = 37;
	stripe_color[2] = 165;
	truththread_color[0] = 240;
	truththread_color[1] = 10;
	truththread_color[2] = 10;


	for (int i=0; i < 3; i++)
	{
		thread_color_float[i] = ((float)thread_color[i])/255.0;
		stripe_color_float[i] = ((float)stripe_color[i])/255.0;
	}

	addTexture_stripe();
}


void GLThread::set_end_constraint(Vector3d end_pos, Matrix3d end_rot) {
  _thread->set_end_constraint(end_pos, end_rot);
}


void GLThread::printThreadData()
{
  updateThreadPoints();

  std::cout << "points:\n";
  for (int piece_ind=0; piece_ind < points.size(); piece_ind++)
  {
    std::cout << points[piece_ind].transpose() << "  ";
  }
  std::cout << "\nangles:\n";
  for (int piece_ind=0; piece_ind < twist_angles.size()-1; piece_ind++)
  {
    std::cout << twist_angles[piece_ind] << "  ";
  }
  std::cout << std::endl;

}

void GLThread::addTexture_stripe()
{

	const int TEXTURE_SIZE = 256;

	glMatrixMode (GL_TEXTURE); glLoadIdentity ();

   unsigned char * pixmap = (unsigned char *) malloc (TEXTURE_SIZE*TEXTURE_SIZE*3*sizeof (unsigned char));
   for (int i=0; i< TEXTURE_SIZE; i++) {
     for (int j=0; j< TEXTURE_SIZE; j++) {

       if (j < TEXTURE_SIZE/12.0 || j > TEXTURE_SIZE*11.0/12.0)
       {
         pixmap [3*TEXTURE_SIZE*i + 3*j] = stripe_color[0];
         pixmap [3*TEXTURE_SIZE*i + 3*j + 1] = stripe_color[1];
         pixmap [3*TEXTURE_SIZE*i + 3*j + 2] = stripe_color[2];
       } else {
         pixmap [3*TEXTURE_SIZE*i + 3*j] = thread_color[0];
         pixmap [3*TEXTURE_SIZE*i + 3*j + 1] = thread_color[1];
         pixmap [3*TEXTURE_SIZE*i + 3*j + 2] = thread_color[2];
       }

     }
   }

	gluBuild2DMipmaps (GL_TEXTURE_2D, 3, 
			TEXTURE_SIZE,
			TEXTURE_SIZE,
			GL_RGB, GL_UNSIGNED_BYTE, 
			(void *) (pixmap));

	delete pixmap;


   glMatrixMode (GL_TEXTURE);
   glLoadIdentity ();
   glScalef (1.0, 0.1, 1.0);
   glMatrixMode (GL_MODELVIEW);
	 gleTextureMode (GLE_TEXTURE_ENABLE | GLE_TEXTURE_VERTEX_MODEL_CYL);

   glEnable (GL_TEXTURE_2D);

}

void GLThread::removeTexture_stripe()
{
	const int TEXTURE_SIZE = 256;

	glMatrixMode (GL_TEXTURE); glLoadIdentity ();

	unsigned char * pixmap = (unsigned char *) malloc (TEXTURE_SIZE*TEXTURE_SIZE*3*sizeof (unsigned char));
	for (int i=0; i< TEXTURE_SIZE; i++) {
		for (int j=0; j< TEXTURE_SIZE; j++) {

			pixmap [3*TEXTURE_SIZE*i + 3*j] = thread_color[0];
			pixmap [3*TEXTURE_SIZE*i + 3*j + 1] = thread_color[1];
			pixmap [3*TEXTURE_SIZE*i + 3*j + 2] = thread_color[2];
		}
	}

	gluBuild2DMipmaps (GL_TEXTURE_2D, 3, 
			TEXTURE_SIZE,
			TEXTURE_SIZE,
			GL_RGB, GL_UNSIGNED_BYTE, 
			(void *) (pixmap));

	delete pixmap;


   glMatrixMode (GL_TEXTURE);
   glLoadIdentity ();
   glScalef (1.0, 0.1, 1.0);
   glMatrixMode (GL_MODELVIEW);
	 gleTextureMode (GLE_TEXTURE_ENABLE | GLE_TEXTURE_VERTEX_MODEL_CYL);

   glEnable (GL_TEXTURE_2D);
}

void GLThread::setTexture_truth()
{
	const int TEXTURE_SIZE = 256;

	glMatrixMode (GL_TEXTURE); glLoadIdentity ();

	unsigned char * pixmap = (unsigned char *) malloc (TEXTURE_SIZE*TEXTURE_SIZE*3*sizeof (unsigned char));
	for (int i=0; i< TEXTURE_SIZE; i++) {
		for (int j=0; j< TEXTURE_SIZE; j++) {

			pixmap [3*TEXTURE_SIZE*i + 3*j] = truththread_color[0];
			pixmap [3*TEXTURE_SIZE*i + 3*j + 1] = truththread_color[1];
			pixmap [3*TEXTURE_SIZE*i + 3*j + 2] = truththread_color[2];
		}
	}

	gluBuild2DMipmaps (GL_TEXTURE_2D, 3, 
			TEXTURE_SIZE,
			TEXTURE_SIZE,
			GL_RGB, GL_UNSIGNED_BYTE, 
			(void *) (pixmap));

	delete pixmap;


   glMatrixMode (GL_TEXTURE);
   glLoadIdentity ();
   glScalef (1.0, 0.1, 1.0);
   glMatrixMode (GL_MODELVIEW);
	 gleTextureMode (GLE_TEXTURE_ENABLE | GLE_TEXTURE_VERTEX_MODEL_CYL);

   glEnable (GL_TEXTURE_2D);
}
