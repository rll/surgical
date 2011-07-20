#include "EnvObjects.h"

/******************************************************* CYLINDER *******************************************************/
Cylinder::Cylinder(const Vector3d& pos, const Matrix3d& rot, double h, double r, float c0, float c1, float c2) {
	height = h;
	radius = r;
	setColor(c0, c1, c2);
	cyl_obj = new Intersection_Object();
	cyl_obj->_radius = r;
	setTransform(pos, rot);
  add_object_to_env(cyl_obj);
}

Cylinder::~Cylinder() {
	remove_object_from_env(cyl_obj);
	delete cyl_obj;
	cyl_obj = NULL;
}

void Cylinder::setTransform(const Vector3d& pos, const Matrix3d& rot) {
	cyl_obj->_start_pos = pos;
	cyl_obj->_end_pos 	= pos + rot*Vector3d(-height, 0.0, 0.0);
}

void Cylinder::setColor(float c0, float c1, float c2) {
	color0 = c0;
	color1 = c1;
	color2 = c2;
}

void Cylinder::draw() {
	glColor3f(color0, color1, color2);
  Vector3d vector_array[4];
  double point_array[4][3];

  vector_array[0] = cyl_obj->_start_pos - (cyl_obj->_end_pos - cyl_obj->_start_pos);
  vector_array[1] = cyl_obj->_start_pos;
  vector_array[2] = cyl_obj->_end_pos;
  vector_array[3] = cyl_obj->_end_pos + (cyl_obj->_end_pos - cyl_obj->_start_pos);

  for (int pt_ind = 0; pt_ind < 4; pt_ind++)
  {
    point_array[pt_ind][0] = vector_array[pt_ind](0);
    point_array[pt_ind][1] = vector_array[pt_ind](1);
    point_array[pt_ind][2] = vector_array[pt_ind](2);
  }

  glePolyCylinder(4, point_array, NULL, cyl_obj->_radius);
}

/***************************************************** END_EFFECTOR *****************************************************/
EndEffector::EndEffector(const Vector3d& pos, const Matrix3d& rot) {
	attachment = NULL;
	color0 = color1 = color2 = 0.7;
	degrees = 0.0;
	constraint = constraint_ind = -1;
	
	Intersection_Object* short_handle = new Intersection_Object();
	short_handle->_radius = 1.6;
  ee_objs.push_back(short_handle);
  add_object_to_env(short_handle);
  
	Intersection_Object* handle = new Intersection_Object();
	handle->_radius = handle_r;
  ee_objs.push_back(handle);
  add_object_to_env(handle);
  
  for (int piece=0; piece<pieces; piece++) {
 		Intersection_Object* tip_piece = new Intersection_Object();
 		tip_piece->_radius = 0.5+((double) piece)*((0.8*handle_r)-0.5)/((double) pieces-1);
  	ee_objs.push_back(tip_piece);
  	add_object_to_env(tip_piece);
	}
  
  for (int piece=0; piece<pieces; piece++) {
 		Intersection_Object* tip_piece = new Intersection_Object();
 		tip_piece->_radius = 0.5+((double) piece)*((0.8*handle_r)-0.5)/((double) pieces-1);
  	ee_objs.push_back(tip_piece);
  	add_object_to_env(tip_piece);
	}
	
	setTransform(pos, rot);
}

EndEffector::~EndEffector() {
	vector<Intersection_Object*>* objects;
  objects = get_objects_in_env();
	cout << "objects->size() before removing: " << objects->size() << endl;
	for (int i=0; i<ee_objs.size(); i++) {
		remove_object_from_env(ee_objs[i]);
		delete ee_objs[i];
		ee_objs[i] = NULL;
	}
	objects = get_objects_in_env();
	cout << "objects->size() after removing: " << objects->size() << endl;
	cout << "is ee_objs[0] NULL?: " << ((ee_objs[0]==NULL)?"YES":"NO") << endl;	
}

void EndEffector::setTransform(const Vector3d& pos, const Matrix3d& rot) {
	if (attachment != NULL) {
		degrees = attachment->open ? 15.0 : 0.0;
	} else
		degrees = 0.0;
	Vector3d start_pos;
	Vector3d end_pos;
	Vector3d new_pos;
	Matrix3d new_rot;
	Matrix3d open_rot = (Matrix3d) AngleAxisd(-degrees*M_PI/180, rot*Vector3d::UnitZ());
	
	start_pos = rot * Vector3d(grab_offset-3.0, 0.0, 0.0) + pos;
	end_pos = rot * Vector3d(grab_offset, 0.0, 0.0) + pos;
	ee_objs[0]->_start_pos = start_pos;
	ee_objs[0]->_end_pos 	= end_pos;
	
	start_pos = rot * Vector3d(end, 0.0, 0.0) + pos;
	end_pos = rot * Vector3d(end+30.0, 0.0, 0.0) + pos;
	ee_objs[1]->_start_pos = start_pos;
	ee_objs[1]->_end_pos 	= end_pos;
	
	for (int piece=0; piece<pieces; piece++) {
		double r = ee_objs[piece+pieces+2]->_radius;
		new_rot = open_rot * rot;
		new_pos = open_rot * rot * Vector3d(-end, 0.0, 0.0) + rot * Vector3d(end, 0.0, 0.0) + pos;
		start_pos = new_rot * Vector3d(start+((double) piece)*h, r, 0.0) + new_pos;
		end_pos = new_rot * Vector3d(start+((double) piece+1)*h, r, 0.0) + new_pos;
 		ee_objs[piece+2]->_start_pos = start_pos;
		ee_objs[piece+2]->_end_pos 	= end_pos;
	}
	
	for (int piece=0; piece<pieces; piece++) {
		double r = ee_objs[piece+pieces+2]->_radius;
		new_rot = open_rot.transpose() * rot;
		new_pos = open_rot.transpose() * rot * Vector3d(-end, 0.0, 0.0) + rot * Vector3d(end, 0.0, 0.0) + pos;
		start_pos = new_rot * Vector3d(start+((double) piece)*h, -r, 0.0) + new_pos;
		end_pos = new_rot * Vector3d(start+((double) piece+1)*h, -r, 0.0) + new_pos;
  	ee_objs[piece+pieces+2]->_start_pos = start_pos;
		ee_objs[piece+pieces+2]->_end_pos 	= end_pos;;
	}  
}

void EndEffector::attach(Cursor* cursor) {
	degrees = cursor->open ? 15.0 : 0.0;
	attachment = cursor;
}

void EndEffector::dettach() {
	if (attachment == NULL)
		cout << "Internal errror: EndEffector::dettach(): end effector cannot dettach since it does't have a cursor attached" << endl;
	attachment = NULL;
}

void EndEffector::draw() {
	glColor3f(color0, color1, color2);
  Vector3d vector_array[4];
  double point_array[4][3];

	for (int obj_ind=ee_objs.size()-1; obj_ind >= 0; obj_ind--) {
	  vector_array[0] = ee_objs[obj_ind]->_start_pos - (ee_objs[obj_ind]->_end_pos - ee_objs[obj_ind]->_start_pos);
	  vector_array[1] = ee_objs[obj_ind]->_start_pos;
	  vector_array[2] = ee_objs[obj_ind]->_end_pos;
	  vector_array[3] = ee_objs[obj_ind]->_end_pos + (ee_objs[obj_ind]->_end_pos - ee_objs[obj_ind]->_start_pos);

	  for (int pt_ind = 0; pt_ind < 4; pt_ind++)
	  {
	    point_array[pt_ind][0] = vector_array[pt_ind](0);
	    point_array[pt_ind][1] = vector_array[pt_ind](1);
	    point_array[pt_ind][2] = vector_array[pt_ind](2);
	  }
		
		if (obj_ind==0) { glColor3f(0.3, 0.3, 0.0); }
	  glePolyCylinder(4, point_array, NULL, ee_objs[obj_ind]->_radius);
  }
  unhighlight();
}

/******************************************************** CURSOR ********************************************************/
Cursor::Cursor(const Vector3d& pos, const Matrix3d& rot) {
	end_eff = NULL;
	open = false;
	last_open  = false;
	attach_dettach_attempt = false;
	cursor_obj = new Intersection_Object();
	cursor_obj->_radius = radius;
	setTransform(pos, rot);
  add_object_to_env(cursor_obj);
}

Cursor::~Cursor() {
	vector<Intersection_Object*>* objects;
  objects = get_objects_in_env();
	cout << "objects->size() before removing: " << objects->size() << endl;
	remove_object_from_env(cursor_obj);
	cout << "objects->size() after removing: " << objects->size() << endl;
	delete cursor_obj;
	cursor_obj = NULL;
	cout << "is cursor_obj NULL?: " << ((cursor_obj==NULL)?"YES":"NO") << endl;
}

void Cursor::setTransform(const Vector3d& pos, const Matrix3d& rot) {
	cursor_obj->_start_pos = pos;
	cursor_obj->_end_pos 	= pos - height * rot.col(0);
}

void Cursor::attach(EndEffector* ee) {
	end_eff = ee;
	end_eff->attach(this);
}

// Dettaches the cursor from the end effector it is holding. It has to be holding an end effector. If the end effector isn't holding the thread, it is removed from the environment.
void Cursor::dettach() {
	if (end_eff == NULL)
		cout << "Internal errror: Cursor::dettach(): cursor cannot dettach since it does't have an end effector attached" << endl;
	end_eff->dettach();
	if (end_eff->constraint<0)
		delete end_eff;
	end_eff = NULL;
}

void Cursor::draw() {
	glColor3f(open?0.0:0.5, open?0.5:0.0, 0.0);
  Vector3d vector_array[4];
  double point_array[4][3];
	
  vector_array[0] = cursor_obj->_start_pos - (cursor_obj->_end_pos - cursor_obj->_start_pos);
  vector_array[1] = cursor_obj->_start_pos;
  vector_array[2] = cursor_obj->_end_pos;
  vector_array[3] = cursor_obj->_end_pos + (cursor_obj->_end_pos - cursor_obj->_start_pos);
	
  for (int pt_ind = 0; pt_ind < 4; pt_ind++)
  {
    point_array[pt_ind][0] = vector_array[pt_ind](0);
    point_array[pt_ind][1] = vector_array[pt_ind](1);
    point_array[pt_ind][2] = vector_array[pt_ind](2);
  }

  glePolyCylinder(4, point_array, NULL, cursor_obj->_radius);
}
