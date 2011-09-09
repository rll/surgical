#ifndef _threadpiece_discrete_h
#define _threadpiece_discrete_h

#include <math.h>

#include "threadutils_discrete.h"

using namespace std;
USING_PART_OF_NAMESPACE_EIGEN

static double BEND_COEFF = 1;
static Matrix2d B = Matrix2d::Identity()*BEND_COEFF;
static double TWIST_COEFF = BEND_COEFF*3.00;
static double STRETCH_COEFF = 0.1 * BEND_COEFF;
static double GRAV_COEFF = BEND_COEFF*1e-4;
static double REPULSION_COEFF = 0.5 * BEND_COEFF;

static Matrix2d J = Matrix2d(Eigen::Rotation2Dd(M_PI/2.0));
static Matrix2d JB = J*B;

#include "Collisions/collisionUtils.h"

const double grad_eps = 1e-4;

static Vector3d grad_offsets[3];

class Thread;

class ThreadPiece
{
  public:
    ThreadPiece(const Vector3d& vertex, const double angle_twist, const double rest_length, Thread* my_thread);
    ThreadPiece(const Vector3d& vertex, const double angle_twist, const double rest_length, ThreadPiece* prev, ThreadPiece* next, Thread* my_thread);
    ThreadPiece(const ThreadPiece& rhs);
    ThreadPiece(const ThreadPiece& rhs, Thread* my_thread);

    ThreadPiece();
    virtual ~ThreadPiece();

    //Data Structures
    void set_vertex(const Vector3d& vertex);
    void offset_vertex(const Vector3d& offset_vertex){_vertex += offset_vertex;}
    void set_angle_twist(double angle_twist){ _angle_twist = angle_twist;}
    void offset_angle_twist(const double offset_angle_twist){_angle_twist += offset_angle_twist;}
    void set_rest_length(double rest_length){ _rest_length = rest_length;}
    void set_prev(ThreadPiece* prev);
    void set_next(ThreadPiece* next);
    void set_material_frame(const Matrix3d& material_frame){_material_frame = material_frame;}
    void set_bishop_frame(const Matrix3d& bishop_frame){_bishop_frame = bishop_frame;} //only makes sense for first piece
    const Matrix3d& material_frame(void) const {return _material_frame;}
    const Matrix3d& bishop_frame(void) const {return _bishop_frame;} //only makes sense for first piece
    const ThreadPiece* prev_piece(void) const {return _prev_piece;}
    const ThreadPiece* next_piece(void) const {return _next_piece;} //only makes sense for first piece
		const Vector3d& edge(void) const {return _edge;}
		const double edge_norm(void) const {return _edge_norm;}
		const Vector3d& curvature_binormal(void) { 
			if (_prev_piece == NULL) { _curvature_binormal = Vector3d::Zero(); return _curvature_binormal; }
			else if (_next_piece == NULL) { _curvature_binormal = Vector3d::Zero(); return _curvature_binormal; }
			else { calculateBinormal(); return _curvature_binormal; } //const {return _curvature_binormal;}
		}
		const double curvature_binormal_norm(void) { return (curvature_binormal()).norm(); } //const {return _curvature_binormal.norm();}
		const double angle() const { 
			if (_prev_piece == NULL) { return 0.0; }
			else { return angle_between(-_prev_piece->_edge, _edge); }
		}

    //Geometry
    void initializeFrames();
    bool is_material_frame_consistent();
    void set_total_length_and_first_last();

    void updateFrames();
		void updateFrames_all(); //update frames for many vertex position changes
    void updateFrames_twistOnly(); //simple update procedure if we only changed twist
    void updateFrames_firstpiece(); //update procedure if we changed the first piece
    void updateFrames_lastpiece(); //update procedure if we changed the second piece


    //Energy
    double energy();
    double energy_curvature();
    double energy_twist();
    double energy_stretch();
		double energy_grav();
    double energy_repulsion();

		//Energy Params
		void set_bend_coeff(double bend_coeff);
		void set_bend_matrix(const Matrix2d& bend_matrix);
		void set_twist_coeff(double twist_coeff);
		void set_grav_coeff(double grav_coeff);
		double get_bend_coeff(void);
		Matrix2d get_bend_matrix(void); //this is intentionally NOT pass by reference
		double get_twist_coeff(void);
		double get_grav_coeff(void);

    //Gradients
    void gradient_twist(double& grad);
    void gradient_vertex(Vector3d& grad);
    void gradient_vertex_numeric(Vector3d& grad);
    void calc_del_kb_k(Matrix3d& del_kb_k, const ThreadPiece* other_piece, Matrix3d& edge_skew_prev, Matrix3d& edge_skew_next);
    void add_sum_writhe(ThreadPiece* other_piece, Vector3d& curr_sum);

    void update_material_frame();
		//double calculate_holonomy();

    //variable-length thread_pieces
    void splitPiece(ThreadPiece* new_piece);
    void mergePiece();
    void fixPointersSplit(ThreadPiece* new_piece);
		void fixPointersMerge();

		void copyData(const ThreadPiece& rhs);
    void set_my_thread(Thread* my_thread){_my_thread = my_thread;};
    
    //overload operators
    ThreadPiece& operator=(const ThreadPiece& rhs);
    /*bool operator<(const ThreadPiece& rhs) const {
    	_rest_length < rhs._rest_length;
    }*/

    const Vector3d& vertex(void) const {return _vertex;}
    const double angle_twist(void) const {return _angle_twist;}
    const double rest_length(void) const {return _rest_length;}

 // protected:
    ThreadPiece* _prev_piece;
    ThreadPiece* _next_piece;

    Vector3d _vertex;
    double _angle_twist;
    double _rest_length;

    //calculated params (need to be updated)
    Vector3d _edge;
    double _edge_norm;
    Vector3d _curvature_binormal;
    Matrix3d _material_frame;
    Matrix3d _bishop_frame;

    Thread* _my_thread;


		Matrix3d skew_i;
		Matrix3d skew_i_im1;
		Matrix3d del_kb_i_im1;
		Matrix3d del_kb_i_ip1;
		Vector3d del_psi_i_im1;
		Vector3d del_psi_i_ip1;

    //void calculateBinormal(const double rest_length_prev, const Vector3d& edge_prev, 
		//											 const double rest_length_after, const Vector3d& edge_after, Vector3d& binormal);
		void calculateBinormal();
		//void calculateBinormal_withLength(const Vector3d& edge_prev, const Vector3d& edge_after, Vector3d& binormal);
		//void calculateBinormal_withLength();
    double twist_angle_error();
    void offset_and_update_locally(const Vector3d& offset);
    void offset_and_update(const Vector3d& offset);



    void update_edge();
    void update_bishop_frame();
		void update_bishop_frame_firstPiece();
		void update_bishop_frame_lastPiece();


    //faster calculation of angle axis rotations
    Matrix3d rot;
    void set_rotation(const double& angle, const Vector3d& axis);


};

#endif

