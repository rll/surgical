
#ifndef _thread_discrete_h
#define _thread_discrete_h

#include <algorithm>
#include "omp.h"
#include "float.h"
#include "threadutils_discrete.h"
#include "threadpiece_discrete.h"
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <Eigen/Geometry>

#ifdef ISOTROPIC 
    #define MAX_MOVEMENT_VERTICES 0.2
    #define MAX_ROTATION_TWIST (M_PI/30.0)
    #define MOMENTUM_CONSTANT 0.0 /*how much of the last gradient do we use*/

    #define MIN_MOVEMENT_VERTICES 1e-6 //speedy at 1e-4
    #define MIN_ROTATION_TWIST (M_PI/1000.0)
    
    #define ENERGY_FOR_CONVERGENCE 5e-7 //speedy at 1e-5
    #define NUM_MAX_ITERS 6000 //speedy at 6000
#else

    #define MAX_MOVEMENT_VERTICES 0.2
    #define MAX_ROTATION_TWIST (M_PI/30.0)
    #define MOMENTUM_CONSTANT 0.0 /*how much of the last gradient do we use*/

    #define MIN_MOVEMENT_VERTICES 1e-6
    #define MIN_ROTATION_TWIST (M_PI/5000.0)

    #define ENERGY_FOR_CONVERGENCE 1e-20
    #define NUM_MAX_ITERS 16000

#endif

#define DEFAULT_REST_LENGTH 3 /*default rest length for each threadpiece*/
#define LENGTH_THRESHHOLD 0.5 /*we must be this much shorter than the total length */

#define REFINE_THRESHHOLD 0.5
#define REFINE_SINGLE_THRESHHOLD 0.6
#define UNREFINE_THRESHHOLD 0.2 /*must be smaller than REFINE_THRESHHOLD otherwise thread will be unstable */
#define NATURAL_REST_LENGTH 3.0
#define REFINEMENT_DEPTH 4
#define MIN_REST_LENGTH 0.1

#define INTERSECTION_PUSHBACK_EPS 0.03 
//#define NUM_THREADS_PARALLEL_FOR 2
#define num_iters_twist_est_max 0

#define THREAD_RADIUS 0.2     /* MUST BE ATLEAST MAX_MOVEMENT_VERTICES */
#define COLLISION_CHECKING true


using namespace std;
USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

struct Self_Intersection
{
  int _piece_ind_a;
  int _piece_ind_b;
  double _dist;

  Self_Intersection() {}
  Self_Intersection(int piece_ind_a, int piece_ind_b, double dist)
    : _piece_ind_a(piece_ind_a), _piece_ind_b(piece_ind_b), _dist(dist) {}
  Self_Intersection(const Self_Intersection& cpy)
    : _piece_ind_a(cpy._piece_ind_a), _piece_ind_b(cpy._piece_ind_b), _dist(cpy._dist) {}
  
};

struct Intersection
{
  int _piece_ind;
  int _object_ind;
  double _dist;

  Intersection() {}
  Intersection(int piece_ind, int object_ind, double dist)
    : _piece_ind(piece_ind), _object_ind(object_ind), _dist(dist) {}
  Intersection(const Intersection& cpy)
    : _piece_ind(cpy._piece_ind), _object_ind(cpy._object_ind), _dist(cpy._dist) {}
};

struct Intersection_Object
{
  double _radius;
  Vector3d _start_pos;
  Vector3d _end_pos;

  Intersection_Object() {}
  Intersection_Object(double radius, Vector3d& start_pos, Vector3d& end_pos)
    : _radius(radius), _start_pos(start_pos), _end_pos(end_pos) {}
  Intersection_Object(const Intersection_Object& cpy)
    : _radius(cpy._radius), _start_pos(cpy._start_pos), _end_pos(cpy._end_pos) {}

};

static vector<Intersection_Object> objects_in_env;
void add_object_to_env(Intersection_Object& obj);
vector<Intersection_Object>* get_objects_in_env();



class Thread
{
  public:
    Thread();
    Thread(const VectorXd& vertices, const VectorXd& twists, const Matrix3d& start_rot);
    Thread(vector<Vector3d>& vertices, vector<double>& twist_angles, Matrix3d& start_rot, Matrix3d& end_rot);
    Thread(vector<Vector3d>& vertices, vector<double>& twist_angles, vector<double>& rest_lengths, Matrix3d& start_rot, Matrix3d& end_rot);
    Thread(vector<Vector3d>& vertices, vector<double>& twist_angles, Matrix3d& start_rot);
    Thread(vector<Vector3d>& vertices, vector<double>& twist_angles, Matrix3d& start_rot, const double rest_length);
    Thread(vector<Vector3d>& vertices, vector<double>& twist_angles, vector<double>& rest_lengths, Matrix3d& start_rot);
    Thread(const Thread& rhs);
    virtual ~Thread();

    //getting thread configuration
    void get_thread_data(vector<Vector3d>& points);
    void get_thread_data(vector<Vector3d>& points, vector<double>& twist_angles);
    void get_thread_data(vector<Vector3d>& points, vector<double>& twist_angles, vector<double>& rest_lengths);
    void get_thread_data(vector<Vector3d>& points, vector<Matrix3d>& material_frames);
    void get_thread_data(vector<Vector3d>& points, vector<double>& twist_angles, vector<Matrix3d>& material_frames);
    void get_thread_data(vector<double>& lengths, vector<double>& edge_norms);
    void set_all_angles_zero();
    void set_all_pieces_mythread();

    //energy minimization
    //
#ifdef ISOTROPIC
    bool minimize_energy(int num_opt_iters=NUM_MAX_ITERS, double min_move_vert=MIN_MOVEMENT_VERTICES, double max_move_vert=MAX_MOVEMENT_VERTICES, double energy_error_for_convergence=ENERGY_FOR_CONVERGENCE);
    void minimize_energy_hessian(int num_opt_iters=NUM_MAX_ITERS, double min_move_vert=MIN_MOVEMENT_VERTICES, double max_move_vert=MAX_MOVEMENT_VERTICES, double energy_error_for_convergence=ENERGY_FOR_CONVERGENCE);
#else
    void minimize_energy(int num_opt_iters=NUM_MAX_ITERS, double min_move_vert=MIN_MOVEMENT_VERTICES, double max_move_vert=MAX_MOVEMENT_VERTICES, double energy_error_for_convergence=ENERGY_FOR_CONVERGENCE);
    void minimize_energy_hessian(int num_opt_iters=NUM_MAX_ITERS, double min_move_vert=MIN_MOVEMENT_VERTICES, double max_move_vert=MAX_MOVEMENT_VERTICES, double energy_error_for_convergence=ENERGY_FOR_CONVERGENCE);
#endif
    void minimize_energy_twist_angles();

    void one_step_project(double step_size = 0.1, bool normalize_gradient = true);
    double one_step_grad_change(double step_size);
    void match_start_and_end_constraints(Thread& to_match, int num_steps, int num_steps_break = INT_MAX);
    void match_start_and_end_constraints(Thread& to_match, int num_steps, int num_steps_break, vector<Thread*>& intermediates);


    //setting end constraints
    void set_constraints(const Vector3d& start_pos, const Matrix3d& start_rot, const Vector3d& end_pos, const Matrix3d& end_rot);
    void set_start_constraint_nearEnd(Vector3d& start_pos, Matrix3d& start_rot);
    void set_end_constraint_nearEnd(Vector3d& end_pos, Matrix3d& end_rot);
    void set_constraints_nearEnds(Vector3d& start_pos, Matrix3d& start_rot, Vector3d& end_pos, Matrix3d& end_rot);
    void set_start_constraint(const Vector3d& start_pos, const Matrix3d& start_rot, bool backup=true);
    void set_end_constraint(const Vector3d& end_pos, const Matrix3d& end_rot, bool backup=true);
    void restore_constraints(const Vector3d& start_pos, const Matrix3d& start_rot, const Vector3d& end_pos, const Matrix3d& end_rot);
    void rotate_end_by(double degrees);
    void apply_motion(Frame_Motion& motion); //applies motion to end points/rotations
    void apply_motion(Two_Motions& motion);
    void apply_motion_nearEnds(Frame_Motion& motion, bool mini_energy = true); //applies motion to 2nd and 2nd to last points/rotations, and clamps to ensure constraints are not violated
    void apply_motion_nearEnds(Two_Motions& motion, bool mini_energy = true);
    void unviolate_total_length_constraint();
    void copy_data_from_vector(VectorXd& toCopy);
    
    //void project_length_constraint_old();
    bool project_length_constraint(int recursive_depth=250);
    //void project_length_constraint_slow();

    const Matrix3d& start_rot(void) const {return _thread_pieces.front()->material_frame();}
    const Matrix3d& end_rot(void) const {return _thread_pieces[_thread_pieces.size()-2]->material_frame();}
    const Matrix3d& end_bishop(void) const {return _thread_pieces[_thread_pieces.size()-2]->bishop_frame();}
    const double start_angle(void) const {return _thread_pieces.front()->angle_twist();}
    const double end_angle(void) const {return _thread_pieces[_thread_pieces.size()-2]->angle_twist();}
    const double angle_at_ind(int i) const {return _thread_pieces[i]->angle_twist();}
    const double total_length(void) const {return _total_length;}
    const double start_rest_length(void) const {return _thread_pieces.front()->rest_length();}
    const double end_rest_length(void) const {return _thread_pieces[_thread_pieces.size()-2]->rest_length();}
    const double rest_length_at_ind(int i) const {return _thread_pieces[i]->rest_length();}
    const Vector3d& vertex_at_ind(int i) const {return _thread_pieces[i]->vertex();}
    const Vector3d& edge_at_ind(int i) const {return _thread_pieces[i]->edge();}
    const Matrix3d& bishop_at_ind(int i) const {return _thread_pieces[i]->bishop_frame();}
    const Matrix3d& material_at_ind(int i) const {return _thread_pieces[i]->material_frame();}

    const Vector3d& start_pos(void) const {return _thread_pieces.front()->vertex();}
    const Vector3d& end_pos(void) const {return _thread_pieces.back()->vertex();}

    const Vector3d& start_edge(void) const {return _thread_pieces.front()->edge();}
    const Vector3d& end_edge(void) const {return _thread_pieces[_thread_pieces.size()-2]->edge();}


    void toVector(VectorXd* vec) const {
      vec->resize(3*num_pieces());
      for(int i = 0; i < num_pieces(); i++) {
        vec->segment<3>(3*i) = _thread_pieces[i]->vertex();
      }
    }
    void getTwists(VectorXd* vec) const {
      vec->resize(num_pieces());
      for(int i = 0; i < num_pieces(); i++) {
        (*vec)(i) = _thread_pieces[i]->angle_twist();
      }
    }

    void getEdges(VectorXd* vec) const {
      vec->resize(3*(num_pieces()-1));
      for (int i = 0; i < num_pieces() - 1; i++) {
        vec->segment<3>(3*i) = _thread_pieces[i]->edge(); 
      }
    }

    void getCurvatureBinormal(VectorXd* vec) const {
      vec->resize(3*num_pieces());
      for (int i = 0; i < num_pieces(); i++) { 
        vec->segment<3>(3*i) = _thread_pieces[i]->curvature_binormal();
      }
    }
    
    void getCurvatureBinormalNorm(vector<double>& vec) const {
      vec.resize(num_pieces());
      for (int i = 0; i < num_pieces(); i++) { 
        vec[i] = (_thread_pieces[i]->curvature_binormal()).norm();
      }
    }
    //debugging tools
    bool is_consistent();
    double calculate_holonomy();
    void print_vertices();

    //overloaded operators
    Thread& operator=(const Thread& rhs);

    //energy and gradient functions
    double calculate_energy();
    double calculate_energy_inefficient();
    void calculate_gradient(vector<Vector3d>& vertex_gradients, vector<double>& angle_twist_gradients);
    void calculate_gradient_vertices(vector<Vector3d>& vertex_gradients);
    void calculate_gradient_vertices_vectorized(VectorXd* vertex_gradients);
    void calculate_hessian_vertices(MatrixXd& hessian);
    void calculate_gradient_twist(vector<double>& angle_twist_gradients);
    void make_max_norm_one(vector<Vector3d>& to_normalize);
    void make_max_norm_one_allPieces(vector<Vector3d>& to_normalize);

    //energy coefficients
	void set_bend_coeff(double bend_coeff){_thread_pieces.front()->set_bend_coeff(bend_coeff);}
	void set_bend_matrix(const Matrix2d& bend_matrix){_thread_pieces.front()->set_bend_matrix(bend_matrix);}
	void set_twist_coeff(double twist_coeff){_thread_pieces.front()->set_twist_coeff(twist_coeff);}
	void set_grav_coeff(double grav_coeff){_thread_pieces.front()->set_grav_coeff(grav_coeff);}
	double get_bend_coeff(void){return _thread_pieces[2]->get_bend_coeff();}
	Matrix2d get_bend_matrix(void){return _thread_pieces[2]->get_bend_matrix();}
	double get_twist_coeff(void){return _thread_pieces[2]->get_twist_coeff();}
	double get_grav_coeff(void){return _thread_pieces[2]->get_grav_coeff();}
    void set_coeffs_normalized(double bend_coeff, double twist_coeff, double grav_coeff);
    void set_coeffs_normalized(const Matrix2d& bend_matrix, double twist_coeff, double grav_coeff);

    void set_end_twist(double twist) { 
      _thread_pieces[_thread_pieces.size()-2]->set_angle_twist(twist);
      _thread_pieces[_thread_pieces.size()-2]->update_material_frame();

    }

    const int num_pieces() const {return _thread_pieces.size();};
    const int num_edges() const {return _thread_pieces.size()-1;};

    void save_angle_twists();
    void restore_angle_twists();
    //void save_angle_twist_changes(vector<double>& start_angles, vector<double>& end_angles);
    void save_thread_pieces();  //these DO NOT copy the prev and next pointers...just copy frames
    void restore_thread_pieces();
    void restore_thread_pieces(vector<ThreadPiece*>& to_restore);
    void save_thread_pieces(vector<ThreadPiece*>& to_save);
    void save_thread_pieces_and_resize(vector<ThreadPiece*>& to_save);
    void restore_thread_pieces_and_resize(vector<ThreadPiece*>& to_restore);
    void set_prev_next_pointers(vector<ThreadPiece*> pieces);
    void delete_current_threadpieces();
    void delete_threadpieces(vector<ThreadPiece*> thread_pieces);


    void evaluate_twist_scores(vector<double>& twist_to_try, vector<double>& twist_scores);
    void evaluate_twist_scores(vector<double>& twist_to_try, vector<double>& twist_scores, vector<double>& angle_start, vector<double>& angle_end);
    //void set_twist_and_minimize(double twist);
    void set_twist_and_minimize(double twist, vector<Vector3d>& orig_pts);

    double _total_length;

    //intersection
    double self_intersection(int i, int j, double radius); //do these two pieces intersect?
    double obj_intersection(int piece_ind, double piece_radius, int obj_ind, double obj_radius);
    double intersection(const Vector3d& a_start_in, const Vector3d& a_end_in, const double a_radius, const Vector3d& b_start_in, const Vector3d& b_end_in, const double b_radius);

    bool check_for_intersection(vector<Self_Intersection>& self_intersections, vector<Intersection>& intersections);
    void fix_intersections();
    
    //variable-length thread_pieces
    //void split_thread_piece(int thread_piece_ind);
    //void merge_thread_piece(int thread_piece_ind);
    
    void split_thread_piece(ThreadPiece* this_piece, ThreadPiece* this_piece_backup);
    void merge_thread_piece(ThreadPiece* this_piece, ThreadPiece* this_piece_backup);
    void adapt_links();
    void refine_links();
    void unrefine_links();

  //protected:
    vector<ThreadPiece*> _thread_pieces;
    vector<ThreadPiece*> _thread_pieces_backup;
    vector<double> _angle_twist_backup;

    Vector3d _start_pos_backup;
    Matrix3d _start_rot_backup;
    Vector3d _end_pos_backup;
    Matrix3d _end_rot_backup; 
    vector<ThreadPiece*> _thread_pieces_collision_backup;

    void add_momentum_to_gradient(vector<Vector3d>& vertex_gradients, vector<Vector3d>& new_gradients, double last_step_size);

    void apply_vertex_offsets(vector<Vector3d>& offsets, bool skip_edge_cases = false, double step_size = 1.0, bool update_frames=true);
    void apply_vertex_offsets_vectorized(const VectorXd& offsets, bool skip_edge_cases = false, double step_size = 1.0, bool update_frames=true);
    void apply_angle_twist_offsets(vector<double>& offsets, bool skip_edge_cases = false, double step_size = 1.0);

};

#endif

