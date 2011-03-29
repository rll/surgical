
#ifndef _thread_discrete_h
#define _thread_discrete_h

#include "threadpiece_discrete.h"
#include <algorithm>
#include "omp.h"
#include "float.h"
#include <Eigen/Cholesky>
#include <Eigen/LU>


#ifdef ISOTROPIC 
    #define MAX_MOVEMENT_VERTICES 0.2
    #define MAX_ROTATION_TWIST (M_PI/30.0)
    #define MOMENTUM_CONSTANT 0.0 /*how much of the last gradient do we use*/

    #define MIN_MOVEMENT_VERTICES 1e-4
    #define MIN_ROTATION_TWIST (M_PI/1000.0)
#else

    #define MAX_MOVEMENT_VERTICES 0.2
    #define MAX_ROTATION_TWIST (M_PI/30.0)
    #define MOMENTUM_CONSTANT 0.0 /*how much of the last gradient do we use*/

    #define MIN_MOVEMENT_VERTICES 1e-6
    #define MIN_ROTATION_TWIST (M_PI/5000.0)

#endif



//#define NUM_THREADS_PARALLEL_FOR 2
#define num_iters_twist_est_max 0




using namespace std;
USING_PART_OF_NAMESPACE_EIGEN

class Thread
{
  public:
    Thread();
    Thread(const VectorXd& vertices, const VectorXd& twists, const Matrix3d& start_rot);
    Thread(vector<Vector3d>& vertices, vector<double>& twist_angles, Matrix3d& start_rot, Matrix3d& end_rot);
    Thread(vector<Vector3d>& vertices, vector<double>& twist_angles, Matrix3d& start_rot);
    Thread(const Thread& rhs);
    virtual ~Thread();

    //getting thread configuration
    void get_thread_data(vector<Vector3d>& points, vector<double>& twist_angles);
    void get_thread_data(vector<Vector3d>& points, vector<Matrix3d>& material_frames);
    void get_thread_data(vector<Vector3d>& points, vector<double>& twist_angles, vector<Matrix3d>& material_frames);

    //energy minimization
    //
#ifdef ISOTROPIC
    void minimize_energy(int num_opt_iters=6000, double min_move_vert=MIN_MOVEMENT_VERTICES, double max_move_vert=MAX_MOVEMENT_VERTICES, double energy_error_for_convergence=1e-5);
    void minimize_energy_hessian(int num_opt_iters=6000, double min_move_vert=MIN_MOVEMENT_VERTICES, double max_move_vert=MAX_MOVEMENT_VERTICES, double energy_error_for_convergence=1e-5);
#else
    void minimize_energy(int num_opt_iters=16000, double min_move_vert=MIN_MOVEMENT_VERTICES, double max_move_vert=MAX_MOVEMENT_VERTICES, double energy_error_for_convergence =1e-20);
    void minimize_energy_hessian(int num_opt_iters=16000, double min_move_vert=MIN_MOVEMENT_VERTICES, double max_move_vert=MAX_MOVEMENT_VERTICES, double energy_error_for_convergence =1e-20);
#endif
    void minimize_energy_twist_angles();

    void one_step_project(double step_size = 0.1, bool normalize_gradient = true);
    double one_step_grad_change(double step_size);
    void match_start_and_end_constraints(Thread& to_match, int num_steps, int num_steps_break = INT_MAX);


    //setting end constraints
    void set_constraints(const Vector3d& start_pos, const Matrix3d& start_rot, const Vector3d& end_pos, const Matrix3d& end_rot);
    void set_start_constraint(const Vector3d& start_pos, const Matrix3d& start_rot);
    void set_end_constraint(const Vector3d& end_pos, const Matrix3d& end_rot);
    void rotate_end_by(double degrees);
    void apply_motion(Frame_Motion& motion);
    
    void project_length_constraint_old();
    void project_length_constraint();
    void project_length_constraint_slow();

    const Matrix3d& start_rot(void) const {return _thread_pieces.front()->material_frame();}
    const Matrix3d& end_rot(void) const {return _thread_pieces[_thread_pieces.size()-2]->material_frame();}
    const Matrix3d& end_bishop(void) const {return _thread_pieces[_thread_pieces.size()-2]->bishop_frame();}
    const double end_angle(void) const {return _thread_pieces[_thread_pieces.size()-2]->angle_twist();}
    const double angle_at_ind(int i) const {return _thread_pieces[i]->angle_twist();}
    const Vector3d& vertex_at_ind(int i) const {return _thread_pieces[i]->vertex();}

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


    //debugging tools
    bool is_consistent();
    double calculate_holonomy();

    //overloaded operators
    Thread& operator=(const Thread& rhs);

    //energy and gradient functions
    double calculate_energy();
    void calculate_gradient(vector<Vector3d>& vertex_gradients, vector<double>& angle_twist_gradients);
    void calculate_gradient_vertices(vector<Vector3d>& vertex_gradients);
    void calculate_gradient_vertices_vectorized(VectorXd* vertex_gradients);
    void calculate_inv_hessian_vertices(MatrixXd& hessian);
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


    int num_pieces() const {return _thread_pieces.size();};

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


    vector<ThreadPiece*> _thread_pieces;
    vector<ThreadPiece*> _thread_pieces_backup;
    vector<double> _angle_twist_backup;
  protected:


    void add_momentum_to_gradient(vector<Vector3d>& vertex_gradients, vector<Vector3d>& new_gradients, double last_step_size);

    void apply_vertex_offsets(vector<Vector3d>& offsets, bool skip_edge_cases = false, double step_size = 1.0, bool update_frames=true);
    void apply_vertex_offsets_vectorized(const VectorXd& offsets, bool skip_edge_cases = false, double step_size = 1.0, bool update_frames=true);
    void apply_angle_twist_offsets(vector<double>& offsets, bool skip_edge_cases = false, double step_size = 1.0);

};

#endif

