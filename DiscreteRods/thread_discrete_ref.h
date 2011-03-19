#ifndef _thread_discrete_h
#define _thread_discrete_h

#include "threadpiece_discrete.h"
#include <vector>
#include <algorithm>
#include "omp.h"
#include "float.h"
#include <Eigen/Cholesky>
#include <Eigen/LU>
#define MAX_MOVEMENT_VERTICES 0.2
#define MAX_ROTATION_TWIST (M_PI/30.0)
#define MOMENTUM_CONSTANT 0.0 /*how much of the last gradient do we use*/

#define MIN_MOVEMENT_VERTICES 1e-3
#define MIN_ROTATION_TWIST (M_PI/1000.0)

//#define NUM_THREADS_PARALLEL_FOR 2
#define num_iters_twist_est_max 0




using namespace std;
USING_PART_OF_NAMESPACE_EIGEN

class Thread
{
  public:
    Thread();
    Thread(vector<Vector3d>& vertices, vector<double>& twist_angles, Matrix3d& start_rot, Matrix3d& end_rot);
    Thread(vector<Vector3d>& vertices, vector<double>& twist_angles, Matrix3d& start_rot);
    Thread(const Thread& rhs);
    ~Thread();

    //getting thread configuration
    void get_thread_data(vector<Vector3d>& points, vector<double>& twist_angles);
    void get_thread_data(vector<Vector3d>& points, vector<Matrix3d>& material_frames);
    void get_thread_data(vector<Vector3d>& points, vector<double>& twist_angles, vector<Matrix3d>& material_frames);

    //energy minimization
    void minimize_energy_bfgs();
    void minimize_energy_noiseStart();
    void minimize_energy();
    void one_step_project();
    //these energy minimizers minimize the total energy (from curvature AND twist) by only altering
    //either the twist angles, or the vertex positions
    void minimize_energy_twist_angles(bool force_optimization = false);
    void minimize_energy_vertices();



    //setting end constraints
    void set_constraints(const Vector3d& start_pos, const Matrix3d& start_rot, const Vector3d& end_pos, const Matrix3d& end_rot);
    void set_start_constraint(const Vector3d& start_pos, const Matrix3d& start_rot);
    void set_end_constraint(const Vector3d& end_pos, const Matrix3d& end_rot);
    void project_length_constraint_old();
    void project_length_constraint();
    void project_length_constraint_slow();

    const Matrix3d& start_rot(void) const {return _thread_pieces.front().material_frame();}
    const Matrix3d& end_rot(void) const {return _thread_pieces[_thread_pieces.size()-2].material_frame();}
    const Matrix3d& end_bishop(void) const {return _thread_pieces[_thread_pieces.size()-2].bishop_frame();}
    const double end_angle(void) const {return _thread_pieces[_thread_pieces.size()-2].angle_twist();}
    const double angle_at_ind(int i) const {return _thread_pieces[i].angle_twist();}

    const Vector3d& start_pos(void) const {return _thread_pieces.front().vertex();}
    const Vector3d& end_pos(void) const {return _thread_pieces.back().vertex();}


    void toVector(VectorXd* vec) {
      vec->resize(3*num_pieces());
      for(int i = 0; i < num_pieces(); i++) {
        vec->segment<3>(3*i) = _thread_pieces[i].vertex();
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
    void calculate_gradient_twist(vector<double>& angle_twist_gradients);
    void make_max_norm_one(vector<Vector3d>& to_normalize);

    //energy coefficients
		void set_bend_coeff(double bend_coeff){_thread_pieces.front().set_bend_coeff(bend_coeff);}
		void set_bend_matrix(const Matrix2d& bend_matrix){_thread_pieces.front().set_bend_matrix(bend_matrix);}
		void set_twist_coeff(double twist_coeff){_thread_pieces.front().set_twist_coeff(twist_coeff);}
		void set_grav_coeff(double grav_coeff){_thread_pieces.front().set_grav_coeff(grav_coeff);}
    void set_coeffs_normalized(double bend_coeff, double twist_coeff, double grav_coeff);
    void set_coeffs_normalized(const Matrix2d& bend_matrix, double twist_coeff, double grav_coeff);


    int num_pieces(){return _thread_pieces.size();};

    void save_angle_twists();
    void restore_angle_twists();
    //void save_angle_twist_changes(vector<double>& start_angles, vector<double>& end_angles);
    void save_thread_pieces();  //these DO NOT copy the prev and next pointers...just copy frames
    void restore_thread_pieces();

  protected:
    vector<ThreadPiece> _thread_pieces;
    vector<ThreadPiece> _thread_pieces_backup;
    vector<double> _angle_twist_backup;


    void add_momentum_to_gradient(vector<Vector3d>& vertex_gradients, vector<Vector3d>& new_gradients, double last_step_size);

    void apply_vertex_offsets(vector<Vector3d>& offsets, bool skip_edge_cases = false, double step_size = 1.0, bool update_frames=true);
    void apply_vertex_offsets_vectorized(const VectorXd& offsets, bool skip_edge_cases = false, double step_size = 1.0, bool update_frames=true);
    void apply_angle_twist_offsets(vector<double>& offsets, bool skip_edge_cases = false, double step_size = 1.0);






    int num_iters_twist_est;
    //double _saved_last_theta;
    //double _saved_last_theta_change;
    //vector<double> _saved_last_theta_changes;

};

#endif

