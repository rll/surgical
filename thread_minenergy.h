#ifndef _thread_minenergy_h
#define _thread_minenergy_h

#include "threadpiece_minenergy.h"
#include "thread_utils.h"
#include <Eigen/Geometry> 

      
#define CONSTANT_ROTATION_ERROR_PENALTY 1.0
#define CONSTANT_POSITION_ERROR_PENALTY 3.0
#define CONSTANT_THREAD_CURVATURE_ERROR_PENALTY 1.0
#define CONSTANT_THREAD_TORSION_ERROR_PENALTY 1.0
#define CONSTANT_THREAD_DIFF_CURVATURE_ERROR_PENALTY 1.0
#define CONSTANT_THREAD_DIFF_TORSION_ERROR_PENALTY 1.0
#define CONSTANT_TOTAL_ERROR_PENALTY 500.0
#define GRAVITY_CONSTANT 0.000004

#define MANYPOINTS_COEFF_ENERGY 5.0
#define MANYPOINTS_COEFF_DIFF_PARAMS 5.0
#define MANYPOINTS_COEFF_POINT 10000 //CONSTANT_TOTAL_ERROR_PENALTY
#define MANYPOINTS_COEFF_POINT_EXPONENTIAL 100.0 //CONSTANT_POSITION_ERROR_PENALTY

#define DISTANCE_THRESH 0.01
#define MIN_SEG_LENGTH 0.02 			//not quite...just wont try to split it if we end up below length

#define RANDOM_THREAD_MAX_CURVATURE_INIT 2.0
#define RANDOM_THREAD_MAX_TORSION_INIT 1.0




#define NOISE_EACH_ITER_POS 1.0
#define NOISE_EACH_ITER_TAN 0.08
#define NOISE_EACH_ITER_CURVATURE 0.1
#define NOISE_EACH_ITER_TORSION 0.1
#define NOISE_EACH_ITER_DIFF_CURVATURE 0.1
#define NOISE_EACH_ITER_DIFF_TORSION 0.1
#define NOISE_EACH_ITER_GRAV 0.05


struct optimization_info
{
	Vector3d position_end;
	Vector3d tangent_start;
	Vector3d tangent_end;

	Matrix4d transform_back;
  double length_back;
  double curvature_back, torsion_back;
	Matrix4d transform_front;
  double length_front;
  double curvature_front, torsion_front;

	double length_per_segment;
	double num_segments;
	NEWMAT::ColumnVector* orig_params_each_piece; //each curvature and torsion, and lastly the angle to rotate around the x axis to start

  double gravity_multipler;
  double total_length;

  double thread_curvature_error_penalty;
  double thread_torsion_error_penalty;
  double thread_diff_curvature_error_penalty;
  double thread_diff_torsion_error_penalty;
  double thread_position_error_penalty;
  double thread_rotation_error_penalty;
  double thread_total_error_penalty;
};

static optimization_info opt_params; //will have to change this if we have multiple threads?

void energyEvalFunction_init(int ndim, NEWMAT::ColumnVector& x);
void energyEvalFunction(int ndim, const NEWMAT::ColumnVector& x, double& fx, int& result);


struct optimization_info_many_points
{
	vector<Vector3d> points;

	double length_per_segment;
	int num_segments;
  int num_pts_between;
	NEWMAT::ColumnVector* orig_params_each_piece; 

  double gravity_multipler;
  double total_length;
};

static optimization_info_many_points opt_params_many_points; //will have to change this if we have multiple threads?

void energyEvalFunctionManyPoints_init(int ndim, NEWMAT::ColumnVector& x);
void energyEvalFunctionManyPoints(int ndim, const NEWMAT::ColumnVector& x, double& fx, int& result);



class Thread
{
	public:
		Thread(double* curvature, double* torsion, double* length, int numPieces, Vector3d* positions, Vector3d* tangents);
    Thread(double length, Vector3d& startPos, Vector3d& startTan);

    Thread(const Thread* toCopy);
    Thread(Thread* toGetPoints, int num_constraints, int num_pieces_between);
    Thread(MatrixXd& constraints, int num_pieces_between, double length_thread);
    Thread(MatrixXd& constraints, int num_pieces_between, double length_thread, Matrix3d& init_rot);


		//Thread();
		~Thread();

    void setConstraints(Vector3d* positions, Vector3d* tangents);
    void setEndConstraint(Vector3d& end_position, Vector3d& end_tangent);
    void moveEndConstraint(Vector3d& move_end_pos, Matrix3d& rotate_end_tangent);
		void getPoints(MatrixXd& points);
    void getStartTransform(Matrix4d& toStart);
    void getWantedEndPosition(Vector3d& endPos);
    void getWantedEndTangent(Vector3d& endTan);
    void getActualEndPosition(Vector3d& endPos);
    void getActualEndTangent(Vector3d& endTan);
		void minimize_energy();
		double findMaxDistance(ThreadPiece** maxDistPiece);
    void upsample_minLength(double minLength);
    void upsampleAndOptimize_minLength(double minLength);
    ThreadPiece* firstPiece(){return threadList;}
    
    void turnOnNoise(double noiseVal);  //noiseVal should be between 0 and 1
    void turnOffNoise();
    
    double length(){return _length;}
  

    void printThreadInfo();

    double noiseLevel;

    bool optimizeManyPoints(Thread* origp_thread, int num_constraints, int num_pts_between);
    void optimizeManyPoints_MyParams(MatrixXd& constraints, int num_pts_between);
    void optimizeManyPoints(MatrixXd& constraints, int num_pts_between, vector<double>&curvatures_resampled, vector<double>& torsions_resampled, Matrix4d& start_transform);//assumes it should skip the first point

    bool resamplePointsAndParams(MatrixXd& resampled_points, vector<double>& curvatures, vector<double>& torsions);
    void optimizeManyPointsAndParams(MatrixXd& constraints, int num_pieces_between, double length_thread, Matrix3d& init_rot);

    void setStationaryOptParams(bool addNoise = true);

    //params for energy minimization
    void initEnergyParams();
    void copyEnergyParams(const Thread* toCopy);
      

    double thread_curvature_error_penalty;
    double thread_torsion_error_penalty;
    double thread_diff_curvature_error_penalty;
    double thread_diff_torsion_error_penalty;
    double position_error_penalty;
    double rotation_error_penalty;
    double total_error_penalty;
    double gravity_penalty;




  protected:
		ThreadPiece* threadList;
		double _angle_first_rot;
		double _length;
    Vector3d _positions[2];
    Vector3d _tangents[2];
		Matrix4d _transform_to_start; //rotation for transform from the algorithm (which starts at [0,0,0] with tangent [1,0,0] to our thread 
    Matrix4d _translate_to_start; //translation for transform from our algorithm to the thread
		Matrix4d _transform_to_unit;  //transform from our thread to the algorithm

    double _noiseAmount;
    Vector3d _currNoisePoints[2];
    Vector3d _currNoiseTans[2];
    double _currNoiseCurveCurvature;
    double _currNoiseCurveTorsion;
    double _currNoiseCurveDiffCurvature;
    double _currNoiseCurveDiffTorsion;
    double _currNoiseGrav;




/*    void optimize_GSS(int numParams, OPTPP::USERFCN0 eval, OPTPP::INITFCN init, NEWMAT::ColumnVector& solution, double fcnTol=1e-7);
    void optimize_FDNLF(int numParams, OPTPP::USERFCN0 eval, OPTPP::INITFCN init, NEWMAT::ColumnVector& solution, double fcnTol=1e-7);
*/

};

//describes the motion we want to apply to one side between nodes
struct Thread_Motion
{
	Vector3d pos_movement;
	Matrix3d tan_rotation;

	Thread_Motion(){};

  Thread_Motion(const Thread_Motion& toCopy) :
    pos_movement(toCopy.pos_movement), tan_rotation(toCopy.tan_rotation){};

	Thread* applyMotion(const Thread* start);
  void setRotationMatrixFromAngs(double ang1, double ang2);
	Thread_Motion& operator=(const Thread_Motion& rhs);

};




#endif
