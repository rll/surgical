#include "worldSQP.h"
#include <boost/progress.hpp>
#include <boost/thread.hpp>

#define MATLAB_INSTALL "matlab"
#define COLOCATION_METHOD true
#define TIMER_ENABLED true 
#define LAMBDA_U 0.0000001
#define LAMBDA_U_DOT 0.0001
#define LAMBDA_DIST_FROM_GOAL 0.0
#define LAMBDA_DIST_FROM_PAIR 0.0
#define STATE_INIT_CONSTRAINT 100.0 //0.3
#define TRANSL_INIT_CONSTRAINT 100.0 //0.4 // 0.01
#define ROT_INIT_CONSTRAINT 100.0 


WorldSQP::WorldSQP(int num_worlds, int size_each_state, int num_traj)
{
  _num_worlds = 0;
  _size_each_state = 0;
  _num_traj = 0;
  _wsqp_manager = new WorldManager();
  
  resize_controller(num_worlds, size_each_state, num_traj);
  strcpy(_namestring, "");
}

WorldSQP::~WorldSQP()
{

}

/* 
 * num_worlds is the length of individual trajectory
 * size_each_state is the size of each world
 * num_traj is the number of trajectories
 *
 */
void WorldSQP::resize_controller(int num_worlds, int size_each_state, int num_traj)
{
  if (num_worlds == _num_worlds && size_each_state == _size_each_state && _num_traj == num_traj)
    return;

  _num_worlds = num_worlds;
  _num_traj = num_traj; 
  _size_each_state = size_each_state; 
  _cols_all_unknown_states = num_traj*(num_worlds-2)*_size_each_state;
  _all_trans.resize(_size_each_state*(_num_worlds-1)*_num_traj, (_num_worlds-2)*_size_each_state*_num_traj + (_num_worlds-1)*_size_each_control);

  _all_trans.setZero();
  init_all_trans();
}

void WorldSQP::initializeClosedLoopStepper(World* start, vector<vector<World*> >& target) {
  VectorXd state;
  world_to_state(start, state); 
  resize_controller(target[0].size() + 1, state.size(), target.size()); 

  current_states.resize(target.size());
  current_jacobians.resize(target.size());
  current_controls.resize(target[0].size());

  #pragma omp parallel for
  for (int i = 0; i < target.size(); i++) {
    current_states[i].resize(target[i].size());
    current_jacobians[i].resize(target[i].size());
    for (int j = 0; j < target[i].size(); j++) {
      current_states[i][j]    = new World(*target[i][j]);
      current_jacobians[i][j] = MatrixXd();
    }
  }
  
  for (int k = 0; k < target[0].size(); k++) {
    current_controls[k].resize(2);
    for (int j = 0; j < 2; j++) { //hack
      current_controls[k][j] = new Control();
    }
  }

  pushStart(start);
}


void WorldSQP::pushGoal(vector<World*>& states) {
#if COLOCATION_METHOD
  assert(states.size() == current_states.size());
  for (int i = 0; i < current_states.size(); i++) { 
    current_states[i].push_back(new World(*states[i]));
    current_jacobians[i].push_back(MatrixXd());
  }
  //cout << "Push Goal: " << current_states[0].size() << endl;
#else
  assert(false);
#endif
}

void WorldSQP::pushGoal(World* state) {
#if COLOCATION_METHOD
  vector<World*> goal_states;
  for (int i = 0; i < current_states.size(); i++) {
    goal_states.push_back(state); //other method handles memory
  }
  pushGoal(goal_states);
#else 
  assert(false); 
#endif
}

void WorldSQP::pushStart(vector<World*>& states) {
#if COLOCATION_METHOD
  assert(states.size() == current_states.size());
  pushGoal(states[0]); // this is corrected for
  for (int j = 0; j < current_states.size(); j++) { 
    int max_ind = current_states[j].size() - 1;
    delete current_states[j][max_ind]; //fix memory leak
    for (int i = max_ind; i > 0; i--) {
      current_states[j][i]    = current_states[j][i-1];
      current_jacobians[j][i] = current_jacobians[j][i-1];
    }
    current_states[j][0] = new World(*states[j]);
    current_jacobians[j][0] = MatrixXd();
  }
  //cout << "Push Start: " << current_states[0].size() << endl;
#else
  assert(false);
#endif
}

void WorldSQP::pushStart(World* state) {
#if COLOCATION_METHOD
  vector<World*> start_states;
  for (int i = 0; i < current_states.size(); i++) {
    start_states.push_back(state); //other method handles memory
  }
  pushStart(start_states);
#else
  assert(false);
#endif
}

void WorldSQP::popStart() {
#if COLOCATION_METHOD
  for (int j = 0; j < current_states.size(); j++) { 
    int max_ind = current_states[j].size() - 1;
    delete current_states[j][0];
    current_states[j][0] = NULL;
    
    for (int i = 0; i < max_ind - 1; i++) {
      current_states[j][i]    = current_states[j][i+1];
      current_jacobians[j][i] = current_jacobians[j][i+1];
    }
    current_states[j][max_ind - 1] = new World(*current_states[j][max_ind]);
    current_jacobians[j][max_ind - 1] = current_jacobians[j][max_ind]; 
  }
  popGoal();
  //cout << "Pop Start: " << current_states[0].size() << endl;
#else
  assert(false);
#endif
}

void WorldSQP::popGoal() {
#if COLOCATION_METHOD
  for (int i = 0; i < current_states.size(); i++) {
    delete current_states[i].back();
    current_states[i].pop_back();
    current_jacobians[i].pop_back();
  }
  //cout << "Pop Goal: " << current_states[0].size() << endl;
#else
  assert(false);
#endif
}

VectorXd WorldSQP::getStartControl() {
#if COLOCATION_METHOD
  vector<Control*> cu = current_controls[0];
  VectorXd vu; 
  current_states[0][0]->ControlToVectorXd(cu, vu);
  vu = current_states[0][0]->JacobianControlStripper(vu);
  return vu; 
#else
  assert(false);
#endif
}

void WorldSQP::solve() {
#if TIMER_ENABLED
  StartClock();
#endif 

#if COLOCATION_METHOD
  assert(current_states.size() == _num_traj);
  cout << current_controls.size() << " " << _num_worlds - 1 << endl;
  assert(current_controls.size() == _num_worlds-1);
  for (int i = 0; i < current_states.size(); i++) {
    assert(current_states[i].size() == _num_worlds); // memory leak if false
  }
#else
  assert(false); //solver not implemented for SHOOTING_METHOD
#endif

  //vector to contain the new states
  VectorXd new_states(_num_traj*(_num_worlds-2)*_size_each_state + (_num_worlds-1)*_size_each_control);

  //setup filenames
  char filename_goalvec[256];
  sprintf(filename_goalvec, "%s/%s_%s", SQP_BASE_FOLDER, _namestring, FILENAME_GOALVEC);
  char filename_alltrans[256];
  sprintf(filename_alltrans, "%s/%s_%s", SQP_BASE_FOLDER, _namestring, FILENAME_ALLTRANS);
  char filename_initctrls[256];
  sprintf(filename_initctrls, "%s/%s_%s", SQP_BASE_FOLDER, _namestring, FILENAME_INIT_CTRLS);

  //compute Jacobians
  compute_all_jacobians();

  double t0 = GetClock(); 
  //write out all Jacobians to file
  SparseMatrix<double> all_trans_sparse(_all_trans);
  Matrix_To_File(all_trans_sparse, filename_alltrans);

  //write out all states to file 
  VectorXd goal_vector(_size_each_state*_num_worlds*_num_traj);
  VectorXd state_for_file(_size_each_state);
  for (int j=0; j < current_states.size(); j++) {
    for (int i=0; i < current_states[j].size(); i++) {
      world_to_state(current_states[j][i], state_for_file);
      goal_vector.segment((_num_worlds)*_size_each_state*j + i*_size_each_state, _size_each_state) = state_for_file;
    }
  }
  Vector_To_File(goal_vector, filename_goalvec);

  //write out all controls to file 
  VectorXd init_controls(_size_each_control*(_num_worlds-1));
  for (int j = 0; j < current_controls.size(); j++) 
	{
    vector<Control*> cu = current_controls[j];
    VectorXd vu;
    current_states[0][j]->ControlToVectorXd(cu, vu);
    vu = current_states[0][j]->JacobianControlStripper(vu);
    assert(vu.size() == _size_each_control);
    init_controls.segment(j*_size_each_control, _size_each_control) = vu;
  }
  Vector_To_File(init_controls, filename_initctrls);


  //prepare to execute solver (MATLAB)
  char filename_statevec_thisiter[256];
  sprintf(filename_statevec_thisiter, "%s/%s_%s%d.txt", SQP_BASE_FOLDER, _namestring, FILENAME_STATEVEC_BASE, 1);
  //char matlab_command[1024];
  //sprintf(matlab_command, "%s -nodisplay -nodesktop -nojvm -r \"solve_sparse(%d, %d, \'%s\', %d, %d, \'%s\', \'%s\', %d, %d, %d)\"", MATLAB_INSTALL, _all_trans.rows(), _all_trans.cols(), filename_alltrans, goal_vector.rows(), goal_vector.cols(), filename_goalvec, filename_statevec_thisiter, _num_worlds, _size_each_state, _size_each_control);
  //char matlab_command[1024];
  //sprintf(matlab_command, "java -jar MatlabClient.jar \"solve_sparse(%d, %d, \'%s\', %d, %d, \'%s\', \'%s\', %d, %d, %d)\"", _all_trans.rows(), _all_trans.cols(), filename_alltrans, goal_vector.rows(), goal_vector.cols(), filename_goalvec, filename_statevec_thisiter, _num_worlds, _size_each_state, _size_each_control);
  char python_command[1024];
  sprintf(python_command, "python ../MotionPlanning/SQPSolver.py solver %d %d \'%s\' %d %d \'%s\' \'%s\' \'%s\' %d %d %d %d %f %f %f %f %f %f %f", _all_trans.rows(), _all_trans.cols(), filename_alltrans, goal_vector.rows(), goal_vector.cols(), filename_goalvec, filename_initctrls, filename_statevec_thisiter, _num_traj, _num_worlds, _size_each_state, _size_each_control, LAMBDA_U, LAMBDA_U_DOT, LAMBDA_DIST_FROM_GOAL, LAMBDA_DIST_FROM_PAIR, STATE_INIT_CONSTRAINT, TRANSL_INIT_CONSTRAINT, ROT_INIT_CONSTRAINT );
  std::cout << "command: " << python_command << std::endl;

#if TIMER_ENABLED
  double JCT = GetClock();
  StartClock();
#endif

  //run solver (MATLAB) // NONO! ITS PYTHON/MOSEK!
  //int return_value = system(matlab_command);
  int return_value = system(python_command);


#if TIMER_ENABLED
  double MST = GetClock();
  StartClock();
#endif 

  //Read solver output
  File_To_Vector(filename_statevec_thisiter, new_states);

  vector<vector<VectorXd> > sqp_intermediate_states;
  sqp_intermediate_states.resize(_num_traj);
  //copy out new states
  for (int j = 0; j < sqp_intermediate_states.size(); j++) { 
    sqp_intermediate_states[j].resize(_num_worlds-2);
    for (int i = 0; i < _num_worlds-2; i++) { 
      sqp_intermediate_states[j][i] = new_states.segment((_num_worlds-2)*j + _size_each_state*i, _size_each_state); 
    }
  }

  //update controls
  vector<VectorXd> new_controls;
  new_controls.resize(_num_worlds-1);
  for (int i=0; i < _num_worlds-1; i++)
  {
    new_controls[i] = new_states.segment(_num_traj*_size_each_state*(_num_worlds-2) + i*_size_each_control, _size_each_control);
  }

  vector<vector<Control*> > c_new_controls; 
  for (int i = 0; i < current_controls.size(); i++) {
    vector<Control*> cu ;
    cu.resize(2); 
    VectorXd vu = current_states[0][i]->JacobianControlWrapper(new_controls[i]);
    current_states[0][i]->VectorXdToControl(vu, cu);
    for (int j = 0; j < 2; j++) {
      cu[j]->setButton(UP, current_controls[i][j]->getButton(UP));
      delete current_controls[i][j];
    }
    current_controls[i].clear();
    c_new_controls.push_back(cu);
  }
  current_controls = c_new_controls;

#if COLOCATION_METHOD
  //take sqp takes, transform to real states, and iterate
  /*vector< vector<World*> > newStates;
  newStates.resize(current_states.size());
  for (int j = 0; j < newStates.size(); j++) { 
    newStates[j].resize(current_states[j].size());
    for (int i = 0; i < current_states[j].size(); i++) {
      newStates[j][i] = new World(*current_states[j][i]);
      delete current_states[j][i]; // TODO: Dependent on memory leak above
      current_states[j][i] = NULL;

    }
    current_states[j].resize(0);
  }
  current_states.resize(0);*/

  //TODO: MOVE THIS OUT OF COLOCATION METHOD!!!

  for (int j = 0; j < current_states.size(); j++) {
    vector<World*> openLoopWorlds;
    openLoopController(current_states[j], current_controls, openLoopWorlds);
    openLoopWorlds.pop_back();
    openLoopWorlds.push_back(new World(*current_states[j].back()));
    for (int i = 0; i < current_states[j].size(); i++) { 
      delete current_states[j][i];
    }
    current_states[j] = openLoopWorlds;
  }
  

  /*cout << "Projecting SQP States into Legal States" << endl; 
  boost::thread_group group; 

  for (int j = 0; j < sqp_intermediate_states.size(); j++) { 
    for (int i = 0; i < sqp_intermediate_states[j].size(); i++) {
      group.create_thread(boost::bind(setWorldFromState,
            newStates[j][i+1], &sqp_intermediate_states[j][i]));
    }
  }
  group.join_all();
  
  
  current_states = newStates;
  */

  for (int j = 0; j < current_jacobians.size(); j++) { 
    for (int i = 0; i < current_jacobians[j].size(); i++) {
      current_jacobians[j][i] = MatrixXd();
    }
  }

#else
  assert(false); // not implemented for SHOOTING_METHOD

#endif

#if TIMER_ENABLED
  cout << "Jacobian Computation Time: " << JCT << endl;
  cout << "MATLAB Solver Time: " << MST << endl;
  cout << "Total Solver Time: " << JCT + MST + GetClock() << endl;
#endif 
}



bool WorldSQP::iterative_control_opt(vector<vector<World*> >& trajectory, vector<vector<Control*> >& controls, int num_opts, bool return_best_opt, double threshold) 
{

  current_states.resize(trajectory.size());
  current_jacobians.resize(trajectory.size());

  for (int i = 0; i < trajectory.size(); i++) {
    current_states[i].resize(trajectory[i].size());
    current_jacobians[i].resize(trajectory[i].size());
    for (int j = 0; j < trajectory[i].size(); j++) { 
      current_states[i][j] = new World(*trajectory[i][j]);
      current_jacobians[i][j] = MatrixXd();
    }
  }
  
  assert(controls.size() == _num_worlds-1);
  current_controls = controls; 

  for (int opt_iter = 0; opt_iter < num_opts; opt_iter++) {
    solve();
  }
 
  
  
  trajectory = current_states;
  controls = current_controls;

  

  return true;
}


void WorldSQP::init_all_trans()
{

  DynamicSparseMatrix<double> D;
  compute_difference_block(D);
  for (int i = 0; i < _num_traj; i++)
  { 
    block(_all_trans,i*_size_each_state*(_num_worlds-1),i*(_num_worlds-2)*_size_each_state, D); 
  }
}


void WorldSQP::compute_difference_block(DynamicSparseMatrix<double>& m)
{
  m.resize(_size_each_state*(_num_worlds-1), _size_each_state*(_num_worlds-2));
  for (int i = 0; i < _num_worlds-1; i++) { 
    int num_rows_start = i*_size_each_state;
    for (int r = 0; r < _size_each_state; r++) {
      if (i != 0) 
        m.coeffRef(num_rows_start+r,num_rows_start+r - _size_each_state) = 1;
      if (i != _num_worlds-2) 
        m.coeffRef(num_rows_start+r,num_rows_start+r) = -1;
    }
  }
}

void WorldSQP::compute_all_jacobians()
{
  
  boost::thread_group group;

  for (int traj_ind = 0; traj_ind < _num_traj; traj_ind++) {
    cout << "Computing Jacobians on traj = " << traj_ind << endl; ;
    for (int i=0; i < current_states[traj_ind].size()-1; i++)
    {
      if (current_jacobians[traj_ind][i].rows() == 0) {
        group.create_thread( boost::bind(computeJacobian, 
              current_states[traj_ind][i],
              &current_jacobians[traj_ind][i]) );
      }
    }
  }

  group.join_all();

  for (int traj_ind = 0; traj_ind < _num_traj; traj_ind++) { 
    DynamicSparseMatrix<double> J; 
    J.resize((_num_worlds-1)*_size_each_state, (_num_worlds-1)*_size_each_control);
    for (int i = 0; i < current_states[traj_ind].size()-1; i++) 
    {
      int num_rows_start = i*_size_each_state;
      int num_cols_start = i*_size_each_control;
      for (int r=0; r < _size_each_state; r++)
      {
        for (int c=0; c < _size_each_control; c++)
        {
          J.coeffRef(num_rows_start+r, num_cols_start+c) = current_jacobians[traj_ind][i](r,c);
        }
      }
      //++progress; 
    }
    block(_all_trans,traj_ind*_size_each_state*(_num_worlds-1), _cols_all_unknown_states, J);
  }
}

void WorldSQP::world_to_state(World* world, VectorXd& state)
{
  world->getStateForJacobian(state); 
}

void Matrix_To_File(SparseMatrix<double> mat, const char* filename)
{
  ofstream toFile(filename);
  toFile.precision(10);
  for (int k=0; k < mat.outerSize(); ++k) {
    for (SparseMatrix<double>::InnerIterator it(mat,k); it; ++it)
    {
      toFile << (it.row()+1) << "\t" << (it.col()+1) << "\t" << it.value() <<"\n";
    }
  }
  toFile.close();
}

void File_To_Vector(const char* filename, VectorXd& vec)
{
  ifstream fromFile(filename);
  char temp_holder[256];
  for (int i=0; i < vec.rows(); i++)
  {
    fromFile >> temp_holder;
    vec(i) = atof(temp_holder);
  }
}

void Vector_To_File(VectorXd& vec, const char* filename)
{
  ofstream toFile(filename);
  toFile << vec << std::endl;
  toFile.close();
}

void block(DynamicSparseMatrix<double>& container, int start_row, int start_col, DynamicSparseMatrix<double>& data) {
  for (int k = 0; k < data.outerSize(); ++k) {
    for(DynamicSparseMatrix<double>::InnerIterator it(data,k); it; ++it) {
      container.coeffRef(start_row+it.row(), start_col+it.col()) = it.value();
    }
  }
}

void block(DynamicSparseMatrix<double>& container, int start_row, int start_col, MatrixXd& data) {
  for (int i = 0; i < data.rows(); i++) { 
    for (int j = 0; j < data.cols(); j++) {
      container.coeffRef(start_row+i, start_col+j) = data(i,j);
    }
  }
}

void computeJacobian(World* w, MatrixXd* J) {
  w->computeJacobian(J);
}

void setWorldFromState(World* w, VectorXd* s) {
  w->setStateForJacobian(*s);
  w->projectLegalState();
}

