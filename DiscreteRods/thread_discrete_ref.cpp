#include "thread_discrete.h"

Thread::Thread()
{
  _thread_pieces.resize(0);
}

Thread::Thread(vector<Vector3d>& vertices, vector<double>& twist_angles, Matrix3d& start_rot)
{
  _thread_pieces.resize(vertices.size());
  _thread_pieces_backup.resize(vertices.size());
  _angle_twist_backup.resize(vertices.size());
  for (int i=0; i < vertices.size(); i++)
  {
    _thread_pieces[i] = ThreadPiece(vertices[i], twist_angles[i]);
//    _thread_pieces.push_back(ThreadPiece(vertices[i], twist_angles[i]));
  }

  for (int i=1; i < vertices.size(); i++)
  {
    _thread_pieces[i].set_prev(&_thread_pieces[i-1]);
  }

  for (int i=0; i < vertices.size()-1; i++)
  {
    _thread_pieces[i].set_next(&_thread_pieces[i+1]);
  }

  _thread_pieces.front().set_bishop_frame(start_rot);
  _thread_pieces.front().set_material_frame(start_rot);

  _thread_pieces.front().initializeFrames();


 // _saved_last_thetas.resize(_thread_pieces.size());
  //_saved_last_theta_changes.resize(_thread_pieces.size());




  set_start_constraint(vertices.front(), start_rot);




  Matrix3d end_bishop = _thread_pieces[_thread_pieces.size()-2].bishop_frame();
  Matrix3d end_rot = Eigen::AngleAxisd(twist_angles[twist_angles.size()-2], end_bishop.col(0).normalized())*end_bishop;



  set_end_constraint(vertices.back(), end_rot);




  //set_end_constraint(vertices.back(), end_rot);
  //project_length_constraint();



/*
  vector<Vector3d> vertices_actual;
  vector<Matrix3d> material_frames;

  get_thread_data(vertices_actual, material_frames);

  std::cout << "init data:\n";
  for (int i=0; i < _thread_pieces.size(); i++)
  {
    if (i < _thread_pieces.size()-2)
    _thread_pieces[i].update_material_frame();
    std::cout << "vertex:\n" << vertices_actual[i] << "\nmaterial frame:\n" << material_frames[i] << std::endl;
  }


  Matrix3d last_bishop = _thread_pieces[_thread_pieces.size()-2].bishop_frame();
  std::cout << "last bishop:\n" << last_bishop << std::endl;
  std::cout << "last material:\n" << _thread_pieces.back().material_frame() << std::endl;
  std::cout << "last angle:\n" << _thread_pieces[_thread_pieces.size()-2].angle_twist() << std::endl;


  Vector3d offset(2.0, 2.0, 0.0);
 // set_start_constraint(offset, start_rot);
  _thread_pieces[3].offset_vertex(offset);

*/
  /*
  int numInit = 50;
  Vector3d positions[numInit];
  double angles[numInit];
  for (int i=0; i < numInit; i++)
  {
    positions[i](0) = 10*(int)((i)/2.0)%2;
    positions[i](1) = 10*floor((i+1)/2);
    positions[i](2) = i*10;
    angles[i] =0;
    _thread_pieces.push_back(ThreadPiece(positions[i], angles[i]));
    //std::cout << positions[i] << std::endl << std::endl;
  }


  Vector3d tan = (_thread_pieces[1].vertex() - _thread_pieces[0].vertex()).normalized();
  Vector3d toRotAxis;
  Matrix3d rot_for_frame;
  if ( (tan -Vector3d::UnitX()).norm() < 0.01 )
  {
    rot_for_frame = Matrix3d::Identity();
  }
  else
  {
    toRotAxis = Vector3d::UnitX().cross(tan);
    double Axisnorm = toRotAxis.norm();
    double toRotAng = asin(Axisnorm);

    toRotAxis /= Axisnorm;
   // std::cout << "axis: " << toRotAxis << std::endl;
   // std::cout << "ang: " << toRotAng << std::endl;

    rot_for_frame = (Eigen::AngleAxisd(toRotAng, toRotAxis));
  }
  //std::cout << "rot for frame: " << rot_for_frame << std::endl;

  _start_rot = rot_for_frame*Matrix3d::Identity();

  //std::cout << "start frame: " << _start_rot << std::endl;

  std::cout << "energy: " << calculate_energy() << std::endl;
*/

}




Thread::Thread(vector<Vector3d>& vertices, vector<double>& twist_angles, Matrix3d& start_rot, Matrix3d& end_rot)
{
  //_thread_pieces.resize(vertices.size());
  _thread_pieces_backup.resize(vertices.size());
  _angle_twist_backup.resize(vertices.size());
  _thread_pieces.resize(vertices.size());
  for (int i=0; i < vertices.size(); i++)
  {
    _thread_pieces[i] = ThreadPiece(vertices[i], twist_angles[i]);
    //_thread_pieces.push_back(ThreadPiece(vertices[i], twist_angles[i]));
  }

  for (int i=1; i < vertices.size(); i++)
  {
    _thread_pieces[i].set_prev(&_thread_pieces[i-1]);
  }

  for (int i=0; i < vertices.size()-1; i++)
  {
    _thread_pieces[i].set_next(&_thread_pieces[i+1]);
  }

  _thread_pieces.front().set_bishop_frame(start_rot);
  _thread_pieces.front().set_material_frame(start_rot);

  _thread_pieces.front().initializeFrames();

 // _saved_last_thetas.resize(_thread_pieces.size());
  //_saved_last_theta_changes.resize(_thread_pieces.size());

  set_constraints(vertices.front(), start_rot, vertices.back(), end_rot);



  //set_end_constraint(vertices.back(), end_rot);
  //project_length_constraint();



/*
  vector<Vector3d> vertices_actual;
  vector<Matrix3d> material_frames;

  get_thread_data(vertices_actual, material_frames);

  std::cout << "init data:\n";
  for (int i=0; i < _thread_pieces.size(); i++)
  {
    if (i < _thread_pieces.size()-2)
    _thread_pieces[i].update_material_frame();
    std::cout << "vertex:\n" << vertices_actual[i] << "\nmaterial frame:\n" << material_frames[i] << std::endl;
  }


  Matrix3d last_bishop = _thread_pieces[_thread_pieces.size()-2].bishop_frame();
  std::cout << "last bishop:\n" << last_bishop << std::endl;
  std::cout << "last material:\n" << _thread_pieces.back().material_frame() << std::endl;
  std::cout << "last angle:\n" << _thread_pieces[_thread_pieces.size()-2].angle_twist() << std::endl;


  Vector3d offset(2.0, 2.0, 0.0);
 // set_start_constraint(offset, start_rot);
  _thread_pieces[3].offset_vertex(offset);

*/
  /*
  int numInit = 50;
  Vector3d positions[numInit];
  double angles[numInit];
  for (int i=0; i < numInit; i++)
  {
    positions[i](0) = 10*(int)((i)/2.0)%2;
    positions[i](1) = 10*floor((i+1)/2);
    positions[i](2) = i*10;
    angles[i] =0;
    _thread_pieces.push_back(ThreadPiece(positions[i], angles[i]));
    //std::cout << positions[i] << std::endl << std::endl;
  }


  Vector3d tan = (_thread_pieces[1].vertex() - _thread_pieces[0].vertex()).normalized();
  Vector3d toRotAxis;
  Matrix3d rot_for_frame;
  if ( (tan -Vector3d::UnitX()).norm() < 0.01 )
  {
    rot_for_frame = Matrix3d::Identity();
  }
  else
  {
    toRotAxis = Vector3d::UnitX().cross(tan);
    double Axisnorm = toRotAxis.norm();
    double toRotAng = asin(Axisnorm);

    toRotAxis /= Axisnorm;
   // std::cout << "axis: " << toRotAxis << std::endl;
   // std::cout << "ang: " << toRotAng << std::endl;

    rot_for_frame = (Eigen::AngleAxisd(toRotAng, toRotAxis));
  }
  //std::cout << "rot for frame: " << rot_for_frame << std::endl;

  _start_rot = rot_for_frame*Matrix3d::Identity();

  //std::cout << "start frame: " << _start_rot << std::endl;

  std::cout << "energy: " << calculate_energy() << std::endl;
*/

}

Thread::Thread(const Thread& rhs)
{
  _thread_pieces.resize(rhs._thread_pieces.size());
  _thread_pieces_backup.resize(rhs._thread_pieces.size());
  _angle_twist_backup.resize(rhs._thread_pieces.size());

  for (int piece_ind =0; piece_ind < rhs._thread_pieces.size(); piece_ind++)
  {
    //_thread_pieces.push_back(rhs._thread_pieces[piece_ind]);
    _thread_pieces[piece_ind] = rhs._thread_pieces[piece_ind];
  }

  for (int i=1; i < _thread_pieces.size(); i++)
  {
    _thread_pieces[i].set_prev(&_thread_pieces[i-1]);
  }

  for (int i=0; i < _thread_pieces.size()-1; i++)
  {
    _thread_pieces[i].set_next(&_thread_pieces[i+1]);
  }


  _thread_pieces.front().set_bishop_frame(rhs.start_rot());
  _thread_pieces.front().set_material_frame(rhs.start_rot());

  _thread_pieces.front().initializeFrames();

  //_saved_last_theta_changes.resize(_thread_pieces.size());

  Matrix3d start_rot = rhs.start_rot();
  Matrix3d end_rot = rhs.end_rot();
  Vector3d start_pos = rhs.start_pos();
  Vector3d end_pos = rhs.end_pos();

  //set_constraints(start_pos, start_rot, end_pos, end_rot);
  set_start_constraint(start_pos, start_rot);


  set_end_constraint(end_pos, this->end_rot());
  //project_length_constraint();


}

Thread::~Thread()
{

}


double Thread::calculate_energy()
{
  /*
  double energy = 0.0;
  Matrix3d frame_prev = _start_rot;
  _thread_pieces.front().applyTwist(frame_prev);
  Matrix3d frame_after;
  Vector3d edge_prev = _thread_pieces[1].vertex() - _thread_pieces[0].vertex();
  Vector3d edge_after;

  for (int piece_ind = 1; piece_ind < _thread_pieces.size()-2; piece_ind++)
  {
    //std::cout << "piece ind: " << piece_ind << "  out of " <<  _thread_pieces.size() << std::endl;
    //std::cout << "vertex:\n" << _thread_pieces[piece_ind].vertex() << std::endl;
    energy += _thread_pieces[piece_ind].energy(frame_prev, edge_prev, _thread_pieces[piece_ind+1], frame_after, edge_after);
    //std::cout << energy << std::endl;
    //std::cout << "frame after:\n" <<  frame_after << std::endl;
    frame_prev = frame_after;
    edge_prev = edge_after;
    if (isnan(energy))
    {
      std::cout << "energy is nan in calc energy" << std::endl;
      exit(0);
    }
  }


  //for final piece, tell it to twist to goal frame
  energy += _thread_pieces[_thread_pieces.size()-2].energy_with_goalframe(frame_prev, edge_prev, _thread_pieces[_thread_pieces.size()-1], frame_after, edge_after, _end_rot);



  return energy;

  */
#ifdef ISOTROPIC
  double energy = _thread_pieces[2].get_twist_coeff()*(pow(_thread_pieces[_thread_pieces.size()-2].angle_twist()-_thread_pieces.front().angle_twist(),2))/(2.0*_rest_length*(_thread_pieces.size()-2));

  for (int piece_ind = 0; piece_ind < _thread_pieces.size(); piece_ind++)
  {
    energy += _thread_pieces[piece_ind].energy_curvature() + _thread_pieces[piece_ind].energy_grav();
  }
  return energy;
#else
  double energy = 0.0;
  for (int piece_ind = 0; piece_ind < _thread_pieces.size(); piece_ind++)
  {
    energy += _thread_pieces[piece_ind].energy();
  }
  return energy;
#endif
}



void Thread::minimize_energy_bfgs()
{
  double step_in_grad_dir_vertices = 1.0;
  double grad_alpha = 0.4;
  double grad_beta = 0.8;



  num_iters_twist_est = num_iters_twist_est_max;

  const int num_opt_iters = 6000;
  const double energy_error_for_convergence = 1e-5;


  int N = _thread_pieces.size();
  VectorXd vertex_gradients(3*N);
  vertex_gradients.setZero();
  VectorXd new_gradients(3*N);
  new_gradients.setZero();
  VectorXd yk(3*N);
  new_gradients.setZero();
  MatrixXd Binv = MatrixXd::Identity(3*N,3*N);

  int opt_iter;

  double max_movement_vertices = MAX_MOVEMENT_VERTICES;
  for (opt_iter = 0; opt_iter < num_opt_iters; opt_iter++)
    {
      save_thread_pieces();

      calculate_gradient_vertices_vectorized(&new_gradients);
      yk = new_gradients - vertex_gradients;
      vertex_gradients = Binv*new_gradients;

      //add_momentum_to_gradient(vertex_gradients, new_gradients, step_in_grad_dir_vertices);

      double curr_energy = calculate_energy();
      double next_energy = 0.0;

      //binary search on energy function while satisfying wolfe condition
      step_in_grad_dir_vertices = max_movement_vertices/2.0;
      apply_vertex_offsets_vectorized(vertex_gradients, true, -(step_in_grad_dir_vertices));
      double search_step = max_movement_vertices/4.0;
      double start_energy = calculate_energy();
      while (search_step > MIN_MOVEMENT_VERTICES)
        //while (search_step > 1e-12)
        {


          //energy for adding search_step to current step size
          apply_vertex_offsets_vectorized(vertex_gradients, true, -search_step);
          minimize_energy_twist_angles();
          double energy_add = calculate_energy();

          //energy for subtracting search_step to current step size
          apply_vertex_offsets_vectorized(vertex_gradients, true, 2.0*search_step);
          minimize_energy_twist_angles();
          double energy_subtract = calculate_energy();

          if (start_energy < energy_add || start_energy < energy_subtract) {



              if (energy_add < energy_subtract) {
                apply_vertex_offsets_vectorized(vertex_gradients, true, -2.0*search_step);
                step_in_grad_dir_vertices += search_step;
                start_energy = energy_add;
              } else {
                step_in_grad_dir_vertices -= search_step;
                start_energy = energy_subtract;
              }
            }

          search_step /= 2.0;

        }




      minimize_energy_twist_angles();
      double energy_before_projection = calculate_energy();
      project_length_constraint();
      minimize_energy_twist_angles(true);


      next_energy = calculate_energy();

      if (next_energy + energy_error_for_convergence > curr_energy)
        {
          max_movement_vertices = step_in_grad_dir_vertices / 2.0;
          restore_thread_pieces();

          if (max_movement_vertices <= MIN_MOVEMENT_VERTICES)
            break;


        } else {
        max_movement_vertices = min(step_in_grad_dir_vertices*2.0, MAX_MOVEMENT_VERTICES);
      }

      //std::cout << "max: " << max_movement_vertices << std::endl;
      VectorXd sk = -step_in_grad_dir_vertices*vertex_gradients;
      double skTyk = (sk.transpose()*yk)(0);
      Binv = Binv + (skTyk + (yk.transpose()*Binv*yk)(0))*(sk*sk.transpose()) / (skTyk*skTyk) - (Binv*yk*sk.transpose() + sk*yk.transpose()*Binv)/skTyk;
      cout << "Binv: " << endl;
      for(int i = 0; i < 3*N; i++) {
        for(int j = 0; j < 3*N; j++) {
          if (abs(i-j) > 0) {
            Binv(i,j) = 0;
          }
          if (Binv(i,j) < 1e-3) {
            Binv(i,j) = 0;
          }

          cout << Binv(i,j) << " ";
        }
        cout << endl;
      }
    }


  double curr_energy, next_energy;
  curr_energy = calculate_energy();

  project_length_constraint();
  minimize_energy_twist_angles(true);

  next_energy = calculate_energy();

  //std::cout << "num iters: " << opt_iter << " curr energy final: " << curr_energy << "   next energy final: " << next_energy <<  std::endl;


}



void Thread::minimize_energy_noiseStart()
{
  //add noise
  double vertex_noise_factor = 0.1;
  vector<Vector3d> noise_offsets(_thread_pieces.size());
  Vector3d ones_vec(1,1,1);
  for (int piece_ind =2; piece_ind < _thread_pieces.size()-2; piece_ind++)
  {
   Vector3d noise( ((double)(rand()%20000))/ 10000.0, ((double)(rand()%20000)) / 10000.0, ((double)(rand()%20000)) / 10000.0);
   noise_offsets[piece_ind] = noise-ones_vec;
  }
  apply_vertex_offsets(noise_offsets, true, vertex_noise_factor);

  minimize_energy();
}



void Thread::minimize_energy()
{
  double step_in_grad_dir_vertices = 1.0;
//  double grad_alpha = 0.4;
//  double grad_beta = 0.8;


  //vector<Vector3d> old_vertices;
  //vector<double> old_angles;


  num_iters_twist_est = num_iters_twist_est_max;

  const int num_opt_iters = 6000;
  const double energy_error_for_convergence = 1e-5;

  vector<Vector3d> vertex_gradients(_thread_pieces.size());
  for (int piece_ind=0; piece_ind < vertex_gradients.size(); piece_ind++)
  {
    vertex_gradients[piece_ind].setZero();
  }
  //vector<Vector3d> new_gradients(_thread_pieces.size());

  int opt_iter;

  double max_movement_vertices = MAX_MOVEMENT_VERTICES;
  project_length_constraint();
  bool recalc_vertex_grad = true;

  double curr_energy = calculate_energy();
  double next_energy = 0.0;

  for (opt_iter = 0; opt_iter < num_opt_iters; opt_iter++)
  {
    //save_thread_pieces();
    //_saved_last_theta = _thread_pieces[_thread_pieces.size()-2].angle_twist();
    /*for (int piece_ind=0; piece_ind < _thread_pieces.size()-2; piece_ind++)
    {
      _saved_last_thetas[piece_ind] = _thread_pieces[piece_ind].angle_twist();
    }*/
    //std::cout << "opt iter vert: " << opt_iter << std::endl;
    //calculate_gradient_vertices(vertex_gradients);
    //calculate_gradient_vertices(new_gradients);
   // add_momentum_to_gradient(vertex_gradients, new_gradients, step_in_grad_dir_vertices);

    //for backtracing line search:


    /*double wolfe_condition_vertices = 0.0;
    for (int piece_ind = 2; piece_ind < _thread_pieces.size()-2; piece_ind++)
    {
      wolfe_condition_vertices += new_gradients[piece_ind].squaredNorm();
      //std::cout << "piece " << piece_ind << " has gradient " << vertex_gradients[piece_ind].transpose() << std::endl;
    }
    wolfe_condition_vertices *= -grad_alpha;
*/
   double curr_energy = calculate_energy();
    double next_energy = 0.0;
    //get_thread_data(old_vertices, old_angles);

    //double curr_energy = calculate_energy();
    //double next_energy = 0.0;

    //add_momentum_to_gradient(vertex_gradients, new_gradients, step_in_grad_dir_vertices);

    //step_in_grad_dir_vertices = 1.2/grad_beta;
    //step_in_grad_dir_vertices = MAX_MOVEMENT_VERTICES;


    //make sure step size not greater than limit
    /*double max_grad_norm = (*(std::max_element(vertex_gradients.begin()+2, vertex_gradients.end()-2, compare_vector_norms))).norm();
    //std::cout << "max grad norm: " << max_grad_norm << std::endl;
    if (max_grad_norm*step_in_grad_dir_vertices > MAX_MOVEMENT_VERTICES*_rest_length)
    {
      step_in_grad_dir_vertices = (MAX_MOVEMENT_VERTICES*_rest_length / max_grad_norm);
      //std::cout << "new step: " << step_in_grad_dir_vertices << std::endl;
    }*/

/*
    apply_vertex_offsets(vertex_gradients, true, -(2.0-grad_beta)*(step_in_grad_dir_vertices));
    do {
      step_in_grad_dir_vertices *= grad_beta;
      apply_vertex_offsets(vertex_gradients, true, (1.0-grad_beta)*step_in_grad_dir_vertices);


      next_energy = calculate_energy();
      if (isnan(next_energy))
      {
        std::cout << "energy nan" << std::endl;
        exit(0);
      }
    /std::cout << "opt iter num: " << opt_iter << " curr energy final: " << curr_energy << "   next energy final: " << next_energy <<  "wolfe: "  << wolfe_condition_vertices*step_in_grad_dir_vertices << std::endl;
    } while (next_energy > curr_energy + wolfe_condition_vertices*step_in_grad_dir_vertices && step_in_grad_dir_vertices > 1e-20);
*/

    //binary search on energy function while satisfying wolfe condition



    save_thread_pieces();
    if (recalc_vertex_grad)
    {
      calculate_gradient_vertices(vertex_gradients);
      make_max_norm_one(vertex_gradients);
      curr_energy = calculate_energy();
    }
   
 

    step_in_grad_dir_vertices = max_movement_vertices/2.0;
    apply_vertex_offsets(vertex_gradients, true, -(step_in_grad_dir_vertices));
    minimize_energy_twist_angles();
    double search_step = max_movement_vertices/4.0;
    while (search_step > MIN_MOVEMENT_VERTICES)
    {


      //energy for adding search_step to current step size
      apply_vertex_offsets(vertex_gradients, true, -search_step);      
#ifndef ISOTROPIC
      minimize_energy_twist_angles();
#endif
      double energy_add = calculate_energy();
      //  if (energy_add > curr_energy + wolfe_condition_vertices*(step_in_grad_dir_vertices+search_step))
      //    energy_add = DBL_MAX;

      //energy for subtracting search_step to current step size
      apply_vertex_offsets(vertex_gradients, true, 2.0*search_step);
#ifndef ISOTROPIC
      minimize_energy_twist_angles();
#endif
      double energy_subtract = calculate_energy();
      //     if (energy_subtract > curr_energy + wolfe_condition_vertices*(step_in_grad_dir_vertices-search_step))
      //      energy_subtract = DBL_MAX;

      //std::cout << "energy add: " << energy_add << "  energy_subtract: " << energy_subtract << "  search step: " << search_step << " curr step: " << step_in_grad_dir_vertices << std::endl;

      if (energy_add < energy_subtract) {
        apply_vertex_offsets(vertex_gradients, true, -2.0*search_step);
        step_in_grad_dir_vertices += search_step;
      } else {
        step_in_grad_dir_vertices -= search_step;
      }
      search_step /= 2.0;

      minimize_energy_twist_angles();

    }

    minimize_energy_twist_angles();
    double energy_before_projection = calculate_energy();
    project_length_constraint();
    minimize_energy_twist_angles(true);


    next_energy = calculate_energy();


    //std::cout << "curr energy: " << curr_energy << "   next energy: " << next_energy << "  before projection: " << energy_before_projection << "  last step: " << step_in_grad_dir_vertices <<  std::endl;

    recalc_vertex_grad = true;
    if (next_energy + energy_error_for_convergence > curr_energy)
    {
      if (next_energy >= curr_energy) {
        restore_thread_pieces();
        recalc_vertex_grad = false;
      } 

      if (max_movement_vertices <= MIN_MOVEMENT_VERTICES)
      {
        break;
      }
      max_movement_vertices = step_in_grad_dir_vertices / 2.0;
    } else {
      max_movement_vertices = min(step_in_grad_dir_vertices*2.0, MAX_MOVEMENT_VERTICES);
      //max_movement_vertices = MAX_MOVEMENT_VERTICES;
    }


  /*

    if (next_energy + energy_error_for_convergence > curr_energy)
    {
        restore_thread_pieces();
        recalc_vertex_grad = false;
        max_movement_vertices = step_in_grad_dir_vertices / 2.0;

      if (max_movement_vertices <= MIN_MOVEMENT_VERTICES)
      {
        break;
      }
    } else {
      max_movement_vertices = min(step_in_grad_dir_vertices*2.0, MAX_MOVEMENT_VERTICES);
      //max_movement_vertices = MAX_MOVEMENT_VERTICES;
    }
 
    //std::cout << "max: " << max_movement_vertices << std::endl;

  }
*/

  curr_energy = calculate_energy();

  project_length_constraint();
  minimize_energy_twist_angles(true);

  next_energy = calculate_energy();

  //std::cout << "num iters: " << opt_iter << " curr energy final: " << curr_energy << "   next energy final: " << next_energy <<  std::endl;


}


void Thread::one_step_project()
{


  vector<Vector3d> vertex_gradients(_thread_pieces.size());
 /* for (int piece_ind=0; piece_ind < vertex_gradients.size(); piece_ind++)
  {
    vertex_gradients[piece_ind].setZero();
  }
  */

  calculate_gradient_vertices(vertex_gradients);

  double norm_vecs = 0.0;
  for (int piece_ind=2; piece_ind < _thread_pieces.size()-2; piece_ind++)
  {
    norm_vecs += pow(vertex_gradients[piece_ind].norm(),2);
    //norm_vecs += vertex_gradients[piece_ind].norm();
  }
  norm_vecs = sqrt(norm_vecs);

  //std::cout << "norm vecs: " << norm_vecs << std::endl;


 // make_max_norm_one(vertex_gradients);
  apply_vertex_offsets(vertex_gradients, true, -(0.1/norm_vecs), true);
  project_length_constraint();

}




void Thread::minimize_energy_twist_angles(bool force_optimization)
{
#ifdef ISOTROPIC
  double angle_per = (_thread_pieces[_thread_pieces.size()-2].angle_twist())/((double)_thread_pieces.size()-2);
  //std::cout << "angle per: " << angle_per << std::endl;

  //#pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
  for (int piece_ind = 1; piece_ind < _thread_pieces.size()-2; piece_ind++)
  {
    _thread_pieces[piece_ind].set_angle_twist(angle_per*piece_ind);
    _thread_pieces[piece_ind].updateFrames_twistOnly();
  }

#else

   /*
  //if (num_iters_twist_est < num_iters_twist_est_max && !force_optimization && abs(_saved_last_theta_change) > 1e-20)
  if (num_iters_twist_est < num_iters_twist_est_max && !force_optimization)
  {
    num_iters_twist_est++;
    //this time, just estimate

  /*
    double theta_ratio = (_thread_pieces[_thread_pieces.size()-2].angle_twist() -_saved_last_theta)/_saved_last_theta_change;
    double new_thetas[_thread_pieces.size()];

    //apply_angle_twist_offsets(_saved_last_theta_changes, true, theta_change);

    new_thetas[0] = _thread_pieces[0].angle_twist();
    #pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
    for (int piece_ind = 1; piece_ind < _thread_pieces.size()-2; piece_ind++)
    {
      //offset_thetas[piece_ind] = _saved_last_theta_changes[piece_ind]
      new_thetas[piece_ind] = _thread_pieces[piece_ind].angle_twist() + _saved_last_theta_changes[piece_ind]*theta_ratio;
    }

    #pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
    for (int piece_ind = 1; piece_ind < _thread_pieces.size()-2; piece_ind++)
    {
      _thread_pieces[piece_ind].set_angle_twist(new_thetas[piece_ind]);
      _thread_pieces[piece_ind].updateFrames_twistOnly();
    }

*/


/*
    double total_theta = _thread_pieces[_thread_pieces.size()-2].angle_twist();
    double theta_ratio = total_theta/_saved_last_theta;
    double new_thetas[_thread_pieces.size()];

    #pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
    for (int piece_ind = 1; piece_ind < _thread_pieces.size()-2; piece_ind++)
    {
      new_thetas[piece_ind] = ((_thread_pieces[piece_ind].angle_twist() - _thread_pieces[piece_ind-1].angle_twist()) * theta_ratio + _thread_pieces[piece_ind-1].angle_twist());
    }

    #pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
    for (int piece_ind = 1; piece_ind < _thread_pieces.size()-2; piece_ind++)
    {
      _thread_pieces[piece_ind].set_angle_twist(new_thetas[piece_ind]);
      _thread_pieces[piece_ind].updateFrames_twistOnly();
    }


    return;
  }
*/

  num_iters_twist_est = 0;


  double step_in_grad_dir_twist = 1.0;
  double max_rotation_twist = MAX_ROTATION_TWIST;
 // double grad_alpha = 0.4;
 // double grad_beta = 0.8;

  const int num_opt_iters = 100;
  const double energy_error_for_convergence = 1e-20;


  vector<double> angle_twist_gradients;
  angle_twist_gradients.resize(_thread_pieces.size());



  //vector<double> angles_before(_thread_pieces.size());
  //vector<double> angles_after(_thread_pieces.size());
  //save_angle_twists(angles_before);
  int opt_iter; 
  for (opt_iter = 0; opt_iter < num_opt_iters; opt_iter++)
  {
   // std::cout << "opt iter twist: " << opt_iter << std::endl;
    calculate_gradient_twist(angle_twist_gradients);
    double max_grad_norm = (*(std::max_element(angle_twist_gradients.begin()+1, angle_twist_gradients.end()-2)));
    double mult_const = 1.0/max_grad_norm;

    for (int piece_ind=1; piece_ind < _thread_pieces.size()-1; piece_ind++)
    {
      angle_twist_gradients[piece_ind] *= mult_const;
    }

    


 /*   //for backtracing line search:
    double wolfe_condition_angles = 0.0;
    for (int piece_ind = 1; piece_ind < _thread_pieces.size()-2; piece_ind++)
    {
      wolfe_condition_angles += angle_twist_gradients[piece_ind]*angle_twist_gradients[piece_ind];
    }
    */

    double curr_energy = calculate_energy();
    double next_energy = 0.0;

    step_in_grad_dir_twist = max_rotation_twist/2.0;
    apply_angle_twist_offsets(angle_twist_gradients, true, -(step_in_grad_dir_twist));   
    double search_step = max_rotation_twist/4.0;
    while (search_step > MIN_ROTATION_TWIST)
    {
      //energy for adding search_step to current step size
      apply_angle_twist_offsets(angle_twist_gradients, true, -search_step);
      double energy_add = calculate_energy();

      apply_angle_twist_offsets(angle_twist_gradients, true, 2.0*search_step);
      double energy_subtract = calculate_energy();

      if (energy_add < energy_subtract) {
        apply_angle_twist_offsets(angle_twist_gradients, true, -2.0*search_step);
        step_in_grad_dir_twist += search_step;
      } else {
        step_in_grad_dir_twist -= search_step;
      }
      search_step /= 2.0;
    }


    next_energy = calculate_energy();


    //std::cout << "curr energy: " << curr_energy << "   next energy: " << next_energy << "  last step: " << step_in_grad_dir_twist <<  std::endl;

    if (next_energy + energy_error_for_convergence > curr_energy)
    {
        break;
    } else {
      max_rotation_twist = min(step_in_grad_dir_twist*2.0, MAX_ROTATION_TWIST);
    }

    //std::cout << "max: " << max_movement_vertices << std::endl;

  }
  double curr_energy, next_energy;
  curr_energy = calculate_energy();
  next_energy = calculate_energy();

  //std::cout << "num iters: " << opt_iter << " curr energy final: " << curr_energy << "   next energy final: " << next_energy <<  std::endl;




    
    /*step_in_grad_dir_twist *= 2.0/grad_beta;
    wolfe_condition_angles *= -grad_alpha;

    //std::cout << "energy: " << curr_energy  << "   plus: " << wolfe_condition << std::endl;

    apply_angle_twist_offsets(angle_twist_gradients, true, -(2.0-grad_beta)*(step_in_grad_dir_twist*grad_beta));
    do {
      //std::cout << "step size: " << step_in_grad_dir << std::endl;

      step_in_grad_dir_twist *= grad_beta;
      apply_angle_twist_offsets(angle_twist_gradients, true, (1.0-grad_beta)*step_in_grad_dir_twist);


      next_energy = calculate_energy();
      //std::cout << "next energy: " << next_energy << std::endl;
      if (isnan(next_energy))
      {
        std::cout << "energy nan" << std::endl;
        exit(0);
      }
    } while (next_energy > curr_energy + wolfe_condition_angles*step_in_grad_dir_twist && step_in_grad_dir_twist > 1e-10);


    next_energy = calculate_energy();



    if (next_energy + energy_error_for_convergence > curr_energy)
      break;

  }
  */

  //_saved_last_theta_change = _thread_pieces[_thread_pieces.size()-2].angle_twist() - _saved_last_theta;
  //save_angle_twists(angles_after);
  //save_angle_twist_changes(angles_before, angles_after);


#endif

}

void Thread::minimize_energy_vertices()
{
  double step_in_grad_dir_vertices = 1.0;
  double grad_alpha = 0.4;
  double grad_beta = 0.8;

  const int num_opt_iters = 1000;
  const double energy_error_for_convergence = 1e-6;

  vector<Vector3d> vertex_gradients;
  vertex_gradients.resize(_thread_pieces.size());

  for (int opt_iter = 0; opt_iter < num_opt_iters; opt_iter++)
  {
    //std::cout << "opt iter vert: " << opt_iter << std::endl;
    calculate_gradient_vertices(vertex_gradients);

    //for backtracing line search:
    double wolfe_condition_vertices = 0.0;
    for (int piece_ind = 2; piece_ind < _thread_pieces.size()-2; piece_ind++)
    {
      wolfe_condition_vertices += vertex_gradients[piece_ind].squaredNorm();
    //  std::cout << "piece " << piece_ind << " has gradient " << vertex_gradients[piece_ind].transpose() << std::endl;
    }

    double curr_energy = calculate_energy();
    double next_energy = 0.0;
    step_in_grad_dir_vertices *= 2.0/grad_beta;
    wolfe_condition_vertices *= -grad_alpha;
    apply_vertex_offsets(vertex_gradients, true, -(2.0-grad_beta)*(step_in_grad_dir_vertices*grad_beta));
    do {
      step_in_grad_dir_vertices *= grad_beta;
      apply_vertex_offsets(vertex_gradients, true, (1.0-grad_beta)*step_in_grad_dir_vertices);


      next_energy = calculate_energy();
      if (isnan(next_energy))
      {
        std::cout << "energy nan" << std::endl;
        exit(0);
      }
    } while (next_energy > curr_energy + wolfe_condition_vertices*step_in_grad_dir_vertices && step_in_grad_dir_vertices > 1e-10);

    project_length_constraint();
    next_energy = calculate_energy();
 /*

    double curr_energy = calculate_energy();
    std::cout << "curr energy: " << curr_energy << std::endl;
    step_in_grad_dir_vertices = pow(0.95, opt_iter);
    apply_vertex_offsets(vertex_gradients, true, step_in_grad_dir_vertices);
    project_length_constraint();
    minimize_energy_twist_angles();
*/
  }

  project_length_constraint();

}




void Thread::calculate_gradient(vector<Vector3d>& vertex_gradients, vector<double>& angle_twist_gradients)
{
  calculate_gradient_vertices(vertex_gradients);
  calculate_gradient_twist(angle_twist_gradients);
}

void Thread::calculate_gradient_vertices_vectorized(VectorXd* gradient) {
  Vector3d tmp;
  for (int piece_ind = 2; piece_ind < _thread_pieces.size()-2; piece_ind++)
  {
    _thread_pieces[piece_ind].gradient_vertex(tmp);
    gradient->segment<3>(3*piece_ind) = tmp;
  }
}


void Thread::calculate_gradient_vertices(vector<Vector3d>& vertex_gradients)
{
  //#pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
  for (int piece_ind = 2; piece_ind < _thread_pieces.size()-2; piece_ind++)
  {
    //_thread_pieces[piece_ind].gradient_vertex_numeric(vertex_gradients[piece_ind]);
    //std::cout << "piece ind " << piece_ind << " old grad: " << vertex_gradients[piece_ind].transpose() << std::endl;
    _thread_pieces[piece_ind].gradient_vertex(vertex_gradients[piece_ind]);
    //std::cout << "piece ind " << piece_ind << " new grad: " << vertex_gradients[piece_ind].transpose() << std::endl;

  }
}

void Thread::make_max_norm_one(vector<Vector3d>& to_normalize)
{
  double max_grad_norm = (*(std::max_element(to_normalize.begin()+2, to_normalize.end()-2, compare_vector_norms))).norm();

//  #pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
  for (int pieceInd = 2; pieceInd < _thread_pieces.size()-2; pieceInd++)
  {
    to_normalize[pieceInd] /= max_grad_norm;
  }

}

void Thread::add_momentum_to_gradient(vector<Vector3d>& vertex_gradients, vector<Vector3d>& new_gradients, double last_step_size)
{
  double max_grad_norm = (*(std::max_element(new_gradients.begin()+2, new_gradients.end()-2, compare_vector_norms))).norm();
  double mult_const = 1.0/max_grad_norm;
  double last_vertex_mult_const = MOMENTUM_CONSTANT*last_step_size/MAX_MOVEMENT_VERTICES;

  //#pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
  for (int piece_ind = 2; piece_ind < new_gradients.size()-2; piece_ind++)
  {
    vertex_gradients[piece_ind] = new_gradients[piece_ind]*mult_const + last_vertex_mult_const*vertex_gradients[piece_ind];
  }

}


void Thread::calculate_gradient_twist(vector<double>& angle_twist_gradients)
{
  //#pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
  for (int piece_ind = 1; piece_ind < _thread_pieces.size()-2; piece_ind++)
  {
    _thread_pieces[piece_ind].gradient_twist(angle_twist_gradients[piece_ind]);
  }

}

void Thread::project_length_constraint_old()
{
  const int num_iters_project = 5000;
  const double projection_scale_factor = 0.75;
  const double max_norm_to_break = 1e-5;


  vector<Vector3d> vertex_offsets(_thread_pieces.size());
  for (int i=0; i < _thread_pieces.size(); i++)
  {
    vertex_offsets[i].setZero();
  }


  int iter_num;
  for (iter_num=0; iter_num < num_iters_project; iter_num++)
  {
    //first and last 2 pieces are special cases - assumed length is correct
    //so 2nd edge and 2nd to last edge are treated as special cases
    Vector3d curr_edge = _thread_pieces[2].vertex() - _thread_pieces[1].vertex();
    double edge_norm = curr_edge.norm();
    bool projected_enough = abs(edge_norm-_rest_length) < max_norm_to_break;
    vertex_offsets[2] = (curr_edge/edge_norm)*((_rest_length-edge_norm));
    for (int edge_ind = 2; edge_ind < _thread_pieces.size()-3; edge_ind++)
    {
      curr_edge = _thread_pieces[edge_ind+1].vertex() - _thread_pieces[edge_ind].vertex();
      edge_norm = curr_edge.norm();
      curr_edge = curr_edge/edge_norm;
      //this already set by previous iteration, so just add this offset
      vertex_offsets[edge_ind] += (curr_edge)*((edge_norm - _rest_length)/2.0);
      //initialize this vertex
      vertex_offsets[edge_ind+1] = (curr_edge)*((_rest_length - edge_norm)/2.0);
      projected_enough &= (abs(edge_norm-_rest_length) < max_norm_to_break);
    }

    curr_edge = _thread_pieces[_thread_pieces.size()-2].vertex() - _thread_pieces[_thread_pieces.size()-3].vertex();
    edge_norm = curr_edge.norm();
    vertex_offsets[_thread_pieces.size()-3] += (curr_edge/edge_norm)*((edge_norm - _rest_length));
    projected_enough &= (abs(edge_norm-_rest_length) < max_norm_to_break);
    /*

    Vector3d edge_before = _thread_pieces[2].vertex() - _thread_pieces[1].vertex();
    double edge_before_norm = edge_before.norm();
    Vector3d edge_after = _thread_pieces[2].vertex() - _thread_pieces[3].vertex();
    double edge_after_norm = edge_after.norm();
    vertex_offsets[2] = (edge_before/edge_before_norm) * (_rest_length-edge_before_norm) + 0.5*(edge_after/edge_after_norm) * (_rest_length-edge_after_norm);

    bool projected_enough = abs(edge_before_norm-_rest_length) < max_norm_to_break;

    edge_before = _thread_pieces[_thread_pieces.size()-3].vertex() - _thread_pieces[_thread_pieces.size()-4].vertex();
    edge_before_norm = edge_before.norm();
    edge_after = _thread_pieces[_thread_pieces.size()-3].vertex() - _thread_pieces[_thread_pieces.size()-2].vertex();
    edge_after_norm = edge_after.norm();
    vertex_offsets[_thread_pieces.size()-3] = 0.5*(edge_before/edge_before_norm)*(_rest_length-edge_before_norm) + (edge_after/edge_after_norm) * (_rest_length-edge_after_norm);

    //projected_enough &= abs(edge_after_norm-_rest_length) < max_norm_to_break;

    #pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
    for (int edge_ind = 3; edge_ind < _thread_pieces.size()-4; edge_ind++)
    {
      Vector3d edge_before_curr = _thread_pieces[edge_ind].vertex() - _thread_pieces[edge_ind-1].vertex();
      double edge_before_norm_curr = edge_before_curr.norm();
      Vector3d edge_after_curr = _thread_pieces[edge_ind].vertex() - _thread_pieces[edge_ind+1].vertex();
      double edge_after_norm_curr = edge_after_curr.norm();
      vertex_offsets[edge_ind] = 0.5* ((edge_before_curr/edge_before_norm_curr) * (_rest_length-edge_before_norm_curr) + (edge_after_curr/edge_after_norm_curr) * (_rest_length-edge_after_norm_curr));

      if (abs(edge_before_norm_curr-_rest_length) > max_norm_to_break)
        projected_enough = false;
    }

*/
 /*     if (vertex_offsets[_thread_pieces.size()-3].norm() > 3)
      {
        std::cout << "iter " << iter_num << "piece " << (_thread_pieces.size()-3) << " projection offset big " << vertex_offsets[_thread_pieces.size()-3]<< std::endl;
      }
      */
    //apply the actual offsets
    apply_vertex_offsets(vertex_offsets, true, projection_scale_factor, iter_num == (num_iters_project-1) || projected_enough);

    if (projected_enough)
      break;

  }


  //std::cout << "num projs: " << iter_num << std::endl;

}

void Thread::project_length_constraint()
{
  const int num_iters_project = 50;
  const double projection_scale_factor = 1.0;
  const double max_norm_to_break = 1e-5;

  // set up the augmented lagrangian system: need C, grad C, mass mat.
  // y is the coordinates of all vertices
  // dy is what we are solving for
  // optimize dy ' * dy + C lambda
  // what is h?
  // \nabla C(x) nabla C(x)^T lambda = C,
  // dy = nabla C(x_j) lambda
  // dy = -nabla C(x_j)^T (nabla C(x) nabla C(x)^T)^-1 C(x)

  int N = _thread_pieces.size();
  VectorXd C(N-3);
  MatrixXd gradC(N-3, 3*(N-4));


  vector<Vector3d> vertex_offsets(_thread_pieces.size());


  int iter_num;

  Vector3d next_point;
  Vector3d cur_point;
  double normerror;
  VectorXd dy(3*(N-4));

  for (iter_num=0; iter_num < num_iters_project; iter_num++)
  {
    C.setZero();
    gradC.setZero();
    dy.setZero();

    for(int i = 0; i < N-3; i++) {
      next_point = _thread_pieces[i+2].vertex();
      cur_point = _thread_pieces[i+1].vertex();

      C[i] = (next_point - cur_point).squaredNorm() - _rest_length*_rest_length;
    }

    gradC.block<1,3>(0,0)         = (_thread_pieces[2].vertex() - _thread_pieces[1].vertex())*2;
    for(int i = 1; i < N-4; i++) {
      gradC.block<1,3>(i,3*(i-1)) = (_thread_pieces[i+2].vertex() - _thread_pieces[i+1].vertex())*-2;
      gradC.block<1,3>(i,3*i)     = (_thread_pieces[i+2].vertex() - _thread_pieces[i+1].vertex())*2;
    }
    gradC.block<1,3>(N-4,3*(N-5)) = (_thread_pieces[N-2].vertex() - _thread_pieces[N-3].vertex())*-2;

    // slower lu decomposition
    // VectorXd dl(N-5);
    // (gradC*gradC.transpose()).lu().solve(C,&dl);
    // VectorXd dy = -gradC.transpose()*dl;

    (gradC*gradC.transpose()).llt().solveInPlace(C);
    dy = -gradC.transpose()*C;


    for(int i = 2; i < N-2; i++) {
      for(int j = 0; j < 3; j++) {
        vertex_offsets[i](j) = dy(3*(i-2)+j);
      }
    }


    bool projected_enough = true;
    normerror = 0.0;
    for(int i = 0; i < N-5; i++) {
      projected_enough &= (abs(C[i]) < max_norm_to_break);
      normerror += abs(C[i]);
    }

    //apply the actual offsets
    apply_vertex_offsets(vertex_offsets, true, projection_scale_factor, iter_num == (num_iters_project-1) || projected_enough);

    if (projected_enough)
      break;

  }
  // cout << iter_num << " " << normerror << endl;

}

void Thread::project_length_constraint_slow()
{
  const int num_iters_project = 900000;
  const double projection_scale_factor = 0.33;
  const double max_norm_to_break = 1e-8;


  vector<Vector3d> vertex_offsets(_thread_pieces.size());
  for (int i=0; i < _thread_pieces.size(); i++)
  {
    vertex_offsets[i] = Vector3d::Zero();
  }


  int iter_num;
  for (iter_num=0; iter_num < num_iters_project; iter_num++)
  {
    //first and last 2 pieces are special cases - assumed length is correct
    //so 2nd edge and 2nd to last edge are treated as special cases
    Vector3d curr_edge = _thread_pieces[2].vertex() - _thread_pieces[1].vertex();
    double edge_norm = curr_edge.norm();
    bool projected_enough = abs(edge_norm-_rest_length) < max_norm_to_break;
    vertex_offsets[2] = (curr_edge/edge_norm)*((_rest_length-edge_norm));
    for (int edge_ind = 2; edge_ind < _thread_pieces.size()-3; edge_ind++)
    {
      curr_edge = _thread_pieces[edge_ind+1].vertex() - _thread_pieces[edge_ind].vertex();
      edge_norm = curr_edge.norm();
      curr_edge = curr_edge/edge_norm;
      //this already set by previous iteration, so just add this offset
      vertex_offsets[edge_ind] += (curr_edge)*((edge_norm - _rest_length)/2.0);
      //initialize this vertex
      vertex_offsets[edge_ind+1] = (curr_edge)*((_rest_length - edge_norm)/2.0);
      projected_enough &= (abs(edge_norm-_rest_length) < max_norm_to_break);
    }

    curr_edge = _thread_pieces[_thread_pieces.size()-2].vertex() - _thread_pieces[_thread_pieces.size()-3].vertex();
    edge_norm = curr_edge.norm();
    vertex_offsets[_thread_pieces.size()-3] += (curr_edge/edge_norm)*((edge_norm - _rest_length));
    projected_enough &= (abs(edge_norm-_rest_length) < max_norm_to_break);
    /*

    Vector3d edge_before = _thread_pieces[2].vertex() - _thread_pieces[1].vertex();
    double edge_before_norm = edge_before.norm();
    Vector3d edge_after = _thread_pieces[2].vertex() - _thread_pieces[3].vertex();
    double edge_after_norm = edge_after.norm();
    vertex_offsets[2] = (edge_before/edge_before_norm) * (_rest_length-edge_before_norm) + 0.5*(edge_after/edge_after_norm) * (_rest_length-edge_after_norm);

    bool projected_enough = abs(edge_before_norm-_rest_length) < max_norm_to_break;

    edge_before = _thread_pieces[_thread_pieces.size()-3].vertex() - _thread_pieces[_thread_pieces.size()-4].vertex();
    edge_before_norm = edge_before.norm();
    edge_after = _thread_pieces[_thread_pieces.size()-3].vertex() - _thread_pieces[_thread_pieces.size()-2].vertex();
    edge_after_norm = edge_after.norm();
    vertex_offsets[_thread_pieces.size()-3] = 0.5*(edge_before/edge_before_norm)*(_rest_length-edge_before_norm) + (edge_after/edge_after_norm) * (_rest_length-edge_after_norm);

    //projected_enough &= abs(edge_after_norm-_rest_length) < max_norm_to_break;

    #pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
    for (int edge_ind = 3; edge_ind < _thread_pieces.size()-4; edge_ind++)
    {
      Vector3d edge_before_curr = _thread_pieces[edge_ind].vertex() - _thread_pieces[edge_ind-1].vertex();
      double edge_before_norm_curr = edge_before_curr.norm();
      Vector3d edge_after_curr = _thread_pieces[edge_ind].vertex() - _thread_pieces[edge_ind+1].vertex();
      double edge_after_norm_curr = edge_after_curr.norm();
      vertex_offsets[edge_ind] = 0.5* ((edge_before_curr/edge_before_norm_curr) * (_rest_length-edge_before_norm_curr) + (edge_after_curr/edge_after_norm_curr) * (_rest_length-edge_after_norm_curr));

      if (abs(edge_before_norm_curr-_rest_length) > max_norm_to_break)
        projected_enough = false;
    }

*/
 /*     if (vertex_offsets[_thread_pieces.size()-3].norm() > 3)
      {
        std::cout << "iter " << iter_num << "piece " << (_thread_pieces.size()-3) << " projection offset big " << vertex_offsets[_thread_pieces.size()-3]<< std::endl;
      }
      */
    //apply the actual offsets
    apply_vertex_offsets(vertex_offsets, true, projection_scale_factor, iter_num == (num_iters_project-1) || projected_enough);

    if (projected_enough)
      break;

  }


  //std::cout << "num projs: " << iter_num << std::endl;

}


void Thread::apply_vertex_offsets(vector<Vector3d>& offsets, bool skip_edge_cases, double step_size, bool update_frames)
{
  if (!skip_edge_cases)
  {
    exit(0);
    _thread_pieces[0].offset_vertex(step_size*offsets[0]);
    _thread_pieces[1].offset_vertex(step_size*offsets[1]);
    _thread_pieces[_thread_pieces.size()-2].offset_vertex(step_size*offsets[_thread_pieces.size()-2]);
    _thread_pieces[_thread_pieces.size()-1].offset_vertex(step_size*offsets[_thread_pieces.size()-1]);
  }



 // #pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
  for (int piece_ind = 2; piece_ind < (_thread_pieces.size()-2); piece_ind++)
  {
    _thread_pieces[piece_ind].offset_vertex(step_size*offsets[piece_ind]);
  }

  if (update_frames)
  {
    _thread_pieces.front().updateFrames_all();
    /*
   // std::cout << "last angle before: " << _thread_pieces[_thread_pieces.size()-2].angle_twist() << std::endl;
    Matrix3d end_rot = this->end_rot();
   // std::cout << "end rot before:\n" << end_rot << std::endl;
    Vector3d end_pos = _thread_pieces[_thread_pieces.size()-1].vertex();
    _thread_pieces.front().initializeFrames();
    //update_all_frames();
//    std::cout << "last angle mid: " << _thread_pieces[_thread_pieces.size()-2].angle_twist() << std::endl;
    end_rot = this->end_rot();//_thread_pieces[_thread_pieces.size()-1].material_frame();
//    std::cout << "end rot mid:\n" << end_rot << std::endl;

    set_end_constraint(end_pos, end_rot);


 //   std::cout << "last angle after: " << _thread_pieces[_thread_pieces.size()-2].angle_twist() << std::endl;
    end_rot = this->end_rot();//_thread_pieces[_thread_pieces.size()-1].material_frame();
   // std::cout << "end rot after:\n" << end_rot << std::endl;
   // */
  }


}

void Thread::apply_vertex_offsets_vectorized(const VectorXd& offsets, bool skip_edge_cases, double step_size, bool update_frames)
{
  if (!skip_edge_cases)
    {
      exit(0);
      // _thread_pieces[0].offset_vertex(step_size*offsets[0]);
      // _thread_pieces[1].offset_vertex(step_size*offsets[1]);
      // _thread_pieces[_thread_pieces.size()-2].offset_vertex(step_size*offsets[_thread_pieces.size()-2]);
      // _thread_pieces[_thread_pieces.size()-1].offset_vertex(step_size*offsets[_thread_pieces.size()-1]);
    }

  // #pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
  for (int piece_ind = 2; piece_ind < (_thread_pieces.size()-2); piece_ind++)
    {
      Vector3d offset = offsets.segment<3>(3*piece_ind);
      _thread_pieces[piece_ind].offset_vertex(step_size*offset);
    }

  if (update_frames)
    {
      _thread_pieces.front().updateFrames_all();
    }
}


void Thread::apply_angle_twist_offsets(vector<double>& offsets, bool skip_edge_cases, double step_size)
{
 if (!skip_edge_cases)
  {
    exit(0);
    _thread_pieces[_thread_pieces.size()-2].offset_angle_twist(step_size*offsets[_thread_pieces.size()-2]);
    _thread_pieces[_thread_pieces.size()-1].offset_angle_twist(step_size*offsets[_thread_pieces.size()-1]);
  }

//  #pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
 for (int piece_ind = 1; piece_ind < _thread_pieces.size()-2; piece_ind++)
  {
    _thread_pieces[piece_ind].offset_angle_twist(step_size*offsets[piece_ind]);
    _thread_pieces[piece_ind].updateFrames_twistOnly();
  }

}

/*void Thread::save_angle_twists(vector<double>& angles)
{
  #pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
  for (int piece_ind=1; piece_ind < _thread_pieces.size()-2; piece_ind++)
  {
    angles[piece_ind] = _thread_pieces[piece_ind].angle_twist() - _thread_pieces[piece_ind-1].angle_twist();
  }
}


void Thread::save_angle_twist_changes(vector<double>& start_angles, vector<double>& end_angles)
{
 // std::cout << "\nsaved angles\n";
  #pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
  for (int piece_ind=0; piece_ind < _thread_pieces.size()-2; piece_ind++)
  {
    _saved_last_theta_changes[piece_ind] = end_angles[piece_ind] - start_angles[piece_ind];
//    std::cout << _saved_last_theta_changes[piece_ind] << std::endl;
  }
//  std::cout << "\n";
}
*/

void Thread::save_thread_pieces()
{
  for (int piece_ind=0; piece_ind < _thread_pieces.size(); piece_ind++)
  {
    _thread_pieces_backup[piece_ind] = _thread_pieces[piece_ind];
  }

  for (int i=1; i < _thread_pieces_backup.size(); i++)
  {
    _thread_pieces_backup[i].set_prev(&_thread_pieces_backup[i-1]);
  }

  for (int i=0; i < _thread_pieces_backup.size()-1; i++)
  {
    _thread_pieces_backup[i].set_next(&_thread_pieces_backup[i+1]);
  }

}

void Thread::restore_thread_pieces()
{
  for (int piece_ind=0; piece_ind < _thread_pieces_backup.size(); piece_ind++)
  {
    _thread_pieces[piece_ind] = _thread_pieces_backup[piece_ind];
  }

  for (int i=1; i < _thread_pieces.size(); i++)
  {
    _thread_pieces[i].set_prev(&_thread_pieces[i-1]);
  }

  for (int i=0; i < _thread_pieces.size()-1; i++)
  {
    _thread_pieces[i].set_next(&_thread_pieces[i+1]);
  }

}


void Thread::save_angle_twists()
{
  for (int piece_ind=0; piece_ind < _thread_pieces.size(); piece_ind++)
  {
    _angle_twist_backup[piece_ind] = _thread_pieces[piece_ind].angle_twist();
  }

}

void Thread::restore_angle_twists()
{
  for (int piece_ind=0; piece_ind < _angle_twist_backup.size(); piece_ind++)
  {
    _thread_pieces[piece_ind].set_angle_twist(_angle_twist_backup[piece_ind]);
    _thread_pieces[piece_ind].update_material_frame();
  }

}



void Thread::get_thread_data(vector<Vector3d>& points, vector<double>& twist_angles)
{
  points.resize(_thread_pieces.size());
  twist_angles.resize(_thread_pieces.size());
  for (int piece_ind=0; piece_ind < _thread_pieces.size(); piece_ind++)
  {
    points[piece_ind] = _thread_pieces[piece_ind].vertex();
    twist_angles[piece_ind] = _thread_pieces[piece_ind].angle_twist();
  }
}

void Thread::get_thread_data(vector<Vector3d>& points, vector<Matrix3d>& material_frames)
{
  points.resize(_thread_pieces.size());
  material_frames.resize(_thread_pieces.size());
  for (int piece_ind=0; piece_ind < _thread_pieces.size(); piece_ind++)
  {
    points[piece_ind] = _thread_pieces[piece_ind].vertex();
    material_frames[piece_ind] = _thread_pieces[piece_ind].material_frame();
  }
}

void Thread::get_thread_data(vector<Vector3d>& points, vector<double>& twist_angles, vector<Matrix3d>& material_frames)
{
  points.resize(_thread_pieces.size());
  twist_angles.resize(_thread_pieces.size());
  material_frames.resize(_thread_pieces.size());
  for (int piece_ind=0; piece_ind < _thread_pieces.size(); piece_ind++)
  {
    points[piece_ind] = _thread_pieces[piece_ind].vertex();
    twist_angles[piece_ind] = _thread_pieces[piece_ind].angle_twist();
    material_frames[piece_ind] = _thread_pieces[piece_ind].material_frame();
  }
}

bool Thread::is_consistent()
{
  bool toRtn = true;
  for (int piece_ind=0; piece_ind < _thread_pieces.size()-1; piece_ind++)
  {
    toRtn &= _thread_pieces[piece_ind].is_material_frame_consistent();
  }

  return toRtn;
}

double Thread::calculate_holonomy()
{
/*
  vector<ThreadPiece> pieces_for_calc(4);
  pieces_for_calc[0] = ThreadPiece(_thread_pieces[0].vertex(), 0.0);
  pieces_for_calc[1] = ThreadPiece(_thread_pieces[1].vertex(), 0.0);
  //Vector3d edge_first2 = pieces_for_calc[1].vertex() - pieces_for_calc[0].vertex();
  //Vector3d edge_last2 = _thread_pieces[_thread_pieces.size()-1].vertex() - _thread_pieces[_thread_pieces.size()-2].vertex();
  pieces_for_calc[2] = ThreadPiece(pieces_for_calc[1].vertex()+edge_first2, 0.0);
  pieces_for_calc[3] = ThreadPiece(_thread_pieces[_thread_pieces.size()-1].vertex()-pieces_for_calc[2].vertex(), 0.0);
  for (int i=1; i < pieces_for_calc.size(); i++)
  {
    pieces_for_calc[i].set_prev(&pieces_for_calc[i-1]);
  }
  for (int i=0; i < pieces_for_calc.size()-1; i++)
  {
    pieces_for_calc[i].set_next(&pieces_for_calc[i+1]);
  }

  pieces_for_calc.front().set_bishop_frame(_thread_pieces.front().bishop_frame());
  pieces_for_calc.front().set_material_frame(_thread_pieces.front().bishop_frame());

  Matrix3d start_rot = pieces_for_calc.front().bishop_frame();
  Vector3d start_pos = pieces_for_calc.front().vertex();


  Matrix3d end_rot = _thread_pieces[_thread_pieces.size()-2].bishop_frame();

  pieces_for_calc.front().initializeFrames();

  //to fix tangent, set 2nd point as well
  pieces_for_calc.front().updateFrames_firstpiece();
  //pieces_for_calc.back().set_material_frame(end_rot);

  //pieces_for_calc.back().updateFrames_lastpiece();

*/
  Vector3d edge_first2 = _thread_pieces[1].vertex() - _thread_pieces.front().vertex();
  Vector3d edge_last2 = _thread_pieces[_thread_pieces.size()-1].vertex() - _thread_pieces[_thread_pieces.size()-2].vertex();

  Matrix3d no_twist_bishop;

  Vector3d to_twist_around = edge_first2.cross(edge_last2)/(_rest_length*_rest_length + edge_first2.dot(edge_last2));
  to_twist_around.normalize();
  if (isnan(to_twist_around(0)))
  {
    no_twist_bishop = _thread_pieces.front().bishop_frame();
  } else {
  //double ang_to_twist = asin(to_twist_around.norm());
  double ang_to_twist = edge_first2.dot(edge_last2)/(edge_first2.norm()*edge_last2.norm());
  ang_to_twist = max( min ( ang_to_twist, 1.0), -1.0);
  ang_to_twist = acos(ang_to_twist);


  no_twist_bishop = Eigen::AngleAxisd(ang_to_twist, to_twist_around.normalized())*_thread_pieces.front().bishop_frame();
  }

  double holonomy = angle_mismatch(_thread_pieces[_thread_pieces.size()-2].bishop_frame(), no_twist_bishop);
//  double holonomy = angle_mismatch(_thread_pieces[_thread_pieces.size()-2].bishop_frame(), pieces_for_calc[pieces_for_calc.size()-2].bishop_frame());

  return holonomy;
}

void Thread::set_constraints(const Vector3d& start_pos, const Matrix3d& start_rot, const Vector3d& end_pos, const Matrix3d& end_rot)
{
  set_start_constraint(start_pos, start_rot);
  set_end_constraint(end_pos, end_rot);
}

void Thread::set_start_constraint(const Vector3d& start_pos, const Matrix3d& start_rot)
{
  _thread_pieces.front().set_vertex(start_pos);
  //_start_rot = start_rot;
  _thread_pieces.front().set_bishop_frame(start_rot);
  _thread_pieces.front().set_material_frame(start_rot);

  //to fix tangent, set 2nd point as well
  Vector3d fixed_point = start_pos + start_rot.col(0)*_rest_length;
  _thread_pieces[1].set_vertex(fixed_point);
  _thread_pieces.front().updateFrames_firstpiece();
}


//assumes bishop frame for end is already calculated
void Thread::set_end_constraint(const Vector3d& end_pos, const Matrix3d& end_rot)
{
  _thread_pieces.back().set_vertex(end_pos);
  //_end_rot = end_rot;
  _thread_pieces.back().set_material_frame(end_rot);

  //to fix tangent, set 2nd to last point as well
  Vector3d fixed_point = end_pos - end_rot.col(0)*_rest_length;
  _thread_pieces[_thread_pieces.size()-2].set_vertex(fixed_point);
  _thread_pieces.back().updateFrames_lastpiece();
}


void Thread::set_coeffs_normalized(double bend_coeff, double twist_coeff, double grav_coeff)
{
  double norm_factor = 1.0/sqrt(pow(bend_coeff,2) + pow(twist_coeff,2) + pow(grav_coeff,2));
  _thread_pieces.front().set_bend_coeff(bend_coeff*norm_factor);
  _thread_pieces.front().set_twist_coeff(twist_coeff*norm_factor);
  _thread_pieces.front().set_grav_coeff(grav_coeff*norm_factor);
}

void Thread::set_coeffs_normalized(const Matrix2d& bend_matrix, double twist_coeff, double grav_coeff)
{
  double norm_factor = 1.0/sqrt(bend_matrix.determinant() + pow(twist_coeff,2) + pow(grav_coeff,2));
  _thread_pieces.front().set_bend_matrix(bend_matrix*norm_factor);
  _thread_pieces.front().set_twist_coeff(twist_coeff*norm_factor);
  _thread_pieces.front().set_grav_coeff(grav_coeff*norm_factor);
}

Thread& Thread::operator=(const Thread& rhs)
{
  //_thread_pieces.resize(rhs._thread_pieces.size());
  _thread_pieces.resize(rhs._thread_pieces.size());
  _thread_pieces_backup.resize(rhs._thread_pieces.size());
  _angle_twist_backup.resize(rhs._thread_pieces.size());
  for (int piece_ind =0; piece_ind < rhs._thread_pieces.size(); piece_ind++)
  {
    //_thread_pieces.push_back(rhs._thread_pieces[piece_ind]);
    _thread_pieces[piece_ind] = rhs._thread_pieces[piece_ind];
  }

  for (int i=1; i < _thread_pieces.size(); i++)
  {
    _thread_pieces[i].set_prev(&_thread_pieces[i-1]);
  }

  for (int i=0; i < _thread_pieces.size()-1; i++)
  {
    _thread_pieces[i].set_next(&_thread_pieces[i+1]);
  }

  _thread_pieces.front().set_bishop_frame(rhs.start_rot());
  _thread_pieces.front().set_material_frame(rhs.start_rot());

  _thread_pieces.front().initializeFrames();

  //_saved_last_theta_changes.resize(_thread_pieces.size());

  Matrix3d start_rot = rhs.start_rot();
  Matrix3d end_rot = rhs.end_rot();
  Vector3d start_pos = rhs.start_pos();
  Vector3d end_pos = rhs.end_pos();

  set_constraints(start_pos, start_rot, end_pos, end_rot);
  //project_length_constraint();

/*
  Matrix3d end_bishop = rhs._thread_pieces[_thread_pieces.size()-2].bishop_frame();
  double end_angle = rhs._thread_pieces[_thread_pieces.size()-2].angle_twist();
  Matrix3d end_rot_calculated = Eigen::AngleAxisd(end_angle, end_bishop.col(0).normalized())*end_bishop;
  set_end_constraint(end_pos, end_rot_calculated);
 */

  //set_end_constraint(vertices.back(), end_rot);


  return *this;

}

