#include "threadhypoth_vision_discrete.h"

using namespace cv;

Thread_Hypoth::Thread_Hypoth(Thread_Vision* thread_vision)
    :Thread(), _thread_vision(thread_vision)
{
}

Thread_Hypoth::Thread_Hypoth(const Thread_Hypoth& rhs)
    :_thread_vision(rhs._thread_vision)
{
    _thread_pieces.resize(rhs._thread_pieces.size());
    _thread_pieces_backup.resize(rhs._thread_pieces_backup.size());
    _angle_twist_backup.resize(rhs._thread_pieces.size());

    for (int piece_ind =0; piece_ind < rhs._thread_pieces.size(); piece_ind++)
    {
    //_thread_pieces.push_back(rhs._thread_pieces[piece_ind]);
        _thread_pieces[piece_ind] = new ThreadPiece_Vision(*((ThreadPiece_Vision*)rhs._thread_pieces[piece_ind]));
    }

    for (int i=1; i < _thread_pieces.size(); i++)
    {
        _thread_pieces[i]->set_prev(_thread_pieces[i-1]);
    }

    for (int i=0; i < _thread_pieces.size()-1; i++)
    {
        _thread_pieces[i]->set_next(_thread_pieces[i+1]);
    }


    //setup backups
    for (int piece_ind=0; piece_ind < rhs._thread_pieces_backup.size(); piece_ind++)
    {
        _thread_pieces_backup[piece_ind] = new ThreadPiece_Vision(*((ThreadPiece_Vision*)rhs._thread_pieces_backup[piece_ind]));
    //_thread_pieces.push_back(ThreadPiece(vertices[i], twist_angles[i]));
    }

    for (int i=1; i < _thread_pieces_backup.size(); i++)
    {
        _thread_pieces_backup[i]->set_prev(_thread_pieces_backup[i-1]);
    }

    for (int i=0; i < _thread_pieces_backup.size()-1; i++)
    {
        _thread_pieces_backup[i]->set_next(_thread_pieces_backup[i+1]);
    }



    _thread_pieces.front()->set_bishop_frame(rhs.start_rot());
    _thread_pieces.front()->set_material_frame(rhs.start_rot());

    _thread_pieces.front()->initializeFrames();

  //project_length_constraint();
}

Thread_Hypoth::~Thread_Hypoth()
{
}


void Thread_Hypoth::optimize_visual()
{
  //std::cout << "optimizing visual" << std::endl;;
    double step_in_grad_dir_vertices = 1.0;

    const int num_opt_iters = 1000;
    const double energy_error_for_convergence = 1e-5;

    vector<Vector3d> vertex_gradients(_thread_pieces.size());
    for (int piece_ind=0; piece_ind < vertex_gradients.size(); piece_ind++)
    {
        vertex_gradients[piece_ind].setZero();
    }

    int opt_iter;

    double max_movement_vertices = MAX_MOVEMENT_VERTICES_VISION;


    project_length_constraint();

    bool recalc_vertex_grad = true;

    double curr_energy;
    double next_energy;

    for (opt_iter = 0; opt_iter < num_opt_iters; opt_iter++)
    {
        curr_energy = calculate_visual_energy();
        next_energy = 0.0;

        save_thread_pieces();
        if (recalc_vertex_grad)
        {
            calculate_visual_gradient_vertices(vertex_gradients);
            make_max_norm_one_allPieces(vertex_gradients);
            curr_energy = calculate_visual_energy();
        }

        step_in_grad_dir_vertices = max_movement_vertices;
        apply_vertex_offsets(vertex_gradients, false, -step_in_grad_dir_vertices);      
#ifndef ISOTROPIC
        minimize_energy_twist_angles();
#endif
        while (step_in_grad_dir_vertices > MIN_MOVEMENT_VERTICES_VISION)
        {
            next_energy = calculate_visual_energy();
            if (next_energy < curr_energy)
                break;

      //energy didn't improve, take a step in pos direction
            step_in_grad_dir_vertices /= 2.0;
            apply_vertex_offsets(vertex_gradients, false, step_in_grad_dir_vertices);
        }


        minimize_energy_twist_angles();
        double energy_before_projection = calculate_visual_energy();
        project_length_constraint();
        minimize_energy_twist_angles();


        next_energy = calculate_visual_energy();


//   std::cout << "curr energy: " << curr_energy << "   next energy: " << next_energy << "  before projection: " << energy_before_projection << "  last step: " << step_in_grad_dir_vertices <<   std::endl;



        recalc_vertex_grad = true;
        if (next_energy + energy_error_for_convergence > curr_energy)
        {
            if (next_energy >= curr_energy) {
                restore_thread_pieces();
                recalc_vertex_grad = false;
            } 

            if (max_movement_vertices <= MIN_MOVEMENT_VERTICES_VISION)
            {
                break;
            }
            max_movement_vertices = step_in_grad_dir_vertices / 2.0;
        } else {
            max_movement_vertices = min(step_in_grad_dir_vertices*2.0, MAX_MOVEMENT_VERTICES_VISION);
      //max_movement_vertices = MAX_MOVEMENT_VERTICES_VISION;
        }
    }

    curr_energy = calculate_visual_energy();

    project_length_constraint();
    minimize_energy_twist_angles();

    next_energy = calculate_visual_energy();
    double energy_no_vis = calculate_energy();


}




double Thread_Hypoth::calculate_visual_energy()
{
  /*double energy = Thread::calculate_energy();
  for (int piece_ind = 0; piece_ind < _thread_pieces.size(); piece_ind++)
  {
  energy += ((ThreadPiece_Vision*)_thread_pieces[piece_ind])->energy_dist();
  }
  return energy;*/

      double energy = 0.0;
  for (int piece_ind = 0; piece_ind < _thread_pieces.size(); piece_ind++)
  {
      energy += _thread_pieces[piece_ind]->energy_curvature() + _thread_pieces[piece_ind]->energy_grav();
      energy += ((ThreadPiece_Vision*)_thread_pieces[piece_ind])->energy_dist();
  }
  return energy;

}

double Thread_Hypoth::calculate_visual_energy_only()
{
    double energy = 0.0;
  //std::cout << "energy from thread: " << energy << std::endl;
    for (int piece_ind = 0; piece_ind < _thread_pieces.size(); piece_ind++)
    {
        energy += ((ThreadPiece_Vision*)_thread_pieces[piece_ind])->energy_dist();
    }
  //std::cout << "visual energy: " << energy << std::endl;
    return energy;
}

void Thread_Hypoth::calculate_score()
{
    _score = calculate_visual_energy();
}


void Thread_Hypoth::calculate_visual_gradient_vertices(vector<Vector3d>& vertex_gradients)
{
  //#pragma omp parallel for num_threads(NUM_THREADS_PARALLEL_FOR)
    for (int piece_ind = 2; piece_ind < _thread_pieces.size(); piece_ind++)
    {
        ((ThreadPiece_Vision*)_thread_pieces[piece_ind])->gradient_vertex_vis(vertex_gradients[piece_ind]);
    }
}






void Thread_Hypoth::add_first_threadpieces(corresponding_pts& start_pt, tangent_and_score& start_tan)
{
  //need to assign start rotation. Arbitrary for now
    Vector3d perp_col = Vector3d::UnitY();
    make_vectors_perpendicular(start_tan.tan, perp_col);
    Matrix3d start_rot;
    start_rot.col(0) = start_tan.tan;
    start_rot.col(1) = perp_col;
    start_rot.col(2) = (start_rot.col(0).cross(start_rot.col(1))).normalized();

    _thread_pieces.resize(2);

    Vector3d to_set_vertex;
    OpencvToEigen(start_pt.pt3d, to_set_vertex);
    _thread_pieces[0] = new ThreadPiece_Vision(to_set_vertex, 0.0, NULL, NULL, _thread_vision);
    _thread_pieces[1] = new ThreadPiece_Vision(to_set_vertex+_rest_length*start_tan.tan, 0.0, (ThreadPiece_Vision*)_thread_pieces[0], NULL, _thread_vision);
    _thread_pieces[0]->set_next(_thread_pieces[1]);

    _thread_pieces.front()->set_bishop_frame(start_rot);
    _thread_pieces.front()->initializeFrames();


    _thread_pieces_backup.resize(2);
    _thread_pieces_backup[0] = new ThreadPiece_Vision(*(ThreadPiece_Vision*)_thread_pieces[0]);
    _thread_pieces_backup[1] = new ThreadPiece_Vision(*(ThreadPiece_Vision*)_thread_pieces[1]);
    _thread_pieces_backup[0]->set_next(_thread_pieces_backup[1]);
    _thread_pieces_backup[1]->set_prev(_thread_pieces_backup[0]);
}

void Thread_Hypoth::project_length_constraint()
{
    if (_thread_pieces.size() < 10)
    {
        project_length_constraint_old();
    } else {
        Thread::project_length_constraint();
    }
}





void Thread_Hypoth::add_possible_next_hypoths(vector<Thread_Hypoth*>& extra_next_hypoths)
{
  /*
  vector<Scalar> debugColors(6);
  debugColors[0] = Scalar(0,0,200);
  debugColors[1] = Scalar(0,200,0);
  debugColors[2] = Scalar(200,0,0);
  debugColors[3] = Scalar(0,200,200);
  debugColors[4] = Scalar(200,200,0);
  debugColors[5] = Scalar(200,0,200);
  */

  Vector3d new_vertex_init;
  new_vertex_init = _thread_pieces[_thread_pieces.size()-2]->edge() + _thread_pieces.back()->vertex();
      //This code adds next hypoth by following last curvature 
      /*
      int ind = _thread_pieces.size()-2;
      Vector3d toRotAround = _thread_pieces[ind]->curvature_binormal().normalized();

//this is last material frame rotated with curvature
if (isnan(toRotAround(0)))
{
new_vertex_init = _thread_pieces[ind]->edge() + _thread_pieces.back()->vertex();
} else {
double for_ang = _thread_pieces[ind-1]->edge().dot(_thread_pieces[ind]->edge()) / (_thread_pieces[ind-1]->edge_norm()*_thread_pieces[ind]->edge_norm());
for_ang = max( min ( for_ang, 1.0), -1.0);
double ang_to_rot = acos(for_ang);

Matrix3d rotation;
rotation = Eigen::AngleAxisd(ang_to_rot, toRotAround);
new_vertex_init= (rotation*_thread_pieces[ind]->edge()) + _thread_pieces.back()->vertex();

} 
*/



double new_angle = (_thread_pieces.size() <= 2 ? 0 : _thread_pieces[_thread_pieces.size()-2]->angle_twist() + _thread_pieces[_thread_pieces.size()-2]->angle_twist()/((double)_thread_pieces.size()-2) );
_thread_pieces.push_back(new ThreadPiece_Vision(new_vertex_init, new_angle, (ThreadPiece_Vision*)_thread_pieces.back(), NULL, _thread_vision));
_thread_pieces[_thread_pieces.size()-2]->set_next((ThreadPiece_Vision*)_thread_pieces.back());
_thread_pieces[_thread_pieces.size()-2]->set_angle_twist(new_angle);

  //this could be sped up - only need to update last piece
_thread_pieces.front()->initializeFrames();


  //alter location of tangent based on visual information (and possibly dot product)
vector<tangent_and_score> tan_and_scores;
if (!find_next_tan_visual(tan_and_scores))
    std::cout << "error! no tans" << std::endl;

_thread_pieces.back()->set_vertex(tan_and_scores.front().tan*_rest_length + _thread_pieces[_thread_pieces.size()-2]->vertex());
_thread_pieces.front()->initializeFrames();
calculate_score();


_thread_pieces_backup.push_back(new ThreadPiece_Vision(*(ThreadPiece_Vision*)_thread_pieces.back()) );
_thread_pieces_backup[_thread_pieces_backup.size()-2]->set_next((ThreadPiece_Vision*)_thread_pieces_backup.back());
_thread_pieces_backup.back()->set_prev((ThreadPiece_Vision*)_thread_pieces_backup[_thread_pieces_backup.size()-2]);


  //add the new hypoths
extra_next_hypoths.resize(tan_and_scores.size()-1);
for (int tan_ind =1; tan_ind < tan_and_scores.size(); tan_ind++)
{
    Thread_Hypoth* to_add_extra = new Thread_Hypoth(*this);
    to_add_extra->_thread_pieces.back()->set_vertex(tan_and_scores[tan_ind].tan*_rest_length + _thread_pieces[_thread_pieces.size()-2]->vertex());
    to_add_extra->_thread_pieces.front()->initializeFrames(); 
    to_add_extra->calculate_score();


    extra_next_hypoths[tan_ind-1] = to_add_extra;
}


}


bool Thread_Hypoth::find_next_tan_visual(vector<tangent_and_score>& tangents)
{
    const double length_for_tan = _rest_length;
    const double ang_to_rotate = M_PI/60.0;
  //const int num_reprojs_per_tan = 10;

  //rotate as in yaw, pitch, roll - no roll, since we only care about direction of tangent
    vector<tangent_and_score> tan_scores;
    Vector3d axis1 = this->end_rot().col(2).normalized();
    double angle_min = DBL_MAX;
    double angle_max = DBL_MIN;
    for (double rot1_ang = -M_PI/2.0; rot1_ang <= M_PI/2.0; rot1_ang+=(ang_to_rotate))
    {
        Matrix3d rot1(Eigen::AngleAxisd(rot1_ang, axis1));
        Vector3d axis2 = rot1*(this->end_rot().col(1));
        axis2.normalize();
        for (double rot2_ang = -M_PI/2.0; rot2_ang <= M_PI/2.0; rot2_ang +=(ang_to_rotate))
        {
            Matrix3d currRotation = Eigen::AngleAxisd(rot2_ang, axis2)*rot1;
            Vector3d currTangent = Eigen::AngleAxisd(rot2_ang, axis2)*rot1*(this->end_rot().col(0).normalized());

            currTangent.normalize();

            double currScore = TAN_SCORE_VISUAL_COEFF*_thread_vision->scoreProjection3dPointAndTanget(_thread_pieces[_thread_pieces.size()-2]->vertex(), currTangent*_rest_length);

            currScore += TAN_SCORE_DOT_PROD_COEFF*(currTangent.dot(this->end_rot().col(0).normalized()));

      //std::cout << "currScore: " << currScore << "  threadvision: " << _thread_pieces[_thread_pieces.size()-2]->vertex().transpose() << std::endl;
            if (currScore < TANGENT_ERROR_THRESH)
            {
                tangent_and_score toAdd(currTangent, currScore);
                tan_scores.push_back(toAdd);
            }

        }
    }

    if (tan_scores.size() <= 0)
        return false;



    suppress_tangents(tan_scores, tangents);

    return true;


}




void Thread_Hypoth::save_thread_pieces_and_resize(vector<ThreadPiece*>& to_save)
{
    while (to_save.size() < _thread_pieces.size())
    {
        to_save.push_back(new ThreadPiece_Vision(*(ThreadPiece_Vision*)(_thread_pieces[to_save.size()])));
    }

    for (int i = to_save.size()-1; i >= (int)_thread_pieces.size(); i--)
    {
        delete to_save[i];
        to_save.pop_back();
    }

    save_thread_pieces(to_save);

}

void Thread_Hypoth::restore_thread_pieces_and_resize(vector<ThreadPiece*>& to_restore)
{
    while (_thread_pieces.size() < to_restore.size())
    {
        _thread_pieces.push_back(new ThreadPiece_Vision(*(ThreadPiece_Vision*)(to_restore[_thread_pieces.size()])) );
        _thread_pieces_backup.push_back(new ThreadPiece_Vision(*(ThreadPiece_Vision*)(to_restore[_thread_pieces_backup.size()])));
    }

    for (int i = _thread_pieces.size()-1; i >= (int)to_restore.size(); i--)
    {
        delete _thread_pieces[i];
        delete _thread_pieces_backup[i];
        _thread_pieces.pop_back();
        _thread_pieces_backup.pop_back();
    }

    set_prev_next_pointers(_thread_pieces);
    set_prev_next_pointers(_thread_pieces_backup);

    restore_thread_pieces(to_restore);

}


void suppress_hypoths(vector<Thread_Hypoth*>& hypoths)
{
    vector<int> inds_to_keep;
    suppress_hypoths(hypoths, inds_to_keep);

    int num_hypoths_to_keep = min((int)(inds_to_keep.size()), NUM_HYPOTHS_MAX);
    Thread_Hypoth* hypoths_to_keep[num_hypoths_to_keep];
    for (int i=0; i < num_hypoths_to_keep; i++)
    {
        hypoths_to_keep[i] = hypoths[inds_to_keep[i]];
        hypoths[inds_to_keep[i]] = NULL;
    }

  //delete the hypoths we didn't keep
    for (int i=0; i < hypoths.size(); i++)
    {
        if (hypoths[i] != NULL)
            delete hypoths[i];
    }

    hypoths.resize(num_hypoths_to_keep);
    for (int i=0; i < num_hypoths_to_keep; i++)
    {
        hypoths[i] = hypoths_to_keep[i];
    }
}

void suppress_hypoths(vector<Thread_Hypoth*>& hypoths, vector<int>& inds_to_keep)
{
  //const double dot_prod_thresh = 0.1;
    const double dot_prod_thresh = 0.2;
    const double position_norm_thresh = 2.0;
    const double total_score_thresh = 4.0;
    inds_to_keep.resize(0);

    sort(hypoths.begin(), hypoths.end(), lessthan_Thread_Hypoth);
    int ind_checking;
    for (ind_checking = 0; ind_checking < hypoths.size(); ind_checking++)
    {
        if (inds_to_keep.size() > 0 && hypoths[ind_checking]->score() > hypoths.front()->score() * total_score_thresh)
            break;

        bool keep_this_ind = true;
        for (int ind_comparing = 0; ind_comparing < inds_to_keep.size(); ind_comparing++)
        {
            double dot_prod = hypoths[ind_checking]->end_edge().dot(hypoths[inds_to_keep[ind_comparing]]->end_edge());
            double pos_err = (hypoths[ind_checking]->end_pos() - hypoths[inds_to_keep[ind_comparing]]->end_pos()).norm();
            if (dot_prod > dot_prod_thresh && pos_err < position_norm_thresh)
            {
                keep_this_ind = false;
                break;
            }
        }
        if (keep_this_ind)
            inds_to_keep.push_back(ind_checking);
    }

}



bool operator <(const Thread_Hypoth& a, const Thread_Hypoth& b)
{
    return a.score() < b.score();
}

bool lessthan_Thread_Hypoth(const Thread_Hypoth* a, const Thread_Hypoth* b)
{
    return a->score() < b->score();
}


