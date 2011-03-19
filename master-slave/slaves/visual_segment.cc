#include "visual_segment.h"

Visual_segment:: Visual_segment(double duration, std::vector<double>servo_times, Visual_feedback* vision, Motion_planning_segment* prev_segment, double* goal_offset_input)
                    :Motion_planning_segment(duration, prev_segment){

    _vision = vision;
    _requested_goal_pos = false;
    _received_goal_pos = false;
    _requested_current_pos = false;
    _received_current_pos = false;
    _servo_times = servo_times;
    _servo_index = 0;
    _have_start = false;


    for (int i=0; i < num_dof; i++)
    {
        _goal_offset_from_servo[i] = 0.0;
    }

    if (goal_offset_input == NULL)
    {
        for (int i=0; i < num_dof; i++)
        {
            _goal_offset_from_input[i] = 0.0;
        }
    } else
    {
        for (int i=0; i < num_dof; i++)
        {
            _goal_offset_from_input[i] = goal_offset_input[i];
        }
    }


}

Visual_segment:: ~Visual_segment(){
    //Nothing to delete
}

bool Visual_segment:: get_goal(double* goal_pos){
    if(!_requested_goal_pos){
        _vision->request_goal_position();
        _requested_goal_pos = true;
    }
    else if(!_received_goal_pos) {
        if(_vision->get_goal_position(_goal_pos)){
            _received_goal_pos = true;
            Motion_planning_segment::set_grips_to_start(_goal_pos);
            for (int i = 0; i < num_dof; i++)
            {
                _goal_pos[i] += _goal_offset_from_input[i];
            }
            copy_pos(_goal_pos, goal_pos);
            return true;
        }
        else {
            return false;            
        }
    }
    else{
        copy_pos(_goal_pos, goal_pos);
        return true;
    }
}



/*bool Visual_segment::get_start(double* start_pos)
{
    if (!_have_start)
    {
        if (Motion_planning_segment::get_start(_start_pos))
        {
            copy_pos(_start_pos, start_pos);
            _have_start = true;
            return true;
        } else
        {
            return false;    
        }
    }
   
    printf("new start pos");
    copy_pos(_start_pos, start_pos);
    return true;
}*/


bool Visual_segment:: get_position_at_time(double* position, double time){
    if (time_to_servo(time)){
        _requested_current_pos = false;
        request_servo();
        return false;
    }
    else if (currently_servoing()){
        return false;
    }
    else
        return Motion_planning_segment::get_position_at_time(position, time);
}


//incorporate the offsets we may have, either from wanting to not hit the target, or from servoing
/*bool Visual_segment::get_position_at_time_with_offsets(double* position, double time)
{
    if (!Motion_planning_segment::get_position_at_time(position, time))
    {
        return false;
    }

    double fraction_input = time/_duration;

    //time of last servo is equivalent to _servo_index-1
    double fraction_servo = 0;
    if (_servo_index > 0){
        fraction_servo = (time - _servo_times.at(_servo_index-1)) / (_duration - _servo_times.at(_servo_index-1));
    }

    printf("Fraction input: %f      Fraction servo: %f \n", fraction_input, fraction_servo);

    printf("Offsets: ");
    for (int i=0; i < num_dof; i++)
    {
        position[i] += fraction_input*_goal_offset_from_input[i] + fraction_servo*_goal_offset_from_servo[i];
        printf("%f  ", (fraction_input*_goal_offset_from_input[i] + fraction_servo*_goal_offset_from_servo[i]));
    }   
    printf("\n");

    return true;

}
*/

void Visual_segment::set_new_start_and_goal()
{
    if(!(get_start(_start_pos) && get_goal(_goal_pos)) )
        return;

    if (_servo_index == 0)
        return;
    
    double time_adjust = _servo_times.at(_servo_index-1);
    double start_fraction = time_adjust/(_duration - time_adjust);
    for (int i=0; i < num_dof; i++)
    {
        _goal_pos[i] += _goal_offset_from_servo[i];
        _absolute_start_pos[i] -= start_fraction*_goal_offset_from_servo[i];
    }

}



bool Visual_segment:: time_to_servo(double time){
    if (!_requested_current_pos && _servo_index < _servo_times.size() && _servo_times.at(_servo_index) <= time) {
        return true;
    }
    else {
        return false;
    }
}

void Visual_segment:: request_servo(){
    if (!_requested_current_pos)
    {
        _vision->request_current_position();
        _requested_current_pos = true;
        _received_current_pos = false;
    }
    
}

bool Visual_segment:: currently_servoing(){
    double pos_from_request[num_dof];
    if (_requested_current_pos && !_received_current_pos)
    {
        if (!_vision->get_current_position(pos_from_request))
        {
            return true;
        }   
        else
        {
            //now that we successfully servod, incremenet the servo index
            _servo_index++;
            _received_current_pos = true;
            _requested_current_pos = false;
            double pos_currently_planning_to[num_dof];
            if (!Motion_planning_segment::get_position_at_time(pos_currently_planning_to, _servo_times.at(_servo_index-1)))
            {
                printf("servoing fail...\n");
                exit(0);
                return false;
            }
            
           /* for (int i=0; i < num_dof; i++)
            {
                _goal_offset_from_servo[i] += pos_currently_planning_to[i] - pos_from_request[i];
            }*/

            _goal_offset_from_servo[iX] = (pos_currently_planning_to[iX]- _goal_offset_from_servo[iX]) - pos_from_request[iX];
            _goal_offset_from_servo[iY] = (pos_currently_planning_to[iY]- _goal_offset_from_servo[iY]) - pos_from_request[iY];
            _goal_offset_from_servo[iZ] = (pos_currently_planning_to[iZ]- _goal_offset_from_servo[iZ]) - pos_from_request[iZ];
           

            printf("Calculated offsets of   %f     %f    %f \n", _goal_offset_from_servo[iX], _goal_offset_from_servo[iY], _goal_offset_from_servo[iZ]);

            set_new_start_and_goal();

            return false;
        }

    } else {
        return false;
    }

}

