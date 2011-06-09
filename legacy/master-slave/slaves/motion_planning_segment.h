#ifndef _MOTION_PLANNING_SEGMENT_H_
#define _MOTION_PLANNING_SEGMENT_H_

#include "util.h"
#include <stdlib.h>

/* A motion planning segment is a "part" of a whole multi-planner. It can be defined in terms of absolute start and end positions, or
relative ones, and has knowledge of its preceding segment (or NULL if there was none) */
class Motion_planning_segment{
    public:
        Motion_planning_segment* _prev_segment;
        double  _duration;
        double  _absolute_start_pos[num_dof];
        bool    _has_absolute_start_pos;
        
        Motion_planning_segment(double duration, Motion_planning_segment* prev_segment=NULL);
        ~Motion_planning_segment();
        
        /* For the case of the very first segment: this will set an absolute start position, which overrides any possible backwards checking. If this is set, no matter what
         *,it will be considered the start position. */
        void set_absolute_start_pos(double* absolute_start_pos);
        /* Populates start_pos with the position which I am starting at. Returns false if it is unable to do so due to constraints. */
        bool get_start(double* start_pos);
        void get_last_target(double* last_target);
       
        /* Sets the grip positions to what the start position as */
        void set_grips_to_start(double* pos_to_alter);


        /* Populates goal_pos with the position which I end at, including all offsets */
        virtual bool get_goal(double* goal_pos)=0;
        /*  Uses interpolation to calculate, on the fly, where I should be aiming for at a given time. For visual, this will act as a "polling" time as well, to check
            whether it's time for a visual servo. For non visual, this is simply an interpolation between start and end, with no offsets required. Returns false if
            there is not enough information to compute this. */
        virtual bool get_position_at_time(double* position, double time);
    



};

#endif
