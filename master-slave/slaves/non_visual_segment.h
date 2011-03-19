#ifndef _NON_VISUAL_SEGMENT_H_
#define _NON_VISUAL_SEGMENT_H_

#include "motion_planning_segment.h"

typedef enum {RELATIVE, ABSOLUTE} non_visual_type;

class Non_visual_segment : public Motion_planning_segment{
   
public:

    double _goal_pos[num_dof];
    
    Non_visual_segment(double duration, double* relative_goal_pos, Motion_planning_segment* prev_segment, non_visual_type relative_or_abs = RELATIVE);
    ~Non_visual_segment();
    
    non_visual_type _my_type;

    /* Our goal has already been defined. We simply return that value. */
    bool get_goal(double* goal_pos);

};




#endif
