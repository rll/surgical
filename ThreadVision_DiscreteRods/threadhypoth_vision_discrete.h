#ifndef _threadhypoth_vision_discrete_h
#define _threadhypoth_vision_discrete_h

#include "threadpiece_vision_discrete.h"
#include "../DiscreteRods/trajectory_recorder.h"
#include "thread_vision_discrete.h"
#include "thread_vision_utils.h"
//#include "../DiscreteRods/thread_discrete.h"


#define MAX_MOVEMENT_VERTICES_VISION 1.0
#define MIN_MOVEMENT_VERTICES_VISION 1e-5

#define TAN_SCORE_VISUAL_COEFF 1.0
#define TAN_SCORE_DOT_PROD_COEFF 0.0

#define NUM_HYPOTHS_MAX 10

class Thread_Vision;

class Thread_Hypoth: public Thread
{
public:
    Thread_Hypoth(Thread_Vision* thread_vision);
    Thread_Hypoth(const Thread_Hypoth& rhs);
    ~Thread_Hypoth();

    void add_first_threadpieces(corresponding_pts& start_pt, tangent_and_score& start_tan);
    void add_first_threadpieces(corresponding_pts& start_pt, tangent_and_score& start_tan, double startTwist);
    void optimize_visual();

    /* Add next piece based on visual reprojection only. If this hypoth splits,
     * new hypoths are added to extra_next_hypoths */
    void add_possible_next_hypoths(vector<Thread_Hypoth*>& extra_next_hypoths);

    /* Calculates possible new pieces (edge+vertex) to add at the end of hypothesis */
    bool find_next_tan_visual(vector<tangent_and_score>& tangents);

    /* Methods for mutating the thread pieces */
    void reverseThreadPieces();
    void appendThread(Thread* aThread);

    /* Helper functions to save and restore state of thread pieces */
    void restore_thread_pieces_and_resize(vector<ThreadPiece*>& to_restore);
    void save_thread_pieces_and_resize(vector<ThreadPiece*>& to_save);

    double calculate_visual_energy();
    double calculate_visual_energy_only();
    void calculate_score();
    void calculate_visual_gradient_vertices(vector<Vector3d>& vertex_gradients);
    void project_length_constraint();

    const double score(void) const
    {
        return _score;
    };

    void initializeFrames()
    {
        _thread_pieces.front()->initializeFrames();
    };


    double _score; //this is not ensured to be updated!! 
    double _previous_energy;
    Thread_Vision* _thread_vision;
};

void suppress_hypoths(vector<Thread_Hypoth*>& hypoths);
void suppress_hypoths(vector<Thread_Hypoth*>& hypoths,
        vector<int>& inds_to_keep);
bool operator <(const Thread_Hypoth& a, const Thread_Hypoth& b);
bool lessthan_Thread_Hypoth(const Thread_Hypoth* a, const Thread_Hypoth* b);
bool lessThanThreadHypothVisualEnergy(Thread_Hypoth *a, Thread_Hypoth *b);

#endif
