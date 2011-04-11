#ifndef _thread_vision_utils_h
#define _thread_vision_utils_h

#include <string.h>
#include <map>
#include <stack>
#include <queue>
#include <vector>
#include <Eigen/Geometry>
#include "../DiscreteRods/threadutils_discrete.h"
#include "../vision/ThreeCam.h"
#include <time.h>

#define init_timing_fence time_t startTimeXX; \
    int timeElapsedXX;
#define start_timing_fence startTimeXX = time(NULL);
#define end_timing_fence(a) timeElapsedXX = difftime(time(NULL), startTimeXX); \
    cout << "Time Elapsed for " << a << ": " << timeElapsedXX << endl;

USING_PART_OF_NAMESPACE_EIGEN

class Thread_Hypoth;

struct tangent_and_score
{
    Vector3d tan;
    double score;

    tangent_and_score(const Vector3d& tanIn, const double scoreIn) :
    tan(tanIn), score(scoreIn) {}

    tangent_and_score(const tangent_and_score& in) :
    tan(in.tan), score(in.score){}

    tangent_and_score(){}
};

struct thread_hypoth_pair
{
    Thread_Hypoth *thread1;
    Thread_Hypoth *thread2;
};

typedef enum
{
    MatchingNone,
    MatchingStartStart,
    MatchingStartEnd,
    MatchingEndStart,
    MatchingEndEnd
} MatchingEnds;

bool operator <(const tangent_and_score& a, const tangent_and_score& b);


void suppress_tangents(vector<tangent_and_score>& tangents, vector<tangent_and_score>& tangents_to_keep);
void suppress_tangents(vector<tangent_and_score>& tangents, vector<int>& inds_to_keep);
bool isEqualUnordered(thread_hypoth_pair pair1, thread_hypoth_pair pair2);
MatchingEnds matchingEndsForThreads(Thread_Hypoth* thread1, Thread_Hypoth* thread2, double distanceThreshold);

#endif
