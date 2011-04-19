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
#include <sys/time.h>

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

void adjacentPoints(Point2i &aPoint, vector<Point2i> &adjacentPoints, int maxX, int maxY);

class Timer
{
  public:
    Timer() {
      gettimeofday(&start_tv, NULL);
    }
    void restart() {
      gettimeofday(&start_tv, NULL);
    }
    double elapsed() {
      gettimeofday(&tv, NULL);
      return  (tv.tv_sec - start_tv.tv_sec) +
        (tv.tv_usec - start_tv.tv_usec) / 1000000.0;
    }

  private:
    struct timeval tv;
    struct timeval start_tv;

};
#endif
