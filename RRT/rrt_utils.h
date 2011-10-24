#ifndef _rrt_utils_h
#define _rrt_utils_h

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include <vector>

#include "../DiscreteRods/threadutils_discrete.h"
#include "../DiscreteRods/ThreadConstrained.h"

USING_PART_OF_NAMESPACE_EIGEN

void uniformlyRandomTranslation(Vector3d& translation, double max_dist);
void uniformlyRandomRotation(Quaterniond& rotation);
void uniformlyRandomRotation(Matrix3d& rotation);

/**
 * Takes an already existing thread, and modifies its state (i.e. start
 * position and material frames) to a random one. Material frames are chosen 
 * uniformly at random such that adjacent material frames don't differ by too 
 * much (i.e. adjayent edges are not too bent). The thread's start position 
 * is chosen such that the midpoint between the start and end of the thread 
 * is at a random position within the cube of side 2*translation_offset_limit.
 */
void generateRandomThread(ThreadConstrained* thread, double translation_offset_limit = 40.0);

#endif //_rrt_utils_h
