#include "thread_vision_utils.h"
#include "threadhypoth_vision_discrete.h"

void suppress_tangents(vector<tangent_and_score>& tangents, vector<tangent_and_score>& tangents_to_keep)
{
    vector<int> inds_to_keep;
    suppress_tangents(tangents, inds_to_keep);

    tangents_to_keep.resize(0);
    for (int i=0; i < inds_to_keep.size(); i++)
    {
        tangents_to_keep.push_back(tangents[inds_to_keep[i]]);
    }
}

void suppress_tangents(vector<tangent_and_score>& tangents, vector<int>& inds_to_keep)
{
    const double dot_prod_thresh = M_PI / 4;
    const double position_norm_thresh = 2.0;
    const double total_score_thresh = 4.0;
    inds_to_keep.resize(0);

    sort(tangents.begin(), tangents.end());
    int ind_checking;
    for (ind_checking = 0; ind_checking < tangents.size(); ind_checking++)
    {
        if (inds_to_keep.size() > 5 && tangents[ind_checking].score > tangents.front().score * total_score_thresh)
            break;

        bool keep_this_ind = true;
        for (int ind_comparing = 0; ind_comparing < inds_to_keep.size(); ind_comparing++)
        {
            double dot_prod = tangents[ind_checking].tan.dot(tangents[inds_to_keep[ind_comparing]].tan);
            double cosAngleBetween = dot_prod / (tangents[ind_checking].tan.norm() * tangents[inds_to_keep[ind_comparing]].tan.norm());
            double angleBetween = acos(cosAngleBetween);

            /* Assume that the angle between the two vectors can't be too big */
            if (angleBetween < dot_prod_thresh)
            {
                keep_this_ind = false;
            }
        }
        if (keep_this_ind)
            inds_to_keep.push_back(ind_checking);
    }

}


bool operator <(const tangent_and_score& a, const tangent_and_score& b)
{
    return a.score < b.score;
}

bool isEqualUnordered(thread_hypoth_pair pair1, thread_hypoth_pair pair2)
{
    return (pair1.thread1 == pair2.thread1 && pair1.thread2 == pair2.thread2) || (pair1.thread1 == pair2.thread2 && pair1.thread2 == pair2.thread1);
};

MatchingEnds matchingEndsForThreads(Thread_Hypoth* thread1, Thread_Hypoth* thread2, double distanceThreshold)
{
    if (distance_between_points(thread1->start_pos(), thread2->start_pos()) < distanceThreshold)
    {
        return MatchingStartStart;
    }
    else if (distance_between_points(thread1->start_pos(), thread2->end_pos()) < distanceThreshold)
    {
        return MatchingStartEnd;
    }
    else if (distance_between_points(thread1->end_pos(), thread2->start_pos()) < distanceThreshold)
    {
        return MatchingEndStart;
    }
    else if (distance_between_points(thread1->end_pos(), thread2->end_pos()) < distanceThreshold)
    {
        return MatchingEndEnd;
    }

    return MatchingNone;
}


void adjacentPoints(Point2i &aPoint, vector<Point2i> &adjacentPoints, int maxX, int maxY)
{
    adjacentPoints.clear();
    for (int xadd = -1; xadd <= 1; xadd++)
    {
        int x_next = aPoint.x + xadd;
        if (x_next >= 0 && x_next < maxX)
        {
            for (int yadd=-1; yadd <= 1; yadd++)
            {
                int y_next = aPoint.y + yadd;
                if (y_next >= 0 && y_next < maxY)
                {
                    if (!(xadd == 0 && yadd == 0))
                    {
                        adjacentPoints.push_back(Point2i(x_next, y_next));
                    }
                }
            }
        }
    }
}

double distance(Point2f aPoint, Vector2d aVector, Point2i startPoint)
{
    Vector2d v = aVector;
    Vector2d w(aPoint.x - startPoint.x, aPoint.y - startPoint.y);

    double c1 = v.dot(w);
    double c2 = v.dot(v);
    
    if (c1 <= 0){
        Point2d tmp;
        tmp.x = startPoint.x;
        tmp.y = startPoint.y;
        return distance(aPoint, tmp);
    }
    if (c2 <= 0){
        Point2d endPoint;
        endPoint.x = startPoint.x + (float) aVector[0];
        endPoint.y = startPoint.y + (float) aVector[1];

        return distance(aPoint, endPoint);
    }

    double b = c1 / c2;

    Point2d pb(startPoint.x + b * aVector[0], startPoint.y + b * aVector[1]);

    return distance(aPoint, pb);
}

double distance(Point2f a, Point2f b)
{
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}
