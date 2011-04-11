#include "thread_vision_discrete.h"
#define _USE_MATH_DEFINES

Thread_Vision::Thread_Vision()
{
    _names[0] = "cam1";
    _names[1] = "cam2";
    _names[2] = "cam3";

#ifdef FAKEIMS
    _captures[0] = new Capture(0, //id
        _names[0].c_str(), // cam name
        650,  // gain
        "optimized", // optimized or measured
        107109, // camera uid
        (string(SAVED_IMAGE_BASE) + "1-").c_str());
    //IM_DIR_1); 
    _captures[0]->setExposure(5000);
    //_captures[0]->setExposure(11000);


    _captures[1] = new Capture(1, //id
        _names[1].c_str(),// cam name
        650,  // gain
        "optimized", // optimized or measured
        107110, // camera uid
        (string(SAVED_IMAGE_BASE) + "2-").c_str());
    //        IM_DIR_2); 
    _captures[1]->setExposure(3000);
    //_captures[1]->setExposure(7500);

    _captures[2] = new Capture(2, //id
        _names[2].c_str(),// cam name
        650,  // gain
        "optimized", // optimized or measured
        107111, // camera uid
        (string(SAVED_IMAGE_BASE) + "3-").c_str());
    //          IM_DIR_3); 
    _captures[2]->setExposure(3500);

#elif defined NYLON
    _captures[0] = new Capture(0, //id
        _names[0].c_str(), // cam name
        650,  // gain
        "optimized", // optimized or measured
        107109, // camera uid
        "../vision/captures/suturenylon1-");
    //IM_DIR_1); 
    _captures[0]->setExposure(5000);
    //_captures[0]->setExposure(11000);


    _captures[1] = new Capture(1, //id
        _names[1].c_str(),// cam name
        650,  // gain
        "optimized", // optimized or measured
        107110, // camera uid
        "../vision/captures/suturenylon2-");
    //        IM_DIR_2); 
    _captures[1]->setExposure(3000);
    //_captures[1]->setExposure(7500);

    _captures[2] = new Capture(2, //id
        _names[2].c_str(),// cam name
        650,  // gain
        "optimized", // optimized or measured
        107111, // camera uid
        "../vision/captures/suturenylon3-");
    //          IM_DIR_3); 
    _captures[2]->setExposure(3500);

#elif defined PURPLE
    _captures[0] = new Capture(0, //id
        _names[0].c_str(), // cam name
        650,  // gain
        "optimized", // optimized or measured
        107109, // camera uid
        "/home/pabbeel/rll/code/trunk/surgical/vision/captures/suturepurple1-");
    //IM_DIR_1); 
    _captures[0]->setExposure(5000);
    //_captures[0]->setExposure(11000);


    _captures[1] = new Capture(1, //id
        _names[1].c_str(),// cam name
        650,  // gain
        "optimized", // optimized or measured
        107110, // camera uid
        "/home/pabbeel/rll/code/trunk/surgical/vision/captures/suturepurple2-");
    //        IM_DIR_2); 
    _captures[1]->setExposure(3000);
    //_captures[1]->setExposure(7500);

    _captures[2] = new Capture(2, //id
        _names[2].c_str(),// cam name
        650,  // gain
        "optimized", // optimized or measured
        107111, // camera uid
        "/home/pabbeel/rll/code/trunk/surgical/vision/captures/suturepurple3-");
    //          IM_DIR_3); 
    _captures[2]->setExposure(3500);

#else
    _captures[0] = new Capture(0, //id
        _names[0].c_str(), // cam name
        650,  // gain
        "optimized", // optimized or measured
        107109, // camera uid
        "/home/pabbeel/rll/code/trunk/surgical/vision/captures/sutureblack1-");
    //IM_DIR_1); 
    _captures[0]->setExposure(5000);
    //_captures[0]->setExposure(11000);


    _captures[1] = new Capture(1, //id
        _names[1].c_str(),// cam name
        650,  // gain
        "optimized", // optimized or measured
        107110, // camera uid
        "/home/pabbeel/rll/code/trunk/surgical/vision/captures/sutureblack2-");
    //        IM_DIR_2); 
    _captures[1]->setExposure(3000);
    //_captures[1]->setExposure(7500);

    _captures[2] = new Capture(2, //id
        _names[2].c_str(),// cam name
        650,  // gain
        "optimized", // optimized or measured
        107111, // camera uid
        "/home/pabbeel/rll/code/trunk/surgical/vision/captures/sutureblack3-");
    //          IM_DIR_3); 
    _captures[2]->setExposure(3500);


#endif

    char names_char[NUMCAMS][256];
    char canny_char[NUMCAMS][256];
    for (int camNum=0; camNum < NUMCAMS; camNum++)
    {
        sprintf(names_char[camNum], "%s%d", DISPLAY_ORIG_BASE, camNum);
        _orig_display_names[camNum].assign(names_char[camNum]);
        namedWindow(_orig_display_names[camNum], CV_WINDOW_AUTOSIZE);
        sprintf(canny_char[camNum], "%s%d", DISPLAY_CANNY_BASE, camNum);
        _canny_display_names[camNum].assign(canny_char[camNum]);
        namedWindow(_canny_display_names[camNum], CV_WINDOW_AUTOSIZE);
    }



    _captures[0]->init("./calib_params/");
    _captures[1]->init("./calib_params/");
    _captures[2]->init("./calib_params/");
    /*
    _captures[0]->init("./calib_params/2010_9_15/");
    _captures[1]->init("./calib_params/2010_9_15/");
    _captures[2]->init("./calib_params/2010_9_15/");
    */

    cvWaitKey(1000);							// segfaulted without this


    //add information about other cameras for stereo
    _captures[0]->AddOtherCameraInformation(*_captures[1]);
    _captures[0]->AddOtherCameraInformation(*_captures[2]);
    _captures[1]->AddOtherCameraInformation(*_captures[0]);
    _captures[1]->AddOtherCameraInformation(*_captures[2]);
    _captures[2]->AddOtherCameraInformation(*_captures[0]);
    _captures[2]->AddOtherCameraInformation(*_captures[1]);


    //initialize threecam wrapper
    //thread
    _cams = new ThreeCam(_captures);
    /*
    float width[] = {1.50, 1.50, 1.50};
    float edge_sigma[] = {0.50, 0.50, 0.50};
    float blur_sigma[] = {1.5, 1.5, 1.5};
    double thresh1[] = {4.0, 4.0, 4.0};
    double thresh2[] = {80.0, 80.0, 80.0};
    */
#ifdef PURPLE
    float width[] = {1.50, 1.50, 1.50};
    float edge_sigma[] = {0.50, 0.50, 0.50};
    float blur_sigma[] = {1.5, 1.5, 1.5};
    double thresh1[] = {2.0, 2.0, 2.0};
    double thresh2[] = {8.0, 8.0, 8.0};
#else

    float width[] = {1.50, 1.50, 1.50};
    float edge_sigma[] = {0.50, 0.50, 0.50};
    float blur_sigma[] = {1.5, 1.5, 1.5};
    double thresh1[] = {2.0, 2.0, 2.0};
    double thresh2[] = {8.0, 8.0, 8.0};
#endif

    _cams->initializeCanny(width, edge_sigma, blur_sigma, thresh1, thresh2);

    //initialize distance image
    for (int i=0; i < NUMCAMS; i++)
    {
        _frames = _cams->frames();
        rows[i] = _frames[i].rows;
        cols[i] = _frames[i].cols;
        _cannyIms_display[i] = Mat(_frames[i].size(), CV_8UC3);
    }


    _max_length_thread = 0.0;
    reproj_points_fix_canny = false;


}

Thread_Vision::Thread_Vision(char* im_base)
{
    _names[0] = "cam1";
    _names[1] = "cam2";
    _names[2] = "cam3";

    char im_names_total[400];
    sprintf(im_names_total, "%s%d-", im_base, 1);
    _captures[0] = new Capture(0, //id
        _names[0].c_str(), // cam name
        650,  // gain
        "optimized", // optimized or measured
        107109, // camera uid
        im_names_total);
    //IM_DIR_1); 
    _captures[0]->setExposure(5000);
    //_captures[0]->setExposure(11000);


    sprintf(im_names_total, "%s%d-", im_base, 2);
    _captures[1] = new Capture(1, //id
        _names[1].c_str(),// cam name
        650,  // gain
        "optimized", // optimized or measured
        107110, // camera uid
        im_names_total);
    //        IM_DIR_2); 
    _captures[1]->setExposure(3000);
    //_captures[1]->setExposure(7500);


    sprintf(im_names_total, "%s%d-", im_base, 3);
    _captures[2] = new Capture(2, //id
        _names[2].c_str(),// cam name
        650,  // gain
        "optimized", // optimized or measured
        107111, // camera uid
        im_names_total);
    //          IM_DIR_3); 
    _captures[2]->setExposure(3500);




    //_captures[2]->setExposure(8000);

    /*namedWindow(_names[0], CV_WINDOW_AUTOSIZE);
    namedWindow(_names[1], CV_WINDOW_AUTOSIZE);
    namedWindow(_names[2], CV_WINDOW_AUTOSIZE);
    */

    char names_char[NUMCAMS][256];
    char canny_char[NUMCAMS][256];
    for (int camNum=0; camNum < NUMCAMS; camNum++)
    {
        sprintf(names_char[camNum], "%s%d", DISPLAY_ORIG_BASE, camNum);
        _orig_display_names[camNum].assign(names_char[camNum]);
        namedWindow(_orig_display_names[camNum], CV_WINDOW_AUTOSIZE);
        sprintf(canny_char[camNum], "%s%d", DISPLAY_CANNY_BASE, camNum);
        _canny_display_names[camNum].assign(canny_char[camNum]);
        namedWindow(_canny_display_names[camNum], CV_WINDOW_AUTOSIZE);
    }


/*
_captures[0]->init("./calib_params/");
_captures[1]->init("./calib_params/");
_captures[2]->init("./calib_params/");
*/
_captures[0]->init("./calib_params/2010_9_15/");
_captures[1]->init("./calib_params/2010_9_15/");
_captures[2]->init("./calib_params/2010_9_15/");

cvWaitKey(1000);							// segfaulted without this


    //add information about other cameras for stereo
_captures[0]->AddOtherCameraInformation(*_captures[1]);
_captures[0]->AddOtherCameraInformation(*_captures[2]);
_captures[1]->AddOtherCameraInformation(*_captures[0]);
_captures[1]->AddOtherCameraInformation(*_captures[2]);
_captures[2]->AddOtherCameraInformation(*_captures[0]);
_captures[2]->AddOtherCameraInformation(*_captures[1]);


    //initialize threecam wrapper
    //thread
_cams = new ThreeCam(_captures);
float width[] = {1.50, 1.50, 1.50};
float edge_sigma[] = {0.50, 0.50, 0.50};
float blur_sigma[] = {1.5, 1.5, 1.5};
double thresh1[] = {4.0, 4.0, 4.0};
double thresh2[] = {80.0, 80.0, 80.0};

_cams->initializeCanny(width, edge_sigma, blur_sigma, thresh1, thresh2);

    //initialize distance image
for (int i=0; i < NUMCAMS; i++)
{
    _frames = _cams->frames();
    rows[i] = _frames[i].rows;
    cols[i] = _frames[i].cols;
    _cannyIms_display[i] = Mat(_frames[i].size(), CV_8UC3);
}


_max_length_thread = 0.0;
reproj_points_fix_canny = false;


}



Thread_Vision::~Thread_Vision()
{
    delete _cams;
    for (int i=0; i < NUMCAMS; i++){
        _captures[i]->endCapture();
        delete _captures[i];
    }

    for (int i = 0; i < _thread_hypoths.size(); i++){
        vector<Thread_Hypoth*> current_thread_hypoths = _thread_hypoths[i];
        for (int hypoth_ind=0; hypoth_ind < current_thread_hypoths.size(); hypoth_ind++)
        {
            delete current_thread_hypoths[hypoth_ind];
        }
        _thread_hypoths.clear();
    }
}



void Thread_Vision::initThreadSearch()
{
    updateCanny();

    findStartPoints();

    /* Depends on findStartPoints() */
    best_thread_hypoths = &_thread_hypoths[0];

    stepNumber = 1;
    cout << "Thread Search Init Finished" << endl;

    for (int i = 0; i < 5; i++) {
        generateNextSetOfHypoths();
    }

    cout << "First 5 steps finished" << endl;

    hasInit = true;
}

bool Thread_Vision::findStartPoints()
{
    cout << "Finding start points" << endl;
    for (int i = 0; i < _start_data.size(); i++){
        start_data initial_pt_and_tan = _start_data[i];
        vector<corresponding_pts> start_pts;
        vector<tangent_and_score> start_tangents;

        if (findNextStartPoint(start_pts, initial_pt_and_tan.pt))
        {
            std::cout << "Found start" << std::endl;
            if (findTangent(start_pts[0], initial_pt_and_tan.tangent, start_tangents))
            {
                std::cout << "Found tan" << std::endl;

                _thread_hypoths.resize(_thread_hypoths.size() + 1);

                vector<Thread_Hypoth*>& current_thread_hypoths = _thread_hypoths.back();
                current_thread_hypoths.resize(start_tangents.size());

                for (int hypoth_ind = 0; hypoth_ind < start_tangents.size(); hypoth_ind++)
                {
                    /* Generate possible twists from -2pi/maxlength to 2pi/maxlength */
                    int twistIncrements = 10;
                    for (int j = 0; j < twistIncrements; j++) {
                        int numThreadPieces = _max_length_thread / _rest_length;
                        double baseTwist = 2.0 * M_PI / _rest_length;
                        double startTwist = -1 * baseTwist + j * (baseTwist * 2 / twistIncrements);
                        current_thread_hypoths[hypoth_ind] = new Thread_Hypoth(this);
                        current_thread_hypoths[hypoth_ind]->add_first_threadpieces(start_pts[0], start_tangents[hypoth_ind], startTwist);
                    }
                }
            }
        }
    }
}


void Thread_Vision::get_thread_data(vector<Vector3d>& points, vector<double>& twist_angles)
{
    best_thread_hypoths->at(curr_hypoth_ind)->get_thread_data(points, twist_angles);
}

void Thread_Vision::get_thread_data(vector<Vector3d>& points, vector<Matrix3d>& material_frames)
{
    best_thread_hypoths->at(curr_hypoth_ind)->get_thread_data(points, material_frames);
}

void Thread_Vision::get_thread_data(vector<Vector3d>& points, vector<double>& twist_angles, vector<Matrix3d>& material_frames)
{
    best_thread_hypoths->at(curr_hypoth_ind)->get_thread_data(points, twist_angles, material_frames);
}

bool Thread_Vision::generateNextSetOfHypoths() {
    vector<Thread_Hypoth*> &current_thread_hypoths = _thread_hypoths[0];

    if (!isDone())
    {
        add_possible_next_hypoths(current_thread_hypoths);

        suppress_hypoths(current_thread_hypoths);

        for (int hypoth_ind=0; hypoth_ind < current_thread_hypoths.size(); hypoth_ind++)
        {
            /* Run the optimization algorithm, using visual distance and thread energy */
            current_thread_hypoths[hypoth_ind]->optimize_visual();
            current_thread_hypoths[hypoth_ind]->minimize_energy_twist_angles();
            current_thread_hypoths[hypoth_ind]->calculate_score();
        }

        suppress_hypoths(current_thread_hypoths);

        cout << endl;
    }

    sort_hypoths(*best_thread_hypoths);
    curr_hypoth_ind = 0;

    cout << "Finished step number: " << stepNumber << endl;
    stepNumber++;
}

bool Thread_Vision::runThreadSearch()
{
    while(!isDone()) {
        generateNextSetOfHypoths(); 
    }
    return true;
}

bool Thread_Vision::isDone() {
    /* TODO uses the thread hypoths from the first start point */
    if (_thread_hypoths.size() == 0 || _thread_hypoths[0].front()->num_pieces() < 5) {
        return false;
    }
    return (_thread_hypoths[0].front()->num_pieces()*_rest_length >= _max_length_thread);
}

vector<thread_hypoth_pair>* Thread_Vision::nearbyPairsOfThreadHypoths()
{
    vector<thread_hypoth_pair>* allPairs = new vector<thread_hypoth_pair>();
    for (int i = 0; i < _thread_hypoths.size(); i++) {
        vector<Thread_Hypoth*> current_thread_hypoths = _thread_hypoths[i];
        for (int j = 0; j < current_thread_hypoths.size(); j++) {
            Thread_Hypoth *current_thread = current_thread_hypoths[j];
            //Now compare to every other thread

            for (int k = 0; k < _thread_hypoths.size(); k++) {
                for (int l = 0; l < _thread_hypoths[k].size(); l++) {
                    Thread_Hypoth *other_thread = _thread_hypoths[k][l];
                    if (current_thread != other_thread) {
                        //Check end points
                        bool isCollision = false;

                        MatchingEnds match = matchingEndsForThreads(current_thread, other_thread, CLOSE_DISTANCE_COEFF);

                        if (match != MatchingNone) {
                            thread_hypoth_pair pair = {current_thread, other_thread};
                            bool exists = false;
                            for (int a = 0; a < allPairs->size(); a++) {
                                if (isEqualUnordered(pair, pair)) {
                                    exists = true;
                                }
                            }
                            if (!exists) {
                                allPairs->push_back(pair);
                            }
                        }
                        //std::cout << distance_between_points(current_thread->start_pos(), current_thread->end_pos()) << "\n";
                    }
                }
            }
        }
    }
    return allPairs;
}

/* For efficiency, threads are mutated */
Thread_Hypoth* Thread_Vision::mergeThreads(Thread_Hypoth* thread1, Thread_Hypoth* thread2)
{
    MatchingEnds match = matchingEndsForThreads(thread1, thread2, CLOSE_DISTANCE_COEFF);
    if (match == MatchingStartStart) {
        thread1->reverseThreadPieces();
        thread1->appendThread(thread2);
    }
    else if (match == MatchingStartEnd) {
        thread2->appendThread(thread1);
    }
    else if (match == MatchingEndStart) {
        thread1->appendThread(thread2);
    }
    else if (match == MatchingEndEnd) {
        thread2->reverseThreadPieces();
        thread1->appendThread(thread2);
    }

    return thread1;
}

/* Adds new hypoths based on tangents. Hypoths in current_thread_hypoths will
 * be mutated, and extra ones will be appended to the back */
void Thread_Vision::add_possible_next_hypoths(vector<Thread_Hypoth*>& current_thread_hypoths)
{
    int curr_number_hypoths = current_thread_hypoths.size();
    vector<Thread_Hypoth*> extra_hypoths;
    for (int hypoth_ind=0; hypoth_ind < curr_number_hypoths; hypoth_ind++)
    {
        current_thread_hypoths[hypoth_ind]->add_possible_next_hypoths(extra_hypoths);
        for (int new_hypoth_ind=0; new_hypoth_ind < extra_hypoths.size(); new_hypoth_ind++)
        {
            current_thread_hypoths.push_back(extra_hypoths[new_hypoth_ind]);
        } 
    }
}

Thread* Thread_Vision::flip_to_hypoth(int hypoth_ind)
{
    curr_hypoth_ind = hypoth_ind;
    return best_thread_hypoths->at(hypoth_ind);
}

void Thread_Vision::next_hypoth()
{
    curr_hypoth_ind = min(curr_hypoth_ind+1, (int)best_thread_hypoths->size()-1);
}

void Thread_Vision::prev_hypoth()
{
    curr_hypoth_ind = max(curr_hypoth_ind-1, 0);
}

Thread* Thread_Vision::curr_thread()
{
    return best_thread_hypoths->at(curr_hypoth_ind);
}


void Thread_Vision::sort_hypoths(vector<Thread_Hypoth*>& current_thread_hypoths)
{
    sort(current_thread_hypoths.begin(), current_thread_hypoths.end(), lessthan_Thread_Hypoth);
}

/* Find points close to initPt, then scores and projects them onto 2d points */
bool Thread_Vision::findNextStartPoint(vector<corresponding_pts>& pts, Point3f& initPt)
{
    double minDist = -0.5;
    double maxDist = 0.5;
    double addDist = 0.1;

    for (double z = initPt.z+minDist; z <= initPt.z+maxDist; z+= addDist)
    {
        for (double y = initPt.y+minDist; y <= initPt.y+maxDist; y+= addDist)
        {
            for (double x = initPt.x+minDist; x <= initPt.x+maxDist; x+= addDist)
            {
                pts.resize(pts.size()+1);
                pts.back().pt3d.x = x;
                pts.back().pt3d.y = y;
                pts.back().pt3d.z = z;
                _cams->project3dPoint(pts.back().pt3d, pts.back().pts2d);
                pts.back().score = scoreProjection3dPoint(pts.back().pt3d);
            }
        }
    }

    int numPtsToResize = min((int)pts.size(),NUM_START_PTS_TO_INIT);
    partial_sort(pts.begin(), pts.begin()+numPtsToResize, pts.end());

    return (pts.size() > 0);

}

bool Thread_Vision::findNextStartPoint(vector<corresponding_pts>& pts, Point2i& initPtCenterIm)
{
    const int maxDist = 15;
    int dist;
    bool done = false;

    for (dist = 0; dist < maxDist 
        && _captures[CENTER_IM_IND]->inRange(initPtCenterIm.y-dist, initPtCenterIm.x-dist)
        && _captures[CENTER_IM_IND]->inRange(initPtCenterIm.y+dist, initPtCenterIm.x-dist) 
        && _captures[CENTER_IM_IND]->inRange(initPtCenterIm.y+dist, initPtCenterIm.x+dist) 
        && _captures[CENTER_IM_IND]->inRange(initPtCenterIm.y-dist, initPtCenterIm.x+dist); dist++)
    {
        vector<Point2i> points_to_check;
        PointsForSquare(initPtCenterIm, dist, points_to_check);
        for (int ptInd = 0; ptInd < points_to_check.size(); ptInd++)
        {
            if (_cannyIms[CENTER_IM_IND].at<uchar>(points_to_check[ptInd].y, points_to_check[ptInd].x) == IM_VALUE_NOT_SEEN)
            {
                _cannyIms[CENTER_IM_IND].at<uchar>(points_to_check[ptInd].y, points_to_check[ptInd].x) = IM_VALUE_CHECKED_FOR_BEGIN;
                vector<corresponding_pts> pts_this;
                if (findCorrespondingPointsOtherIms(pts_this, points_to_check[ptInd], CENTER_IM_IND))
                {
                    done = true;
                    pts.insert(pts.end(), pts_this.begin(), pts_this.end());
                }
            }
        }

        if (done){
            int numPtsToResize = min((int)pts.size(),NUM_START_PTS_TO_INIT);
            partial_sort(pts.begin(), pts.begin()+numPtsToResize, pts.end());
            break;
        }

    }

    return done;

}





//TODO: speedup by intersecting epipolar lines, and searching around that region
//TODO: speedup by not even adding point to list if below thresh 
bool Thread_Vision::findCorrespondingPointsOtherIms(vector<corresponding_pts>& pts, Point2i initPt, int camWithPt)
{
    vector<Point2i> pts_on_line[NUMCAMS];
    pts_on_line[camWithPt].push_back(initPt);
    for (int i=0; i < NUMCAMS; i++)
    {
        if (i == camWithPt)
            continue;

        //get epipolar line
        Mat lineParams = Mat(3,1,CV_32FC1);

        _captures[i]->GetEpipolarLine(*_captures[camWithPt], initPt, lineParams);
        float a = lineParams.at<float>(0,0);
        float b = lineParams.at<float>(1,0);
        float c = lineParams.at<float>(2,0);

        //searches across lines offset in this direction
        int offset_each_dir = 2;
        Point p1;
        Point p2;
        bool vertical;


        if (abs(a) > abs(b))
        {
            //line is more vertical
            vertical = true;
            p1.y = 0;
            p2.y = _cannyIms[i].rows;
            p1.x = floor(0.5 + (-c)/a);
            p2.x = floor(0.5 + (-c-b*p2.y)/a);
        } else {
            //line is more horizontal
            vertical = false;
            p1.x = 0;
            p2.x = _cannyIms[i].cols;
            p1.y = floor(0.5 + (-c)/b);
            p2.y = floor(0.5 + (-c-a*p2.x)/b);
        }

        //FOR DEBUG - draw epipolar lines
        /*for (int offset_each_dir_iter = -offset_each_dir; offset_each_dir_iter <= offset_each_dir; offset_each_dir_iter++)
        {
        display_for_debug[i].resize(display_for_debug[i].size()+1);
        display_for_debug[i][display_for_debug[i].size()-1].pts = new Point[2];
        int x_off_each_dir = (vertical ? offset_each_dir_iter : 0);
        int y_off_each_dir = (vertical ? 0 : offset_each_dir_iter);
        display_for_debug[i][display_for_debug[i].size()-1].pts[0] = Point(p1.x + x_off_each_dir, p1.y + y_off_each_dir);
        display_for_debug[i][display_for_debug[i].size()-1].pts[1] = Point(p2.x + x_off_each_dir, p2.y + y_off_each_dir);
        display_for_debug[i][display_for_debug[i].size()-1].color = Scalar(0,0,255);
        display_for_debug[i][display_for_debug[i].size()-1].size = 2;
        }*/



            int offset_each_dir_multiplier = (vertical ? sizeof(uchar) : _cannyIms[i].step);

        //iterate across this line
        cv::LineIterator it(_cannyIms[i], p1, p2, 8);
        for (int iter=0; iter < it.count; iter++, ++it)
        {

            for (int offset_each_dir_iter = -offset_each_dir; offset_each_dir_iter <= offset_each_dir; offset_each_dir_iter++)
            {
                int addr_off = offset_each_dir_multiplier*offset_each_dir_iter;



                if (*(it.ptr+addr_off) != 0)
                {
                    int offset = it.ptr+addr_off - (uchar*)(_cannyIms[i].data);
                    Point2i matchPt;
                    matchPt.y =  offset/_cannyIms[i].step;
                    matchPt.x = (offset - matchPt.y*_cannyIms[i].step)/(sizeof(uchar));

                    pts_on_line[i].push_back(matchPt);


                    //FOR DEBUG - mark points
                    /*display_for_debug[i].resize(display_for_debug[i].size()+1);
                    display_for_debug[i][display_for_debug[i].size()-1].pts = new Point[4];
                    display_for_debug[i][display_for_debug[i].size()-1].pts[0] = Point(matchPt.x-1, matchPt.y+1);
                    display_for_debug[i][display_for_debug[i].size()-1].pts[1] = Point(matchPt.x+1, matchPt.y+1);
                    display_for_debug[i][display_for_debug[i].size()-1].pts[2] = Point(matchPt.x+1, matchPt.y-1);
                    display_for_debug[i][display_for_debug[i].size()-1].pts[3] = Point(matchPt.x-1, matchPt.y-1);
                    display_for_debug[i][display_for_debug[i].size()-1].color = Scalar(0,255,0);
                    display_for_debug[i][display_for_debug[i].size()-1].size = 4;
                    */

                }
            }
        }
    }


    //Check to see which triplets work
    int numPts = 1;
    for (int i=0; i < NUMCAMS; i++)
    {
        numPts *= pts_on_line[i].size();
    }

    if (numPts == 0)
    {
        return false;
    }

    pts.resize(numPts);

    int currMultiplier = 1;
    for (int i=0; i < NUMCAMS; i++)
    {
        int currInd = 0;
        for (int numTimesToCopy = 0; numTimesToCopy < numPts/(currMultiplier*pts_on_line[i].size()); numTimesToCopy++)
        {
            for (int j=0; j < pts_on_line[i].size(); j++)
            {
                int currIndBeforeAdds = currInd;
                for (; currInd < currIndBeforeAdds+currMultiplier; currInd++)
                {
                    pts[currInd].pts2d[i] = pts_on_line[i].at(j);
                }
            }
        }

        currMultiplier *= pts_on_line[i].size();
    }



    for (int i=0; i < numPts; i++)
    {
        _cams->scoreCorrespondingPts(pts[i], false);
    }

    int numPtsToResize = min(numPts,NUM_START_PTS_TO_INIT);
    partial_sort(pts.begin(), pts.begin()+numPtsToResize, pts.end());
    pts.resize(numPtsToResize);

    //remove those with too much error
    corresponding_pts forInd;
    forInd.score = CORRESPONDING_PTS_ERROR_THRESH;
    int indToRemove = (int)(lower_bound(pts.begin(), pts.end(), forInd) - pts.begin());
    pts.resize(indToRemove);

    if (pts.size() == 0)
        return false;



    //FOR DEBUG - mark best match
    /*int minInd = 0;

for (int j = 0; j < NUMCAMS; j++)
{
display_for_debug[j].resize(display_for_debug[j].size()+1);
display_for_debug[j][display_for_debug[j].size()-1].pts = new Point[4];
display_for_debug[j][display_for_debug[j].size()-1].pts[0] = Point(pts[minInd].pts2d[j].x, pts[minInd].pts2d[j].y);
display_for_debug[j][display_for_debug[j].size()-1].pts[1] = Point(pts[minInd].pts2d[j].x, pts[minInd].pts2d[j].y);
display_for_debug[j][display_for_debug[j].size()-1].pts[2] = Point(pts[minInd].pts2d[j].x, pts[minInd].pts2d[j].y);
display_for_debug[j][display_for_debug[j].size()-1].pts[3] = Point(pts[minInd].pts2d[j].x, pts[minInd].pts2d[j].y);
display_for_debug[j][display_for_debug[j].size()-1].color = Scalar(127,0,127);
display_for_debug[j][display_for_debug[j].size()-1].size = 4;
}*/



    return true;

}


bool Thread_Vision::findTangent(corresponding_pts& start, Vector3d& init_tangent, vector<tangent_and_score>& tangents)
{
    const double length_for_tan = _rest_length;
    const double ang_to_rotate = M_PI/40.0;

    //rotate as in yaw, pitch, roll - no roll, since we only care about direction of tangent
    vector<tangent_and_score> tan_scores;
    Vector3d start_pos;
    OpencvToEigen(start.pt3d, start_pos);

    for (double rot1_ang = -M_PI/4.0; rot1_ang <= M_PI/4.0; rot1_ang+=(ang_to_rotate))
    {
        Matrix3d rot1(Eigen::AngleAxisd(rot1_ang, Vector3d::UnitZ()));
        Vector3d axis2 = rot1*Vector3d::UnitY();
        axis2.normalize();
        for (double rot2_ang = -M_PI/4.0; rot2_ang <= M_PI/4.0; rot2_ang +=(ang_to_rotate))
        {
            Matrix3d currRotation = Eigen::AngleAxisd(rot2_ang, axis2)*rot1;
            Vector3d currTangent = Eigen::AngleAxisd(rot2_ang, axis2)*rot1*(init_tangent);

            currTangent.normalize();
            /*
            tanLocation.x = ((float)(length_for_tan*currTangent(0))) + start.pt3d.x;
            tanLocation.y = ((float)(length_for_tan*currTangent(1))) + start.pt3d.y;
            tanLocation.z = ((float)(length_for_tan*currTangent(2))) + start.pt3d.z;

            double currScore = scoreProjection3dPoint(tanLocation);
            */
            double currScore = scoreProjection3dPointAndTanget(start_pos, currTangent*length_for_tan);

            if (currScore < TANGENT_ERROR_THRESH)
            {
                tan_scores.resize(tan_scores.size()+1);
                tan_scores.back().tan = currTangent;
                tan_scores.back().score = currScore;
            }
        }
    }

    if (tan_scores.size() <= 0)
        return false;



    //tangents.push_back(*min_element(tan_scores.begin(), tan_scores.end()));
    suppress_tangents(tan_scores, tangents);

    std::cout << "initial tans: " << std::endl;
    for (int i=0; i < tangents.size(); i++)
    {
        std::cout << "Tangent: (" <<  tangents[i].tan.transpose() << ")" << "\t\t" << "Score: " << tangents[i].score << std::endl;
    }

    /*
    //FOR DEBUG - mark best match AND Tan
    Vector3d tangent;
    tangent = tangents[0].tan;


    Point2i proj_tan[NUMCAMS]; 
    Point3f bestTanLocation(  (float)(length_for_tan*tangent(0)) + start.pt3d.x,
    (float)(length_for_tan*tangent(1)) + start.pt3d.y, 
    (float)(length_for_tan*tangent(2)) + start.pt3d.z);


    _cams->project3dPoint(bestTanLocation, proj_tan);


    for (int j = 0; j < NUMCAMS; j++)
    {
    display_for_debug[j].resize(display_for_debug[j].size()+1);
    display_for_debug[j][display_for_debug[j].size()-1].pts = new Point[8];
    display_for_debug[j][display_for_debug[j].size()-1].pts[0] = Point(start.pts2d[j].x-1, start.pts2d[j].y+1);
    display_for_debug[j][display_for_debug[j].size()-1].pts[1] = Point(start.pts2d[j].x+1, start.pts2d[j].y+1);
    display_for_debug[j][display_for_debug[j].size()-1].pts[2] = Point(start.pts2d[j].x+1, start.pts2d[j].y-1);
    display_for_debug[j][display_for_debug[j].size()-1].pts[3] = Point(start.pts2d[j].x-1, start.pts2d[j].y-1);
    display_for_debug[j][display_for_debug[j].size()-1].pts[4] = Point(proj_tan[j].x-1, proj_tan[j].y+1);
    display_for_debug[j][display_for_debug[j].size()-1].pts[5] = Point(proj_tan[j].x+1, proj_tan[j].y+1);
    display_for_debug[j][display_for_debug[j].size()-1].pts[6] = Point(proj_tan[j].x+1, proj_tan[j].y-1);
    display_for_debug[j][display_for_debug[j].size()-1].pts[7] = Point(proj_tan[j].x-1, proj_tan[j].y-1);
    display_for_debug[j][display_for_debug[j].size()-1].color = Scalar(255,255,0);
    display_for_debug[j][display_for_debug[j].size()-1].size = 8;
    }

    gl_display_for_debug.resize(gl_display_for_debug.size()+1);
    gl_display_for_debug.back().vertices = new Point3f[2];
    gl_display_for_debug.back().vertices[0] = start.pt3d;
    gl_display_for_debug.back().vertices[1] = bestTanLocation;
    gl_display_for_debug.back().size = 2;
    gl_display_for_debug.back().color[0] = 1.0;
    gl_display_for_debug.back().color[1] = 0.0;
    gl_display_for_debug.back().color[2] = 0.0;


    */
    return true;
}


void Thread_Vision::write_hypoths_to_file(char* filename)
{
    /*
    vector<ThreadPiece*> backups;
    save_thread_pieces_and_resize(backups);

Trajectory_Recorder traj_recorder(filename);

for (int hypoth_ind=0; hypoth_ind < _thread_pieces_hypoths.size(); hypoth_ind++)
{
restore_thread_pieces_and_resize(_thread_pieces_hypoths[hypoth_ind]);
traj_recorder.add_thread_to_list(*this);
}
traj_recorder.write_threads_to_file();

restore_thread_pieces_and_resize(backups);
*/
}






double Thread_Vision::scoreProjection3dPoint(const Point3f& pt3d, double* scores)
{
    //project 3d point to images, and see how far away the projection is (in pixels) to nearest thread pixel
    if (scores == NULL)
    {
        double scoresTemp[NUMCAMS];
        scores = scoresTemp;
    }
    double score = 0.0;
    Point2f pts2d[NUMCAMS];
    _cams->project3dPoint(pt3d, pts2d);

    for (int camNum=0; camNum < NUMCAMS; camNum++)
    {
        scores[camNum] = score2dPoint(pts2d[camNum], camNum);
        score += scores[camNum];
    }

    return score;
}

double Thread_Vision::scoreProjection3dPointAndTanget(const Vector3d& startpt3d, const Vector3d& tan, double* scores)
{
    const int num_reprojs_per_tan = (int)_rest_length * 3;

    //project 3d point to images, and see how far away the projection is (in pixels) to nearest thread pixel
    if (scores == NULL)
    {
        double scoresTemp[NUMCAMS];
        scores = scoresTemp;
    }
    double score = 0.0;
    Point2f pts2d[NUMCAMS];
    for (int camNum=0; camNum < NUMCAMS; camNum++)
    {
        scores[camNum] = 0.0;
    }


    Vector3d tanLocation;
    Point3f pt3d;

    //std::cout << "checking for points" << std::endl;
    for (int reproj_num=0; reproj_num < num_reprojs_per_tan; reproj_num++)
    {
        tanLocation = ((double)(reproj_num+1))/((double)(num_reprojs_per_tan))*tan + startpt3d;
        EigenToOpencv(tanLocation, pt3d);

        _cams->project3dPoint(pt3d, pts2d);
        for (int camNum=0; camNum < NUMCAMS; camNum++)
        {
            scores[camNum] += score2dPoint(pts2d[camNum], camNum);
        }

    }

    for (int camNum=0; camNum < NUMCAMS; camNum++)
    {
        scores[camNum] /= ((double)(num_reprojs_per_tan));
        score += scores[camNum];
    }



    return score;
}


double Thread_Vision::score2dPoint(const Point2f& pt, int camNum)
{
    double thisMinScore = DIST_FOR_SCORE_CHECK;
    Point2i rounded = Point2i(floor(pt.x + 0.5), floor(pt.y + 0.5));

    if (_captures[camNum]->inRange(rounded.y, rounded.x))
    {
        int key = keyForHashMap(camNum, rounded.y, rounded.x);
        if (_cannyDistanceScores[camNum].count(key) > 0)
        {
            location_and_distance* curr = &_cannyDistanceScores[camNum][key];
            while (curr != NULL)
            {
                thisMinScore = min(thisMinScore, sqrt((double)(pow(pt.x-(float)curr->col,2) + pow(pt.y-(float)curr->row,2))));
                curr = curr->next;
            }
        }
    } else {
        thisMinScore = SCORE_OUT_OF_VIEW;
    }

    if (thisMinScore < SCORE_THRESH_SET_TO_CONST)
        thisMinScore = SCORE_THRESH_SET_TO_CONST;

    return thisMinScore;
}


bool Thread_Vision::isEndPiece(const Point3f pt)
{
    Point2i pts2d[NUMCAMS];
    _cams->project3dPoint(pt, pts2d);

    int numContinue = NUMCAMS;

    for (int camNum=0; camNum< NUMCAMS; camNum++)
    {
        if (isEndPiece(camNum, pts2d[camNum]))
            numContinue--;
//      return true;
    }

    //return false;
    return numContinue < 2;
}


bool Thread_Vision::isEndPiece(const int camNum, const Point2i pt)
{
    //check dist_for_thread_check pixels away (in a square) for thread pixels
    //for all we find, see if there are other pixels nearby
    //if we find 1 grouping only, it is an end piece

    const int dist_for_thread_check = 8;
    const double dist_limit = sqrt(2);

    if (!_captures[camNum]->inRange(pt.y-dist_for_thread_check, pt.x-dist_for_thread_check) ||
        !_captures[camNum]->inRange(pt.y+dist_for_thread_check, pt.x-dist_for_thread_check) ||
        !_captures[camNum]->inRange(pt.y+dist_for_thread_check, pt.x+dist_for_thread_check) ||
        !_captures[camNum]->inRange(pt.y-dist_for_thread_check, pt.x+dist_for_thread_check))
        return true;

    vector<Point2i> points_with_thread;
    vector<Point2i> points_to_check;
    PointsForSquare(pt, dist_for_thread_check, points_to_check);
    for (int i=0; i < points_to_check.size(); i++)
    {
        if (_cannyIms[camNum].at<uchar>(points_to_check[i]) > IM_VALUE_USED_IN_THREAD)
        {
            if (points_with_thread.size() == 0)
            {
                points_with_thread.push_back(points_to_check[i]);
            } else {
                bool canAttach = false;
                for (int j=0; j < points_with_thread.size(); j++)
                {
                    if (norm(points_with_thread[j] - points_to_check[i]) <= dist_limit)
                    {
                        canAttach = true;
                        points_with_thread.push_back(points_to_check[i]);
                        break;
                    }
                }

                //if we weren't able to group it with something we saw...this is a new piece!
                if (!canAttach)
                    return false;
            }
        }
    }
    return true;



}



void Thread_Vision::updateCanny()
{
    //FOR DEBUG - don't undistort
#ifdef FAKEIMS
    _cams->updateImagesBlockingNoUndistort();
    _cams->convertToGrayscale();
    //_frames = _cams->frames();
    //_cams->filterCanny();
    //_cannyIms = _cams->cannyIms();
    gray_to_canny();
    precomputeDistanceScores();
#else
    init_timing_fence
    start_timing_fence
    _cams->updateImagesBlocking();
    _cams->convertToGrayscale();
    _frames = _cams->frames();
    _cams->filterCanny();
    _cannyIms = _cams->cannyIms();

    if (reproj_points_fix_canny)
    {
        reproj_points_for_canny();

    }

    precomputeDistanceScores();
    end_timing_fence("canny")
#endif
}


void Thread_Vision::gray_to_canny()
{
    _cannyIms = _cams->frames_gray();
    for (int camNum=0; camNum < NUMCAMS; camNum++)
    {
        for (int row=0; row < _cannyIms[camNum].rows; row++)
        {
            unsigned char* ptr = _cannyIms[camNum].ptr<unsigned char>(row);
            for (int col=0; col < _cannyIms[camNum].cols; col++)
            {
                if (ptr[col] >= 127)
                {
                    ptr[col] = 255;
                }
            }

        }
    }

}

void Thread_Vision::set_reproj_fix_canny(const char* filename)
{
    reproj_points_fix_canny = true;
    ifstream points_in;
    points_in.open(filename);


    while (!points_in.eof())
    {
        double set_num;
        points_in >> set_num;

        //seems to read one last time when data ends...this fixes it
        if (points_in.eof())
            break;

        //skip 3d points
        double for_skip;
        for (int i=0; i < 3*34; i++)
        {
            points_in >> for_skip;
        }

        //save points
        for (int camNum= 0; camNum < NUMCAMS; camNum++)
        {
            to_reproj_clicks[camNum].resize(to_reproj_clicks[camNum].size()+1);
            for (int i=0; i < 34; i++)
            {
                Point2i nextClick;
                points_in >> nextClick.x >> nextClick.y;
                to_reproj_clicks[camNum].back().push_back(nextClick);
            }
        }
    }

}


void Thread_Vision::reproj_points_for_canny()
{
    int image_number = (_captures[0]->getImageNumber())-2;
    //std::cout << "image number " << image_number << std::endl;

    const int num_pts = 75;
    const double score_thresh_reproj = 1.35;

    for (int camNum=0; camNum < NUMCAMS; camNum++)
    {
        //std::cout << "size: " << to_reproj_clicks[camNum][image_number].size() << std::endl;
        for (int i=0; i < to_reproj_clicks[camNum][image_number].size(); i++)
        {
            //std::cout << "checking point" << to_reproj_clicks[camNum][image_number][i] << std::endl;
            if (_captures[camNum]->inRange(to_reproj_clicks[camNum][image_number][i].y, to_reproj_clicks[camNum][image_number][i].x) && score2dPoint(to_reproj_clicks[camNum][image_number][i], camNum) > score_thresh_reproj)
            { 
                Point2f curr_pt((float)to_reproj_clicks[camNum][image_number][i].x, (float)to_reproj_clicks[camNum][image_number][i].y);
                if (i > 0)
                {
                    Point2f prev_pt((float)to_reproj_clicks[camNum][image_number][i-1].x, (float)to_reproj_clicks[camNum][image_number][i-1].y);
                    for (int j=0; j < num_pts; j++)
                    {
                        Point2f point_between = prev_pt + (curr_pt - prev_pt)*(((double)j)/((double)num_pts));
                        //std::cout << "adding point before " << (int)(point_between.y+0.5) << " " << (int)(point_between.x+0.5) << std::endl;
                        _cannyIms[camNum].at<uchar>((int)(point_between.y+0.5), (int)(point_between.x+0.5)) = (unsigned char)255;
                    }
                }
                if (i < to_reproj_clicks[camNum][image_number].size()-1)
                {
                    Point2f next_pt((float)to_reproj_clicks[camNum][image_number][i+1].x, (float)to_reproj_clicks[camNum][image_number][i+1].y);
                    for (int j=0; j < num_pts; j++)
                    {
                        Point2f point_between = curr_pt + (next_pt - curr_pt)*(((double)j)/((double)num_pts));
                        //std::cout << "adding point after " << (int)(point_between.y+0.5) << " " << (int)(point_between.x+0.5) << std::endl;
                        _cannyIms[camNum].at<uchar>((int)(point_between.y+0.5), (int)(point_between.x+0.5)) = (unsigned char)255;
                    }
                }

            }
        }
    }


}


void Thread_Vision::precomputeDistanceScores()
{
    for (int camNum=0; camNum < NUMCAMS; camNum++)
    {
        _cannyDistanceScores[camNum].clear();
        queue<location_and_distance_for_queue> loc_and_dist_to_add;

        for( int y = 0; y < _cannyIms[camNum].rows; y++)
        {
            uchar* Uptr = _cannyIms[camNum].ptr<uchar>(y);
            for( int x = 0; x < _cannyIms[camNum].cols; x++ )
            {
                if (Uptr[x] == (uchar)255)
                {
                    location_and_distance toAdd(y,x,0.0);
                    _cannyDistanceScores[camNum][keyForHashMap(camNum, y, x)] = toAdd;


                    //add to queue
                    for (int xadd=-1; xadd <= 1; xadd++)
                    {
                        int x_next = x+xadd;
                        if (x_next >= 0 && x_next < cols[camNum])
                        {
                            for (int yadd=-1; yadd <= 1; yadd++)
                            {
                                int y_next = y+yadd;
                                if (y_next >= 0 && y_next < rows[camNum])
                                {
                                    if (!(xadd == 0 && yadd == 0))
                                    {
                                        location_and_distance forQueue(y, x, (xadd*xadd + yadd*yadd));
                                        location_and_distance_for_queue toAddToQueue(y_next,x_next,forQueue);
                                        loc_and_dist_to_add.push(toAddToQueue);
                                    }
                                }
                            }
                        }
                    }


                }
            }
        }


        //process queue
        while (!loc_and_dist_to_add.empty())
        {
            location_and_distance_for_queue fromQueue = loc_and_dist_to_add.front();
            location_and_distance ldFromQueue = fromQueue.ld;
            loc_and_dist_to_add.pop();

            int key = keyForHashMap(camNum, fromQueue.rowCheck, fromQueue.colCheck);
            //if we need to replace/add another thread distance
            bool toProcess = false;

            if (_cannyDistanceScores[camNum].count(key) == 0 || ldFromQueue.dist < _cannyDistanceScores[camNum][key].dist)
            {
                _cannyDistanceScores[camNum][key] = ldFromQueue;
                toProcess=true;
            } else if (ldFromQueue.dist == _cannyDistanceScores[camNum][key].dist) {
                location_and_distance* curr = &(_cannyDistanceScores[camNum][key]);
                toProcess = true;
                while (curr->next != NULL)
                {
                    if (curr->row == ldFromQueue.row && curr->col == ldFromQueue.col)
                    {
                        toProcess = false;
                        break;
                    }
                    curr = curr->next;
                }
                if (toProcess)
                {
                    curr->next = new location_and_distance(ldFromQueue.row, ldFromQueue.col, ldFromQueue.dist);
                }
            }


            if (toProcess)
            {
                //add to queue
                for (int xadd=-1; xadd <= 1; xadd++)
                {
                    int x_next = fromQueue.colCheck+xadd;
                    if (x_next >= 0 && x_next < cols[camNum])
                    {
                        for (int yadd=-1; yadd <= 1; yadd++)
                        {
                            int y_next = fromQueue.rowCheck+yadd;
                            if (y_next >= 0 && y_next < rows[camNum])
                            {
                                int nextDist = (pow(y_next-ldFromQueue.row,2) + pow(x_next-ldFromQueue.col,2));
                                if (!(xadd == 0 && yadd == 0) && (nextDist <= DIST_FOR_SCORE_CHECK))
                                {
                                    location_and_distance forQueue(ldFromQueue.row, ldFromQueue.col, nextDist);
                                    location_and_distance_for_queue toAddToQueue(y_next,x_next,forQueue);
                                    loc_and_dist_to_add.push(toAddToQueue);
                                }
                            }
                        }
                    }
                }


            }

        }

    }

}



void Thread_Vision::initializeOnClicks()
{
    _cams->initializeOnClicks();
}

void Thread_Vision::clickOnPoints(Point2i* clickPoints)
{
    _cams->getClickedPoints(clickPoints);
}


void Thread_Vision::clickOnPoints(Point3f& clickPoints)
{
    _cams->getClickedPoints(clickPoints);
}










void Thread_Vision::addThreadPointsToDebug(const Scalar& color)
{
    Point3f currPoint;
    Point2i imPoints[NUMCAMS];
    int numPoints = best_thread_hypoths->at(curr_hypoth_ind)->num_pieces();

    gl_display_for_debug.resize(gl_display_for_debug.size()+1);
    gl_display_for_debug[gl_display_for_debug.size()-1].vertices = new Point3f[(numPoints)];
    gl_display_for_debug[gl_display_for_debug.size()-1].size = (numPoints);
    gl_display_for_debug[gl_display_for_debug.size()-1].color[0] = ((double)color[2])/255.0;
    gl_display_for_debug[gl_display_for_debug.size()-1].color[1] = ((double)color[1])/255.0;
    gl_display_for_debug[gl_display_for_debug.size()-1].color[2] = ((double)color[0])/255.0;


    vector<Vector3d> points;
    vector<double> angles;
    best_thread_hypoths->at(curr_hypoth_ind)->get_thread_data(points, angles);

    for (int ind=0; ind < numPoints; ind++)
    {
        EigenToOpencv(points[ind], currPoint);
        gl_display_for_debug.back().vertices[ind] = currPoint;
    }

}

void Thread_Vision::addThreadPointsToDebugImages(const Scalar& color, Thread* thread)
{
    Point3f currPoint;
    Point2i imPoints[NUMCAMS];
    vector<Vector3d> points;
    vector<double> angles;
    thread->get_thread_data(points, angles);
    int numPoints = points.size();

    for (int j = 0; j < NUMCAMS; j++)
    {
        display_for_debug[j].resize(display_for_debug[j].size()+1);
        display_for_debug[j][display_for_debug[j].size()-1].pts = new Point[(numPoints)*4];
        display_for_debug[j][display_for_debug[j].size()-1].size = (numPoints)*4;
        display_for_debug[j][display_for_debug[j].size()-1].color = color;
    }



    for (int ind=0; ind < numPoints; ind++)
    {
        EigenToOpencv(points[ind], currPoint);
        _cams->project3dPoint(currPoint,imPoints);

        //std::cout << "curr curve params: " << curr->_curvature << "  " << curr->_torsion << std::endl;     


        for (int j = 0; j < NUMCAMS; j++)
        {
            display_for_debug[j][display_for_debug[j].size()-1].pts[4*ind  ] = Point(imPoints[j].x-1, imPoints[j].y-1);
            display_for_debug[j][display_for_debug[j].size()-1].pts[4*ind+1] = Point(imPoints[j].x-1, imPoints[j].y+1);
            display_for_debug[j][display_for_debug[j].size()-1].pts[4*ind+2] = Point(imPoints[j].x+1, imPoints[j].y+1);
            display_for_debug[j][display_for_debug[j].size()-1].pts[4*ind+3] = Point(imPoints[j].x+1, imPoints[j].y-1);
        }
    }


    //FOR DEBUG - actually display

    for (int i=0; i < NUMCAMS; i++)
    {
        for (int disp_ind = 0; disp_ind < display_for_debug[i].size(); disp_ind++)
        {
            const Point* pt[] = {display_for_debug[i][disp_ind].pts};

            cv::polylines(_frames[i], pt, &display_for_debug[i][disp_ind].size, 1, false, display_for_debug[i][disp_ind].color, 2);

        }

    }



    //_cams->saveImages(false);

    for (int i=0; i < NUMCAMS; i++)
    {
        for (int disp_ind = 0; disp_ind < display_for_debug[i].size(); disp_ind++)
        {
            const Point* pt[] = {display_for_debug[i][disp_ind].pts};

            cv::polylines(_cannyIms_display[i], pt, &display_for_debug[i][disp_ind].size, 1, false, display_for_debug[i][disp_ind].color, 2);

        }

    }



}



void Thread_Vision::addThreadPointsToDebugImages(const Scalar& color)
{
    addThreadPointsToDebugImages(color, best_thread_hypoths->at(curr_hypoth_ind));
}

void Thread_Vision::add_debug_points_to_ims()
{
    for (int i=0; i < NUMCAMS; i++)
    {
        for (int disp_ind = 0; disp_ind < display_for_debug[i].size(); disp_ind++)
        {
            const Point* pt[] = {display_for_debug[i][disp_ind].pts};

            cv::polylines(_frames[i], pt, &display_for_debug[i][disp_ind].size, 1, false, display_for_debug[i][disp_ind].color, 2);

        }

    }



    //_cams->saveImages(false);

    for (int i=0; i < NUMCAMS; i++)
    {
        cvtColor(_cannyIms[i], _cannyIms_display[i], CV_GRAY2BGR);
        for (int disp_ind = 0; disp_ind < display_for_debug[i].size(); disp_ind++)
        {
            const Point* pt[] = {display_for_debug[i][disp_ind].pts};

            cv::polylines(_cannyIms_display[i], pt, &display_for_debug[i][disp_ind].size, 1, false, display_for_debug[i][disp_ind].color, 2);

        }

    }



}



void Thread_Vision::display()
{


    /*
    char cannyName[256];
    sprintf(cannyName, "%scanny_", IMAGE_SAVE_BASE);

_cams->saveImages(true, cannyName);
*/

for (int i=0; i < NUMCAMS; i++)
{
    cv::imshow(_canny_display_names[i], _cannyIms_display[i]); 
    cv::imshow(_orig_display_names[i], _frames[i]); 
}
waitKey(5);

}


void Thread_Vision::saveImages(const char* image_save_base, int im_num)
{
    char filename[256];
    for (int camNum=0; camNum < NUMCAMS; camNum++)
    { 
        sprintf(filename, "%s%d-%d.jpg", image_save_base, (camNum+1), im_num);
        imwrite(filename, _frames[camNum]);
        sprintf(filename, "%s_canny%d-%d.jpg", image_save_base, (camNum+1), im_num);
        imwrite(filename, _cannyIms[camNum]);
        sprintf(filename, "%s_cannyDisplay%d-%d.jpg", image_save_base, (camNum+1), im_num);
        imwrite(filename, _cannyIms_display[camNum]);
    }
}






const Matrix3d& Thread_Vision::start_rot(void) {return best_thread_hypoths->at(curr_hypoth_ind)->start_rot();}
const Matrix3d& Thread_Vision::end_rot(void) {return best_thread_hypoths->at(curr_hypoth_ind)->end_rot();}
const Matrix3d& Thread_Vision::end_bishop(void) {return best_thread_hypoths->at(curr_hypoth_ind)->end_bishop();}
const double Thread_Vision::end_angle(void) {return best_thread_hypoths->at(curr_hypoth_ind)->end_angle();}
const double Thread_Vision::angle_at_ind(const int i) {return best_thread_hypoths->at(curr_hypoth_ind)->angle_at_ind(i);}

const Vector3d& Thread_Vision::start_pos(void) {return best_thread_hypoths->at(curr_hypoth_ind)->start_pos();}
const Vector3d& Thread_Vision::end_pos(void) {return best_thread_hypoths->at(curr_hypoth_ind)->end_pos();}

const Vector3d& Thread_Vision::start_edge(void) {return best_thread_hypoths->at(curr_hypoth_ind)->start_edge();}
const Vector3d& Thread_Vision::end_edge(void) {return best_thread_hypoths->at(curr_hypoth_ind)->end_edge();}







location_and_distance::~location_and_distance()
{
    if (next != NULL)
    {
        delete next;
        next = NULL;
    }

}


