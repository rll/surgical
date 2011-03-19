# include <iostream>
# include <opencv/cv.h>
# include <opencv/highgui.h>
# include "utils.h"

using namespace cv;

/*
 * Inputs: 
 *  filepath -- path to grayscale image
 *  ex_loc_x, ex_loc_y -- where you would expect to find the model origin in the image.
 *  ex_rho -- expected perpendicular distance to image origin. 
 *            Note: opencv's hough transform uses a different origin than Matlab's (negate Matlab rho?)
 *  ex_angle -- expected angle in degrees
 *  cannythresh_low, cannythresh_high -- parameters for the Canny edge detector.
 *  houghthresh -- how many votes a line needs to have in order to be considered 
 *  magthresh -- only points with gradient magnitude larger than this are put into the pool of candidates. 
 *  debug -- if this argument is present, then a lot of intermediate variables are saved to disk.
 * 
 * Example: 
 *  ./locateMid /home/jinna/surgical/tracking/data/2010Jul23/rect/knottie1-_rect573.ppm  507 459 100 -38
 */
int main(int argc, char** argv) {
    // inputs and parameters

    int numArgs = 10;
    if (argc < numArgs) {
        std::cerr << "Usage: exe filepath ex_loc_x ex_loc_y ex_rho ex_angle(deg) cannythresh_low cannythresh_high houghthresh magthresh [debug]" << std::endl;
        exit(1);
    }
    std::string filepath  = argv[1];
    Point2f ex_loc(atof(argv[2]), atof(argv[3]));
    float ex_rho = atof(argv[4]);
    float ex_angle = atof(argv[5]);
    double cannythresh[] = {atof(argv[6]), atof(argv[7])};
    int houghthresh = atoi(argv[8]);    
    float magthresh = atof(argv[9]);
    float simthresh = .9;
    bool left = true;
    bool debug = false;
    if (argc > numArgs) {
        std::string debugStr = argv[argc-1];
        if (debugStr == "d") {
            debug = true;
            std::cout << "debug mode" << std::endl;
        }
    }    
    
    Mat img;
    
    img = imread(filepath.c_str(), GRAYSCALE_IMAGE);
    if (img.data == NULL) {
        std::cerr << "there was an error reading the image" << std::endl;
        exit(1);
    }
    if (img.channels() != 1) {
        std::cerr << "img must be grayscale. channels= " << img.channels() << std::endl;
        exit(1);
    }
    
//     namedWindow("orig image", CV_WINDOW_AUTOSIZE);
//     imshow("orig image", img);
    
    int nx = img.size().width;
    int ny = img.size().height;
    float ex_gray = .1;
    float gray_tol = .15;
    float rho_tol = 50;
    float angle_tol = 2;
    float armw_min = 60;
    float armw_max = 80;
    float angd_tol = 1;
    float sig = 2;
//     float cannythresh[] = {.05, .1};
    
    // Smooth image
    Mat imgf(img.size(), CV_32FC1);
    img.convertTo(imgf, CV_32FC1);
    imgf = imgf/(255.0);
    
    Mat imgsm(img.size(), CV_32FC1);
    GaussianBlur(imgf, imgsm, Size(17,17), sig, sig);
//     namedWindow("blurred", CV_WINDOW_AUTOSIZE);
//     imshow("blurred", imgsm);
    if (debug) {
        saveToFile<float>(imgsm, "imgsm_locateMid.txt");
    }
    
    // Perform Canny detection
    // first make sure array is in right format
    Mat imgsm256(imgsm.size(), CV_8UC1);
//     imgsm = imgsm*255;    
    imgsm.convertTo(imgsm256, CV_8UC1, 255.0);
    Mat imgsm256_output(imgsm.size(), DataType<int>::type);
    imgsm256.convertTo(imgsm256_output, DataType<int>::type);
    if (debug) {
        saveToFile<int>(imgsm256_output, "imgsm256.txt");
    }
    
    Mat bw;
    std::cout << "cannythresh: " << cannythresh[0] << " " << cannythresh[1] << std::endl;
    Canny(imgsm256, bw, cannythresh[0], cannythresh[1]);
//     namedWindow("blurred2", CV_WINDOW_AUTOSIZE);
//     imshow("blurred2", imgsm256);
    
//     namedWindow("canny edges", CV_WINDOW_AUTOSIZE);
//     imshow("canny edges", bw);
    if (debug) {
        saveToFile<int>(bw, "bw.txt");
    }
    
    // Hough lines
    vector<Vec2f> lines;
    double rhoRes = .5;
    double thetaRes = .1;
    
    HoughLines(bw, lines, rhoRes, thetaRes*RADIANS_PER_DEGREE, houghthresh);
    std::cout << lines.size() << " lines found" << std::endl;
    
    // suppress neighborhods -- matlab does this automatically
    float rho_nhood = 5; // pixels
    float theta_nhood = 2*RADIANS_PER_DEGREE;
    bool yes[lines.size()];
    int total = 0;
    
    for( int i = 0; (unsigned)i < lines.size(); i++ )
    {
        float rho = lines[i][0];
        float theta = lines[i][1];
        
        bool take = true;
        for (int j=0; j<i; j++) {
            if (yes[j] && abs(rho - lines[j][0]) < rho_nhood && abs(theta - lines[j][1]) < theta_nhood) {
                take = false;
            }
        }
        yes[i] = take;
        if (take) {
            total++;
        }
    }
    
    vector<Vec2f> lines_sparse(total);
    int place = 0;
    for (int i=0; (unsigned)i<lines.size(); i++) {
        if (yes[i]) {
            lines_sparse[place][0] = lines[i][0];
            lines_sparse[place][1] = lines[i][1];
            place++;
        }
    }

    // display
//     Mat bw_disp2 = draw_lines(bw, lines);
//     namedWindow("all hough lines", CV_WINDOW_AUTOSIZE);
//     imshow("all hough lines", bw_disp2);

    
//     Mat bw_disp = draw_lines(bw, lines_sparse);
//     namedWindow("sparse hough lines", CV_WINDOW_AUTOSIZE);
//     imshow("sparse hough lines", bw_disp);

    // filter peaks according to closeness to expected angle and rho
    total = 0;
    bool yes_peaks[lines_sparse.size()];
    for (int i=0; (unsigned)i<lines_sparse.size(); i++) {
        float angle = lines_sparse[i][1]/((float)RADIANS_PER_DEGREE) ;
        float tmpdiff = abs(angle - ex_angle); // in degrees
        tmpdiff = fmod(tmpdiff, 180);
        float angleDiff = min(tmpdiff, 180-tmpdiff);
        float rhoDiff = abs(lines_sparse[i][0] - ex_rho) ;
        std::cout << "angle =" << angle << ", angleDiff=" << angleDiff << std::endl;
        std::cout << "rho=" << lines_sparse[i][0] << ", rhoDiff=" << rhoDiff << std::endl;
        if (angleDiff < angle_tol && rhoDiff < rho_tol) {
            yes_peaks[i] = true;   
            total++;
        } else {
            yes_peaks[i] = false;
        }   
    }
    std::cout << total << " lines chosen" << std::endl;
    
    vector<Vec2f> lines_chosen(total);
    place = 0;
    for (int i=0; (unsigned)i<lines_sparse.size(); i++) {
        if (yes_peaks[i]) {
            lines_chosen[place][0] = lines_sparse[i][0];
            lines_chosen[place][1] = lines_sparse[i][1];
            place++;
        }
    }
    
//     Mat disp_chosen = draw_lines(bw, lines_chosen);
//     namedWindow("chosen lines", CV_WINDOW_AUTOSIZE);
//     imshow("chosen lines", disp_chosen);
    
    // find parallel lines the right width apart
    // get indices
    int ii[lines_chosen.size()*lines_chosen.size()];
    int jj[lines_chosen.size()*lines_chosen.size()];
    place = 0;
    for (int i=0; (unsigned)i<lines_chosen.size(); i++) {
        float angle1 = lines_chosen[i][1]/(float)RADIANS_PER_DEGREE;
        float rho1 = lines_chosen[i][0];
        for (int j=0; j<i; j++) {
            float angle2 = lines_chosen[j][1]/(float)RADIANS_PER_DEGREE;
            float rho2 = lines_chosen[j][0];
            
            float tmpdiff = abs(angle1-angle2);
            float angd = min(tmpdiff, 180-tmpdiff);
            float dist = abs(rho1-rho2);
            
            if (dist<armw_max && dist>armw_min && angd<angd_tol) {
                ii[place] = i;
                jj[place] = j;
                place++;
            }
        }
    }
    total = place;
    std::cout << "num pairs: " << total << std::endl;

    // retrieve lines from indices
    vector<Vec2f> lines_chosen1(total);
    vector<Vec2f> lines_chosen2(total);
    for (int l=0; l<total; l++) {
        lines_chosen1[l][0] = lines_chosen[ii[l]][0];
        lines_chosen1[l][1] = lines_chosen[ii[l]][1];
        lines_chosen2[l][0] = lines_chosen[jj[l]][0];
        lines_chosen2[l][1] = lines_chosen[jj[l]][1];
    }
    
    // display all pairs
//     Mat disp;
//     vector<Vec2f> tmplines(2);
//     for (int i=0; (unsigned)i<lines_chosen1.size(); i++) {
//         namedWindow("pair " + ((char)i + '0'), CV_WINDOW_AUTOSIZE);
//         tmplines[0][0] = lines_chosen1[i][0];
//         tmplines[0][1] = lines_chosen1[i][1];
//         tmplines[1][0] = lines_chosen2[i][0];
//         tmplines[1][1] = lines_chosen2[i][1];
//         disp = draw_lines(bw, tmplines);
//         imshow("pair " + ((char)i + '0'), disp);
//     }

    // for each pair, find midline
    vector<Vec2f> midlines(lines_chosen1.size());
    for (int i=0; (unsigned)i<lines_chosen1.size(); i++) {
        midlines[i][0] = (lines_chosen1[i][0] + lines_chosen2[i][0])/2;
        midlines[i][1] = lines_chosen1[i][1];       // angles are supposed to be so close together that it won't matter.
    }
    Mat disp = draw_lines(bw, midlines);
    namedWindow("midlines", CV_WINDOW_AUTOSIZE);
    imshow("midlines", disp);
    
    // walk along midlines, find high gradients in right direction 
    // first get orientations
    Mat Dx(img.size(), CV_32FC1);
    Mat Dy(img.size(), CV_32FC1);
    Mat dMags(img.size(), CV_32FC1);
    int nconsider = 6;
    
    Mat kernel_x, kernel_y;
    kernel_x = dgauss(sig);
    kernel_y = dgauss(sig).t();
    flip(kernel_x, kernel_x, 1);    // convolution, not correlation, kernels
    flip(kernel_y, kernel_y, 0);    
    
    filter2D(imgf, Dx, Dx.depth(), kernel_x, Point(-1,-1), 0, BORDER_REPLICATE);
    filter2D(imgf, Dy, Dy.depth(), kernel_y, Point(-1,-1), 0, BORDER_REPLICATE);
    if (debug) {
        saveToFile<float>(Dx, "Dx_locateMid.txt");
        saveToFile<float>(Dy, "Dy_locateMid.txt");
    }
    
    magnitude(Dx, Dy, dMags);
    
    Point2f candidate_locs[midlines.size()*nconsider];
    float candMags[midlines.size()*nconsider];
    float candSims[midlines.size()*nconsider];
    
    cvtColor(bw, disp, CV_GRAY2RGB);    
    total=0;
    int candPlace = 0;
    for (int i=0; (unsigned)i<midlines.size(); i++) {
        float s = sin(midlines[i][1]);
        float c = cos(midlines[i][1]);
        std::cout << "cos=" << c << ", sin=" << s << std::endl;
        std::cout << "rho=" << midlines[i][0] << std::endl;
        
        float dx = s;
        float dy = -c;
        
        float stepx = dx/max(fabs(dx), fabs(dy));
        float stepy = dy/max(fabs(dx), fabs(dy));
//         std::cout << "max=" << max(fabs(dx), fabs(dy)) << ", fabs(dx)=" << fabs(dx) << ", fabs(dy)" << fabs(dy) << std::endl;
        std::cout << "stepx=" << stepx << ", stepy=" << stepy << std::endl;
        
        int sgn = 1;

        if (dx < 0 && left) {
            sgn = -1;
        } else if (dx > 0 && ~left) {
            sgn = -1;
        }
        
        std::cout << "dx=" << dx << ", dy=" << dy << std::endl;
        
        int nsteps = min(floor(fabs(nx/stepx)), floor(abs(ny/stepy)));
        float icept_y = midlines[i][0]/s;
        float icept_x = midlines[i][0]/c;
        std::cout << "icept_y " << icept_y << std::endl;
        std::cout << "icept_x " << icept_x << std::endl;
        
        float tx,ty;
        float gradMags[nsteps-2];
        float vals[nsteps-2];
        Mat tX(2, nsteps-2, CV_32FC1);
        Mat graddirs(2, nsteps-2, CV_32FC1);
        Mat gradDirSims(nsteps-2, 1, CV_32FC1);
        
        std::cout << "nsteps=" << nsteps << std::endl;
        for (int j=0; j<nsteps-2; j++) {// to be super-safe about boundaries
            if (left) {
                tx = dx*(j+1);
            } else {
                tx = nx + dx*(j+1);   
            }
            ty = dy*(j+1);

            if (icept_y >= 0) {
                ty = ty+icept_y;
            } else {
                tx = tx+icept_x;   
            }
            tX.at<float>(0,j) = tx;
            tX.at<float>(1,j) = ty;
//             std::cout << "tX=" << tx << " " << ty << std::endl;
            
            vals[j] = imgsm.at<float>(sround(ty), sround(tx));
            float gradx = Dx.at<float>(sround(ty),sround(tx));
            float grady = Dy.at<float>(sround(ty),sround(tx));
//             std::cout << "gradx=" << gradx << ", grady=" << grady << std::endl;
            float norm = sqrt(gradx*gradx + grady*grady);
            gradMags[j] = norm;
            if (norm > 0) {
                graddirs.at<float>(0,j) = gradx/norm;
                graddirs.at<float>(1,j) = grady/norm;
                gradDirSims.at<float>(j,0) = (dx*gradx + dy*grady)/norm;
            } else {
                graddirs.at<float>(0,j) = 0;
                graddirs.at<float>(1,j) = 0;
                gradDirSims.at<float>(j,0) = 0;
            }
        }
        
        if (debug) {
            saveArrayToFile<float>(gradMags, nsteps-2, "gradMags.txt");
            saveToFile<float>(graddirs, "graddirs.txt");
            saveToFile<float>(gradDirSims, "gradDirSims.txt");
            saveArrayToFile<float>(vals, nsteps-2, "vals.txt");
            saveToFile<float>(tX, "tX.txt");
        }
        
        int n_this_line=0;
        for (int j=0; j<nsteps-2; j++) {// to be super-safe about boundaries
            if ( (gradMags[j] > magthresh) 
                && (gradDirSims.at<float>(j,0) > simthresh)
                && ( j<=2 || abs(vals[j-3] - ex_gray) < gray_tol )) {
                    candidate_locs[candPlace].x = tX.at<float>(0,j);
                    candidate_locs[candPlace].y = tX.at<float>(1,j);
                    candMags[candPlace] = gradMags[j];
                    candSims[candPlace] = gradDirSims.at<float>(j,0);
                    n_this_line++;
                    candPlace++;
            }
            if (n_this_line >= nconsider) {
                break;   
            }
        }    
    } // for i from 0 to midlines.size()
    int numCands = candPlace;
    std::cout << "numCands " << numCands << std::endl;
    
    int midpt_ind;
    float min_score = 100000;
    for (int i=0; i<numCands; i++) {
        float diffx = candidate_locs[i].x - ex_loc.x;
        float diffy = candidate_locs[i].y - ex_loc.y;
        float dist = sqrt(diffx*diffx + diffy*diffy);
        float score = dist/candMags[i];
        
        if (score < min_score) {
            min_score = score;
            midpt_ind = i;
        }
    }

    if (numCands <= 0) {
        std::cout << "NO GOOD MATCHES FOUND" << std::endl;   
        return 1;
    }
    Point2f midpt(-1,-1);
    midpt.x = candidate_locs[midpt_ind].x;
    midpt.y = candidate_locs[midpt_ind].y;   
    std::cout << "MIDPOINT EQUALS (" << midpt.x << ", " << midpt.y << ")" << std::endl;
    
    cvtColor(imgsm256, disp, CV_GRAY2RGB);    
    for (int i=0; i<numCands; i++) {
        circle(disp, candidate_locs[i], 2, Scalar(0,0,255), 1);   
    }
    circle(disp, midpt, 1, Scalar(255, 255, 0), 1);
    namedWindow("midpt", CV_WINDOW_AUTOSIZE);
    imshow("midpt", disp);
    waitKey(0);
}

