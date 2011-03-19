# include <iostream>
# include <opencv/cv.h>
# include <opencv/highgui.h>
# include "utils.h"

using namespace cv;
int main(int argc, char** argv) {
    int cwa = CV_WINDOW_AUTOSIZE;
    
    Mat img, mask;
    std::string inpath = argv[1];
    mask = imread(inpath.c_str(), GRAYSCALE_IMAGE);
    if (mask.data == NULL) {
        std::cerr << "ERROR reading image 2" << std::endl;
        exit(-1);
    }
    namedWindow("mask", cwa);
    imshow("mask", mask);
    
    vector<vector<Point> > contours;
    findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    std::cout << "contours size" << contours.size() << std::endl;
    std::cout << "1st contour size" << contours[0].size() << std::endl;
    
    Mat disp;
    cvtColor(mask, disp, CV_GRAY2RGB);
    drawContours(disp, contours, -1, Scalar(0,255,0),2,4);
    namedWindow("contours", cwa);
    imshow("contours", disp);

    float x = atof(argv[2]);
    float y = atof(argv[3]);
    Point midpt(x,y);
    float dx = atof(argv[4]);
    float dy = atof(argv[5]);
    float z = sqrt(dx*dx + dy*dy); 
    dx = dx/z;
    dy = dy/z;
    Vec2f norm(-dy, dx);
    std::cout << "in params: " << x << " " << y << " " << dx << " " << dy << std::endl;
    
    float radius = 10;
    float ex_armw = 23;
    Point point1(norm[0]*ex_armw + x, norm[1]*ex_armw + y);
    Point point2(-norm[0]*ex_armw + x, -norm[1]*ex_armw + y);
    std::cout << "point1=" << point1.x << " " << point1.y << std::endl;
    std::cout << "point2=" << point2.x << " " << point2.y << std::endl;
    
    vector<Point> chosenPts1; 
    vector<Point> chosenPts2; 
    
    for (int i=0; (unsigned)i<contours.size(); i++) {
        for (int j=0; (unsigned)j<contours[i].size(); j++) {
            Point diff1 = point1 - contours[i][j];
            Point diff2 = point2 - contours[i][j];
            float dist1sq = diff1.x*diff1.x + diff1.y*diff1.y;
            float dist2sq = diff2.x*diff2.x + diff2.y*diff2.y;
            
            if (dist1sq <= radius*radius) {
                Point newpoint(contours[i][j]);
                chosenPts1.push_back(newpoint);
            }
            
            if (dist2sq <= radius*radius) {
                Point newpoint(contours[i][j]);
                chosenPts2.push_back(newpoint);
            }
            
        }
    }
    
    Mat Dx, Dy;
    inpath = argv[6];
    int ny = mask.size().height;
    int nx = mask.size().width;
//     Dx = imread(inpath.c_str(), GRAYSCALE_IMAGE);
    Dx = readFromTxt<float>(inpath, ny, nx, CV_32FC1);
    if (Dx.data == NULL) {
        std::cerr << "ERROR reading image 3" << std::endl;
        exit(-1);
    }

    inpath = argv[7];
    Dy = readFromTxt<float>(inpath, ny, nx, CV_32FC1);
    if (Dy.data == NULL) {
        std::cerr << "ERROR reading image 4" << std::endl;
        exit(-1);
    }
    namedWindow("Dx", cwa);
    imshow("Dx", Dx);
    namedWindow("Dy", cwa);
    imshow("Dy", Dy);
    
    std::cout << "Dx, Dy type: " << Dx.type() << std::endl;
    float dirSimThresh = .8;
    Point bestPt1(0,0);
    float bestscore = 10000000;
    
    float ex_gray1 = atof(argv[8]);
    float ex_gray2 = atof(argv[9]);
    inpath = argv[9];
    img = imread(inpath.c_str(), GRAYSCALE_IMAGE);
    std::cout << "imdepth=" << img.depth() << std::endl;
    std::cout << "imchannels=" << img.channels() << std::endl;
    if (img.data == NULL) {
        std::cerr << "ERROR reading image 6" << std::endl;
        exit(-1);
    }
    Mat imgf; img.convertTo(imgf, CV_32FC1);
    imgf = imgf/255.0;
    namedWindow("img", cwa);
    imshow("img", img);
    
    waitKey(0);
    
    for (int i=0; (unsigned)i<chosenPts1.size(); i++) {
        int r = (int)chosenPts1[i].y;
        int c = (int)chosenPts1[i].x;
        
        float gradx = Dx.at<float>(r,c);
        float grady = Dy.at<float>(r,c);
        
        float dirSim = (gradx*dx + grady*dy)/sqrt(gradx*gradx + grady*grady);
        float gradStrength = sqrt(gradx*gradx + grady*grady);
        std::cout << c << " " << r << std::endl;
//         std::cout << gradStrength << std::endl;
//         std::cout << gradx << " " << grady << std::endl;
//         std::cout << dirSim << std::endl;
        
        float vecLen = 2;
        int test_c = c+vecLen*floor(-dy+dx);
        int test_r = r+vecLen*floor(-dx-dy);
//         std::cout << test_r << " " << test_c << std::endl;
        float test_gray_val1 = imgf.at<float>(test_r, test_c);
//         std::cout << "test_gray_val=" << test_gray_val << std::endl;
        test_c = c+vecLen*floor(dx-dy);
        test_r = r+vecLen*floor(dy+dx);
        float test_gray_val2 = imgf.at<float>(test_r, test_c);
        float valdiff1 = fabs(test_gray_val1 - ex_gray1);
        float valdiff2 = fabs(test_gray_val2 - ex_gray2);
        
        float score = valdiff1 + valdiff2;
//         std::cout << valdiff1 << " " << valdiff2 << std::endl;

//         if (dirSim > dirSimThresh) {// && abs(test_gray_val - ex_gray) < .05) {
//             std::cout << chosenPts1[i].x << " " << chosenPts1[i].y << std::endl;
//             if (gradStrength > bestscore) {
//                 bestPt1.x = chosenPts1[i].x;   
//                 bestPt1.y = chosenPts1[i].y;
//                 bestscore = gradStrength;
//             }
//         }
    }
    std::cout << "bestPt1=" << bestPt1.x << " " << bestPt1.y << std::endl;
}
