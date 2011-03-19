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
    waitKey(0);

    float x = atof(argv[2]);
    float y = atof(argv[3]);
    Point midpt(x,y);
    float dx = atof(argv[4]);
    float dy = atof(argv[5]);
    float z = sqrt(dx*dx + dy*dy); 
    dx = dx/z;
    dy = dy/z;
    Vec2f norm(dx, dy);
    std::cout << "in params: " << x << " " << y << " " << dx << " " << dy << std::endl;
    
    float radius = 100;
    float ex_armw = 23;
    Point point1(norm[0]*ex_armw + x, norm[1]*ex_armw + y);
    Point point2(-norm[0]*ex_armw + x, -norm[1]*ex_armw + y);
    
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
    Dx = imread(inpath.c_str(), GRAYSCALE_IMAGE);
    if (Dx.data == NULL) {
        std::cerr << "ERROR reading image 3" << std::endl;
        exit(-1);
    }

    inpath = argv[7];
    Dy = imread(inpath.c_str(), GRAYSCALE_IMAGE);
    if (Dy.data == NULL) {
        std::cerr << "ERROR reading image 4" << std::endl;
        exit(-1);
    }
    namedWindow("Dx", cwa);
    imshow("Dx", Dx);
    namedWindow("Dy", cwa);
    imshow("Dy", Dy);
    
    std::cout << "Dx, Dy type: " << Dx.type() << std::endl;
    float dirSimThresh = .9;
    Point bestPt1(0,0);
    float bestscore = -10000000;
    
    for (int i=0; (unsigned)i<chosenPts1.size(); i++) {
        int r = (int)chosenPts1[i].y;
        int c = (int)chosenPts1[i].x;
        
        float gradx = Dx.at<float>(r,c);
        float grady = Dy.at<float>(r,c);
        
        float dirSim = (gradx*dx + grady*dy)/sqrt(gradx*gradx + grady*grady);
        float gradStrength = sqrt(gradx*gradx + grady*grady);
        
        if (dirSim > dirSimThresh) {
            std::cout << chosenPts1[i].x << " " << chosenPts1[i].y << std::endl;
            if (gradStrength > bestscore) {
                bestPt1.x = chosenPts1[i].x;   
                bestPt1.y = chosenPts1[i].y;
                bestscore = gradStrength;
            }
        }
    }
    
    std::cout << "bestPt1=" << bestPt1.x << " " << bestPt1.y << std::endl;

}
