
# include <opencv/cv.h>
# include <opencv/highgui.h>
# include <iostream> 

# define RECT_WIDTH 93
# define RECT_HEIGHT 62
# define RECT_X_PADDING 0
# define RECT_Y_PADDING 0

/* 
 * 
 * Inputs: 
 *  x*,y* -- image location of trapezoid around band
 *  prefix, imNum, suffix -- in_file_path = prefix + imNum + suffix;
 * 
 * Outputs: Writes a warped/rectified image of the band to a hardcoded location on disk.
 * 
 * See harness.py
 */
int main(int argc, char * argv[]) {    
    // declarations
    cv::Point2f srcQuad[4], dstQuad[4];
    cv::Mat warp_matrix, im;
    std::string filename;
    cv::Size dsize(2*RECT_X_PADDING+RECT_WIDTH, 2*RECT_Y_PADDING+RECT_HEIGHT);
    cv::Mat warpedIm(dsize, CV_32FC1);
    
    std::cout << "hello world!" << std::endl;
    
    if (!(argc == 12)) {
        std::cout << "Usage: rectifyIm x0 y0 x1 y1 x2 y2 x3 y3 prefix imNum suffix" << std::endl;
        exit(1);
    }

    // Coordinates of trapezoid
    for(int i=0; i<=3; i++) {
        srcQuad[i].x = atof(argv[2*i+1]);
        srcQuad[i].y = atof(argv[2*i+2]);
        //std::cout << "srcQuad" << i << ": "<<srcQuad[i].x << " " << srcQuad[i].y << std::endl;
    }
   
    // Destination rectangle coordinates
    dstQuad[0].x = RECT_X_PADDING;
    dstQuad[0].y = RECT_Y_PADDING + RECT_HEIGHT;
    dstQuad[1].x = RECT_X_PADDING;
    dstQuad[1].y = RECT_Y_PADDING;
    dstQuad[2].x = RECT_X_PADDING + RECT_WIDTH;
    dstQuad[2].y = RECT_Y_PADDING;
    dstQuad[3].x = RECT_X_PADDING + RECT_WIDTH;
    dstQuad[3].y = RECT_Y_PADDING + RECT_HEIGHT;    
    for(int i=0; i<=3; i++) {
        //std::cout << "dstQuad" << i << ": " << dstQuad[i].x << " " << dstQuad[i].y << std::endl;   
    }
    
    // get the transformation matrix
    warp_matrix = cv::getPerspectiveTransform(srcQuad, dstQuad);
    std::cout << " warp_matrix " << std::endl;
    for (int i=0; i<3; i++) {
        std::cout << warp_matrix.at<float>(i,0) << " " << warp_matrix.at<float>(i,1) << " " << warp_matrix.at<float>(i,2) << std::endl;
    }

    // transform image
    std::string prefix = argv[9];
    std::string imNum = argv[10];
    std::string suffix = argv[11];
    filename = prefix + imNum + suffix;
    std::cout << filename << std::endl;
    im = cv::imread(filename, 0);
    if (im.data == NULL) {
        std::cout << "Not able to read image." << std::endl;
        exit(-1);
    }
    std::cout << "depth: " << im.depth() << std::endl;
/*    cv::namedWindow("Before", CV_WINDOW_AUTOSIZE);
    cv::imshow("Before", im);*/
 
    int flags = cv::INTER_LINEAR;
    cv::warpPerspective(im, warpedIm, warp_matrix, dsize, flags, cv::BORDER_CONSTANT, cv::Scalar());
/*    cv::namedWindow("After", CV_WINDOW_AUTOSIZE);
    cv::imshow("After", warpedIm);
    cv::waitKey(0);*/

    std::string prefix1 = "/home/jinna/tracking/data/2010Jul01/rect/BandOnly/warped_test1-";
    std::string outfile = prefix1  + imNum + suffix;
    std::cout << outfile << std::endl;
    std::cout << "out depth: " << warpedIm.depth() << std::endl;
    cv::imwrite(outfile, warpedIm);
    return 0;
}
