/** 
  Calculate HOG descriptors of all metal band images
**/
# include <opencv/cv.h>
# include <opencv/highgui.h>
# include <opencv/cvaux.h>
# include <iostream> 
# include <math.h>
# include "utils.h"

using namespace cv;

void compute_ori_gradients(Mat im, int nbins, Mat& magMap, Mat& obinMap);
Mat compute_sp_descriptor(Mat magMap, Mat obinMap, int nlevels, int nbins, double thresh);
Mat compute_histogram(Mat& obinMap, Mat& magMap, int ay, int by, int ax, int bx, int nbins, double thresh);
void set_vals(Mat& mat1, int place, Mat& mat2);
void dgaussTest();

/*
 * Executable takes as arguments...
 *  path to image of wrist, warped to fit a rectangle exactly
 *  nbins = number of orientation bins
 *  nlevels = number of levels in the spatial pyramid
 *  descriptor-thresh = gradient magnitude at this point has to be greater 
    than descriptor-thresh in order to be included in the histogram.
 * 
 * Compiles to the executable "calchog". 
 * Example: ./calchog ~/tracking/data/2010Jul01/rect/BandOnly/warped_test1-411.ppm 16 3 .02
 */

int main(int argc, char** argv) {
    // args: path to the image, nbins, nlevels
    if (argc == 1) {
        std::cerr << "No args: automatically doing dgauss test" << std::endl;
        dgaussTest();
        exit(0);
    }
    if (argc < 5) {
        std::cerr << "Usage: program path-to-warped-wrist-image nbins nlevels descriptor-thresh" << std::endl;
        exit(1);
    }
        
    Mat img, magMap, obinMap;    
    Mat desc;
    int nbins = atoi(argv[2]);
    int nlevels = atoi(argv[3]);
    double thresh = atof(argv[4]);
    std::string filepath = argv[1];
    
    // image is automatically grayscaled
    img = imread(filepath, GRAYSCALE_IMAGE);
    std::cout << "infile depth: " << img.depth() << std::endl;
    compute_ori_gradients(img, nbins, magMap, obinMap);
    desc = compute_sp_descriptor(magMap, obinMap, nlevels, nbins, thresh);

    saveToFile<float>(desc, "desc.txt");
}

/* 
 * Inputs: 
 *  img -- grayscale image
 *  nbins -- number of orientation bins
 * Outputs: 
 *  magMap -- magnitudes
 *  obinMap -- orientation bins
 */
void compute_ori_gradients(Mat img, int nbins, Mat& magMap, Mat& obinMap) {
    int sig1 = 2;
    int sig2 = 2;
    Mat Dx(img.size(), CV_32FC1);
    Mat Dy(img.size(), CV_32FC1);
    Mat magSq(img.size(), CV_32FC1);
    
    // do some smoothing
    Mat kernel_x, kernel_y;
    bool doNormalize = false;
    
    kernel_x = dgauss(sig1);
    kernel_y = dgauss(sig2).t();
    saveToFile<float>(kernel_x, "kernel_x.txt");
    saveToFile<float>(kernel_y, "kernel_y.txt");
    
    // make convolution kernels instead of correlation
    Point anchor_x(ceil(kernel_x.cols/2), 0); // old anchors
    Point anchor_y(0, ceil(kernel_y.rows/2));
    anchor_x.x = kernel_x.cols - anchor_x.x - 1; anchor_x.y = kernel_x.rows - anchor_x.y - 1; // does nothing right now, since at midpoint already
    anchor_y.x = kernel_y.cols - anchor_y.x - 1; anchor_y.y = kernel_y.rows - anchor_y.y - 1;
    flip(kernel_x, kernel_x, 1);    
    flip(kernel_y, kernel_y, 0);    
    saveToFile<float>(kernel_x, "kernel_x_conv.txt");
    saveToFile<float>(kernel_y, "kernel_y_conv.txt");
    
    Mat img2(img.size(), CV_32FC1);
    img.convertTo(img2, CV_32FC1);
    img2 = img2/255;
    saveToFile<float>(img2, "img2.txt");
    
    // smooth image.
    Mat imgsm(img.size(), CV_32FC1);
    GaussianBlur(img2, imgsm, Size(13,13), sig1, sig2);
    namedWindow("blurred", CV_WINDOW_AUTOSIZE);
    imshow("blurred", imgsm);
    saveToFile<float>(imgsm, "imgsm.txt");
    std::cout << "smoothed image depth: " << imgsm.depth() << std::endl;
    
    // get horizontal and vertical components of gradient
    filter2D(img2, Dx, Dx.depth(), kernel_x, anchor_x, 0, BORDER_REPLICATE);
    filter2D(img2, Dy, Dy.depth(), kernel_y, anchor_y, 0, BORDER_REPLICATE);
    saveToFile<float>(Dx, "Dx.txt");
    saveToFile<float>(Dy, "Dy.txt");
    
    // calculate magnitudes
    magnitude(Dx, Dy, magMap);
    saveToFile<float>(magMap, "magMap.txt");
    
    //     debug    
    namedWindow("Dx", CV_WINDOW_AUTOSIZE);
    Mat dispDx = abs(Dx);
    double mx, my, mm; 
    minMaxLoc(abs(Dx), NULL, &mx);
    imshow("Dx", dispDx/mx);    
    namedWindow("Dy", CV_WINDOW_AUTOSIZE);
    Mat dispDy = abs(Dy);
    minMaxLoc(abs(Dy), NULL, &my);
    imshow("Dy", dispDy/mx);
    minMaxLoc(magMap, NULL, &mm);
    namedWindow("mags", CV_WINDOW_AUTOSIZE);
    imshow("mags", magMap/mm);

    // calculate orientations
    obinMap.create(magMap.rows, magMap.cols, CV_32FC1);
    for (int i=0; i<magMap.rows; i++) {
        for (int j=0; j<magMap.cols; j++) {
            float o = atan2(Dy.at<float>(i,j), Dx.at<float>(i,j)); //todo: change to atan2
            float prebin = (o+PI)* (1/(2*PI)) * nbins;
            float bin = ceil(prebin)-1;
            if (bin < 0) {
                bin = 0;
            }
//             std::cout << o << " " << prebin << " " << bin << std::endl;
            obinMap.at<float>(i,j) = bin;   
        }
    }
    saveToFile<float>(obinMap, "obinMap.txt");
    
//     std::cout << "obinMap: " << std::endl;
//     printMat(obinMap);    
    double mo; 
    minMaxLoc(obinMap, NULL, &mo);
    namedWindow("obin", CV_WINDOW_AUTOSIZE);
    imshow("obin", obinMap/mo);
    return;
}

/*
 * returns the descriptor as a nx1 vector
 * 
 * magMap = gradient magnitude at each pixel
 * obinMap = orientation bin at each pixel
 * nlevels = number of spatial pyramid levels
 * nbins = number of orientation bins.
 * thresh = magnitude threshold to be considered.
 */
Mat compute_sp_descriptor(Mat magMap, Mat obinMap, int nlevels, int nbins, double thresh) {
    // first just get size of descriptor
    int len = 0;
    for (int l=0; l<nlevels; l++) {
        len += pow(4,l)*nbins;    
    }
    
    Mat desc(len, 1, CV_32FC1);
    int place = 0;
    for (int l=0; l<nlevels; l++) {
        int n = pow(2,l);
        float sy = ((float)magMap.size().height)/n;
        float sx = ((float)magMap.size().width)/n;
        float weight;
        if (l==0)
            weight = 1;
        else
            weight = pow(2, l-1);
        
        for (int iy=0; iy<n; iy++) {
            int ay = (int)(iy*sy);
            int by = (int)(iy*sy + sy)-1;
            for (int ix=0; ix<n; ix++) {
                int ax = (int)(ix*sx);
                int bx = (int)(ix*sx+sx)-1;
//                 std::cout << ay << " " << by << " " << ax << " " << bx << std::endl;
                Mat cellHist = compute_histogram(obinMap, magMap, ay, by, ax, bx, nbins, thresh);
                int lenchange = cellHist.size().height;
                cellHist = cellHist*weight;
                set_vals(desc, place, cellHist);
                place = place + lenchange;
            }
        }
    }
    return desc;
}

/* 
 * Inputs: 
 *  obinMap
 *  magMap
 *  ay,by,ax,bx
 *  nbins
 *  thresh 
 * 
 * Outputs:
 *  Returns histogram as an nx1 matrix.
 */
Mat compute_histogram(Mat& obinMap, Mat& magMap, int ay, int by, int ax, int bx, int nbins, double thresh) {
    // ay,by,ax,bx = limits of the cell in question.
    Mat hist = Mat::zeros(nbins, 1, CV_32FC1);
    
    for (int iy=ay; iy<=by; iy++) {
        for (int ix=ax; ix<=bx; ix++) {
            int ind = (int)obinMap.at<float>(iy,ix);
            if (magMap.at<float>(iy,ix) >= thresh) {
                hist.at<float>(ind, 0) = hist.at<float>(ind, 0)+1;
            }
        }
    }
//     std::cout << "histogram before normalizing: " << std::endl;
//     printMat(hist);
    
    float Z = (by-ay+1)*(bx-ax+1);
//     std::cout << "Z=" << Z<< std::endl;
    hist = hist/Z;
    return hist;   
}

/* copies mat2 to a slice of mat1 */
void set_vals(Mat& mat1, int place, Mat& mat2) { 
    int len = mat2.size().height;
    for (int i=0; i<len; i++) {
        mat1.at<float>(i+place, 0) = mat2.at<float>(i,0);   
    }
}

/* test the function dgauss */
void dgaussTest() {
    float sigmas[] = {.1, .5, 1, 1.5, 2, 3, 10};
    
    for (int i=0; i<(sizeof(sigmas)/sizeof(float)); i++) {
        float sig = sigmas[i];
        Mat dg = dgauss(sig);
        printMat(dg);
        std::cout << std::endl;
    }
}
