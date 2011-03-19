#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <stdio.h>

using namespace cv;

class CannyOrient
{
public:
    CannyOrient(float width_, float edge_sigma_, float blur_sigma_);
    void filter(const Mat& src, Mat& cannyMat, Mat& cannyAngs,
                    double thresh1, double thresh2);

private:
    
  #define NUM_ORIENTATIONS 6
	#define ORIENTATION_THRESH1 CV_PI/8
	#define ORIENTATION_THRESH2 3*CV_PI/8
	#define ORIENTATION_THRESH3 5*CV_PI/8
	#define ORIENTATION_THRESH4 7*CV_PI/8
  double orientations_Degrees [NUM_ORIENTATIONS];
  double orientations [NUM_ORIENTATIONS];
  
  //vector<vector<Mat> > oriented_filters;
  Mat oriented_filters [2][NUM_ORIENTATIONS];
  float width, edge_sigma, blur_sigma;

  void rotate_filter(const Mat& filter_src, Mat& filter_rotated, double theta); //Rotates filter using bilinear interpolation
	void CannyFollow( int r, int c, float low_thresh, float* mag, unsigned char* dst, const int ofs[][2], int rows, int cols ); //for hystersis

};
