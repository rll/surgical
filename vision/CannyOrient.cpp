#include "CannyOrient.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

#include <iostream>
#include <stack>

using namespace cv;
using namespace std;



CannyOrient::CannyOrient(float width_, float edge_sigma_, float blur_sigma_)
{
  width = width_;
  edge_sigma = edge_sigma_;
  blur_sigma = blur_sigma_;


  // Parameters
  int scale = 1;
  int length = 4;

  //orientations_Degrees = new double [6];
  orientations_Degrees[0] = 0.0;
  orientations_Degrees[1] = 30.0;
  orientations_Degrees[2] = 60.0;
  orientations_Degrees[3] = 90.0;
  orientations_Degrees[4] = 120.0;
  orientations_Degrees[5] = 150.0;
  //orientations = new double [6];

  float mus[2] = {-width, width};
  float signs[2] = {1.0, -1.0};

  int numPoints = 2*(length+1)*scale + 1;
  float pointsForGaussians [numPoints];
  for (int i=0; i < numPoints; i++)
  {
    pointsForGaussians[i] = ((float)i)/(float)scale - (float)(length+1);
  }


  //find orientations in radians
  for (int i = 0; i < NUM_ORIENTATIONS; i++)
  {
    orientations[i] = ((float)orientations_Degrees[i])*CV_PI / 180.0;
  }

  //setup gaussian blur kernel
  Mat blur_kernel = Mat(1,numPoints,CV_32FC1);
  for (int i = 0; i < blur_kernel.cols; i++)
  {
    blur_kernel.at<float>(0,i) = exp(-pow(pointsForGaussians[i],2) / (2.0*pow(blur_sigma,2)));
  }

  //setup edge kernel
  Mat edge_kernels[2];
  edge_kernels[0] = Mat(numPoints,1,CV_32FC1);
  edge_kernels[1] = Mat(numPoints,1,CV_32FC1);

  for (int dir = 0; dir <= 1; dir++)
  {
    for (int i = 0; i < edge_kernels[0].rows; i++)
    {
      edge_kernels[dir].at<float>(i,0) =  
        signs[dir]*exp(-pow(mus[dir]-pointsForGaussians[i],2) / (2.0*pow(edge_sigma,2))) * (mus[dir]-pointsForGaussians[i]);
    }
  }

  //setup the edge filters
  Mat edge_filters[2];
  edge_filters[0] = Mat(numPoints, numPoints,CV_32FC1);
  edge_filters[1] = Mat(numPoints, numPoints,CV_32FC1);

  for (int dir = 0; dir <= 1; dir++)
  {
    edge_filters[dir] = edge_kernels[dir]*blur_kernel;
  }


  //create oriented filters
  //First, create the larger filters, from which we will extract everything but the border
  Mat oriented_filters_temp [2][NUM_ORIENTATIONS];

  for (int dir=0; dir <= 1; dir++)
  {
    for (int orientation = 0; orientation < NUM_ORIENTATIONS; orientation++)
    {
      oriented_filters_temp[dir][orientation] = Mat(edge_filters[dir].size(),edge_filters[dir].type());
      rotate_filter(edge_filters[dir], oriented_filters_temp[dir][orientation], orientations[orientation]);
    }
  }

  //because we want to discard the border completely, don't just get submatrix
  //Copy data to a new matrix

  for (int dir=0; dir <= 1; dir++)
  {
    for (int orientation = 0; orientation < NUM_ORIENTATIONS; orientation++)
    {
      oriented_filters[dir][orientation] = Mat(edge_filters[dir].rows-2,edge_filters[dir].cols-2,edge_filters[dir].type());
      for (int r=0; r < edge_filters[dir].rows-2; r++)
      {
        for (int c=0; c < edge_filters[dir].cols-2; c++)
        {
          oriented_filters[dir][orientation].at<float>(r,c) = oriented_filters_temp[dir][orientation].at<float>(r+1,c+1);
        }
      }
    }
  }


/*
  for (int dir = 0; dir <= 1; dir++)
  {
    for (int orientation = 0; orientation < sizeof(orientations)/sizeof(orientations[0]); orientation++)
    {
      printf("Rotated Filter, dir: %d, orientation: %f: rows: %d  cols: %d\n", dir, orientations_Degrees[orientation], oriented_filters[dir][orientation].rows, oriented_filters[dir][orientation].cols);
      for (int r = 0; r < oriented_filters[dir][orientation].rows; r++) {
        for (int c = 0; c < oriented_filters[dir][orientation].cols; c++)
        {
          printf("%f ",oriented_filters[dir][orientation].at<float>(r,c));
        }
        printf("\n");
      }
    }
  } */ 
}

void CannyOrient::rotate_filter(const Mat &filter_src, Mat &filter_rotated, double theta)
{
  if (theta == 0)
  {
    filter_rotated = Mat(filter_src);
    return;
  }

  CvMat filter_src_cvmat = filter_src;
  CvMat filter_rotated_cvmat = filter_rotated; 

  CvMat* R = cvCreateMat(2,3,CV_32FC1);
  cvmSet(R,0,0,cos(theta));
  cvmSet(R,0,1,-sin(theta));
  cvmSet(R,0,2,(filter_src.cols-1.0)/2.0);
  cvmSet(R,1,0,sin(theta));
  cvmSet(R,1,1,cos(theta));
  cvmSet(R,1,2,(filter_src.rows-1.0)/2.0);

  cvGetQuadrangleSubPix(&filter_src_cvmat, &filter_rotated_cvmat, R);
}



void CannyOrient::CannyFollow( int r, int c, float low_thresh, float* mag, unsigned char* dst, const int ofs[][2], int rows, int cols )
{

  stack<int> threadPoints;
  threadPoints.push(r*cols+c);

  int maxVal = rows*cols;

  while (!threadPoints.empty())
  {
    int next = threadPoints.top();
    threadPoints.pop();
    dst[next] = (unsigned char)255;

    int c = next % cols;
    int r = next / cols;

    for(int i = 0; i < 8; i++ )
    {
      int c1 = c + ofs[i][0];
      int r1 = r + ofs[i][1];
      int nextCheck = r1*cols+c1;
      if ((unsigned)nextCheck < maxVal && mag[nextCheck] > low_thresh && !dst[nextCheck])
      {       
        threadPoints.push(nextCheck);
      }
    }


  }

}



//Filtering function. Assumes src and cannyMat are both CV_8UC1, and cannyAngs is CV_32FC1
void CannyOrient::filter(const Mat &src, Mat &cannyMat, Mat &cannyAngs, 
    double thresh1, double thresh2)
{
  
  float low_thresh = (float)MIN(thresh1, thresh2);
  float high_thresh = (float)MAX(thresh1, thresh2);
  Point anchor = Point(oriented_filters[0][0].rows/2, oriented_filters[0][0].cols/2);
  Mat convolved_imgs [2][NUM_ORIENTATIONS];
  int dir, orientation, r, rInd, c, i;

  //Convolve images with filters
  Mat toConv(src.size(),CV_32FC1);
  src.convertTo(toConv, CV_32FC1);
       
  for ( dir=0; dir <= 1; dir++)
  {
    for ( orientation = 0; orientation < NUM_ORIENTATIONS; orientation++)
    {
      convolved_imgs[dir][orientation] = Mat(src.size(),CV_32FC1);

      filter2D(toConv,convolved_imgs[dir][orientation], convolved_imgs[dir][orientation].depth(), oriented_filters[dir][orientation], anchor); 
    
    }
  }



  //define filter response to be the max(product of filter responses, 0)
  Mat filter_response [NUM_ORIENTATIONS];
  for ( i=0; i < NUM_ORIENTATIONS; i++)
  {
    filter_response[i] = Mat(src.size(), CV_32FC1);
    max(convolved_imgs[0][i].mul(convolved_imgs[1][i]),0.001,filter_response[i]);
  }

  //set the magnitude to be the maximum of these responses
  Mat mag = Mat(filter_response[0]);
  float* mag_array = (float*) mag.data;
  int max_response_ind [mag.rows*mag.cols];
  memset(max_response_ind, 0, mag.rows*mag.cols*sizeof(int));
  for (orientation = 1; orientation < NUM_ORIENTATIONS; orientation++)
  { 
    float* filter_response_array = (float*) filter_response[orientation].data;
    for ( i = 0; i < mag.rows*mag.cols; i++)
    {
      if (filter_response_array[i] > mag_array[i])
      {
        mag_array[i] = filter_response_array[i];
        max_response_ind[i] = orientation;
      }
    }
  }

  
  Scalar mean_scalar = mean(mag);
  //std::cout << "mean: " << mean_scalar.val[0] << std::endl;
  mag = mag/mean_scalar.val[0];
  

  //set the angle to be a weighted sum of the max response angle, and it's nearby angles
  double orientations_interp [NUM_ORIENTATIONS + 2];
  //orientations_interp[0] = orientations[0] - (orientations[1] - orientations[0]);
  orientations_interp[0] = 2*orientations[0] - orientations[1];
  //orientations_interp[NUM_ORIENTATIONS + 1] = orientations[NUM_ORIENTATIONS-1]  + (orientations[NUM_ORIENTATIONS-1] - orientations[NUM_ORIENTATIONS-2]);
  orientations_interp[NUM_ORIENTATIONS + 1] = 2*orientations[NUM_ORIENTATIONS-1] - orientations[NUM_ORIENTATIONS-2];

  for ( i = 0; i < NUM_ORIENTATIONS; i++)
  {
    orientations_interp[i+1] = orientations[i];
  }

  float* cannyAngs_array = (float*) cannyAngs.data;
  for ( r = 0; r < mag.rows; r++)
  {
    rInd = r*mag.cols;
    for ( c = 0; c < mag.cols; c++)
    {
      //indices indicating max response}

    int max_response = max_response_ind[c+r*mag.cols];

    float point_max = filter_response[max_response].at<float>(r,c);
    float point_L = filter_response[ (max_response-1)%NUM_ORIENTATIONS].at<float>(r,c);
    float point_R = filter_response[ (max_response+1)%NUM_ORIENTATIONS].at<float>(r,c);          

    float norm = point_max + point_L + point_R;
    cannyAngs_array[rInd+c] = (point_max*orientations_interp[max_response+1] 
        + point_L*orientations_interp[max_response] 
        + point_R*orientations_interp[max_response+2]
        )  / norm; 
    }
  }


  // nonmaxima suppression 
 // float max_mag = 0.0;
  for( r = 0; r < mag.rows; r++ )
  {
    rInd = r*mag.cols;
    for(  c = 0; c < mag.cols; c++ )
    {
      float magnitude = mag_array[rInd + c];

     // max_mag = max(max_mag, magnitude);

      if( magnitude <= low_thresh )
        continue;

      int r1 = 0, r2 = 0, c1 = 0, c2 = 0;
      float angle = cannyAngs_array[rInd + c];
      float magnitude1 = 0.0, magnitude2 = 0.0;


      if( angle < ORIENTATION_THRESH1)
      {
        c1 = c2 = c; r1 = r + 1; r2 = r - 1;
      }
      else if( angle < ORIENTATION_THRESH2)
      {
        r1 = r + 1; r2 = r - 1; c1 = c + 1; c2 = c - 1;
      }
      else if( angle < ORIENTATION_THRESH3) 
      {
        r1 = r2 = r; c1 = c + 1; c2 = c - 1;
      }
      else
      {
        r1 = r + 1; r2 = r - 1; c1 = c - 1; c2 = c + 1;
      }

      if( (unsigned)r1 < (unsigned)mag.rows && (unsigned)c1 < (unsigned)mag.cols )
      {
        magnitude1 = mag_array[r1*mag.cols + c1];
      }

      if( (unsigned)r2 < (unsigned)mag.rows && (unsigned)c2 < (unsigned)mag.cols )
      {
        magnitude2 = mag_array[r2*mag.cols + c2];
      }

      if( magnitude >= magnitude1 && magnitude >= magnitude2)
        ;
      else
      {
        mag_array[rInd + c] = 0.0;
      }
    }
  }

 /* std::cout << "max " << max_mag << std::endl;
  mag = mag/max_mag;
  imshow("1",mag);
  mag *= max_mag;
  waitKey(10);
*/

// hysteresis threshold
unsigned char* cannyMat_array = (unsigned char*)cannyMat.data;
memset(cannyMat_array, 0, cannyMat.rows*cannyMat.cols*sizeof(unsigned char));
const int ofs[][2] = {{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1},{0,1},{1,1}};
for( r = 0; r < mag.rows; r++ )
{
  rInd = r*mag.cols;
  for( c = 0; c < mag.cols; c++ )
  {
    if( mag_array[rInd+c] > high_thresh && !cannyMat_array[rInd+c] )
    {
      CannyFollow( r, c, low_thresh, mag_array, cannyMat_array, ofs, mag.rows, mag.cols );
    }
  }
}


/*
   float* filterArr = (float*) filter_response[0].data;

   for (int i = 5000; i < 10000; i++)
   {
   printf("%f %f \n", filter_response[0].at<float>(i/src.cols, i%src.cols), filterArr[i]);
   if (i%100 == 0)
   printf("\n");
   }
   */

}






