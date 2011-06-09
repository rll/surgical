
#include <math.h>
#include "TMatrix.h"

void ransac_plane(TVector3 inputPoints[], int numPoints, TVector4 &planeCoeffs, TVector3 inliers[], int &numInliers);
int test_plane();

int test_plane() { //test ransac_plane
  float a = 1;
  float b = 2;
  float c = 3;
  float d = 4;

  int numInliers = 100;
  int numOutliers = 10;
  TVector3 points[numInliers + numOutliers];
  int pointsIndex=0;
  
  //generate inliers, with noise 
  for (int i=0; i<numInliers;i++) {
    // generate random x and y from 0 to 10
    float x = rand() % 10;
    float y = rand() % 10;
    // find z, add noise
    float z = (-a*x - b*y - d)/c + (rand() % 100)/100;
    points[pointsIndex++] = TVector3(x,y,z);
    //std::cout << points[pointsIndex-1].transpose();
  }

  for (int i=0; i<numOutliers; i++) {
    points[pointsIndex++] = TVector3(rand()%10, rand()%10, rand()%10);
  }


  // run function
  //  TVector3 inputPoints[7] = {TVector3(1,0,0),TVector3(0,1,0),TVector3(0,0,1), TVector3(1.000, 0.001, 0), TVector3(0.707, 0.707, 0), TVector3(0, 0.707, 0.707), TVector3(0.707, 0, 0.707)};
  //  TVector3 inputPoints[7] = {TVector3(1,0,0),TVector3(0,1,0),TVector3(0,0,1), TVector3(1.000, 0.001, 0), TVector3(0.707, 0.707, 0), TVector3(2, 500, 0), TVector3(1,1,0.0001)};
  TVector4 output_planeCoeffs;
  TVector3 output_inliers[pointsIndex];
  int output_numInliers;
  ransac_plane(points, pointsIndex, output_planeCoeffs, output_inliers, output_numInliers);

  std::cout << "abcd found by ransac: " << output_planeCoeffs.transpose();
  std::cout << "num inliers found by ransac: " << output_numInliers << std::endl;
  
}

void ransac_plane(TVector3 inputPoints[], int numPoints, TVector4 &planeCoeffs, TVector3 inliers[], int &numInliers) {
  TVector3 bestPlaneNormal;
  TVector3 bestP0;
  float bestScore = -99999;
  int numIters = 100;
  for (int i = 0; i<numIters; i++) {
    //std::cout  << std::endl;
    
    //sample 3 points
    int ind0 = rand() % numPoints;
    int ind1 = rand() % numPoints;
    int ind2 = rand() % numPoints;
    //std::cout << "indices: " << ind0 << " " << ind1 << " " << ind2 << std::endl;
    if ((ind0==ind1) || (ind1==ind2) || (ind2==ind0)) {
      //std::cout << "Throwing out this iteration." << std::endl;
      continue;
    }
	
    TVector3 p0 = inputPoints[ind0];
    TVector3 p1 = inputPoints[ind1];
    TVector3 p2 = inputPoints[ind2];
    //std::cout << "Points: " << std::endl << p0.transpose() << p1.transpose() << p2.transpose();

    // fit plane
    //std::cout << "plane vectors: " << (p1-p0).transpose() << ", " << (p2-p0).transpose();
    TVector3 planeNormal = (p1-p0).cross(p2-p0);
    //    planeNormal = planeNormal/planeNormal.norm();
    //std::cout << "planeNormal: " << planeNormal.transpose();

    // get inliers
    float minDist = 0.001;
    TVector3 currInliers[numPoints];
    int currNumInliers = 0;
    
    for (int j=0; j<numPoints; j++) {
      TVector3 candidatePoint = inputPoints[j];
      //std::cout << "candidatePoint" << candidatePoint.transpose();
      TVector3 tmpVec = candidatePoint - p0;
      //std::cout << "tmpVec" << tmpVec.transpose();
      float distToPlane = fabs(tmpVec.dot(planeNormal)/planeNormal.norm());
      //std::cout << "distToPlane" << distToPlane << std::endl;
      
      if (distToPlane < minDist) {
	//std::cout << "found inlier number " << currNumInliers+1 << std::endl;
	currInliers[currNumInliers] = candidatePoint;
	currNumInliers++;
      }
    }
    //    std::cout << "Final number of inliers: " << currNumInliers << std::endl;

    // set if max
    float score = currNumInliers;
    if (score > bestScore) {
      bestPlaneNormal = planeNormal;
      bestP0 = p0;
      bestScore = score;
      inliers = currInliers;
      numInliers = currNumInliers;
    }
  }

  std::cout << "bestScore = " << bestScore << std::endl;
  // return planeCoeffs abcd, where ax+by+cz+d = 0;
  float d = -bestPlaneNormal.dot(bestP0);
  planeCoeffs.setSubMatrix<0,0>(bestPlaneNormal);
  planeCoeffs[3] = d;
  std::cout << "Coefficients: " << planeCoeffs.transpose();
}
 
