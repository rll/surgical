#include "checkerboard_data_vis.h"

int main(int argc, char** argv)
{
  Checkerboard checkerData;
  checkerData.initCams();
  while (1)
  {
    if (checkerData.findCheckerboard())
    {
      checkerData.drawCheckerboard();
      checkerData.calculate3dPoints();


      Mat startRotCheck;
      Mat startTransCheck;
      checkerData.getEstimated3dPose(startRotCheck, startTransCheck);


      Mat startRotCheckCopy = startRotCheck.clone();
      Mat temp;
      /*temp = startRotCheck.col(2);
        startRotCheckCopy.col(0).copyTo(temp);
      //temp = startRotCheck.col(1);
      //startRotCheckCopy.col(1).copyTo(temp);
      startRotCheck.col(1) *= -1;
      temp = startRotCheck.col(0);
      startRotCheckCopy.col(2).copyTo(temp);
      */

      //startTransCheck += startRotCheck.col(1)*-15.0 + startRotCheck.col(0)*((CHECKERS_PER_COL-1)/2.0)*SIZE_EACH_CHECKER;

      Matrix3d startRot = Matrix3d::Identity();
      for (int r=0; r < 3; r++)
      {
        for (int c=0; c < 3; c++)
        {
          startRot(r,c) = startRotCheck.at<float>(r,c);
        }
      }
      Vector3d startPos;
      startPos(0) = startTransCheck.at<float>(0,0);
      startPos(1) = startTransCheck.at<float>(1,0);
      startPos(2) = startTransCheck.at<float>(2,0);
      std::cout << startPos << std::endl;
      std::cout << startRot << std::endl;
      //   startPos += startRot.block(0,0,3,1)*15.0 + startRot.block(0,1,3,1)*((CHECKERS_PER_COL-1)/2.0)*SIZE_EACH_CHECKER;
      Vector3d offsetGuess = Vector3d::Zero();
      offsetGuess(1) = 0.0;
      offsetGuess(0) = 0.0; //-((CHECKERS_PER_COL-1)/2.0)*SIZE_EACH_CHECKER;
      double offsetAngGuess = 0;

      vector<Vector3d> checkerPtsRobotHand;
      checkerData.estimatePointsInRobotHand(startPos, startRot, offsetGuess, offsetAngGuess, checkerPtsRobotHand);

      for (int camNum = 0; camNum < NUMCAMS; camNum++)
      {
        checkerData._captures[camNum]->drawPose(startRotCheck, startTransCheck);
      }





      checkerData.drawCheckerboard();


    }
    checkerData.displayIms();
  }
}

