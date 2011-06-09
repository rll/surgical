#include "capture2.h"
#include "util2.h"
#include "../utils/clock.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <time.h>
#include <iostream>
#include <string.h>
#include <math.h>
#include "StereoOnClicks.h"
#include "../slaves/messaging.h"

using namespace std;
using namespace cv;

void runCalibration(vector<Point3d>& fromPos, vector<Point3d>& toPos, Mat& Rot, Mat& Trans);
void optimizeCalibParams(vector<Point3d>& fromPos, vector<Point3d>& toPos, Mat& Rot, Mat& Trans);
double calculateError(vector<Point3d>& fromPos, vector<Point3d>& toPos, Mat& Rot, Mat& Trans);

int main(int argc, char** argv)
{
    if (argc < 2) {
    cerr << "Usage: calibrate_robots (0 for no capture, 1 to capture slave1, 2 to capture slave2)" << endl;
    return 1;
    }

    vector<Point3d> slave1Pos, slave2Pos, slave1PosVision, slave2PosVision;
    Point3d slave, slave1, slave2, cam1, cam2;
    int slaveToCalib = atoi(argv[1]);
    if (slaveToCalib >= 1 && slaveToCalib <= 2)
    {       
        StereoOnClicks cams = StereoOnClicks();
        ofstream file;
        char filename[256];
        sprintf(filename, "robot_calib_params_slave%d.txt", slaveToCalib);
        cout << "outputting to file " << filename << endl;

        file.open(filename);
          
        Point3f point;

        /****************************************************
        //   Initialize Socket Connection with Controller
        *****************************************************/
        UDPSocket _incoming = UDPSocket();
        UDPSocket _outgoing = UDPSocket();

        if(!_incoming.create())
            std::cout << "error creating server socket" << std::endl;
        if(!_incoming.bind(VISION_IN_PORT))
            std::cout << "error binding to port " << VISION_IN_PORT << std::endl;

        if(!_outgoing.create())
            std::cout << "error creating server socket" << std::endl;
        if(!_outgoing.bind(CONTROLLER_IN_PORT))
            std::cout << "error binding to port " << CONTROLLER_IN_PORT << std::endl;

        _incoming.set_non_blocking(true);
        _incoming.set_timeout(0);             // using recv()

        _outgoing.set_non_blocking(true);
        _outgoing.set_timeout(0);
        _outgoing.setDestination(CONTROLLER_IP, CONTROLLER_IN_PORT);


        double x1, y1, z1, x2, y2, z2;

        while(1)
        {      
            
            while (!Messaging::receive_message(_incoming, Messaging::CALIBRATION_REQUEST, &x1, &y1, &z1, &x2, &y2, &z2))
            {
                cams.updateImages(false, true);
            }
            Messaging::send_message(_outgoing, Messaging::RECEIVED);
            
            if (slaveToCalib == 1)
                slave = Point3d((double)x1, (double)y1, (double)z1);
            else
                slave = Point3d((double)x2, (double)y2, (double)z2);
           

            //if (slaveToCalib == 1) 
            //{   

                cams.updateImages(true, true);  //grab new images once more
                cams.getNewPoint_NoImageUpdate(point);
                cout << "point from cam: " << point << endl;
            //} else {
            //    cams.getNewPoint(point);
            //    cout << "point2: " << point << endl;
            //}

            //cout << "point2: " << point2 << endl;
            //cout << "norm: " << norm(point2-point1) << endl;

            file << slave << "      " << point << endl; 
            
        }

        file.close();

    }
    else
    {
    //Read arguments from file

   
        ifstream inputFile_slave1, inputFile_slave2;
        string line;
        inputFile_slave1.open("robot_calib_params_slave1.txt");
        inputFile_slave2.open("robot_calib_params_slave2.txt");
        double slave1x, slave1y, slave1z, slave2x, slave2y, slave2z, cam1x, cam1y, cam1z, cam2x, cam2y, cam2z;
        while(! inputFile_slave1.eof())
        {
            getline(inputFile_slave1, line);
            if (line.size() < 6)
                continue;

            char* cstr = new char [line.size()+1];
            strcpy (cstr, line.c_str());  
            
            char* numString = strtok (cstr," ");

            slave1x = atof(numString);
            numString = strtok (NULL, " ");

            slave1y = atof(numString);
            numString = strtok (NULL, " ");
            
            slave1z = atof(numString);
            numString = strtok (NULL, " ");
    
            cam1x = atof(numString);
            numString = strtok (NULL, " ");

            cam1y = atof(numString);
            numString = strtok (NULL, " ");
            
            cam1z = atof(numString);
            numString = strtok (NULL, " ");
   
            slave1 = Point3d((double)slave1x, (double)slave1y, (double)slave1z);
            cam1 = Point3d((double)cam1x, (double)cam1y, (double)cam1z);

            slave1Pos.push_back(slave1);
            slave1PosVision.push_back(cam1);
        }
    

        
        while(! inputFile_slave2.eof())
        {
            getline(inputFile_slave2, line);
            if (line.size() < 6)
                continue;
            
            char* cstr = new char [line.size()+1];
            strcpy (cstr, line.c_str());  
            
            char* numString = strtok (cstr," ");

            slave2x = atof(numString);
            numString = strtok (NULL, " ");

            slave2y = atof(numString);
            numString = strtok (NULL, " ");

            slave2z = atof(numString);
            numString = strtok (NULL, " ");

            cam2x = atof(numString);
            numString = strtok (NULL, " ");

            cam2y = atof(numString);
            numString = strtok (NULL, " ");

            cam2z = atof(numString);
            numString = strtok (NULL, " ");


            slave2 = Point3d((double)slave2x, (double)slave2y, (double)slave2z);
            cam2 = Point3d((double)cam2x, (double)cam2y, (double)cam2z);

            slave2Pos.push_back(slave2);
            slave2PosVision.push_back(cam2);
        }
    }
    
    Mat slave1ToVisRot = Mat(3,3,CV_32FC1);
    Mat slave1ToVisTrans = Mat(3,1,CV_32FC1);
    Mat slave2ToVisRot = Mat(3,3,CV_32FC1);
    Mat slave2ToVisTrans = Mat(3,1,CV_32FC1);
    runCalibration(slave1Pos, slave1PosVision, slave1ToVisRot, slave1ToVisTrans);
    runCalibration(slave2Pos, slave2PosVision, slave2ToVisRot, slave2ToVisTrans);
    
    ofstream slave1RotFile, slave1TransFile, slave2RotFile, slave2TransFile;
    slave1RotFile.open("Slave1ToVisionRotation.txt");
    slave1TransFile.open("Slave1ToVisionTranslation.txt");
    slave2RotFile.open("Slave2ToVisionRotation.txt");
    slave2TransFile.open("Slave2ToVisionTranslation.txt");

    Mat visToSlave1Rot = Mat(3,3,CV_32FC1);
    Mat visToSlave1Trans = Mat(3,1,CV_32FC1);
    Mat visToSlave2Rot = Mat(3,3,CV_32FC1);
    Mat visToSlave2Trans = Mat(3,1,CV_32FC1);
    runCalibration(slave1PosVision, slave1Pos, visToSlave1Rot, visToSlave1Trans);
    runCalibration(slave2PosVision, slave2Pos, visToSlave2Rot, visToSlave2Trans);

    slave1ToVisRot = visToSlave1Rot.t();
    slave1ToVisTrans = slave1ToVisRot*visToSlave1Trans;
    for (int i = 0; i < slave1ToVisTrans.rows; i++)
        slave1ToVisTrans.at<float>(i,0) = -1*slave1ToVisTrans.at<float>(i,0);
    slave2ToVisRot = visToSlave2Rot.t();
    slave2ToVisTrans = slave2ToVisRot*visToSlave2Trans;
    for (int i = 0; i < slave2ToVisTrans.rows; i++)
        slave2ToVisTrans.at<float>(i,0) = -1*slave2ToVisTrans.at<float>(i,0);

    ofstream vision1RotFile, vision1TransFile, vision2RotFile, vision2TransFile;
    vision1RotFile.open("VisionToSlave1Rotation.txt");
    vision1TransFile.open("VisionToSlave1Translation.txt");
    vision2RotFile.open("VisionToSlave2Rotation.txt");
    vision2TransFile.open("VisionToSlave2Translation.txt");
    
    cout << visToSlave1Rot << endl;
    cout << visToSlave1Trans << endl;
    cout << slave1ToVisTrans << endl;
    cout << slave1ToVisRot << endl;

    slave1RotFile << slave1ToVisRot;
    slave1TransFile << slave1ToVisTrans;
    slave2RotFile  << slave2ToVisRot;    
    slave2TransFile << slave2ToVisTrans;
    vision1RotFile <<   visToSlave1Rot;
    vision1TransFile << visToSlave1Trans;
    vision2RotFile  <<  visToSlave2Rot;    
    vision2TransFile << visToSlave2Trans;
}

void runCalibration(vector<Point3d>& fromPos, vector<Point3d>& toPos, Mat& Rot, Mat& Trans)
{
    //Least squares doesn't seem to work with doubles, so use floats
    //Mat A = Mat::zeros(15 /*3*fromPos.size()*/, 12, CV_32FC1);
    //Mat b = Mat::zeros(15 /*3*fromPos.size()*/, 1, CV_32FC1);
    Mat A = Mat::zeros(3*fromPos.size(), 12, CV_32FC1);
    Mat b = Mat::zeros(3*fromPos.size(), 1, CV_32FC1);

    for (int i = 0; i < fromPos.size(); i++)
    //for (int i = 0; i < 5; i++)
    {        
        A.at<float>(3*i, 0) = A.at<float>(3*i+1, 3) = A.at<float>(3*i+2, 6) = (float)fromPos[i].x;
        A.at<float>(3*i, 1) = A.at<float>(3*i+1, 4) = A.at<float>(3*i+2, 7) = (float)fromPos[i].y;
        A.at<float>(3*i, 2) = A.at<float>(3*i+1, 5) = A.at<float>(3*i+2, 8) = (float)fromPos[i].z;
        A.at<float>(3*i, 9) = A.at<float>(3*i+1, 10) = A.at<float>(3*i+2, 11) = 1.0;

        b.at<float>(3*i,0) = (float) toPos[i].x;       
        b.at<float>(3*i+1,0) = (float)toPos[i].y;       
        b.at<float>(3*i+2,0) = (float)toPos[i].z;       
    }

    Mat sol = Mat(12,1, CV_32FC1);
    sol = (A.t()*A).inv()*(A.t()*b);

    Trans.at<float>(0,0) = sol.at<float>(9,0);
    Trans.at<float>(1,0) = sol.at<float>(10,0);
    Trans.at<float>(2,0) = sol.at<float>(11,0);

    Mat RInvalid = Mat(3,3,CV_32FC1);
    RInvalid.at<float>(0,0) = sol.at<float>(0,0);
    RInvalid.at<float>(0,1) = sol.at<float>(1,0);
    RInvalid.at<float>(0,2) = sol.at<float>(2,0);
    RInvalid.at<float>(1,0) = sol.at<float>(3,0);
    RInvalid.at<float>(1,1) = sol.at<float>(4,0);
    RInvalid.at<float>(1,2) = sol.at<float>(5,0);
    RInvalid.at<float>(2,0) = sol.at<float>(6,0);
    RInvalid.at<float>(2,1) = sol.at<float>(7,0);
    RInvalid.at<float>(2,2) = sol.at<float>(8,0);


    //SVD svd = SVD(RInvalid);  Rot = (svd.u * svd.vt);

    Rot = RInvalid.clone();

    optimizeCalibParams(fromPos, toPos, Rot, Trans); 




}



void optimizeCalibParams(vector<Point3d>& fromPos, vector<Point3d>& toPos, Mat& Rot, Mat& Trans)
{

    cout << "Error Before Optimization: " << calculateError(fromPos, toPos, Rot, Trans) << endl;

    for (int numItersOuter = 0; numItersOuter < 15; numItersOuter++) {
        float scale = 1/(1 << numItersOuter);
        for (int numItersInner = 0; numItersInner < 500; numItersInner++) {
            for (int iterType = 0; iterType < 6; iterType++) {
                for (int dir = -1; dir <= 1; dir+=2)
                {          
                    float theta = 0.0001*dir*scale;
                    float dx = 0.1*dir*scale;
                    Mat RotAlter = Mat::eye(3,3,CV_32FC1);
                    Mat TransAlter = Mat::zeros(3,1,CV_32FC1);
                    
                    switch (iterType)
                    {
                        case 0:
                            RotAlter.at<float>(1,1) = cos(theta);
                            RotAlter.at<float>(2,1) = sin(theta);
                            RotAlter.at<float>(1,2) = -sin(theta);
                            RotAlter.at<float>(2,2) = cos(theta);
                            break;
                        case 1:
                            RotAlter.at<float>(0,0) = cos(theta);
                            RotAlter.at<float>(0,2) = sin(theta);
                            RotAlter.at<float>(2,0) = -sin(theta);
                            RotAlter.at<float>(2,2) = cos(theta);
                            break;
                        case 2:
                            RotAlter.at<float>(0,0) = cos(theta);
                            RotAlter.at<float>(1,0) = sin(theta);
                            RotAlter.at<float>(0,1) = -sin(theta);
                            RotAlter.at<float>(1,1) = cos(theta);
                            break;
                        case 3:
                            TransAlter.at<float>(0,0) = dx;
                            break;
                        case 4:
                            TransAlter.at<float>(1,0) = dx; 
                            break;
                        case 5:
                            TransAlter.at<float>(2,0) = dx; 
                            break;
                    }       

                    float origError = calculateError(fromPos, toPos, Rot, Trans);
                    Mat newRot = RotAlter*Rot;
                    Mat newTrans = TransAlter + Trans;
                    float newError = calculateError(fromPos, toPos, newRot, newTrans);  


                    if (newError < origError)
                    {
                        Rot = newRot;
                        Trans = newTrans;
                    }

                }             
            }
        }
    }

    cout << "Error After Optimization: " << calculateError(fromPos, toPos, Rot, Trans) << endl;


}

double calculateError(vector<Point3d>& fromPos, vector<Point3d>& toPos, Mat& Rot, Mat& Trans)
{
    double Err = 0;
    for (int i=0; i < fromPos.size(); i++)
    {
        Mat fromMat = Mat(3,1,CV_32FC1);
        fromMat.at<float>(0,0) = fromPos[i].x;
        fromMat.at<float>(1,0) = fromPos[i].y;
        fromMat.at<float>(2,0) = fromPos[i].z;
        
        Mat toMat = Mat(3,1,CV_32FC1);
        toMat.at<float>(0,0) = toPos[i].x;
        toMat.at<float>(1,0) = toPos[i].y;
        toMat.at<float>(2,0) = toPos[i].z;

        Mat toMatPredicted = Rot*fromMat+Trans;

        Err += (double)norm(toMatPredicted - toMat);
    }

    Err = Err / fromPos.size();
   
    return Err; 
}
