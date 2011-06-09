#include "capture2.h"
#include "util2.h"
#include "../utils/clock.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <time.h>
#include <iostream>
#include <string.h>
#include "StereoOnClicks.h"
#include "../slaves/messaging.h"
#include "string.h"

#define  mR_Slave1ToCam_filename "./calib-apr21/Slave1ToVisionRotation.txt"
#define  mT_Slave1ToCam_filename "./calib-apr21/Slave1ToVisionTranslation.txt"
#define  mR_Slave2ToCam_filename "./calib-apr21/Slave2ToVisionRotation.txt"
#define  mT_Slave2ToCam_filename "./calib-apr21/Slave2ToVisionTranslation.txt"
#define  mR_CamToSlave1_filename "./calib-apr21/VisionToSlave1Rotation.txt"
#define  mT_CamToSlave1_filename "./calib-apr21/VisionToSlave1Translation.txt"
#define  mR_CamToSlave2_filename "./calib-apr21/VisionToSlave2Rotation.txt"
#define  mT_CamToSlave2_filename "./calib-apr21/VisionToSlave2Translation.txt"

#define NUMSLAVES 2

using namespace std;
using namespace cv;

void connectToController(UDPSocket& _incoming, UDPSocket& _outgoing, const int VisPort, const int ControllerPort, const string ControllerIP);
void readParamsFromFile(const char* filename, Mat& mat);

int main(int argc, char** argv)
{

        //Initialize Cameras
        StereoOnClicks cams = StereoOnClicks();

        UDPSocket _incoming[2];
        UDPSocket _outgoing[2];
        connectToController(_incoming[0], _outgoing[0], VISION_IN_PORT_SLAVE1, CONTROLLER_IN_PORT_SLAVE1, CONTROLLER_IP);
        connectToController(_incoming[1], _outgoing[1], VISION_IN_PORT_SLAVE2, CONTROLLER_IN_PORT_SLAVE2, CONTROLLER_IP);

        //Get the Parameters
        Mat mR_SlaveToCam[2] = { Mat(3,3,CV_32FC1), Mat(3,3,CV_32FC1) };
        Mat mR_CamToSlave[2] = { Mat(3,3,CV_32FC1), Mat(3,3,CV_32FC1) };
        Mat mT_SlaveToCam[2] = { Mat(3,1,CV_32FC1), Mat(3,1,CV_32FC1) };
        Mat mT_CamToSlave[2] = { Mat(3,1,CV_32FC1), Mat(3,1,CV_32FC1) };


        readParamsFromFile(mR_Slave1ToCam_filename, mR_SlaveToCam[0]);
        readParamsFromFile(mR_CamToSlave1_filename, mR_CamToSlave[0]);
        readParamsFromFile(mT_Slave1ToCam_filename, mT_SlaveToCam[0]);
        readParamsFromFile(mT_CamToSlave1_filename, mT_CamToSlave[0]);
        readParamsFromFile(mR_Slave2ToCam_filename, mR_SlaveToCam[1]);
        readParamsFromFile(mR_CamToSlave2_filename, mR_CamToSlave[1]);
        readParamsFromFile(mT_Slave2ToCam_filename, mT_SlaveToCam[1]);
        readParamsFromFile(mT_CamToSlave2_filename, mT_CamToSlave[1]);

        double x, y, z;
        int i;
        while(1)
        {      

            RequestType messageTypeToCams;
            Messaging::MessageType messageType;

            int requestFrom = -1;

            //Loop until we recieve a message
            while(1)
            {

                for (i = 0; i < NUMSLAVES; i++)
                {

                    if (Messaging::receive_message(_incoming[i], Messaging::THREAD_REQUEST))
                    {
                        messageType = Messaging::THREAD_REQUEST;
                        requestFrom = i;
                        i = NUMSLAVES + 1;
                        break;
                    } else if (Messaging::receive_message(_incoming[i], Messaging::POSITION_REQUEST))
                    {
                        messageType = Messaging::POSITION_REQUEST;
                        requestFrom = i;
                        i = NUMSLAVES + 1;
                        break;
                    }
                }
               break; 
                if (requestFrom != -1)
                    break;
        
                cams.updateImages();
                //Point3f p1, p2; cams.getNewPoint(p1, Slave1Request); cams.getNewPoint(p2, Slave1Request);  printf("Point 1: (%f, %f)    Point2: (%f, %f)     norm: %f \n", p1.x, p1.y, p2.x, p2.y, norm(p2-p1));
            }

            cout << "Request from slave " << requestFrom << endl;
            printf("Request from slave %d \n", requestFrom+1);

            
            //Messaging::send_message(_outgoing, Messaging::RECEIVED);

            messageType = Messaging::POSITION_REQUEST;

            //Message Recieved - assign correct type
            if (messageType == Messaging::POSITION_REQUEST)
            {
                if (requestFrom == 0)
                {
                    messageTypeToCams = Slave1Request;
                    cout << "Position Request Recieved From Slave1" << endl;
                } else
                {
                    messageTypeToCams = Slave2Request;
                    cout << "Position Request Recieved From Slave2" << endl;
                }
                   
            }
            else if (messageType == Messaging::THREAD_REQUEST)
            {
                if (requestFrom == 0)
                {
                    messageTypeToCams = Thread1Request;
                    cout << "Thread Request Recieved From Slave1" << endl;
                } else
                {
                    messageTypeToCams = Thread2Request;
                    cout << "Thread Request Recieved From Slave2" << endl;
                }
            }
            else
            {
                cout << "Invalid Request" << endl;
                continue;
            }
           
           
            //ask cameras for point 
            Point3f pointFromCams((float)x, (float)y, (float)z);
           
            cams.getNewPoint(pointFromCams, messageTypeToCams);
            
            Mat pointCamCoords = Mat(3,1,CV_32FC1);
            pointCamCoords.at<float>(0,0) = pointFromCams.x;
            pointCamCoords.at<float>(1,0) = pointFromCams.y;
            pointCamCoords.at<float>(2,0) = pointFromCams.z;

            //Convert Point to slave coordinates
            Mat pointRobotCoords = mR_CamToSlave[requestFrom]*pointCamCoords + mT_CamToSlave[requestFrom];

            x = (double)(pointRobotCoords.at<float>(0,0));
            y = (double)(pointRobotCoords.at<float>(1,0));
            z = (double)(pointRobotCoords.at<float>(2,0));

            cout << "Pos:  " << x << " " << y << " " << z << endl;

            if (messageType == Messaging::POSITION_REQUEST)
            {
                Messaging::send_message(_outgoing[requestFrom], Messaging::POSITION_RESPONSE, x, y, z);
                cout << "Position Response Sent" << endl;
            
            }
            else if (messageType == Messaging::THREAD_REQUEST)
            {
                Messaging::send_message(_outgoing[requestFrom], Messaging::THREAD_RESPONSE, x, y, z);
                cout << "Thread Response Sent" << endl;
            }

            //now, we go back to the top of the loop and wait...

        }
        
}



void connectToController(UDPSocket& _incoming, UDPSocket& _outgoing, const int VisPort, const int ControllerPort, const string ControllerIP)
{
        _incoming = UDPSocket();
        _outgoing = UDPSocket();

        if(!_incoming.create())
            std::cout << "error creating server socket" << std::endl;
        if(!_incoming.bind(VisPort))
            std::cout << "error binding to port " << VisPort << std::endl;

        if(!_outgoing.create())
            std::cout << "error creating server socket" << std::endl;
        if(!_outgoing.bind(ControllerPort))
            std::cout << "error binding to port " << ControllerPort << std::endl;

        _incoming.set_non_blocking(true);
        _incoming.set_timeout(0);             // using recv()

        _outgoing.set_non_blocking(true);
        _outgoing.set_timeout(0);
        _outgoing.setDestination(ControllerIP, ControllerPort);



}



void readParamsFromFile(const char* filename, Mat& mat)
{
    //Assumes a Mat of floats
    std::ifstream in;
    in.open(filename);

    float* mat_array = (float*)mat.data;
    for (int i=0; i < mat.rows*mat.cols; i++)
    {
        in >> mat_array[i];
    }

    in.close();

}
