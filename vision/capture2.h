#ifndef CAPTURE_H
#define CAPTURE_H

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include <ctype.h>
#include <string>

#include "../utils/UavQuaternion.h"
#include "../utils/TMatrix.h"

//#include "blocked.h"
#include "util2.h"

#include <vector>
#include <iostream>
#include <fstream>

#include <unistd.h>
#include <time.h>
#include <signal.h>

#define _STDCALL
#define TRUE     0


#define PVDECL
#define _LINUX
#define _x64

#include "libPvAPI/PvApi.h"

using namespace cv;


// saturation value for the camera: between 0 (B&W) and 4096 (full saturation)
#define SATURATION 3000
// brightness value for the camera: between 0 (dark) and 255
#define BRIGHTNESS 250
// brightness value for the camera: between 0 (dark) and 4096
#define HUE 2000
// brightness value for the camera: between 0 (dark) and 680
// default: 50
#define GAIN 650

#define CH_MONO 1	// Single channel for mono image
#define CH_COLOR 3	// 3 channels for color image
#define NFRAMES 1       // number of frames to buffer
#define FRAMERATE 15

//Mode of Image Captures
enum {CONTINUOUS, SINGLEFRAME};
#define CAPTUREMODE CONTINUOUS


class Capture
{
public:
    //firewire
    //Capture(int camId, const char *camName, int gain, const char *extrinsics_version, const char* imageFiles = NULL);
    //gige
    Capture(int camId, const char *camName, int gain, const char *extrinsics_version, int uid, const char* imageFiles = NULL);
    Capture();
    ~Capture();

    // 	Reads the parameters (intrinsics, extrinsics, ...)
    void init(const char *path);

    // 	Returns the name
    const char * name(void) const { return nameInternal.c_str() ;}

    // 	returns a matrix with the camera position taken from the extrinsics
    //CvMat* cameraPosition(void) const {return mCameraPosition ; }

    const Mat cameraPosition(void) const {return mCameraPosition ; }
    const Mat extRotationVector(void) const {return mExtRotationVector;}
    const Mat cameraMatrix(void) const {return intrinsicMatrix;}

    const Mat extTranslationVector(void) const {return mExtTranslationVector;}
    const Mat rot_world2cam(void) const {return mR_world2cam;}
    const Mat rot_cam2world(void) const {return mR_cam2world;}

    // 	takes a 3d point, passes it to the ColorCentroid for display and returns its prjection on the camera frame
    void projectPointUndistorted(const Mat& point3d, Point2f& point2d);
    void projectPointUndistorted(const Mat& point3d, Point2i& point2d);
    //Point2D displayEstimatedLocation(const Point3D &);
    //Point2D getEstimatedLocationProjection(const Point3D &);

    // 	returns a pointer to the current frame
    /* IplImage * currentFrame(void) const {return mFrame;}
           IplImage * getUndistortedImage(void);
           IplImage * undistortImage(const tPvFrame* frame);*/

    const Mat& currentFrame(void) const {return mFrame;}
    Mat getUndistortedImage(void);
    void undistortImage(const tPvFrame* frame, Mat& toUndistort);
    void undistortImage(const Mat& img, Mat& toUndistort);
    double currentTimestamp(void) const { return frameTimestamp; }

    // whatever needs to handle point undistortion
    Mat ptMat;
    Mat ptUndistorted;
    Mat mRayCam;
    Mat mRayWorld;

    //CvScalar undistortPoint(const Point2D& pt) const;
    //const Mat getUndistortedRay(const Point2D& pt) const;
    void getWorldRay(const Point& pt, Mat& ray);
    void getWorldRay(const Point2f& pt, Mat& ray);

    // grabs one frame
    bool grabFrame(void);

    //grabs frame and places it in queue
    void startFrameGrab(void);
    //returns it from queue
    bool grabFrameAlreadyQueued(void);
    bool grabFrameAlreadyQueuedUndistorted(void);

    void copyImage(const tPvFrame* frame, Mat& dest) const;

    // 	tries to grab a frame, returns true if success
    bool waitForFrame(void);
    bool waitForFrameUndistorted(void);

    void startCapture(void);
    void endCapture(void);

    void syncFrameCaptureSetCenter(vector<Capture*>& otherCams); //sets this as sync out, otherCams with syncIn
    void syncFrameCaptureSetCenter();
    void setSyncIn();


    void OpenCamera(int uid);

    void setExposure(int exposure);
    void AddOtherCameraInformation(const Capture& otherCamera);
    void GetEpipolarLine(const Capture& otherCamera, const Point2f& pointSeenOtherFrame, Mat& lineParams);
    void GetEpipolarLine(const Capture& otherCamera, const Point2i& pointSeenOtherFrame, Mat& lineParams);

    bool inRange(int row, int col) {return row >= 0 & row < mFrame.rows & col >= 0 & col < mFrame.cols;}


    void drawPose(Mat& rotation_matrix, Mat& translation_vector);

    void setImageNumber(int imNum){imageNumber = imNum;};
    void subtractImageNumber(){imageNumber--;};
    int getImageNumber() { return imageNumber; };

//private:
    static int camsActive;

    int idInternal;
    bool initDone;
    std::string nameInternal;
    int mGain;
    bool mGigE;

    //	Camera  parameters;
    unsigned long   UID;
    tPvHandle       Handle;
    tPvFrame        tmpFrame;
    tPvFrame*        Frames;
    bool            Abort;
    tPvCameraInfo	cameraInfo;
    unsigned long	frameSize;
    tPvErr			Errcode;
    tPvUint32   width, height;
    bool frameReady;

    int* frameNums;
    int curFrame;
    double frameTimestamp;

    Mat   bgrImage;

    // 	Intrinsic parameters:
    double focalLength[2];
    double principalPoint[2];
    double distortion_coeffs[4];

    Mat intrinsicMatrix;
    Mat intrinsicMatrixCorrected;
    Mat distortionMatrix;

    // 	Extrinsic parameters
    Mat mExtTranslationVector;
    Mat mExtRotationVector;


    Uav::Quaternion cam_q;
    TVector3 cam_ned;

    Mat mCameraPosition;
    Mat mR_world2cam;
    Mat mR_cam2world;


    Mat mFrame;
    Mat mDistortedImage;
    Mat mUndistortedImage;
    Mat xDistortMap;
    Mat yDistortMap;
    //	CvCapture* mCapture;


    bool capturing;

    bool mFromFileFlag;
    std::ifstream mImageFilesStream;
    char* baseImageName;
    char imageNum[256];
    int imageNumber;


    const char *m_extrinsics_version;


    void WaitForCamera(unsigned int i = 1)
    {
        printf("waiting for a camera ...\n");
        while((PvCameraCount()<i) && !Abort)
            usleep(250);
        printf("\n");
    }

    struct InterCameraInfo {
        Mat OtherCameraPositionMyFrame;
        Point2f OtherCameraPositionMyPix;  //note, this pixel is is [x,y], not [row,col]
        Mat RotationToMyFrame;
    };

    vector<InterCameraInfo> otherCamParams;

};



#endif
