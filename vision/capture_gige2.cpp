#include "capture2.h"
#include "util2.h"
#include "../utils/clock.h"

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <time.h>

using namespace std;
using std::cout;
using std::endl;
using std::cerr;
using namespace cv;

int Capture::camsActive = 0;

void FrameCB0(tPvFrame* frame) {
    bool* rdyflag = (bool*)frame->Context[0];
    tPvHandle* handle = (tPvHandle*)frame->Context[1];

    int* frameNum = (int*)frame->Context[2];
    int* nextFrame = (int*)frame->Context[3];

    (*nextFrame) = (*frameNum);
    //cout << "handling frame: "<< *frameNum << " and setting: " << nextFrame << endl;

    (*rdyflag) = true;
    PvCaptureQueueFrame(*handle, frame, FrameCB0);
}

void Capture::init(const char *path)
{
    //debugThis("begin init");
    cout << nameInternal << endl;
    cout << __FILE__ << "\t line " << __LINE__ << " : " << "initializing intrinsics...";
    if(path == 0)
    {
        cerr << endl << "Can not perform intrinsics calibration, no path available for intrinsics files." << endl;
        initDone = false;
        exit(1);
    }

    char intrinsics_filename[256];

    sprintf(intrinsics_filename, "%sintrinsics.basic_params.%s.txt", path, nameInternal.c_str());

    std::ifstream in;
    in.open(intrinsics_filename);
    if(in.fail())
    {
        cerr << __FILE__ << "\t line " << __LINE__ << " : " << "Could not open intrinsics file: " << intrinsics_filename << std::endl;
        cerr << __FILE__ << "\t line " << __LINE__ << " : " << "Initialization failed for " << nameInternal << std::endl;
        initDone = false;
        exit(1);
    }

    in >> focalLength[0] >> focalLength[1] >> principalPoint[0] >> principalPoint[1] >> distortion_coeffs[0] >> distortion_coeffs[1] >> distortion_coeffs[2] >> distortion_coeffs[3];


    in.close();


    // intrinsic matrix
    intrinsicMatrix.at<float>(2, 2) = 1.0;
    intrinsicMatrix.at<float>(0, 0) = focalLength[0];
    intrinsicMatrix.at<float>(1, 1) = focalLength[1];
    intrinsicMatrix.at<float>(0, 2) = principalPoint[0];
    intrinsicMatrix.at<float>(1, 2) = principalPoint[1];
    distortionMatrix.at<float>(0, 0) = distortion_coeffs[0];
    distortionMatrix.at<float>(0, 1) = distortion_coeffs[1];
    distortionMatrix.at<float>(0, 2) = distortion_coeffs[2];
    distortionMatrix.at<float>(0, 3) = distortion_coeffs[3];

    
    //   CvMat* xtmp = cvCreateMat(width, height, CV_32FC1);
    //   CvMat* ytmp = cvCreateMat(width, height, CV_32FC1);
    //   cvInitUndistortMap(intrinsicMatrix, distortionMatrix, xtmp, ytmp);
    //   xDistortMap = cvCreateMat(width*3, height, CV_32FC3);
    //   yDistortMap = cvCreateMat(width*3, height, CV_32FC3);
    //   for(unsigned int i = 0; i < width; i++) {
    //     for(unsigned int j= 0; j < height; j++) {
    //       cvSet2D(xDistortMap, i,j, cvScalarAll(cvGetReal2D(xtmp, i, j)));
    //       cvSet2D(yDistortMap, i,j, cvScalarAll(cvGetReal2D(ytmp, i, j)));
    //     }
    //   }
    xDistortMap = Mat(width, height, CV_32FC1);
    yDistortMap = Mat(width, height, CV_32FC1);
    intrinsicMatrixCorrected = Mat(intrinsicMatrix);
    initUndistortRectifyMap(intrinsicMatrix, distortionMatrix,  Mat::eye(3, 3, CV_32FC1), intrinsicMatrixCorrected, Size(width,height), CV_32FC1, xDistortMap, yDistortMap);

    cout << "done" << endl;


    cout << __FILE__ << "\t line " << __LINE__ << " : initializing extrinsics...";


    char extrinsic_filename[256];



    sprintf(extrinsic_filename, "%sextrinsics.rotationVector.%s.%s.txt", path, nameInternal.c_str(), m_extrinsics_version);
    in.open(extrinsic_filename);

    if(in.fail())
    {
        cerr << __FILE__ << "\t line " << __LINE__ << " : cannot open: " << extrinsic_filename << endl;
        cerr << __FILE__ << "\t line " << __LINE__ << " : " << "Initialization failed for " << nameInternal << endl;
        initDone = false;
        exit(1);
    }

    in >> mExtRotationVector;
    in.close();

    sprintf(extrinsic_filename, "%sextrinsics.translationVector.%s.%s.txt", path, nameInternal.c_str(), m_extrinsics_version);
    in.open(extrinsic_filename);

    if(in.fail())
    {
        cerr << __FILE__ << "\t line " << __LINE__ << " : cannot open: " << extrinsic_filename << endl;
        cerr << __FILE__ << "\t line " << __LINE__ << " : " << "Initialization failed for " << nameInternal << endl;
        initDone = false;
        exit(1);
    }

    in >> mExtTranslationVector;

    in.close();
    cout << "done" << endl;

    cerr << "ExtPositionVector: " << mExtTranslationVector.at<float>(0, 0) << " "
        << mExtTranslationVector.at<float>(1, 0) << " "
        << mExtTranslationVector.at<float>(2, 0) << endl;

    cerr << "ExtRotationVector: " << mExtRotationVector.at<float>(0, 0) << " "
        << mExtRotationVector.at<float>(1, 0) << " "
        << mExtRotationVector.at<float>(2, 0) << endl;

    Rodrigues(mExtRotationVector, mR_world2cam);
    // camera position in world = - R'*offset;  R converts: worldframe -> cameraframe  coordinates.
    transpose(mR_world2cam, mR_cam2world);

    mCameraPosition = mR_cam2world*mExtTranslationVector;

    for (int i=0; i<3; ++i)
        mCameraPosition.at<float>(i, 0) = -mCameraPosition.at<float>(i, 0);





    initDone = true;
}

void Capture::setExposure(int exposure) {
    PvAttrUint32Set(Handle,"ExposureValue",exposure);
}

void Capture::OpenCamera(int uid) {
    // handle constructor / deconstructor initializing pvapi
    int err;
    camsActive++;
    if(camsActive == 1){
        if((err = PvInitialize())){
            cout << __FILE__ << "\t line " << __LINE__ << " : PvApi is not initialized, err = " << err <<  endl;
            exit(1);
        }
    }

    Abort = false;

    WaitForCamera();

    tPvUint32 count,connected;
    tPvCameraInfo list[10];

    bool done = false;
    bool failed = true;
    unsigned int i;
    while (!done) {
        count = PvCameraList(list,10,&connected);
        cout << "count = " << count << endl;


        for(i=0; i<count; i++){
            if(list[i].UniqueId == uid) {
                cout << "UID = " << list[i].UniqueId << "cam uid = " << uid << endl;
                UID = list[i].UniqueId;
                if(PvCameraOpen(UID,ePvAccessMaster,&(Handle))){
                    cout << __FILE__ << "\t line " << __LINE__ << " : Camera could not be opened" << endl;
                }
                else {
                    done = true;
                    failed = false;
                    break;
                }
            }
        }
        usleep(50000);
    }
    if(failed){
        cout << __FILE__ << "\t line " << __LINE__ << " : Camera could not be grabbed" << endl;
        exit(1);
    }
    printf("grabbed camera %s\n",list[i].SerialString);


    /******************
     * Camera settings
     ******************/

    tPvUint32 lMaxSize = 8228;
    // adjust the packet size according to the current network capacity
    PvCaptureAdjustPacketSize(Handle,lMaxSize);
    // get the last packet size set on the camera
    PvAttrUint32Get(Handle,"PacketSize",&lMaxSize);
    cout << "Max packet size: " << lMaxSize << endl;

    // cam1 209 135 cam2 199 143
    tPvUint32 blueValue = 300;
    tPvUint32 redValue = 95;
    PvAttrEnumSet(Handle,"PixelFormat","Bgr24");
    PvAttrUint32Set(Handle,"WhitebalValueBlue",blueValue);
    PvAttrUint32Set(Handle,"WhitebalValueRed",redValue);

    //   PvAttrEnumSet(Handle,"WhitebalMode","Auto");
    //   PvAttrEnumSet(Handle,"ExposureMode","Auto");
    PvAttrEnumSet(Handle,"WhitebalMode","Manual");
    PvAttrEnumSet(Handle,"ExposureMode","Manual");

    tPvUint32 exposureValue = 8300;
    //cout << "exposureValue: " << exposureValue << endl;
    //this->setExposure(exposureValue);




    if(PvAttrUint32Get(Handle,"TotalBytesPerFrame",&frameSize)){
        cout << __FILE__ << "\t line " << __LINE__ << " : Framesize could not be acquired" << endl;
        exit(1);
    }

    if(!mFromFileFlag) {
        frameNums = new int[NFRAMES];
        for(int i = 0; i < NFRAMES; i++) {
            frameNums[i] = i;
        }

        Frames = new tPvFrame[NFRAMES];
        for(int i = 0; i < NFRAMES; i++) {
            Frames[i].ImageBuffer = new char[frameSize];
            Frames[i].ImageBufferSize = frameSize;
        }
    }

    tmpFrame.ImageBuffer = new char[frameSize];
    tmpFrame.ImageBufferSize = frameSize;

    tPvErr Errcode;

    unsigned long lCapturing;
    if(PvCaptureQuery(Handle, &lCapturing)){
        cout << __FILE__ << "\t line " << __LINE__ << " : Camera could not be queried" << endl;
        exit(1);
    }
    if(lCapturing) {
        cout << __FILE__ << "\t line " << __LINE__ << " : Camera is not idle" << endl;
        exit(1);
    }

    if(PvCaptureStart(Handle)){
        cout << __FILE__ << "\t line " << __LINE__ << " : Camera could not be started" << endl;
        exit(1);
    }


    //if (CAPTUREMODE == CONTINUOUS) {
        PvAttrEnumSet(Handle, "AcquisitionMode", "Continuous");
        PvAttrEnumSet(Handle, "FrameStartTriggerMode", "Freerun");
    /*}else if (CAPTUREMODE == SINGLEFRAME) {
        PvAttrEnumSet(Handle, "AcquisitionMode", "SingleFrame");
        PvAttrEnumSet(Handle, "FrameStartTriggerMode", "Software");
        PvAttrEnumSet(Handle, "AcqStartTriggerMode", "Disabled");
        PvAttrEnumSet(Handle, "AcqEndTriggerMode", "Disabled");
        printf("\n Single Frame \n");
    }*/
    
    //PvAttrEnumSet(Handle, "ExposureMode", "AutoOnce");

    if(PvCommandRun(Handle,"AcquisitionStart")){
        PvCaptureEnd(Handle) ;
        cout << __FILE__ << "\t line " << __LINE__ << " : Camera could not be started" << endl;
        exit(1);
    }

    PvAttrUint32Get(Handle, "Width", &width);
    PvAttrUint32Get(Handle, "Height", &height);

}



//GIGE capture constructor
    Capture::Capture(int camId, const char *camName, int gain, const char *extrinsics_version, int uid, const char* imageFiles)
: idInternal(camId),
    initDone(false),
    nameInternal(camName),
    mGain(gain),
    capturing(false)
{
    intrinsicMatrix        = Mat::zeros(3, 3, CV_32FC1);
    distortionMatrix       = Mat::zeros(1, 4, CV_32FC1);
    ptMat                  = Mat(1,1,CV_32FC2);
    ptUndistorted          = Mat(1,1,CV_32FC2);

    mRayCam = Mat(3, 1, CV_32FC1);
    mRayWorld = Mat(3, 1, CV_32FC1);
    mRayCam.at<float>(2, 0) =  1.0;

    mExtRotationVector     = Mat(3, 1, CV_32FC1);
    mExtTranslationVector  = Mat(3, 1, CV_32FC1);
    mR_world2cam           = Mat(3, 3, CV_32FC1);
    mR_cam2world           = Mat(3, 3, CV_32FC1);
    mCameraPosition        = Mat(3, 1, CV_32FC1);

    // distortion coeffs
    m_extrinsics_version = strdup(extrinsics_version);


    //mFrame = 0;

    cout << __FILE__ << "\t line " << __LINE__ << " : Creating the capture class for camera #" << idInternal << endl;

    if(imageFiles == NULL) {
        mFromFileFlag = false;


        // opening camera
        OpenCamera(uid);

        bgrImage = Mat((int)height, (int)width, CV_8UC3);
        //bgrImage->widthStep = (int)width * CH_COLOR;

        mDistortedImage = Mat((int)height, (int)width, CV_8UC3);
        //mDistortedImage->widthStep = (int)width * CH_COLOR;

        mUndistortedImage = Mat((int)height, (int)width, CV_8UC3);
        //mUndistortedImage->widthStep = (int)width * CH_COLOR;

        // cvSetCaptureProperty(mCapture, CV_CAP_PROP_FRAME_WIDTH,	640);
        // cvSetCaptureProperty(mCapture, CV_CAP_PROP_FRAME_HEIGHT, 480);
        // cvSetCaptureProperty(mCapture, CV_CAP_PROP_FPS,					20);
        // cvSetCaptureProperty(mCapture, CV_CAP_PROP_SATURATION,	SATURATION);
        // cvSetCaptureProperty(mCapture, CV_CAP_PROP_BRIGHTNESS,	BRIGHTNESS);
        // cvSetCaptureProperty(mCapture, CV_CAP_PROP_GAIN,				mGain);
        // cvSetCaptureProperty(mCapture, CV_CAP_PROP_HUE,					HUE);
        //cout << __FILE__ << "\t line " << __LINE__ << " brightness:" << cvGetCaptureProperty(mCapture, CV_CAP_PROP_BRIGHTNESS)   << endl;  // diff for gigE
        cout << __FILE__ << "\t line " << __LINE__ << " width :"     << width  << endl;
        cout << __FILE__ << "\t line " << __LINE__ << " height :"    << height << endl;
    }
    else {
        mFromFileFlag = true;
        baseImageName = new char[256];
        strcpy(baseImageName, imageFiles);
        //baseImageName = imageFiles;
        std::cout << baseImageName << std::endl;
        imageNumber = 1;
    }

    // always grab one frame at construction for consistency
    grabFrame();
		imageNumber=1;

    cout << "Capture Device initialized. mCamId = " << camId << ", camName = " << nameInternal << std::endl;
}


Capture::~Capture()
{
    cout << "Destroying the capture class for camera #" << idInternal << "(" << nameInternal << ")" << endl;
    endCapture();

    PvCommandRun(Handle,"AcquisitionStop");
    PvCaptureEnd(Handle);
    PvCaptureQueueClear(Handle);
    // then close the camera
    printf("closing the camera\n");
    PvCameraClose(Handle);

    // delete all the allocated buffers
    if(!mFromFileFlag) {
        for(int i = 0; i < NFRAMES; i++) {
            delete [] (char*)Frames[i].ImageBuffer;
        }
        delete [] Frames;
        delete [] frameNums;
    }

    Handle = NULL;
    camsActive--;
    if (camsActive == 0) {
        PvUnInitialize();
        cout << "closing api: " << endl;
    }
}

bool Capture::grabFrame(void) {
    if(mFromFileFlag){
        sprintf(imageNum, "%s%d.tif", baseImageName, imageNumber);
        std::cout << "from file, im: "<< imageNum << std::endl;
        std::cout << "loading: " << imageNum << std::endl;
        mFrame = imread(imageNum, -1);
        width = mFrame.cols;
        height = mFrame.rows;
        imageNumber++;
        return true;
   // } else if (CAPTUREMODE == CONTINUOUS) {
   } else {
        PvCaptureQueueFrame(Handle,&tmpFrame,NULL);
        PvCaptureWaitForFrameDone(Handle,&tmpFrame,PVINFINITE);
        bgrImage.data = (uchar *)tmpFrame.ImageBuffer;
        mFrame = bgrImage;
        frameTimestamp = GetClock();
        return true;
    }/* else {
        PvCommandRun(Handle, "FrameStartTriggerSoftware");
        printf("queue...");
        PvCaptureQueueFrame(Handle,&tmpFrame,NULL);
        PvCommandRun(Handle,"AcquisitionStart");
        printf("...done! \n");
        printf("queue2...");
        PvCaptureWaitForFrameDone(Handle,&tmpFrame,PVINFINITE);
        printf("...done! \n");
        bgrImage.data = (uchar *)tmpFrame.ImageBuffer;
        //copyImage(&tmpFrame, bgrImage);
        //cvUndistort2(bgrImage, mUndistortedImage, intrinsicMatrix, distortionMatrix);
        //mFrame = mUndistortedImage;
        mFrame = bgrImage;
        frameTimestamp = GetClock();
        return true;
    }*/
}

 void Capture::startFrameGrab(void) {
   //if (CAPTUREMODE == CONTINUOUS) {
        PvCaptureQueueFrame(Handle,&tmpFrame,NULL);
    /*} else if (CAPTUREMODE == SINGLEFRAME) {
        PvCommandRun(Handle, "FrameStartTriggerSoftware");
        PvCaptureQueueFrame(Handle,&tmpFrame,NULL);
        PvCommandRun(Handle,"AcquisitionStart");
    }*/
}

bool Capture::grabFrameAlreadyQueued(void) {
    if(mFromFileFlag){
        sprintf(imageNum, "%s%d.tif", baseImageName, imageNumber);
        std::cout << "loading: " << imageNum << std::endl;
        mFrame = imread(imageNum, -1);
        width = mFrame.cols;
        height = mFrame.rows;
        imageNumber++;
        return true;
   } else {
        PvCaptureWaitForFrameDone(Handle,&tmpFrame,PVINFINITE);
        mFrame.data = (uchar *)tmpFrame.ImageBuffer;
        //copyImage(&tmpFrame, bgrImage);
        //cvUndistort2(bgrImage, mUndistortedImage, intrinsicMatrix, distortionMatrix);
        //mFrame = mUndistortedImage;
        //mFrame = bgrImage;
        frameTimestamp = GetClock();
        return true;
    }/* else {
        PvCommandRun(Handle, "FrameStartTriggerSoftware");
        printf("queue...");
        PvCaptureQueueFrame(Handle,&tmpFrame,NULL);
        PvCommandRun(Handle,"AcquisitionStart");
        printf("...done! \n");
        printf("queue2...");
        PvCaptureWaitForFrameDone(Handle,&tmpFrame,PVINFINITE);
        printf("...done! \n");
        bgrImage.data = (uchar *)tmpFrame.ImageBuffer;
        //copyImage(&tmpFrame, bgrImage);
        //cvUndistort2(bgrImage, mUndistortedImage, intrinsicMatrix, distortionMatrix);
        //mFrame = mUndistortedImage;
        mFrame = bgrImage;
        frameTimestamp = GetClock();
        return true;
    }*/
}

bool Capture::grabFrameAlreadyQueuedUndistorted(void) {
    if(mFromFileFlag){
        sprintf(imageNum, "%s%d.tif", baseImageName, imageNumber);
        std::cout << "loading: " << imageNum << std::endl;
        undistortImage(imread(imageNum, -1), mFrame);
        width = mFrame.cols;
        height = mFrame.rows;
        imageNumber++;
        return true;
   } else {
        PvCaptureWaitForFrameDone(Handle,&tmpFrame,PVINFINITE);
        //bgrImage.data = (uchar *)tmpFrame.ImageBuffer;
        //cvUndistort2(bgrImage, mUndistortedImage, intrinsicMatrix, distortionMatrix);
        mFrame.data = NULL;
        undistortImage(&tmpFrame, mFrame);
        //mFrame = undistortImage(bgrImage);
        //mFrame = undistortImage(mDistortedImage);
        //undistort(mDistortedImage, mFrame, intrinsicMatrix, distortionMatrix);
        //mFrame = mUndistortedImage;
        frameTimestamp = GetClock();
        return true;
    }/* else {
        PvCommandRun(Handle, "FrameStartTriggerSoftware");
        printf("queue...");
        PvCaptureQueueFrame(Handle,&tmpFrame,NULL);
        PvCommandRun(Handle,"AcquisitionStart");
        printf("...done! \n");
        printf("queue2...");
        PvCaptureWaitForFrameDone(Handle,&tmpFrame,PVINFINITE);
        printf("...done! \n");
        bgrImage.data = (uchar *)tmpFrame.ImageBuffer;
        //copyImage(&tmpFrame, bgrImage);
        //cvUndistort2(bgrImage, mUndistortedImage, intrinsicMatrix, distortionMatrix);
        //mFrame = mUndistortedImage;
        mFrame = bgrImage;
        frameTimestamp = GetClock();
        return true;
    }*/
}


/*void Capture::copyImage(const tPvFrame* frame, IplImage* dest) const {
  char*       lDest   = dest->imageData;
  const unsigned char* lSrc    = (unsigned char*)frame->ImageBuffer;
  const unsigned char* lSrcEnd = lSrc + (frame->Width * frame->Height * 3);

// for bgr24 cap mode
while(lSrc < lSrcEnd) {
lDest[0] = lSrc[0];
lDest[1] = lSrc[1];
lDest[2] = lSrc[2];
lSrc += 3;
lDest += 3;
}
}*/

void Capture::startCapture(void) {
    if(!mFromFileFlag) {
        // begin grabbing frames with callback
        PvCommandRun(Handle,"AcquisitionEnd");
        frameReady = false;
        capturing = true;
        for(int i = 0; i < NFRAMES; i++) {
            PvCaptureWaitForFrameDone(Handle,&(Frames[i]),100);
            Frames[i].Context[0] = &frameReady;
            Frames[i].Context[1] = &Handle;

            Frames[i].Context[2] = &(frameNums[i]);
            Frames[i].Context[3] = &curFrame;

            PvCaptureQueueFrame(Handle, &(Frames[i]), FrameCB0);
        }
        if(PvCommandRun(Handle,"AcquisitionStart")){
            PvCaptureEnd(Handle) ;
            cout << __FILE__ << "\t line " << __LINE__ << " : Camera could not be started" << endl;
            exit(1);
        }

    }
}

void Capture::endCapture(void) {
    capturing = false;
    for(int i = 0; i < NFRAMES; i++) {
        PvCaptureWaitForFrameDone(Handle,&(Frames[i]),100);
    }
}


void Capture::syncFrameCaptureSetCenter(vector<Capture*>& otherCams) //sets this as sync out, otherCams with syncIn
{
  /*PvAttrEnumSet(Handle, "SyncOut2Mode", "Exposing");
  PvAttrEnumSet(Handle, "FrameStartTriggerMode", "FixedRate");
  PvAttrFloat32Set(Handle,"FrameRate",FRAMERATE);
	*/
	syncFrameCaptureSetCenter();

  for (int i=0; i < otherCams.size(); i++)
  {
		otherCams[i]->setSyncIn();
    //PvAttrEnumSet(otherCams[i]->Handle, "FrameStartTriggerMode", "SyncIn2");
  }
}

void Capture::syncFrameCaptureSetCenter()
{
  PvAttrEnumSet(Handle, "SyncOut2Mode", "Exposing");
  PvAttrEnumSet(Handle, "FrameStartTriggerMode", "FixedRate");
  PvAttrFloat32Set(Handle,"FrameRate",FRAMERATE);
}

void Capture::setSyncIn()
{
	PvAttrEnumSet(Handle, "FrameStartTriggerMode", "SyncIn2");
}

bool Capture::waitForFrame(void)
{
    if(mFromFileFlag)
    {
      /*
        std::string s;
        mImageFilesStream >> s;


        std::cout << s << std::endl;

        if(!mImageFilesStream.fail()) {
            mFrame = imread(s.c_str(), -1);
            return true;
        }
        else
            cerr << __FILE__ << "\t line " << __LINE__ << " :end of imageFile reached" << endl;

        if(mFrame.empty())
            cerr << __FILE__ << "\t line " << __LINE__ << " :failed to capture a frame" << endl;
            */

      sprintf(imageNum, "%s%d.tif", baseImageName, imageNumber);
      std::cout << "loading: " << imageNum << std::endl;
      mFrame = imread(imageNum, -1);
      width = mFrame.cols;
      height = mFrame.rows;
      imageNumber++;
      return true;


    }
    else
    {
        if(frameReady) {
            frameReady = false;
            bgrImage.data = (uchar *)Frames[curFrame].ImageBuffer;
            //mFrame = bgrImage.clone();
            mFrame.data = bgrImage.data;
            frameTimestamp = GetClock();
            return true;
        }

    }
    return false;
}

bool Capture::waitForFrameUndistorted(void)
{
    if(mFromFileFlag)
    {
      sprintf(imageNum, "%s%d.tif", baseImageName, imageNumber);
      std::cout << "loading: " << imageNum << std::endl;
      mFrame = imread(imageNum, -1);
      width = mFrame.cols;
      height = mFrame.rows;
      imageNumber++;
      return true;

      /*
        std::string s;
        mImageFilesStream >> s;

        std::cout << s.c_str() << std::endl;

        if(!mImageFilesStream.fail()) {
            undistortImage(imread(s.c_str(),-1), mFrame);
            return true;
        }
        else
            cerr << __FILE__ << "\t line " << __LINE__ << " :end of imageFile reached" << endl;

        if(mFrame.empty())
            cerr << __FILE__ << "\t line " << __LINE__ << " :failed to capture a frame" << endl;
            */
    }
    else
    {
        if(frameReady) {
            frameReady = false;        
            bgrImage.data = (uchar *)Frames[curFrame].ImageBuffer;
            undistortImage(bgrImage.clone(), mFrame);
            frameTimestamp = GetClock();
            return true;
        }

    }
    return false;
}


void Capture::undistortImage(const tPvFrame* frame, Mat &toUndistort) {
    bgrImage.data = (uchar *)frame->ImageBuffer;
    remap(bgrImage, toUndistort, xDistortMap, yDistortMap,
            INTER_LINEAR,BORDER_CONSTANT, cvScalarAll(0));
}

void Capture::undistortImage(const Mat &img, Mat &toUndistort){
    remap(img, toUndistort, xDistortMap, yDistortMap,
            INTER_LINEAR,BORDER_CONSTANT, cvScalarAll(0));
}

void Capture::projectPointUndistorted(const Mat &point3d, Point2f &point2d)
{
    float data[3];
    Mat X = Mat(3,1,CV_32FC1, data);
    X = mR_world2cam*point3d + mExtTranslationVector;
    data[0] = data[0]/data[2];
    data[1] = data[1]/data[2];
    data[2] = 1.;
    Mat temp = intrinsicMatrix*X;
    point2d.x = temp.at<float>(0,0);
    point2d.y = temp.at<float>(1,0);
}

void Capture::projectPointUndistorted(const Mat &point3d, Point2i &point2d)
{
    float data[3];
    Mat X = Mat(3,1,CV_32FC1, data);
    X = mR_world2cam*point3d + mExtTranslationVector;
    data[0] = data[0]/data[2];
    data[1] = data[1]/data[2];
    data[2] = 1.;
    Mat temp = intrinsicMatrix*X;
    point2d.x = (int)temp.at<float>(0,0);
    point2d.y = (int)temp.at<float>(1,0);
}



/*CvScalar Capture::undistortPoint(const Point2D& pt) const {
  cvSet2D(ptMat,0,0, cvScalar(pt.x, pt.y));
  cvUndistortPoints(ptMat, ptUndistorted, intrinsicMatrix, distortionMatrix, NULL, intrinsicMatrix);
  return cvGet2D(ptUndistorted,0,0);
  }
*/
  
void Capture::getWorldRay(const Point& pt, Mat& ray) {
    mRayCam.at<float>(0,0) = (float)(pt.x - principalPoint[0])/focalLength[0];
    mRayCam.at<float>(1,0) = (float)(pt.y - principalPoint[1])/focalLength[1];
    ray = mR_cam2world*mRayCam;
}

void Capture::getWorldRay(const Point2f& pt, Mat& ray) {
    mRayCam.at<float>(0,0) = (float)(pt.x - principalPoint[0])/focalLength[0];
    mRayCam.at<float>(1,0) = (float)(pt.y - principalPoint[1])/focalLength[1];
    ray = mR_cam2world*mRayCam;
}

void Capture::AddOtherCameraInformation(const Capture& otherCamera)
{
    if (!initDone)
    {
        cerr << "Error: Other Camera is not Initialized yet" << endl;
        return;           
    } else if(!otherCamera.initDone)
    {
        cerr << "Error: Other Camera is not Initialized yet" << endl;
    }

    if (otherCamParams.size() < otherCamera.idInternal+1){        
        otherCamParams.resize(otherCamera.idInternal+1);
    }

//    if (otherCamParams.at(otherCamera.idInternal) != NULL)
//        cerr << "This position has already been initialized. Continuing anyway" << endl;


    Mat RotationToMyFrame = Mat(3,3,CV_32FC1);
    Mat OtherCameraPositionMyFrame = Mat(3,1,CV_32FC1);

    RotationToMyFrame = mR_world2cam*otherCamera.mR_cam2world;
    OtherCameraPositionMyFrame = mR_world2cam * otherCamera.mCameraPosition + mExtTranslationVector;
    
    InterCameraInfo info;
    info.OtherCameraPositionMyFrame = OtherCameraPositionMyFrame.clone();
    info.RotationToMyFrame = RotationToMyFrame*otherCamera.intrinsicMatrix.inv();

    for (int i = 0; i < 3; i ++)
        OtherCameraPositionMyFrame.at<float>(i,0) = OtherCameraPositionMyFrame.at<float>(i,0)/OtherCameraPositionMyFrame.at<float>(2,0);
    Mat temp = intrinsicMatrix*OtherCameraPositionMyFrame;

    info.OtherCameraPositionMyPix.x = temp.at<float>(0,0);
    info.OtherCameraPositionMyPix.y = temp.at<float>(1,0);

    otherCamParams[otherCamera.idInternal] = info;
}


void Capture::GetEpipolarLine(const Capture& otherCamera, const Point2f& pointSeenOtherFrame, Mat& lineParams)
{
    //convert point to 3d
    //point seen is in [x,y], not [r,c]
    Mat p_otherCam = Mat(3,1,CV_32FC1);
    p_otherCam.at<float>(0,0) = pointSeenOtherFrame.x;
    p_otherCam.at<float>(1,0) = pointSeenOtherFrame.y;
    p_otherCam.at<float>(2,0) = 1;

    InterCameraInfo info = otherCamParams[otherCamera.idInternal]; 

    p_otherCam = info.OtherCameraPositionMyFrame + info.RotationToMyFrame*p_otherCam;
         
    p_otherCam.at<float>(0,0) = p_otherCam.at<float>(0,0)/p_otherCam.at<float>(2,0);
    p_otherCam.at<float>(1,0) = p_otherCam.at<float>(1,0)/p_otherCam.at<float>(2,0);
    p_otherCam.at<float>(2,0) = 1.0;

    Mat temp = intrinsicMatrix*p_otherCam;

    Point2f p_otherCam_myPix(temp.at<float>(0,0), temp.at<float>(1,0));
   
    TwoPointsToLine(info.OtherCameraPositionMyPix, p_otherCam_myPix, lineParams);
}

void Capture::GetEpipolarLine(const Capture& otherCamera, const Point2i& pointSeenOtherFrame, Mat& lineParams)
{
  Point2f pt((float)pointSeenOtherFrame.x, (float)pointSeenOtherFrame.y);
  GetEpipolarLine(otherCamera, pt, lineParams);
}

void Capture::drawPose(Mat& rotation_matrix, Mat& translation_vector)
{

  Point3f start_pt_3d;
  start_pt_3d.x = translation_vector.at<float>(0,0);
  start_pt_3d.y = translation_vector.at<float>(1,0);
  start_pt_3d.z = translation_vector.at<float>(2,0);

  Scalar xColor(0,0,255);
  Mat x_mat_3d = translation_vector + rotation_matrix.col(0)*5.0;
  Point3d x_pt_3d;
  x_pt_3d.x = x_mat_3d.at<float>(0,0);
  x_pt_3d.y = x_mat_3d.at<float>(1,0);
  x_pt_3d.z = x_mat_3d.at<float>(2,0);

  Scalar yColor(0,255,0);
  Mat y_mat_3d = translation_vector + rotation_matrix.col(1)*5.0;
  Point3d y_pt_3d;
  y_pt_3d.x = y_mat_3d.at<float>(0,0);
  y_pt_3d.y = y_mat_3d.at<float>(1,0);
  y_pt_3d.z = y_mat_3d.at<float>(2,0);

  Scalar zColor(255,0,0);
  Mat z_mat_3d = translation_vector + rotation_matrix.col(2)*5.0;
  Point3d z_pt_3d;
  z_pt_3d.x = z_mat_3d.at<float>(0,0);
  z_pt_3d.y = z_mat_3d.at<float>(1,0);
  z_pt_3d.z = z_mat_3d.at<float>(2,0);


  //FOR DEBUG - replace checker points with estimated points
  Point2f start_pt;
  Point2f x_pt;
  Point2f y_pt;
  Point2f z_pt;
  projectPointUndistorted(translation_vector, start_pt);
  projectPointUndistorted(x_mat_3d, x_pt);
  projectPointUndistorted(y_mat_3d, y_pt);
  projectPointUndistorted(z_mat_3d, z_pt);
  cv::line(mFrame, start_pt, x_pt, xColor, 2);
  cv::line(mFrame, start_pt, y_pt, yColor, 2);
  cv::line(mFrame, start_pt, z_pt, zColor, 2);
}


/*
  const CvMat* Capture::getUndistortedRay(const Point2D& pt) const {
  CvScalar pos = undistortPoint(pt);
  cvSetReal2D(mRayCam, 0, 0, (double)((pos.val[0] - principalPoint[0]) / focalLength[0]));
  cvSetReal2D(mRayCam, 1, 0, (double)((pos.val[1] - principalPoint[1]) / focalLength[1]));
  cvMatMul(mR_cam2world, mRayCam, mRayWorld);
  return mRayWorld;
  }

  IplImage * Capture::getUndistortedImage(void) {
  if(capturing) {
  return undistortImage(&Frames[curFrame]);
  }
  else {
  return undistortImage(&tmpFrame);
  }
  }


  Point2D Capture::getEstimatedLocationProjection(const Point3D &p)
  {
  CvMat* posMat		= cvCreateMat(3, 4, CV_32FC1);
  CvMat* projMat	= cvCreateMat(2, 4, CV_32FC1);
  for (int j = 0; j < 4; j++)
  {
  cvSetReal2D(posMat, 0, j, p.x);
  cvSetReal2D(posMat, 1, j, p.y);
  cvSetReal2D(posMat, 2, j, p.z);
  }

  cvProjectPoints2(posMat, mExtRotationVector, mExtTranslationVector, intrinsicMatrix, distortionMatrix, projMat);

  Point2D res;
  res.x = cvGetReal2D(projMat, 0, 0);
  res.y = cvGetReal2D(projMat, 1, 0);

  cvReleaseMat(&posMat);
  cvReleaseMat(&projMat);

  return res;
  }

  Point2D Capture::displayEstimatedLocation(const Point3D & p)
  {
  CvMat* posMat		= cvCreateMat(3, 4, CV_32FC1);
  CvMat* projMat	= cvCreateMat(2, 4, CV_32FC1);
  for (int j = 0; j < 4; j++)
  {
  cvSetReal2D(posMat, 0, j, p.x);
  cvSetReal2D(posMat, 1, j, p.y);
  cvSetReal2D(posMat, 2, j, p.z);
  }

  cvProjectPoints2(posMat, mExtRotationVector, mExtTranslationVector, intrinsicMatrix, distortionMatrix, projMat);

  Point2D res;
  res.x = cvGetReal2D(projMat, 0, 0);
  res.y = cvGetReal2D(projMat, 1, 0);

  cvReleaseMat(&posMat);
  cvReleaseMat(&projMat);

  return res;
  }
 */


