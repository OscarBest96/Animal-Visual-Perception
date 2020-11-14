/*
Phil Culverhouse Oct 2016 (c) Plymouth University
James Rogers Jan 2020     (c) Plymouth University

This demo code will compare the left and right images to produce a disparity map.

Before trying this code, you will need to calibrate servos so the cameras are parallel,
and calibrate the cameras using the stereo calibration program in the tools folder.

Use this code as a base for your assignment.

*/

#include <iostream>
#include <fstream>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include <stdio.h>

#include <string>
#include <sys/types.h>
#ifndef _WIN32
#include <unistd.h>
#endif
#include "owl-pwm.h"
#include "owl-comms.h"
#include "owl-cv.h"
#include<cmath>

using namespace cv;
using namespace std;


int main(int argc, char** argv)
{


    //Setup TCP coms
    ostringstream CMDstream; // string packet
    string CMD;
    string PiADDR = "10.0.0.10";
    int PORT=12345;
    SOCKET u_sock = OwlCommsInit(PORT, PiADDR);

    //Set servo positions to their center-points
    Rx = RxC; Lx = LxC;
    Ry = RyC; Ly = LyC;
    Neck= NeckC;

    // move servos to centre of field
    CMDstream.str("");
    CMDstream.clear();
    CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
    CMD = CMDstream.str();
    string RxPacket= OwlSendPacket (u_sock, CMD.c_str());



    string intrinsic_filename = "../../Data/intrinsics.xml";
    string extrinsic_filename = "../../Data/extrinsics.xml";

    int SADWindowSize=3;
    int numberOfDisparities=256;
    double scale = 1;

    //Check input variables
    if ( numberOfDisparities < 1 || numberOfDisparities % 16 != 0 ){
        printf("The max disparity must be a positive integer divisible by 16\n");
        return -1;
    }

    if (scale < 0){
        printf("The scale factor must be a positive floating-point number\n");
        return -1;
    }

    if (SADWindowSize < 1 || SADWindowSize % 2 != 1)
    {
        printf("The SADWindowSize must be a positive odd number\n");
        return -1;
    }

    // reading calibration data
    Rect roi1, roi2;
    Mat Q;
    Size img_size = {640,480};

    FileStorage fs(intrinsic_filename, FileStorage::READ);
    if(!fs.isOpened()){
        printf("Failed to open file %s\n", intrinsic_filename.c_str());
        return -1;
    }

    Mat M1, D1, M2, D2;
    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;

    M1 *= scale;
    M2 *= scale;

    fs.open(extrinsic_filename, FileStorage::READ);
    if(!fs.isOpened())
    {
        printf("Failed to open file %s\n", extrinsic_filename.c_str());
        return -1;
    }
    Mat R, T, R1, P1, R2, P2;
    fs["R"] >> R;
    fs["T"] >> T;

    stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );

    Mat map11, map12, map21, map22;
    initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
    initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

    //Open video stream
    string source ="http://10.0.0.10:8080/stream/video.mjpeg";
    VideoCapture cap (source);              // Open input
    if (!cap.isOpened())
    {
        cout  << "Could not open the input video: " << source << endl;
        return -1;
    }

    Mat Frame,Left,Right, disp, disp8;
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);


    //Send new motor positions to the owl servos
    CMDstream.str("");
    CMDstream.clear();
    CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
    CMD = CMDstream.str();
    RxPacket= OwlSendPacket (u_sock, CMD.c_str());


    while (1){
        if (!cap.read(Frame))
        {
            cout  << "Could not open the input video: " << source << endl;
            break;
        }

        //flip input image as it comes in reversed
        Mat FrameFlpd;
        flip(Frame,FrameFlpd,1);

        // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG iamge
        Left= FrameFlpd(Rect(0, 0, 640, 480)); // using a rectangle
        Right=FrameFlpd(Rect(640, 0, 640, 480)); // using a rectangle


        //Distort image to correct for lens/positional distortion
        Mat Leftr, Rightr;
        remap(Left, Left, map11, map12, INTER_LINEAR);
        remap(Right, Right, map21, map22, INTER_LINEAR);

        //Match left and right images to create disparity image
        numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;
        int cn = Left.channels();
        int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;

        sgbm->setBlockSize(sgbmWinSize);
        sgbm->setPreFilterCap(63);
        sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
        sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
        sgbm->setMinDisparity(0);
        sgbm->setNumDisparities(numberOfDisparities);
        sgbm->setUniquenessRatio(10);
        sgbm->setSpeckleWindowSize(100);
        sgbm->setSpeckleRange(32);
        sgbm->setDisp12MaxDiff(1);
        sgbm->setMode(StereoSGBM::MODE_SGBM);

        int64 t = getTickCount();
        sgbm->compute(Left, Right, disp);
        t = getTickCount() - t;
        printf("Time elapsed: %fms\n", t*1000/getTickFrequency());

        //Draw a circle in the middle of the left and right image (usefull for aligning both cameras)
        circle(Left,Point(Left.size().width/2,Left.size().height/2),10,Scalar(255,255,255),1);
        circle(Right,Point(Right.size().width/2,Right.size().height/2),10,Scalar(255,255,255),1);

        //Convert disparity map to an 8-bit greyscale image so it can be displayed
        disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
        imshow("left", Left);
        imshow("right", Right);
        imshow("disparity", disp8);
        waitKey(10);
    }

    return 0;
}
