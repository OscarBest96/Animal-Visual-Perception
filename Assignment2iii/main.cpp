/*
Phil Culverhouse Oct 2016 (c) Plymouth University
James Rogers Jan 2020     (c) Plymouth University

This demo code will simulate a human attention system as discribed in Itti & Koch’s bottom-up Saccadic model.
The following feature maps are currently implemented:
    -DoG edge detection
    -Fovea (bias for central field targets)
    -Familiarity (bias for targets that havent been observed often)

You will need to add at least two more feature maps, as well as a distance estimation to the current target



Use this code as a base for your assignment.

*/

#define PI 3.14159265

#include <iostream>
#include <fstream>
#include <math.h>
#include <string>

#include <sys/types.h>
#ifndef _WIN32
#include <unistd.h>
#endif
#include "owl-pwm.h"
#include "owl-comms.h"
#include "owl-cv.h"

#include "opencv2/calib3d.hpp"

#define PX2DEG  0.0768
#define DEG2PWM 10.730
#define IPD 58.3

#define xOffset  -0.3815
#define yOffset -54.6682
#define xScale   26.4842
#define yScale   91.4178

using namespace std;
using namespace cv;

string ServoAbs( double DEGRx,double DEGRy,double DEGLx,double DEGLy,double DEGNeck);
string ServoRel(double DEGRx,double DEGRy,double DEGLx,double DEGLy,double DEGNeck);
string TrackCorrelTarget (OwlCorrel OWL);
Mat DoGFilter(Mat src, int k, int g);

//TCP coms constants
static string PiADDR = "10.0.0.10";
static int PORT=12345;
static SOCKET u_sock = OwlCommsInit ( PORT, PiADDR);
static ostringstream CMDstream; // string packet
static string CMD;

//Default feature map weights
static int ColourWeight    = 60 ; //Saturation and Brightness
static int DoGHighWeight    = 60 ; //Groups of edges in a small area
static int DoGLowWeight     = 30 ; //DoG edge detection
static int FamiliarWeight = 5  ; //Familiarity of the target, how much has the owl focused on this before
static int foveaWeight    = 50 ; //Distance from fovea (center)


int main(int argc, char *argv[])
{
    //==========================================Initialize Variables=================================
    //Frame Size
    Size imageSize;
    imageSize.width=640;
    imageSize.height=480;

    //Local Feature Map  - implements FOVEA as a bias to the saliency map to central targets, rather than peripheral targets
    // eg. for a primate vision system
    Mat fovea=Mat(480, 640, CV_8U, double(0));
    fovea=Mat(480, 640, CV_8U, double(0));
    circle(fovea, Point(320,240), 150, 255, -1);
    cv::blur(fovea, fovea, Size(301,301));
    fovea.convertTo(fovea, CV_32FC1);
    fovea*=foveaWeight;

    //Initilize Mats
    const Mat OWLresult;// correlation result passed back from matchtemplate
    Mat Frame;
    Mat Left, Right, OWLtempl; // images
    Mat familiar=Mat(1600, 2500, CV_8U, double(255));

    //Video stream source
    string source ="http://10.0.0.10:8080/stream/video.mjpeg"; // was argv[1];           // the source file name

    //Set center neck positions
    Rx = RxC; Lx = LxC;
    Ry = RyC; Ly = LyC;
    Neck= NeckC;

    //Variable declorations
    double minVal; double maxVal; Point minLoc; Point maxLoc;
    Point minLocTarget; Point maxLocTarget;

    //========================================Initialize Servos========================================
    CMDstream.str("");
    CMDstream.clear();
    CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
    CMD = CMDstream.str();
    string RxPacket= OwlSendPacket (u_sock, CMD.c_str());

    //========================================Open Video Steam========================================
    VideoCapture cap (source);
    if (!cap.isOpened()){
        cout  << "Could not open the input video: " << source << endl;
        return -1;
    }

    while (1){//Main processing loop
        //cout<<"Capture Frame"<<endl;
        //==========================================Capture Frame============================================

        if (!cap.read(Frame)){
            cout  << "Could not open the input video: " << source << endl;
        }

        Mat FrameFlpd;
        flip(Frame,FrameFlpd,1);// Note that Left/Right are reversed now

        // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG iamge
        Left= FrameFlpd( Rect(0, 0, 640, 480));
        Right=FrameFlpd( Rect(640, 0, 640, 480));

        Mat LeftGrey;                                   //Make a grey copy of Left
        cvtColor(Left, LeftGrey, COLOR_BGR2GRAY);


        // ======================================CALCULATE FEATURE MAPS ====================================
        //============================================DoG low bandpass Map============================================
        Mat DoGLow = DoGFilter(LeftGrey,3,51);
        Mat DoGLow8;
        normalize(DoGLow, DoGLow8, 0, 255, CV_MINMAX, CV_8U);
        imshow("DoG Low", DoGLow8);

        //=====================================Initialise Global Position====================================
        //cout<<"Globe Pos"<<endl;
        Point GlobalPos;    // Position of camera view within the range of movement of the OWL
        GlobalPos.x=static_cast<int>(900+((-(Neck-NeckC)+(Lx-LxC))/DEG2PWM)/PX2DEG);
        GlobalPos.y=static_cast<int>(500+((Ly-LyC)/DEG2PWM)/PX2DEG);

        if(GlobalPos.x<0){
            GlobalPos.x=0;
        }else if(GlobalPos.x>=(familiar.size().width-Left.size().width)){
            GlobalPos.x=(familiar.size().width-Left.size().width)-1;
        }

        if(GlobalPos.y<0){
            GlobalPos.y=0;
        }else if(GlobalPos.y>=(familiar.size().height-Left.size().height)){
            GlobalPos.y=(familiar.size().height-Left.size().height)-1;
        }

        Mat familiarLocal=familiar(Rect(GlobalPos.x,GlobalPos.y,Left.cols,Left.rows));
        //imshow("familiarLocal",familiarLocal);

        //====================================Combine maps into saliency map=====================================
        //cout<<"Salience"<<endl;


        //Convert 8-bit Mat to 32bit floating point
        DoGLow.convertTo(DoGLow, CV_32FC1);
        DoGLow*=DoGLowWeight;
        familiarLocal.convertTo(familiarLocal, CV_32FC1);

        // Linear combination of feature maps to create a salience map
        Mat Salience=cv::Mat(Left.size(),CV_32FC1,0.0); // init map

        add(Salience,DoGLow,Salience);
        add(Salience,fovea,Salience);
        Salience=Salience.mul(familiarLocal);
        normalize(Salience, Salience, 0, 255, CV_MINMAX, CV_32FC1);

        //imshow("SalienceNew",Salience);

        //=====================================Find & Move to Most Salient Target=========================================

        minMaxLoc(Salience, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
        // Calculate relative servo correction and magnitude of correction
        double xDifference = static_cast<double>((maxLoc.x-320)*PX2DEG);
        double yDifference = static_cast<double>((maxLoc.y-240)*PX2DEG);

        rectangle(Left,Point(maxLoc.x-32,maxLoc.y-32),Point(maxLoc.x+32,maxLoc.y+32),Scalar::all(255),2,8,0); //draw rectangle on most salient area
        // Move left eye based on salience, move right eye to be parallel with left eye
        ServoRel(((Lx-LxC+RxC-Rx)/DEG2PWM)+xDifference,-((LyC-Ly+RyC-Ry)/DEG2PWM)+yDifference,xDifference,yDifference,(Lx-LxC)/100);

        // Update Familarity Map //
        // Familiar map to inhibit salient targets once observed (this is a global map)
        Mat familiarNew=familiar.clone();
        circle(familiarNew, GlobalPos+maxLoc, static_cast<int>(60/cos(40*PI/180)), 0, -1);
        cv::blur(familiarNew, familiarNew, Size(151,151)); //Blur used to save on processing
        normalize(familiarNew, familiarNew, 0, 255, CV_MINMAX, CV_8U);
        addWeighted(familiarNew, (static_cast<double>(FamiliarWeight)/100), familiar, (100-static_cast<double>(FamiliarWeight))/100, 0, familiar);

        Mat familiarSmall;
        resize(familiar,familiarSmall,familiar.size()/4);
        imshow("Familiar",familiarSmall);

        //=================================Convert Saliency into Heat Map=====================================
        //this is just for visuals
        Mat SalienceHSVnorm;
        Salience.convertTo(Salience, CV_8UC1);
        normalize(Salience, SalienceHSVnorm, 130, 255, CV_MINMAX, CV_8U);
        normalize(Salience, Salience, 0, 255, CV_MINMAX, CV_8U);
        Mat SalienceHSV;
        cvtColor(Left, SalienceHSV, COLOR_BGR2HSV);

        for(int y=0;y<480;y++){
            for(int x=0; x<640;x++){
                SalienceHSV.at<Vec3b>(y,x)=Vec3b(255-SalienceHSVnorm.at<uchar>(y,x),255,255);
            }
        }
        cvtColor(SalienceHSV, SalienceHSV, COLOR_HSV2BGR);


        //=======================================Update Global View===========================================
        resize(Left,Left,Left.size()/2);
        imshow("Left",Left);
        resize(SalienceHSV,SalienceHSV,SalienceHSV.size()/2);
        imshow("SalienceHSV",SalienceHSV);

        //=========================================Control Window for feature weights =============================================
        //cout<<"Control Window"<<endl;
        namedWindow("Control", CV_WINDOW_AUTOSIZE);
        cvCreateTrackbar("LowFreq"    , "Control", &DoGLowWeight    , 100);
        cvCreateTrackbar("FamiliarW", "Control", &FamiliarWeight, 100);
        cvCreateTrackbar("foveaW"   , "Control", &foveaWeight   , 100);

        waitKey(10);
    }
}

//====================================================================================//
// SERVO FUNCTIONS

string ServoAbs( double DEGRx,double DEGRy,double DEGLx,double DEGLy,double DEGNeck){
    int Rx,Ry,Lx,Ly, Neck;
    //    Rx=static_cast<int>(DEGRx*DEG2PWM)+RxC;
    //    Ry=static_cast<int>(-DEGRy*DEG2PWM)+RyC;
    //    Lx=static_cast<int>(DEGLx*DEG2PWM)+LxC;
    //    Ly=static_cast<int>(DEGLy*DEG2PWM)+LyC;
    //    Neck=static_cast<int>(-DEGNeck*DEG2PWM)+NeckC;
    Rx=static_cast<int>(DEGRx*DEG2PWM);
    Ry=static_cast<int>(DEGRy*DEG2PWM);
    Lx=static_cast<int>(DEGLx*DEG2PWM);
    Ly=static_cast<int>(DEGLy*DEG2PWM);
    Neck=static_cast<int>(DEGNeck*DEG2PWM);

    if(Rx>RxRm){
        Rx=RxRm;
    }else if(Rx<RxLm){
        Rx=RxLm;
    }

    if(Lx>LxRm){
        Lx=LxRm;
    }else if(Lx<LxLm){
        Lx=LxLm;
    }

    if(Ry>RyTm){
        Ry=RyTm;
    }else if(Ry<RyBm){
        Ry=RyBm;
    }

    if(Ly>LyBm){
        Ly=LyBm;
    }else if(Ly<LyTm){
        Ly=LyTm;
    }

    if(Neck>NeckL){
        Neck=NeckL;
    }else if(Neck<NeckR){
        Neck=NeckR;
    }

    CMDstream.str("");
    CMDstream.clear();
    CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
    string CMD = CMDstream.str();
    string retSTR = OwlSendPacket (u_sock, CMD.c_str());
    return (retSTR);
}

string ServoRel(double DEGRx,double DEGRy,double DEGLx,double DEGLy,double DEGNeck){
    //int Rx,Ry,Lx,Ly, Neck;
    Rx=static_cast<int>(DEGRx*DEG2PWM)+Rx;
    Ry=static_cast<int>(-DEGRy*DEG2PWM)+Ry;
    Lx=static_cast<int>(DEGLx*DEG2PWM)+Lx;
    Ly=static_cast<int>(DEGLy*DEG2PWM)+Ly;
    Neck=static_cast<int>(-DEGNeck*DEG2PWM)+Neck;

    if(Rx>RxRm){
        Rx=RxRm;
    }else if(Rx<RxLm){
        Rx=RxLm;
    }

    if(Lx>LxRm){
        Lx=LxRm;
    }else if(Lx<LxLm){
        Lx=LxLm;
    }

    if(Ry>RyTm){
        Ry=RyTm;
    }else if(Ry<RyBm){
        Ry=RyBm;
    }

    if(Ly>LyBm){
        Ly=LyBm;
    }else if(Ly<LyTm){
        Ly=LyTm;
    }

    if(Neck>NeckL){
        Neck=NeckL;
    }else if(Neck<NeckR){
        Neck=NeckR;
    }

    CMDstream.str("");
    CMDstream.clear();
    CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
    string CMD = CMDstream.str();
    string retSTR=OwlSendPacket (u_sock, CMD.c_str());
    return(retSTR);
}

// create DoG bandpass filter, with g being odd always and above 91 for low pass, and >9 for high pass
// k is normally 3 or 5
Mat DoGFilter(Mat src, int k, int g){
    Mat srcC;
    src.convertTo(srcC,CV_32FC1);
    Mat g1, g2;
    GaussianBlur(srcC, g1, Size(g,g), 0);
    GaussianBlur(srcC, g2, Size(g*k,g*k), 0);
    srcC = (g1 - g2)*2;
    return srcC;

}


















