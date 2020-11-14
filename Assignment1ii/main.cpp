/*
Phil Culverhouse Oct 2016 (c) Plymouth University
James Rogers Jan 2020     (c) Plymouth University

This demo code will move eye and neck servos with kepresses.
Use this code as a base for your assignment.

*/

#include <iostream>
#include <fstream>
#include <string>
#include <sys/types.h>
#ifndef _WIN32
#include <unistd.h>
#endif
#include "owl-pwm.h"
#include "owl-comms.h"
#include "owl-cv.h"

using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
/********************************
* Initialise flags and variables
********************************/
    //flags to enable "case function"
    bool lookLeft = 0;
    bool lookRight = 0;
    bool crossEyed = 0;
    bool eyeRoll = 0;
    bool fullCircle = 0;
    bool chameleon = 0;
    bool eyePan = 0;

    //flags that monitor "case function" state
    bool eyeRollInit = 0;
    bool fullCircleInit = 0;
    bool fullCircleStepFlag = 0;    //changes direction of x pan


    //variables
    double eyeRollCount = 0;    //counts through sin function in 0.1 steps
    double fullCircleCount = 0; //counts through sin function in 0.1 steps
    int fullCircleStep = 21;    //defined as variable not a constant sign - (minimum X range value for both eyes[=670])/(Pi/step size [=0.1])
    int chameleonCount = 0;     //count value to only enter main chameleon functionality once every 10 loops of main to allow camera feed to update
    int eyePanStep = 10;

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

    // move servos to center of field
    CMDstream.str("");
    CMDstream.clear();
    CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
    CMD = CMDstream.str();
    string RxPacket= OwlSendPacket (u_sock, CMD.c_str());

    Mat Frame, Left, Right;

    //Open video feed
    string source = "http://10.0.0.10:8080/stream/video.mjpeg";
    VideoCapture cap (source);
    if (!cap.isOpened())
    {
        cout  << "Could not open the input video: " << source << endl;
        return -1;
    }

    //main program loop
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

        //Draw a circle in the middle of the left and right image (usefull for aligning both cameras)
        circle(Left,Point(Left.size().width/2,Left.size().height/2),10,Scalar(255,255,255),1);
        circle(Right,Point(Right.size().width/2,Right.size().height/2),10,Scalar(255,255,255),1);

        //Display left and right images
        imshow("Left",Left);
        imshow("Right", Right);

        //Read keypress and move the corresponding motor
        int key = waitKey(10);
        switch (key){         
        case 'w': //up
            Ry=Ry+5;
            break;
        case 's'://down
            Ry=Ry-5;
            break;
        case 'a'://left
            Rx=Rx-5;
            break;
        case 'd'://right
            Rx=Rx+5;
            break;
        case 'i': //up
            Ly=Ly-5;
            break;
        case 'k'://down
            Ly=Ly+5;
            break;
        case 'j'://left
            Lx=Lx-5;
            break;
        case 'l'://right
            Lx=Lx+5;
            break;
        case 'e'://right
            Neck=Neck+5;
            break;
        case 'q'://left
            Neck=Neck-5;
            break;

/********************************
* additional cases - 1 to 6
********************************/

        case '1'://left
            if((lookLeft == 0) & (lookRight == 0) & (crossEyed == 0) & (eyeRoll == 0) & (fullCircle == 0) & (chameleon == 0) & (eyePan == 0)){
                lookLeft = 1;   //enable flag, functionality coded below
            }
            break;
        case '2'://right
            if((lookLeft == 0) & (lookRight == 0) & (crossEyed == 0) & (eyeRoll == 0) & (fullCircle == 0) & (chameleon == 0) & (eyePan == 0)){
                lookRight = 1;
            }
            break;
        case '3'://cross eyed
            if((lookLeft == 0) & (lookRight == 0) & (crossEyed == 0) & (eyeRoll == 0) & (fullCircle == 0) & (chameleon == 0) & (eyePan == 0)){
                crossEyed = 1;
            }
            break;
        case '4'://eye roll
            if((lookLeft == 0) & (lookRight == 0) & (crossEyed == 0) & (eyeRoll == 0) & (fullCircle == 0) & (chameleon == 0) & (eyePan == 0)){
                eyeRoll = 1;
            }
            break;
        case '5'://full circle
            if((lookLeft == 0) & (lookRight == 0) & (crossEyed == 0) & (eyeRoll == 0) & (fullCircle == 0) & (chameleon == 0) & (eyePan == 0)){
                fullCircle = 1;
            }
            break;
        case '6'://chameleon
            if((lookLeft == 0) & (lookRight == 0) & (crossEyed == 0) & (eyeRoll == 0) & (fullCircle == 0) & (chameleon == 0) & (eyePan == 0)){
                chameleon = 1;
            }
            break;
        case '7'://eye pan
            if((lookLeft == 0) & (lookRight == 0) & (crossEyed == 0) & (eyeRoll == 0) & (fullCircle == 0) & (chameleon == 0) & (eyePan == 0)){
                eyePan = 1;
            }
            break;


/********************************
* case 9 - stop
********************************/
        case '9':
            //sets all flags to zero thus ceases execution
            crossEyed = 0;
            lookLeft = 0;
            lookRight = 0;
            eyeRoll = 0;
            fullCircle = 0;
            chameleon = 0;
            eyePan = 0;

            //reset function flags
            eyeRollCount = 0;
            eyeRollInit = 0;
            fullCircleCount = 0;
            fullCircleInit = 0;
            fullCircleStep = 21;
            fullCircleStepFlag = 0;
            break;

/********************************
* case 0 - stop and reset
********************************/
        case '0':
            //stop code (case 9)
            crossEyed = 0;
            lookLeft = 0;
            lookRight = 0;
            eyeRoll = 0;
            fullCircle = 0;
            chameleon = 0;
            eyePan = 0;
            eyeRollCount = 0;
            eyeRollInit = 0;
            fullCircleCount = 0;
            fullCircleInit = 0;
            fullCircleStep = 21;
            fullCircleStepFlag = 0;

            //Set servo positions to their center-points
            Rx = RxC; Lx = LxC;
            Ry = RyC; Ly = LyC;
            Neck= NeckC;

            break;

        }//case statement

/********************************************************************************
* case 1 - eyes left <- code that executes utilising the main loop if flag enabled
********************************************************************************/
        if(lookLeft == 1){

            //terminating condition
            if((Lx <= LxLm) | (Rx <= RxLm)){    //if an eye reaches its leftmost X limit, reset case 1 flag
                lookLeft=0;
            }else{

                //PWM
                Rx=Rx-20;   //step 20 to the left
                Lx=Lx-20;

            }//else
        }//if

/********************************
* case 2 - eyes right
********************************/
        if(lookRight == 1){

            //terminating condition
            if((Lx >= LxRm) | (Rx >= RxRm)){    //if an eye reaches its rightmost X limit, reset case 2 flag
                lookRight=0;
            }else{

                //PWM
                Rx=Rx+20;   //step 20 to the right
                Lx=Lx+20;

            }//else
        }//if

/********************************
* case 3 - cross eyed
********************************/
        if(crossEyed == 1){

            //terminating condition
            if((Lx >= LxRm) | (Rx <= RxLm)){    //if the left eye reaches its rightmost X limit or the right eye reaches its leftmost X limit, reset case 3 flag
                crossEyed=0;
            }else{

                //PWM
                Rx=Rx-10;   //step 10 to the left
                Lx=Lx+10;   //step 10 to the right

            }//else
        }//if

/********************************
* case 4 - eye roll
********************************/
        if(eyeRoll == 1){

            //initial state
            if(eyeRollInit == 0){ //on first loop of case 4 set servo positions to leftmost values
                Lx = LxLm;
                Rx = RxLm;
                eyeRollInit = 1;    //set flag high to disable setup code
            }

            //terminating condition
            if(eyeRollCount > 3.14){
                eyeRollCount = 0;  //reset flags on exit
                eyeRollInit = 0;
                eyeRoll = 0;

            }else{

                eyeRollCount = eyeRollCount+0.1;   //step through sin function by 0.1; ie sin(0.1), sin(0.2), sin(0.3)

                //PWM
                Rx=Rx+21; // step size = (minimum X range value for both eyes[=670])/(Pi/step size [=0.1])
                Lx=Lx+21;
                Ry = 330*sin(eyeRollCount)+RyC;    //stretch amplitude of sin wave to match range of PWM and shift values to superimpose the ranges
                Ly = 330*sin(-eyeRollCount)+LyC;   //eyeRollCount negative as motor mounted in opposite direction

             }//else
        }//if

/********************************
* case 5 - full circle
********************************/
        if(fullCircle == 1){

            //initial state
            if(fullCircleInit == 0){   //on first loop of case 5 set servo positions to leftmost values
                Lx = LxLm;
                Rx = RxLm;
                fullCircleInit = 1;    //set flag high to disable setup code
            }

            //terminating condition
            if(fullCircleCount > 2*3.14){
                fullCircleStep = 21;   //reset to positive X pan
                fullCircleCount = 0;   //rest flags
                fullCircleInit = 0;
                fullCircleStepFlag = 0;
                fullCircle = 0;

            }else{

                if((fullCircleCount > 3.14) & (fullCircleStepFlag == 0)){  //invert direction of the X pan at half the period of the sine wave
                    fullCircleStep = -21;      //set to negative X pan, for full circle
                    fullCircleStepFlag = 1;    //flag to ensure only change sign of step once
                }
                fullCircleCount = fullCircleCount+0.1; //step through sin function by 0.1; ie sin(0.1), sin(0.2), sin(0.3)

                //PWM
                Rx=Rx+fullCircleStep;
                Lx=Lx+fullCircleStep;
                Ry = 330*sin(fullCircleCount)+RyC;     //stretch amplitude of sin wave to match range of PWM and shift values to superimpose the ranges
                Ly = 330*sin(-fullCircleCount)+LyC;    //fullCircleCount negative as motor mounted in opposite direction

            }//else
        }//if

 /********************************
 * case 6 - chameleon
 ********************************/
        if(chameleon == 1){

            if (chameleonCount == 10){ //if count value 10, enter main chameleon functionality. Allows camera feed to update

                if(rand() % 3 == 0){  //condition to randomise whether action occurs
                    Rx = rand()%((RxRm - RxLm)+RxLm);  //generate random position variables within range of servos
                    Ry = rand()%((RyTm - RyBm)+RyBm);
                }

                if(rand() % 2 == 0){   //condition to randomise whether action occurs - not same condition as before!
                    Lx = rand()%((LxRm - LxLm)+LxLm);
                    Ly = rand()%((LyBm - LyTm)+LyTm);
                }

                chameleonCount = 0;    //reset count
            }else{
                chameleonCount++;  //increase count
            }
        }//if chameleon case


/********************************
* case 7 - eye pan
********************************/
        if(eyePan == 1){

            Rx=Rx+eyePanStep;   //pan
            Lx=Lx+eyePanStep;

            if((Lx <= LxLm) | (Rx <= RxLm)){    //if an eye reaches its leftmost X limit
                eyePanStep = eyePanStep*(-1);
            }else if((Lx >= LxRm) | (Rx >= RxRm)){    //if an eye reaches its rightmost X limit
                eyePanStep = eyePanStep*(-1);
            }//if
         }//if


        //update camera feed
        CMDstream.str("");
        CMDstream.clear();
        CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
        CMD = CMDstream.str();
        RxPacket= OwlSendPacket (u_sock, CMD.c_str());

    } // END cursor control loop

    // close windows down
    destroyAllWindows();


#ifdef _WIN32
    RxPacket= OwlSendPacket (u_sock, CMD.c_str());
    closesocket(u_sock);
#else
    OwlSendPacket (clientSock, CMD.c_str());
    close(clientSock);
#endif
    exit(0); // exit here for servo testing only
}
