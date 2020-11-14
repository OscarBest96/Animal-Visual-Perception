#ifndef OWLCV_H
#define OWLCV_H

#endif // OWLCV_H

/* Phil Culverhouse
 *
 * Vision Processing for OWL camera system
 *  Currently provides Normalised Cross Correlation for template match
 *  uses opencv, assumes 3.1 or similar
 *  uses the Right eye for template source.
 * (c) Plymouth University, 2016
 */
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

struct OwlCorrel {
    Point Match;
    Mat Result;
};

Mat OWLtempl; // used in correlation
Rect target = Rect(320-32, 240-32, 64, 64); // target is at the centre of the camera FOV


//Given an image and a template, this function returns the best match for the templates position within the source image
OwlCorrel Owl_matchTemplate(Mat src, Mat templ){

    // Create the result matrix
    int result_cols =  src.cols - templ.cols + 1;
    int result_rows =  src.rows - templ.rows + 1;

    static OwlCorrel OWL;
    OWL.Result.create(result_rows, result_cols,  CV_32FC1);

    // Do the Matching and Normalize
    int match_method = 5; // CV_TM_CCOEFF_NORMED;
    matchTemplate(src, templ, OWL.Result, match_method);

    // Localizing the best match with minMaxLoc
    double minVal; double maxVal; Point minLoc; Point maxLoc;
    Point matchLoc;

    minMaxLoc( OWL.Result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

    // For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
    if( match_method  == TM_SQDIFF || match_method == TM_SQDIFF_NORMED ){
        OWL.Match = minLoc;
    }else{
        OWL.Match = maxLoc;
    }

    return OWL;
}


//Save a given number of images to a folder path, used to save calibration images
void OwlCalCapture(VideoCapture &cap, string Folder, int count){

    Mat Frame,Right,Left;

    for (int i=0;i<count;i++){

        //Display left and right streams, untill the user presses 's'.
        while(waitKey(10)!='s'){
            if (!cap.read(Frame))
            {
                cout<<"Could not open video stream"<<endl;
            }

            //flip input image as it comes in reversed
            Mat FrameFlpd;
            flip(Frame,FrameFlpd,1);

            // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG iamge
            Left= FrameFlpd(Rect(0, 0, 640, 480)); // using a rectangle
            Right=FrameFlpd(Rect(640, 0, 640, 480)); // using a rectangle

            imshow("Left",Left);
            imshow("Right",Right);
        }

        //create unique file name for each image
        string fnameR=(Folder + "/right" + to_string(i) + ".jpg");
        string fnameL=(Folder + "/left" +  to_string(i) + ".jpg");

        //save stereo pair to folder
        imwrite(fnameL, Left);
        imwrite(fnameR, Right);
        cout << "Saved " << i << " stereo pair" << Folder <<endl;
    }
}
