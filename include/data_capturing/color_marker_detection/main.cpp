#include <opencv2/opencv.hpp>
#include "CMDetect.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <unistd.h>

using namespace std;

// Sample code for color marker detection

/************** TO BE SET BY THE USER **************************/

// Full pathname of the user parameter file.
#define USERPARSFILENAME   "/Users/manduchi/Developer/ColorMarker_OpenCV/CMUserPars.xml"

// Full pathname of the classifier parameter file
#define CLASSPARSFILENAME "/Users/manduchi/Developer/ColorMarker_OpenCV/DesignPars.xml"

/***************************************************************/

// Comment this to use the computer's camera instead of loading an image file
#define READ_FROM_FILE

#ifdef READ_FROM_FILE 

// Path name of iamge file to be loaded
#define INPUTIMAGE "/Users/manduchi/Developer/ColorMarker_OpenCV/MarkerImage0.png"

#endif

int main() {
    
    IplImage* img;
    IplImage* img2;

    int width,height;
    
    CMDetect theDetector(USERPARSFILENAME,CLASSPARSFILENAME);   //// <======== constructor
    cvNamedWindow( "Output", CV_WINDOW_NORMAL );

#ifdef READ_FROM_FILE
    img = cvLoadImage(INPUTIMAGE);
    width = img->width;
    height = img->height;
#else
    CvCapture* theCamera;    
    theCamera = cvCreateCameraCapture(0);
    width = (int)cvGetCaptureProperty(theCamera, CV_CAP_PROP_FRAME_WIDTH);
    height = (int)cvGetCaptureProperty(theCamera, CV_CAP_PROP_FRAME_HEIGHT);
#endif
    
    img2 = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
    

    while (1) {
        if( cvWaitKey(15)==27) break;

#ifdef READ_FROM_FILE
        img = cvLoadImage(INPUTIMAGE);
#else
        img = cvQueryFrame(theCamera);
#endif
        cvCopy(img,img2);
           //cvFlip(img2,img2,1);     /// <=== Try this in case the camera is swapping the images left-right
        
        // Access image data
        theDetector.AccessImage((uchar*)img2->imageData, img2->width, img2->height ,img2->widthStep);
        
        // Run detector
        if (theDetector.FindTarget()){
            ostringstream oss;
            string message;
            oss << theDetector.outValues.perm;
            message = "Permutation ID:" + oss.str();
            
            // Display permutation index
            CvFont theFont;
            cvInitFont(&theFont, CV_FONT_HERSHEY_PLAIN, 1., 1.);
            cvPutText(img2, message.c_str(), cvPoint(20, 20), &theFont, cvScalar(0,0,255));
            
            // Show the five points found
            cvEllipse(img2, cvPoint(theDetector.outValues.center.iX, theDetector.outValues.center.iY),
                      cvSize(5,5),
                      0., 0., 360., cvScalar(0,0,255),2,8);
            cvEllipse(img2, cvPoint(theDetector.outValues.top.iX, theDetector.outValues.top.iY),
                      cvSize(5,5),
                      0., 0., 360., cvScalar(0,0,255),2,8);
            cvEllipse(img2, cvPoint(theDetector.outValues.bottom.iX, theDetector.outValues.bottom.iY),
                      cvSize(5,5),
                      0., 0., 360., cvScalar(0,0,255),2,8);
            cvEllipse(img2, cvPoint(theDetector.outValues.right.iX, theDetector.outValues.right.iY),
                      cvSize(5,5),
                      0., 0., 360., cvScalar(0,0,255),2,8);
            cvEllipse(img2, cvPoint(theDetector.outValues.left.iX, theDetector.outValues.left.iY),
                      cvSize(5,5),
                      0., 0., 360., cvScalar(0,0,255),2,8);
            
        }
        
        cvShowImage("Original", img2 );
        
    }
    
    cvReleaseImage( &img2 );
#ifndef READ_FROM_FILE
    cvReleaseCapture(&theCamera);
#endif
    cvDestroyWindow( "Output" );
}