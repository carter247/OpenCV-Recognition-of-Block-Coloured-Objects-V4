//
//  main.cpp
//  Recognition of Block Coloured Object V4
//
//  Created by George Carter on 28/11/2017.
//  Copyright © 2017 George Carter. All rights reserved.
//



#include <sstream>
#include <fstream>
#include <string>
#include <iostream>
#include <string>
#include <stdio.h>
#include <cmath>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <armadillo>

using namespace std;
using namespace cv;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -//

// Definitions

#define PI 3.14159265   // Define PI

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -//

// Global Variables

Mat cameraFeed;     // Matrix storage for webcam feed
Mat HSV;            // Matrix storage for HSV image
Mat threshold1;     // Matrix storage for binary threshold image 1
Mat threshold2;     // Matrix storage for binary threshold image 2
Mat thresholdComb;  // Matrix storage for binary threshold image combination
Mat erodeElement;   // Matrix storage for eroded binary threshold image combination
Mat dilateElement;  // Matrix storage for dilated binary threshold image combination

bool found = false;

int upperLimit = 255;       // Set upper limit for trackbar
int H_MIN_1 = 0;
int H_MAX_1 = 255;
int S_MIN = 0;
int S_MAX = 255;
int V_MIN = 0;              // Initialise values for minimum & maximum values of Hue, Saturation & Value
int V_MAX = 255;
int H_MIN_2 = 0;
int H_MAX_2 = 255;

int KNOWN_WIDTH;            // Width of object in mm
int KNOWN_HEIGHT;           // Height of object in mm
int KNOWN_DEPTH;            // Depth of object in mm

float width = 0;            // Stores bounding box width in pixels
float height = 0;           // Stores bounding box height in pixels
float U;                    // X value of centre point in pixels
float V;                    // Y value of centre point in pixels
float focal = 772;          // Stores updated focal length, initially 700
float Yp;                   // Stores difference between object centre and frame centre along the y-axis

float X;                    // Objects X coordinate
float Y;                    // Objects Y coordinate
float Z;                    // Objects Z coordinate
float X1;                   // X cooridinate of Joint 2, 3, 4, 5 & 6 plane
float Y1;                   // Y cooridinate of Joint 2, 3, 4, 5 & 6 plane

int stage = 0;              // Move/Jump variable
int iter = 0;               // Iterater

const int FRAME_WIDTH = 640;    // Window width
const int FRAME_HEIGHT = 480;   // Window height
int idx = 1;                    // Camera Index. 0 for webcam, 1 for external.

char ch = 0;                // Store user input at the end of each scan
int CH = 0;                 // Integer value of user input

float link1 = 12.75;         // Length of link 1 in cm (Distance from ground to joint)
float link2 = 13.5;         // Length of link 2 in cm
float link3 = 9.8;          // Length of link 3 in cm
float link4 = 5.5;          // Length of link 4 in cm
float xlink5 = 3.0;         // Length of link 5 in cm parallel to x-axis
float ylink5 = 3.0;         // Length of link 5 in cm parallel to y-axis
float link6 = 4.5;          // Length of link 6 in cm

float objectVector;         // Vector magnitude of object from joint 2
float link23Min;            // Magnitude of link 2 + 3 when at min/max range
float link23Max;            // Magnitude of link 2 + 3 when both in home position

int joint1 = 90;            // Joint 1 resultant angle
int joint2 = 97;            // Joint 2 resultant angle
int joint3 = 95;            // Joint 3 resultant angle
int joint4 = 90;            // Joint 4 resultant angle
int joint5 = 80;            // Joint 5 resultant angle
int joint6 = 70;            // Joint 6 resultant angle

float theta2;               // Displacement of joint 2
float theta3;               // Displacement of joint 3
float theta4;               // Displacement of joint 4

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -//

// Functions

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -//

// Display Text to the Screen

void textHue(Mat &thresholdComb, int &H_MIN_1, int &H_MAX_1, int &H_MIN_2, int &H_MAX_2){
    stringstream textHue;
    
    if(H_MIN_2 != 0 || H_MAX_2 != 255){
        
        textHue << "Hue: " << H_MIN_1 << " - " << H_MAX_1 << " & " << H_MIN_2 << " - " << H_MAX_2;
    }
    else{
        
        textHue << "Hue: " << H_MIN_1 << " - " << H_MAX_1;
    }
    
    putText(thresholdComb, textHue.str(), Point(15, 30), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255), 1);
}   // Displays text to the screen

void textSaturation(Mat& thresholdComb, int& S_MIN, int& S_MAX){
    stringstream hue2;
    hue2 << "Saturation: " << S_MIN << " - " << S_MAX;
    putText(thresholdComb, hue2.str(), Point(15, 60), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255), 1);
}   // Displays text to the screen

void textValue(Mat& thresholdComb, int& V_MIN, int& V_MAX){
    stringstream textValue;
    textValue << "Value: " << V_MIN << " - " << V_MAX;
    putText(thresholdComb, textValue.str(), Point(15, 90), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255), 1);
}   // Displays text to the screen

void textObjectCentre(Mat &cameraFeed, float &U, float &V){
    int a = (int)U;
    int b = (int)V;
    stringstream textObjectCentre;
    textObjectCentre << "Object Location: [" << a << ", " << b << "] pxls";
    putText(cameraFeed, textObjectCentre.str(), Point(15, 30), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255), 1);
}   // Displays text to the screen

void textObjectDimensions(Mat &cameraFeed, float &width, float &height){
    int a = (int)width;
    int b = (int)height;
    stringstream textObjectDimensions;
    textObjectDimensions << "Object Dimensions: [" << a << ", " << b << "] pxls";
    putText(cameraFeed, textObjectDimensions.str(), Point(15, 60), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255), 1);
}   // Displays text to the screen

void textXCoordinate(Mat &cameraFeed, float &X){
    int a = (int)X;
    stringstream textXCoordinate;
    textXCoordinate << "X Coordinate: [" << a << "] mm";
    putText(cameraFeed, textXCoordinate.str(), Point(15, 90), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255), 1);
}   // Displays text to the screen

void textYCoordinate(Mat &cameraFeed, float &Y){
    int a = (int)Y;
    stringstream textYCoordinate;
    textYCoordinate << "Y Coordinate: [" << a << "] mm";
    putText(cameraFeed, textYCoordinate.str(), Point(15, 120), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255), 1);
}   // Displays text to the screen

void textZCoordinate(Mat &cameraFeed, float &Z){
    int a = (int)Z;
    stringstream textZCoordinate;
    textZCoordinate << "Z Coordinate: [" << a << "] mm";
    putText(cameraFeed, textZCoordinate.str(), Point(15, 150), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255), 1);
}   // Displays text to the screen

void textXCoordinate2(Mat &cameraFeed, float &X){
    int a = (int)X;
    stringstream textXCoordinate2;
    textXCoordinate2 << "X Coordinate: [" << a << "] mm";
    putText(cameraFeed, textXCoordinate2.str(), Point(15, 150), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255), 1);
}   // Displays text to the screen

void textYCoordinate2(Mat &cameraFeed, float &Y){
    int a = (int)Y;
    stringstream textYCoordinate2;
    textYCoordinate2 << "Y Coordinate: [" << a << "] mm";
    putText(cameraFeed, textYCoordinate2.str(), Point(15, 180), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255), 1);
}   // Displays text to the screen

void textZCoordinate2(Mat &cameraFeed, float &Z){
    int a = (int)Z;
    stringstream textZCoordinate2;
    textZCoordinate2 << "Z Coordinate: [" << a << "] mm";
    putText(cameraFeed, textZCoordinate2.str(), Point(15, 210), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255), 1);
}   // Displays text to the screen

void textObjectWidth(Mat &cameraFeed, int &KNOWN_WIDTH){
    stringstream textObjectWidth;
    textObjectWidth << "Object Width: [" << KNOWN_WIDTH << "] mm";
    putText(cameraFeed, textObjectWidth.str(), Point(15, 90), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255), 1);
}   // Displays text to the screen

void textObjectHeight(Mat &cameraFeed, int &KNOWN_HEIGHT){
    stringstream textObjectHeight;
    textObjectHeight << "Object Height: [" << KNOWN_HEIGHT << "] mm";
    putText(cameraFeed, textObjectHeight.str(), Point(15, 120), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255), 1);
}   // Displays text to the screen

void textFocal(Mat &cameraFeed, float focal){
    int a = (int)focal;
    stringstream textFocal;
    textFocal << "Focal: [" << a << "] mm";
    putText(cameraFeed, textFocal.str(), Point(15, 240), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255), 1);
}   // Displays text to the screen

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -//

// Object Detection

void setBlueValues(int& H_MIN_1, int& H_MAX_1, int& H_MIN_2, int& H_MAX_2, int& S_MIN, int& S_MAX, int& V_MIN, int& V_MAX){
    
    H_MIN_1 = 80;
    H_MAX_1 = 220;
    H_MIN_2 = 0;
    H_MAX_2 = 255;
    S_MIN = 100;
    S_MAX = 255;
    V_MIN = 100;
    V_MAX = 255;
}   // Set values for blue object

void setGreenValues(int& H_MIN_1, int& H_MAX_1, int& H_MIN_2, int& H_MAX_2, int& S_MIN, int& S_MAX, int& V_MIN, int& V_MAX){
    
    H_MIN_1 = 25;
    H_MAX_1 = 140;
    H_MIN_2 = 0;
    H_MAX_2 = 255;
    S_MIN = 100;
    S_MAX = 255;
    V_MIN = 100;
    V_MAX = 255;
}   // Set values for green object

void setRedValues(int& H_MIN_1, int& H_MAX_1, int& H_MIN_2, int& H_MAX_2, int& S_MIN, int& S_MAX, int& V_MIN, int& V_MAX){
    
    H_MIN_1 = 0;
    H_MAX_1 = 45;
    H_MIN_2 = 200;
    H_MAX_2 = 255;
    S_MIN = 100;
    S_MAX = 255;
    V_MIN = 100;
    V_MAX = 255;
}   // Set values for red object

void setYellowValues(int& H_MIN_1, int& H_MAX_1, int& H_MIN_2, int& H_MAX_2, int& S_MIN, int& S_MAX, int& V_MIN, int& V_MAX){
    
    H_MIN_1 = 0;
    H_MAX_1 = 80;
    H_MIN_2 = 0;
    H_MAX_2 = 255;
    S_MIN = 155;
    S_MAX = 255;
    V_MIN = 28;
    V_MAX = 255;
}   // Set values for yellow object

void on_trackbar( int, void* ){
    
    // This function gets called whenever trackbar position is changed
}

void createTrackbarsRed(int &H_MIN_1, int &H_MAX_1, int &H_MIN_2, int &H_MAX_2, int &S_MIN, int &S_MAX, int &V_MIN, int &V_MAX, int &upperLimit){
    
    namedWindow("Trackbars", CV_WINDOW_NORMAL);     // Create window for trackbars
    resizeWindow("Trackbars", 640, 260);            // Resize window
    
    createTrackbar( "Hue Min 1", "Trackbars", &H_MIN_1, upperLimit, on_trackbar );
    createTrackbar( "Hue Max 1", "Trackbars", &H_MAX_1, upperLimit, on_trackbar );
    createTrackbar( "Hue Min 2", "Trackbars", &H_MIN_2, upperLimit, on_trackbar );
    createTrackbar( "Hue Max 2", "Trackbars", &H_MAX_2, upperLimit, on_trackbar );
    createTrackbar( "Saturation Min", "Trackbars", &S_MIN, upperLimit, on_trackbar );
    createTrackbar( "Saturation Max", "Trackbars", &S_MAX, upperLimit, on_trackbar );
    createTrackbar( "Value Min", "Trackbars", &V_MIN, upperLimit, on_trackbar );
    createTrackbar( "Value Max", "Trackbars", &V_MAX, upperLimit, on_trackbar );
    
    // Create trackbars and insert them into window, 5 parameters are:
    // 1. The title of the trackbar
    // 2. The title of the window
    // 3. The variable that is changing when the trackbar is moved
    // 4. The max value the trackbar can move to
    // 5. The function that is called whenever the trackbar is moved
    
}   // Trackbar is called if object is red

void createTrackbars(int &H_MIN_1, int &H_MAX_1, int &S_MIN, int &S_MAX, int &V_MIN, int &V_MAX, int &upperLimit){
    
    namedWindow("Trackbars", CV_WINDOW_NORMAL);
    resizeWindow("Trackbars", 640, 210);
    
    createTrackbar( "Hue Min", "Trackbars", &H_MIN_1, upperLimit, on_trackbar );
    createTrackbar( "Hue Max", "Trackbars", &H_MAX_1, upperLimit, on_trackbar );
    createTrackbar( "Saturation Min", "Trackbars", &S_MIN, upperLimit, on_trackbar );
    createTrackbar( "Saturation Max", "Trackbars", &S_MAX, upperLimit, on_trackbar );
    createTrackbar( "Value Min", "Trackbars", &V_MIN, upperLimit, on_trackbar );
    createTrackbar( "Value Max", "Trackbars", &V_MAX, upperLimit, on_trackbar );
    
}   // Trackbar is called if object is not red

void createTrackbarsObject(int &KNOWN_WIDTH, int &KNOWN_HEIGHT, int &upperLimit){
    
    namedWindow("Dimensions", CV_WINDOW_NORMAL);
    resizeWindow("Dimensions", 640, 90);
    
    createTrackbar( "Width", "Dimensions", &KNOWN_WIDTH, upperLimit, on_trackbar );
    createTrackbar( "Height", "Dimensions", &KNOWN_HEIGHT, upperLimit, on_trackbar );
    
}   // Trackbar for object dimensions

void morphOps(Mat &erodeElement, Mat &dilateElement, Mat &thresholdComb){
    
    erodeElement = getStructuringElement(MORPH_RECT,Size(5,5));
    dilateElement = getStructuringElement(MORPH_RECT,Size(5,5));
    // Returns a structuring element of the size denoted
    
    erode(thresholdComb,thresholdComb,erodeElement);
    erode(thresholdComb,thresholdComb,erodeElement);
    // Erodes the specified structuring element
    
    dilate(thresholdComb,thresholdComb,dilateElement);
    dilate(thresholdComb,thresholdComb,dilateElement);
    // Dilates the specified structuring element
    
    GaussianBlur(thresholdComb, thresholdComb, Size(9, 9), 4, 4);
    // Blurs an image using Gaussian Blur
}

void objectParameters(Mat &cameraFeed, Mat &thresholdComb, float &U, float &V, float &width, float &height, int &KNOWN_WIDTH, int &KNOWN_HEIGHT, float &focal){
    
    vector<vector<Point> > contours;    // Stores the X, Y co-ordinates of contours in a vector within a vector
    vector<Vec4i> hierarchy;            // Stores image topology in a 4D vector within a vector
    vector<Rect> objectBox;             // Stores contours of object in a vector
    
    findContours(thresholdComb, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
    // Finds the contours of the binary image
    
    if(hierarchy.empty()){
        
        width = 0;
        height = 0;
        
        imshow("Camera Feed", cameraFeed);      // Show camera feed
        moveWindow("Camera Feed", 40, 0);
        
    } // If no contours are found then just show camera feed and reset box width & height to zero
    
    if(!hierarchy.empty()){
        
        vector<Moments> mu(contours.size());        // Creates a vector using as many points as there are contour points
        
        for(size_t i = 0; i < contours.size(); i++){
            
            mu[i] = moments(contours[i], false );
            
        }   // Calculates all the moments and stores in mu[i]
        
        vector<Point2f> mc(contours.size());    // Creates a vector of floating point using as many points as there are contour points
        
        for(size_t i = 0; i < contours.size(); i++){
            
            mc[i] = Point2f( static_cast<float>(mu[i].m10/mu[i].m00) , static_cast<float>(mu[i].m01/mu[i].m00));
        }   // Calculates the mass centre using the moments
        
        Point2f centre(0,0);    // Creates a floating point row vector to store X & Y value of object centre
        
        for( int i = 0; i < mc.size(); i++ ){
            
            centre += mc[i];
            centre.x /= mc.size();
            centre.y /= mc.size();
        }   // Takes the centre of mass and stores pixel value in row vector
        
        Point2i objectCentre = centre;          // Creates an integer row vector to store X & Y value of object centre
        U = objectCentre.x;                     // Stores X value of centre point in U
        V = objectCentre.y;                     // Stores Y value of centre point in V
        textObjectCentre(cameraFeed, U, V);     // Displays pixel values of object centre to screen
        
        for(size_t i = 0; i< contours.size(); i++){
            
            Rect bBox;                                          // Declaring bBox as a rectangle object
            bBox = boundingRect(contours[i]);                   // Stores the up-right bounding rectangle from the contours
            width = bBox.width;                                 // Stores width of the bounding box
            height = bBox.height;                               // Stores height of the bounding box
            textObjectDimensions(cameraFeed, width, height);    // Displays bounding box dimensions to screen
            
            objectBox.push_back(bBox);      // Passes the up-right bounding rectangle dimensions into objectBox
            
            Scalar color(50, 255, 255);     // Defines color of contours
            drawContours(cameraFeed, contours, (int)i, color, 2, 8, hierarchy, 0, Point()); // Draws contours around object
            Scalar colourBox(150, 50, 50);   // Defines color of boxed outline
            rectangle(cameraFeed, objectBox[i], colourBox, 2);  // Draws a box around the object
            circle(cameraFeed, mc[i], 4, color, -2, 4, 0 );     // Draws a circle at the centre of the object
        }
    }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -//

// Program Functionality

inline bool doesFileExist(const string& name){
    ifstream file(name);
    if(!file)            // If the file was not found, then return false
        return false;
    else                 // If the file was found, then return true
        return true;
}

void saveObject(int &H_MIN_1, int &H_MAX_1, int &H_MIN_2, int &H_MAX_2, int &S_MIN, int &S_MAX, int &V_MIN, int &V_MAX, int &KNOWN_WIDTH, int &KNOWN_HEIGHT, int &iter, bool &found){
    
    arma::umat A(5, 2);     // Create a 5 x 2 matrix
    A.zeros();              // Fill it with zeros
    
    A.at(0, 0) = H_MIN_1;
    A.at(0, 1) = H_MAX_1;
    A.at(1, 0) = S_MIN;
    A.at(1, 1) = S_MAX;         // Pass the variables of Hue, Saturation, Value and known width & height into the matrix
    A.at(2, 0) = V_MIN;
    A.at(2, 1) = V_MAX;
    A.at(3, 0) = H_MIN_2;
    A.at(3, 1) = H_MAX_2;
    A.at(4, 0) = KNOWN_WIDTH;
    A.at(4, 1) = KNOWN_HEIGHT;
    
    found = doesFileExist("Object1.txt");       // Search if file exists, if file doesn't exist then save under that name.
    
    int object = 1;     // Iterator used to define name of object being saved
    
    if(!found && object == 1){
        
        A.save("Object1.txt", arma::arma_ascii);        // Save matrix as object 1
        imwrite("Object1_Picture.JPEG", cameraFeed);    // Save respective picture of object
        iter = 1;                                       // Iter will write a message to the screen. See storedObjectStatus()
    }
    if(found && object == 1){
        
        object = 2;
        found = false;
        found = doesFileExist("Object2.txt");           // If object 1 exists check for object 2
    }
    if(!found && object == 2){
        
        A.save("Object2.txt", arma::arma_ascii);        // Save matrix as object 2
        imwrite("Object2_Picture.JPEG", cameraFeed);    // Save respective picture of object
        iter = 31;                                      // Iter will write a message to the screen. See storedObjectStatus()
    }
    if(found && object == 2){
        
        object = 3;
        found = false;
        found = doesFileExist("Object3.txt");           // If object 2 exists check for object 3
    }
    if(!found && object == 3){
        
        A.save("Object3.txt", arma::arma_ascii);        // Save matrix as object 3
        imwrite("Object3_Picture.JPEG", cameraFeed);    // Save respective picture of object
        iter = 61;                                      // Iter will write a message to the screen. See storedObjectStatus()
    }
    if(found && object == 3){
        
        object = 4;
        found = false;
        found = doesFileExist("Object4.txt");           // If object 3 exists check for object 4
    }
    if(!found && object == 4){
        
        A.save("Object4.txt", arma::arma_ascii);        // Save matrix as object 4
        imwrite("Object4_Picture.JPEG", cameraFeed);    // Save respective picture of object
        iter = 91;                                      // Iter will write a message to the screen. See storedObjectStatus()
    }
    if(found && object == 4){
        
        object = 5;
        found = false;
        found = doesFileExist("Object5.txt");           // If object 4 exists check for object 5
    }
    if(!found && object == 5){
        
        A.save("Object5.txt", arma::arma_ascii);        // Save matrix as object 5
        imwrite("Object5_Picture.JPEG", cameraFeed);    // Save respective picture of object
        iter = 121;                                     // Iter will write a message to the screen. See storedObjectStatus()
    }
    if(found && object == 5){
        
        object = 6;
        found = false;
        found = doesFileExist("Object6.txt");           // If object 5 exists check for object 6
    }
    if(!found && object == 6){
        
        A.save("Object6.txt", arma::arma_ascii);        // Save matrix as object 6
        imwrite("Object6_Picture.JPEG", cameraFeed);    // Save respective picture of object
        iter = 151;                                     // Iter will write a message to the screen. See storedObjectStatus()
    }
    if(found && object == 6){
        
        object = 7;
        found = false;
        found = doesFileExist("Object7.txt");           // If object 6 exists check for object 7
    }
    if(!found && object == 7){
        
        A.save("Object7.txt", arma::arma_ascii);        // Save matrix as object 7
        imwrite("Object7_Picture.JPEG", cameraFeed);    // Save respective picture of object
        iter = 181;                                     // Iter will write a message to the screen. See storedObjectStatus()
    }
    if(found && object == 7){
        
        object = 8;
        found = false;
        found = doesFileExist("Object8.txt");           // If object 7 exists check for object 8
    }
    if(!found && object == 8){
        
        A.save("Object8.txt", arma::arma_ascii);        // Save matrix as object 8
        imwrite("Object8_Picture.JPEG", cameraFeed);    // Save respective picture of object
        iter = 211;                                     // Iter will write a message to the screen. See storedObjectStatus()
    }
    if(found && object == 8){
        
        object = 9;
        found = false;
        found = doesFileExist("Object9.txt");           // If object 8 exists check for object 9
    }
    if(!found && object == 9){
        
        A.save("Object9.txt", arma::arma_ascii);        // Save matrix as object 9
        imwrite("Object9_Picture.JPEG", cameraFeed);    // Save respective picture of object
        iter = 241;                                     // Iter will write a message to the screen. See storedObjectStatus()
    }
    if(found && object == 9){
        
        iter = 271;                                     // Iter will write a message to the screen. See storedObjectStatus()
    }
    
    object = 1;
}

void storedObjectStatus(int &iter){
    
    if(1 <= iter && iter <= 29){
        
        putText(cameraFeed, "File Stored as Object 1", Point(50, 240), FONT_HERSHEY_DUPLEX, 1.5, CV_RGB(255, 255, 255));
        iter++;
    }
    if(31 <= iter && iter <= 59){
        
        putText(cameraFeed, "File Stored as Object 2", Point(50, 240), FONT_HERSHEY_DUPLEX, 1.5, CV_RGB(255, 255, 255));
        iter++;
    }
    if(61 <= iter && iter <= 89){
        
        putText(cameraFeed, "File Stored as Object 3", Point(50, 240), FONT_HERSHEY_DUPLEX, 1.5, CV_RGB(255, 255, 255));
        iter++;
    }
    if(91 <= iter && iter <= 119){
        
        putText(cameraFeed, "File Stored as Object 4", Point(50, 240), FONT_HERSHEY_DUPLEX, 1.5, CV_RGB(255, 255, 255));
        iter++;
    }
    if(121 <= iter && iter <= 149){
        
        putText(cameraFeed, "File Stored as Object 5", Point(50, 240), FONT_HERSHEY_DUPLEX, 1.5, CV_RGB(255, 255, 255));
        iter++;
    }
    if(151 <= iter && iter <= 179){
        
        putText(cameraFeed, "File Stored as Object 6", Point(50, 240), FONT_HERSHEY_DUPLEX, 1.5, CV_RGB(255, 255, 255));
        iter++;
    }
    if(181 <= iter && iter <= 209){
        
        putText(cameraFeed, "File Stored as Object 7", Point(50, 240), FONT_HERSHEY_DUPLEX, 1.5, CV_RGB(255, 255, 255));
        iter++;
    }
    if(211 <= iter && iter <= 239){
        
        putText(cameraFeed, "File Stored as Object 8", Point(50, 240), FONT_HERSHEY_DUPLEX, 1.5, CV_RGB(255, 255, 255));
        iter++;
    }
    if(241 <= iter && iter <= 269){
        
        putText(cameraFeed, "File Stored as Object 9", Point(50, 240), FONT_HERSHEY_DUPLEX, 1.5, CV_RGB(255, 255, 255));
        iter++;
    }
    if(271 <= iter && iter <= 299){
        
        putText(cameraFeed, "Max No of Files Stored", Point(50, 240), FONT_HERSHEY_DUPLEX, 1.5, CV_RGB(255, 255, 255));
        iter++;
    }
    if(301 <= iter && iter <= 329){
        
        iter++;
    }
    if(331 <= iter && iter <= 359){
        
        putText(cameraFeed, "Object Too Close", Point(100, 280), FONT_HERSHEY_DUPLEX, 1.5, CV_RGB(255, 255, 255));
        iter++;
    }
    
    if(361 <= iter && iter <= 389){
        
        putText(cameraFeed, "Object Too Far", Point(110, 280), FONT_HERSHEY_DUPLEX, 1.5, CV_RGB(255, 255, 255));
        iter++;
    }
}

void checkNumberOfObjects(bool &found){
    
    int object = 1;
    
    found = doesFileExist("Object1.txt");
    
    if(found && object == 1){
        
        putText(cameraFeed, "1: Object 1", Point(15, 60), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
    }   // If object 1 exists, display to user
    
    object = 2;
    found = doesFileExist("Object2.txt");
    
    if(found && object == 2){
        
        putText(cameraFeed, "2: Object 2", Point(15, 90), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
    }   // If object 2 exists, display to user
    
    object = 3;
    found = doesFileExist("Object3.txt");
    
    if(found && object == 3){
        
        putText(cameraFeed, "3: Object 3", Point(15, 120), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
    }   // If object 3 exists, display to user
    
    object = 4;
    found = doesFileExist("Object4.txt");
    
    if(found && object == 4){
        
        putText(cameraFeed, "4: Object 4", Point(15, 150), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
    }   // If object 4 exists, display to user
    
    object = 5;
    found = doesFileExist("Object5.txt");
    
    if(found && object == 5){
        
        putText(cameraFeed, "5: Object 5", Point(15, 180), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
    }   // If object 5 exists, display to user
    
    object = 6;
    found = doesFileExist("Object6.txt");
    
    if(found && object == 6){
        
        putText(cameraFeed, "6: Object 6", Point(15, 210), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
    }   // If object 6 exists, display to user
    
    object = 7;
    found = doesFileExist("Object7.txt");
    
    if(found && object == 7){
        
        putText(cameraFeed, "7: Object 7", Point(15, 240), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
    }   // If object 7 exists, display to user
    
    object = 8;
    found = doesFileExist("Object8.txt");
    
    if(found && object == 8){
        
        putText(cameraFeed, "8: Object 8", Point(15, 270), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
    }   // If object 8 exists, display to user
    
    object = 9;
    found = doesFileExist("Object9.txt");
    
    if(found && object == 9){
        
        putText(cameraFeed, "9: Object 9", Point(15, 300), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
    }   // If object 9 exists, display to user
}

void deleteObject(const char *name){
    
    remove(name);
}   // Deletes selected file

void loadObject(string name){
    
    arma::umat A(5, 2);     // Creates a 5 x 2 matrix
    A.zeros();              // Fills it with zeros
    
    A.load(name);           // Load matrix of a specified name
    
    H_MIN_1 = A.at(0, 0);
    H_MAX_1 = A.at(0, 1);
    S_MIN = A.at(1, 0);
    S_MAX = A.at(1, 1);
    V_MIN = A.at(2, 0);         // Passes values from loaded matrix to its respective variables
    V_MAX = A.at(2, 1);
    H_MIN_2 = A.at(3, 0);
    H_MAX_2 = A.at(3, 1);
    KNOWN_WIDTH = A.at(4, 0);
    KNOWN_HEIGHT = A.at(4, 1);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -//

// Control Algorithm

float updateFocal(float &X){
    
    if(X > 0){
        
        return (0.2537 * X) + 658.7;
    }   // If X coordinate is known update focal
    
    else{
        
        return 772;
    }   // Otherwise set to 772
}

int getXCoordinate(float &focal, int &KNOWN_HEIGHT, int & KNOWN_WIDTH, float &height, float &width){
    
    if(height == 0 || width == 0){
        
        return 0;
    }   // If no object has been detected then return zero
    
    float a = (focal - 600) / 100;
    width = width * (0.025 * pow(a, 2) + 1);
    height = height * (0.025 * pow(a, 2) + 1);
    
    return ( (((focal * KNOWN_HEIGHT) / height) + ((focal * KNOWN_WIDTH) / width)) / 2);
}   // Return X coordinate

int getYCoordinate(float &X, float &focal, float &U){
    
    float Xp = 320 - U;   // Centre of frame (X-axis) minus X value of centre point, both in pixels
    
    float a = X / 100;
    Xp = Xp * (0.05 * pow(a, 2) + 1);
    
    return (X * Xp) / focal;
    
}   // Return Y coordinate

int getZCoordinate(float &X, float &focal, float &V){
    
    float Yp = 240 - V;   // Centre of frame (Y-axis) minus Y value of centre point, both in pixels
    
    return (X * Yp) / focal;
}   // Returns Z coordinate

void rotateCoordinates(float &X, float &Y, float &Z){
    
    double x = X;
    double y = Y;
    double z = Z;
    double theta = -0.7853981634;
    double alpha = cos(theta);
    double beta = -sin(theta);
    double delta = sin(theta);
    double epsilon = cos(theta);
    
    arma::vec A = { x, y, z };
    arma::mat B = { {alpha, beta, 0}, {delta, epsilon, 0}, {0, 0, 1} };
    arma::vec C = B*A;
    
    X = C.at(0);
    Y = C.at(1);
    Z = C.at(2);
}

void translateCoordinates(float &X, float &Y, float &Z, float &U){
    
    int shiftX = 13;
    int shiftY = 358;
    int shiftZ = 73;
    
    X = X + shiftX;
    Y = Y + shiftY;
    Z = shiftZ + Z;
}

float calculateX1(float &X, float &Y, float &link4, float &xlink5, float &link6){
    
    return sqrt(pow(X, 2) + pow(Y, 2)) - link4 - xlink5 - link6;
} // Calculates X value for X-Y plane (1)

float calculateY1(float &Z, float &link1, float &ylink5){
    
    return Z - link1 - ylink5;
} // Calculates Y value for X-Y plane (1)

float calculateMagnitude(float &a, float &b){
    
    return sqrt(pow(a, 2) + pow(b, 2));
} // Calculates magintude of a vector

void checkObjectDistance(float &objectVector, float &link23Min, float &link23Max, float &X, float &Y, float &Z, float &X1, float &Y1, float &link1, float &link4, float &xlink5, float &ylink5, float &link6){
    
    if(objectVector < link23Min) {
        
        iter = 331;
    }   // The object distance needs to be greater than the hypotenuse of link 2 & 3 when at min range
    
    if(objectVector > link23Max){
        
        iter = 361;
    }   // The objects distance needs to be less than the magnitude of link 2 and 3 when at max range
}

void checkJointAngles(int &joint1, int &joint2, int &joint3, int &joint4, int &joint5, int &joint6){
    
    const int JOINT_1_LOWER_LIMIT = 0;
    const int JOINT_1_UPPER_LIMIT = 180;
    const int JOINT_2_LOWER_LIMIT = 80;
    const int JOINT_2_UPPER_LIMIT = 160;
    const int JOINT_3_LOWER_LIMIT = 0;
    const int JOINT_3_UPPER_LIMIT = 180;
    const int JOINT_4_LOWER_LIMIT = 0;
    const int JOINT_4_UPPER_LIMIT = 180;
    const int JOINT_5_LOWER_LIMIT = 0;
    const int JOINT_5_UPPER_LIMIT = 180;
    const int JOINT_6_LOWER_LIMIT = 70;
    const int JOINT_6_UPPER_LIMIT = 140;
    
    if(joint1 < JOINT_1_LOWER_LIMIT || JOINT_1_UPPER_LIMIT < joint1){
        
        cout << "Error computing angle. Joint 1: " << joint1 << endl;
    }
    if(joint2 < JOINT_2_LOWER_LIMIT || JOINT_2_UPPER_LIMIT < joint2){
        
        cout << "Error computing angle. Joint 2: " << joint2 << endl;
    }
    if(joint3 < JOINT_3_LOWER_LIMIT || JOINT_3_UPPER_LIMIT < joint3){
        
        cout << "Error computing angle. Joint 3: " << joint3 << endl;
    }
    if(joint4 < JOINT_4_LOWER_LIMIT || JOINT_4_UPPER_LIMIT < joint4){
        
        cout << "Error computing angle. Joint 4: " << joint4 << endl;
    }
    if(joint5 < JOINT_5_LOWER_LIMIT || JOINT_5_UPPER_LIMIT < joint5){
        
        cout << "Error computing angle. Joint 5: " << joint5 << endl;
    }
    if(joint6 < JOINT_6_LOWER_LIMIT || JOINT_6_UPPER_LIMIT < joint6){
        
        cout << "Error computing angle. Joint 6: " << joint6 << endl;
    }
    
}

void saveJointAngles(int joint1, int joint2, int joint3, int joint4, int joint5, int joint6, int &KNOWN_WIDTH, int &KNOWN_HEIGHT){
    
    string name = "jointangles.txt";    // Title of file
    
    ofstream outStream;
    outStream.open(name);               // Open the named file
    
    if(outStream){
        
        outStream << joint1;
        outStream << "n";
        outStream << joint2;
        outStream << "n";
        outStream << joint3;
        outStream << "n";
        outStream << joint4;            // Save all the joint angles seperated by an 'n'. See serial comms notes in user manual
        outStream << "n";
        outStream << joint5;
        outStream << "n";
        outStream << joint6;
        outStream << "n";
        outStream << KNOWN_WIDTH;
        outStream << "n";
        outStream << KNOWN_HEIGHT;
        outStream << "n";
        outStream.close();
    }
    
    else{
        
        cout << "Couldn't find file" << "\n";
    }
}   // Saves joint angles into text file

int calculateJoint1(float &X, float &Y){
    
#define PI 3.14159265                 // Define PI
    
    double x = X;                     // Convert X coordinate to double to perform mathematical operations
    double y = Y;                     // Convert Y coordinate to double to perform mathematical operations
    arma::vec A = { x, y };           // Object Vector
    arma::vec B = { 0.0, -1.0 };      // Joint 1 set point
    arma::vec C = normalise(A);       // Object vector normalised
    double D = dot(B, C);             // Dot product of normalised vector and home vector
    double E = acos(D);               // Result in radians
    double F = E*180/PI;           // Result in degrees
    
    if(F < 90){
        
        double G = (3.5 * exp(F/50)) - 15;
        return F - G;
    }
    if(F > 90){
        
        double G = (20.25 * exp(- (F - 90) / 50)) - 14;
        return F + G;
    }
    else{
        
        return 90;
    }
    
} // Calculates Joint 1. See control algorithm notes for explanation

int calculateJoint2(float &X1, float &Y1, float &link2, float &link3, float &theta2, float &theta3){
    
    int joint2Home = 90;    // Joint 2 set point
    float alpha, beta;
    
    beta = atan2(Y1, X1)*180/PI;            // Displacement angle
    
    alpha = atan2( link3*( sin(theta3*PI/180) ), link2 + link3*( cos(theta3*PI/180) ) ) *180/PI;
    // Displacement angle from beta to theta2
    
    theta2 = beta + alpha;          // theta2 is the displacement angle from the X-axis
    
    return joint2Home + theta2;   // Resultant angle for joint 2
    
} // Calculates Joint 2. See control algorithm notes for explanation

int calculateJoint3(float &X1, float &Y1, float &link2, float &link3, float &theta3){
    
    int joint3Home = 90;             // Joint 3 set point
    float a, b, d, d1, phi;          // Variables used in calculations
    
    a = pow(link2, 2) + pow(link3, 2);      // Magnitude of link 2 & 3
    b = pow(X1, 2) + pow(Y1, 2);            // Magnitude of X & Y
    
    if(X1 == 0 || Y1 == 0){
        
        d = -1;
    }   // Compensates for computational error when X1 or Y1 is zero
    
    else if(Y1 < 0){
        
        Y1 = -1*Y1;
        d = (a - b)/(2 * link2 * link3);
        Y1 = -1*Y1;
    }   // Compensates for computational error when Y is negative
    
    else{
        
        d = (a - b)/(2 * link2 * link3);
    }   // Equation to calculate the X value to 'phi'
    
    d1 = -1 * sqrt(1-pow(d, 2));        // Equation to calculate the Y value to 'phi'
    
    if(d1 == -0){
        
        d1 = 0;
    }   // Compensate for computational error when 'd1' is -0
    
    phi = atan2(d1, d)*180/PI;          // phi is the opposite angle to theta3
    
    theta3 = 180 - phi;                 // theta3 is the displacement angle of joint 3
    
    if(theta3 >=0 && theta3 <= 180){
        
        return joint3Home + theta3;
    }
    
    else{
        
        theta3 = 360 - theta3;
        return joint3Home + theta3;
    }   // Compensates for computational error when theta is greater than 180 degrees
    
} // Calculates Joint 3. See control algorithm notes for explanation

int calculateJoint4(float &theta2, float &theta3, float &theta4){
    
    int joint4Home = 90;            // Joint 4 set point
    
    theta4 = theta3 - theta2;
    
    return joint4Home - theta4;     // Theta4 is the displacement of joint 4
    
} // Calculates Joint 4. See control algorithm notes for explanation

void controlAlgorithm(float &X, float &Y, float &Z, float &X1, float &Y1, float &link1, float &link2, float &link3, float &link4, float &xlink5, float &ylink5, float &link6, float &objectVector, float &link23Min, float &link23Max, int &iter, int &stage){
    
    X = X/10;
    Y = Y/10;        // Convert from mm to cm
    Z = Z/10;
    
    X1 = calculateX1(X, Y, link4, xlink5, link6);       // Calculates X1
    Y1 = calculateY1(Z, link1, ylink5);                 // Calculates Y1
    
    objectVector = calculateMagnitude(X1, Y1);          // Calculates magnitude of X1 & Y1
    link23Min = calculateMagnitude(link2, link3);       // Calculates magnitude of Link2 & Link3
    link23Max = link2 + link3;                          // Calculates max length when Link2 & Link3 form a straight line
    
    checkObjectDistance(objectVector, link23Min, link23Max, X, Y, Z, X1, Y1, link1, link4, xlink5, ylink5, link6);
    // Checks objects distance and compares to parameters
    
    if(iter > 330 && iter < 389){
        
        return;
    }   // If the object is too close then return and user will be alerted from checkObjectDistance() function
    
    else{
        
        if(stage == 222){
            
            stage = 2221;
        }
        
        if(stage == 40){
            
            stage = 400;
        }
        
        if(stage == 41){
            
            stage = 401;
        }
        
        joint1 = calculateJoint1(X, Y);                                     // Calculates joint 1
        joint3 = calculateJoint3(X1, Y1, link2, link3, theta3);             // Calculates joint 3
        joint2 = calculateJoint2(X1, Y1, link2, link3, theta2, theta3);     // Calculates joint 2
        joint4 = calculateJoint4(theta2, theta3, theta4);                   // Calculates joint 4
        joint5 = 80;                                                        // Joint 5 always set at 80
        joint6 = 70;                                                        // Joint 6 function TBD
        checkJointAngles(joint1, joint2, joint3, joint4, joint5, joint6);   // Check angles are within range
        saveJointAngles(joint1, joint2, joint3, joint4, joint5, joint6, KNOWN_WIDTH, KNOWN_HEIGHT);    // Save angles to txt file
        system("open Serial2Arm.stc");                                      // Open Serial Comms program 'CoolTerm'
    }
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

// Main

int main(){
    
    VideoCapture capture;         // Video capture object to acquire webcam feed
    capture.open(idx);            // Open capture object at location zero (default location for webcam)
    
    capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);           // Set the frame width
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);         // Set the frame height
    
    while(ch != 'e' && ch != 'E'){          //Press 'E' or 'e' to exit program
        
        capture.read(cameraFeed);           // Store image to matrix "cameraFeed"
        
        if(!capture.read(cameraFeed)){
            
            return -1;          // Exit protocol needs TBD
        }
        
        if(stage == 0){
            
            putText(cameraFeed, "1: Save", Point(15, 30), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "2: Load", Point(15, 60), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "3: Delete", Point(15, 90), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "4: Play", Point(15, 120), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            // Display text to the screen
            
            switch(ch){
                    
                case 49:
                    
                    stage = 1;
                    ch = 0;             // If '1' is pressed move to stage 1
                    break;
                    
                case 50:
                    
                    stage = 2;
                    ch = 0;             // If '2' is pressed move to stage 2
                    break;
                    
                case 51:
                    
                    stage = 3;
                    ch = 0;             // If '3' is pressed move to stage 3
                    break;
                    
                case 52:
                    
                    stage = 4;
                    ch = 0;             // If '4' is pressed move to stage 4
                    break;
                    
            }
        }
        
        if(stage == 1){
            
            putText(cameraFeed, "What colour is the object?", Point(15, 30), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Press 'b' for blue", Point(15, 60), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(0, 0, 255));
            putText(cameraFeed, "Press 'g' for green", Point(15, 90), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(0, 255, 0));
            putText(cameraFeed, "Press 'r' for red", Point(15, 120), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 0, 0));
            putText(cameraFeed, "Press 'y' for yellow", Point(15, 150), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 0));
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            
            switch(ch){
                    
                case 98:
                    stage = 10;
                    setBlueValues(H_MIN_1, H_MAX_1, H_MIN_2, H_MAX_2, S_MIN, S_MAX, V_MIN, V_MAX);
                    break;      // If 'b' is pressed move to stage 10 and preset HSV parameters for blue object
                    
                case 103:
                    stage = 10;
                    setGreenValues(H_MIN_1, H_MAX_1, H_MIN_2, H_MAX_2, S_MIN, S_MAX, V_MIN, V_MAX);
                    break;      // If 'g' is pressed move to stage 11 and preset HSV parameters for green object
                    
                case 114:
                    stage = 10;
                    setRedValues(H_MIN_1, H_MAX_1, H_MIN_2, H_MAX_2, S_MIN, S_MAX, V_MIN, V_MAX);
                    break;      // If 'r' is pressed move to stage 12 and preset HSV parameters for red object
                    
                case 121:
                    stage = 10;
                    setYellowValues(H_MIN_1, H_MAX_1, H_MIN_2, H_MAX_2, S_MIN, S_MAX, V_MIN, V_MAX);
                    break;      // If 'y' is pressed move to stage 13 and preset HSV parameters for yellow object
                    
                case 113:
                    stage = 0;
                    break;      // If 'q' is pressed, return to main menu
            }
        }
        
        if(stage == 10){      // If object is blue/green/yellow
            
            textObjectWidth(cameraFeed, KNOWN_WIDTH);
            textObjectHeight(cameraFeed, KNOWN_HEIGHT);
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "S: Save", Point(260, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
            
            if((H_MIN_2 != 0) || (H_MAX_2 != 255)){
                
                createTrackbarsRed(H_MIN_1, H_MAX_1, H_MIN_2, H_MAX_2, S_MIN, S_MAX, V_MIN, V_MAX, upperLimit);
                moveWindow("Trackbars", 360, 525);
                createTrackbarsObject(KNOWN_WIDTH, KNOWN_HEIGHT, upperLimit);
                moveWindow("Dimensions", 360, 785);
                inRange(HSV,Scalar(H_MIN_1,S_MIN,V_MIN),Scalar(H_MAX_1,S_MAX,V_MAX),threshold1);
                inRange(HSV,Scalar(H_MIN_2,S_MIN,V_MIN),Scalar(H_MAX_2,S_MAX,V_MAX),threshold2);
                add(threshold1, threshold2, thresholdComb);
                
                
            }   // If the Hue is between a particular range, e.g 0-20 & 50-150 this case will be called
            
            else{
                
                createTrackbars(H_MIN_1, H_MAX_1, S_MIN, S_MAX, V_MIN, V_MAX, upperLimit);
                moveWindow("Trackbars", 360, 525);
                createTrackbarsObject(KNOWN_WIDTH, KNOWN_HEIGHT, upperLimit);
                moveWindow("Dimensions", 360, 735);
                inRange(HSV,Scalar(H_MIN_1,S_MIN,V_MIN),Scalar(H_MAX_1,S_MAX,V_MAX),thresholdComb);
                
            }   // Else if the Hue is only in one range
            
            morphOps(erodeElement, dilateElement, thresholdComb);
            objectParameters(cameraFeed, thresholdComb, U, V, width, height, KNOWN_WIDTH, KNOWN_HEIGHT, focal);
            textHue(thresholdComb, H_MIN_1, H_MAX_1, H_MIN_2, H_MAX_2);
            textSaturation(thresholdComb, S_MIN, S_MAX);
            textValue(thresholdComb, V_MIN, V_MAX);
            imshow("Colour Threshold", thresholdComb);
            moveWindow("Colour Threshold", 680, 0);
            
            switch(ch){
                    
                case 113:
                    stage = 1;
                    destroyWindow("Colour Threshold");
                    destroyWindow("Trackbars");             // If 'q' is pressed, destory windows and return to previous screen
                    destroyWindow("Dimensions");
                    break;
                    
                case 115:
                    saveObject(H_MIN_1, H_MAX_1, H_MIN_2, H_MAX_2, S_MIN, S_MAX, V_MIN, V_MAX, KNOWN_WIDTH, KNOWN_HEIGHT, iter, found);
                    break;      // If 's' is pressed, save object parameters
            }
        }
        
        if(stage == 2){
            
            putText(cameraFeed, "Press the Number of Object to Load", Point(15, 30), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            checkNumberOfObjects(found);
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            
            switch(ch){
                    
                case 49:
                    
                    found = doesFileExist("Object1.txt");
                    
                    if(found){
                        stage = 21;         // If object 1 exists, open and move to stage 21
                    }
                    else{
                        stage = 2;          // If not then do nothing
                        found = false;
                    }
                    break;
                    
                case 50:
                    
                    found = doesFileExist("Object2.txt");
                    
                    if(found){
                        stage = 22;         // If object 2 exists, open and move to stage 22
                    }
                    else{
                        stage = 2;          // If not then do nothing
                        found = false;
                    }
                    break;
                    
                case 51:
                    
                    found = doesFileExist("Object3.txt");
                    
                    if(found){
                        stage = 23;         // If object 3 exists, open and move to stage 23
                    }
                    else{
                        stage = 2;          // If not then do nothing
                        found = false;
                    }
                    break;
                    
                case 52:
                    
                    found = doesFileExist("Object4.txt");
                    
                    if(found){
                        stage = 24;         // If object 4 exists, open and move to stage 24
                    }
                    else{
                        stage = 2;          // If not then do nothing
                        found = false;
                    }
                    break;
                    
                case 53:
                    
                    found = doesFileExist("Object5.txt");
                    
                    if(found){
                        stage = 25;         // If object 5 exists, open and move to stage 25
                    }
                    else{
                        stage = 2;          // If not then do nothing
                        found = false;
                    }
                    break;
                    
                case 54:
                    
                    found = doesFileExist("Object6.txt");
                    
                    if(found){
                        stage = 26;         // If object 6 exists, open and move to stage 26
                    }
                    else{
                        stage = 2;          // If not then do nothing
                        found = false;
                    }
                    break;
                    
                case 55:
                    
                    found = doesFileExist("Object7.txt");
                    
                    if(found){
                        stage = 27;         // If object 7 exists, open and move to stage 27
                    }
                    else{
                        stage = 2;          // If not then do nothing
                        found = false;
                    }
                    break;
                    
                case 56:
                    
                    found = doesFileExist("Object8.txt");
                    
                    if(found){
                        stage = 28;         // If object 8 exists, open and move to stage 28
                    }
                    else{
                        stage = 2;          // If not then do nothing
                        found = false;
                    }
                    break;
                    
                case 57:
                    
                    found = doesFileExist("Object9.txt");
                    
                    if(found){
                        stage = 29;         // If object 9 exists, open and move to stage 29
                    }
                    else{
                        stage = 2;          // If not then do nothing
                        found = false;
                    }
                    break;
                    
                case 113:
                    stage = 0;              // If 'q' is pressed return to main menu
                    break;
            }
        }
        
        if(stage == 21 && doesFileExist("Object1.txt")){
            
            putText(cameraFeed, "1: Object 1", Point(15, 60), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "L: Load", Point(270, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            
            Mat object1 = imread("Object1_Picture.JPEG");
            imshow("Object 1", object1);
            moveWindow("Object 1", 680, 0);
            
            switch(ch){
                    
                case 108:
                    
                    stage = 211;
                    loadObject("Object1.txt");      // If 'l' is pressed, object 1 parameters will be loaded and and destroy windows
                    destroyWindow("Object 1");
                    break;
                    
                case 113:
                    stage = 2;
                    destroyWindow("Object 1");      // If 'q' is pressed it will return to prior screen and destroy windows
                    break;
            }
        }
        
        if(stage == 22 && doesFileExist("Object2.txt")){
            
            putText(cameraFeed, "2: Object 2", Point(15, 90), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "L: Load", Point(270, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            
            Mat object2 = imread("Object2_Picture.JPEG");
            imshow("Object 2", object2);
            moveWindow("Object 2", 680, 0);
            
            switch(ch){
                    
                case 108:
                    stage = 211;
                    loadObject("Object2.txt");      // If 'l' is pressed, object 2 parameters will be loaded and and destroy windows
                    destroyWindow("Object 2");
                    break;
                    
                case 113:
                    stage = 2;
                    destroyWindow("Object 2");      // If 'q' is pressed it will return to prior screen and destroy windows
                    break;
            }
        }
        
        if(stage == 23 && doesFileExist("Object3.txt")){
            
            putText(cameraFeed, "3: Object 3", Point(15, 120), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "L: Load", Point(270, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            
            Mat object3 = imread("Object3_Picture.JPEG");
            imshow("Object 3", object3);
            moveWindow("Object 3", 680, 0);
            
            switch(ch){
                    
                case 108:
                    stage = 211;
                    loadObject("Object3.txt");      // If 'l' is pressed, object 3 parameters will be loaded and destroy windows
                    destroyWindow("Object 3");
                    break;
                    
                case 113:
                    stage = 2;
                    destroyWindow("Object 3");      // If 'q' is pressed it will return to prior screen and destroy windows
                    break;
            }
        }
        
        if(stage == 24 && doesFileExist("Object4.txt")){
            
            putText(cameraFeed, "4: Object 4", Point(15, 120), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "L: Load", Point(270, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            
            Mat object4 = imread("Object4_Picture.JPEG");
            imshow("Object 4", object4);
            moveWindow("Object 4", 680, 0);
            
            switch(ch){
                    
                case 108:
                    stage = 211;
                    loadObject("Object4.txt");      // If 'l' is pressed, object 4 parameters will be loaded and destroy windows
                    destroyWindow("Object 4");
                    break;
                    
                case 113:
                    stage = 2;
                    destroyWindow("Object 4");      // If 'q' is pressed it will return to prior screen and destroy windows
                    break;
            }
        }
        
        if(stage == 25 && doesFileExist("Object5.txt")){
            
            putText(cameraFeed, "5: Object 5", Point(15, 120), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "L: Load", Point(270, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            
            Mat object5 = imread("Object5_Picture.JPEG");
            imshow("Object 5", object5);
            moveWindow("Object 5", 680, 0);
            
            switch(ch){
                    
                case 108:
                    stage = 211;
                    loadObject("Object5.txt");      // If 'l' is pressed, object 5 parameters will be loaded and destroy windows
                    destroyWindow("Object 5");
                    break;
                    
                case 113:
                    stage = 2;
                    destroyWindow("Object 5");      // If 'q' is pressed it will return to prior screen and destroy windows
                    break;
            }
        }
        
        if(stage == 26 && doesFileExist("Object6.txt")){
            
            putText(cameraFeed, "6: Object 6", Point(15, 120), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "L: Load", Point(270, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            
            Mat object6 = imread("Object6_Picture.JPEG");
            imshow("Object 6", object6);
            moveWindow("Object 6", 680, 0);
            
            switch(ch){
                    
                case 108:
                    stage = 211;
                    loadObject("Object6.txt");      // If 'l' is pressed, object 6 parameters will be loaded and destroy windows
                    destroyWindow("Object 6");
                    break;
                    
                case 113:
                    stage = 2;
                    destroyWindow("Object 6");      // If 'q' is pressed it will return to prior screen and destroy windows
                    break;
            }
        }
        
        if(stage == 27 && doesFileExist("Object7.txt")){
            
            putText(cameraFeed, "7: Object 7", Point(15, 120), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "L: Load", Point(270, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            
            Mat object7 = imread("Object7_Picture.JPEG");
            imshow("Object 7", object7);
            moveWindow("Object 7", 680, 0);
            
            switch(ch){
                    
                case 108:
                    stage = 211;
                    loadObject("Object7.txt");      // If 'l' is pressed, object 7 parameters will be loaded and destroy windows
                    destroyWindow("Object 7");
                    break;
                    
                case 113:
                    stage = 2;
                    destroyWindow("Object 7");      // If 'q' is pressed it will return to prior screen and destroy windows
                    break;
            }
        }
        
        if(stage == 28 && doesFileExist("Object8.txt")){
            
            putText(cameraFeed, "8: Object 8", Point(15, 120), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "L: Load", Point(270, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            
            Mat object8 = imread("Object8_Picture.JPEG");
            imshow("Object 8", object8);
            moveWindow("Object 8", 680, 0);
            
            switch(ch){
                    
                case 108:
                    stage = 211;
                    loadObject("Object8.txt");      // If 'l' is pressed, object 8 parameters will be loaded and destroy windows
                    destroyWindow("Object 8");
                    break;
                    
                case 113:
                    stage = 2;
                    destroyWindow("Object 8");      // If 'q' is pressed it will return to prior screen and destroy windows
                    break;
            }
        }
        
        if(stage == 29 && doesFileExist("Object9.txt")){
            
            putText(cameraFeed, "9: Object 9", Point(15, 120), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "L: Load", Point(270, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            
            Mat object9 = imread("Object9_Picture.JPEG");
            imshow("Object 9", object9);
            moveWindow("Object 9", 680, 0);
            
            switch(ch){
                    
                case 108:
                    stage = 211;
                    loadObject("Object9.txt");      // If 'l' is pressed, object 9 parameters will be loaded and destroy windows
                    destroyWindow("Object 9");
                    break;
                    
                case 113:
                    stage = 2;
                    destroyWindow("Object 9");      // If 'q' is pressed it will return to prior screen and destroy windows
                    break;
            }
        }
        
        if(stage == 211){
            
            putText(cameraFeed, "Edit the object paremeters?", Point(15, 30), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Y: Yes", Point(15, 60), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "N: No", Point(15, 90), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            
            switch(ch){
                    
                case 121:
                    stage = 221;            // If 'y' is pressed, object parameters can be edited in stage 221
                    break;
                    
                case 110:
                    stage = 222;            // If 'n' is pressed, object parameters will be passed as they are to stage 222
                    break;
                    
                case 113:
                    stage = 2;              // If 'q' is pressed, return to main 'load' screen
                    break;
            }
        }
        
        if(stage == 221){
            
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "S: Store", Point(270, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            
            cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);   // Convert image from BGR to HSV colorspace
            if((H_MIN_2 != 0) || (H_MAX_2 != 255)){
                
                createTrackbarsRed(H_MIN_1, H_MAX_1, H_MIN_2, H_MAX_2, S_MIN, S_MAX, V_MIN, V_MAX, upperLimit);
                moveWindow("Trackbars", 360, 525);
                createTrackbarsObject(KNOWN_WIDTH, KNOWN_HEIGHT, upperLimit);
                moveWindow("Dimensions", 360, 785);
                inRange(HSV,Scalar(H_MIN_1,S_MIN,V_MIN),Scalar(H_MAX_1,S_MAX,V_MAX),threshold1);
                inRange(HSV,Scalar(H_MIN_2,S_MIN,V_MIN),Scalar(H_MAX_2,S_MAX,V_MAX),threshold2);
                add(threshold1, threshold2, thresholdComb);
                
            }   // If the Hue is between a particular range, e.g 0-20 & 50-150 this case will be called
            
            else{
                
                createTrackbars(H_MIN_1, H_MAX_1, S_MIN, S_MAX, V_MIN, V_MAX, upperLimit);
                moveWindow("Trackbars", 360, 525);
                createTrackbarsObject(KNOWN_WIDTH, KNOWN_HEIGHT, upperLimit);
                moveWindow("Dimensions", 360, 735);
                inRange(HSV,Scalar(H_MIN_1,S_MIN,V_MIN),Scalar(H_MAX_1,S_MAX,V_MAX),thresholdComb);
                
            }   // Else if the Hue is only in one range
            
            morphOps(erodeElement, dilateElement, thresholdComb);
            objectParameters(cameraFeed, thresholdComb, U, V, width, height, KNOWN_WIDTH, KNOWN_HEIGHT, focal);
            imshow("Colour Threshold", thresholdComb);
            moveWindow("Colour Threshold", 680, 0);
            textObjectWidth(cameraFeed, KNOWN_WIDTH);
            textObjectHeight(cameraFeed, KNOWN_HEIGHT);
            
            switch(ch){
                    
                case 113:
                    stage = 2;
                    destroyWindow("Trackbars");
                    destroyWindow("Colour Threshold");
                    destroyWindow("Dimensions");            // If 'q' is pressed, return to main 'load' screen and destroy windows
                    break;
                    
                case 115:
                    stage = 222;
                    destroyWindow("Trackbars");
                    destroyWindow("Colour Threshold");
                    destroyWindow("Dimensions");            // If 's' is pressed, object parameters will be passed onto stage 222
                    ch = 0;
                    break;
            }
        }
        
        if(stage == 222){
            
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "S: Send", Point(270, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            
            cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);   // Convert image from BGR to HSV colorspace
            if((H_MIN_2 != 0) || (H_MAX_2 != 255)){
                
                inRange(HSV,Scalar(H_MIN_1,S_MIN,V_MIN),Scalar(H_MAX_1,S_MAX,V_MAX),threshold1);
                inRange(HSV,Scalar(H_MIN_2,S_MIN,V_MIN),Scalar(H_MAX_2,S_MAX,V_MAX),threshold2);
                add(threshold1, threshold2, thresholdComb);
                
            }   // If the Hue is between a particular range, e.g 0-20 & 50-150 this case will be called
            
            else{
                
                inRange(HSV,Scalar(H_MIN_1,S_MIN,V_MIN),Scalar(H_MAX_1,S_MAX,V_MAX),thresholdComb);
                
            }   // Else if the Hue is only in one range
            
            morphOps(erodeElement, dilateElement, thresholdComb);
            objectParameters(cameraFeed, thresholdComb, U, V, width, height, KNOWN_WIDTH, KNOWN_HEIGHT, focal);
            X = getXCoordinate(focal, KNOWN_HEIGHT, KNOWN_WIDTH, height, width);
            Y = getYCoordinate(X, focal, U);
            Z = getZCoordinate(X, focal, V);
            rotateCoordinates(X, Y, Z);
            translateCoordinates(X, Y, Z, U);
            textXCoordinate(cameraFeed, X);
            textYCoordinate(cameraFeed, Y);
            textZCoordinate(cameraFeed, Z);
            focal = updateFocal(X);
            
            switch(ch){
                    
                case 113:
                    stage = 2;          // If 'q' is pressed, return to main 'load' screen
                    break;
                    
                case 115:
                    controlAlgorithm(X, Y, Z, X1, Y1, link1, link2, link3, link4, xlink5, ylink5, link6, objectVector, link23Min, link23Max, iter, stage);
                    break;
            }
        }
        
        if(stage == 2221){
            
            destroyWindow("Colour Threshold");
            destroyWindow("Trackbars");
            destroyWindow("Dimensions");
            putText(cameraFeed, "Follow Instructions in New Window", Point(15, 30), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            
            switch(ch){
                    
                case 113:
                    stage = 222;
                    system("Killall CoolTerm");     // If 'q' is pressed, return to previous screen
                    break;
            }
        }
        
        if(stage == 3){
            
            putText(cameraFeed, "Press the Number to Delete Object", Point(15, 30), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            checkNumberOfObjects(found);
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            
            switch(ch){
                    
                case 49:
                    
                    found = doesFileExist("Object1.txt");
                    
                    if(found){
                        stage = 31;             // If object number is pressed and it exists move to stage 31
                    }
                    else{
                        stage = 3;              // If not then do nothing
                        found = false;
                    }
                    break;
                    
                case 50:
                    
                    found = doesFileExist("Object2.txt");
                    
                    if(found){
                        stage = 32;             // If object number is pressed and it exists move to stage 31
                    }
                    else{
                        stage = 3;              // If not then do nothing
                        found = false;
                    }
                    break;
                    
                case 51:
                    
                    found = doesFileExist("Object3.txt");
                    
                    if(found){
                        stage = 33;             // If object number is pressed and it exists move to stage 31
                    }
                    else{
                        stage = 3;              // If not then do nothing
                        found = false;
                    }
                    break;
                    
                case 52:
                    
                    found = doesFileExist("Object4.txt");
                    
                    if(found){
                        stage = 34;             // If object number is pressed and it exists move to stage 31
                    }
                    else{
                        stage = 3;              // If not then do nothing
                        found = false;
                    }
                    break;
                    
                case 53:
                    
                    found = doesFileExist("Object5.txt");
                    
                    if(found){
                        stage = 35;             // If object number is pressed and it exists move to stage 31
                    }
                    else{
                        stage = 3;              // If not then do nothing
                        found = false;
                    }
                    break;
                    
                case 54:
                    
                    found = doesFileExist("Object6.txt");
                    
                    if(found){
                        stage = 36;             // If object number is pressed and it exists move to stage 31
                    }
                    else{
                        stage = 3;              // If not then do nothing
                        found = false;
                    }
                    break;
                    
                case 55:
                    
                    found = doesFileExist("Object7.txt");
                    
                    if(found){
                        stage = 37;             // If object number is pressed and it exists move to stage 31
                    }
                    else{
                        stage = 3;              // If not then do nothing
                        found = false;
                    }
                    break;
                    
                case 56:
                    
                    found = doesFileExist("Object8.txt");
                    
                    if(found){
                        stage = 38;             // If object number is pressed and it exists move to stage 31
                    }
                    else{
                        stage = 3;              // If not then do nothing
                        found = false;
                    }
                    break;
                    
                case 57:
                    
                    found = doesFileExist("Object9.txt");
                    
                    if(found){
                        stage = 39;             // If object number is pressed and it exists move to stage 31
                    }
                    else{
                        stage = 3;              // If not then do nothing
                        found = false;
                    }
                    break;
                    
                case 113:
                    stage = 0;                  // If 'q' is pressed, return to main menu
                    break;
            }
        }
        
        if(stage == 31 && doesFileExist("Object1.txt")){
            
            putText(cameraFeed, "1: Object 1", Point(15, 60), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "D: Delete", Point(270, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            
            Mat object1 = imread("Object1_Picture.JPEG");
            imshow("Object 1", object1);
            moveWindow("Object 1", 680, 0);
            
            switch(ch){
                    
                case 100:
                    deleteObject("Object1.txt");
                    deleteObject("Object1_Picture.JPEG");       // If 'd' is pressed, object data will be deleted
                    stage = 3;
                    destroyWindow("Object 1");
                    break;
                    
                case 113:
                    stage = 3;
                    destroyWindow("Object 1");                  // If 'q' is pressed, return to prior screen
                    break;
            }
        }
        
        if(stage == 32 && doesFileExist("Object2.txt")){
            
            putText(cameraFeed, "2: Object 2", Point(15, 90), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "D: Delete", Point(270, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            
            Mat object2 = imread("Object2_Picture.JPEG");
            imshow("Object 2", object2);
            moveWindow("Object 2", 680, 0);
            
            switch(ch){
                    
                case 100:
                    deleteObject("Object2.txt");
                    deleteObject("Object2_Picture.JPEG");       // If 'd' is pressed, object data will be deleted
                    stage = 3;
                    destroyWindow("Object 2");
                    break;
                    
                case 113:
                    stage = 3;
                    destroyWindow("Object 2");                  // If 'q' is pressed, return to prior screen
                    break;
            }
        }
        
        if(stage == 33 && doesFileExist("Object3.txt")){
            
            putText(cameraFeed, "3: Object 3", Point(15, 120), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "D: Delete", Point(270, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            
            Mat object3 = imread("Object3_Picture.JPEG");
            imshow("Object 3", object3);
            moveWindow("Object 3", 680, 0);
            
            switch(ch){
                    
                case 100:
                    deleteObject("Object3.txt");
                    deleteObject("Object3_Picture.JPEG");       // If 'd' is pressed, object data will be deleted
                    stage = 3;
                    destroyWindow("Object 3");
                    break;
                    
                case 113:
                    stage = 3;
                    destroyWindow("Object 3");                  // If 'q' is pressed, return to prior screen
                    break;
            }
        }
        
        if(stage == 34 && doesFileExist("Object4.txt")){
            
            putText(cameraFeed, "4: Object 4", Point(15, 120), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "D: Delete", Point(270, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            
            Mat object4 = imread("Object4_Picture.JPEG");
            imshow("Object 4", object4);
            moveWindow("Object 4", 680, 0);
            
            switch(ch){
                    
                case 100:
                    deleteObject("Object4.txt");
                    deleteObject("Object4_Picture.JPEG");       // If 'd' is pressed, object data will be deleted
                    stage = 3;
                    destroyWindow("Object 4");
                    break;
                    
                case 113:
                    stage = 3;
                    destroyWindow("Object 4");                  // If 'q' is pressed, return to prior screen
                    break;
            }
        }
        
        if(stage == 35 && doesFileExist("Object5.txt")){
            
            putText(cameraFeed, "5: Object 5", Point(15, 120), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "D: Delete", Point(270, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            
            Mat object5 = imread("Object5_Picture.JPEG");
            imshow("Object 5", object5);
            moveWindow("Object 5", 680, 0);
            
            switch(ch){
                    
                case 100:
                    deleteObject("Object5.txt");
                    deleteObject("Object5_Picture.JPEG");       // If 'd' is pressed, object data will be deleted
                    stage = 3;
                    destroyWindow("Object 5");
                    break;
                    
                case 113:
                    stage = 3;
                    destroyWindow("Object 5");                  // If 'q' is pressed, return to prior screen
                    break;
            }
        }
        
        if(stage == 36 && doesFileExist("Object6.txt")){
            
            putText(cameraFeed, "6: Object 6", Point(15, 120), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "D: Delete", Point(270, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            
            Mat object6 = imread("Object6_Picture.JPEG");
            imshow("Object 6", object6);
            moveWindow("Object 6", 680, 0);
            
            switch(ch){
                    
                case 100:
                    deleteObject("Object6.txt");
                    deleteObject("Object6_Picture.JPEG");       // If 'd' is pressed, object data will be deleted
                    stage = 3;
                    destroyWindow("Object 6");
                    break;
                    
                case 113:
                    stage = 3;
                    destroyWindow("Object 6");                  // If 'q' is pressed, return to prior screen
                    break;
            }
        }
        
        if(stage == 37 && doesFileExist("Object7.txt")){
            
            putText(cameraFeed, "7: Object 7", Point(15, 120), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "D: Delete", Point(270, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            
            Mat object7 = imread("Object7_Picture.JPEG");
            imshow("Object 7", object7);
            moveWindow("Object 7", 680, 0);
            
            switch(ch){
                    
                case 100:
                    deleteObject("Object7.txt");
                    deleteObject("Object7_Picture.JPEG");       // If 'd' is pressed, object data will be deleted
                    stage = 3;
                    destroyWindow("Object 7");
                    break;
                    
                case 113:
                    stage = 3;
                    destroyWindow("Object 7");                  // If 'q' is pressed, return to prior screen
                    break;
            }
        }
        
        if(stage == 38 && doesFileExist("Object8.txt")){
            
            putText(cameraFeed, "8: Object 8", Point(15, 120), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "D: Delete", Point(270, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            
            Mat object8 = imread("Object8_Picture.JPEG");
            imshow("Object 8", object8);
            moveWindow("Object 8", 680, 0);
            
            switch(ch){
                    
                case 100:
                    deleteObject("Object8.txt");
                    deleteObject("Object8_Picture.JPEG");       // If 'd' is pressed, object data will be deleted
                    stage = 3;
                    destroyWindow("Object 8");
                    break;
                    
                case 113:
                    stage = 3;
                    destroyWindow("Object 8");                  // If 'q' is pressed, return to prior screen
                    break;
            }
        }
        
        if(stage == 39 && doesFileExist("Object9.txt")){
            
            putText(cameraFeed, "9: Object 9", Point(15, 120), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "D: Delete", Point(270, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            
            Mat object9 = imread("Object9_Picture.JPEG");
            imshow("Object 9", object9);
            moveWindow("Object 9", 680, 0);
            
            switch(ch){
                    
                case 100:
                    deleteObject("Object9.txt");
                    deleteObject("Object9_Picture.JPEG");       // If 'd' is pressed, object data will be deleted
                    stage = 3;
                    destroyWindow("Object 9");
                    break;
                    
                case 113:
                    stage = 3;
                    destroyWindow("Object 9");                  // If 'q' is pressed, return to prior screen
                    break;
            }
        }
        
        if(stage == 4){
            
            putText(cameraFeed, "What colour is the object?", Point(15, 30), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Press 'b' for blue", Point(15, 60), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(0, 0, 255));
            putText(cameraFeed, "Press 'g' for green", Point(15, 90), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(0, 255, 0));
            putText(cameraFeed, "Press 'r' for red", Point(15, 120), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 0, 0));
            putText(cameraFeed, "Press 'y' for yellow", Point(15, 150), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 0));
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            
            switch(ch){
                    
                case 98:
                    stage = 40;
                    setBlueValues(H_MIN_1, H_MAX_1, H_MIN_2, H_MAX_2, S_MIN, S_MAX, V_MIN, V_MAX);
                    break;      // If 'b' is pressed move to stage 40 and preset HSV parameters for blue object
                    
                case 103:
                    stage = 40;
                    setGreenValues(H_MIN_1, H_MAX_1, H_MIN_2, H_MAX_2, S_MIN, S_MAX, V_MIN, V_MAX);
                    break;      // If 'g' is pressed move to stage 40 and preset HSV parameters for green object
                    
                case 114:
                    stage = 40;
                    setRedValues(H_MIN_1, H_MAX_1, H_MIN_2, H_MAX_2, S_MIN, S_MAX, V_MIN, V_MAX);
                    break;      // If 'r' is pressed move to stage 41 and preset HSV parameters for red object
                    
                case 121:
                    stage = 40;
                    setYellowValues(H_MIN_1, H_MAX_1, H_MIN_2, H_MAX_2, S_MIN, S_MAX, V_MIN, V_MAX);
                    break;      // If 'y' is pressed move to stage 40 and preset HSV parameters for yellow object
                    
                case 113:
                    stage = 0;
                    break;      // If 'q' is pressed, return to main menu
            }
        }
        
        if(stage == 40){
            
            cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
            
            if((H_MIN_2 != 0) || (H_MAX_2 != 255)){
                
                createTrackbarsRed(H_MIN_1, H_MAX_1, H_MIN_2, H_MAX_2, S_MIN, S_MAX, V_MIN, V_MAX, upperLimit);
                moveWindow("Trackbars", 360, 525);
                createTrackbarsObject(KNOWN_WIDTH, KNOWN_HEIGHT, upperLimit);
                moveWindow("Dimensions", 360, 785);
                inRange(HSV,Scalar(H_MIN_1,S_MIN,V_MIN),Scalar(H_MAX_1,S_MAX,V_MAX),threshold1);
                inRange(HSV,Scalar(H_MIN_2,S_MIN,V_MIN),Scalar(H_MAX_2,S_MAX,V_MAX),threshold2);
                add(threshold1, threshold2, thresholdComb);
                
                
            }   // If the Hue is between a particular range, e.g 0-20 & 50-150 this case will be called
            
            else{
                
                createTrackbars(H_MIN_1, H_MAX_1, S_MIN, S_MAX, V_MIN, V_MAX, upperLimit);
                moveWindow("Trackbars", 360, 525);
                createTrackbarsObject(KNOWN_WIDTH, KNOWN_HEIGHT, upperLimit);
                moveWindow("Dimensions", 360, 735);
                inRange(HSV,Scalar(H_MIN_1,S_MIN,V_MIN),Scalar(H_MAX_1,S_MAX,V_MAX),thresholdComb);
                
            }   // Else if the Hue is only in one range
            
            morphOps(erodeElement, dilateElement, thresholdComb);
            objectParameters(cameraFeed, thresholdComb, U, V, width, height, KNOWN_WIDTH, KNOWN_HEIGHT, focal);
            textObjectWidth(cameraFeed, KNOWN_WIDTH);
            textObjectHeight(cameraFeed, KNOWN_HEIGHT);
            X = getXCoordinate(focal, KNOWN_HEIGHT, KNOWN_WIDTH, height, width);
            Y = getYCoordinate(X, focal, U);
            Z = getZCoordinate(X, focal, V);
            rotateCoordinates(X, Y, Z);
            translateCoordinates(X, Y, Z, U);
            textXCoordinate2(cameraFeed, X);
            textYCoordinate2(cameraFeed, Y);
            textZCoordinate2(cameraFeed, Z);
            focal = updateFocal(X);
            textHue(thresholdComb, H_MIN_1, H_MAX_1, H_MIN_2, H_MAX_2);
            textSaturation(thresholdComb, S_MIN, S_MAX);
            textValue(thresholdComb, V_MIN, V_MAX);
            imshow("Colour Threshold", thresholdComb);
            moveWindow("Colour Threshold", 680, 0);
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "S: Send", Point(260, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            
            switch(ch){
                    
                case 113:
                    stage = 4;
                    destroyWindow("Colour Threshold");
                    destroyWindow("Dimensions");
                    destroyWindow("Trackbars");             // If 'q' is pressed retrun to main 'play' menu
                    break;
                    
                case 115:
                    controlAlgorithm(X, Y, Z, X1, Y1, link1, link2, link3, link4, xlink5, ylink5, link6, objectVector, link23Min, link23Max, iter, stage);
                    break;
            }
        }
        
        if(stage == 400){
            
            destroyWindow("Colour Threshold");
            destroyWindow("Trackbars");
            destroyWindow("Dimensions");
            putText(cameraFeed, "Follow Instructions in New Window", Point(15, 30), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "E: Exit", Point(15, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            putText(cameraFeed, "Q: Return", Point(120, 465), FONT_HERSHEY_DUPLEX, 0.75, CV_RGB(255, 255, 255));
            
            switch(ch){
                    
                case 113:
                    stage = 40;
                    system("Killall CoolTerm");     // If 'q' is pressed, return to previous screen
                    break;
            }
        }
        
        storedObjectStatus(iter);                   // Check object status and alert user on screen
        
        imshow("Camera Feed", cameraFeed);          // Display camera feed
        moveWindow("Camera Feed", 40, 0);
        
        ch = waitKey(20);                           // Check user input
        CH = (int)ch;                               // Convert to integer vaue
    }
    
    system("Killall CoolTerm");                     // Confirm 'CoolTerm' is closed when exiting program
    
    return 0;
    
}
