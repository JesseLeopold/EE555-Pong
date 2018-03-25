/*=============================================================================
| Assignment: Final - Pong Automation Program
|
| Author: Jesse Leopold
| Language: C++
|
| Class: EE-555 Embedded Systems II: Embedded Software
| Instructor: Allan Douglas
|
+-----------------------------------------------------------------------------
|
| Description: 
| The pong automation program is designed to interface with a video stream 
| of a pong game in progress. Assessment is to be made of the state of the 
| game and the player's paddle is to be moved in order to play pong.
*===========================================================================*/

// GENERAL GLOBAL INCLUDES
#include <iostream>                     // IO Stream Library
#include <string>                       // General C++ String Library
#include <sstream>                      // Standard Serial Stream Library
#include <stdio.h>                      // Standard IO Library
#include <math.h>                       // C++ Math Library
#include <opencv2/cudacodec.hpp>        // OpenCV CUDA Library
#include <opencv2/opencv.hpp>           // OpenCV General Library
#include <opencv2/highgui/highgui.hpp>  // OpenCV GUI Library
#include <opencv2/imgproc/imgproc.hpp>  // OpenCV Image Processing Library

// Select Video Source -  The MP4 demo uses a ROI for better tracking of the moving object
#define TEST_LIVE_VIDEO

// VERBOSE DEBUG PRINT SWITCHES
bool GUI_DEBUG_PRINT  = true;

// NAMESPACE REFERENCES
using namespace cv;
using namespace std;

// GENERAL USE CONFIGURATION VARIABLES
int theObject[2] = {0,0};
cv::Rect objectBoundingRectangle = cv::Rect(0,0,0,0);   // Bounding Rectangle of Pong Ball 

// Region of Interest Point Specifications
int roiXMin = 520;          // MIN POSSIBLE = 0
int roiXMax = 825;          // MAX POSSIBLE = 1279
int roiYMin =  90;          // MIN POSSIBLE = 0
int roiYMax = 623;          // MAX POSSIBLE = 719
int roiYMinPadTop =  65;    // MIN POSSIBLE = 0
int roiYMaxPadTop = 100;    // MAX POSSIBLE = 719
int roiYMinPadBot = 618;    // MIN POSSIBLE = 0
int roiYMaxPadBot = 653;    // MAX POSSIBLE = 719

// Position and Velocity Tracking Variables
bool prevObjectDetected = false;
int prevXPos, prevYPos = 0;
float xVelocity, yVelocity, velocity = 0;

//Board Corner Tracking Variables
Point2i boardCorner[4];

// PROGRAM CONSTANTS - TRANSFORM
bool TRANSFORM_DYNAMIC = false;

// PROGRAM CONSTANTS - COLOR FILTER
bool TRACK_OBJECTS = false;         // General Program Switch - Track Objects in Video Feed 
bool USE_MORPH_OPS = true;          // General Program Switch - Use Video Feed Morphing Manipulations
int H_MIN = 0;                      // Hue Minimum Filter Default
int H_MAX = 256;                    // Hue Maximum Filter Default
int S_MIN = 0;                      // Saturation Minimum Filter Default
int S_MAX = 256;                    // Saturation Maximum Filter Default
int V_MIN = 0;                      // Value Minimum Filter Default
int V_MAX = 256;                    // Value Maximum Filter Default
const int FRAME_WIDTH = 640;        // Default Input Stream Pixel Width
const int FRAME_HEIGHT = 480;       // Default Input Stream Pixel Height
const int MAX_NUM_OBJECTS=50;       // Number of Objects To Track
const int MIN_OBJECT_AREA = 20*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
const string windowName = "Original Image";                 // Output Window Title - Original Image
const string windowName1 = "HSV Image";                     // Output Window title - HSV Color Scheme Equivalent
const string windowName2 = "Thresholded Image";             // Output Window Title - Threshold Output 
const string trackbarWindowName = "HSV Threshold Controls"; // User Interface Window Title - Filter Slider Bars

// PROGRAM CONSTANTS - CONTROLLER
const char* USB_INTERFACE = "/dev/ttyUSB0";

// General Function Prototypes
string intToString(int number);
string floatToString(float number);

#include "gameBoard.cpp"
#include "paddleDriver.cpp"

// Main Program Specific Function Prototypes
void searchForMovement(cv::Mat thresholdImage, cv::Mat &cameraFeed,   gameBoard& currentGame);
void printMovementData(cv::Mat &cameraFeed, int x, int y, float xVelocity, float yVelocity, float velocity);
void createTrackbars(void);
void searchForPaddles(cv::Rect roi, cv::Mat thresholdImage, cv::Mat &cameraFeed, Point2i &paddlelocation);
void findCorners(cv::Mat src_gray, cv::Mat &cameraFeed);

//-----------------------------------------------------------------------------------------------------------------
// main
//-----------------------------------------------------------------------------------------------------------------
int main() 
{
    // LOCAL VARIABLE DECLARATION
    // ------ Data Matrices ------
    cv::Mat             frame0, frame1,                             // OpenCV Frame - Raw Input Frames
                        frame0_warped,frame1_warped,                // OpenCV Frame - Perspective Shifted Frames
                        result;                                     // OpenCV Frame - Temporary Data Transfer

    cv::cuda::GpuMat    gpu_frame0, gpu_frame1,                     // GPU Matrix Input Frames
                        gpu_frame0_warped, gpu_frame1_warped,       // GPU Matrix Perspective Warp Frames
                        gpu_grayImage0, gpu_grayImage1,             // GPU Matrix Gray Scale Frames
                        gpu_differenceImage, gpu_thresholdImage;    // GPU MAtrix Compute Result Frames

    // ------ Gameplay Tracking Objects ------
    cv::Mat frame0_warped_HSV;              // Frame Pair 0 - Warped Perspective, HSV Formatted 
    cv::Mat frame1_warped_HSV;              // Frame Pair 0 - Warped Perspective, HSV Formatted
    cv::Mat thresholdPadBot;                // Binary Threshold Filtered Image For Bot Paddle Detection
    cv::Mat thresholdPadTop;                // Binary Threshold Filtered Image For Top Paddle Detection
    cv::Mat thresholdCorners;               // Binary Threshold Filtered Image For Corner Detection
    
    // ------ Iteration Variables ------
    int toggle, frame_count;
    
    // ------ Perspective Transform Variables -------
    Point2f inputQuad[4];                   // Input Point Array
    Point2f outputQuad[4];                  // Output Point Array
    cv::Mat lambda(2,4, CV_32FC1);          // Lambda Matrix

    if(TRANSFORM_DYNAMIC)
    {
        inputQuad[0] = Point2f(boardCorner[0].x, boardCorner[0].y); 
        inputQuad[1] = Point2f(boardCorner[1].x, boardCorner[1].y); 
        inputQuad[2] = Point2f(boardCorner[2].x, boardCorner[2].y); 
        inputQuad[3] = Point2f(boardCorner[3].x, boardCorner[3].y); 
    }
    else
    {
        //-----------------------------------------------
        #ifdef TEST_LIVE_VIDEO
        inputQuad[3] = Point2f(285, 243);       // Video Feed Input Perspective Point - Top-Left 
        inputQuad[0] = Point2f(901, 217);       // Video Feed Input Perspective Point - Top-Right
        inputQuad[1] = Point2f(934, 575);       // Video Feed Input Perspective Point - Bottom-Right
        inputQuad[2] = Point2f(254, 588);       // Video Feed Input Perspective Point - Bottom-Left
        #else
        inputQuad[0] = Point2f(520, 80);        // Video Feed Input Perspective Point - Top-Left 
        inputQuad[1] = Point2f(880, 77);        // Video Feed Input Perspective Point - Top-Right
        inputQuad[2] = Point2f(923, 672);       // Video Feed Input Perspective Point - Bottom-Right
        inputQuad[3] = Point2f(472, 655);       // Video Feed Input Perspective Point - Bottom-Left
        #endif
   }

    outputQuad[0] = Point2f(437, 0);        // Video Feed Output Perspective Point - Top-Left
    outputQuad[1] = Point2f(842, 0);        // Video Feed Output Perspective Point - Top-Right
    outputQuad[2] = Point2f(842, 719);      // Video Feed Output Perspective Point - Bottom-Right
    outputQuad[3] = Point2f(437, 728);      // Video Feed Output Perspective Point - Bottom-Left

    lambda = cv::getPerspectiveTransform(inputQuad, outputQuad);  //Transform Reference for Perspective Shift

    // ------ Gameplay Tracking Objects ------
    cv::Rect roi(roiXMin, roiYMin, roiXMax-roiXMin, roiYMax-roiYMin);
    cv::Rect roiPadTop(roiXMin, roiYMinPadTop, roiXMax-roiXMin, roiYMaxPadTop-roiYMinPadTop);
    cv::Rect roiPadBot(roiXMin, roiYMinPadBot, roiXMax-roiXMin, roiYMaxPadBot-roiYMinPadBot);

    gameBoard currentGame(roi, roiXMin, roiXMax, roiYMin, roiYMax);
    paddleDriver controller;
    Point2i topPaddleLocation, botPaddleLocation;

    // SPECFIICATION OF PROGRAM'S VIDEO SOURCE
    #ifdef TEST_LIVE_VIDEO
    // Camera video pipeline
    std::string pipeline = "nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)I420, framerate=(fraction)30/1 ! nvvidconv flip-method=0 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    #else
    // MP4 file pipeline
    std::string pipeline = "filesrc location=/home/nvidia/workspace/Lab04/pong_video.mp4 ! qtdemux name=demux ! h264parse ! omxh264dec ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    #endif

    std::cout << "Using pipeline: " << pipeline << std::endl;

    // Pull a Frame from the Video Feed to Test Connection
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

    // Verify that the Input Stream is Open and Ready to Supply Frame Data
    if (!cap.isOpened()) 
    {
        std::cout << "Connection failed" << std::endl;  // Print out a Notice of Failed Input Stream
        return -1;                                      // Error out of Program on Failed Input Stream
    }
    
    // GENERATE A UI ELEMENT FOR HSV FILTRATION CONTROL
	createTrackbars();  // Create a Slider Bar For User Control of HSV Filtration

    // PULL A VIDEO FRAME, PERFORM APPROPERIATE TRANSFORMATIONS
    cap >> frame0;                                                                      // Get a new frame from file
    gpu_frame0.upload(frame0);                                                          // Upload to GPU
    cv::cuda::warpPerspective(gpu_frame0,gpu_frame0_warped,lambda,gpu_frame0.size());   // Warp Perspective
    gpu_frame0_warped.download(frame0_warped);                                          // Download Warped Frame
    cv::cuda::cvtColor(gpu_frame0_warped,gpu_grayImage0,cv::COLOR_BGR2GRAY);            // Convert to Grayscale

    // Initialize Looping Variables
    toggle = 0;
    frame_count = 0;

    #ifdef TEST_LIVE_VIDEO 
    while(1)
    #else
    while (frame_count < 1000)      // Iterate through a set number of Video Frames 
    #endif
    {

        if (toggle == 0) 
        {
            // PULL A VIDEO FRAME, PERFORM APPROPERIATE TRANSFORMATIONS
            //--------------------------------
            cap >> frame1;                                                                          // Get a new frame from file
            //--------------------------------
            inRange(frame1,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),thresholdCorners);   // Perform HSV-Based Filtration 
            //findCorners(thresholdCorners, frame1);                                                  // Find Corners
            //--------------------------------
	        gpu_frame1.upload(frame1);                                                              // Upload to GPU
            cv::cuda::warpPerspective(gpu_frame1,gpu_frame1_warped,lambda,gpu_frame1.size());       // Warp Perspective
            gpu_frame1_warped.download(frame1_warped);                                              // Download Warped Frame
            cv::cuda::cvtColor(gpu_frame1_warped,gpu_grayImage1,cv::COLOR_BGR2GRAY);                // Convert to Grayscale
            //--------------------------------
            cvtColor(frame1_warped,frame1_warped_HSV,COLOR_BGR2HSV);                                // Convert Video from RGB to HSV
            #ifdef TEST_LIVE_VIDEO
            //inRange(frame1_warped_HSV,Scalar( 60, 70,160),Scalar( 80,126,186),thresholdPadTop);     // Perform HSV-Based Filtration for Top Paddle
            //inRange(frame1_warped_HSV,Scalar( 45, 55,130),Scalar( 65,112,196),thresholdPadBot);     // Perform HSV-Based Filtration for Bot Paddle
            inRange(frame1_warped_HSV,Scalar( 40, 30,130),Scalar(116, 96,166),thresholdPadTop);     // Perform HSV-Based Filtration for Top Paddle
            inRange(frame1_warped_HSV,Scalar(  0, 10,143),Scalar( 30, 66,256),thresholdPadBot);     // Perform HSV-Based Filtration for Bot Paddle
            #else
            inRange(frame1_warped_HSV,Scalar( 50,200, 30),Scalar(100,256,256),thresholdPadTop);     // Perform HSV-Based Filtration for Top Paddle
            inRange(frame1_warped_HSV,Scalar( 30, 20,240),Scalar( 66,100,256),thresholdPadBot);     // Perform HSV-Based Filtration for Bot Paddle
            #endif
            searchForPaddles(roiPadTop, thresholdPadTop, frame1_warped_HSV, topPaddleLocation);     // Locate Top Paddle
            searchForPaddles(roiPadBot, thresholdPadBot, frame1_warped_HSV, botPaddleLocation);     // Locate Bottom Paddle
            //--------------------------------
            toggle = 1;                                                                             // Update Toggle State
        } 
        else 
        {
            // PULL A VIDEO FRAME, PERFORM APPROPERIATE TRANSFORMATIONS
            //--------------------------------
            cap >> frame0;                                                                          // Get a new frame from file
            //--------------------------------
            inRange(frame0,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),thresholdCorners);   // Perform HSV-Based Filtration 
            //findCorners(thresholdCorners, frame0);                                                  // Find Corners 
            //--------------------------------
            gpu_frame0.upload(frame0);                                                              // Upload to GPU
            cv::cuda::warpPerspective(gpu_frame0,gpu_frame0_warped,lambda,gpu_frame0.size());       // Warp Perspective
            gpu_frame0_warped.download(frame0_warped);                                              // Download Warped Frame
            cv::cuda::cvtColor(gpu_frame0_warped,gpu_grayImage0,cv::COLOR_BGR2GRAY);                // Convert to Grayscale
            //--------------------------------
            cvtColor(frame0_warped,frame0_warped_HSV,COLOR_BGR2HSV);                                // Convert Video from RGB to HSV
            #ifdef TEST_LIVE_VIDEO
            //inRange(frame0_warped_HSV,Scalar( 60, 70,160),Scalar( 80,126,186),thresholdPadTop);     // Perform HSV-Based Filtration for Top Paddle
            //inRange(frame0_warped_HSV,Scalar( 45, 55,130),Scalar( 65,112,196),thresholdPadBot);     // Perform HSV-Based Filtration for Bot Paddle
            inRange(frame0_warped_HSV,Scalar( 40, 30,130),Scalar(116, 96,166),thresholdPadTop);     // Perform HSV-Based Filtration for Top Paddle
            inRange(frame0_warped_HSV,Scalar(  0, 10,143),Scalar( 30, 66,255),thresholdPadBot);     // Perform HSV-Based Filtration for Bot Paddle
            #else
            inRange(frame0_warped_HSV,Scalar( 50,200, 30),Scalar(100,256,256),thresholdPadTop);     // Perform HSV-Based Filtration for Top Paddle
            inRange(frame0_warped_HSV,Scalar( 30, 20,240),Scalar( 66,100,256),thresholdPadBot);     // Perform HSV-Based Filtration for Bot Paddle
            #endif
            searchForPaddles(roiPadTop, thresholdPadTop, frame0_warped_HSV,topPaddleLocation);      // Locate Top Paddle
            searchForPaddles(roiPadBot, thresholdPadBot, frame0_warped_HSV,botPaddleLocation);      // Locate Bottom Paddle
            //--------------------------------
            toggle = 0;                                                                             // Update Toggle State
	    }

        // COMPARE FRAMES FOR MOVEMENT
	    cv::cuda::absdiff(gpu_grayImage0, gpu_grayImage1, gpu_differenceImage);                     // Find Movement 
        cv::cuda::threshold(gpu_differenceImage, gpu_thresholdImage, 50, 255, cv::THRESH_BINARY);   // Filter Noise
        gpu_thresholdImage.download(result);                                                        // Download Filtered Diff Frame 

        imshow("result",result);

	    // Find the location of any moving object and show the final frame
	    if (toggle == 0) 
        {
            searchForMovement(result,frame0_warped, currentGame);               // Submit Difference Frame to Search for Movement 
	        
            //imshow("Frame", frame0_warped);                                     // Show Perspective Shifted Frame

            cv::line(frame0, inputQuad[0], inputQuad[1], Scalar(0,255,0),2);    // Draw Line Around Perspective Bound - Top Left-Right
            cv::line(frame0, inputQuad[1], inputQuad[2], Scalar(0,255,0),2);    // Draw Line Around Perspective Bound - Top-Bottom Right
            cv::line(frame0, inputQuad[2], inputQuad[3], Scalar(0,255,0),2);    // Draw Line Around Perspective Bound - Bottom Right-Left
            cv::line(frame0, inputQuad[3], inputQuad[0], Scalar(0,255,0),2);    // Draw Line Around Perspective Bound - Bottom-Top Left
            
            imshow("Original", frame0);                                           // Show Original Frame Data
            //imshow("WarpedHSV", frame0_warped_HSV);                               // Show Warped HSV Frame Data
            //imshow("Paddle Top Filtered", thresholdPadTop);                       // Show Warped Filtered HSV For Top Paddle
            //imshow("Paddle Bot Filtered", thresholdPadBot);                       // Show Warped Filtered HSV For Bottom Paddle
            //imshow("Corners Filtered", thresholdCorners);                         // Show Warped Filtered HSV For Bottom Paddle
	    }

	    else 
        {
            searchForMovement(result,frame1_warped, currentGame);               // Submit Difference Frame to Search for Movement
	        
            //imshow("Frame", frame1_warped);                                     // Show Perspective Shifted Frame

            cv::line(frame1, inputQuad[0], inputQuad[1], Scalar(0,255,0),2);    // Draw Line Around Perspective Bound - Top Left-Right
            cv::line(frame1, inputQuad[1], inputQuad[2], Scalar(0,255,0),2);    // Draw Line Around Perspective Bound - Top-Bottom Right
            cv::line(frame1, inputQuad[2], inputQuad[3], Scalar(0,255,0),2);    // Draw Line Around Perspective Bound - Bottom Right-Left
            cv::line(frame1, inputQuad[3], inputQuad[0], Scalar(0,255,0),2);    // Draw Line Around Perspective Bound - Bottom-Top Left
            
            imshow("Original", frame1);                                           // Show Original Frame Data
            //imshow("WarpedHSV", frame1_warped_HSV);                               // Show Warped HSV Frame Data
            //imshow("Paddle Top Filtered", thresholdPadTop);                       // Show Warped Filtered HSV For Top Paddle
            //imshow("Paddle Bot Filtered", thresholdPadBot);                       // Show Warped Filtered HSV For Bottom Paddle 
            //imshow("Corners Filtered", thresholdCorners);                         // Show Warped Filtered HSV For Bottom Paddle
	    }

        //TODO: FIND CURRENT BALL INTERCEPT, SET PADDLE TO INTERCEPT
        // UPDATE PADDLE LOCATION BASED UPON PREDICTED PATH
        
        cout << "BALL INTERCEPT: " + intToString(currentGame.getBallIntercept()) << std::endl;
        cout << "PADLE LOCATION: " + intToString(botPaddleLocation.x) << std::endl;

        if(botPaddleLocation.x < currentGame.getBallIntercept())
        {
            cout << ">>MOVING PADDLE RIGHT>>" << std::endl;
            controller.movePaddleRight();
        }
        if(botPaddleLocation.x > currentGame.getBallIntercept())
        {   
            cout << "<<MOVING PADDLE LEFT<<" << std::endl;
            controller.movePaddleLeft();
        }

	    frame_count++;
        
        cv::waitKey(1); //needed to show frame
    }
}


//-----------------------------------------------------------------------------------------------------------------
// Search for Moving Object
//-----------------------------------------------------------------------------------------------------------------
void searchForMovement(cv::Mat thresholdImage, cv::Mat &cameraFeed, gameBoard &currentGame)
{
    //notice how we use the '&' operator for objectDetected and cameraFeed. This is because we wish
    //to take the values passed into the function and manipulate them, rather than just working with a copy.
    //eg. we draw to the cameraFeed to be displayed in the main() function.

    bool objectDetected = false;
    int xpos, ypos; 
    float xVelocity, yVelocity;

    Point2i detectedObject;

    //these two vectors needed for output of findContours
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;

    cv::Mat temp;

    thresholdImage.copyTo(temp);


    cv::Rect roi(roiXMin, roiYMin, roiXMax-roiXMin, roiYMax-roiYMin);
    cv::Mat roi_temp = temp(roi); 

    //find contours of filtered image using openCV findContours function
    cv::findContours(temp,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );// retrieves external contours

    //if contours vector is not empty, we have found some objects
    if(contours.size()>0)
	    objectDetected = true;
    else
	    objectDetected = false;

    if(objectDetected)
    {
        //the largest contour is found at the end of the contours vector
        //we will simply assume that the biggest contour is the object we are looking for.
        vector< vector<Point> > largestContourVec;
        largestContourVec.push_back(contours.at(contours.size()-1));

        //make a bounding rectangle around the largest contour then find its centroid
        //this will be the object's final estimated position.
        objectBoundingRectangle = boundingRect(largestContourVec.at(0));

        // Find the X and Y Coordinate of the Ball
        xpos = objectBoundingRectangle.x+objectBoundingRectangle.width/2;
        ypos = objectBoundingRectangle.y+objectBoundingRectangle.height/2;

        //update the objects positions by changing the 'theObject' array values
        theObject[0] = xpos , theObject[1] = ypos;
    }
	
    int x = theObject[0]+roiXMin;
    int y = theObject[1]+roiYMin;
    detectedObject.x = theObject[0];
    detectedObject.y = theObject[1];
 
    // Compute Movement Velocity
    xVelocity = (xpos - prevXPos) / 0.333;  // Compute X-Direction Pixel Difference per Second
    yVelocity = (ypos - prevYPos) / 0.333;  // Compute Y-Direction Pixel Difference per Second
    velocity = sqrt( (xVelocity * xVelocity) + (yVelocity * yVelocity) ); // Compute Vector Magnitude

    // Print Movement Information to Frame
    //printMovementData(cameraFeed, x, y, xVelocity, yVelocity, velocity);

    // Track Gameplay
    currentGame.updateBallPosition(cameraFeed, detectedObject);

    // Store Positions for Subsequent Velocity Computations
    prevXPos = xpos;
    prevYPos = ypos;
    prevObjectDetected = objectDetected;
}

//-----------------------------------------------------------------------------------------------------------------
// Print Movement Data
//-----------------------------------------------------------------------------------------------------------------
void printMovementData(cv::Mat &cameraFeed, int x, int y, float xVelocity, float yVelocity, float velocity)
{
    if (GUI_DEBUG_PRINT)
    {
        // DRAW REGION OF INTEREST AROUND BOUNDING BOX
        line(cameraFeed,Point(roiXMin,roiYMin),Point(roiXMin,roiYMax),Scalar(255,0,0),2);
        line(cameraFeed,Point(roiXMin,roiYMax),Point(roiXMax,roiYMax),Scalar(255,0,0),2);
        line(cameraFeed,Point(roiXMax,roiYMax),Point(roiXMax,roiYMin),Scalar(255,0,0),2);
        line(cameraFeed,Point(roiXMin,roiYMin),Point(roiXMax,roiYMin),Scalar(255,0,0),2);

        //write the position of the object to the screen
        putText(cameraFeed,"(" + intToString(x)+","+intToString(y)+")",Point(x,y),1,1,Scalar(255,0,0),2);

        // Print Velocity Vector of Object
        putText(cameraFeed, "xVel:     " + floatToString(xVelocity),  Point(100,100),1,1,Scalar(255,0,0),2);
        putText(cameraFeed, "YVel:     " + floatToString(yVelocity),  Point(100,115),1,1,Scalar(255,0,0),2);
        putText(cameraFeed, "VelVect: "  + floatToString(velocity),   Point(100,130),1,1,Scalar(255,0,0),2);

        // Print the object Velocity
        if (yVelocity > 0) line(cameraFeed,Point(x,y),Point(x+xVelocity,y+yVelocity),Scalar(0,0,255),2); 
        else               line(cameraFeed,Point(x,y),Point(x+xVelocity,y+yVelocity),Scalar(255,0,0),2);
    }
}


//-----------------------------------------------------------------------------------------------------------------
//  on_trackbar
//  Generic Event Handler Function for Trackbar UI Window Change.
//  Trackbar Data is Pulled on Each Processed Frame - No Actions Req'd.
//-----------------------------------------------------------------------------------------------------------------
void on_trackbar( int, void*){}

//-----------------------------------------------------------------------------------------------------------------
//  createTrackbars
//  Function to Setup and Configure Trackbar UI Window
//-----------------------------------------------------------------------------------------------------------------
void createTrackbars()
{

    namedWindow(trackbarWindowName,0);      // Generate a Blank Window
	char TrackbarName[50];                  // Create a Memory Location for UI Window Text
	sprintf( TrackbarName, "H_MIN", H_MIN); // Print Hue Minimum Text to Window
	sprintf( TrackbarName, "H_MAX", H_MAX); // Print Hue Maximum Text to Window
	sprintf( TrackbarName, "S_MIN", S_MIN); // Print Saturation Minimum Text to Window
	sprintf( TrackbarName, "S_MAX", S_MAX); // Print Saturation Maximum Text to Window
	sprintf( TrackbarName, "V_MIN", V_MIN); // Print Value Minimum Text to Window
	sprintf( TrackbarName, "V_MAX", V_MAX); // Print Value Maximum Text to Window

	// create trackbars and insert them into window
	// 3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	// the max value the trackbar can move (eg. H_HIGH), 
	// and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
    createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar ); // Add Trackbar for Hue Minimum Threshold Slider
    createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar ); // Add Trackbar for Hue Maximum Threshold Slider
    createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar ); // Add Trackbar for Saturation Minimum Threshold Slider 
    createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar ); // Add Trackbar for Saturation Maximum Threshold Slider
    createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar ); // Add Trackbar for Value Minimum Threshold Slider
    createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar ); // Add Trackbar for Value Maximum Threshold Slider
}


//-----------------------------------------------------------------------------------------------------------------
//  findCorners
//  Find the corners in a Frame
//-----------------------------------------------------------------------------------------------------------------
void findCorners(cv::Mat src_gray, cv::Mat &cameraFeed)
{
    cv::Mat dst, dst_norm;
    dst = cv::Mat::zeros( src_gray.size(), CV_32FC1 );

    // Detector Parameters
    int blockSize = 2;
    int aperatureSize = 9; 
    double k = 0.04;
    int thresh = 200;
    int cornerCount = 0;
    Point2i foundCornerList[4];
    int xMidPoint = 1279/2;
    int yMidPoint = 719/2;
    // Detecting Corners
    cv::cornerHarris( src_gray, dst, blockSize, aperatureSize, k, BORDER_DEFAULT );

    // Normalizing
    cv::normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );

    // Drawing a Circle Around Corners
    for(int j = 0; j < dst_norm.rows; j++)
    {
        for(int i = 0; i < dst_norm.cols; i++ )
        {
            if( (int) dst_norm.at<float>(j,i) > thresh )
            {
                // Print Corner to Camera Feed
                cv::circle(cameraFeed, Point(i,j), 5, Scalar(0,255,0), 2, 8, 0);
               
                if(cornerCount < 4 )
                {
                    // Save corner to list
                    foundCornerList[cornerCount].x = i;
                    foundCornerList[cornerCount].y = j;
                }
                cornerCount++;
            }
        }
    }

    // If exactly 4 corners are found, add each to the corner list
    if(cornerCount == 4)
    {
        // !!!!! CORNER LIST ASSUMES THAT THERE IS EXACTLY 1 CORNER IN EACH QUADRANT!
        for(int i = 0; i < cornerCount; i++ )
        {
            if((foundCornerList[i].x < xMidPoint) && (foundCornerList[i].y < yMidPoint)) boardCorner[0] = foundCornerList[i];
            if((foundCornerList[i].x > xMidPoint) && (foundCornerList[i].y < yMidPoint)) boardCorner[1] = foundCornerList[i]; 
            if((foundCornerList[i].x < xMidPoint) && (foundCornerList[i].y > yMidPoint)) boardCorner[3] = foundCornerList[i];
            if((foundCornerList[i].x > xMidPoint) && (foundCornerList[i].y > yMidPoint)) boardCorner[2] = foundCornerList[i];
        }
    }

    cout << "Corners Found: " + intToString(cornerCount) << std::endl;
}


//-----------------------------------------------------------------------------------------------------------------
// Search for Paddles 
//-----------------------------------------------------------------------------------------------------------------
void searchForPaddles(cv::Rect roi, cv::Mat thresholdImage, cv::Mat &cameraFeed, Point2i &paddleLocation)
{
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    cv::Rect Pad = cv::Rect(0,0,0,0);
    cv::Mat temp;

    thresholdImage.copyTo(temp);

    cv::Mat roi_temp = temp(roi);
    cv::findContours(roi_temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);

    if(contours.size()>0)
    {
        vector< vector<Point> > largestContourVec;
        largestContourVec.push_back(contours.at(contours.size()-1));
        Pad = boundingRect(largestContourVec.at(0));
    }

    // Draw the ROI
    line(cameraFeed,Point(roi.x,roi.y),Point(roi.x+roi.width,roi.y),Scalar(255,0,255),2);
    line(cameraFeed,Point(roi.x,roi.y),Point(roi.x,roi.y+roi.height),Scalar(255,0,255),2);
    line(cameraFeed,Point(roi.x+roi.width,roi.y),Point(roi.x+roi.width,roi.y+roi.height),Scalar(255,0,255),2);
    line(cameraFeed,Point(roi.x,roi.y+roi.height),Point(roi.x+roi.width,roi.y+roi.height),Scalar(255,0,255),2);

    Pad.x = Pad.x + roi.x;
    Pad.y = Pad.y + roi.y;

    // Draw the Paddle
    if(contours.size()>0)
    {
        line(cameraFeed,Point(Pad.x,Pad.y),Point(Pad.x+Pad.width,Pad.y),Scalar(255,0,255),2);
        line(cameraFeed,Point(Pad.x,Pad.y),Point(Pad.x,Pad.y+Pad.height),Scalar(255,0,255),2);
        line(cameraFeed,Point(Pad.x+Pad.width,Pad.y),Point(Pad.x+Pad.width,Pad.y+Pad.height),Scalar(255,0,255),2);
        line(cameraFeed,Point(Pad.x,Pad.y+Pad.height),Point(Pad.x+Pad.width,Pad.y+Pad.height),Scalar(255,0,255),2);
        putText(cameraFeed,"(" + intToString(Pad.x) + "," + intToString(Pad.y) + ")",Point(Pad.x,Pad.y),1,1,Scalar(255,0,0),2);

    }

    // TODO: Verify if im setting to corner of paddle or middle of paddle.
    paddleLocation.x = Pad.x;
    paddleLocation.y = Pad.y;
}



//-----------------------------------------------------------------------------------------------------------------
// Int to String - Helper Function
//-----------------------------------------------------------------------------------------------------------------
string intToString(int number)
{
    std::stringstream ss;   // Generate a String Stream Object
    ss << number;           // Add integer as String
    return ss.str();        // Return Int as String object
}


//-----------------------------------------------------------------------------------------------------------------
//·Float to String Helper Function
//-----------------------------------------------------------------------------------------------------------------
string floatToString(float number)
{
    std::stringstream ss;  // Generate a String Stream Object
    ss << number;          // Add Number to String
    return ss.str();       // Return Float as String
}





