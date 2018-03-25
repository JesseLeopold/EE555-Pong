// LSF BALL TRACKER
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

// NAMESPACE REFERENCES
using namespace cv;
using namespace std;

// Function Prototypes
string intToStringLSF(int number);
string floatToStringLSF(float number);

#include "LSFBallTracker.h"

LSFBallTracker::LSFBallTracker()
{
    frameCount = 0;
}

void LSFBallTracker::addFrameData(Point2i newPosition)
{
    // Shift Each Ball Coordinate In A Frame 
    for (int i = 7; i > 0; i--)
    {
        pointArray[i] = pointArray[i-1];
    }
    
    // Insert New Ball Coordinate at Front of List
    pointArray[0] = newPosition;

    // Increment Frame Count
    frameCount++;
}

int LSFBallTracker::testMonotonic() { testMonotonic(7);}

int LSFBallTracker::testMonotonic(int numFrames)
{
    // Reset Each Direction Flag
    increasingX = increasingY = decreasingX = decreasingY = constantX = true;
    
    // Check Each Directional Case Against Each Location Pair
    for (int i = 0; i < numFrames; i++)
    {
        // If Any Coordinate Change Doesn't Match, Reset the Flag
        if(pointArray[i].x <= pointArray[i+1].x) increasingX = false;
        if(pointArray[i].x >= pointArray[i+1].x) decreasingX = false;        
        if(pointArray[i].x != pointArray[i+1].x) constantX   = false;
        if(pointArray[i].y <= pointArray[i+1].y) increasingY = false;
        if(pointArray[i].y >= pointArray[i+1].y) decreasingY = false;
    }

    // Print Flag States for Reference
    if (increasingX) cout << "X-INC"  << std::endl;
    if (decreasingX) cout << "X-DEC"  << std::endl;
    if (constantX)   cout << "X-CON"  << std::endl;
    if (increasingY) cout << "Y-INC"  << std::endl;
    if (decreasingY) cout << "Y-DEC"  << std::endl;

    // Return True Only on Monotonic Data
    if( (increasingX || decreasingX || constantX) && (increasingY || decreasingY) ) return 1;
    else                                                                            return 0;
}

Point2f LSFBallTracker::computeAvgVelocity() {computeAvgVelocity(8);}

Point2f LSFBallTracker::computeAvgVelocity(int numFrames)
{
    // Local Variable Declarations
    Point2f ballVelocity;
    float sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;
    float avgX = 0, avgY = 0, slope = 0;

    // Compute X and Y Averages
    for (int i = 0; i < numFrames; i++)
    {
        sumX += pointArray[i].x;
        sumY += pointArray[i].y;
    }
    avgX = sumX /  ((float) numFrames);
    avgY = sumY /  ((float) numFrames);


    // Compute Product Sums
    for(int i = 0; i < numFrames; i++)
    {
        sumXY += (pointArray[i].x - avgX) * (pointArray[i].y - avgY);
        sumXX += (pointArray[i].x - avgX) * (pointArray[i].x - avgX);
    }

    // Compute Slope
    slope = sumXY / sumXX;

    //-------------------------------------------------
    if(sumXX == 0)
    {
        ballVelocity.x = 0;
        ballVelocity.y = 1;
    }
    else if( (increasingX) && (slope > 0) && (slope < 100) )
    {
        ballVelocity.x = 1;
        ballVelocity.y = slope;
    }
    else if( (decreasingX) && (slope < 0) && (slope > -100) )
    {
        ballVelocity.x = 1;
        ballVelocity.y =  abs(slope);
    }
    else
    {
        ballVelocity.x = ballVelocity.y = 0;
    }

    computedVelocity = ballVelocity;

    cout << 
        ">>> X:Data:{" + intToStringLSF(pointArray[0].x) + " " +
                         intToStringLSF(pointArray[1].x) + " " +
                         intToStringLSF(pointArray[2].x) + " " +
                         intToStringLSF(pointArray[3].x) + " " +
                         intToStringLSF(pointArray[4].x) + " " +
                         intToStringLSF(pointArray[5].x) + " " +
                         intToStringLSF(pointArray[6].x) + " " +
                         intToStringLSF(pointArray[7].x) + "} " <<std::endl;
        cout << 
        ">>> Y:Data:{" + intToStringLSF(pointArray[0].y) + " " +
                         intToStringLSF(pointArray[1].y) + " " +
                         intToStringLSF(pointArray[2].y) + " " +
                         intToStringLSF(pointArray[3].y) + " " +
                         intToStringLSF(pointArray[4].y) + " " +
                         intToStringLSF(pointArray[5].y) + " " +
                         intToStringLSF(pointArray[6].y) + " " +
                         intToStringLSF(pointArray[7].y) + "} " <<std::endl;
        cout <<
        std::string(">>>|----sumX-----|") +
                       "|----sumY----|" +
                       "|----sumXY---|" +
                       "|----sumXX---|" +
                       "|----slope---|" +
                       "|---BVel.x---|" +
                       "|---BVel.y---|" + 
                       "\n";

        cout <<
        ">>>|\t" + floatToStringLSF(sumX)              + " \t|" + 
           "|\t" + floatToStringLSF(sumY)              + " \t|" + 
           "|\t" + floatToStringLSF(sumXY)             + " \t|" + 
           "|\t" + floatToStringLSF(sumXX)             + " \t|" + 
           "|\t" + floatToStringLSF(slope)             + " \t|" + 
           "|\t" + floatToStringLSF(ballVelocity.x)    + " \t|" + 
           "|\t" + floatToStringLSF(ballVelocity.y)    + " \t|" +
           "\n";

    return ballVelocity;
}


//-----------------------------------------------------------------------------------------------------------------
// Int to String - Helper Function
// //-----------------------------------------------------------------------------------------------------------------
string intToStringLSF(int number)
{
    std::stringstream ss;   // Generate a String Stream Object
    ss << number;           // Add integer as String
    return ss.str();        // Return Int as String object
}


//-----------------------------------------------------------------------------------------------------------------
//·Float to String Helper Function
//-----------------------------------------------------------------------------------------------------------------
string floatToStringLSF(float number)
{
    std::stringstream ss;  // Generate a String Stream Object
    ss << number;          // Add Number to String
    return ss.str();       // Return Float as String
}
