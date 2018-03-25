/*=============================================================================
| Assignment: Final - Flight Path Exploration
|
| Author: Jesse Leopold
| Language: C++
|
| Class: EE-555 Embedded Systems II: Embedded Software
| Instructor: Allan Douglas
+-----------------------------------------------------------------------------
|
| Description: 
| The LSF Ball Tracker class is designed to analyze the path of the ball moving
| across a video feed representation of pong game. This class is to retain the 
| historical X-Y positions of the ball for enough frames to understand where
| the ball is headed.
|
*===========================================================================*/

#ifndef _LSF_BALL_TRACKER_H_
#define _LSF_BALL_TRACKER_H_

class LSFBallTracker
{
    // CLASS MEMBERS
    cv::Rect roi;                                                           // Member - Region of Interest Tracker for Ball Moment
    Point2i pointArray[8];                                                  // Member - Ball Position History Array
    Point2f computedVelocity;                                               // Member - Velocity from Ball Position History
    int frameCount;                                                         // Member - Number of Processed Frames
    bool increasingX,increasingY, decreasingX, decreasingY,constantX;       // Member - Direction Tracking Variables

    // CLASS METHODS
    public:
    LSFBallTracker();                                                       // Constructor - General Least Squares Fit Constructor
    void addFrameData(Point2i);                                             // Modifier - New Data Point Processor
    int testMonotonic(void);                                                // Getter - Tests for Monotonicity for All Frames
    int testMonotonic(int numFrames);                                       // Getter - Tests For Monotonicity for a Specified Number of Frames
    Point2f computeAvgVelocity(void);                                       // Getter - Compute Average Velocity for All Frames
    Point2f computeAvgVelocity(int numFrames);                              // Getter - Compute Average Velocity for a Specified Number of Frames
    Point2i getPosition(void) { return pointArray[8]; }                     // Getter - Returns Last Recorded Position
    Point2f getComputedVelocity(void) {return computedVelocity;}            // Getter - Returns the LSF Computed Velocity for All Frames
    void setROI(cv::Rect inputROI) { roi = inputROI;}                       // Setter - Sets Region of Interest
};

#endif

