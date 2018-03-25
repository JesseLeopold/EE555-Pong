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

#include "LSFBallTracker.cpp"

#include "gameBoard.h"

// NAMESPACE REFERENCES
using namespace cv;
using namespace std;

// GAME TRACKING VARIABLES
enum gameState {BALL_OUT_OF_PLAY=1, BALL_APPROACHING, BALL_DEPARTING};
enum ballXDir  {MOVING_LEFT, MOVING_RIGHT, MOVING_VERTICALLY};

float ANNOTATION_SPEEDUP = 10;

// VERBOSE DEBUG PRINT SWITCHES
bool DBG_TERM_FLT_PTH_PRINT    = false;
bool DBG_TERM_GAME_STATE_PRINT = true;

// Function Prototypes
string intToStringGB(int number);
string floatToStringGB(float number);

gameBoard::gameBoard(cv::Rect inputROI, int roiMinimumX, int roiMaximumX, int roiMinimumY, int roiMaximumY)
{
    // INITIALIZE CLASS VARIABLES
    roi = inputROI;                                                     // Store Input Region of Interes
    roiMinX = roiMinimumX;
    roiMaxX = roiMaximumX;
    roiMinY = roiMinimumY;
    roiMaxY = roiMaximumY;
    player1Score = player2Score = 0;                                    // Initialize Player Scores
    player1PaddleLocation = player2PaddleLocation = 0;                  // Initialize Player Paddle Locations
    currentGameState = previousGameState = BALL_OUT_OF_PLAY;            // Specify Game State Tracking
    currentBallXDirection = previousBallXDirection = MOVING_VERTICALLY; // Specify Ball X Movement Tracking
    ballTracker.setROI(inputROI);                                       // Create a LSF Ball Tracker
}

void gameBoard::updateBallPosition(cv::Mat &cameraFeed, Point2i newBallPosition)
{
    // LOCAL VARIABLE DECLARATIONS
    Point2f currentAvgVelocity;
    currentAvgVelocity.x = currentAvgVelocity.y = 0.0;
    
    // UPDATE GAME TRACKING FIELDS
    currentBallPosition = newBallPosition;
    instBallVelocity = currentBallPosition - previousBallPosition;

    // TRACK Y DIRECTION MOVEMENT FOR GENERAL REFERENCE
    if      (instBallVelocity.y > 0)    currentGameState = BALL_APPROACHING;
    else if (instBallVelocity.y < 0)    currentGameState = BALL_DEPARTING;
    else                                currentGameState = BALL_OUT_OF_PLAY;

    // TRACK X DIRECTION MOVEMENT FOR GENERAL REFERENCE
    if      (instBallVelocity.x > 0)   currentBallXDirection = MOVING_RIGHT;
    else if (instBallVelocity.x < 0)   currentBallXDirection = MOVING_LEFT;
    else                               currentBallXDirection = MOVING_VERTICALLY;

    ballTracker.addFrameData(currentBallPosition);

    // PRINT CURRENT GAME STATE STATUS
    if(DBG_TERM_GAME_STATE_PRINT) printStatus();


    if (currentGameState == BALL_APPROACHING)
    {
        int monotonicGuess = 7;
        bool monotonicFound = false;

        while( (monotonicGuess > 1) && (monotonicFound == false) )
        {
            cout << "Monotonic Guess: " + intToString(monotonicGuess) << std::endl;
            if (ballTracker.testMonotonic(monotonicGuess))
            {
                monotonicFound = true;
                currentAvgVelocity = ballTracker.computeAvgVelocity(monotonicGuess);
                if(currentAvgVelocity.y >= 1)
                    annotateFrame(cameraFeed, currentAvgVelocity,currentBallXIntercept);
            }

            monotonicGuess--;
        }
    }
    
    // UPDATE FRAME COUNT
    frameCount++;
 
    // UPDATE GAME STATE
    previousGameState = currentGameState;
    previousBallXDirection = currentBallXDirection;
    previousBallPosition = currentBallPosition;
}

void gameBoard::printStatus(void)
{

    string gameStateText, ballXDirText;
    enum ballXDir  {MOVING_LEFT, MOVING_RIGHT, MOVING_VERTICALLY};

    switch (currentGameState)
    {
        case(BALL_OUT_OF_PLAY): gameStateText = " Out of Play "; break;
        case(BALL_APPROACHING): gameStateText = " Approaching "; break;
        case(BALL_DEPARTING):   gameStateText = "  Departing  "; break;
        default:                gameStateText = " <UNDEFINED> "; break;
    }

    switch (currentBallXDirection)
    {
        case(MOVING_LEFT):       ballXDirText = " Moving Left "; break;
        case(MOVING_RIGHT):      ballXDirText = " Moving Rght "; break;
        case(MOVING_VERTICALLY): ballXDirText = " Moving Vert "; break;
        default:                 ballXDirText = " <UNDEFINED> "; break;
    }


    if(frameCount % 20 == 0)
    {
        cout <<
        std::string("|--Frame Count--|") +
                    "|-Game Status--|" +
                    "|-Ball X Dir---|" +
                    "|----XPos------|" +
                    "|----YPos------|" +
                    "|--Inst X-Vel--|" +
                    "|--Inst Y-Vel--|" +
                    "|--BTrkMono----|" +
                    "|--Avg X Vel---|" + 
                    "|--Avg Y Vel---|"
                    << std::endl;
    }

    cout << 
        "|\t" + intToStringGB(frameCount)                             + " \t|" +                
        "|"   + gameStateText                                         + " |" + 
        "|"   + ballXDirText                                          + " |" + 
        "|\t" + intToStringGB(currentBallPosition.x)                  + " \t|" + 
        "|\t" + intToStringGB(currentBallPosition.y)                  + " \t|" + 
        "|\t" + intToStringGB(instBallVelocity.x)                     + " \t|" + 
        "|\t" + intToStringGB(instBallVelocity.y)                     + " \t|" + 
        "|\t" + intToStringGB(ballTracker.testMonotonic())            + " \t|" +
        "|\t" + floatToStringGB(ballTracker.getComputedVelocity().x)  + " \t|" +
        "|\t" + floatToStringGB(ballTracker.getComputedVelocity().y)  + " \t|"
        << std::endl;
}

void gameBoard::annotateFrame(cv::Mat &cameraFeed, Point2f compVel, int &endPaddlePos)
{
    // Compute and Draw Estimated Flight Path
    int tempXStart, tempYStart, tempXEnd, tempYEnd, tempBallXDir, predictionCount;
    float avgXVelocity, avgYVelocity;
    
    tempXStart = currentBallPosition.x;
    tempYStart = currentBallPosition.y;
    tempBallXDir = currentBallXDirection;
    predictionCount = 0;
    avgXVelocity = compVel.x * ANNOTATION_SPEEDUP;
    avgYVelocity = compVel.y * ANNOTATION_SPEEDUP; 

    if(DBG_TERM_FLT_PTH_PRINT)
    {
        cout << "-------------------STARTING PREDICTION LOOP--------------------------\n";
        cout << "X Position: " + intToStringGB(currentBallPosition.x) + "\n";
        cout << "Y Position: " + intToStringGB(currentBallPosition.y) + "\n";
        cout << "Average X Velocity: " +  floatToStringGB(avgXVelocity) + "\n";
        cout << "Average Y Velocity: " +  floatToStringGB(avgYVelocity) + "\n";
    }
    
    while(tempYStart <= roiMaxY)                            // Draw a Line Until Bottom Of Play Area Reached
    {
        if (tempBallXDir == MOVING_LEFT)                    // If ball is moving left, Move End Coordinate Left
        {
            tempXEnd = tempXStart - avgXVelocity;           // Compute X Position After 1 Frame
            tempYEnd = tempYStart + avgYVelocity;           // Compute Y Position After 1 Frame  
            
            if (tempXEnd < 0)                               // Compare Ball Position Against Left Boundary
            {
                tempXEnd = abs(tempXEnd);
                tempBallXDir = MOVING_RIGHT;                // Simulate a Wall Hit By Switching Ball Direction
            }
        }
        else if (tempBallXDir == MOVING_RIGHT)              // If Ball is Moving Rright
        {
            tempXEnd = tempXStart + avgXVelocity;           // Compute X Position After 1 Frame
            tempYEnd = tempYStart + avgYVelocity;           // Compute Y Position After 1 Frame
            
            if (tempXEnd > (roiMaxX - roiMinX))             // Compare Ball Position Against Right Boundary
            {
                tempXEnd -= (tempXEnd - (roiMaxX-roiMinX));
                tempBallXDir = MOVING_LEFT;                 // Simulate a Wall Hit By Switching Ball Direction 
            }
        }
        else                                                // If Ball is Moving Straight Down
        {
            tempXEnd = tempXStart - avgXVelocity;           // Compute X Position After 1 Frame
            tempYEnd = tempYStart + avgYVelocity;           // Compute Y Position After 1 Frame
            
            if (tempXEnd < 0)                   tempBallXDir = MOVING_RIGHT;    // Simulate a Wall Hit By Switching Ball Direction
            if (tempXEnd > (roiMaxX - roiMinX)) tempBallXDir = MOVING_LEFT;     // Simulate a Wall Hit By Switching Ball Direction 

        }

        // Print Out Estimated Ball Flight Path Segment
         line(cameraFeed,Point(tempXStart+roiMinX,tempYStart+roiMinY),Point(tempXEnd+roiMinX,tempYEnd+roiMinY),Scalar(0,0,255),2);

        if(DBG_TERM_FLT_PTH_PRINT)
        {
            cout << "Iteration: " + intToStringGB(predictionCount) + "\n";
            if (tempBallXDir == MOVING_LEFT) cout << "Direction: X-MOVING-LEFT\n"; 
            else                             cout << "Direction: X-MOVING-RIGHT\n";
            cout << "Start: (" + intToStringGB(tempXStart) + "," + intToStringGB(tempYStart) + ")\n";
            cout << "End:   (" + intToStringGB(tempXEnd)   + "," + intToStringGB(tempYEnd)   + ")\n";
        }
        
        // Update Start and End Position For Next Incremental Loop 
        tempXStart = tempXEnd;
        tempYStart = tempYEnd;


        // Increment Count of Prediction Iterations
        predictionCount++;
    }
    
    if(DBG_TERM_FLT_PTH_PRINT) cout << "-------------------ENDING PREDICTION LOOP--------------------------\n";

    endPaddlePos = tempXEnd;

}



int gameBoard::getBallIntercept(void)
{
    switch(currentGameState)
    {
        case(BALL_OUT_OF_PLAY): cout << "STATE: OUT OF PLAY" << std::endl; break;
        case(BALL_APPROACHING): cout << "STATE: APPROACHING" << std::endl; break;
        case(BALL_DEPARTING):   cout << "STATE: DEPARTING" << std::endl;   break;
    }

    if( (currentGameState == BALL_OUT_OF_PLAY) || (currentGameState == BALL_DEPARTING) ) return 650;
    else                                                                                 return currentBallXIntercept;
}


//-----------------------------------------------------------------------------------------------------------------
// Int to String - Helper Function
//-----------------------------------------------------------------------------------------------------------------
string intToStringGB(int number)
{
    std::stringstream ss;   // Generate a String Stream Object
    ss << number;           // Add integer as String
    return ss.str();        // Return Int as String object
}


//-----------------------------------------------------------------------------------------------------------------
//·Float to String Helper Function
//-----------------------------------------------------------------------------------------------------------------
string floatToStringGB(float number)
{
    std::stringstream ss;  // Generate a String Stream Object
    ss << number;          // Add Number to String
    return ss.str();       // Return Float as String
}


