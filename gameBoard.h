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
| The Game Board class is desigend to encapsulate the information known of a 
| pong game currently in progress being viewed through a video stream. The 
| game board is responsible for understanding the state of the pong game, 
| directing analysis of ball and paddles through the video feed. 
|
*===========================================================================*/


#ifndef _GAME_BOARD_H_
#define _GAME_BOARD_H_

class gameBoard
{
    // CLASS MEMBERS
    int player1Score,player2Score;                                              // Member - Player Game Scores 
    int player1PaddleLocation, player2PaddleLocation;                           // Member - Player Paddle Positions
    int previousGameState, currentGameState;                                    // Member - Current and Previous Game States
    int previousBallXDirection, currentBallXDirection;                          // Member - Current and Previous Lateral Ball Movement Patterns
    int roiMinX, roiMaxX, roiMinY, roiMaxY;                                     // Member - Region of Interest Boundary Coordinates
    int frameCount;                                                             // Member - Count of Number of Frames Processed By Game Board
    int currentBallXIntercept;                                                  // Member - Current Estimate of Ball Intercept Position
    cv::Rect roi;                                                               // Member - Region of Interest for Ball Movement
    Point2i currentBallPosition, previousBallPosition;                          // Member - Current and Previous Ball Positions
    Point2i instBallVelocity;                                                   // Member - Instantaneous Ball Velocity
    LSFBallTracker ballTracker;                                                 // Member - Ball Tracking Object

    // CLASS METHODS
    public:
   
    gameBoard(cv::Rect inputROI, int roiMinimumX, int roiMaximumX,              // Constructor - Generic Constructor With Spec'd Region of Interest 
                                 int roiMinimumY, int roiMaximumY);
    gameBoard(cv::Rect);                                                        
    void updateBallPosition(cv::Mat&, Point2i);                                 // Modifier - General Update to Ball Position              
    void annotateFrame(cv::Mat &cameraFeed, Point2f compVel, int &endPaddlePos);// Modifier - Video Frame to Draw On

    void printStatus(void);                                                     // Status Printer
    
    int getPlayer1Score(void)           { return player1Score; }                // Getter - Returns Player 1's Score
    int getPlayer2Score(void)           { return player2Score; }                // Getter - Returns Player 2's Score
    int getPlayer1PaddleLocation(void)  { return player1PaddleLocation; }       // Getter - Returns Player 1's Paddle Location
    int getPlayer2PaddleLocation(void)  { return player2PaddleLocation; }       // Getter - Returns Player 2's Paddle Location
    Point2i getBallPosition(void)       { return currentBallPosition; }         // Getter - Returns the Current Ball Position
    
    int getBallIntercept(void);                                                 // Getter - Get Paddle Position
};

#endif
