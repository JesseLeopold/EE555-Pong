/*=============================================================================
| Assignment: Final - Paddle Driver
|
| Author: Jesse Leopold
| Language: C++
|
| Class: EE-555 Embedded Systems II: Embedded Software
| Instructor: Allan Douglas
+-----------------------------------------------------------------------------
|
| Description: 
| The paddle drive class is desigend to control the microcontroller associated
| with paddle control. The microcontroller is managed using a UART interface
| through a USB port.
|
*===========================================================================*/


#ifndef _PADDLE_DRIVER_
#define _PADDLE_DRIVER_

class paddleDriver
{
    // CLASS MEMBERS
    String USBInterface;                    // Member - Describes Interface Used to Communicate with Microcontroller
    int fd;                                 // Member - File Descriptor of USB Interface
    char write_buffer[10];                   // Member - Write Buffer
    char read_buffer[10];                    // Member - Read Buffer

    // CLASS METHODS
    public:
    paddleDriver();                         // Constructior - Creates a New Driver

    void movePaddleLeft(void);              // Modifier - Configures a Move Left Command to Controller
    void movePaddleRight(void);             // Modifier - Configures a Move Right Command to Controller
    void serveBall(void);                   // Modifier - Configures a Serve Ball Command to Controller
    void transmitAndCheck(void);            // Modifier - Sends a Configured Command
};

#endif
