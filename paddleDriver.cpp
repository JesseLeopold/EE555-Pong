
// GENERAL GLOBAL INCLUDES
#include <stdio.h>      // Standard input/output definitions
#include <fcntl.h>      // File Control Definitions
#include <termios.h>    // POSIX Terminal Control Definitions
#include <unistd.h>     // UNIX Standard Definitions
#include <errno.h>      // ERROR Number Definitions
#include <string.h>     // String Operator Definitions

#include "paddleDriver.h"

// NAMESPACE REFERENCES
using namespace cv;
using namespace std;

// FUNCTION PROTOTYPES
void transmitAndCheck(void);

paddleDriver::paddleDriver(void)
{
    struct termios options;           // Terminal options

  fd = open(USB_INTERFACE,O_RDWR | O_NOCTTY);   // Open tty device for RD and WR

  if(fd == 1) {
     printf("\n  Error! in Opening ttyS0\n");
  }
  else
     printf("\n  ttyUSB0 Opened Successfully\n");

    tcgetattr(fd, &options);                    // Get the current options for the port
    cfsetispeed(&options, B115200);             // Set the baud rates to 115200          
    cfsetospeed(&options, B115200);                   
    options.c_cflag |= (CLOCAL | CREAD);        // Enable the receiver and set local mode           
    options.c_cflag &= ~PARENB;                 // No parity                 
    options.c_cflag &= ~CSTOPB;                 // 1 stop bit                  
    options.c_cflag &= ~CSIZE;                  // Mask data size         
    options.c_cflag |= CS8;                     // 8 bits
    options.c_cflag &= ~CRTSCTS;                // Disable hardware flow control  

    options.c_lflag &= ~(ICANON | ECHO | ISIG); // Enable Data to be Processed as Raw Input
     
    tcsetattr(fd, TCSANOW, &options);           // Apply options immediately
    fcntl(fd, F_SETFL, FNDELAY);    
}


void paddleDriver::movePaddleLeft(void)
{
    memset(&write_buffer[0],0,sizeof(write_buffer));
    write_buffer[0] = 'L';
    write_buffer[1] = '1';
    write_buffer[2] = '0';
    transmitAndCheck();
}


void paddleDriver::movePaddleRight(void)
{
    memset(&write_buffer[0],0,sizeof(write_buffer));
    write_buffer[0] = 'R';
    write_buffer[1] = '1';
    write_buffer[2] = '0';
    transmitAndCheck();

}


void paddleDriver::serveBall(void)
{
    memset(&write_buffer[0],0,sizeof(write_buffer));
    write_buffer[0] = 'S';
    transmitAndCheck();
}


void paddleDriver::transmitAndCheck(void)
{
    int bytes_written = write(fd, write_buffer, 1);

    if (bytes_written == -1)    printf("W Error=%d\n", errno);
    else                        printf("Wrote=%c\n", write_buffer[0]);
 
    // TODO: Reenable checks at some later point in time
    //while(read(fd, &read_buffer, 1) != 1) {}

    printf("Read=%c\n", read_buffer[0]); 
}
