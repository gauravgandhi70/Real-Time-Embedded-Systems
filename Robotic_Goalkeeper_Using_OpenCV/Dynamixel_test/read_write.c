/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

//
// *********     Read and Write Example      *********
//
//
// Available DXL model on this example : All models using Protocol 1.0
// This example is designed for using a Dynamixel MX-28, and an USB2DYNAMIXEL.
// To use another Dynamixel model, such as X series, see their details in E-Manual(support.robotis.com) and edit below "#define"d variables yourself.
// Be sure that Dynamixel MX properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 1000000 [1M])
//

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>

#include <stdlib.h>
#include <stdio.h>
#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library

// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting

//Defining motor IDs
#define BASE                          1
#define LOWER_ARM_1                   2
#define LOWER_ARM_2                   3
#define UPPER_ARM_1                   4
#define UPPER_ARM_2                   5
#define WRIST                         6
#define CLAW                          7

#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      95                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      250                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

#define OFFSET 20

#define CENTRE 500
#define RIGHT 200
#define LEFT  850

int getch()
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

int main()
{
  // Initialize PortHandler Structs
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  int port_num = portHandler(DEVICENAME);
  int x_cord=0;
  struct timespec start_time;
  struct timespec end_time;
  int exec_time;

  // Initialize PacketHandler Structs
  packetHandler();

  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  int dxl_goal_position[2] = { DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE };         // Goal position

   uint8_t dxl_error = 0;                          // Dynamixel error
  // uint16_t dxl_present_position = 0;              // Present position

  char decision;
  //Variables for position
  int base= 500; //centre the base initially
  int lower_arm = 100; // keep the arm bebt to the lowest
  int upper_arm = 512; // keep the arm flexed initially
  int wrist = 100; //
  int claw =220; //keep the claw open initially

  // Open port
  if (openPort(port_num))
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (setBaudRate(port_num, BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }
/***********************************************Robotic arm motor initialization*************************************/
  //Enable all the Dynamixel Torque on all motors
  write1ByteTxRx(port_num, PROTOCOL_VERSION, BASE, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS){
    printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
  }else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0){
    printRxPacketError(PROTOCOL_VERSION, dxl_error);
  }else{
    printf("Base motor has been successfully connected \n");
  }

  write1ByteTxRx(port_num, PROTOCOL_VERSION, LOWER_ARM_1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS){
    printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
  }else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0){
    printRxPacketError(PROTOCOL_VERSION, dxl_error);
  }else{
    printf("Lower arm 1 has been successfully connected \n");
  }

  write1ByteTxRx(port_num, PROTOCOL_VERSION, LOWER_ARM_2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS){
    printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
  }else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0){
    printRxPacketError(PROTOCOL_VERSION, dxl_error);
  }else{
    printf("Lower arm 2 has been successfully connected \n");
  }

  write1ByteTxRx(port_num, PROTOCOL_VERSION, UPPER_ARM_1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS){
    printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
  }else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0){
    printRxPacketError(PROTOCOL_VERSION, dxl_error);
  }else{
    printf("Upper arm 1 has been successfully connected \n");
  }

  write1ByteTxRx(port_num, PROTOCOL_VERSION, UPPER_ARM_2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS){
    printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
  }else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0){
    printRxPacketError(PROTOCOL_VERSION, dxl_error);
  }else{
    printf("Upper arm 2 has been successfully connected \n");
  }

  write1ByteTxRx(port_num, PROTOCOL_VERSION, WRIST, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS){
    printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
  }else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0){
    printRxPacketError(PROTOCOL_VERSION, dxl_error);
  }else{
    printf("Wrist motor has been successfully connected \n");
  }

  write1ByteTxRx(port_num, PROTOCOL_VERSION, CLAW, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS){
    printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
  }else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0){
    printRxPacketError(PROTOCOL_VERSION, dxl_error);
  }else{
    printf("CLAW has been successfully connected \n");
  }
/*****************************************************************************************************************/
write2ByteTxRx(port_num, PROTOCOL_VERSION, BASE, ADDR_MX_GOAL_POSITION, base);

  while (1)
  {
    printf("enter x cordinate\n");
    scanf("%d",&x_cord);

    clock_gettime(CLOCK_REALTIME, &start_time);
    base = 500 + (320 - x_cord);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, BASE, ADDR_MX_GOAL_POSITION, base);
    clock_gettime(CLOCK_REALTIME, &end_time);

    exec_time = (end_time.tv_nsec - start_time.tv_nsec) / 1000000;
    printf("the time of last motion is %d\n", exec_time);
  }

  // Disable Dynamixel Torque


  // Close port
  closePort(port_num);

  return 0;
}
