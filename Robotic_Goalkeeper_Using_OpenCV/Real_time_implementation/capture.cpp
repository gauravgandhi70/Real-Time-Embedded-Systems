#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include<pthread.h>
#include <fcntl.h>
#include <termios.h>
#include "dynamixel_sdk.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

/*************************************************Defines for Robotic ARM****************************************/
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
#define DEVICENAME                      "/dev/ttyUSB1"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      95                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      250                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

//Initial position of the robot when it starts.
#define BASE_POS 500
#define LOWER_ARM_POS 140
#define UPPER_ARM_POS 272
#define WRIST_POS	  210
#define CLAW_POS      430
#define FRAME_CENTRE  320

	//Position variables of the arm
	int base= BASE_POS; //centre the base initially
	int lower_arm = LOWER_ARM_POS; // keep the arm bebt to the lowest
	int upper_arm = UPPER_ARM_POS; // keep the arm flexed initially
	int wrist = WRIST_POS; //Initial wrist motor position
	int claw = CLAW_POS; //keep the claw open initially
	dynamixel::PortHandler *portHandler;
	dynamixel::PacketHandler *packetHandler;

/*****************************************************************************************************************************************/
// Variables for frame capture and processing
Mat hsv1,thr1,thr2,mask,mat_frame;
IplImage* frame;
vector<Vec3f> circles;
int i=0;
CvCapture* capture = cvCreateCameraCapture(0);	
double wcet=0;
Scalar hsv_min1,hsv_max1, hsv_min2,hsv_max2;

uint8_t dxl_error = 0;                          // Dynamixel error variable

int dxl_comm_result = COMM_TX_FAIL;             // Communication result


// Variables for thread creation and Mutexes
pthread_t arm_act_thread,capture_thread, pre_processing_thread, ball_detection_thread, scheduler_thread;
pthread_attr_t arm_act_attr,capture_sched_attr, pre_processing_sched_attr, ball_detection_sched_attr, scheduler_sched_attr, main_sched_attr;
pthread_mutex_t capture_mutex, pre_processing_mutex , ball_detection_mutex,arm_mutex ;
int rt_max_prio, rt_min_prio, abortTest_10 = 0,abortTest_20=0;
double x_coordinate, radius;

// Structure for Thread Parameters

struct sched_param capture_param;
struct sched_param pre_processing_param;
struct sched_param ball_detection_param;
struct sched_param arm_act_param;


double prev_frame_time,curr_frame_time;
struct timespec frame_time_start, frame_time_stop;
double ave_framedt, ave_frame_rate, framedt;
unsigned int frame_count;

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

int init_motors(void){

 if (portHandler->openPort())
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
  if (portHandler->setBaudRate(BAUDRATE))
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
  //Enable all the Dynamixel Torque on all motors. The motors of the robotic arm are initialized using the dynamixel API in max torque configuration.
  //So that they can hold the position assigned to them
	  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BASE, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	  if (dxl_comm_result != COMM_SUCCESS){
		packetHandler->printTxRxResult(dxl_comm_result);
	  }else if ((dxl_error != 0)){
		packetHandler->printRxPacketError(dxl_error);
	  }else{
		printf("Base motor has been successfully connected \n");
	  }

	  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, LOWER_ARM_1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
		packetHandler->printTxRxResult(dxl_comm_result);
	  }else if ((dxl_error != 0)){
		packetHandler->printRxPacketError(dxl_error);
	  }else{
		printf("lower 1 motor has been successfully connected \n");
	  }

	  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, LOWER_ARM_2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
		packetHandler->printTxRxResult(dxl_comm_result);
	  }else if ((dxl_error != 0)){
		packetHandler->printRxPacketError(dxl_error);
	  }else{
		printf("lower 2 motor has been successfully connected \n");
	  }

	  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, UPPER_ARM_1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
		packetHandler->printTxRxResult(dxl_comm_result);
	  }else if ((dxl_error != 0)){
		packetHandler->printRxPacketError(dxl_error);
	  }else{
		printf("upper 1 motor has been successfully connected \n");
	  }

	  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, UPPER_ARM_2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
		packetHandler->printTxRxResult(dxl_comm_result);
	  }else if ((dxl_error != 0)){
		packetHandler->printRxPacketError(dxl_error);
	  }else{
		printf("upper 2 motor has been successfully connected \n");
	  }

	  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, WRIST, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
    packetHandler->printTxRxResult(dxl_comm_result);
    }else if ((dxl_error != 0)){
    packetHandler->printRxPacketError(dxl_error);
    }else{
    printf("wrist motor has been successfully connected \n");
    }

	  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, CLAW, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
		packetHandler->printTxRxResult(dxl_comm_result);
	  }else if ((dxl_error != 0)){
		packetHandler->printRxPacketError(dxl_error);
	  }else{
		printf("claw motor has been successfully connected \n");
	  }
/******************************************************************************************************************************************/

// Place the motors in the inititial position
	packetHandler->write2ByteTxRx(portHandler, BASE, ADDR_MX_GOAL_POSITION, base, &dxl_error);
	packetHandler->write2ByteTxRx(portHandler, LOWER_ARM_1, ADDR_MX_GOAL_POSITION, lower_arm, &dxl_error);
	packetHandler->write2ByteTxRx(portHandler, LOWER_ARM_2, ADDR_MX_GOAL_POSITION, 1024 - lower_arm, &dxl_error);
	packetHandler->write2ByteTxRx(portHandler, UPPER_ARM_1, ADDR_MX_GOAL_POSITION, upper_arm, &dxl_error);
	packetHandler->write2ByteTxRx(portHandler, UPPER_ARM_2, ADDR_MX_GOAL_POSITION, 1024 - upper_arm, &dxl_error);
	packetHandler->write2ByteTxRx(portHandler, WRIST, ADDR_MX_GOAL_POSITION, wrist, &dxl_error);
	packetHandler->write2ByteTxRx(portHandler, CLAW, ADDR_MX_GOAL_POSITION, claw, &dxl_error);
	
}

/*********************************************************************************************************************************************/
// New frame capture Thread
void *frame_capture(void *threadid)
{
	  	
	while(1)
	{
	// Capturing new frame with mutex protection
		pthread_mutex_lock(&capture_mutex);
			frame=cvQueryFrame(capture);
        if(!frame) break;
        printf("\ni");
         
       pthread_mutex_unlock(&pre_processing_mutex);
	usleep(150000);
	}
	

}


// Frame pre processing (Noise Reduction , background substraction) thread
void *pre_processing(void *threadid)
{
	while(1)
	{
	pthread_mutex_lock(&pre_processing_mutex);
	// If frame is not captures then dont proceed
	if(frame)
	 {
	 // Convert frame into matrix
	 
	 Mat mat_frame(frame);
	 	printf("\nii");		// Thread 2
	 	
	 	// Convert BGR image into HSV format
	cvtColor(mat_frame,hsv1,COLOR_BGR2HSV);
	
	// Providing threasholds for Colour detection filter to detect green ball
	hsv_min1 = Scalar(35, 50, 100); // Filter 1
	hsv_max1 = Scalar(65, 210,255);
	hsv_min2 = Scalar(35, 50, 50); // Filter 2
	hsv_max2 = Scalar(65, 100, 100);	

	//Apply the Filters
	inRange(hsv1,hsv_min1,hsv_max1,thr1);	// Detect Bright green
	inRange(hsv1,hsv_min2,hsv_max2,thr2);   // Detect Dark Green
	
	
	add(thr1,thr2,mask);		// Add 2 filtered images
	 
   	GaussianBlur(mask,mask,Size(11,11),0,0); // Blur the images to reduce the noise
	
	imshow("Mask",mask);
	cvWaitKey(1);
	}
	 pthread_mutex_unlock(&ball_detection_mutex);
		}
}

// Ball Detection and Center Detection using Hough circle Transform
void *ball_detection(void *threadid)
{
	while(1)
	{
	pthread_mutex_lock(&ball_detection_mutex);
	
	if(frame)
	 {
	 // Get frame into Matrix
	 Mat mat_frame(frame);
	 	printf("\niii");	// Thread number 3
	 	// Function for detecting circles from the filtered image
	HoughCircles(mask, circles, CV_HOUGH_GRADIENT, 1, mask.rows/8, 15, 15, 10, 0);

	
		// Get the center of the image
	 for( size_t i = 0; i < circles.size(); i++ )
        {
          Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
          int radius = cvRound(circles[i][2]);
          // circle center
          x_coordinate = circles[i][0];
          circle( mat_frame, center, 3, Scalar(0,0,255), -1, 8, 0 );
          //printf("\n C %lf %lf %lf %d",circles[0][0], circles[0][1],circles[0][2],i);
		   break;
          
        }
        imshow("Circles",mat_frame);
		cvWaitKey(1);
		}
		
		pthread_mutex_unlock(&arm_mutex);
        
	}
}

// Ball Speed Detection and arm actuation thread

void *arm_actuation(void *threadid)
{
		// Initialize PortHandler Structs
	// Set the port path
	// Get methods and members of PortHandlerLinux or PortHandlerWindows
	dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

	// Initialize PacketHandler Structs
	dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
	
	

	
				while(1)
				{
					pthread_mutex_lock(&arm_mutex);
        
    	      		if(x_coordinate > 370 && x_coordinate < 640)//Detection of the ball in the second half of the frame
    	      			{
    	      				base = (int)(base + (370 - x_coordinate)/7);//Equation for the movement of arm when ball is detected in the 
    	      																//second half of the frame
    	      			
    	      				if (base <270)
    	      				base = 270;//Prevent the overshoot of the Arm.
    	      				printf("\nNO");
    	      	
    	      			}
    	      		
    	      			
    	      		
    	      		else if(x_coordinate > 30 && x_coordinate < 320)//Equation for detection of the ball in the second part of the frame
    	      		{
    	      			base = (int)(base + (320 - x_coordinate)/5);
    	      			if (base >710)
    	      				base = 710;//Prevent the overshoot of the Arm.
    	      				printf("\nNO");
    	      	
    	      		}
    	      		  
    	      		
    	      	else 	{printf("\nYES");}
    	      	
    	      radius  = cvRound(circles[i][2]);
    	      
    	      
    	      printf("\n Center %f Radius %f Base %d Offset %d",circles[0][0],circles[0][2],base, (int)(FRAME_CENTRE - circles[i][0]));// Print the centre of the circle and the offset for the motor on the console.
			   pthread_mutex_unlock(&capture_mutex);
        
   packetHandler->write2ByteTxRx(portHandler, BASE, ADDR_MX_GOAL_POSITION, base, &dxl_error);//move the base motor
		
        }
}

void print_scheduler(void)
{
   int schedType;

   schedType = sched_getscheduler(getpid());

   switch(schedType)
   {
     case SCHED_FIFO:
	   printf("Pthread Policy is SCHED_FIFO\n");
	   break;
     case SCHED_OTHER:
	   printf("Pthread Policy is SCHED_OTHER\n");
       break;
     case SCHED_RR:
	   printf("Pthread Policy is SCHED_OTHER\n");
	   break;
     default:
       printf("Pthread Policy is UNKNOWN\n");
   }
}


int main( int argc, char** argv )
{
    
   int rc;
    rt_max_prio = sched_get_priority_max (SCHED_FIFO);
  	rt_min_prio = sched_get_priority_min (SCHED_FIFO);
	
    
	//Initialization of Robotic ARM
	init_motors();
	
	// Initializing all the thread attributes
	pthread_attr_init (&capture_sched_attr);
  pthread_attr_setinheritsched (&capture_sched_attr, PTHREAD_EXPLICIT_SCHED);
  pthread_attr_setschedpolicy (&capture_sched_attr, SCHED_FIFO);
    capture_param.sched_priority = rt_max_prio-1;	
	rc=sched_setscheduler(getpid(), SCHED_FIFO, &capture_param);
	
   pthread_attr_setschedparam(&capture_sched_attr,&capture_param);
  
		
    pthread_attr_init (&pre_processing_sched_attr);
  pthread_attr_setinheritsched (&pre_processing_sched_attr, PTHREAD_EXPLICIT_SCHED);
  pthread_attr_setschedpolicy (&pre_processing_sched_attr, SCHED_FIFO);
    pre_processing_param.sched_priority = rt_max_prio-2;
	rc=sched_setscheduler(getpid(), SCHED_FIFO, &pre_processing_param);
	
    pthread_attr_setschedparam (&pre_processing_sched_attr,&pre_processing_param);
  

 pthread_attr_init (&ball_detection_sched_attr);
  pthread_attr_setinheritsched (&ball_detection_sched_attr, PTHREAD_EXPLICIT_SCHED);
  pthread_attr_setschedpolicy (&ball_detection_sched_attr, SCHED_FIFO);
		ball_detection_param.sched_priority = rt_max_prio-3;
	rc=sched_setscheduler(getpid(), SCHED_FIFO, &ball_detection_param);
	
    pthread_attr_setschedparam(&ball_detection_sched_attr,&ball_detection_param);


	pthread_attr_init (&arm_act_attr);
   pthread_attr_setinheritsched (&arm_act_attr, PTHREAD_EXPLICIT_SCHED);
   pthread_attr_setschedpolicy (&arm_act_attr, SCHED_FIFO);
		arm_act_param.sched_priority = rt_max_prio-4;
	rc=sched_setscheduler(getpid(), SCHED_FIFO, &arm_act_param);
	
    pthread_attr_setschedparam(&arm_act_attr,&arm_act_param);
    
  print_scheduler();

  // Spawn Threads
  rc = pthread_create(&capture_thread,&capture_sched_attr,frame_capture ,NULL);
  
  if (rc)
    {
      printf("ERROR; capture pthread_create() rc is %d\n", rc); perror(NULL); exit(-1);
    }
    else
    {
    	printf("cap spawn");
    }
    
  
  rc = pthread_create(&pre_processing_thread, &pre_processing_sched_attr,pre_processing, NULL);
  if (rc)
    {
      printf("ERROR; pre_processing pthread_create() rc is %d\n", rc); perror(NULL); exit(-1);
    }

   rc = pthread_create(&ball_detection_thread, &ball_detection_sched_attr,ball_detection, NULL);
   if (rc)
    {
      printf("ERROR; ball_detection pthread_create() rc is %d\n", rc); perror(NULL); exit(-1);
    }

	 rc = pthread_create(&arm_act_thread, &arm_act_attr,arm_actuation, NULL);
   if (rc)
    {
      printf("ERROR; ball_detection pthread_create() rc is %d\n", rc); perror(NULL); exit(-1);
    }


    while(1)
    {
	}
	
};
