#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include<pthread.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;
Mat hsv1,thr1,thr2,mask,mat_frame;
    IplImage* frame;
	vector<Vec3f> circles;
	int i=0;
	CvCapture* capture = cvCreateCameraCapture(1);	
double wcet=0;

pthread_t capture_thread, pre_processing_thread, ball_detection_thread, scheduler_thread;
pthread_attr_t capture_sched_attr, pre_processing_sched_attr, ball_detection_sched_attr, scheduler_sched_attr, main_sched_attr;
pthread_mutex_t capture_mutex, pre_processing_mutex , ball_detection_mutex ;
int rt_max_prio, rt_min_prio, abortTest_10 = 0,abortTest_20=0;


struct sched_param capture_param;
struct sched_param pre_processing_param;
struct sched_param ball_detection_param;


Scalar hsv_min1,hsv_max1, hsv_min2,hsv_max2;

	
	
double prev_frame_time,curr_frame_time;
	struct timespec frame_time_start, frame_time_stop;
	double ave_framedt, ave_frame_rate, framedt;
    unsigned int frame_count;

void *frame_capture(void *threadid)
{
	  	
	while(1)
	{
		pthread_mutex_lock(&capture_mutex);
			frame=cvQueryFrame(capture);
        if(!frame) break;
        printf("\ni");
         Mat mat_frame(frame);
          imshow("Circles",mat_frame);
		cvWaitKey(1);
       pthread_mutex_unlock(&pre_processing_mutex);
	usleep(1000);
	}

}
void *pre_processing(void *threadid)
{
	while(1)
	{
	pthread_mutex_lock(&pre_processing_mutex);

	if(frame)
	 {
	 Mat mat_frame(frame);
	 	printf("\nii");		
	cvtColor(mat_frame,hsv1,COLOR_BGR2HSV);
	
	hsv_min1 = Scalar(35, 50, 100);
	hsv_max1 = Scalar(65, 210,255);
	hsv_min2 = Scalar(35, 50, 50);
	hsv_max2 = Scalar(65, 100, 100);	

	
	inRange(hsv1,hsv_min1,hsv_max1,thr1);	
	inRange(hsv1,hsv_min2,hsv_max2,thr2);
	
	
	add(thr1,thr2,mask);
	 
   	GaussianBlur(mask,mask,Size(11,11),0,0);
	//imshow("Thr1",thr1);
	//imshow("Thr2",thr2);
	
	imshow("Mask",mask);
	cvWaitKey(1);
	//sleep(10);
	}
	 pthread_mutex_unlock(&capture_mutex);
		}
}

void *ball_detection(void *threadid)
{
	while(1)
	{
	printf("\niii");
	HoughCircles(mask, circles, CV_HOUGH_GRADIENT, 1, mask.rows/8, 15, 15, 10, 0);

	
	Mat mat_frame(frame);
	
	 for( size_t i = 0; i < circles.size(); i++ )
        {
          Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
          int radius = cvRound(circles[i][2]);
          // circle center
          circle( mat_frame, center, 3, Scalar(0,0,255), -1, 8, 0 );
          printf("\n C %lf %lf %lf %d",circles[0][0], circles[0][1],circles[0][2],i);
		   break;
          
        }
      //  imshow("Circles",mat_frame);
		cvWaitKey(1);
        sleep(200);
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
    //	cvNamedWindow("HSV", CV_WINDOW_AUTOSIZE);
   int rc;
    rt_max_prio = sched_get_priority_max (SCHED_FIFO);
  	rt_min_prio = sched_get_priority_min (SCHED_FIFO);
	
      
	hsv_min1 = Scalar(35, 50, 100);
	hsv_max1 = Scalar(65, 210,255);
	hsv_min2 = Scalar(35, 50, 50);
	hsv_max2 = Scalar(65, 100, 100);
	
	
	pthread_attr_init (&capture_sched_attr);
  pthread_attr_setinheritsched (&capture_sched_attr, PTHREAD_EXPLICIT_SCHED);
  pthread_attr_setschedpolicy (&capture_sched_attr, SCHED_FIFO);
    capture_param.sched_priority = rt_max_prio-1;	
	//rc=sched_setscheduler(getpid(), SCHED_FIFO, &capture_param);
	
   pthread_attr_setschedparam(&capture_sched_attr,&capture_param);
  
		
    pthread_attr_init (&pre_processing_sched_attr);
  pthread_attr_setinheritsched (&pre_processing_sched_attr, PTHREAD_EXPLICIT_SCHED);
  pthread_attr_setschedpolicy (&pre_processing_sched_attr, SCHED_FIFO);
    pre_processing_param.sched_priority = rt_max_prio-2;
	//rc=sched_setscheduler(getpid(), SCHED_FIFO, &pre_processing_param);
	
    pthread_attr_setschedparam (&pre_processing_sched_attr,&pre_processing_param);
  
	
	  pthread_attr_init (&ball_detection_sched_attr);
  pthread_attr_setinheritsched (&ball_detection_sched_attr, PTHREAD_EXPLICIT_SCHED);
  pthread_attr_setschedpolicy (&ball_detection_sched_attr, SCHED_FIFO);
		ball_detection_param.sched_priority = rt_max_prio-3;
	rc=sched_setscheduler(getpid(), SCHED_FIFO, &ball_detection_param);
	
    pthread_attr_setschedparam(&ball_detection_sched_attr,&ball_detection_param);
  
 
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

  /*  rc = pthread_create(&ball_detection_thread, &ball_detection_sched_attr,ball_detection, NULL);
   if (rc)
    {
      printf("ERROR; ball_detection pthread_create() rc is %d\n", rc); perror(NULL); exit(-1);
    }*/

	

    while(1)
    {
	}
	
};
