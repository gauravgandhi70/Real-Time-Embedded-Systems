#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;
Mat hsv1,thr1,thr2,mask;
int lr=150,lg=100,lb=100;
int hr=255,hg=200,hb=200;
double wcet=0;
enum transform_t {canny=0,hough,hough_eliptical};

Scalar hsv_min1,hsv_max1, hsv_min2,hsv_max2;

double prev_frame_time,curr_frame_time;
	struct timespec frame_time_start, frame_time_stop;
	double ave_framedt, ave_frame_rate, framedt;
    unsigned int frame_count;

int main( int argc, char** argv )
{
    //	cvNamedWindow("HSV", CV_WINDOW_AUTOSIZE);
    CvCapture* capture = cvCreateCameraCapture(1);
    IplImage* frame;
	vector<Vec3f> circles;
	int i=0;
	hsv_min1 = Scalar(35, 50, 100);
	hsv_max1 = Scalar(65, 210,255);
	hsv_min2 = Scalar(35, 50, 50);
	hsv_max2 = Scalar(65, 100, 100);	
	

    while(1)
    {
	
	
	clock_gettime(CLOCK_REALTIME, &frame_time_start);
                prev_frame_time =((double)frame_time_start.tv_sec * 1000.0) + 
                                ((double)((double)frame_time_start.tv_nsec / 1000000.0));
                frame_count ++;
       
        frame=cvQueryFrame(capture);
     
        if(!frame) break;
  	
        
    clock_gettime(CLOCK_REALTIME, &frame_time_stop);   
	
       

 	
	

         Mat mat_frame(frame);	
	cvtColor(mat_frame,hsv1,COLOR_BGR2HSV);
	
	
	inRange(hsv1,hsv_min1,hsv_max1,thr1);	
	inRange(hsv1,hsv_min2,hsv_max2,thr2);
	
	
	add(thr1,thr2,mask);
	 
   	GaussianBlur(mask,mask,Size(11,11),0,0);
	//imshow("Thr1",thr1);
	//imshow("Thr2",thr2);
		
	
	
	
	  
	imshow("Mask",mask);
	HoughCircles(mask, circles, CV_HOUGH_GRADIENT, 1, mask.rows/8, 15, 15, 15, 0);

	
	
	
	 for( size_t i = 0; i < circles.size(); i++ )
        {
          Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
          int radius = cvRound(circles[i][2]);
          // circle center
          circle( mat_frame, center, 3, Scalar(0,0,255), -1, 8, 0 );
        //  printf("\n C %lf %lf %lf %d",circles[0][0], circles[0][1],circles[0][2],i);
		   break;
          
        }
	
	
	curr_frame_time =((double)frame_time_stop.tv_sec * 1000.0) + 
                                ((double)((double)frame_time_stop.tv_nsec / 1000000.0));
        
	framedt =curr_frame_time  - prev_frame_time ;
	if(frame_count  > 1)
         {       
	ave_framedt = (ave_framedt *(frame_count -1) + framedt )/frame_count ;
	ave_frame_rate =1.0/(ave_framedt /1000.0);

	 }
	
	printf("\n\n Circle - Frame %d Time - %lf Avg Frame Time=%5.2lf msec, Ave Frame Rate - %5.2lf fps  WCET - %5.2lf \n", 
                 frame_count  ,framedt , ave_framedt ,ave_frame_rate,wcet );

        if(framedt >wcet && frame_count>50) wcet =framedt ;
	

	
	 imshow("Circles",mat_frame);

	char c = cvWaitKey(1);
        if( c == 27 ) break;
    }

    cvReleaseCapture(&capture);
   // cvDestroyWindow("Capture Example");
    
};
