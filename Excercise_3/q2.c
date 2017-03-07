#include<stdio.h>
#include<pthread.h>
#include<time.h>

pthread_mutex_t mutx = PTHREAD_MUTEX_INITIALIZER; // Create New mutex and initialize it

// Data strucutre for xyz parameters of acc, roll, yaw, pitch
struct attitude{
		double x;
		double y;
		double z;
}acc,roll,pitch,yaw; 

// Structre to get  real time
struct timespec sample_time;

// Thread - 1 for updating the global structure attitude
void *update()
{
   pthread_mutex_lock(&mutx);        //Lock mutex
		// Update data with random numbers	
    acc.x=1.1; acc.y=2.2 ;acc.z=3.3;
    roll.x=4.4;roll.y=5.5;roll.z=6.6;
    pitch.x = 7.7; pitch.y=8.8 ; pitch.z=9.9;
    yaw.x=10.10 ; yaw.y=11.11 ; yaw.z=12.12;
    clock_gettime(CLOCK_REALTIME,&sample_time);     //Get time when the sample was updated
    pthread_mutex_unlock(&mutx);        //unlock mutex
    pthread_exit(0);				// Exit the thread

}

// Thread - 2 for reading the global structure attitude

void* read_time()
{
	pthread_mutex_lock(&mutx);    //Lock mutex
	// read and print data from the structure attitude
	printf("\n\t Read Acceleration: %fx,%fy, %fz",acc.x,acc.y,acc.z);
	printf("\n\t Read Roll: %fx,%fy, %fz",roll.x,roll.y,roll.z);
	printf("\n\t Read Pitch: %fx,%fy, %fz",pitch.x,pitch.y,pitch.z);
	printf("\n\t Read Yaw: %fx,%fy, %fz",yaw.x,yaw.y,yaw.z);
	printf("\n\tTime: %ld:%ld \n",sample_time.tv_sec,sample_time.tv_nsec); //Print time when the sample was updated
 	pthread_mutex_unlock(&mutx);        //unlock mutex
	pthread_exit(NULL);					// Exit the thread

}

int main()
{
	pthread_t id1,id2;
	
	// Creating 2 pthreads	
	if(pthread_create(&id1,NULL,update ,NULL))
		{
			printf("thread 1 not created ERROR");
		}
	else{printf("\n Thread 1 Created");}

	
	if(pthread_create(&id2,NULL,read_time, NULL))
		{
			printf("Pthread 2 not created ERROR");
		}
	else{printf("\n Thread 2 Created");}
	
	// Joining 2 pthreads	
	pthread_join(id1,NULL);
	pthread_join(id2,NULL);
	
	return 1;
}
