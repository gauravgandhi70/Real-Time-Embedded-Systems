#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/shm.h>
#include<semaphore.h>
#include <stdlib.h>
int main()
{
printf("\nWaiting for semaphore");
int sh_mem_id;
key_t key = 1234;
size_t size = 100 * sizeof(int);
void *mem_addr;
int i;



sh_mem_id = shmget(key,size,IPC_CREAT | 0666);
mem_addr = shmat(sh_mem_id, NULL, 0);



sem_init(mem_addr,1,0);

system("ps");
sem_wait(mem_addr);

printf("\nProcess Completed Successfully\n");
}
