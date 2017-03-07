#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/shm.h>
#include<semaphore.h>

int main()
{
int sh_mem_id;
key_t key = 1234;
size_t size = 100 * sizeof(int);
void *mem_addr;
int i;

sh_mem_id = shmget(key,size,IPC_CREAT | 0666);
mem_addr = shmat(sh_mem_id, NULL, 0);

printf("\nWaiting for semaphore");


sem_init(mem_addr,1,0);


sem_wait(mem_addr);

printf("\nCompleted Process Successfully\n");
}
