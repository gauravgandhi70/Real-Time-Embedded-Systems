#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/shm.h>
#include <semaphore.h>

int main()
{
int sh_mem_id;
key_t key = 1234;
size_t size = 100 * sizeof(int);
void *mem_addr;

sh_mem_id = shmget(key,size,0666);
mem_addr = shmat(sh_mem_id, NULL, 0);

sem_post(mem_addr);

printf("\n Semaphore posted\n");

}
