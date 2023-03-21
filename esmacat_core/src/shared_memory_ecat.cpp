#include "shared_memory_ecat.h"

shared_memory_ecat::shared_memory_ecat()
{
    key = ftok("/home",DEFAULT_KEY_ID);
}

shared_memory_ecat::~shared_memory_ecat(){

}

bool shared_memory_ecat::init(){
    // shmget returns an identifier in shmid
    shared_memory_packet temp;
    shmid = shmget(key, sizeof(temp),0666|IPC_CREAT);
    // shmat to attach to shared memory
    data = (shared_memory_packet*) shmat(shmid,(void*)0,0);
    if (shmid > 0 && key > 0) return true; // no error
    else return false; // error
}

void shared_memory_ecat::detach_shared_memory(){
    shmdt(data);
    shmctl(shmid,IPC_RMID,nullptr);
    printf("shared memory has been detached");
}
