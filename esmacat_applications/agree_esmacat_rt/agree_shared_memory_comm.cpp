/** @file
 * @brief Contains definitions of functions used for the primary executable of Harmony SHR
 *
*/
/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "agree_shared_memory_comm.h"


/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/
agree_shared_memory_comm::agree_shared_memory_comm()
{
    key = ftok("/bin",DEFAULT_ROS_KEY_ID);
}

agree_shared_memory_comm::~agree_shared_memory_comm(){
    detach_shared_memory();
}

bool agree_shared_memory_comm::init(){
    // shmget returns an identifier in shmid
    shared_memory_packet temp;
    shmid = shmget(key, sizeof(temp),0666|IPC_CREAT);
    // shmat to attach to shared memory
    data = (shared_memory_packet*) shmat(shmid,(void*)0,0);
    //data->mode = control_mode_t::standby;
    data->stop = false;
    data->use_ros = false;
    if (shmid > 0 && key > 0) return true; // no error
    else return false; // error
}

void agree_shared_memory_comm::detach_shared_memory(){
    shmdt(data);
    shmctl(shmid,IPC_RMID,NULL);
}

