/** @file
 * @brief Contains definitions of functions used for the primary executable of Harmony SHR
 *
*/
/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "shared_memory_comm.h"


/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/
esmacat_shared_memory_comm::esmacat_shared_memory_comm()
{
    key = ftok("/bin",UI_DEFAULT_KEY_ID);
}

esmacat_shared_memory_comm::~esmacat_shared_memory_comm(){
    detach_shared_memory();
}

bool esmacat_shared_memory_comm::init(){
    // shmget returns an identifier in shmid
    shared_memory_packet temp;
    shmid = shmget(key, sizeof(temp),0666|IPC_CREAT);
    // shmat to attach to shared memory
    data = (shared_memory_packet*) shmat(shmid,(void*)0,0);
    data->mode = robot_control_mode_t::standby;
    if (shmid > 0 && key > 0) return true; // no error
    else return false; // error
}

void esmacat_shared_memory_comm::detach_shared_memory(){
    shmdt(data);
    shmctl(shmid,IPC_RMID,NULL);
}
