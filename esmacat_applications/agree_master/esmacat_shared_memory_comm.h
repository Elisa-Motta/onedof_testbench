#ifndef ESMACAT_SHARED_MEMORY_COMM_H
#define ESMACAT_SHARED_MEMORY_COMM_H
#define DEFAULT_ROS_KEY_ID  202020

#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <iostream>
#include "agree_structs.h"

#define N_DOFS_MAX 5

class esmacat_shared_memory_comm
{
    struct shared_memory_packet {
        bool            stop = false;
        /** Elapsed time in ms */
        double          elapsed_time = 0;
        /** Loop counter */
        uint64_t        loop_cnt    = 0;
        uint64_t        status      = 1;
        uint64_t        command     = 1;
        joint_status_t                   joint_status[N_DOFS_MAX];
        joint_impedance_control_status_t joint_impedance_control_status[N_DOFS_MAX];
        joint_task_control_command_t     joint_task_control_parameters[N_DOFS_MAX];
        robot_configuration_t            robot_config;
    };

private:
    key_t key;
    bool is_the_shared_memory_detached= 0;
    int shmid = 0;
public:
//    static int number_of_process_attached_in_shared_memory;
    shared_memory_packet* data;
    esmacat_shared_memory_comm();
    ~esmacat_shared_memory_comm();
    void init();
    void change_shared_memory_key(key_t k){key= k;} // only use this function before init
    void detach_shared_memory();
};

#endif // ESMACAT_SHARED_MEMORY_COMM_H

