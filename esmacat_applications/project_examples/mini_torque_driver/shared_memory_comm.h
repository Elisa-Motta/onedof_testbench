/** @file
 * @brief Contains declarations used for Harmony SHR's shared memory
 *
*/

#ifndef ESMACAT_SHARED_MEMORY_COMM_H
#define ESMACAT_SHARED_MEMORY_COMM_H
#define UI_DEFAULT_KEY_ID  1300        // street number of Harmonic Bionics

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <iostream>
#include "joint_structs.h"


/*****************************************************************************************
 * STRUCTS
 ****************************************************************************************/
enum robot_control_mode_t{
    standby = 0,
    don_doff = 1,
    weight_support = 2,
    mirror_l2r = 3,
    mirror_r2l = 4,
    exercise = 5,
    freeze = 6,
    shutdown_sequence = 7

};

/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/
class esmacat_shared_memory_comm
{
    struct shared_memory_packet {
        sea_joint_vals_t R_J_vals[9];
        sea_torque_control_values_t R_T_ctrl_vals[9];
        sea_impedance_control_values_t R_I_ctrl_vals[9];

        sea_joint_vals_t L_J_vals[9];
        sea_torque_control_values_t L_T_ctrl_vals[9];
        sea_impedance_control_values_t L_I_ctrl_vals[9];

        double elapsed_time_ms = 0;
        int last_joint_index = 0;
        int err_msg_count = 0;
        bool is_single_joint = 0;

        bool stop = false;
        robot_control_mode_t mode = robot_control_mode_t::standby;
        int exercise_num = 0;
        int right_exercise_num = 0 ;
        int left_exercise_num = 0;
        uint64_t loop_cnt = 0;              // count of loop

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
    bool init();
    void change_shared_memory_key(key_t k){key= k;} // only use this function before init
    key_t get_shared_memory_key(){return key;}
    void detach_shared_memory();
};

#endif // ESMACAT_SHARED_MEMORY_COMM_H
