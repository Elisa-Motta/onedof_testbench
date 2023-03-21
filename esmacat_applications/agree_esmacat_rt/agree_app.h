#ifndef AGREE_APP_H
#define AGREE_APP_H

/*****************************************************************************************
 * DEFINES
 ****************************************************************************************/

#define APPLICATION_TIMEOUT_MS 60*60*1000 // 15 Minutes

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include <iostream>

#include "ethercat_arduino_shield_by_esmacat.h"
#include "agree_ease.h"
#include "agree_joint_controller.h"

#include "agree_shared_memory_comm.h"

#include "agree_robot.h"
#include "agree_common.h"
#include "agree_parameters.h"

using namespace std;

/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/
class onedof_manager : public esmacat_application
{
private:
    void assign_slave_sequence();
    void configure_slaves();
    void loop();
    void init();
    void quit();
    void write2sharedMemory();
    void readsharedMemory();

    agree_joint_controller                      agree_joints[N_DOFS_MAX];
    esmacat_err                                 agree_joint_error[N_DOFS_MAX];
    esmacat_err                                 agree_control_error[N_DOFS_MAX];

    agree_ease                                  agree_arduino;
    agree_shared_memory_comm                    agree_shared_memory;

    agree_robot_class                           agree_robot;
    agree_parameters_class                      agree_parameters;

    robot_control_mode_t control_mode;
    robot_control_mode_t prev_mode;
    robot_control_mode_t control_mode_status;

    double support_traj[N_DOFS_MAX] = {0,0,0,0,0};
    double support_torq[N_DOFS_MAX] = {0,0,0,0,0};
    double support_k[N_DOFS_MAX] = {0,0,0,0,0};
    double support_d[N_DOFS_MAX] = {0,0,0,0,0};

    // Support variables
    bool      impedance_control_first_run       = true;
    double    elapsed_time_ms_offset            = 0;
    double    elapsed_time_ms_offset_exercise   = 0;
    int       exercise_status                   = 0;

    ofstream CSVfile;

    void print_sequential_feedback();
    void print_feedback();
    void openfile();
    void writefile();
    void closefile();

public:
    onedof_manager();
    virtual ~onedof_manager();
};


#endif // AGREE_APP_H
