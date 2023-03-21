#ifndef AGREE_APP_H
#define AGREE_APP_H

/*****************************************************************************************
 * DEFINES
 ****************************************************************************************/

#define APPLICATION_TIMEOUT_MS 60*60*1000 // 60 Minutes

#define EXERCISE_START              M_PI/2.0
#define EXERCISE_DURATION           8000.0
#define EXERCISE_AMPLITUDE          - M_PI/2.0

#define USE_EASE    1
#define USE_LOGGER  1
#define PRINT_DEBUG 0

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include <iostream>

#include "ethercat_arduino_shield_by_esmacat.h"
#include "agree_ease.h"
#include "agree_joint_controller.h"

#include "onedof_shared_memory_comm.h"

#include "onedof_robot.h"
#include "onedof_common.h"

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
    onedof_shared_memory_comm                   onedof_shared_memory;

    agree_robot_class                           agree_robot;

    robot_control_mode_t                        control_mode;
    robot_control_mode_t                        prev_mode;
    robot_control_mode_t                        control_mode_status;


    // Support variables
    bool      impedance_control_first_run       = true;
    double    elapsed_time_ms_offset            = 0;
    double    elapsed_time_ms_offset_exercise   = 0;
    int       exercise_status                   = 0;

    double support_traj[N_DOFS_MAX] = {0,0,0,0,0};
    double support_torq[N_DOFS_MAX] = {0,0,0,0,0};
    double support_k[N_DOFS_MAX] = {0,0,0,0,0};
    double support_d[N_DOFS_MAX] = {0,0,0,0,0};
    int    support_repetitions = 0;

    double target_traj[N_DOFS_MAX] = {0,0,0,0,0};
    double target_torq[N_DOFS_MAX] = {0,0,0,0,0};
    double target_k[N_DOFS_MAX] = {0,0,0,0,0};
    double target_d[N_DOFS_MAX] = {0,0,0,0,0};

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
