/** @file
 * @brief This file contains the declaration of the class associated with the user-defined
 * application for the Esmacat slave project */

#ifndef MY_APP_H
#define MY_APP_H

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include <iostream>
#include "application.h"

//Include the header file for the shared memory commnication
#include "esmacat_shared_memory_comm.h"

//Include the header file for the Esmacat slave you plan to use for e.g. Analog Input slave
#include "ethercat_arduino_shield_by_esmacat.h"
#include "esmacat_epos4_mod.h"

//Include the header file for the actuator controller
#include "agree_motor_controller.h"
#include "agree_robot.h"
#include "agree_common.h"
#include "agree_parameters.h"

using namespace std;

#define EXIT            0
#define STOP            1
#define CURRENT         2
#define TORQUE          3
#define NULLTORQUE      4
#define GRAVITY         5
#define FREEZE          6
#define FREEZE_SOFT     66
#define IMPEDANCE       7
#define HOMING          8
#define POSITION        9
#define WEIGHT          10
#define IMPEDANCE_EXT   11
#define TRIGGER         12
#define ADAPTIVE        13
#define PASSIVE         14
#define RESISTIVE       15
#define CHALLENGING     16

#define HOMING_DONE     108
#define POSITION_DONE   109

#define APPLICATION_TIMEOUT_MS 5*60*1000

//#define N_DOFS 2
/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/
/**
 * @brief Description of your custom application class
 *
 * Your custom application class agree_manager inherits the class 'esmacat_application'
 * Write functions to override the parent functions of 'esmacat_application'
 * Declare an object of your slave class (e.g. ecat_ai)
 * Declare any other variables you might want to add
 * Define the constructor for initialization
 */
class agree_manager : public esmacat_application
{
private:
    void assign_slave_sequence(); /** identify sequence of slaves and their types*/
    void configure_slaves(); /** setup the slave*/
    void init(); /** code to be executed in the first iteration of the loop */
    void loop(); /** control loop*/
    void quit(); /** exit */

    void initialize_write2file();
    void write2file();
    void write2sharedmemory();
    void readsharedmemory();
    void print2screen();

    ofstream CSVfile;

    esmacat_ethercat_arduino_shield_by_esmacat  ecat_ease;

    agree_motor_controller                      agree_joints[N_DOFS_MAX];

    agree_robot_class                           agree_robot;
    agree_parameters_class                      agree_parameters;

    esmacat_shared_memory_comm                  agree_shared_memory;

    esmacat_err                                 agree_error;

    uint64_t state;
    uint64_t prev_state;
    float    elapsed_time_ms_offset;
    float    encoder_position_offset;

public:
    /** A constructor- sets initial values for class members */
    agree_manager();
    /** A constructor- sets initial values for class members */
    virtual ~agree_manager();

};

#endif // MY_APP_H
