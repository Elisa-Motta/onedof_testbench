#ifndef ONEDOF_APP_H
#define ONEDOF_APP_H

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include <iostream>
#include "joint_controller.h"
#include "ethercat_arduino_shield_by_esmacat.h"
#include "shared_memory_comm.h"

using namespace std;

/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/
class onedof_app : public esmacat_application
{
private:
    void assign_slave_sequence();
    void configure_slaves();
    void loop();
    void init();
    void write2sharedMemory();

    joint_controller actuator;
    joint_controller actuator2;
    joint_controller actuator3;
    esmacat_ethercat_arduino_shield_by_esmacat ease0;

    esmacat_shared_memory_comm comm;

    void print_feedback();

public:
    onedof_app();
    virtual ~onedof_app();
};


#endif // ONEDOF_APP_H
