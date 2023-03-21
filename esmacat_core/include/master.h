/** @file
 * @brief This file contains the declaration for the esmacat master
 */

#ifndef ESMACAT_MASTER_H
#define ESMACAT_MASTER_H

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include <inttypes.h>               // to uses more int type
#include "application.h"            // esmacat application
#include "slave.h"                  // ethercat slave
#include <pthread.h>                // multi thread
#include "esmacat_parameter.h"      // common parameters in esmacat
#include "esmacat_utility.h"        // common utility in esmacat
#include "shared_memory_ecat.h"     // for IPC with ethercat application
#include "algorithm"                // to use std::copy
#include "hardware_key_verification.h" // to verify if this hardware is certified

/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/
class esmacat_application;

/** @brief Holds all the functions and variables used by the ethercat master */
class esmacat_master
{
private:
    int cnt = 0;    // multipurpose integer counter;
    struct timespec t_now, t_init, t_period, t_1us;
    bool flag_esmacat_master_loop_thread_stop = false;
    /** @brief Pointer to the user-defined ethercat application to be used */
    esmacat_application* ECAT_app; // esmacat application will be copied to here and be used in threads
    /** @brief this is an intermediate function to use pthread */
    static void * InternalThreadEntry_esmacat_master_loop(void * This) {((esmacat_master *)This)->esmacat_master_loop(); return NULL;}
    /** @brief pthread for esmacat main loop*/
    pthread_t thread_esmacat_main;
    loop_time_stats* esmacat_loop_time_stats;

public:
    esmacat_slave* ECAT_slave[MAX_NUMBER_OF_ETHERCAT_SLAVE];    // array of esmacat slaves
    shared_memory_ecat comm;
    esmacat_master();
    ~esmacat_master();
    // start the threads for esmacat master, Returns true if the thread was successfully started, false if there was an error starting the thread
    bool StartInternalThread();
     // close the threads. Call this function when you finish the program.
    void WaitForInternalThreadToExit();
    void esmacat_master_loop();                         // main RT loop
    bool is_esmacat_master_loop_closed();
    void assign_esmacat_application(esmacat_application* esmacat_app){ECAT_app=esmacat_app;}
    void stop_thread();                                         // stop threads of esmacat master
    void set_one_cycle_loop_time_ns(unsigned long loop_time);
    unsigned long get_one_cycle_loop_time_ns();
};

#endif // ESMACAT_MASTER_H
