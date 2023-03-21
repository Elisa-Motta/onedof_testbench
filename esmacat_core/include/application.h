/** @file
 * @brief This file contains the declarations and enumerations for the generic/parent Esmacat
 * application
 */

#ifndef ESMACAT_APPLICATION_H
#define ESMACAT_APPLICATION_H

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
//#include <vector>
#include <typeinfo>
#include "master.h"
#include "slave.h"
#include "error_list.h"
#include "esmacat_parameter.h"
#include "esmacat_utility.h"
/*****************************************************************************************
 * MACROS
 ****************************************************************************************/

/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/
class esmacat_master;   // forward declared dependencies. needed for co-dependency
/** @brief Class that holds generic Esmacat application variables and functions
 *
 * It is the parent class for all user-defined Esmacat applications (my_app)
*/
class esmacat_application
{
private:
    /** @brief Pointer to the EtherCAT master in which the EtherCAT threads are running at the master app.
     */
    esmacat_master* ecat_master;
protected:
    /** @brief Protected variable - amount of time in ms for which the esmacat_application has been running */
    double elapsed_time_ms;

    /** @brief Protected variable - Counts the number of loops of this application have been executed */
    uint64_t loop_cnt;

    void assign_esmacat_slave_index(esmacat_slave* slave_in_app, int slave_index);
public:
    /** @brief This value is True if the EtherCAT connection to the Master is closed; False otherwise */
    bool is_esmacat_app_closed();
    esmacat_application();                  // constructor
    void stop();                            // stop thread
    void start();                           // start and application
    void close();
    void set_elapsed_time_ms(double elapsed_time_ms_);
    uint64_t get_app_loop_cnt();    //get the value of the loop cnt
    void increment_app_loop_cnt();  // increment the loop cnt
    virtual void assign_slave_sequence() = 0;             // to be over-ridden by child class
    virtual void configure_slaves() = 0;    // setup function, this needs to be over-ridden by a child class
    virtual void init() = 0;                // initialization function to be over-rideen by child class
    virtual void loop() = 0;                // loop function, to be over-ridden by child class
    bool set_one_cycle_time_ns(unsigned long one_cycle_loop_tims_ns); // set the one cycle loop time in ns. If error, return 1
    unsigned long get_one_cycle_time_ns();
};

#endif // ESMACAT_APPLICATION_H
