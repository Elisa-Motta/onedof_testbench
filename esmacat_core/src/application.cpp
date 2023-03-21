/** @file
 * @brief This files contains the definitions for all the functions used by the
 * esmacat_application class
 */

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "application.h"

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/

/** @brief Constructor for the Esmacat application class which initializes all the variables
 *
 * Initializes a master. Sets the initial loop count to 0.
 * Initializes the Vendor ID and Product codes of the maximum possible slaves
 * to default
 */
esmacat_application::esmacat_application()
{
    ecat_master = new esmacat_master;
    loop_cnt = 0;
}

/** @brief Assigns the slave indices for each slave in the EtherCAT communication chain
 *
 * Also assigns their respective Vendor ID and Product code. Proivdes error handling
 * @param esmacat_slave* Pointer to the Esmacat slave application.
 * @param slave_index Position/index of the specified slave in the EtherCAT communication chain. Starts at 0.
 * @return Error status of the function. NO_ERR indicates successful completion.
 */

void esmacat_application::assign_esmacat_slave_index(esmacat_slave* slave_in_app, int slave_index)
{
    bool error_exit = false;
    ecat_master->ECAT_slave[slave_index] = slave_in_app;
    if ( ecat_master->comm.data->slave[slave_index].vendor_id != slave_in_app->get_esmacat_vendor_id()) {
        PLOGE << ERR_ESMACAT_SLAVE_VENDOR_ID_DOES_NOT_MATCH << " Vendor ID of " << slave_index << "th slave does not match";
        PLOGE.printf("Slave Vendor ID in software is %x and in hardware is %x", slave_in_app->get_esmacat_vendor_id(), ecat_master->comm.data->slave[slave_index].vendor_id);
        error_exit = true;
    }
    if ( ecat_master->comm.data->slave[slave_index].eep_id != slave_in_app->get_esmacat_product_id()) {
        PLOGE << ERR_ESMACAT_SLAVE_PRODUCT_ID_DOES_NOT_MATCH << " Product ID of " << slave_index << "th slave does not match";
        PLOGE.printf("Slave Product ID in software is %x and in hardware is %x", slave_in_app->get_esmacat_product_id(), ecat_master->comm.data->slave[slave_index].eep_id);
        error_exit = true;
    }
    if (error_exit == true)
    {
        stop();
    }
}

/** @brief Checks if the EtherCAT master connection is closed.
 *
 * @return True if closed, False if still open
 */
bool esmacat_application::is_esmacat_app_closed()
{
    return ecat_master->is_esmacat_master_loop_closed();
}

/** @brief Assigns this application as the application to be executed by the
 * EtherCAT master. Starts the thread on which this application is executed.
 */
void esmacat_application::start()
{
    ecat_master->assign_esmacat_application(this);
    ecat_master->StartInternalThread();
}

void esmacat_application::close()
{
    delete ecat_master;
}

/** @brief Terminates the thread of the application
 */
void esmacat_application::stop()
{
    ecat_master->stop_thread();
}

/** @brief Returns the esmacat_application loop counter that indicates how many
 * loops of the application have been run
 */
uint64_t esmacat_application::get_app_loop_cnt()
{
    return loop_cnt;
}

// increment the loop cnt
void esmacat_application::increment_app_loop_cnt()
{
    loop_cnt++;
}

bool esmacat_application::set_one_cycle_time_ns(unsigned long one_cycle_loop_tims_ns)
{
    if(one_cycle_loop_tims_ns > 1000000000L){
        PLOGE << "Cycle time needs to be smaller than 1sec";
        return true;
    }
    ecat_master->set_one_cycle_loop_time_ns(one_cycle_loop_tims_ns);
    return false;
}

unsigned long esmacat_application::get_one_cycle_time_ns()
{
    return  ecat_master->get_one_cycle_loop_time_ns();
}

/** @brief Virtual function to be overwritten by the user application my_app class*/
void esmacat_application::configure_slaves()
{
    // do something for initial setup
}

/** @brief Virtual function to be overwritten by the user application my_app class*/
void esmacat_application::set_elapsed_time_ms(double elapsed_time_ms_)
{
    elapsed_time_ms = elapsed_time_ms_;
}

/** @brief Virtual function to be overwritten by the user application my_app class*/
void esmacat_application::loop()
{
    // this is a base function, needs to be overwrode by a child class
}

/** @brief Virtual function to be overwritten by the user application my_app class*/
void esmacat_application::assign_slave_sequence()
{
    // this is a base function, needs to be overwrode by a child class
}

/** @brief Virtual function to be overwritten by the user application my_app class*/
void esmacat_application::init()
{
    // this is a base function, needs to be overwrode by a child class
}
