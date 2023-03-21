/** @file
 * @brief This file defines all the functions used by the Esmacat master class which works
 * for any ethercat master
 */

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "master.h"

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/

/** @brief Constructor for the esmacat_master class
 *
 * Resets the values of currentgroup, shows the master connection to
 * be active, and the threads running
 */
esmacat_master::esmacat_master()
{
    esmacat_loop_time_stats = new loop_time_stats("esmacat_app_loop_time_stats.txt",loop_time_stats::output_mode::fileout_only); // create an object for loop time stats
    PLOGI << "esmacat master has been created.";
    set_one_cycle_loop_time_ns(get_esmacat_default_loop_time_ns());
    t_1us.tv_sec = 0; t_1us.tv_nsec=1000;       // define  1us
    if ( comm.init() == false ) // failed for IPC init
    {
        PLOGE << "Ehercat-Esmacat shared memory init has been failed";
        stop_thread();
    }   // successful for IPC init
    else
    {
        PLOGI << "Ehercat-Esmacat shared memory initialized with key " << static_cast<int>(comm.get_shared_memory_key());    // start the shared memory communication
    }
}


/** @brief Destructor for the esmacat_master class
 *
 * Ends the threads on which the applications are running
 */
esmacat_master::~esmacat_master()
{
    esmacat_loop_time_stats->store_loop_time_stats();    // fileout the loop time stats
    delete esmacat_loop_time_stats;
    stop_thread();
    WaitForInternalThreadToExit();  // join the threads.
}

/** @brief Stops the thread on which the master application is running
 *
 * Also sets the stop_thread_loop flag to 1 which flags for the thread to
 * be terminated
 */
void esmacat_master::stop_thread()
{
    comm.data->esmacat_main_current_sequence = esmacat_flow_sequence::esmacat_terminated;
    flag_esmacat_master_loop_thread_stop = true; // flag on for termination of the thread
}

void esmacat_master::set_one_cycle_loop_time_ns(unsigned long loop_time_ns)
{
    t_period.tv_sec = 0;      // zeroing of the loop time
    t_period.tv_nsec = 0;     // zeroing of the loop time
    while( loop_time_ns > 1000000000L) {   // if loop time is larger than 1 sec, split sec and nsec accordingly
        t_period.tv_sec++;
        loop_time_ns -= 1000000000L;
    }
    t_period.tv_nsec = loop_time_ns;
}

unsigned long esmacat_master::get_one_cycle_loop_time_ns()
{
    return t_period.tv_sec * 1000000000L + t_period.tv_nsec;
}

/** @brief Starts the threads that will be running in parallel
 *
 * One of the threads checks the status of the EtherCAT to ensure that all slaves are
 * operational. The other threads runs the master loop that ensures constant communication
 * and real-time exchange of data with the slaves
 *
 * @return Returns 0 if successful, and 1 if there is an error
 */
bool esmacat_master::StartInternalThread()
{
    int policy; // thread policy
    struct sched_param prio;    // thread priority
    pthread_attr_t attr;        // attributes of thread
    pthread_attr_init( &attr);  // init the thread attributes
    pthread_attr_setinheritsched( &attr, PTHREAD_EXPLICIT_SCHED);   // get the default attribu from PTHREAD_EXPLICIT_SCHED
    policy = SCHED_RR;  // set the thread policy to Real-time Robin (RR)
    pthread_attr_setschedpolicy( &attr, policy);    // schedule policy
    prio.sched_priority = 1;    // set to the highest. In reality, the priority is not important for SCHED_RR
    pthread_attr_setschedparam(&attr,&prio);    // set the priority
    PLOGI << "Esmacat Master Thread is creating!";  // Inform the thread is starting
    return pthread_create(&thread_esmacat_main, &attr, InternalThreadEntry_esmacat_master_loop, this); // create the thread
}

/** @brief This function supports the exchange of real-time data between
 *  the master and slaves
 */
void esmacat_master::esmacat_master_loop()
{
    //-- 1. initialization
    uint8_t shadow_input_data_stream[MAX_ECAT_INPUT_DATA_SIZE_BYTE];    // shadow variable to avoid data access problems
    uint8_t shadow_output_data_stream[MAX_ECAT_OUTPUT_DATA_SIZE_BYTE];  // shadow variable to avoid data access problems
    for(int i=0;i<MAX_ECAT_INPUT_DATA_SIZE_BYTE;i++) shadow_input_data_stream[i] = 0;   // inits of the variables
    for(int i=0;i<MAX_ECAT_OUTPUT_DATA_SIZE_BYTE;i++) shadow_output_data_stream[i] = 0; // inits of the variables
    int sum_of_system_parameter_buffer = 0;     // variable to count the sum of system param buffer
    uint64_t prev_ecat_loop_cnt = UINT64_MAX;
    hardware_key_verification hardware_verification;
    int app_init_flag = 0;
    if ( hardware_verification.is_verified_hardware() == false ) {
        PLOGE << "This hardware is not verified. Contact info@esmacat.com";
        stop_thread();
        return;
    }

    //-- 2. Ethercat status check. Confirm ecat_main works correctly.
    comm.data->esmacat_main_current_sequence = esmacat_flow_sequence::esmacat_app_on;   // change the status of esmacat
    PLOGI << "esmacat application is on!";  // log the esmacat app is now on
    if (flow_control(comm.data->ecat_main_current_sequence,ecat_flow_sequence::ecat_app_on,TIMEOUT_WAITING_US_APP_ON,"Timeout for waiting ecat_application_on")) {
        stop_thread();
        return;
    }
    PLOGI << "ecat application is on!";     // log the ecat app is now on
    if (flow_control(comm.data->ecat_main_current_sequence,ecat_flow_sequence::ecat_setup_done,TIMEOUT_WAITING_US_ECAT_SETUP_DONE,"Timeout for waiting ecat_setup_done")) {
        stop_thread();
        return;
    }
    PLOGI << "ethercat setup is done in ecat_main!"; // log ecat app finished ethercat setup

    //-- 3. Configure all EtherCAT slaves and connect them with the Esmacat Slaves
    for(int i=0;i<comm.data->number_of_ecat_slaves;i++){    // log the slave info
        PLOGI.printf("Name of the %dth slave: %s",i,comm.data->slave[i].name);
        PLOGI.printf("Product ID of the %dth slave: (unit:hex) %8.8x",i, comm.data->slave[i].eep_id);
        PLOGI.printf("Vendor ID of the %dth slave: (unit:hex) %8.8x",i, comm.data->slave[i].vendor_id);
        PLOGI.printf("Input/Output Byte Size of the %dth slave: %d, %d",i, comm.data->slave[i].input_byte_size,comm.data->slave[i].output_byte_size);
    }

    ECAT_app->assign_slave_sequence();  // get the slave pointer in application and assign the pointer to the pointer array in master. Also confirm the slave info defined in app is matched with the actual slave info.
    if(flag_esmacat_master_loop_thread_stop == true)    // if ECAT_app->assign_slave_sequence() turned on the stop flag, then stop the thread.
    {
        PLOGE << "Assigning EtherCAT slaves with Esmacat slaves has been failed";
        return;
    }
    int size_of_ecat_input_data_stream = comm.data->slave[comm.data->number_of_ecat_slaves-1].input_data_starting_index + comm.data->slave[ comm.data->number_of_ecat_slaves-1 ].input_byte_size;
    int size_of_ecat_output_data_stream = comm.data->slave[comm.data->number_of_ecat_slaves-1].output_data_starting_index + comm.data->slave[ comm.data->number_of_ecat_slaves-1 ].output_byte_size;
    PLOGI.printf("size of ecat_input_data_stream: %d, size of ecat_output_data_stream: %d", size_of_ecat_input_data_stream, size_of_ecat_output_data_stream);
    comm.data->esmacat_main_current_sequence = esmacat_flow_sequence::esmacat_setup_done;
    PLOGI << "ecat setup is done in esmacat_main!";

    if (flow_control(comm.data->ecat_main_current_sequence,ecat_flow_sequence::all_states_OP_arrived,TIMEOUT_WAITING_US_ECAT_SETUP_DONE,"Timeout for waiting ecat_setup_done")) {
        stop_thread();
        return;
    }
    PLOGI << "Status of all ethercat slaves arrived OP!"; // log ecat arrrived all status OP

    //-- 3. Configure all the EtherCAT slaves.
        ECAT_app->configure_slaves(); // send the intialization information contained in the application

    //-- 4. Set the one cycle period for ecat_main and standard time for esmacat_application.
    comm.data->one_cycle_time_ns = t_period.tv_sec * 1000000000L + t_period.tv_nsec;
    comm.data->esmacat_main_current_sequence = esmacat_flow_sequence::one_cycle_period_set;
    clock_gettime ( CLOCK_MONOTONIC, &t_init);

    //-- 5. Main loop of esmacat_application
    while(flag_esmacat_master_loop_thread_stop == false)
    {
        esmacat_loop_time_stats->loop_starting_point();      // measure the current loop time to obtain the stats of loop time

        //-- 5.1. Stop if ecat_main does not run
        if ( comm.data->ecat_main_current_sequence == ecat_flow_sequence::ecat_terminated)
        {
            PLOGI << "esmacat is being terminated since ecat_main has been terminated.";
            stop_thread();
            return;
        }

        //-- 5.2. Wait until ecat_main arrives "wait_next_cycle" state in the next cycle.
        int wait_us_for_next_ecat_loop = TIMEOUT_WAITING_NEXT_CYCLE_ECAT_MAIN_US;
        while (comm.data->ecat_loop_cnt == prev_ecat_loop_cnt || comm.data->ecat_main_current_sequence != ecat_flow_sequence::wait_next_cycle ){
            clock_nanosleep ( CLOCK_MONOTONIC, 0, &t_1us, NULL );   // wait 1us for second app till it set up the t_ref;
            if (wait_us_for_next_ecat_loop-- == 0) {
                if ( comm.data->ecat_main_current_sequence == ecat_flow_sequence::ecat_terminated)
                {
                    PLOGI << "esmacat is being terminated since ecat_main has been terminated!";
                    stop_thread();
                    return;
                }
                else
                {
                    PLOGE << "Timeout for waiting the next cycle of ECAT";
                }
                stop_thread();
                return;
            }
        }
        prev_ecat_loop_cnt = comm.data->ecat_loop_cnt;

        //-- 5.3. Copy the EtherCAT packet to shadow variables.
        std::copy( comm.data->ecat_input_data_stream, comm.data->ecat_input_data_stream + size_of_ecat_input_data_stream, shadow_input_data_stream );   // copy the input data stream to the shoadow variable
        std::copy( shadow_output_data_stream, shadow_output_data_stream + size_of_ecat_output_data_stream,  comm.data->ecat_output_data_stream );       // copy the shadow data to the output data stream
        comm.data->esmacat_main_current_sequence = esmacat_flow_sequence::shadow_copy_done_cycle_running;   // change the current status
        PLOGD << "Shadow copy is done and cycle running!";  // if verbose mode, log the current status

        //-- 5.4. Get the loop counter and elapsed time
        comm.data->esmacat_loop_cnt++;  // count++ since the loop just started
        if(comm.data->esmacat_loop_cnt%1000==1) {
            PLOGI.printf("esmacat_main_loop: %d",comm.data->esmacat_loop_cnt);
        }
        clock_gettime ( CLOCK_MONOTONIC, &t_now);
        ECAT_app->set_elapsed_time_ms( timespec_sub_to_msec(&t_now,&t_init) );
        ECAT_app->increment_app_loop_cnt();

        //-- 5.5. Ecat data process
        // for each slave, exchange the data enqueued
        sum_of_system_parameter_buffer = 0;
        for(cnt = 0; cnt < comm.data->number_of_ecat_slaves; cnt++)
        {
            if ((static_cast<int>(comm.data->slave[cnt].output_byte_size) != 0) && (static_cast<int>(comm.data->slave[cnt].input_byte_size) != 0))
            {
                ECAT_slave[cnt]->ecat_data_process( shadow_output_data_stream + comm.data->slave[cnt].output_data_starting_index, comm.data->slave[cnt].output_byte_size, shadow_input_data_stream + comm.data->slave[cnt].input_data_starting_index, comm.data->slave[cnt].input_byte_size );
                sum_of_system_parameter_buffer +=  ECAT_slave[cnt]->flush_one_set_of_system_parameters();
            }
        }
        //data exchange for the cycle has been completed
        if (sum_of_system_parameter_buffer == 0)
        {
            if (app_init_flag== 0)
            {
                ECAT_app->init();
                app_init_flag = 1;
            }
            else
            {
                ECAT_app->loop();
            }
        }
        comm.data->esmacat_main_current_sequence = esmacat_flow_sequence::wait_next_cycle;  // finished the one loop cycle, now wait until the start of the next cycle
    }
    PLOGI << "esmacat application is closing";
    stop_thread();
    return;  // stop theread
}

/** @brief Returns the status of the ethercat master connection
 * @return Boolean value indicating the status of the ethercat master connection
*/
bool esmacat_master::is_esmacat_master_loop_closed()
{
    return flag_esmacat_master_loop_thread_stop;
}

/** @brief Ensures that the thread has been closed successfully */
void esmacat_master::WaitForInternalThreadToExit()
{
    (void) pthread_join(thread_esmacat_main, NULL);
}
