/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/

#include <stdio.h>              // printf
#include <string.h>
#include <time.h>               // cyclic loop
#include <pthread.h>            // multithread
#include "shared_memory_ecat.h" // IPC btw ecat_main and esmacat_main
#include "esmacat_utility.h"    // common functions and definitions.
#include "esmacat_parameter.h"  // all parameters of esmacat
#include "ethercat.h"           // SOEM
#include <algorithm>            // std::copy
#include <signal.h>             // to catch ctrl-c

#define EC_TIMEOUTMON 500

/** @brief Holds the PDO map for CANOpen for all the slaves*/
char IOmap[4096];               // IO map for EtherCAT, used by SOEM
/** @brief Expected value of the working counter (part of EtherCAT protocol)- used by SOEM */
int expectedWKC;
boolean needlf;
/** @brief Value of the work counter - used by SOEM */
volatile int wkc;
/** @brief Indicates that the master is operational and communicating with the slaves - used by SOEM*/
boolean inOP;
/** @brief Group of slaves - used by SOEM */
uint8 currentgroup = 0;
/** @brief flag to terminate ecat_chk */
boolean flag_terminate_app = false;
/** @brief flag to informing if slave is lost*/
boolean flag_slave_lost= false;

void *ecat_main_loop (void *)
{
    loop_time_stats ecat_loop_time_stats("ecat_main_loop_time_stats.txt",loop_time_stats::output_mode::screenout_only);
    shared_memory_ecat comm;    // for IPC btw ecat_main and esmacat_main
    if ( comm.init() == false ) // failed for IPC init
    {
        PLOGE << "Ehercat-Esmacat shared memory init has been failed";
        flag_terminate_app = true;
        comm.data->ecat_main_current_sequence = ecat_flow_sequence::ecat_terminated;
        return nullptr;
    }   // successful for IPC init
    else
    {
        PLOGI << "Ehercat-Esmacat shared memory initialized with key " << static_cast<int>(comm.get_shared_memory_key());    // start the shared memory communication
    }

    struct timespec t_next, t_now;  // declare timespec structs
    struct timespec t_period; t_period.tv_sec = 0; t_period.tv_nsec = 0;   // time variable for one cycle
    comm.data->one_cycle_time_ns = get_esmacat_default_loop_time_ns();
    t_period.tv_nsec = comm.data->one_cycle_time_ns;
    unsigned long int t_overflow = 0;   // measure the overflowed time for each cycle
    int cnt_t_overflow = 0; // counter to count the number of cycle time overflow
    int i, chk, cnt;    // SOEM
    inOP = FALSE;       // SOEM
    uint64_t prev_esmacat_loop_cnt = UINT64_MAX;

    comm.data->ecat_main_current_sequence = ecat_flow_sequence::ecat_app_on ; // let the second app go
    PLOGI << "ecat application is on";

    /* initialise SOEM, bind socket to ifname */
    char ifname[] = ETHERCAT_ADAPTER_NAME;
    if (ec_init(ifname))
    {
        PLOGI.printf("ec_init on %s succeeded.",ifname);
        /* find and auto-config slaves */
        if ( ec_config_init(FALSE) > 0 )
        {
            PLOGI.printf("%d slaves found and configured.",ec_slavecount);
            ec_config_map(&IOmap);
            ec_configdc();

            PLOGI.printf("Slaves mapped, state to SAFE_OP.");
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

            // read slave information and, recognize the slave
            comm.data->number_of_ecat_slaves = ec_slavecount;
            for(cnt = 1; cnt <= ec_slavecount ; cnt++)
            {
                PLOGI.printf("Slave Index: %d", cnt-1);
                PLOGI.printf("Manufacturer code: %8.8x Name: %s Product ID: %8.8x,  Rev: %8.8x iByte: %d oByte: %d", ec_slave[cnt].eep_man, ec_slave[cnt].name, ec_slave[cnt].eep_id, ec_slave[cnt].eep_rev, ec_slave[cnt].Ibytes,ec_slave[cnt].Obytes);
                comm.data->slave[cnt-1].vendor_id = ec_slave[cnt].eep_man;
                strcpy(comm.data->slave[cnt-1].name,ec_slave[cnt].name);
                comm.data->slave[cnt-1].eep_id = ec_slave[cnt].eep_id;
                comm.data->slave[cnt-1].eep_rev = ec_slave[cnt].eep_rev;
                comm.data->slave[cnt-1].input_byte_size = ec_slave[cnt].Ibytes;
                comm.data->slave[cnt-1].output_byte_size = ec_slave[cnt].Obytes;
            }
            for(cnt = 1; cnt < ec_slavecount; cnt++)
            {
                comm.data->slave[cnt].input_data_starting_index = comm.data->slave[cnt-1].input_data_starting_index + comm.data->slave[cnt-1].input_byte_size;
                comm.data->slave[cnt].output_data_starting_index = comm.data->slave[cnt-1].output_data_starting_index + comm.data->slave[cnt-1].output_byte_size;
            }

            PLOGI.printf("segments : %d : %d %d %d %d",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

            PLOGI.printf("Request operational state for all slaves");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            PLOGI.printf("Calculated workcounter %d", expectedWKC);
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);
            chk = 200;
            /* wait for all slaves to reach OP state */

            comm.data->ecat_main_current_sequence = ecat_flow_sequence::ecat_setup_done ;
            PLOGI.printf("ecat setup is done in ecat_main");

            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            }
            while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            if (ec_slave[0].state == EC_STATE_OPERATIONAL )
            {
                comm.data->ecat_main_current_sequence = ecat_flow_sequence::all_states_OP_arrived;
                PLOGI.printf("Operational state reached for all slaves.");
                inOP = TRUE;

                clock_gettime ( CLOCK_MONOTONIC, &t_now);
                comm.data->t_ref = t_now;
                t_next = t_now;

                /* cyclic loop */
                while( flag_terminate_app == false)
                {
                    // periodic loop
                    ecat_loop_time_stats.loop_starting_point();


                    comm.data->ecat_main_current_sequence = ecat_flow_sequence::wait_next_cycle;

                    // //if an ethercat slave is lost, terminate the app
                    if ( flag_slave_lost == true && FLAG_TERMINATE_ECAT_IF_SLAVE_LOST){
                        PLOGE << "EtherCAT slave #" << cnt-1 << " has been lost";
                        flag_terminate_app = true;
                        comm.data->ecat_main_current_sequence = ecat_flow_sequence::ecat_terminated;
                        break;
                    }

                    // //if esmacat app stops,
//                    if ( comm.data->esmacat_loop_cnt > 0 && comm.data->esmacat_loop_cnt == prev_esmacat_loop_cnt && FLAG_TERMINATE_ECAT_WHEN_ESMACAT_APP_STOP ){ // BUG: Changed to try avoid disconnections!
                    if ( comm.data->esmacat_loop_cnt > 0 && comm.data->esmacat_main_current_sequence == esmacat_flow_sequence::esmacat_terminated && FLAG_TERMINATE_ECAT_WHEN_ESMACAT_APP_STOP){
                        PLOGI << "Esmacat app has been stoped. Ecat main is being terminated as well.";
                        flag_terminate_app = true;
                        comm.data->ecat_main_current_sequence = ecat_flow_sequence::ecat_terminated;
                        break;
                    }
                    prev_esmacat_loop_cnt = comm.data->esmacat_loop_cnt;

                    comm.data->ecat_main_current_sequence = ecat_flow_sequence::cycle_running;
                    comm.data->ecat_loop_cnt++;
                    if(comm.data->ecat_loop_cnt%1000==1) {
                        PLOGI.printf("ecat_main_loop: %d",comm.data->ecat_loop_cnt);
                    }
                    ec_send_processdata();
                    wkc = ec_receive_processdata(EC_TIMEOUTRET);

                    if(wkc >= expectedWKC)
                    {
                        //                        printf("Processdata cycle %4d, WKC %d , O:", i, wkc);
                        for(cnt = 1; cnt <= ec_slavecount  ; cnt++)
                        {
                            if ((static_cast<int>(comm.data->slave[cnt-1].output_byte_size) != 0) && (static_cast<int>(comm.data->slave[cnt-1].input_byte_size) != 0))
                            {
                                std::copy( comm.data->ecat_output_data_stream + comm.data->slave[cnt-1].output_data_starting_index, comm.data->ecat_output_data_stream + comm.data->slave[cnt-1].output_data_starting_index + comm.data->slave[cnt-1].output_byte_size, ec_slave[cnt].outputs );
                                std::copy( ec_slave[cnt].inputs, ec_slave[cnt].inputs + ec_slave[cnt].Ibytes, comm.data->ecat_input_data_stream + comm.data->slave[cnt-1].input_data_starting_index );
                            }
                        }
                    }
                    comm.data->ecat_main_current_sequence = ecat_flow_sequence::wait_next_cycle;

                    t_period.tv_sec = 0;
                    t_period.tv_nsec = comm.data->one_cycle_time_ns;
                    TIMESPEC_INCREMENT ( t_next, t_period );
                    clock_nanosleep ( CLOCK_MONOTONIC, TIMER_ABSTIME, &t_next, nullptr );
                    clock_gettime ( CLOCK_MONOTONIC, &t_now);
                    t_overflow = (t_now.tv_sec*1e9 + t_now.tv_nsec) - (t_next.tv_sec*1e9 + t_next.tv_nsec);
                    if(t_overflow > ALLOWED_LOOPTIME_OVERFLOW_NS)
                    {
                        PLOGW.printf("Cycle time exceeded the desired cycle time. Overflowed time: %ld ns",t_overflow);
                        cnt_t_overflow ++;
                    }
                    else
                    {
                        cnt_t_overflow = 0;
                    }
                    if (cnt_t_overflow > MAX_NUMBER_OF_SEQUENTIAL_LOOPTIME_OVERFLOW )
                    {
                        PLOGE.printf("Cycle time exceeded the desired cycle time. Overflowed time: %ld ns",t_overflow);
                        flag_terminate_app = true;
                        comm.data->ecat_main_current_sequence = ecat_flow_sequence::ecat_terminated;
                        break;
                    }
                }
                comm.data->ecat_main_current_sequence = ecat_flow_sequence::ecat_terminated;
                inOP = FALSE;
            }
            else
            {
                PLOGW.printf("Not all slaves reached operational state.");
                ec_readstate();
                for(i = 1; i<=ec_slavecount ; i++)
                {
                    if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        PLOGI.printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s",
                                     i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
            PLOGI.printf("Request init state for all slaves");
            ec_slave[0].state = EC_STATE_INIT;
            /* request INIT state for all slaves */
            ec_writestate(0);
        }
        else
        {
            PLOGW.printf("No slaves found!");
        }
        PLOGI.printf("ecat main loop, close socket");
        /* stop SOEM, close socket */
        flag_terminate_app = 1;
        ec_close();

    }
    else
    {
        PLOGE.printf("No socket connection on %s Excecute as root",ifname);
    }

    ecat_loop_time_stats.store_loop_time_stats();
    comm.detach_shared_memory();
    PLOGI.printf("ecat app has been closed, ecat socket has been closed, shared memory at ecat_app has been detached.  ");
    return NULL;
}

void *ecat_chk (void *)
{
    int slave;
    while(1)
    {
        // if either all slaves are operational and the work counter is less that its expected value
        // or if one of the slaves in the group needs to be checked
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            //read the state of the slave
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                //if the slave in consideration belongs to the current group and is not Operational
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    //flag that the state of the slave under consideration has to be checked
                    ec_group[currentgroup].docheckstate = TRUE;

                    //if the slave is in SAFE_OP and has had an error
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        PLOGW.printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    //if the slave is in the SAFE_OP state
                    else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        //set it to the OP state inform the user
                        PLOGW.printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    //if slave is in any valid state
                    else if(ec_slave[slave].state > EC_STATE_NONE)
                    {
                        //reconfigure it and indicate that the connection has been regained
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            PLOGI.printf("MESSAGE : slave %d reconfigured",slave);
                        }
                    }
                    //if slave is responding
                    else if(!ec_slave[slave].islost)
                    {
                        // check if the stae of the slave is Operational
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        //if the slave is not in a valid state, indicate that the connection has been lost
                        if (ec_slave[slave].state == EC_STATE_NONE)
                        {
                            ec_slave[slave].islost = TRUE;
                            PLOGE.printf("ERROR : slave %d lost",slave);
                            flag_slave_lost = true;
                        }
                    }
                }
                //if slave is not responding (regardless of whether it is in the current group)
                if (ec_slave[slave].islost)
                {   //if slave is not in a valid state
                    if(ec_slave[slave].state == EC_STATE_NONE)
                    {
                        //try to recover the connection with the slave
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            PLOGI.printf("MESSAGE : slave %d recovered",slave);
                        }
                    }
                    else
                    { //slave is in a valid state, indicate that the connection has been regained
                        ec_slave[slave].islost = FALSE;
                        PLOGI.printf("MESSAGE : slave %d found",slave);
                    }
                }
            }
            // all the slaves' status has been checked
            if(!ec_group[currentgroup].docheckstate){
                PLOGI.printf("OK : all slaves resumed OPERATIONAL.");
            }
        }
        //suspend the execution of this thread for 10,000us
        osal_usleep(10000);
        //if the thread has been flagged to stop, then this loop terminates
        if (flag_terminate_app == TRUE)
        {
            break;
        }
    }
    PLOGI.printf("end of the ecat_chk_thread");
    return NULL;
}

void myInterruptHandler (int signum) {
    printf ("ctrl-c has been pressed. Programs will be terminated in sequence.\n");
    flag_terminate_app = true;
}

int main ()
{
    static plog::RollingFileAppender<plog::CsvFormatter> fileAppender("SOEM_ecat_log.csv", 80000, 10); // Create the 1st appender.
    static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender; // Create the 2nd appender.

    signal(SIGINT, myInterruptHandler);

    plog::init(plog::debug, &fileAppender).addAppender(&consoleAppender); // Initialize the logger with the both appenders.

    int policy;
    struct sched_param prio;
    pthread_attr_t attr;

    pthread_t tid_ecat_main_loop;
    pthread_t tid_ecat_chk;

    pthread_attr_init( &attr);
    pthread_attr_setinheritsched( &attr, PTHREAD_EXPLICIT_SCHED);
    policy = SCHED_RR;
    pthread_attr_setschedpolicy( &attr, policy);
    prio.sched_priority = 1; // priority range should be btw -20 to +19
    pthread_attr_setschedparam(&attr,&prio);

    if ( pthread_create(&tid_ecat_main_loop, &attr, ecat_main_loop, nullptr ) ){
        PLOGE <<  "Error: ecat_main_loop" ;
        return 1;
    }

    pthread_attr_init( &attr);
    pthread_attr_setinheritsched( &attr, PTHREAD_EXPLICIT_SCHED);
    policy = SCHED_OTHER;
    pthread_attr_setschedpolicy( &attr, policy);
    prio.sched_priority = 1; // priority range should be btw -20 to +19
    pthread_attr_setschedparam(&attr,&prio);

    if ( pthread_create(&tid_ecat_chk, &attr, ecat_chk, nullptr) ){
        PLOGE << "Error: ecat_chk" ;
        return 1;
    }
    /* wait for threads to finish */
    pthread_join ( tid_ecat_main_loop, NULL );
    pthread_join ( tid_ecat_chk, NULL );
    return 0;
}
