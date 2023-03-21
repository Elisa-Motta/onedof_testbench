/** @file
 * @brief Contains definitions of functions used for the primary executable of Harmony SHR
 * Display
 *
*/
/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include <iostream>
#include <iomanip>
#include <cmath>
#include <thread>
#include <chrono>
#include "onedof_shared_memory_comm.h"
#include "application.h"
#include "onedof_rehamove_app.h"

using namespace  std;

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/
int main()
{
    static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender; //include application.h to get the plog libraries

    plog::init(plog::info, &consoleAppender); // Initialize the logger

    onedof_shared_memory_comm c;
    onedof_rehamove_app app;

    // Initializing the shared memory
    if (c.init())
    {
        PLOGI << "User Interface shared memory initialized with key " << static_cast<int>(c.get_shared_memory_key());    // start the shared memory communication
    }
    else
    {
        PLOGE << "User Interface shared memory initialization has been failed";
        c.detach_shared_memory();
        return 0;
    }

    joint_values_t                    J_vals;
    torque_control_terms_t      T_ctrl_vals;
    impedance_control_terms_t imp_ctrl_vals;


    double elapsed_time_ms;
    int last_joint_index=3;
    int err_msg_count;
    bool is_single_joint;

    while(c.data->stop == false )
    {

        elapsed_time_ms = c.data->elapsed_time_ms;
        last_joint_index = c.data->last_joint_index;
        err_msg_count = c.data->err_msg_count;
        is_single_joint = c.data->use_ros;

        J_vals =c.data->joint_values[0];
        T_ctrl_vals = c.data->torque_control_terms[0];
        imp_ctrl_vals = c.data->impedance_control_terms[0];

        // //////////// PRINTING VALUES TO SCREEN //////////////////

        cout<<endl;
        cout<<endl;
        cout<<endl;
        cout<<endl;
        cout<<endl;
        cout<<endl;



        stringstream ss;
        ss << 0;
        std::string index = ss.str();

        cout
//                << "Time               [ms]         : "<< elapsed_time_ms<< endl
//                << "MODE                            : " << c.data->rehamove_mode << endl
////                << "TORQUE feedback    [mNm]        : " << (c.data->impedance_control_terms->torque_setpoint_mNm)-(c.data->impedance_control_terms->robot_dynamic_torque_mNm+c.data->impedance_control_terms->arm_dynamic_torque_mNm*c.data->impedance_control_command->impedance_control_feedforward_allocation_factor) << endl
//                << "ACTUAL POSITION [deg]           : " << (c.data->joint_values->incremental_encoder_reading_radians)*rad_to_deg << endl
//                << "DESIRED POSITION [deg]          : " << static_cast<double>(c.data->impedance_control_terms->impedance_control_setpoint_rad)*rad_to_deg << endl
//                << "ACTUAL TORQUE [mNm]            : " << c.data->joint_values->loadcell_reading_mNm << endl
//                << "DESIRED TORQUE [mNm]           : " << c.data->impedance_control_terms->torque_setpoint_mNm << endl
//                << "FES ALLOCATED TORQUE     [mNm]   : "  << c.data->impedance_control_terms->arm_dynamic_torque_mNm*(1.0 - c.data->impedance_control_command->impedance_control_feedforward_allocation_factor) << endl
//                << "ROBOT ALLOCATED TORQUE        [mNm]   :" << c.data->impedance_control_terms->robot_dynamic_torque_mNm+c.data->impedance_control_terms->arm_dynamic_torque_mNm*c.data->impedance_control_command->impedance_control_feedforward_allocation_factor<< endl

//                << "TOTAL DYNAMIC TORQUE            : "  << c.data->impedance_control_terms->total_dynamic_torque_mNm<< endl
                << "% WEIGHT ASSISTANCE             : " << c.data->arm_weight_compensation_config.weight_assistance << endl
//                << "Kd STIFFNESS                    : "<<  c.data->impedance_control_command->impedance_control_k_gain_mNm_per_rad   << endl
//                << "Dd DAMPING                      : " << c.data->impedance_control_command->impedance_control_d_gain_mNm_per_rad_per_sec << endl
                << "Q_ADJUSTED                      : " << c.data->rehamove_charge_adj << endl
                << "Q                               : " << c.data->rehamove_charge << endl
//                << "K_FATIGUE                       : " << c.data->rehamove_k_fatigue << endl
                << "# ITERATION                     : " << c.data->rehamove_iteration << endl
                << "MEAN POSITION ERROR flexion     : " << (c.data->rehamove_mean_position_error)*rad_to_deg  << endl
//                << "MEAN VELOCITY flexion      : " << (c.data->rehamove_mean_velocity)*rad_to_deg  << endl
//                << "# BLOCK                         :  " << c.data->rehamove_num_block << endl
//                << "MEAN BLOCK VELOCITY: " << (c.data->rehamove_mean_vel_block)*rad_to_deg  << endl
//                << "STD BLOCK VELOCITY : " << (c.data->rehamove_std_vel_block)* rad_to_deg  << endl
//                << "CURRENT M                      :" <<c.data->rehamove_m*rad_to_deg << endl
//                << "CURRENT UPPER BOUND            :" <<c.data->rehamove_up*rad_to_deg <<endl
//                << "CURRENT LOWER BOUND            :" <<c.data->rehamove_down*rad_to_deg<< endl
//                << "# TIMES M DIDN'T LOWER         : " <<c.data->rehamove_counter_no_change << endl
//                << "REPETITIONS WITH SATURATED Q       : " <<c.data->rehamove_counter_q_adj << endl

                << "% MOTOR ALLOCATION FACTOR       : " << c.data->impedance_control_command->impedance_control_feedforward_allocation_factor << endl
//                << "DELTA ALLOCATION           :    " << c.data->rehamove_delta_alloc << endl


//                << "POSITION ERROR GOAL             : " <<
//                << "DELTA ALLOCATION FACTOR         :" <<  c.data->rehamove_delta_alloc<< endl
//                << "PID DESIRED POSITION [deg]      : " << (c.data->rehamove_desired_position_pid)*rad_to_deg << endl
//                << "AMPIEZZA ZONA D'OMBRA                 : " << c.data->rehamove_mean_std_mean_position_error <<endl
//                << "FINAL POSITION ERROR flexion    : " << (c.data->rehamove_final_reaching_error)*rad_to_deg  << endl


                << endl;

        cout<<endl;
        cout<<endl;
        cout<<endl;
        cout<<endl;




    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}


