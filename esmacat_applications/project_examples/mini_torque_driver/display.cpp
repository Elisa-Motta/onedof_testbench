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
#include "shared_memory_comm.h"
#include "application.h"

using namespace  std;

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/
int main()
{
    static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender; //include application.h to get the plog libraries

    plog::init(plog::info, &consoleAppender); // Initialize the logger

    esmacat_shared_memory_comm c;

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

    sea_joint_vals_t J_vals;
    sea_torque_control_values_t T_ctrl_vals;
    sea_impedance_control_values_t imp_ctrl_vals;


    double elapsed_time_ms;
    int last_joint_index=3;
    int err_msg_count;
    bool is_single_joint;

    while(c.data->stop == false )
    {

        elapsed_time_ms = c.data->elapsed_time_ms;
        last_joint_index = c.data->last_joint_index;
        err_msg_count = c.data->err_msg_count;
        is_single_joint = c.data->is_single_joint;

        J_vals =c.data->R_J_vals[0];
        T_ctrl_vals = c.data->R_T_ctrl_vals[0];
        imp_ctrl_vals = c.data->R_I_ctrl_vals[0];

        // //////////// PRINTING VALUES TO SCREEN //////////////////



        cout    <<showpoint<<showpos
               << "Time(ms): "<< elapsed_time_ms<<"  ";

        stringstream ss;
        ss << 0;
        std::string index = ss.str();

        cout    << "    fault:" <<left       << J_vals.escon_fault
//                << "    Load: "<<setw(8)  << J_vals.loadcell_reading_mNm
                << "    deg: " <<setw(10) << J_vals.incremental_encoder_reading_degrees
                << "Joint P_des:  " <<setw(8) << imp_ctrl_vals.control_setpoint_rad*180/M_PI
                << "    fLoad: "<<setw(8) << J_vals.filtered_load_mNm
                << "    T_des: " <<setw(8)<< imp_ctrl_vals.torque_setpoint_mNm
//                << "    deg: " <<setw(10) << J_vals.abs_encoder_deg

//                << "    abs: " <<setw(5)  << J_vals.abs_encoder_filt_cpt
//                << "    f_abs: " <<setw(5)  << J_vals.abs_encoder_filt_cpt
//                << "    enc: " <<setw(8)  << J_vals.inc_encoder_cpt

//                << "    Pdes: "<<setw(10) << position_control_setpoint_rad*180/M_PI
//                << "    Pdes: "<<setw(10) << impedance_control_setpoint_rad*180/M_PI
//                << "    ErrFlg: "<<setw(4)<< prev_absolute_enc_err_flag
//                << endl
//                << "    T_des: " <<std::setw(10)<<T_ctrl_vals.desired_torque_mNm
//                << "    f_Tdes: " <<std::setw(10)<<T_ctrl_vals.filt_desired_torque_mNm
//                                  << "    T_ff: " <<std::setw(10)<< T_ctrl_vals.feedforward_demanded_torque_mNm
//                << "    T_L_err: " <<std::setw(10)<<T_ctrl_vals.load_err
//                << "    T_p: " <<std::setw(10)<<T_ctrl_vals.feedback_p_torque_mNm
//                << "    T_d: " <<std::setw(10)<<T_ctrl_vals.feedback_d_torque_mNm
//                << "    T_i: " <<std::setw(10)<<T_ctrl_vals.feedback_i_torque_mNm
//                                  << "    T_fb: " <<std::setw(10)<<T_ctrl_vals.feedback_demanded_torque_mNm
//                << "    T_tot: " <<std::setw(10)<<T_ctrl_vals.feedback_demanded_torque_mNm + T_ctrl_vals.feedforward_demanded_torque_mNm
//                << endl
                << "    T_g: "   <<setw(8)<< imp_ctrl_vals.gravity_torque_mNm
                << "    T_con:  " <<setw(8) << imp_ctrl_vals.controller_torque_mNm

                << "    abs_r: " <<setw(5)  << J_vals.signed_raw_absolute_encoder_reading_cpt
//                << "    T_f: "   <<setw(8)<< imp_ctrl_vals.friction_comp_torque_mNm
//                << "    T_st: "  <<setw(8)<< imp_ctrl_vals.soft_stop_torque_mNm

//                << "    K: " <<setw(8)<< imp_ctrl_vals.impedance_control_K_mNm_per_rad/1000.0
//                << "    Lin_act: " <<setw(8)<< J_vals.linear_actuator_length_mm
                << "          "
                << endl;
        cout<<endl;


    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}


