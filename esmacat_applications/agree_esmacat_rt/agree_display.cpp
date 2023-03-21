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
#include "agree_shared_memory_comm.h"
#include "application.h"

#define N_DOFS_MAX 5

using namespace  std;

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/
int main()
{
    static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender; //include application.h to get the plog libraries

    plog::init(plog::info, &consoleAppender); // Initialize the logger

    agree_shared_memory_comm c;

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

    joint_values_t                   J_vals;
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

        J_vals = c.data->joint_values[1];
        T_ctrl_vals = c.data->torque_control_terms[0];
        imp_ctrl_vals = c.data->impedance_control_terms[0];

        // //////////// PRINTING VALUES TO SCREEN //////////////////

        cout
                << "Joint P_act  [deg]: " <<setw(4) << fixed << setprecision(3) << J_vals.incremental_encoder_reading_degrees << endl
                << "Torque T_des [mNm]: " <<setw(6) << fixed << setprecision(3) << J_vals.filtered_load_mNm << endl


//        cout    <<showpoint<<showpos
//                    << "Time          "
//                       "[ms]: "<< elapsed_time_ms<<"  " << endl;

//        stringstream ss;
//        ss << 0;
//        std::string index = ss.str();

//        cout        << "Fault       [bool]: "       << J_vals.escon_fault << endl
//                    << "Joint P_des  [deg]: " <<setw(4) <<fixed << setprecision(3) << imp_ctrl_vals.impedance_control_setpoint_rad*180/M_PI << endl
//                    << "Joint P_act  [deg]: " <<setw(4) << fixed << setprecision(3) << J_vals.incremental_encoder_reading_degrees << endl

//                    << "Torque T_des [mNm]: " <<setw(6) << fixed << setprecision(3) << J_vals.filtered_load_mNm << endl
//                    << "Torque T_act [mNm]: " <<setw(6)<< fixed << setprecision(3) << imp_ctrl_vals.torque_setpoint_mNm << endl

//                    << "Imp K             : " <<setw(8)<< imp_ctrl_vals.impedance_control_k_mNm_rad/1000.0 << endl
//                    << "Imp D             : " <<setw(8)<< imp_ctrl_vals.impedance_control_d_mNm_rad_per_sec/1000.0 << endl





//                << "    Load: "<<setw(8)  << J_vals.loadcell_reading_mNm
//                << "    deg: "

//                << "    fLoad: "<<setw(8) << J_vals.filtered_load_mNm
//                << "    T_des: " <<setw(8)<< imp_ctrl_vals.torque_setpoint_mNm
//                << imp_ctrl_vals.
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
//                << "    T_ff: " <<std::setw(10)<< T_ctrl_vals.feedforward_demanded_torque_mNm
//                << "    T_L_err: " <<std::setw(10)<<T_ctrl_vals.load_err
//                << "    T_p: " <<std::setw(10)<<T_ctrl_vals.feedback_p_torque_mNm
//                << "    T_d: " <<std::setw(10)<<T_ctrl_vals.feedback_d_torque_mNm
//                << "    T_i: " <<std::setw(10)<<T_ctrl_vals.feedback_i_torque_mNm
//                << "    T_fb: " <<std::setw(10)<<T_ctrl_vals.feedback_demanded_torque_mNm
//                << "    T_tot: " <<std::setw(10)<<T_ctrl_vals.feedback_demanded_torque_mNm + T_ctrl_vals.feedforward_demanded_torque_mNm
//                << endl
//                << "    T_g: "   <<setw(8)<< imp_ctrl_vals.gravity_torque_mNm << endl
//                << "    T_con:  " <<setw(8) << imp_ctrl_vals.impedance_control_torque_mNm << endl

//                << "    abs_r: " <<setw(5)  << J_vals.signed_raw_absolute_encoder_reading_cpt
//                << "    T_f: "   <<setw(8)<< imp_ctrl_vals.friction_comp_torque_mNm
//                << "    T_st: "  <<setw(8)<< imp_ctrl_vals.soft_stop_torque_mNm

//                << "    Lin_act: " <<setw(8)<< J_vals.linear_actuator_length_mm
//                << "          "
                << endl;
        cout<<endl;


    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}

/*
void print_feedback(){

    std::cout   << "Status: " << dec << robot_control_labels[control_mode] << ": " << control_mode << ">" << control_mode_status << endl
                << "Torque      [mV]: "   << fixed << setprecision(3)  << agree_joints[0].ethercat_interface.get_loadcell_reading_mV(agree_joint_error[0]) << " " <<
                                                                          agree_joints[1].ethercat_interface.get_loadcell_reading_mV(agree_joint_error[1]) << " " << endl
                << "Torque     [mNm]: "   << fixed << setprecision(3)  << agree_robot.loadcell_torque_mNm.transpose()                               << endl
                << "Gravity:   [mNm]: "   << fixed << setprecision(3)  << agree_robot.gravity_torques_mNm.transpose()                                << endl
                << "Weight:    [mNm]: "   << fixed << setprecision(3)  << agree_robot.weight_compensation_torques_mNm.transpose()                    << endl
                << "Speed:   [rad/s]: "   << fixed << setprecision(3)  << agree_robot.speed_rad_s.transpose()*rad_to_deg                            << endl
                << "Position   [deg]: "   << fixed << setprecision(3)  << agree_robot.position_rad.transpose()*rad_to_deg                           << endl
                << "Setpoint   [deg]: "   << fixed << setprecision(3)  << agree_robot.impedance_control_setpoint_rad.transpose()*rad_to_deg         << endl
                << "K_s:    [Nm/rad]: "   << fixed << setprecision(3)  << agree_joints[0].get_impedance_control_K_mNm_per_rad()         << " >> " << agree_robot.impedance_control_k_gain_mNm_per_rad.transpose()              << endl
                << "D_d:  [Nm/rad*s]: "   << fixed << setprecision(3)  << agree_joints[0].get_impedance_control_d_mNm_per_rad_per_sec() << " >> " << agree_robot.impedance_control_d_gain_mNm_per_rad_per_sec.transpose()      << endl

                << "User parameters : "   << agree_robot.weight_compensation_config.human_height_m << " [m] " << agree_robot.weight_compensation_config.human_weight_kg << " [kg] " << agree_robot.weight_compensation_config.weight_assistance*100.0 << " [%]" << endl

                << "Homing offset   : "     << agree_arduino.get_calibration_reference_counts(0) << " "
                                            << agree_arduino.get_calibration_reference_counts(1) << " "
                                            << agree_arduino.get_calibration_reference_counts(2) << " "
                                            << agree_arduino.get_calibration_reference_counts(3) << " "
                                            << agree_arduino.get_calibration_reference_counts(4) << " "
                                            << agree_arduino.get_calibration_reference_counts(5) << " >> "
                                            << agree_arduino.is_robot_referenced()  << " "
                                            << agree_arduino.is_calibration_ready() << endl << endl

                                            << "Exercise status : " << exercise_status << endl
                << "Joint referenced: "   << agree_robot.joint_is_referenced << " >> " << agree_robot.robot_is_referenced << endl

//                << "Hand Position: " << agree_robot.T_total[N_FRAMES-1].position << endl


                << "Use ROS         : "   << agree_shared_memory.data->use_ros<< endl
//                << "End-Stop  [bool]: "   << agree_joints[0].ethercat_interface.get_digital_input(0) << " " << agree_joints[0].ethercat_interface.get_digital_input(1) << endl
                << "Jacobian: " << endl << agree_robot.Jacobian << endl
//                << agree_joints[0].joint_config.hard_stop_lower_limit_degrees << " " << agree_joints[0].joint_config.hard_stop_upper_limit_degrees
//                << "Joint positioned: "   << agree_robot.joint_is_positioned << " >> " << agree_robot.joint_is_positioned.prod() << endl
//                << "Spring Counterweight: " <<   (-20.75*pow(-agree_joints[1].joint_values.incremental_encoder_reading_radians,2)-14.0*-agree_joints[1].joint_values.incremental_encoder_reading_radians+4.3)*Nm_to_mNm << endl
                << endl;
}
*/
