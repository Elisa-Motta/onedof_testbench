/** @file
 * @brief Contains declarations required for the Series-Elastic Actuator (SEA) Driver
*/
#ifndef SEA_DRIVER_H
#define SEA_DRIVER_H

/*****************************************************************************************
 * MACROS
 ****************************************************************************************/

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "error_list.h"
#include "joint_structs.h"
#include "position_acquisition.h"
#include "load_acquisition.h"
#include "length_acquisition.h"
#include "file_handling/include/json_handling.h"
#include <iostream>



/*****************************************************************************************
 * STRUCTS
 ****************************************************************************************/

/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/

class joint_controller
{
private:
    /** Contains all the values for the joint */
    sea_joint_vals_t joint_values;
    /** Holds the configuration parameters for the join */
    sea_joint_configuration_t joint_config;
    /** Holds the configuration parameters of the SEA */
    sea_controller_configuration_t controller_config;
    /** Holds the status and relevant values of the torque control loop */
    sea_torque_control_values_t torque_control_loop_status;
    /** Holds the status of the impedance control loop */
    sea_impedance_control_values_t joint_impedance_control_terms;
    /** Initial reading of the loadcell - used for sign test */
    float loadcell_init_val_mNm = 0.0;
    /** Initial reading of the absolute encoder - used for sign test */
    float absolute_encoder_init_val_rad = 0.0;
    /** Initial reading of the incremental encoder - used for sign test */
    float incremental_encoder_init_val_rad = 0.0;
    /** Position error in radians from the last cycle */
    float prev_position_error_rad;
    /** Friction compensation torque in milli-Nm from the last cycle */
    float prev_friction_comp_torque_mNm;
    /** Position in radians from the last cycle */
    float prev_position_radians;
    /** Velocity in radians/sec from the last cycle */
    float filtered_velocity_radians_per_sec;
    float derating_param_time_to_arrive_75p_in_msec = 10000.0;

    position_acquisition position_reader;
    load_acquisition load_reader;
    length_acquisition length_reader;
    esmacat_err set_joint_hardware_parameters(sea_joint_configuration_t* op);
    esmacat_err set_joint_controller_parameters(sea_controller_configuration_t* configuration);
    esmacat_err configure_actuator_controller_interface();
    esmacat_err set_derating_param_time_to_arrive_75p_in_msec(float t);

    config_file_exception import_actuator_parameters_from_file(sea_controller_configuration_t *c, sea_joint_configuration_t* o, string joint_controller_param_filename);
    esmacat_err set_joint_controlller_parameters_and_configure_interfce(sea_joint_configuration_t* op, sea_controller_configuration_t* configuration);

    //    /** Torque error on the previous cycle */
    //    float prev_torque_error;
    //    /** Filtered rate of change of error used for the derivative control */
    //    float filtered_deriv_torque_error;
    //    /** Summation of the torque error used for Integral control */
    //    float integ_torque_error;
    //    /** Summation of the position error in radians */
    //    float integ_position_error_rad;
    //    /** Filtered torque setpoint in milli-Nm */
    //    float filtered_torque_setpoint_mNm;
    //    /** Filtered current setpoint used for the ESCON */
    //    float filtered_setpoint;
    //    float soft_to_hard_stop_offset_rad;
    //    /** Position in radians of the soft stop in the positive direction */
    //    float soft_stop_pos_rad;
    //    /** Position in radians of the soft stop in the negative direction */
    //    float soft_stop_neg_rad;


public:
    joint_controller();
    actuator_controller_interface controller_interface;
    esmacat_err update_joint_variables(float elapsed_time_ms, uint64_t loop_cnt);
    esmacat_err configure_joint_controller_from_file(string filename);
    esmacat_err control_torque_with_soft_stop_mNm(float torque_setpoint_mNm,float elapsed_time_ms);                                 // set desired torque in mNm, the reference is loadcell value
    esmacat_err control_torque_with_aux_input_mNm(float torque_setpoint_mNm,float aux_input, float elapsed_time_ms);                                 // set desired torque in mNm, the reference is loadcell value
    esmacat_err control_torque_mNm(float torque_setpoint_mNm,float elapsed_time_ms);                                 // set desired torque in mNm, the reference is loadcell value
    esmacat_err control_ESCON_directly_0to1(float setpoint_0to1);                                 // set desired torque in mNm, the reference is loadcell value
    esmacat_err impedance_control(float setpoint_radians,float gravity_torque_mNm, float elapsed_time_ms);
    sea_joint_vals_t get_joint_vals();
    void test_sign_values(float max_setpoint,float test_time_ms, float elapsed_time_ms);
    sea_torque_control_values_t get_torque_control_terms();
    sea_impedance_control_values_t get_impedance_control_terms();


    void set_impedance_control_K_mNm_per_rad(double input_mNm_per_rad);
    esmacat_err set_motor_derating_factor (float factor);
    void set_one_cycle_time_s(float time_period_s);
    void set_control_mode(actuator_controller_interface::control_mode_t mode){controller_config.control_mode = mode;};
    actuator_controller_interface::control_mode_t get_current_control_mode();
    int get_joint_index();
    float get_incremental_encoder_reading_radians(void);
    float get_linear_actuator_reading_mm(void);
    void incremental_encoder_calibration (float elapsed_time_ms, bool is_init);
    void clear_position_reading(void);
};
#endif // SEA_DRIVER_H
