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
#include "agree_joint_structs.h"
#include "position_acquisition.h"
#include "load_acquisition.h"
#include "length_acquisition.h"
#include "file_handling/include/json_handling.h"
#include <iostream>

#include "agree_common.h"

/*****************************************************************************************
 * STRUCTS
 ****************************************************************************************/

/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/

class agree_joint_controller
{
private:

    /** Holds the configuration parameters of the SEA */
    sea_controller_configuration_t controller_config;
    /** Holds the status and relevant values of the torque control loop */
    torque_control_terms_t torque_control_loop_status;

    /** Initial reading of the loadcell - used for sign test */
    float loadcell_init_val_mNm = 0.0;
    /** Initial reading of the absolute encoder - used for sign test */
    float absolute_encoder_init_val_rad = 0.0;
    /** Initial reading of the incremental encoder - used for sign test */
    float incremental_encoder_init_val_rad = 0.0;


    /*******************************/
    /* Variables for control loops */
    /*******************************/
    /** Position error in radians from the last cycle */
    float prev_position_error_rad;
    /** Position in radians from the last cycle */
    float prev_position_radians;
    /** Velocity in radians/sec from the last cycle */
    float filtered_velocity_radians_per_sec;

    int velocity_loop_flag;


    /** Torque error in mNm from the last cycle */
    float prev_torque_error_mNm;
    /** Filtered rate of change of error used for the derivative control */
    float filtered_deriv_torque_error;
    /** Summation of the torque error used for Integral control */
    float integ_torque_error;

    /** Torque error in mNm from the last cycle */
    float prev_speed_error_rad_s = 0;
    /** Filtered rate of change of error used for the derivative control */
    float filtered_deriv_speed_error = 0;
    /** Summation of the speed error used for Integral control */
    float integ_speed_error = 0;


//    /** Max integration torque error (anti-windup)*/
//    float max_integ_torque_error_mNm = 5000;


    /** Friction compensation torque in milli-Nm from the last cycle */
    float prev_friction_comp_torque_mNm;
    /** Torque Setpoint in mNm from the last cycle */
    double total_filtered_demanded_motor_torque_mNm;

    /** External torque loop gains */
    double torque_control_p_gain,torque_control_i_gain,torque_control_d_gain;
    /** External speed loop gains */
    float speed_control_p_gain,speed_control_i_gain,speed_control_d_gain;


    /** Time in milliseconds to reach 75% of desired setpoint */
    float derating_param_time_to_arrive_75p_in_msec = 2500.0;

    position_acquisition position_reader;
    load_acquisition load_reader;
    length_acquisition length_reader;

    esmacat_err set_joint_hardware_parameters(sea_joint_configuration_t* op);
    esmacat_err set_joint_controller_parameters(sea_controller_configuration_t* configuration);
    esmacat_err configure_actuator_controller_interface();
    esmacat_err set_derating_param_time_to_arrive_75p_in_msec(float t);

    config_file_exception import_actuator_parameters_from_file(sea_controller_configuration_t *c, sea_joint_configuration_t* o, string joint_controller_param_filename);
    esmacat_err set_joint_controller_parameters_and_configure_interface(sea_joint_configuration_t* op, sea_controller_configuration_t* configuration);

    //    /** Summation of the position error in radians */
    //    float integ_position_error_rad;
    //    /** Filtered current setpoint used for the ESCON */
    //    float filtered_setpoint;
    //    float soft_to_hard_stop_offset_rad;
    //    /** Position in radians of the soft stop in the positive direction */
    //    float soft_stop_pos_rad;
    //    /** Position in radians of the soft stop in the negative direction */
    //    float soft_stop_neg_rad;


public:
    agree_joint_controller();

    actuator_controller_interface ethercat_interface;
    /** Contains all the values for the joint */
    joint_values_t joint_values;
    /** Holds the status of the impedance control loop */
    impedance_control_terms_t joint_impedance_control_terms;

    /** Holds the configuration parameters for the join */
    sea_joint_configuration_t joint_config;

    /** Offset value, used for position_control_mode **/
    double impedance_control_offset_pos_rad = 0.0;
    double impedance_control_offset_torque = 0.0;
    double impedance_control_offset_setpoint_rad = 0.0;
    double impedance_control_offset_k = 0.0;
    double impedance_control_offset_d = 0.0;
    bool   impedance_control_first_run = true;


    /** Offset time **/
    double impedance_control_offset_time_ms = 0.0;

    int calibration_status = 100;
    int incremental_encoder_calibration_counter = 0;
    bool joint_is_referenced = false;
    bool joint_is_positioned = false;

    int32_t calibration_reference_counts = 0;

    /************************/
    /* Torque control loops */
    /************************/

    esmacat_err update_joint_variables(double elapsed_time_ms, uint64_t loop_cnt);
    esmacat_err configure_joint_controller_from_file(string filename);
    esmacat_err configure_control_type();

    esmacat_err control_torque_with_soft_stop_mNm   (float torque_setpoint_mNm,float elapsed_time_ms);                                 // set desired torque in mNm, the reference is loadcell value
    esmacat_err control_torque_with_aux_input_mNm   (float torque_setpoint_mNm,float aux_input, float elapsed_time_ms);                                 // set desired torque in mNm, the reference is loadcell value
    esmacat_err control_torque_mNm                  (float torque_setpoint_mNm,float elapsed_time_ms);                                 // set desired torque in mNm, the reference is loadcell value

    esmacat_err control_ESCON_directly_0to1(float setpoint_0to1);                                 // set desired torque in mNm, the reference is loadcell value
    esmacat_err control_torque_directly_mNm(float setpoint_mNm, float elapsed_time_ms);
    esmacat_err control_torque_directly_mA(float setpoint_mA, float elapsed_time_ms);

    esmacat_err control_torque_external_mNm(double torque_setpoint_mNm, double elapsed_time_ms);
    void set_torque_gain(double p, double d, double i);

    /***********************/
    /* Speed control loops */
    /***********************/

    esmacat_err control_speed_rad_per_s(float speed_setpoint_rad_s, float elapsed_time_ms);
    void set_speed_gain(float p,float d,float i);
    void reset_encoder_values();

    joint_values_t get_joint_vals();
    void test_sign_values(float max_setpoint,float test_time_ms, float elapsed_time_ms);
    torque_control_terms_t get_torque_control_terms();
    impedance_control_terms_t get_impedance_control_terms();

    /***************************/
    /* Impedance control loops */
    /***************************/

    /** Impedance control functions */
    esmacat_err impedance_control(float setpoint_radians,float gravity_torque_mNm, float elapsed_time_ms, bool soft_stop_enable);

    void set_impedance_control_K_mNm_per_rad(double input_mNm_per_rad);
    void set_impedance_control_d_mNm_per_rad_per_sec(double input);
    void set_impedance_control_offset_rad(double input){impedance_control_offset_pos_rad = input;}
    void set_impedance_control_setpoint_rad(double setpoint_radians);


    double get_impedance_control_K_mNm_per_rad();
    double get_impedance_control_d_mNm_per_rad_per_sec();
    double get_impedance_control_setpoint_rad();


    void set_joint_config_soft_stop(double lower, double upper);

    /***********************/
    /* Homing control loop */
    /***********************/

    esmacat_err homing_control(float elapsed_time_ms);
    void        calibrate_incremental_encoder(void);

    esmacat_err set_motor_derating_factor (float factor);
    void set_one_cycle_time_s(float time_period_s);

    void set_control_mode(actuator_controller_interface::control_mode_t mode){controller_config.control_mode = (actuator_control_mode_t) mode;}
    actuator_controller_interface::control_mode_t get_current_control_mode();

    int get_joint_index();
    float get_incremental_encoder_reading_radians(void);
    float get_incremental_encoder_speed_rad_s(void);
    float get_incremental_encoder_speed_rad_s_from_driver(void);
    float get_incremental_encoder_reference_from_driver(void);
    float get_linear_actuator_reading_mm(void);
    float get_loadcell_torque_mNm(void);

    void incremental_encoder_calibration (float elapsed_time_ms, bool is_init);
    void clear_position_reading(void);

};
#endif // SEA_DRIVER_H
