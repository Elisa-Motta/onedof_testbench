#ifndef ACTUATOR_STRUCTS
#define ACTUATOR_STRUCTS

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/

#include <iostream>
#include "actuator_controller_interface.h"
#include <stdbool.h>

/*****************************************************************************************
 * STRUCTS
 ****************************************************************************************/

/** @brief Structure that holds all the readings of the actuator for each joint **/
struct sea_joint_vals_t
{
    /** Index of the joint within the Harmony */
    int joint_index = -1;
    /** Error in ESCON (1), No errors in ESCON (0); IN_MD_FAULT on ESCON datasheet */
    int escon_fault = 0;
    /** Loadcell reading for the joint in milli-Nm */
    float loadcell_reading_mNm = 0.0;
    /** Filtered Loadcell reading for the joint in milli-Nm */
    float filtered_load_mNm = 0.0;
    /** Raw reading with sign applied from the absolute encoder in counts per turn */
    int16_t signed_raw_absolute_encoder_reading_cpt = 0;
    /** Unfiltered reading from the absolute encoder in counts per turn */
    int16_t unfiltered_absolute_encoder_reading_cpt = 0;
    /** Filtered reading from the absolute encoder in counts per turn */
    float filtered_absolute_encoder_reading_cpt = 0.0;
    /** Filtered reading from the absolute encoder in degrees */
    float filtered_absolute_encoder_reading_degrees =0.0;
    /** Filtered reading from the absolute encoder in radians */
    float filtered_absolute_encoder_reading_radians =0.0;
    /** Unfiltered reading from the incremental encoder in counts per turn */
    int32_t unfiltered_incremental_encoder_reading_cpt =0;
    /** Reading (with offset applied) from the incremental encoder in counts per turn */
    int incremental_encoder_reading_cpt =0;
//    /** Velocity reading in counts/sec from incremental encoder*/
//    float velocity_counts_per_sec = 0.0;
//    /** Acceleration in counts per sec^2 from incremental encoder */
//    float acceleration_counts_per_sec_square = 0.0;
    /** Reading (with offset applied) from the incremental encoder in degrees*/
    float incremental_encoder_reading_degrees =0.0;
    /** Reading (with offset applied) from the incremental encoder in counts radians */
    float incremental_encoder_reading_radians =0.0;
    /** Length reading the linear actuator in mm */
    float linear_actuator_length_mm =0.0;
    /** Default initialization of all the readings (loadcell readings, absolute encoder readings
     * and incremental encoder) for the joint to 0; Joint index is set to -1
     */
};

//struct sea_joint_vals_t //Currently in actuator_structs.h

/** @brief Structure that holds the configuration of the actuator for each joint **/
struct sea_joint_configuration_t
{
    /** Index of the joint within the Harmony */
    int joint_index = 0;
    /** Torque constant of the motor in milli-Nm per mA */
    float torque_constant_mNm_per_mA = 0.0;
    /** Gear ratio of the motor (output shaft to motor shaft) */
    uint16_t gear_ratio = 0;
//    /** motor_rotor_inertia_g_per_cm2 */
//    float motor_rotor_inertia_g_per_cm2 = 0.0;
//    /** motor_rotor_inertia_g_per_cm2 */
//    float gearhead_rotor_inertia_g_per_cm2 = 0.0;
    /** Gear power efficiency of the motor (output shaft to motor shaft) */
    float gear_power_efficiency = 0.0;
    /** The setpoint for the ESCON is normalized, this is the conversion factor to
     * convert the setpoint back to mA*/
    /** Sign for the current setpoint for the ESCON */
    int desired_torque_sign = 1;
    /** TRUE = Incremental encoder readings are used;
     * FALSE = Incremental encoder readings are not used */
    bool use_incremental_encoder = 1;
    /** Default initialization of the configuration paramters for the joint actuator */
    /** Hard stop position for the joint in positive direction in degrees */
    float hard_stop_upper_limit_degrees = 180.0;
    /** Hard stop position for the joint in negative direction in degrees */
    float hard_stop_lower_limit_degrees = -180.0;

    /** Offset to be applied to the loadcell reading in mV */
    float loadcell_offset_mV = 0.0;
    /** Factor that translates the loadcell reading in mV to load reading in milli-Nm */
    float loadcell_calibration_mV_to_mNm = 0.0;
    /** Sign of the readings with positive load */
    int loadcell_sign = 1;

    /** Offset for the linear actuator reading in mV */
    float linear_actuator_offset_mV = 0.0;
    /** Calibration factor to convert the linear actuator reading from mV to mm */
    float linear_actuator_calibration_mV_to_mm = 0.0;

    /** The setpoint for the ESCON is normalized, this is the conversion factor to
     * convert the setpoint back to mA*/
    float current_conversion_factor_mA_to_setpoint = 0.0;

    /** Offset of the absolute encoder in counts */
    int16_t absolute_encoder_offset_counts = 0;
    /** Offset of the incremental encoder in counts */
    /** Sign of the readings from the absolute encoder with positive position */
    int absolute_encoder_sign = 1;
    /** Resolution of the incremental encoder in counts per turn */
    int incremental_encoder_resolution_cpt = 0;
    /** Offset of the incremental encoder in counts */
    int32_t incremental_encoder_offset_counts = 0;
    /** Sign of the readings from the incremental encoder with positive position */
    int incremental_encoder_sign = 1;

    float escon_analog_output0_voltage_V_to_current_A_offset = 0.0;
    float escon_analog_output0_voltage_V_to_current_A_slope = 0.0;
    float escon_analog_output1_velocity_V_to_current_rpm_offset = 0.0;
    float escon_analog_output1_velocity_V_to_current_rpm_slope = 0.0;

};

/** Holds the configuration parameters for the torque, position and
impedance control loops*/
struct sea_controller_configuration_t
{
    /** Proportional gain for the torque control loop */
    float torque_control_p_gain = 0.0;
    /** Integral gain for the torque control loop */
    float torque_control_i_gain = 0.0;
    /** Derivative gain for the torque control loop */
    float torque_control_d_gain = 0.0;
    /** Max allowable torque change in milli-Nm per interval */
    uint16_t max_torque_change_mNm_per_ms = 0;
    /** Proportional gain for the position control loop */
    float position_control_p_gain = 0.0;
    /** Integral gain for the position control loop */
    float position_control_i_gain = 0.0;
    /** Derivative gain for the position control loop */
    float position_control_d_gain = 0.0;
    /** Proportional gain for the impedance control loop */
    float impedance_control_k_gain_mNm_per_rad = 0.0;
    /** Derivative gain for the impedance control loop */
    float impedance_control_d_gain_mNm_per_rad_per_sec =0.0;
    /** Maximum error in radians (used for limiting the measured error)
     * allowed in the impedance control loop */
    float impedance_control_max_error_radians = 0.0;
    /** Maximum allowed torque in milli-Nm used to limit the torque
     * applied to compensate for friction */
    float friction_comp_max_torque_mNm = 0.0;
    float friction_torque_threshold_rad_per_s = 0.0;

    float soft_to_hard_stop_offset_deg = 0.0;
    float soft_stop_max_torque_mNm = 0.0;
    /** Maximum allowable velocity under impedance control in rads/sec */
    float max_velocity_threshold_rad_per_sec = 0.0;
    /** Maximum allowable loadcell reading in milli-Nm; used to disable
     * motor */

    float max_torque_control_input_mNm = 0.0;

    float min_torque_control_input_mNm = 0.0;

    float velocity_low_pass_filter_weight_for_current_measure = 1.0;

    float loadcell_low_pass_filter_weight_for_current_measure = 1.0;

    float gain_inertia_dynamics_compensation = 0.0;

    float max_integrated_torque_error_mNm = 0.0;

    float max_allowable_redundancy_error_for_motor_current_mA = 0.0;

    float max_allowable_redundancy_error_for_motor_velocity_rad_per_sec = 0.0;

    /** Sets the control mode in which the controller is used: <br>
     * 2 = torque control performed on the slave <br>
     * anything else = current control through ESCON performed on slave*/
    actuator_controller_interface::control_mode_t control_mode;

    /** Initializes all the parameters of the torque, position
    and impedance control loops to 0 */
};


/** Holds all values pertaining to the torque control loop */
struct sea_torque_control_values_t
{
    /** Feedforward torque in milli-Nm that is applied in addition to the PID output */
    float feedforward_demanded_torque_mNm = 0;
    /** Torque setpoint in milli-Nm that is provided as an input to torque control loop */
    float torque_setpoint_mNm;
    /** Difference between the torque setpoint and the measured torque in milli-Nm */
    float load_err;
    /** Torque output by the Proportional control in the PID */
    float pterm_mNm;
    /** Torque output by the Integral control in the PID */
    float iterm_mNm;
    /** Torque output by the Derivative control in the PID */
    float dterm_mNm;
    /** Torque in milli-Nm output by the PID */
    float pid_output_torque_mNm;
    /** Feedforward current setpoint for the ESCON : ranges from -1 to +1 */
    float setpoint_feedforward;
    /** Current setpoint for the ESCON computed from the PID: ranges from -1 to +1 */
    float setpoint_pid;
    /** Final current setpoint that includes both the feedforward and the PID output:
     * ranges from -1 to + 1 */
    float setpoint;
    /** Initialize all values of the struct to 0 */
    sea_torque_control_values_t()
        {
        feedforward_demanded_torque_mNm =0;
        torque_setpoint_mNm=0;
        load_err=0;
        pterm_mNm=0;
        iterm_mNm=0;
        dterm_mNm=0;
        pid_output_torque_mNm=0;
        setpoint_feedforward=0;
        setpoint_pid=0;
        setpoint=0;
    }
};

/** Holds all values pertaining to the impedance control loop */
struct sea_impedance_control_values_t
{
    /** Position setpoint for impedance control in radians */
    float control_setpoint_rad = 0.0;
    /** Difference between the position setpoint and the measured
     * setpoint in radians */
    float error_radians = 0.0;
    /** Output torque of the PD impedance control in milli-Nm */
    float controller_torque_mNm = 0.0;
    /** Torque applied due to gravity in milli-Nm */
    float gravity_torque_mNm = 0.0;
    /** Torque required to compensate for friction in milli-Nm */
    float friction_comp_torque_mNm = 0.0;
    float soft_stop_torque_mNm = 0.0;
    /** Torque setpoint in milli-Nm that is provided as an output of impedance control */
    float torque_setpoint_mNm = 0.0;
    /** Proportional gain for the impedance control loop */
    float impedance_control_K_mNm_per_rad = 0.0;
    // initialize all values to 0
};

#endif // ACTUATOR_STRUCTS

