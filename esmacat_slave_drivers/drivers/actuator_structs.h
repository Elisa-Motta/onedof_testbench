#ifndef ACTUATOR_STRUCTS
#define ACTUATOR_STRUCTS


/*****************************************************************************************
 * STRUCTS
 ****************************************************************************************/

/** @brief Structure that holds all the readings of the actuator for each joint **/
struct sea_joint_vals_t
{
    /** Index of the joint within the Harmony */
    int joint_index;
    /** Error in ESCON (1), No errors in ESCON (0); IN_MD_FAULT on ESCON datasheet */
    int escon_fault;
    /** Loadcell reading for the joint in milli-Nm */
    float loadcell_reading_mNm;
    /** Filtered Loadcell reading for the joint in milli-Nm */
    float filtered_load_mNm;
    /** Raw reading with sign applied from the absolute encoder in counts per turn */
    int16_t signed_raw_absolute_encoder_reading_cpt;
    /** Unfiltered reading from the absolute encoder in counts per turn */
    int16_t unfiltered_absolute_encoder_reading_cpt;
    /** Filtered reading from the absolute encoder in counts per turn */
    float filtered_absolute_encoder_reading_cpt;
    /** Filtered reading from the absolute encoder in degrees */
    float filtered_absolute_encoder_reading_degrees;
    /** Filtered reading from the absolute encoder in radians */
    float filtered_absolute_encoder_reading_radians;
    /** Unfiltered reading from the incremental encoder in counts per turn */
    int32_t unfiltered_incremental_encoder_reading_cpt;
    /** Reading (with offset applied) from the incremental encoder in counts per turn */
    int incremental_encoder_reading_cpt;
    /** Reading (with offset applied) from the incremental encoder in degrees*/
    float incremental_encoder_reading_degrees;
    /** Reading (with offset applied) from the incremental encoder in counts radians */
    float incremental_encoder_reading_radians;
    /** Length reading the linear actuator in mm */
    float linear_actuator_length_mm;
    /** Default initialization of all the readings (loadcell readings, absolute encoder readings
     * and incremental encoder) for the joint to 0; Joint index is set to -1
     */
    sea_joint_vals_t()
    {
        joint_index = -1;
        escon_fault=0;
        loadcell_reading_mNm=0;
        filtered_load_mNm=0;
        signed_raw_absolute_encoder_reading_cpt=0;
        unfiltered_absolute_encoder_reading_cpt=0;
        filtered_absolute_encoder_reading_cpt=0;
        filtered_absolute_encoder_reading_degrees=0;
        filtered_absolute_encoder_reading_radians=0;
        unfiltered_incremental_encoder_reading_cpt=0;
        incremental_encoder_reading_cpt=0;
        incremental_encoder_reading_degrees=0;
        incremental_encoder_reading_radians=0;
        linear_actuator_length_mm=0;
    }

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
    float control_setpoint_rad;
    /** Difference between the position setpoint and the measured
     * setpoint in radians */
    float error_radians;
    /** Output torque of the PD impedance control in milli-Nm */
    float controller_torque_mNm;
    /** Torque applied due to gravity in milli-Nm */
    float gravity_torque_mNm;
    /** Torque required to compensate for friction in milli-Nm */
    float friction_comp_torque_mNm;
    float soft_stop_torque_mNm;
    /** Torque setpoint in milli-Nm that is provided as an output of impedance control */
    float torque_setpoint_mNm;
    /** Proportional gain for the impedance control loop */
    float impedance_control_K_mNm_per_rad;
    // initialize all values to 0
    sea_impedance_control_values_t()
        {
        control_setpoint_rad = 0;
        error_radians =0;
        controller_torque_mNm=0;
        gravity_torque_mNm=0;
        friction_comp_torque_mNm=0;
        soft_stop_torque_mNm=0;
        torque_setpoint_mNm=0;
        impedance_control_K_mNm_per_rad = 0;
    }
};

#endif // ACTUATOR_STRUCTS

