/** @file
 * @brief Contains definitions of functions used for the Series-Elastic Actuator (SEA) Driver
*/

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "joint_controller.h"

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/

joint_controller::joint_controller()
{
    //    std::cout <<"EsmaCAT SEA Driver Object is created by interiting Motor Driver Class" << std::endl;

    controller_interface.esmacat_slave_vendor_id = SEA_DRIVER_VENDOR_ID;
    controller_interface.esmacat_slave_product_id = SEA_DRIVER_PRODUCT_ID;

    loadcell_init_val_mNm = 0;
    absolute_encoder_init_val_rad = 0;
    incremental_encoder_init_val_rad = 0;

}

esmacat_err joint_controller::update_joint_variables(float elapsed_time_ms, uint64_t loop_cnt)
{
    // update variables from sensors
    esmacat_err loadcell_error;
    esmacat_err linear_actuator_error;
    joint_values.joint_index = get_joint_index();
    joint_values.escon_fault = controller_interface.get_escon_fault();
    joint_values.loadcell_reading_mNm = load_reader.get_load_mNm(loadcell_error, &controller_interface);
    joint_values.filtered_load_mNm = load_reader.get_filtered_load_mNm(loadcell_error, &controller_interface);
    joint_values.signed_raw_absolute_encoder_reading_cpt = position_reader.get_signed_raw_absolute_encoder_reading_cpt(&controller_interface);
    joint_values.unfiltered_absolute_encoder_reading_cpt=position_reader.get_unfiltered_absolute_encoder_reading_cpt(&controller_interface);
    joint_values.filtered_absolute_encoder_reading_cpt=position_reader.get_filtered_absolute_encoder_reading_cpt(&controller_interface);
    joint_values.filtered_absolute_encoder_reading_radians = position_reader.get_filtered_absolute_encoder_reading_radians(&controller_interface);
    joint_values.filtered_absolute_encoder_reading_degrees = position_reader.get_filtered_absolute_encoder_reading_radians(&controller_interface)*static_cast<float>(180/M_PI);
    joint_values.unfiltered_incremental_encoder_reading_cpt= position_reader.get_signed_raw_incremental_encoder_reading_cpt(&controller_interface);
    joint_values.incremental_encoder_reading_cpt= position_reader.get_incremental_encoder_reading_cpt(&controller_interface);
    joint_values.incremental_encoder_reading_radians = position_reader.get_incremental_encoder_reading_radians(&controller_interface, joint_config.gear_ratio);
    joint_values.incremental_encoder_reading_degrees = joint_values.incremental_encoder_reading_radians*static_cast<float>(180/M_PI);
    joint_values.linear_actuator_length_mm = length_reader.get_linear_actuator_length_mm(linear_actuator_error, &controller_interface);

    //Setting incremental encoder offset from absolute encoder
//    incremental_encoder_calibration(static_cast<float>(elapsed_time_ms),false);

    // give the loop cnt to controller interface and slave
    controller_interface.set_current_loop_cnt(loop_cnt);

    // read the feedback values sequentially.
    controller_interface.read_feedback_variables_sequentially(loop_cnt,false);

    if (loadcell_error != NO_ERR)
    {
        PLOGE << "Error while updating joint variables of load cell";
        return loadcell_error;
    }
    else if (linear_actuator_error != NO_ERR)
    {
        PLOGE << "Error while updating joint variables of linear actuator";
        return linear_actuator_error;
    }
    else
    {
        return NO_ERR;
    }

}

esmacat_err joint_controller::set_joint_controlller_parameters_and_configure_interfce(sea_joint_configuration_t *joint_config, sea_controller_configuration_t *control_config)
{
    set_joint_hardware_parameters(joint_config);
    set_joint_controller_parameters(control_config);
    configure_actuator_controller_interface();
    return NO_ERR;
}

esmacat_err joint_controller::configure_joint_controller_from_file(std::string filename)
{
    import_actuator_parameters_from_file( &controller_config, &joint_config, filename);
    set_joint_controlller_parameters_and_configure_interfce(&joint_config, &controller_config);
    return NO_ERR;
}


/** @brief Sets the configuration of the joint from the input configuration
 * parameters
 *
 * @param op Struct containing all the input configuration parameters
 * @return Error status of the function
 */
esmacat_err joint_controller::set_joint_hardware_parameters(sea_joint_configuration_t* op)
{
    joint_config = *op;

    load_reader.set_loadcell_calibration(op->loadcell_calibration_mV_to_mNm, &controller_interface);
    load_reader.set_loadcell_zero_offset(op->loadcell_offset_mV, &controller_interface);
    controller_interface.set_current_conversion_factor(op-> current_conversion_factor_mA_to_setpoint);
    position_reader.set_incremental_encoder_resolution_cpt(op->incremental_encoder_resolution_cpt);
    position_reader.set_absolute_encoder_offset_counts(op->absolute_encoder_offset_counts);
    position_reader.set_incremental_encoder_offset_counts(op->incremental_encoder_offset_counts);
    length_reader.set_linear_actuator_zero_offset(op->linear_actuator_offset_mV);
    length_reader.set_linear_actuator_calibration(op->linear_actuator_calibration_mV_to_mm);
    load_reader.set_loadcell_sign(op->loadcell_sign, &controller_interface);
    position_reader.set_absolute_encoder_sign(op->absolute_encoder_sign);
    position_reader.set_incremental_encoder_sign(op->incremental_encoder_sign);
    load_reader.set_loadcell_sign(op->loadcell_sign, &controller_interface);


    return NO_ERR;
}


/** @brief Obtains the joint index set in the configuration
 * @return Joint Index
 */
int joint_controller::get_joint_index()
{
    return joint_config.joint_index;
}

/** @brief Computes all the values for the joint from the sensor readings and sets the
 * values in the struct
 *
 * @return Joint readings in the form of sea_joint_vals_t struct
 */
sea_joint_vals_t joint_controller::get_joint_vals()
{
    return joint_values;
}

/** @brief To test for sign values of both encoders, load cell and ESCON setpoint
 *
 * If the application has run for more than 1 s but less than test_time_ms, the motor is enabled.
 * All three values should read +1 and actuator should move in the positive direction if signs have been set correctly.
 * If encoder values read 0, allow the joint to move more than 1 degree.
 * If load cell value reads 0, resist motion of the joint and increase the max_setpoint inputted
 * If all three values are  -1, likely that ESCON setpoint is incorrect. Check direction of movement to verify.
 *
 * @param max_setpoint Setpoint to test drive the actuator in order to compute the sign values
 * @param test_time_ms Time in ms over which the actuator is driven
 * @param elapsed_time_ms Time that has elapsed since the application has begun executing (in ms)
 */

void joint_controller::test_sign_values(float max_setpoint,float test_time_ms, float elapsed_time_ms)
{
    cout << "test sign values";
    //initialize sign check values
    int loadcell_sign_check =0;
    int abs_enc_sign_check = 0;
    int inc_enc_sign_check = 0;

    esmacat_err loadcell_error;
    // set initial values for the readings
    if(elapsed_time_ms <= 1000)
    {
        loadcell_init_val_mNm = load_reader.get_filtered_load_mNm(loadcell_error, &controller_interface);
        absolute_encoder_init_val_rad = position_reader.get_filtered_absolute_encoder_reading_radians(&controller_interface);
        incremental_encoder_init_val_rad = position_reader.get_incremental_encoder_reading_radians(&controller_interface, joint_config.gear_ratio);
    }
    else
    {
        //the test is executed 1s after the application begins execution, and runs till test_time_ms
        float time_scale = std::min(static_cast<float>(1),std::max(static_cast<float>(0), (elapsed_time_ms-1000)/test_time_ms));

        cout << "a1";
        controller_interface.set_control_setpoint_0to1(max_setpoint*time_scale*joint_config.desired_torque_sign);
        controller_interface.set_escon_enable(1);

        float loadcell_curr_val = load_reader.get_filtered_load_mNm(loadcell_error, &controller_interface);
        float absolute_encoder_curr_val = position_reader.get_filtered_absolute_encoder_reading_radians(&controller_interface);
        float incremental_encoder_curr_val = position_reader.get_incremental_encoder_reading_radians(&controller_interface, joint_config.gear_ratio);

        //if loadcell value has changed by at least 50 mNm, obtain sign
        if(std::abs(loadcell_curr_val-loadcell_init_val_mNm)>50)
        {
            loadcell_sign_check =static_cast<int>((loadcell_curr_val-loadcell_init_val_mNm)/(static_cast<float>(0.9)*std::abs(loadcell_curr_val-loadcell_init_val_mNm)));
        }
        //if the absolute encoder reading has changed by pi/180, obtain sign
        if(std::abs(absolute_encoder_curr_val-absolute_encoder_init_val_rad)>static_cast<float>((1*M_PI/180)))
        {
            abs_enc_sign_check =static_cast<int>((absolute_encoder_curr_val-absolute_encoder_init_val_rad)/(static_cast<float>(0.9)*std::abs(absolute_encoder_curr_val-absolute_encoder_init_val_rad)));
        }
        //if incremental encoder reading has changed by pi/180, obtain sign
        if(std::abs(incremental_encoder_curr_val-incremental_encoder_init_val_rad)>static_cast<float>((1*M_PI/180)))
        {
            inc_enc_sign_check =static_cast<int>((incremental_encoder_curr_val-incremental_encoder_init_val_rad)/(static_cast<float>(0.9)*std::abs(incremental_encoder_curr_val-incremental_encoder_init_val_rad)));
        }

    }
    std::cout
            <<"\r  LC_sign_chk " <<  loadcell_sign_check
           <<"  abs_enc_sgn_chk " <<abs_enc_sign_check
          <<"  inc_enc_sgn_chk " <<inc_enc_sign_check
         <<"  ";
}

/** @brief Returns the terms associated with torque control
 * @return Struct containing all the terms associated with the torque control loop
 */
sea_torque_control_values_t joint_controller::get_torque_control_terms()
{
    return torque_control_loop_status;
}

/** @brief Returns the terms associated with impedance control
 * @return Struct containing all the terms associated with the impedance control loop
 */
sea_impedance_control_values_t joint_controller::get_impedance_control_terms()
{
    return joint_impedance_control_terms;
}

esmacat_err joint_controller::control_torque_with_soft_stop_mNm(float torque_setpoint_mNm, float elapsed_time_ms)
{
    const float delta = controller_config.soft_to_hard_stop_offset_deg;

    float p = joint_values.incremental_encoder_reading_degrees ;
    float p_up = joint_config.hard_stop_upper_limit_degrees;
    float p_low = joint_config.hard_stop_lower_limit_degrees;

    float dist_to_uppper_lim = (p_up - p);
    if (dist_to_uppper_lim < 0.0 ) dist_to_uppper_lim = 0.0;
    if (dist_to_uppper_lim > delta) dist_to_uppper_lim = delta;
    float t_push_down = controller_config.soft_stop_max_torque_mNm * (delta - dist_to_uppper_lim)/delta;

    float dist_to_lower_lim = (p - p_low);
    if (dist_to_lower_lim < 0.0 ) dist_to_lower_lim = 0.0;
    if (dist_to_lower_lim > delta) dist_to_lower_lim = delta;
    float t_push_up = controller_config.soft_stop_max_torque_mNm * (delta - dist_to_lower_lim)/delta;

    control_torque_mNm(torque_setpoint_mNm - t_push_down + t_push_up,elapsed_time_ms );

    return NO_ERR;
}

esmacat_err joint_controller::control_torque_with_aux_input_mNm(float torque_setpoint_mNm, float aux_input, float elapsed_time_ms)
{
    controller_interface.set_aux_setpoint(aux_input);
    control_torque_mNm(torque_setpoint_mNm,elapsed_time_ms );

    return NO_ERR;
}

esmacat_err joint_controller::control_torque_mNm(float torque_setpoint_mNm, float elapsed_time_ms)
{
    if ( get_current_control_mode() != actuator_controller_interface::control_mode_t::torque_control) // if wrong control mode is selected
    {
        set_motor_derating_factor(0);
        controller_interface.set_escon_enable(false);
        controller_interface.set_control_setpoint_0to1(0.0);
        PLOGE << "Torque control has been attempted under non-torque-control mode";
        return ERR_SEA_WRONG_CONTROL_MODE_SELECTED;
    }

    if ( torque_setpoint_mNm > controller_config.max_torque_control_input_mNm  ) // if the control input is larger than its allowed range
    {
        PLOGW << "Torque control input, " << torque_setpoint_mNm << ", is higher than the max allowed torque input, " << controller_config.max_torque_control_input_mNm  << "the torque input is replaced with the max allowed value";
        torque_setpoint_mNm = controller_config.max_torque_control_input_mNm;

    }
    if ( torque_setpoint_mNm < controller_config.min_torque_control_input_mNm  ) // if the control input is larger than its allowed range
    {
        PLOGW << "Torque control input, " << torque_setpoint_mNm << ", is lower than the min allowed torque input, " << controller_config.min_torque_control_input_mNm  << "the torque input is replaced with the max allowed value";
        torque_setpoint_mNm = controller_config.min_torque_control_input_mNm;
    }

    // set derating factor
    float derating_factor = 0.0;
    if (derating_param_time_to_arrive_75p_in_msec < 1e-5) derating_factor = 1.0;
    else derating_factor = smooth_start_func_tanh(elapsed_time_ms/derating_param_time_to_arrive_75p_in_msec);
    set_motor_derating_factor( derating_factor );
    controller_interface.set_control_setpoint(torque_setpoint_mNm);
    return NO_ERR;
}

esmacat_err joint_controller::control_ESCON_directly_0to1(float setpoint)
{
    if ( get_current_control_mode() != actuator_controller_interface::control_mode_t::direct_escon_control )
    {
        setpoint = 0.0;
        PLOGE << "direct_escon_control function was colled wihtout selecting the mode, DIRECT_ESCON_CONTROL ";
        controller_interface.set_escon_enable(false);
        return ERR_SEA_WRONG_CONTROL_MODE_SELECTED;
    }
    else{
        if (setpoint > 1.0)
        {
            setpoint = 1.0;
            PLOGW << "direct_escon_control function attempted to send a setpoint higher than 1.0";
        }
        if (setpoint < -1.0)
        {
            setpoint = -1.0;
            PLOGW << "direct_escon_control function attempted to send a setpoint lower than -1.0";
        }
        controller_interface.set_escon_enable(true);
        cout << "a3";
        controller_interface.set_control_setpoint_0to1(setpoint);
        return  NO_ERR;
    }

}

/** @brief Performs the impedance control and determines the input to the torque control loop
 *
 * Ensure that all signs are set as defined in torque_control before using this function.
 * @param setpoint_radians Impedance control setpoint for position in radians
 * @param gravity_torque_mNm Gravity torque setting in milli-Nm
 * @param elapsed_time_ms Time elapsed since the application has been initialized in ms
 * @return Status of the function which indicates whether it was executed successfully
 * (NO_ERR indicates success; ERR_MEASURED_VELOCITY_OUTSIDE_RANGE, ERR_MEASURED_LOAD_OUTSIDE_RANGE
 * indicate that errors were encountered; additional errors may be passed through from the driver
 */
esmacat_err joint_controller::impedance_control(float setpoint_radians,float gravity_torque_mNm, float elapsed_time_ms)
{
    esmacat_err error = NO_ERR;

    float impedance_control_k_gain_mNm_per_rad = controller_config.impedance_control_k_gain_mNm_per_rad;
    float impedance_control_d_gain_mNm_per_rad_per_sec = controller_config.impedance_control_d_gain_mNm_per_rad_per_sec;
    float impedance_control_max_error_radians = controller_config.impedance_control_max_error_radians;
    float friction_comp_max_torque_mNm = controller_config.friction_comp_max_torque_mNm;
    float friction_torque_threshold_rad_per_s = controller_config.friction_torque_threshold_rad_per_s;

    float soft_stop_max_torque_mNm = controller_config.soft_stop_max_torque_mNm;
    float max_velocity_threshold_rad_per_sec = controller_config.max_velocity_threshold_rad_per_sec;
    float max_load_stop_threshold_mNm = controller_config.soft_stop_max_torque_mNm;

    float kterm_mNm =0;
    float dterm_mNm =0;
    float controller_torque_mNm =0;
    float error_radians = 0;
    float torque_setpoint_mNm = 0;
    float friction_comp_torque_mNm = 0;
    float soft_stop_torque_mNm = 0;

    float measured_position_radians = 0;
    if (joint_config.use_incremental_encoder == true)
    {
        measured_position_radians = position_reader.get_incremental_encoder_reading_radians(&controller_interface,joint_config.gear_ratio);
        error_radians = setpoint_radians- measured_position_radians;
    }
    else
    {
        measured_position_radians = position_reader.get_filtered_absolute_encoder_reading_radians(&controller_interface);
        error_radians = setpoint_radians- measured_position_radians;
    }
    //compute the velocity in radians/sec
    float velocity_radians_per_sec = (measured_position_radians - prev_position_radians)/1000;
    prev_position_radians = measured_position_radians;
    filtered_velocity_radians_per_sec = velocity_radians_per_sec*0.05 + filtered_velocity_radians_per_sec*0.95;

    //Controller Torque
    // limit the error measured to between -max to +max
    error_radians = std::min(std::max(error_radians,-impedance_control_max_error_radians), impedance_control_max_error_radians);
    kterm_mNm = impedance_control_k_gain_mNm_per_rad * error_radians;
    dterm_mNm = -impedance_control_d_gain_mNm_per_rad_per_sec*filtered_velocity_radians_per_sec;
    controller_torque_mNm = kterm_mNm+dterm_mNm;

    //Friction Torque
    if (filtered_velocity_radians_per_sec > friction_torque_threshold_rad_per_s)
    {
        friction_comp_torque_mNm = -friction_comp_max_torque_mNm;
    }
    else if (filtered_velocity_radians_per_sec < -friction_torque_threshold_rad_per_s)
    {
        friction_comp_torque_mNm = friction_comp_max_torque_mNm;
    }
    else
    {
        friction_comp_torque_mNm = 0;
    }
    //filter torque
    friction_comp_torque_mNm = static_cast<float>(0.4) * friction_comp_torque_mNm + static_cast<float>(0.6) * prev_friction_comp_torque_mNm;
    prev_friction_comp_torque_mNm = friction_comp_torque_mNm;

    //safety stops

    //     //Soft end stop
    //        if ((hard_stop_pos_rad - measured_position_radians) <= (1.5*soft_to_hard_stop_offset_rad)  &&(velocity_radians_per_sec>0))
    //		{
    //            float scaled_travel_beyond_soft_stop_pos = (measured_position_radians-soft_stop_pos_rad)/soft_to_hard_stop_offset_rad;
    //            soft_stop_torque_mNm = soft_stop_max_torque_mNm * sigmoid(-scaled_travel_beyond_soft_stop_pos*15)*sigmoid(-velocity_radians_per_sec * pow(10,9));

    //        }
    //		else if ((measured_position_radians - hard_stop_neg_rad) <= (1.5*soft_to_hard_stop_offset_rad)  &&(velocity_radians_per_sec<0))
    //		{
    //            float scaled_travel_beyond_soft_stop_neg = (soft_stop_neg_rad - measured_position_radians)/soft_to_hard_stop_offset_rad;
    //            soft_stop_torque_mNm = -soft_stop_max_torque_mNm * sigmoid(-scaled_travel_beyond_soft_stop_neg*15)*sigmoid(velocity_radians_per_sec * pow(10,9));
    //        }

    // disable motor if velocity exceeds threshold
    if (std::abs(filtered_velocity_radians_per_sec) >= max_velocity_threshold_rad_per_sec)
    {
        controller_interface.set_escon_enable(0);
        error = ERR_MEASURED_VELOCITY_OUTSIDE_RANGE;
        std::cout<<"ERR_MEASURED_VELOCITY_OUTSIDE_RANGE. ESCON Disabled"<<std::endl;
    }
    else
    {
        controller_interface.set_escon_enable(1);
    }

    // disable motor if load exceeds threshold
    esmacat_err loadcell_reading_status;
    float loadcell_reading_mNm = load_reader.get_filtered_load_mNm(loadcell_reading_status, &controller_interface);
    if (loadcell_reading_status == NO_ERR)
    {
        if (std::abs(loadcell_reading_mNm) >= max_load_stop_threshold_mNm)
        {
            controller_interface.set_escon_enable(0);
            error = ERR_MEASURED_LOAD_OUTSIDE_RANGE;
            std::cout << "ERR_MEASURED_LOAD_OUTSIDE_RANGE. J" << joint_config.joint_index << " ESCON Disabled" <</*get_filtered_load_mNm()<<*/std::endl;
        }
        else
        {
            controller_interface.set_escon_enable(1);
        }
    }
    else
    {
        error = loadcell_reading_status;
    }
    //computing the torque setpoint and running torque control
    torque_setpoint_mNm = gravity_torque_mNm + controller_torque_mNm + friction_comp_torque_mNm + soft_stop_torque_mNm;

//    set_desired_torque_mNm(torque_setpoint_mNm, elapsed_time_ms);
    control_torque_with_soft_stop_mNm(torque_setpoint_mNm, elapsed_time_ms);

    //saving values into the struct
    joint_impedance_control_terms.torque_setpoint_mNm = torque_setpoint_mNm;
    joint_impedance_control_terms.control_setpoint_rad = setpoint_radians;
    joint_impedance_control_terms.error_radians = error_radians;
    joint_impedance_control_terms.controller_torque_mNm = controller_torque_mNm;
    joint_impedance_control_terms.gravity_torque_mNm = gravity_torque_mNm;
    joint_impedance_control_terms.friction_comp_torque_mNm = friction_comp_torque_mNm;
    joint_impedance_control_terms.soft_stop_torque_mNm = soft_stop_torque_mNm;
    joint_impedance_control_terms.impedance_control_K_mNm_per_rad = controller_config.impedance_control_k_gain_mNm_per_rad;

    return error;
}

/** @brief Reads the input configuration for the joint controller and sets the internal
 * variables
 *
 * @param configuration Input controller configuration parameters for the joint
 * @return Status of the function; NO_ERR indicates successful execution
*/
esmacat_err joint_controller::set_joint_controller_parameters(sea_controller_configuration_t* configuration)
{
    controller_config = *configuration;
    return NO_ERR;
}

esmacat_err joint_controller::configure_actuator_controller_interface()
{
    controller_interface.configure_gear_ratio(joint_config.gear_ratio);
    controller_interface.configure_gear_power_efficiency(joint_config.gear_power_efficiency);
    controller_interface.configure_torque_constant(joint_config.torque_constant_mNm_per_mA);
    controller_interface.configure_escon_setpoint_sign(joint_config.desired_torque_sign);
    controller_interface.configure_incremental_encoder_resolution_cpt( joint_config.incremental_encoder_resolution_cpt );
    controller_interface.configure_escon_analog_output0_voltage_V_to_current_A_offset(joint_config.escon_analog_output0_voltage_V_to_current_A_offset);
    controller_interface.configure_escon_analog_output0_voltage_V_to_current_A_slope(joint_config.escon_analog_output0_voltage_V_to_current_A_slope);
    controller_interface.configure_escon_analog_output1_velocity_V_to_current_rpm_offset(joint_config.escon_analog_output1_velocity_V_to_current_rpm_offset);
    controller_interface.configure_escon_analog_output1_velocity_V_to_current_rpm_slope(joint_config.escon_analog_output1_velocity_V_to_current_rpm_slope);

//    controller_interface.configure_motor_rotor_inertia_g_per_cm2( joint_config.motor_rotor_inertia_g_per_cm2 );
//    controller_interface.configure_gearhead_rotor_inertia_g_per_cm2( joint_config.gearhead_rotor_inertia_g_per_cm2 );

    controller_interface.configure_max_torque_change_mNm_per_ms(controller_config.max_torque_change_mNm_per_ms);
    controller_interface.configure_torque_control_p_gain(controller_config.torque_control_p_gain);
    controller_interface.configure_torque_control_i_gain(controller_config.torque_control_i_gain);
    controller_interface.configure_torque_control_d_gain(controller_config.torque_control_d_gain);
    controller_interface.configure_control_type( controller_config.control_mode );
    controller_interface.configure_velocity_low_pass_filter_weight_for_current_measure( controller_config.velocity_low_pass_filter_weight_for_current_measure);
    controller_interface.configure_loadcell_low_pass_filter_weight_for_current_measure( controller_config.loadcell_low_pass_filter_weight_for_current_measure);
    controller_interface.configure_gain_inertia_dynamics_compensation( controller_config.gain_inertia_dynamics_compensation );
    controller_interface.configure_max_allowable_redundancy_error_for_motor_current_mA(controller_config.max_allowable_redundancy_error_for_motor_current_mA);
    controller_interface.configure_max_allowable_redundancy_error_for_motor_velocity_rad_per_sec(controller_config.max_allowable_redundancy_error_for_motor_velocity_rad_per_sec);
//    controller_interface.configure_slave_encoder_clear();

    return NO_ERR;
}

esmacat_err joint_controller::set_derating_param_time_to_arrive_75p_in_msec(float t)
{
    if (t < 0 )
    {
        PLOGW << "derating_parameter must be larger than zero";
        t = 0.0;
    }

    derating_param_time_to_arrive_75p_in_msec = t;
    return NO_ERR;
}


/** @brief Set the K term for impedance control
 */
void joint_controller::set_impedance_control_K_mNm_per_rad(double input_mNm_per_rad)
{
    controller_config.impedance_control_k_gain_mNm_per_rad = std::max(0.0,std::min(1000*100.0,input_mNm_per_rad));
}

///** @brief Generates a sigmoid curve using the input parameter
// *
// * Sigmoid f(x) = 1/(1+ exp(-x))
// * @param x used in f(x) for sigmoid function
// * @return f(x) computed output of the sigmoid function
// */
//float joint_controller::sigmoid(float x)
//{
//    float one = static_cast<float>(1);
//    float exponent = static_cast<float>(exp(static_cast<double>(x)));
//    float output = static_cast<float>(one/(one + exponent));
//    return output;
//}



//void joint_controller::set_desired_torque_mNm(float torque_setpoint_mNm, float elapsed_time_ms)
//{
//    joint_impedance_control_terms.torque_setpoint_mNm = torque_setpoint_mNm;
//    torque_control(torque_setpoint_mNm , elapsed_time_ms);
//}

/** Allows for a derating fraction between 0 and 1 to be set
 */
esmacat_err joint_controller::set_motor_derating_factor(float factor)
{
    esmacat_err error = controller_interface.set_escon_derating_factor(factor);
    return error;
}


/** @brief Obtains incremental encoder readings in radians for the position reader */
float joint_controller::get_incremental_encoder_reading_radians()
{
    float output = position_reader.get_incremental_encoder_reading_radians(&controller_interface,joint_config.gear_ratio);
    return output;
}

/** @brief Obtains length in mm */
float joint_controller::get_linear_actuator_reading_mm()
{
    esmacat_err linear_actuator_error;
    float output = length_reader.get_linear_actuator_length_mm(linear_actuator_error,&controller_interface);
    return output;
}


/** @brief Initiates calibration of the incremental encoder in the position reader */
void joint_controller::incremental_encoder_calibration (float elapsed_time_ms, bool is_init)
{
    position_reader.incremental_encoder_calibration (elapsed_time_ms, is_init, &controller_interface, joint_config.gear_ratio);
}

/** @brief Queues the gear power efficiency parameter of the actuator to configure the slave */
void joint_controller::clear_position_reading()
{
    position_reader.clear_incremental_encoder(&controller_interface);
}
/** @brief Sets the time period for the slave application */
void joint_controller::set_one_cycle_time_s(float time_period_s)
{
    controller_interface.set_one_cycle_time_s (time_period_s);
}

actuator_controller_interface::control_mode_t joint_controller::get_current_control_mode()
{
    return controller_config.control_mode;
}



/** @brief Sets the loaded joint parameters into the respective actuator controller and driver objects
 *
 * @param Pointer to the sea_controller object
 * @param Index of the joint (Note that J[0] and J[1] are unused)
 */
config_file_exception joint_controller::import_actuator_parameters_from_file(sea_controller_configuration_t *c, sea_joint_configuration_t* o,string joint_controller_param_filename)
{

    json_data_file onedof_parameters;
    onedof_parameters.parse(joint_controller_param_filename);

    config_file_exception no_error = config_file_exception::SUCCESS;
    config_file_exception error;

    try {
        json_object actuator_parameters = onedof_parameters.get_object("sea_joint_parameters");

        o->joint_index = actuator_parameters.get_scalar<_float>("joint_index").value;
        o->loadcell_calibration_mV_to_mNm = actuator_parameters.get_scalar<_float>("loadcell_calibration_mV_to_mNm").value;
        o->loadcell_offset_mV = actuator_parameters.get_scalar<_float>("loadcell_offset_mV").value;
        o->gear_power_efficiency = actuator_parameters.get_scalar<_float>("gear_power_efficiency").value;
        o->gear_ratio = actuator_parameters.get_scalar<_uint16_t>("gear_ratio").value;
        o->current_conversion_factor_mA_to_setpoint = static_cast<float>(actuator_parameters.get_scalar<_float>("current_conversion_factor_mA_to_setpoint").value/1000);
        o->torque_constant_mNm_per_mA = actuator_parameters.get_scalar<_float>("torque_constant_mNm_per_mA").value;

        o->hard_stop_upper_limit_degrees = actuator_parameters.get_scalar<_float>("hard_stop_upper_limit_degrees").value;
        o->hard_stop_lower_limit_degrees = actuator_parameters.get_scalar<_float>("hard_stop_lower_limit_degrees").value;
        o->incremental_encoder_resolution_cpt = actuator_parameters.get_scalar<_int>("incremental_encoder_resolution_cpt").value;
        o->absolute_encoder_offset_counts = actuator_parameters.get_scalar<_int16_t>("absolute_encoder_offset_counts").value;
        o->loadcell_sign = actuator_parameters.get_scalar<sign_t>("loadcell_sign").value;
        o->absolute_encoder_sign = actuator_parameters.get_scalar<sign_t>("absolute_encoder_sign").value;
        o->incremental_encoder_sign = actuator_parameters.get_scalar<sign_t>("incremental_encoder_sign").value;
        o->desired_torque_sign = actuator_parameters.get_scalar<sign_t>("desired_torque_sign").value;
        o->use_incremental_encoder = actuator_parameters.get_scalar<selector_t>("use_incremental_encoder").value;
        o->linear_actuator_offset_mV = actuator_parameters.get_scalar<_float>("linear_actuator_offset_mV").value;
        o->linear_actuator_calibration_mV_to_mm = actuator_parameters.get_scalar<_float>("linear_actuator_calibration_mV_to_mm").value;
        o->escon_analog_output0_voltage_V_to_current_A_slope = actuator_parameters.get_scalar<_float>("escon_analog_output0_voltage_V_to_current_A_slope").value;
        o->escon_analog_output0_voltage_V_to_current_A_offset= actuator_parameters.get_scalar<_float>("escon_analog_output0_voltage_V_to_current_A_offset").value;
        o->escon_analog_output1_velocity_V_to_current_rpm_offset = actuator_parameters.get_scalar<_float>("escon_analog_output1_velocity_V_to_current_rpm_offset").value;
        o->escon_analog_output1_velocity_V_to_current_rpm_slope= actuator_parameters.get_scalar<_float>("escon_analog_output1_velocity_V_to_current_rpm_slope").value;

        } catch (config_file_exception  exception)
        {
            if(static_cast<int>(exception) < 0)
            {
                error = exception;
                return error;
            }
        }
    float setpoint_to_mNm = o->current_conversion_factor_mA_to_setpoint*o->torque_constant_mNm_per_mA;
    try {
        json_object controller_parameters = onedof_parameters.get_object("sea_controller_parameters");

        c->torque_control_p_gain = (controller_parameters.get_scalar<_float>("torque_control_p_gain").value) * setpoint_to_mNm * 0.001;
        c->torque_control_i_gain = (controller_parameters.get_scalar<_float>("torque_control_i_gain").value) * setpoint_to_mNm * 0.001;
        c->torque_control_d_gain = (controller_parameters.get_scalar<_float>("torque_control_d_gain").value) * setpoint_to_mNm * 0.001;
        c->position_control_p_gain = controller_parameters.get_scalar<_float>("position_control_p_gain").value;
        c->position_control_i_gain = controller_parameters.get_scalar<_float>("position_control_i_gain").value;
        c->position_control_d_gain = controller_parameters.get_scalar<_float>("position_control_d_gain").value;

        c->impedance_control_k_gain_mNm_per_rad = (controller_parameters.get_scalar<_float>("impedance_control_k_gain_mNm_per_rad").value)*1000;
        c->impedance_control_d_gain_mNm_per_rad_per_sec = (controller_parameters.get_scalar<_float>("impedance_control_d_gain_mNm_per_rad_per_sec").value)*1000;
        c->impedance_control_max_error_radians=controller_parameters.get_scalar<_float>("impedance_control_max_error_radians").value;
        c->friction_comp_max_torque_mNm =controller_parameters.get_scalar<_float>("friction_comp_max_torque_mNm").value;
        c->friction_torque_threshold_rad_per_s=controller_parameters.get_scalar<_float>("friction_torque_threshold_rad_per_s").value;
        c->soft_to_hard_stop_offset_deg=controller_parameters.get_scalar<_float>("soft_to_hard_stop_offset_deg").value;
        c->soft_stop_max_torque_mNm=controller_parameters.get_scalar<_float>("soft_stop_max_torque_mNm").value;
        c->max_velocity_threshold_rad_per_sec=controller_parameters.get_scalar<_float>("max_velocity_threshold_rad_per_sec").value;

        c->max_torque_control_input_mNm =controller_parameters.get_scalar<_float>("max_torque_control_input_mNm").value;
        c->min_torque_control_input_mNm =controller_parameters.get_scalar<_float>("min_torque_control_input_mNm").value;

        c->velocity_low_pass_filter_weight_for_current_measure =controller_parameters.get_scalar<_float>("velocity_low_pass_filter_weight_for_current_measure").value;
        c->loadcell_low_pass_filter_weight_for_current_measure = controller_parameters.get_scalar<_float>("loadcell_low_pass_filter_weight_for_current_measure").value;
        c->gain_inertia_dynamics_compensation = controller_parameters.get_scalar<_float>("gain_inertia_dynamics_compensation").value;
        c->max_torque_change_mNm_per_ms = controller_parameters.get_scalar<_float>("max_torque_change_mNm_per_ms").value;
        c->max_integrated_torque_error_mNm = controller_parameters.get_scalar<_float>("max_integrated_torque_error_mNm").value;
        c->max_allowable_redundancy_error_for_motor_current_mA = controller_parameters.get_scalar<_float>("max_allowable_redundancy_error_for_motor_current_mA").value;
        c->max_allowable_redundancy_error_for_motor_velocity_rad_per_sec = controller_parameters.get_scalar<_float>("max_allowable_redundancy_error_for_motor_velocity_rad_per_sec").value;

    } catch (config_file_exception  exception)
    {
        if(static_cast<int>(exception) < 0)
        {
            error = exception;
            return error;
        }
    }

    return no_error;
}
