/** @file
 * @brief Contains definitions of functions used for the Series-Elastic Actuator (SEA) Driver
*/

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "agree_joint_controller.h"

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/

agree_joint_controller::agree_joint_controller()
{
    //    std::cout <<"EsmaCAT SEA Driver Object is created by interiting Motor Driver Class" << std::endl;

    ethercat_interface.esmacat_slave_vendor_id = SEA_DRIVER_VENDOR_ID;
    ethercat_interface.esmacat_slave_product_id = SEA_DRIVER_PRODUCT_ID;

    loadcell_init_val_mNm = 0;
    absolute_encoder_init_val_rad = 0;
    incremental_encoder_init_val_rad = 0;

    prev_position_radians = 0;
    prev_torque_error_mNm = 0;
    prev_position_error_rad = 0;
    prev_friction_comp_torque_mNm = 0;

    filtered_velocity_radians_per_sec = 0;
    total_filtered_demanded_motor_torque_mNm = 0;
    filtered_deriv_torque_error = 0;

}

esmacat_err agree_joint_controller::update_joint_variables(double elapsed_time_ms, uint64_t loop_cnt)
{
    // update variables from sensors
    esmacat_err loadcell_error;

    joint_values.joint_index                                    = get_joint_index();
    joint_values.escon_fault                                    = ethercat_interface.get_escon_fault();

    joint_values.loadcell_reading_mNm                           = load_reader.get_load_mNm(loadcell_error, &ethercat_interface);
    joint_values.filtered_load_mNm                              = load_reader.get_filtered_load_mNm(loadcell_error, &ethercat_interface);

    joint_values.unfiltered_incremental_encoder_reading_cpt     = position_reader.get_signed_raw_incremental_encoder_reading_cpt(&ethercat_interface);
    joint_values.incremental_encoder_reading_radians            = position_reader.get_incremental_encoder_reading_radians(&ethercat_interface, joint_config.gear_ratio);
    joint_values.incremental_encoder_reading_degrees            = joint_values.incremental_encoder_reading_radians*static_cast<float>(180/M_PI);

    joint_values.incremental_encoder_speed_radians_sec          = get_incremental_encoder_speed_rad_s();
//    joint_values.incremental_encoder_speed_radians_sec_driver   = get_incremental_encoder_speed_rad_s_from_driver();


    //    joint_values.incremental_encoder_reading_cpt= position_reader.get_incremental_encoder_reading_cpt(&ethercat_interface);
    //    joint_values.signed_raw_absolute_encoder_reading_cpt = position_reader.get_signed_raw_absolute_encoder_reading_cpt(&controller_interface);
    //    joint_values.unfiltered_absolute_encoder_reading_cpt=position_reader.get_unfiltered_absolute_encoder_reading_cpt(&controller_interface);
    //    joint_values.filtered_absolute_encoder_reading_cpt=position_reader.get_filtered_absolute_encoder_reading_cpt(&controller_interface);
    //    joint_values.filtered_absolute_encoder_reading_radians = position_reader.get_filtered_absolute_encoder_reading_radians(&controller_interface);
    //    joint_values.filtered_absolute_encoder_reading_degrees = position_reader.get_filtered_absolute_encoder_reading_radians(&controller_interface)*static_cast<float>(180/M_PI);
    //    joint_values.linear_actuator_length_mm = length_reader.get_linear_actuator_length_mm(linear_actuator_error, &controller_interface);

    //    Setting incremental encoder offset from absolute encoder
    //    incremental_encoder_calibration(static_cast<float>(elapsed_time_ms),false);

    // give the loop cnt to controller interface and slave
    ethercat_interface.set_current_loop_cnt(loop_cnt);

    // read the feedback values sequentially.
    // ethercat_interface.read_feedback_variables_sequentially(loop_cnt,false);

    if (loadcell_error != NO_ERR)
    {
        PLOGE << "Error while updating joint variables of load cell";
        return loadcell_error;
    }
    else
    {
        return NO_ERR;
    }

}

esmacat_err agree_joint_controller::set_joint_controller_parameters_and_configure_interface(sea_joint_configuration_t *joint_config, sea_controller_configuration_t *control_config)
{
    set_joint_hardware_parameters(joint_config);
    set_joint_controller_parameters(control_config);
    configure_actuator_controller_interface();
    return NO_ERR;
}

esmacat_err agree_joint_controller::configure_joint_controller_from_file(std::string filename)
{
    import_actuator_parameters_from_file( &controller_config, &joint_config, filename);
    set_joint_controller_parameters_and_configure_interface(&joint_config, &controller_config);
    return NO_ERR;
}

/** @brief Sets the configuration of the joint from the input configuration
 * parameters
 *
 * @param op Struct containing all the input configuration parameters
 * @return Error status of the function
 */
esmacat_err agree_joint_controller::set_joint_hardware_parameters(sea_joint_configuration_t* op)
{
    joint_config = *op;

    load_reader.set_loadcell_calibration(op->loadcell_calibration_mV_to_mNm, &ethercat_interface);
    load_reader.set_loadcell_zero_offset(op->loadcell_offset_mV, &ethercat_interface);
    ethercat_interface.set_current_conversion_factor(op-> current_conversion_factor_A_to_setpoint);
    position_reader.set_incremental_encoder_resolution_cpt(op->incremental_encoder_resolution_cpt);
    position_reader.set_absolute_encoder_offset_counts(op->absolute_encoder_offset_counts);
    position_reader.set_incremental_encoder_offset_counts(op->incremental_encoder_offset_counts);
    length_reader.set_linear_actuator_zero_offset(op->linear_actuator_offset_mV);
    length_reader.set_linear_actuator_calibration(op->linear_actuator_calibration_mV_to_mm);
    load_reader.set_loadcell_sign(op->loadcell_sign, &ethercat_interface);
    position_reader.set_absolute_encoder_sign(op->absolute_encoder_sign);
    position_reader.set_incremental_encoder_sign(op->incremental_encoder_sign);

    return NO_ERR;
}

/** @brief Obtains the joint index set in the configuration
 * @return Joint Index
 */
int agree_joint_controller::get_joint_index()
{
    return joint_config.joint_index;
}

/** @brief Computes all the values for the joint from the sensor readings and sets the
 * values in the struct
 *
 * @return Joint readings in the form of sea_joint_vals_t struct
 */
joint_values_t agree_joint_controller::get_joint_vals()
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

void agree_joint_controller::test_sign_values(float max_setpoint,float test_time_ms, float elapsed_time_ms)
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
        loadcell_init_val_mNm = load_reader.get_filtered_load_mNm(loadcell_error, &ethercat_interface);
        absolute_encoder_init_val_rad = position_reader.get_filtered_absolute_encoder_reading_radians(&ethercat_interface);
        incremental_encoder_init_val_rad = position_reader.get_incremental_encoder_reading_radians(&ethercat_interface, joint_config.gear_ratio);
    }
    else
    {
        //the test is executed 1s after the application begins execution, and runs till test_time_ms
        float time_scale = std::min(static_cast<float>(1),std::max(static_cast<float>(0), (elapsed_time_ms-1000)/test_time_ms));

        cout << "a1";
        ethercat_interface.set_control_setpoint_0to1(max_setpoint*time_scale*joint_config.desired_torque_sign);
        ethercat_interface.set_escon_enable(1);

        float loadcell_curr_val = load_reader.get_filtered_load_mNm(loadcell_error, &ethercat_interface);
        float absolute_encoder_curr_val = position_reader.get_filtered_absolute_encoder_reading_radians(&ethercat_interface);
        float incremental_encoder_curr_val = position_reader.get_incremental_encoder_reading_radians(&ethercat_interface, joint_config.gear_ratio);

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
torque_control_terms_t agree_joint_controller::get_torque_control_terms()
{
    return torque_control_loop_status;
}

/** @brief Returns the terms associated with impedance control
 * @return Struct containing all the terms associated with the impedance control loop
 */
impedance_control_terms_t agree_joint_controller::get_impedance_control_terms()
{
    return joint_impedance_control_terms;
}

esmacat_err agree_joint_controller::control_torque_with_soft_stop_mNm(float torque_setpoint_mNm, float elapsed_time_ms)
{
    esmacat_err error = NO_ERR;

    const float delta           = controller_config.soft_to_hard_stop_offset_deg;
    const float push_max_torque = controller_config.soft_stop_max_torque_mNm;

    float p         = joint_values.incremental_encoder_reading_degrees ;
    float p_up      = joint_config.hard_stop_upper_limit_degrees;
    float p_low     = joint_config.hard_stop_lower_limit_degrees;

    // Upper Limits
    float dist_to_uppper_lim = (p_up - p);
    if (dist_to_uppper_lim < 0.0 )
    {
        dist_to_uppper_lim = 0.0;
        //PLOGE << "ERR_MEASURED_POSITION_OUTSIDE_RANGE";
//        ethercat_interface.set_escon_enable(false);
        return ERR_MEASURED_POSITION_OUTSIDE_RANGE;
    }
        if (dist_to_uppper_lim > delta) dist_to_uppper_lim = delta;
    float t_push_down = push_max_torque * (delta - dist_to_uppper_lim)/delta;

    // Lower Limits
    float dist_to_lower_lim = (p - p_low);
    if (dist_to_lower_lim < 0.0 )
    {
        dist_to_lower_lim = 0.0;
        //PLOGE << "ERR_MEASURED_POSITION_OUTSIDE_RANGE";
//        ethercat_interface.set_escon_enable(false);        // TODO: stop motor or just add soft stop when rom is exceeded?
        return ERR_MEASURED_POSITION_OUTSIDE_RANGE;
    }
        if (dist_to_lower_lim > delta) dist_to_lower_lim = delta;
    float t_push_up = push_max_torque * (delta - dist_to_lower_lim)/delta;

    joint_impedance_control_terms.soft_stop_torque_mNm = t_push_up-t_push_down;

    error = control_torque_mNm(torque_setpoint_mNm - t_push_down + t_push_up,elapsed_time_ms );

    return error;
}

esmacat_err agree_joint_controller::control_torque_with_aux_input_mNm(float torque_setpoint_mNm, float aux_input, float elapsed_time_ms)
{
    ethercat_interface.set_aux_setpoint(aux_input);
    control_torque_mNm(torque_setpoint_mNm,elapsed_time_ms );

    return NO_ERR;
}

esmacat_err agree_joint_controller::control_torque_mNm(float torque_setpoint_mNm, float elapsed_time_ms)
{
    esmacat_err error = NO_ERR;

    // Correct for transmission ratio
    torque_setpoint_mNm = torque_setpoint_mNm/joint_config.transmission_ratio;

    if ( get_current_control_mode() != actuator_controller_interface::control_mode_t::torque_control) // if wrong control mode is selected
    {
        set_motor_derating_factor(0);
        ethercat_interface.set_escon_enable(false);
        ethercat_interface.set_control_setpoint_0to1(0.0);
        return ERR_SEA_WRONG_CONTROL_MODE_SELECTED;
    }
    else{

        ethercat_interface.set_escon_enable(true);

    if ( torque_setpoint_mNm > controller_config.max_torque_control_input_mNm  ) // if the control input is larger than its allowed range
    {
        torque_setpoint_mNm = controller_config.max_torque_control_input_mNm;
        error = ERR_CONTROLLER_MAX_TORQUE_SETPOINT_OUT_OF_RANGE;

    }
    if ( torque_setpoint_mNm < controller_config.min_torque_control_input_mNm  ) // if the control input is larger than its allowed range
    {
        torque_setpoint_mNm = controller_config.min_torque_control_input_mNm;
        error = ERR_CONTROLLER_MIN_TORQUE_SETPOINT_OUT_OF_RANGE;
    }

    // set derating factor
    float derating_factor = 0.0;
    if (derating_param_time_to_arrive_75p_in_msec < 1e-5) derating_factor = 1.0;
    else derating_factor = smooth_start_func_tanh(elapsed_time_ms/derating_param_time_to_arrive_75p_in_msec);
    set_motor_derating_factor( derating_factor );

    joint_values.torque_setpoint_mNm = torque_setpoint_mNm;
    // Run inner torque control
    ethercat_interface.set_control_setpoint(torque_setpoint_mNm);

    // Return error
    return error;
    }
}

esmacat_err agree_joint_controller::control_ESCON_directly_0to1(float setpoint_0to1)
{
    if ( get_current_control_mode() != actuator_controller_interface::control_mode_t::direct_escon_control )
    {
        setpoint_0to1 = 0.0;
        PLOGE << "direct_escon_control function was colled wihtout selecting the mode, DIRECT_ESCON_CONTROL ";
        ethercat_interface.set_escon_enable(false);
        return ERR_SEA_WRONG_CONTROL_MODE_SELECTED;
    }
    else{
        if (setpoint_0to1 > 1.0)
        {
            setpoint_0to1 = 1.0;
            PLOGW << "direct_escon_control function attempted to send a setpoint higher than 1.0";
        }
        if (setpoint_0to1 < -1.0)
        {
            setpoint_0to1 = -1.0;
            PLOGW << "direct_escon_control function attempted to send a setpoint lower than -1.0";
        }

        ethercat_interface.set_escon_enable(true);
        ethercat_interface.set_control_setpoint_0to1(setpoint_0to1);
        return  NO_ERR;
    }

}

esmacat_err agree_joint_controller::control_torque_directly_mA(float setpoint_mA, float elapsed_time_ms){

    esmacat_err error = NO_ERR;
    // convert mA to setpoint (current_conversion_factor is computed with 5A = 90% PWM)
    float setpoint_0to1 = setpoint_mA/(joint_config.current_conversion_factor_A_to_setpoint*1000);

    // set derating factor
    float derating_factor = 0.0;
    if (derating_param_time_to_arrive_75p_in_msec < 1e-5) derating_factor = 1.0;
    else derating_factor = smooth_start_func_tanh(elapsed_time_ms/derating_param_time_to_arrive_75p_in_msec);

    set_motor_derating_factor( derating_factor );
    error = control_ESCON_directly_0to1(setpoint_0to1);

    return error;
}

esmacat_err agree_joint_controller::control_torque_directly_mNm(float setpoint_mNm, float elapsed_time_ms){

    esmacat_err error = NO_ERR;
    float setpoint_0to1 = (setpoint_mNm/(joint_config.torque_constant_mNm_per_A/1000)/joint_config.gear_ratio) /(joint_config.current_conversion_factor_A_to_setpoint*1000);

    // set derating factor
    float derating_factor = 0.0;
    if (derating_param_time_to_arrive_75p_in_msec < 1e-5) derating_factor = 1.0;
    else derating_factor = smooth_start_func_tanh(elapsed_time_ms/derating_param_time_to_arrive_75p_in_msec);

    set_motor_derating_factor(derating_factor);
    error = control_ESCON_directly_0to1(setpoint_0to1);
    return error;
};

esmacat_err agree_joint_controller::control_torque_external_mNm(double torque_setpoint_mNm, double elapsed_time_ms){

    esmacat_err error = NO_ERR;
    float actual_loadcell_torque_mNm = get_loadcell_torque_mNm();

    //  Low pass filter for desired torque
        if ( (torque_setpoint_mNm - total_filtered_demanded_motor_torque_mNm) >  50.0 )
        {
            total_filtered_demanded_motor_torque_mNm = total_filtered_demanded_motor_torque_mNm + 50.0;
        }
        else if ( (torque_setpoint_mNm - total_filtered_demanded_motor_torque_mNm) <  -50.0)
        {
            total_filtered_demanded_motor_torque_mNm = total_filtered_demanded_motor_torque_mNm - 50.0;
        }
        else
        {
            total_filtered_demanded_motor_torque_mNm = torque_setpoint_mNm ;
        }

    /** *********************************/
    /** Feedforward torque control loop */
    /** *********************************/

    // Feedforward compensation for gearbox friction.
    double feedforward_friction_compensation_mNm = 0;
    double coulomb_torque_mNm = 0.0;
    double coulomb_velocity_rad_s = 1E-1;
    double viscous_friction_coeff = 200.0;
    feedforward_friction_compensation_mNm = 0; // coulomb_torque_mNm*tanh(joint_values.incremental_encoder_speed_radians_sec/coulomb_velocity_rad_s)   +
                                            //viscous_friction_coeff*joint_values.incremental_encoder_speed_radians_sec;
    //TODO: Add friction compensatino in external control torque loop

    // Feedforward vibration to reduce stiction effect
    double feedforward_vibration = sin(2*3.1415*elapsed_time_ms*msec_to_sec*controller_config.friction_comp_vibration_frequency)*controller_config.friction_comp_vibration_amplitude;

    // Feedforward compensation for gearbox inefficiency
    float feedforward_demanded_motor_torque_mNm = (total_filtered_demanded_motor_torque_mNm+feedforward_friction_compensation_mNm+feedforward_vibration)/joint_config.gear_power_efficiency;

    /** ******************************/
    /** Feedback torque control loop */
    /** ******************************/

    // Proportional Error
    float torque_error_mNm =  total_filtered_demanded_motor_torque_mNm - actual_loadcell_torque_mNm;

    if(torque_error_mNm > 500) torque_error_mNm = 500; //max proportional torque error
    else if(torque_error_mNm < -500) torque_error_mNm = -500;
    // Proportional Term
    float feedback_p_torque_mNm = torque_control_p_gain * torque_error_mNm;

    // Derivative Error
    float deriv_loadcell_error_mNm = ( - prev_torque_error_mNm)/ethercat_interface.get_esmacat_app_one_cycle_time_sec();
    prev_torque_error_mNm = torque_error_mNm;

    // Filtered Derivative Error (Exponential Low-Pass Filter)
    // y(k) = (1-a) * y(k-1) + a * x(k)
    float lambda = 0.05;
    filtered_deriv_torque_error = lambda*deriv_loadcell_error_mNm+(1-lambda)*filtered_deriv_torque_error;

    // Derivative Term
    float feedback_d_torque_mNm = torque_control_d_gain * filtered_deriv_torque_error;

    // Integral Error
    float max_integ_torque_error_mNm = 5000;
    integ_torque_error += torque_error_mNm*ethercat_interface.get_esmacat_app_one_cycle_time_sec();
    if(integ_torque_error > max_integ_torque_error_mNm) integ_torque_error = max_integ_torque_error_mNm; //max integral torque error
    else if(integ_torque_error < -max_integ_torque_error_mNm) integ_torque_error = -max_integ_torque_error_mNm;

    // Integral Term
    float feedback_i_torque_mNm =  torque_control_i_gain * integ_torque_error;

    // Total Feedback Torque
    float feedback_demanded_motor_torque_mNm = feedback_p_torque_mNm + feedback_i_torque_mNm + feedback_d_torque_mNm;

    /** Summing, filtering, and passing setpoint */

    float total_demanded_motor_torque_mNm = feedback_demanded_motor_torque_mNm + feedforward_demanded_motor_torque_mNm;

    // Low-pass Filter the setpoint to avoid peaks
//    filtered_setpoint = 0.9*filtered_setpoint + 0.1*setpoint;
    total_filtered_demanded_motor_torque_mNm = 0.0*total_filtered_demanded_motor_torque_mNm + 1.0*total_demanded_motor_torque_mNm;

    // Set current setpoint
    error = control_torque_directly_mNm(total_filtered_demanded_motor_torque_mNm,elapsed_time_ms);

    // Save torque control status
    torque_control_loop_status.torque_setpoint_mNm              = torque_setpoint_mNm;
    torque_control_loop_status.torque_error_mNm                 = torque_error_mNm;
    torque_control_loop_status.torque_feedback_p_mNm            = feedback_p_torque_mNm;
    torque_control_loop_status.torque_feedback_i_mNm            = feedback_i_torque_mNm;
    torque_control_loop_status.torque_feedback_d_mNm            = feedback_d_torque_mNm;
    torque_control_loop_status.torque_feedforward_demanded_mNm  = feedforward_demanded_motor_torque_mNm;
    torque_control_loop_status.torque_feedback_demanded_mNm     = feedback_demanded_motor_torque_mNm;
    torque_control_loop_status.torque_total_demanded_mNm        = total_filtered_demanded_motor_torque_mNm;
    return error;
}

void agree_joint_controller::set_torque_gain(double p, double d, double i){
    torque_control_p_gain = p;
    torque_control_d_gain = d;
    torque_control_i_gain = i;
}

void agree_joint_controller::set_speed_gain(float p, float d, float i){
    speed_control_p_gain = p;
    speed_control_d_gain = d;
    speed_control_i_gain = i;
}

esmacat_err agree_joint_controller::control_speed_rad_per_s(float setpoint_rad_per_sec, float elapsed_time_ms){

    esmacat_err error = NO_ERR;

    // Proportional error
    float speed_error_rad = setpoint_rad_per_sec - joint_values.incremental_encoder_speed_radians_sec;

    // Derivative error
    float deriv_speed_error = (speed_error_rad - prev_speed_error_rad_s)/ethercat_interface.get_esmacat_app_one_cycle_time_sec();
    prev_speed_error_rad_s = speed_error_rad;

    // Filter derivative error
    float lambda = static_cast<float>(0.5);
    filtered_deriv_speed_error = lambda*deriv_speed_error + (1-lambda)*filtered_deriv_speed_error;

    // Integral error
    integ_speed_error += speed_error_rad*ethercat_interface.get_esmacat_app_one_cycle_time_sec();

    float setpoint_p = speed_control_p_gain*speed_error_rad;
    float setpoint_d = speed_control_d_gain*filtered_deriv_speed_error;//*motor_factor;
    float setpoint_i = speed_control_i_gain*integ_speed_error;//*motor_factor;

//    if(std::abs(setpoint_rad_per_sec) >= controller_config.max_velocity_threshold_rad_per_sec)
//    {
//        ethercat_interface.set_escon_enable(false);
//        error = ERR_CONTROLLER_MAX_SPEED_SETPOINT_OUT_OF_RANGE;
//        return error;
//    }
//    else if (std::abs(joint_values.filtered_load_mNm) >= controller_config.max_torque_control_input_mNm)
//    {
//        ethercat_interface.set_escon_enable(true);
//        error = ERR_MEASURED_LOAD_OUTSIDE_RANGE;
//        return error;
//    }
//    else
//    {
        ethercat_interface.set_escon_enable(true);
//    }

    //set_control_mode(actuator_controller_interface::control_mode_t::direct_escon_control);
    error = control_torque_mNm(setpoint_p + setpoint_d + setpoint_i, elapsed_time_ms);
    return error;
}

void agree_joint_controller::reset_encoder_values(){

    prev_position_radians = 0;
    filtered_velocity_radians_per_sec = 0;
    velocity_loop_flag = 0;
}

/** @brief Homing control loop
 *
 *
 *
 */

esmacat_err agree_joint_controller::homing_control(float elapsed_time_ms){

    esmacat_err error = NO_ERR;

    double support_trajectory, support_torque;

    switch(calibration_status)
    {

    case 100: // Homing called >> Set offset values
        impedance_control_offset_time_ms    = elapsed_time_ms;
        impedance_control_offset_pos_rad    = joint_values.incremental_encoder_reading_radians;
        impedance_control_offset_torque     = joint_values.filtered_load_mNm;
        PLOGW << "Home called: " << impedance_control_offset_pos_rad*rad_to_deg << " deg at " << impedance_control_offset_time_ms << " ms.";
        // Go to next step
        calibration_status = 101;
        break;

    case 101: // Homing called >> Start homing
        // Run outer impedance control
        error = impedance_control(impedance_control_offset_pos_rad,
                                  0.0,
                                  static_cast<float>((elapsed_time_ms)),
                                  false);
        if((elapsed_time_ms-impedance_control_offset_time_ms)>500){ // 500ms of starting

            incremental_encoder_calibration_counter = 0;
            impedance_control_offset_time_ms = elapsed_time_ms;
            // Debug
            PLOGW << "Home started: " << impedance_control_offset_pos_rad*rad_to_deg << " deg at " << (elapsed_time_ms-impedance_control_offset_time_ms) << " ms.";
            // Go to next step
            calibration_status = 102;
        }
        break;

    case 102: // Homing not done -> Moving toward endstop

        PLOGW << "DOING HOMING...";
        support_trajectory = impedance_control_offset_pos_rad+joint_config.calibration_sign*(elapsed_time_ms-impedance_control_offset_time_ms)/1000*0.1;

        //TODO: Check
        support_torque     = 0.0; //joint_values.loadcell_reading_mNm/(1+(elapsed_time_ms-impedance_control_offset_time_ms)/500.0);

        // Run Outer Impedance Control
        error = impedance_control(static_cast<float>(support_trajectory),
                                  static_cast<float>(support_torque),
                                  static_cast<float>(elapsed_time_ms),
                                  false);
        // Reached boundary position
        if(abs(joint_values.filtered_load_mNm)>=joint_config.calibration_threshold_torque){ //TODO: Add speed contraint && abs(joint_values.incremental_encoder_speed_radians_sec) < 1){
            incremental_encoder_calibration_counter++;
            PLOGW << "BOUNDARY REACHED.";

        }

        if(incremental_encoder_calibration_counter>250){
            calibration_status = 103;
            PLOGW << "CHANGE STATUS TO 103.";

        }
        break;

    case 103: // Endstop reached -> Start going back from endstop
        impedance_control_offset_time_ms = elapsed_time_ms;
        impedance_control_offset_pos_rad = 0;
//        impedance_control_offset_rad = 0.0;
        PLOGW << "Endstop reached: " << joint_values.filtered_load_mNm << " mNm " << impedance_control_offset_pos_rad*rad_to_deg << " deg at " << impedance_control_offset_time_ms << " ms.";
//        PLOGW << position_reader.get_incremental_encoder_reading_cpt(&ethercat_interface);
        position_reader.set_incremental_encoder_offset_counts(position_reader.get_signed_raw_incremental_encoder_reading_cpt(&ethercat_interface));
        reset_encoder_values();

        calibration_status = 104;
        PLOGE << joint_config.calibration_offset_rad/180.0*M_PI << " - " << joint_values.incremental_encoder_reading_radians;
        break;

    case 104: // Endstop reached -> Moving back from endstop
        PLOGW << "MOVING BACK FROM ENDSTOP...";

        // Final position received from shared-memory setpoint
        support_trajectory = (joint_config.calibration_offset_rad/180.0*M_PI)/2.0*(tanh((elapsed_time_ms-impedance_control_offset_time_ms)/1000.0-exp(1.0))+1.0);

        support_torque = joint_values.loadcell_reading_mNm/(1+(elapsed_time_ms-impedance_control_offset_time_ms)/500.0);
        // Run Outer Impedance Control
        error = impedance_control(static_cast<float>(support_trajectory),
                                  static_cast<float>(support_torque),
                                  static_cast<float>((elapsed_time_ms)),
                                  false);

        if(abs(joint_config.calibration_offset_rad/180.0*M_PI - joint_values.incremental_encoder_reading_radians)<=M_PI/32.0)
        {
            calibration_status = 105;
            PLOGW << "CHANGE STATUS TO 105.";

        }
         break;

    case 105: // Home reached.
        ethercat_interface.set_escon_enable(false);
        impedance_control_offset_time_ms = elapsed_time_ms;
        impedance_control_offset_pos_rad = joint_values.incremental_encoder_reading_radians;
        calibration_status = 106;
        PLOGW << "Home reached: " << joint_values.filtered_load_mNm << " mNm " << impedance_control_offset_pos_rad*rad_to_deg << " deg at " << impedance_control_offset_time_ms << " ms.";
        PLOGW << position_reader.get_incremental_encoder_reading_cpt(&ethercat_interface);
        position_reader.set_incremental_encoder_offset_counts(position_reader.get_signed_raw_incremental_encoder_reading_cpt(&ethercat_interface));
        calibration_reference_counts = position_reader.get_signed_raw_incremental_encoder_reading_cpt(&ethercat_interface);
        reset_encoder_values();
        break;

    case 106:
        joint_is_referenced = true;
        support_torque = joint_values.loadcell_reading_mNm/(1+(elapsed_time_ms-impedance_control_offset_time_ms)/50.0);
        //error = impedance_control(0.0, support_torque,static_cast<float>(elapsed_time_ms),true);
        break;

    default:
        break;
    }

    return error;

}

void agree_joint_controller::calibrate_incremental_encoder(){

    // Read raw incremental encoder in counts -> actual position is set to
//    calibration_reference_counts = position_reader.get_signed_raw_incremental_encoder_reading_cpt(&ethercat_interface);
    // Set incremental encoder offset with the previously read.
    position_reader.set_incremental_encoder_offset_counts(calibration_reference_counts);
    // Reset
    reset_encoder_values();
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
esmacat_err agree_joint_controller::impedance_control(float setpoint_radians,float gravity_torque_mNm, float elapsed_time_ms, bool soft_stop_enable)
{
    esmacat_err error = NO_ERR;

    float impedance_control_k_gain_mNm_per_rad          = controller_config.impedance_control_k_gain_mNm_per_rad;
    float impedance_control_d_gain_mNm_per_rad_per_sec  = controller_config.impedance_control_d_gain_mNm_per_rad_per_sec;
    float impedance_control_max_error_radians           = controller_config.impedance_control_max_error_radians;

    float friction_comp_max_torque_mNm                  = controller_config.friction_comp_max_torque_mNm;
    float friction_torque_threshold_rad_per_s           = controller_config.friction_torque_threshold_rad_per_s;
    float friction_comp_viscous_coeff                   = controller_config.friction_comp_viscous_coeff;


    float max_velocity_threshold_rad_per_sec            = controller_config.max_velocity_threshold_rad_per_sec;
    float max_load_stop_threshold_mNm                   = controller_config.soft_stop_max_torque_mNm;

    float kterm_mNm =0;
    float dterm_mNm =0;
    float controller_torque_mNm =0;
    float error_radians = 0;
    float torque_setpoint_mNm = 0;
    float friction_comp_torque_mNm = 0;
    float soft_stop_torque_mNm = 0;


    double measured_position_radians = 0;

    measured_position_radians = joint_values.incremental_encoder_reading_radians; // position_reader.get_incremental_encoder_reading_radians(&ethercat_interface,joint_config.gear_ratio);
    error_radians = setpoint_radians- measured_position_radians;


    //Controller Torque
    // limit the error measured to between -max to +max
    error_radians = std::min(std::max(error_radians,-impedance_control_max_error_radians), impedance_control_max_error_radians);
    kterm_mNm = impedance_control_k_gain_mNm_per_rad * error_radians;
    dterm_mNm = -impedance_control_d_gain_mNm_per_rad_per_sec   *   joint_values.incremental_encoder_speed_radians_sec;
    controller_torque_mNm = kterm_mNm+dterm_mNm;

//    //Friction Torque
//    if (filtered_velocity_radians_per_sec > friction_torque_threshold_rad_per_s)
//    {
//        friction_comp_torque_mNm = -friction_comp_max_torque_mNm;
//    }
//    else if (filtered_velocity_radians_per_sec < -friction_torque_threshold_rad_per_s)
//    {
//        friction_comp_torque_mNm = friction_comp_max_torque_mNm;
//    }
//    else
//    {
//        friction_comp_torque_mNm = 0;
//    }
//    //filter torque
      friction_comp_torque_mNm = + friction_comp_viscous_coeff *   joint_values.incremental_encoder_speed_radians_sec;
//        static_cast<float>(0.4) * friction_comp_torque_mNm + static_cast<float>(0.6) * prev_friction_comp_torque_mNm;
//    prev_friction_comp_torque_mNm = friction_comp_torque_mNm;

    // Disable motor if velocity exceeds threshold
    if (std::abs(filtered_velocity_radians_per_sec) >= max_velocity_threshold_rad_per_sec)
    {
        ethercat_interface.set_escon_enable(false);
        error = ERR_MEASURED_VELOCITY_OUTSIDE_RANGE;
        std::cout<<"ERR_MEASURED_VELOCITY_OUTSIDE_RANGE. ESCON Disabled"<<std::endl;
        return error;
    }
    else
    {
        ethercat_interface.set_escon_enable(true);
    }

    // disable motor if load exceeds threshold
    esmacat_err loadcell_reading_status;
    float loadcell_reading_mNm = joint_values.filtered_load_mNm; // load_reader.get_filtered_load_mNm(loadcell_reading_status, &ethercat_interface);
    if (loadcell_reading_status == NO_ERR)
    {
        if (std::abs(loadcell_reading_mNm) >= max_load_stop_threshold_mNm)
        {
            ethercat_interface.set_escon_enable(false);
            error = ERR_MEASURED_LOAD_OUTSIDE_RANGE;
            PLOGE << "ERR_MEASURED_LOAD_OUTSIDE_RANGE. J" << joint_config.joint_index << " ESCON Disabled" <</*get_filtered_load_mNm()<<*/std::endl;
            return error;
        }
        else
        {
            ethercat_interface.set_escon_enable(true);
        }
    }
    else
    {
        error = loadcell_reading_status;
        return error;
    }

    //computing the torque setpoint and running torque control
    torque_setpoint_mNm = gravity_torque_mNm + controller_torque_mNm + friction_comp_torque_mNm + soft_stop_torque_mNm;

    // Run inner torque control with soft-stop
    if(soft_stop_enable)
    {
        error = control_torque_with_soft_stop_mNm(torque_setpoint_mNm, elapsed_time_ms);
    }
    // Run inner torque control without soft-stop
    else
    {
        error = control_torque_mNm(torque_setpoint_mNm,elapsed_time_ms);
    }

    //saving values into the struct
    joint_impedance_control_terms.torque_setpoint_mNm                   = torque_setpoint_mNm;
    joint_impedance_control_terms.impedance_control_setpoint_rad        = setpoint_radians;
    joint_impedance_control_terms.error_radians                         = error_radians;
    joint_impedance_control_terms.impedance_control_torque_mNm          = torque_setpoint_mNm;
    joint_impedance_control_terms.gravity_torque_mNm                    = gravity_torque_mNm;
    joint_impedance_control_terms.friction_comp_torque_mNm              = friction_comp_torque_mNm;
    joint_impedance_control_terms.soft_stop_torque_mNm                  = soft_stop_torque_mNm;
    joint_impedance_control_terms.impedance_control_k_mNm_rad           = impedance_control_k_gain_mNm_per_rad;
    joint_impedance_control_terms.impedance_control_d_mNm_rad_per_sec   = impedance_control_d_gain_mNm_per_rad_per_sec;
    joint_impedance_control_terms.impedance_control_first_run           = false;

    // Return error
    return error;
}

/** @brief Reads the input configuration for the joint controller and sets the internal
 * variables
 *
 * @param configuration Input controller configuration parameters for the joint
 * @return Status of the function; NO_ERR indicates successful execution
*/
esmacat_err agree_joint_controller::set_joint_controller_parameters(sea_controller_configuration_t* configuration)
{
    controller_config = *configuration;
    return NO_ERR;
}

esmacat_err agree_joint_controller::configure_actuator_controller_interface()
{
    ethercat_interface.configure_gear_ratio(joint_config.gear_ratio);
    ethercat_interface.configure_gear_power_efficiency(joint_config.gear_power_efficiency);
    ethercat_interface.configure_torque_constant(joint_config.torque_constant_mNm_per_A);
    ethercat_interface.configure_escon_setpoint_sign(joint_config.desired_torque_sign);
    ethercat_interface.configure_incremental_encoder_resolution_cpt( joint_config.incremental_encoder_resolution_cpt );
    ethercat_interface.configure_escon_analog_output0_voltage_V_to_current_A_offset(joint_config.escon_analog_output0_voltage_V_to_current_A_offset);
    ethercat_interface.configure_escon_analog_output0_voltage_V_to_current_A_slope(joint_config.escon_analog_output0_voltage_V_to_current_A_slope);
    ethercat_interface.configure_escon_analog_output1_velocity_V_to_current_rpm_offset(joint_config.escon_analog_output1_velocity_V_to_current_rpm_offset);
    ethercat_interface.configure_escon_analog_output1_velocity_V_to_current_rpm_slope(joint_config.escon_analog_output1_velocity_V_to_current_rpm_slope);

//    controller_interface.configure_motor_rotor_inertia_g_per_cm2( joint_config.motor_rotor_inertia_g_per_cm2 );
//    controller_interface.configure_gearhead_rotor_inertia_g_per_cm2( joint_config.gearhead_rotor_inertia_g_per_cm2 );

    ethercat_interface.configure_max_torque_change_mNm_per_ms(controller_config.max_torque_change_mNm_per_ms);
    ethercat_interface.configure_torque_control_p_gain(controller_config.torque_control_p_gain);
    ethercat_interface.configure_torque_control_i_gain(controller_config.torque_control_i_gain);
    ethercat_interface.configure_torque_control_d_gain(controller_config.torque_control_d_gain);
    ethercat_interface.configure_control_type( (actuator_controller_interface::control_mode_t) controller_config.control_mode );
    ethercat_interface.configure_velocity_low_pass_filter_weight_for_current_measure( controller_config.velocity_low_pass_filter_weight_for_current_measure);
    ethercat_interface.configure_loadcell_low_pass_filter_weight_for_current_measure( controller_config.loadcell_low_pass_filter_weight_for_current_measure);
    ethercat_interface.configure_gain_inertia_dynamics_compensation( controller_config.gain_inertia_dynamics_compensation );
    ethercat_interface.configure_max_allowable_redundancy_error_for_motor_current_mA(controller_config.max_allowable_redundancy_error_for_motor_current_mA);
    ethercat_interface.configure_max_allowable_redundancy_error_for_motor_velocity_rad_per_sec(controller_config.max_allowable_redundancy_error_for_motor_velocity_rad_per_sec);

    return NO_ERR;
}

esmacat_err agree_joint_controller::configure_control_type(){
    ethercat_interface.configure_control_type( (actuator_controller_interface::control_mode_t) controller_config.control_mode );
    return NO_ERR;
}

esmacat_err agree_joint_controller::set_derating_param_time_to_arrive_75p_in_msec(float t)
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
void agree_joint_controller::set_impedance_control_K_mNm_per_rad(double input_mNm_per_rad)
{
    controller_config.impedance_control_k_gain_mNm_per_rad = std::max(0.0,std::min(1000*100.0,input_mNm_per_rad));
}

double agree_joint_controller::get_impedance_control_K_mNm_per_rad()
{
    return joint_impedance_control_terms.impedance_control_k_mNm_rad;
}

/** @brief Set the K term for impedance control
 */
void agree_joint_controller::set_impedance_control_d_mNm_per_rad_per_sec(double input_mNm_per_rad_per_sec)
{
    controller_config.impedance_control_d_gain_mNm_per_rad_per_sec = std::max(0.0,std::min(1000*100.0,input_mNm_per_rad_per_sec));
}
double agree_joint_controller::get_impedance_control_d_mNm_per_rad_per_sec()
{
    return joint_impedance_control_terms.impedance_control_d_mNm_rad_per_sec;
}

void agree_joint_controller::set_impedance_control_setpoint_rad(double setpoint_radians){
    joint_impedance_control_terms.impedance_control_setpoint_rad = setpoint_radians;
}

double agree_joint_controller::get_impedance_control_setpoint_rad()
{
    return joint_impedance_control_terms.impedance_control_setpoint_rad;
}

double agree_joint_controller::get_impedance_control_torque_mNm()
{
    return joint_impedance_control_terms.impedance_control_torque_mNm;
}

/** @brief Set the K term for impedance control
 */
void agree_joint_controller::set_joint_config_soft_stop(double lower, double upper)
{
    double temp_low,temp_upp;
    temp_low = joint_config.hard_stop_lower_limit_degrees;
    temp_upp = joint_config.hard_stop_upper_limit_degrees;
    joint_config.hard_stop_lower_limit_degrees = std::max(temp_low,std::min(temp_upp,lower));
    joint_config.hard_stop_upper_limit_degrees = std::min(temp_upp,std::max(temp_low,upper));
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



//void joint_controller::set_desired_torque_mNm(float torque_setpoint_mNm, float )
//{
//    joint_impedance_control_terms.torque_setpoint_mNm = torque_setpoint_mNm;
//    torque_control(torque_setpoint_mNm , elapsed_time_ms);
//}

/** Allows for a derating fraction between 0 and 1 to be set
 */
esmacat_err agree_joint_controller::set_motor_derating_factor(float factor)
{
    esmacat_err error = ethercat_interface.set_escon_derating_factor(factor);
    return error;
}


/** @brief Obtains incremental encoder readings in radians for the position reader */
float agree_joint_controller::get_incremental_encoder_reading_radians()
{
    float output = position_reader.get_incremental_encoder_reading_radians(&ethercat_interface,joint_config.gear_ratio);
    return output;
}

float agree_joint_controller::get_incremental_encoder_speed_rad_s(){

    float velocity_rad_per_sec;


    if (velocity_loop_flag == 0)
    {
        velocity_rad_per_sec = 0;
        filtered_velocity_radians_per_sec = velocity_rad_per_sec;
    }
    else
    {
        velocity_rad_per_sec =     velocity_rad_per_sec = (position_reader.get_incremental_encoder_reading_radians(&ethercat_interface,joint_config.gear_ratio) - prev_position_radians)/ethercat_interface.get_esmacat_app_one_cycle_time_sec();
    }

    velocity_loop_flag++;
    prev_position_radians = position_reader.get_incremental_encoder_reading_radians(&ethercat_interface,joint_config.gear_ratio);


    float lambda = controller_config.velocity_low_pass_filter_weight_for_current_measure;
    filtered_velocity_radians_per_sec = lambda*velocity_rad_per_sec + (1-lambda)*filtered_velocity_radians_per_sec;

    // Set velocity = 0 below 1E-3 threshold //TODO: Set a lower threshold!
    //if (abs(filtered_velocity_radians_per_sec) < 1E-3) filtered_velocity_radians_per_sec = 0.0;

    return filtered_velocity_radians_per_sec;
}

float agree_joint_controller::get_incremental_encoder_speed_rad_s_from_driver(){
    //{0x0617,"motor velocity rad/s",0,1}
    float incremental_encoder_speed_rad_s_from_driver = 0;
    ethercat_interface.OUT_system_parameter_type = 0x0617;
    ethercat_interface.OUT_system_parameter_value = 0x0000;
    if (ethercat_interface.IN_system_parameter_type == 0x0617){
        incremental_encoder_speed_rad_s_from_driver = convert_ecat_format_to_float(ethercat_interface.IN_system_parameter_value)*joint_config.incremental_encoder_sign*static_cast<float>(2*M_PI/(joint_config.incremental_encoder_resolution_cpt*4*joint_config.gear_ratio));
        return incremental_encoder_speed_rad_s_from_driver;
    }
    return 0;
}

/* Reads the specified system parameter
float agree_joint_controller::get_incremental_encoder_reference_from_driver(){

    float reference;
    ethercat_interface.OUT_system_parameter_type = 0x0602;
    ethercat_interface.OUT_system_parameter_value = 0x0000;
    if (ethercat_interface.IN_system_parameter_type == 0x0602){
        reference = convert_ecat_format_to_float(ethercat_interface.IN_system_parameter_value);
        return reference;
    }
    return 0;
}
*/

/** @brief Obtains length in mm */
float agree_joint_controller::get_linear_actuator_reading_mm()
{
    esmacat_err linear_actuator_error;
    float output = length_reader.get_linear_actuator_length_mm(linear_actuator_error,&ethercat_interface);
    return output;
}

/** @brief Obtains loadcell torque in mNm */
float agree_joint_controller::get_loadcell_torque_mNm()
{
    esmacat_err loadcell_error;
    float output = load_reader.get_filtered_load_mNm(loadcell_error,&ethercat_interface);
    return output;
}


/** @brief Initiates calibration of the incremental encoder in the position reader */
void agree_joint_controller::incremental_encoder_calibration (float elapsed_time_ms, bool is_init)
{
    position_reader.incremental_encoder_calibration (elapsed_time_ms, is_init, &ethercat_interface, joint_config.gear_ratio);
}

/** @brief Queues the gear power efficiency parameter of the actuator to configure the slave */
void agree_joint_controller::clear_position_reading()
{
    position_reader.clear_incremental_encoder(&ethercat_interface);
}
/** @brief Sets the time period for the slave application */
void agree_joint_controller::set_one_cycle_time_s(float time_period_s)
{
    ethercat_interface.set_one_cycle_time_s (time_period_s);
}

actuator_controller_interface::control_mode_t agree_joint_controller::get_current_control_mode()
{
    return (actuator_controller_interface::control_mode_t) controller_config.control_mode;
}

/*********************************************************/

/** @brief Sets the loaded joint parameters into the respective actuator controller and driver objects
 *
 * @param Pointer to the sea_controller object
 * @param Index of the joint (Note that J[0] and J[1] are unused)
 */
config_file_exception agree_joint_controller::import_actuator_parameters_from_file(sea_controller_configuration_t *c, sea_joint_configuration_t* o,string joint_controller_param_filename)
{

    json_data_file onedof_parameters;
    onedof_parameters.parse(joint_controller_param_filename);

    config_file_exception no_error = config_file_exception::SUCCESS;
    config_file_exception error;

    try {
        json_object actuator_parameters = onedof_parameters.get_object("sea_joint_parameters");

        o->joint_index                              = actuator_parameters.get_scalar<_float>("joint_index").value;
        o->loadcell_calibration_mV_to_mNm           = actuator_parameters.get_scalar<_float>("loadcell_calibration_mV_to_mNm").value;
        o->loadcell_offset_mV                       = actuator_parameters.get_scalar<_float>("loadcell_offset_mV").value;
        o->gear_power_efficiency                    = actuator_parameters.get_scalar<_float>("gear_power_efficiency").value;
        o->gear_ratio                               = actuator_parameters.get_scalar<_uint16_t>("gear_ratio").value;
        o->transmission_ratio                       = actuator_parameters.get_scalar<_uint16_t>("transmission_ratio").value;
        o->current_conversion_factor_A_to_setpoint  = static_cast<float>(actuator_parameters.get_scalar<_float>("current_conversion_factor_A_to_setpoint").value); //TODO: restore to /1000 when using torque control.
        o->torque_constant_mNm_per_A                = actuator_parameters.get_scalar<_float>("torque_constant_mNm_per_A").value;

        o->hard_stop_upper_limit_degrees = actuator_parameters.get_scalar<_float>("hard_stop_upper_limit_degrees").value;
        o->hard_stop_lower_limit_degrees = actuator_parameters.get_scalar<_float>("hard_stop_lower_limit_degrees").value;
        o->incremental_encoder_resolution_cpt = actuator_parameters.get_scalar<_int>("incremental_encoder_resolution_cpt").value;
        o->absolute_encoder_offset_counts = actuator_parameters.get_scalar<_int16_t>("absolute_encoder_offset_counts").value;
        o->loadcell_sign = actuator_parameters.get_scalar<sign_t>("loadcell_sign").value;
        o->absolute_encoder_sign = actuator_parameters.get_scalar<sign_t>("absolute_encoder_sign").value;

        o->incremental_encoder_sign = actuator_parameters.get_scalar<sign_t>("incremental_encoder_sign").value;
        o->calibration_sign = actuator_parameters.get_scalar<sign_t>("encoder_calibration_sign").value;
        o->calibration_offset_rad =  actuator_parameters.get_scalar<_float>("encoder_calibration_offset_rad").value;
        o->calibration_threshold_torque =  actuator_parameters.get_scalar<_float>("encoder_calibration_threshold_torque").value;

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
    float setpoint_to_mNm = o->current_conversion_factor_A_to_setpoint*o->torque_constant_mNm_per_A;
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
        c->friction_comp_vibration_frequency = controller_parameters.get_scalar<_float>("friction_comp_vibration_frequency").value;
        c->friction_comp_vibration_amplitude = controller_parameters.get_scalar<_float>("friction_comp_vibration_amplitude").value;
        c->friction_comp_viscous_coeff       = controller_parameters.get_scalar<_float>("friction_comp_viscous_coeff").value;
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
