#include "agree_motor_controller.h"

agree_motor_controller::agree_motor_controller()
{
    // Initialization of all parameters
    filtered_setpoint_torque_mNm = 0;
    filtered_setpoint = 0;
    encoder_is_referenced = false;
}

/*****************************************************************************************
 * POSITION SENSING
 ****************************************************************************************/

///
/// \brief agree_motor_controller::get_encoder_counter
/// \return
/// Reads encoder counter along the positive direction.
int32_t agree_motor_controller::get_encoder_counter(){
    return esmacat_epos4::get_encoder_counter();
}

///
/// \brief agree_motor_controller::get_encoder_position
/// \return
/// Reads encoder counter, removes offset counts.
int32_t agree_motor_controller::get_encoder_position(){
    int32_t incremental_enc_val = (esmacat_epos4::get_encoder_counter() - incremental_encoder_offset_counts) * incremental_encoder_sign;
    return  incremental_enc_val;
}

///
/// \brief agree_motor_controller::get_encoder_position_radians
/// \return
/// Reads encoder position in radians
float agree_motor_controller::get_encoder_position_radians(){
    return get_encoder_position() *  (2*static_cast<float>(M_PI)/(static_cast<float>(incremental_encoder_resolution_cpt)*4.0*gear_ratio*gear_ratio_transmission)) + incremental_encoder_offset_rad;
}

///
/// \brief agree_motor_controller::get_encoder_computed_speed_radians
/// \return
/// Computes encoder velocity in radians, filtered finite difference
float agree_motor_controller::get_encoder_computed_speed_radians(){

    // Finite difference
    position_rad = get_encoder_position_radians();
    float computed_velocity_rad_per_s = (position_rad - prev_position_rad) / (esmacat_app_one_cycle_time_ms/1000);

    // Filtered Velocity (Exponential Low-Pass Filter)
    // y(k) = (1-a) * y(k-1) + a * x(k)
    float lambda = 0.025;
    computed_velocity_rad_per_s = lambda*computed_velocity_rad_per_s+(1-lambda)*prev_velocity_rad;
    prev_velocity_rad = computed_velocity_rad_per_s;
    prev_position_rad = position_rad;

    return computed_velocity_rad_per_s;
}

///
/// \brief agree_motor_controller::get_encoder_filt_speed_radians
/// \return
/// Reads encoder velocity from EPOS4 driver, filtered at 10Hz
float agree_motor_controller::get_encoder_filt_speed_radians(){
    return get_encoder_filt_speed()*2*M_PI/(60.0*gear_ratio*gear_ratio_transmission); // 10^3 rpm to rad/s
}

///
/// \brief agree_motor_controller::clear_encoder_counter
/// \return
/// Updates encoder counte offset
esmacat_err agree_motor_controller::clear_encoder_counter(){
    incremental_encoder_offset_counts = get_encoder_counter();
    return NO_ERR;
}

/*****************************************************************************************
 * POSITION CONTROL
 ****************************************************************************************/

///
/// \brief agree_motor_controller::set_position_control_rad
/// \param desired_position_rad
/// \param elapsed_time_ms
/// \param setpoint_feedforward
/// \return
///
esmacat_err agree_motor_controller::set_position_control_rad(float desired_position_rad, float elapsed_time_ms, float setpoint_feedforward)
{
    float current_position_rad;
    current_position_rad = get_encoder_position_radians();

    // Proportional Error
    double position_error_rad = desired_position_rad-current_position_rad; //When not using the filtered value

    // Derivative Error
    double deriv_error_rad_per_s = (position_error_rad - prev_position_error_rad)/ esmacat_app_one_cycle_time_ms;
    prev_position_error_rad = position_error_rad;

    // Filtered Derivative Error (Exponential Low-Pass Filter)
    // Filtered Velocity (Exponential Low-Pass Filter)
    // y(k) = (1-a) * y(k-1) + a * x(k)
    double lambda = 0.05;
    deriv_error_rad_per_s = lambda*deriv_error_rad_per_s+(1-lambda)*prev_deriv_error_rad_per_s;
    prev_deriv_error_rad_per_s = deriv_error_rad_per_s;

    // Integral Error
    integ_position_error_rad += position_error_rad*esmacat_app_one_cycle_time_ms;

    // Anti-Windup
    if (integ_position_error_rad > max_integ_position_error_rad) {
        integ_position_error_rad = max_integ_position_error_rad;
    }
    if (integ_position_error_rad < -max_integ_position_error_rad ) {
        integ_position_error_rad = -max_integ_position_error_rad;
    }

    float motor_factor = incremental_encoder_resolution_cpt*4*gear_ratio/(2*M_PI);
    float setpoint_p_gain = position_control_rad_p_gain*motor_factor*position_error_rad; // setpoint for p-gain
    float setpoint_d_gain = position_control_rad_d_gain*motor_factor*deriv_error_rad_per_s; // setpoint for d-gain
    float setpoint_i_gain = position_control_rad_i_gain*motor_factor*integ_position_error_rad; // setpoint for i-gain
    float setpoint_position_ctrl = setpoint_p_gain + setpoint_d_gain + setpoint_i_gain;
    //if (setpoint_position_ctrl > 1) setpoint_position_ctrl = 1;
    //if (setpoint_position_ctrl < -1) setpoint_position_ctrl = -1;
    float sigm_val = 1; sigmoid(3-elapsed_time_ms/1000); //Sigmoid function to avoid instability at the beginning of the loop
    set_target_torque( static_cast<int16_t>((setpoint_position_ctrl + setpoint_feedforward )*sigm_val));

    //    For Debug purpose only
    //    std::cout << "des: " << desired_position_rad << " act: " << current_position_rad << endl;
    std::cout << "ep: " << position_error_rad << " ed: " << deriv_error_rad_per_s << " ei: " << integ_position_error_rad << std::endl;
    //    std::cout <<"spp "  << setpoint_p_gain<<"  spd "<<setpoint_d_gain<<"  spi "<<setpoint_i_gain<<"  sp "<<setpoint_position_ctrl << std::endl;
    //    std::cout<<"  C_time  " << esmacat_app_one_cycle_time_sec << std::endl;

    return NO_ERR;

}

// ////////////////////////////////////////////////////////////////
// /////////////////////// SPEED CONTROL //////////////////////////
/// \brief agree_motor_controller::set_speed_control
/// \param desired_speed_rad_s
/// \param elapsed_time_ms
/// \param setpoint_feedforward
/// \return
///
esmacat_err agree_motor_controller::set_speed_control(float desired_speed_rad_s, float setpoint_feedforward)
{
    // Proportional Error
    float speed_error_rad = desired_speed_rad_s - velocity_rad_per_s;

    // Derivative Error
    float speed_deriv_error_rad_s = (speed_error_rad - prev_speed_error_rad)/ esmacat_app_one_cycle_time_ms;
    prev_speed_error_rad = speed_error_rad;

    // Filtered Derivative Error (Exponential Low-Pass Filter)
    // Filtered Velocity (Exponential Low-Pass Filter)
    // y(k) = (1-a) * y(k-1) + a * x(k)
    float lambda = 0.05;
    speed_deriv_error_rad_s = lambda*speed_deriv_error_rad_s+(1-lambda)*prev_deriv_error_rad_per_s;
    prev_deriv_error_rad_per_s = speed_deriv_error_rad_s;

    // Integral Error
    speed_integ_error_rad += speed_error_rad * esmacat_app_one_cycle_time_ms;

    // Anti-Windup
    if (speed_integ_error_rad > max_integ_speed_error_rad) {
        speed_integ_error_rad = max_integ_speed_error_rad;
    }
    if (speed_integ_error_rad < -max_integ_speed_error_rad ) {
        speed_integ_error_rad = -max_integ_speed_error_rad;
    }

    // Compute conversion from
    float motor_factor = incremental_encoder_resolution_cpt*4*gear_ratio*gear_ratio_transmission/(2*static_cast<float>(M_PI));

    // P-I-D Setpoints
    float setpoint_p_gain = speed_control_rad_p_gain*motor_factor*speed_error_rad;
    float setpoint_i_gain = speed_control_rad_i_gain*motor_factor*speed_integ_error_rad;        // setpoint for i-gain
    float setpoint_d_gain = speed_control_rad_d_gain*motor_factor*speed_deriv_error_rad_s;      // setpoint for d-gain

    // Total Setpoint
    float setpoint_position_ctrl = setpoint_p_gain + setpoint_d_gain + setpoint_i_gain;

    // Set current target to motor driver (Open-loop torque control)
    set_target_torque( static_cast<int16_t>(setpoint_position_ctrl + setpoint_feedforward ));

    return NO_ERR;

}

///
/// \brief agree_motor_controller::set_homing_mode
/// \return
///
bool agree_motor_controller::set_homing_mode(){

    // If motor is already homed
    if(get_statusword_bit(15)){
        encoder_is_referenced = true;
    }
        start_motor();

        if(input_statusword == 1591){
            set_controlword_bit(4,1);
        }

        if(get_statusword_bit(15)){
            if (encoder_is_referenced == false){
                clear_encoder_counter();
                encoder_is_referenced = true;
            }
    }

    return encoder_is_referenced;
}

void agree_motor_controller::set_position_control_rad_pid_gain(float p, float d, float i){
    position_control_rad_p_gain = p*0.001;
    position_control_rad_d_gain = d*0.001;
    position_control_rad_i_gain = i*0.001;
}

void agree_motor_controller::set_speed_control_rad_pid_gain(float p, float d, float i){
    speed_control_rad_p_gain = p*0.001;
    speed_control_rad_d_gain = d*0.001;
    speed_control_rad_i_gain = i*0.001;
}

//// ////////////////////////////////////////////////////////////////
//// /////////////////////// Torque Sensing /////////////////////////
//// ////////////////////////////////////////////////////////////////

///
/// \brief agree_motor_controller::get_loadcell_torque_mNm
/// \return joint loadcell torque in mNm
///
double agree_motor_controller::get_loadcell_torque_mNm(){

    // Torque sensing
    // 200 Lbs = 22.5969658 Nm
    // 100 Lbs = 11.2984829 Nm
    // 1 Lbs   = 0.112984829 Nm

    double loadcell_torque_Nm = (static_cast<double>(get_analog_input_mV())/10.0*loadcell_fullscale_lbs*0.112984829*loadcell_sign - loadcell_offset_mNm)*gear_ratio_transmission;

    // Filtered Velocity (Exponential Low-Pass Filter)
    // y(k) = (1-a) * y(k-1) + a * x(k)
    // Filter at about 20 Hz
    double lambda = 0.118;
    loadcell_torque_Nm = lambda*loadcell_torque_Nm+(1-lambda)*prev_loadcell_torque;
    prev_loadcell_torque = loadcell_torque_Nm;
    return loadcell_torque_Nm;
}

//// ////////////////////////////////////////////////////////////////
//// //////////////////////// Torque Control ////////////////////////
//// ////////////////////////////////////////////////////////////////

// ////////////////////////////////////////////////////////////////
// ////////////// OPEN-LOOP TORQUE CONTROL ////////////////////////
/// \brief agree_motor_controller::set_feedforward_torque_mNm
/// \param desired_torque_mNm
/// \return
///
esmacat_err agree_motor_controller::set_feedforward_torque_mNm(float desired_torque_mNm)
{
    // Converts desired torque setpoint to current setpoint according to torque_constant (mNm/A) nominal_current (A) and gear_ratio (156:1)
    // load_torque = target_torque / 1000 * (torque_constant * nominal current * gear_ratio)
    set_target_torque(desired_torque_mNm*1000.0/(nominal_current_A*torque_constant_mNm_per_mA*gear_ratio));
    return NO_ERR;
}

// ////////////////////////////////////////////////////////////////
// ////////////// CLOSED-LOOP TORQUE CONTROL //////////////////////
/// \brief agree_motor_controller::set_torque_control_mNm
/// \param desired torque in mNm
/// \return esmacat error
///
esmacat_err agree_motor_controller::set_torque_control_mNm(double desired_torque_mNm){

    double actual_loadcell_torque_mNm = get_loadcell_torque_mNm();

    //  Low pass filter for desired torque
        if ( (desired_torque_mNm - filtered_setpoint_torque_mNm) >  50.0 )
        {
            filtered_setpoint_torque_mNm = filtered_setpoint_torque_mNm + 50.0;
        }
        else if ( (desired_torque_mNm - filtered_setpoint_torque_mNm) <  -50.0)
        {
            filtered_setpoint_torque_mNm = filtered_setpoint_torque_mNm - 50.0;
        }
        else
        {
            filtered_setpoint_torque_mNm = desired_torque_mNm ;
        }

    /** *********************************/
    /** Feedforward torque control loop */
    /** *********************************/

    // Feedforward compensation for gearbox friction.
    double feedforward_friction_compensation_mNm = 0;
    float coulomb_torque_mNm = 0.0;
    float coulomb_velocity_rad_s = static_cast<float>(1E-1);
    feedforward_friction_compensation_mNm = coulomb_torque_mNm*tanh(velocity_rad_per_s/coulomb_velocity_rad_s)   +
                                            viscous_friction_coeff*velocity_rad_per_s;

    double feedforward_vibration = sin(2*3.1415*elapsed_time/15.0)*2500;
    float  gearbox_efficiency = 1.0;

    // Feedforward compensation for gearbox inefficiency
    float feedforward_efficiency_compensation_mNm = (filtered_setpoint_torque_mNm+feedforward_friction_compensation_mNm+feedforward_vibration)/gearbox_efficiency;

    // Convertion for Feedforward torque to setpoint
    float feedforward_demanded_motor_torque_mNm = feedforward_efficiency_compensation_mNm;

    float setpoint_feedforward = mNm_to_setpoint(feedforward_efficiency_compensation_mNm);

    /** ******************************/
    /** Feedback torque control loop */
    /** ******************************/

    // Proportional Error
    float loadcell_error_mNm =  filtered_setpoint_torque_mNm - actual_loadcell_torque_mNm;

    if(loadcell_error_mNm > 2000) loadcell_error_mNm = 2000; //max proportional torque error
    else if(loadcell_error_mNm < -2000) loadcell_error_mNm = -2000;
    // Proportional Term
    float feedback_p_torque_mNm = joint_impedance_control_config.torque_control_p_gain * loadcell_error_mNm;

    // Derivative Error
    float deriv_loadcell_error_mNm = ( - prev_loadcell_error_mNm)/esmacat_app_one_cycle_time_ms;
    prev_loadcell_error_mNm = loadcell_error_mNm;

    // Filtered Derivative Error (Exponential Low-Pass Filter)
    // y(k) = (1-a) * y(k-1) + a * x(k)
    float lambda = 0.05;
    deriv_loadcell_error_mNm = lambda*deriv_loadcell_error_mNm+(1-lambda)*prev_deriv_loadcell_error_mNm;
    prev_deriv_loadcell_error_mNm = deriv_loadcell_error_mNm;

    // Derivative Term
    float feedback_d_torque_mNm = joint_impedance_control_config.torque_control_d_gain * deriv_loadcell_error_mNm;

    // Integral Error
    integ_loadcell_error_mNm += loadcell_error_mNm*esmacat_app_one_cycle_time_ms;
    if(integ_loadcell_error_mNm > max_integ_torque_error_mNm) integ_loadcell_error_mNm = max_integ_torque_error_mNm; //max integral torque error
    else if(integ_loadcell_error_mNm < -max_integ_torque_error_mNm) integ_loadcell_error_mNm = -max_integ_torque_error_mNm;

    // Integral Term
    float feedback_i_torque_mNm =  joint_impedance_control_config.torque_control_i_gain * integ_loadcell_error_mNm;

    // Total Feedback Torque
    float feedback_demanded_torque_mNm = feedback_p_torque_mNm + feedback_i_torque_mNm + feedback_d_torque_mNm;

    /** Summing, filtering, and passing setpoint */

    // Convertion from Torque to setpoint
    float setpoint_feedback = mNm_to_setpoint(feedback_demanded_torque_mNm);

    float setpoint = setpoint_feedback + setpoint_feedforward;

    // Low-pass Filter the setpoint to avoid peaks
//    filtered_setpoint = 0.9*filtered_setpoint + 0.1*setpoint;
    filtered_setpoint = 0.0*filtered_setpoint + 1.0*setpoint;

    // Set current setpoint
    set_target_torque(filtered_setpoint);

    // Save torque control status
    joint_torque_control_status.desired_torque_mNm              = desired_torque_mNm;
    joint_torque_control_status.feedforward_demanded_torque_mNm = feedforward_demanded_motor_torque_mNm;
    joint_torque_control_status.loadcell_error_mNm              = loadcell_error_mNm;
    joint_torque_control_status.feedback_p_torque_mNm           = feedback_p_torque_mNm;
    joint_torque_control_status.feedback_i_torque_mNm           = feedback_i_torque_mNm;
    joint_torque_control_status.feedback_d_torque_mNm           = feedback_d_torque_mNm;
    joint_torque_control_status.setpoint_feedforward            = setpoint_feedforward;
    joint_torque_control_status.setpoint_feedback               = setpoint_feedback;
    joint_torque_control_status.setpoint                        = setpoint;
    return NO_ERR;
}

void agree_motor_controller::set_torque_control_pid_gain(float p, float d, float i){
    joint_impedance_control_config.torque_control_p_gain = p;
    joint_impedance_control_config.torque_control_d_gain = d;
    joint_impedance_control_config.torque_control_i_gain = i;
}

/*

//Friction Torque
if (velocity_rad_per_s > friction_torque_threshold_rad_per_s){
    friction_comp_torque_mNm = -friction_comp_max_torque_mNm;
}else if (velocity_rad_per_s < -friction_torque_threshold_rad_per_s){
    friction_comp_torque_mNm = friction_comp_max_torque_mNm;
}else{
    friction_comp_torque_mNm = 0;
}
friction_comp_torque_mNm = 0.4*friction_comp_torque_mNm+0.6*prev_friction_comp_torque_mNm; //Filtering friction comp torque
prev_friction_comp_torque_mNm = friction_comp_torque_mNm;

*/
// ////////////////////////////////////////////////////////////////
// ////////////////////// IMPEDANCE CONTROL ///////////////////////
/// \brief agree_motor_controller::set_impedance_control_rad
/// \param impedance_control_setpoint_rad
/// \param gravity_torque_mNm
/// \return
///
esmacat_err agree_motor_controller::set_impedance_control_rad(float impedance_control_setpoint_rad,float gravity_torque_mNm)
{

    float controller_torque_k_mNm =0;
    float controller_torque_d_mNm =0;
    float impedance_control_torque_mNm =0;
    float impedance_control_error_rad = 0;
    float friction_comp_torque_mNm = 0;
    float soft_stop_torque_mNm = 0;
    float impedance_control_max_error_rad = M_PI/8; joint_impedance_control_config.impedance_control_max_error_rad;
    float impedance_control_k_mNm_per_rad = joint_impedance_control_config.impedance_control_k_gain_mNm_per_rad;
    float impedance_control_d_mNm_s_per_rad = joint_impedance_control_config.impedance_control_d_gain_mNm_per_rad_per_sec;

    double velocity_stop_threshold_rad_per_sec = M_PI*10; joint_impedance_control_config.max_velocity_threshold_rad_per_sec;
    double max_load_stop_threshold_mNm = 10000;

    // Stiffness Control
    impedance_control_error_rad = impedance_control_setpoint_rad - get_encoder_position_radians();
    if( impedance_control_error_rad > impedance_control_max_error_rad) impedance_control_error_rad = impedance_control_max_error_rad;
    else if (impedance_control_error_rad < -impedance_control_max_error_rad) impedance_control_error_rad = -impedance_control_max_error_rad;
    controller_torque_k_mNm =  impedance_control_k_mNm_per_rad * impedance_control_error_rad;

    // Damping Control
    controller_torque_d_mNm = - impedance_control_d_mNm_s_per_rad * get_encoder_filt_speed_radians();

    // Impedance control
    impedance_control_torque_mNm = controller_torque_k_mNm + controller_torque_d_mNm;

    // Soft End-stops to prevend collision with hard End-Stops
    /** Positive soft end-stop in radians */
    float soft_stop_max_rad = hard_stop_max_rad - soft_to_hard_stop_offset_rad;
    /** Negative soft end-stop in radians */
    float soft_stop_min_rad = hard_stop_min_rad + soft_to_hard_stop_offset_rad;

    // If beyond positive soft end-stop
    if ((position_rad >= soft_stop_max_rad))
    {
        float scaled_travel_beyond_soft_stop_pos = (position_rad-soft_stop_max_rad)/soft_to_hard_stop_offset_rad;
        soft_stop_torque_mNm = + soft_stop_max_torque_mNm/2
                               - soft_stop_max_torque_mNm * (sigmoid(-scaled_travel_beyond_soft_stop_pos))
                               - soft_stop_damping * get_encoder_filt_speed_radians();
    }
    // If beyond positive soft end-stop
    else if ((position_rad <= soft_stop_min_rad))
    {
        float scaled_travel_beyond_soft_stop_neg = (soft_stop_min_rad - position_rad )/soft_to_hard_stop_offset_rad;
        soft_stop_torque_mNm =  - soft_stop_max_torque_mNm/2
                                + soft_stop_max_torque_mNm * sigmoid(-scaled_travel_beyond_soft_stop_neg)
                                - soft_stop_damping * get_encoder_filt_speed_radians();
    }

    // Max velocity stop
    if (fabs(velocity_rad_per_s) >= velocity_stop_threshold_rad_per_sec){
        stop_motor();
        std::cout<<"ERR_MEASURED_VELOCITY_OUTSIDE_RANGE. EPOS Disabled"<<std::endl;
    }else{

    }

    // Max load stop
    if (fabs(loadcell_torque_mNm) >= max_load_stop_threshold_mNm){
        stop_motor();
        std::cout<<"ERR_MEASURED_LOAD_OUTSIDE_RANGE. EPOS Disabled"<<std::endl;
    }


    // Summing up Impedance Controller and Feedforward terms */

    // Filtered Setpoint (Exponential Low-Pass Filter)
    // y(k) = (1-a) * y(k-1) + a * x(k)
    double lambda = 1;
    filtered_setpoint_torque_mNm = (1-lambda)*filtered_setpoint_torque_mNm
                                   + lambda  *(gravity_torque_mNm + impedance_control_torque_mNm + friction_comp_torque_mNm + soft_stop_torque_mNm);
    // Torque loop
    set_torque_control_mNm(filtered_setpoint_torque_mNm);

    /** Save Impedance Controller Status */
    joint_impedance_control_status.impedance_control_setpoint_rad    = impedance_control_setpoint_rad;
    joint_impedance_control_status.impedance_control_error_rad       = impedance_control_error_rad;
    joint_impedance_control_status.impedance_control_torque_mNm      = impedance_control_torque_mNm;
    joint_impedance_control_status.impedance_control_k_mNm_per_rad   = impedance_control_k_mNm_per_rad;
    joint_impedance_control_status.impedance_control_d_mNm_s_per_rad = impedance_control_d_mNm_s_per_rad;
    joint_impedance_control_status.gravity_torque_mNm                = gravity_torque_mNm;
    joint_impedance_control_status.filtered_setpoint_torque_mNm      = filtered_setpoint_torque_mNm;

    return NO_ERR;
}

void agree_motor_controller::set_impedance_control_gain(float k, float d){
    joint_impedance_control_config.impedance_control_k_gain_mNm_per_rad           = k * 1000;
    joint_impedance_control_config.impedance_control_d_gain_mNm_per_rad_per_sec   = d * 1000;
    //joint_impedance_control_config.impedance_control_max_error_rad                = max_err ;
}

void agree_motor_controller::save_joint_status(){
    joint_status.incremental_encoder_position_cpt     = get_encoder_position();
    joint_status.incremental_encoder_position_radians = get_encoder_position_radians();
    joint_status.loadcell_torque_mNm                  = get_loadcell_torque_mNm();
    joint_status.velocity_rad_per_s                   = get_encoder_filt_speed_radians();
    joint_status.velocity_computed_rad_per_s          = get_encoder_computed_speed_radians();
}

//// ////////////////////////////////////////////////////////////////
//// ///////////////////// COMMON ///////////////////////////////////
//// ////////////////////////////////////////////////////////////////

float agree_motor_controller::mNm_to_setpoint(float torque_mNm)
{
    return torque_mNm*1000/(nominal_current_A*torque_constant_mNm_per_mA*gear_ratio);
}

void agree_motor_controller::set_joint_index(int j){
    joint_index = j;
}
void agree_motor_controller::set_torque_constant(float t){
    torque_constant_mNm_per_mA = t;
}
void agree_motor_controller::set_nominal_current(float i){
    nominal_current_A = i;
}
void agree_motor_controller::set_encoder_cpt(int32_t cpt){
    incremental_encoder_resolution_cpt = cpt;
}
void agree_motor_controller::set_gear_ratio(float gear){
    gear_ratio = gear;
}
void agree_motor_controller::set_gear_ratio_transmission(float g){
    gear_ratio_transmission = g;
}
void agree_motor_controller::set_encoder_sign(int s){
    incremental_encoder_sign = s;
}
void agree_motor_controller::set_encoder_offset(float offset){
    incremental_encoder_offset_rad = offset;
}
void agree_motor_controller::set_loadcell_fullscale_lbs(float f){
    loadcell_fullscale_lbs = f;
}
void agree_motor_controller::set_loadcell_offset_mNm(float offset){
    loadcell_offset_mNm = offset;
}
void agree_motor_controller::set_loadcell_sign(int sign){
    loadcell_sign = sign;
}
void agree_motor_controller::set_viscous_friction_coefficient(float v){
    viscous_friction_coeff = v;
}
void agree_motor_controller::set_coulomb_friction_mNm(float c){
    coulomb_friction_mNm = c;
}
void agree_motor_controller::set_joint_limits(float min, float max){
    hard_stop_max_rad = max*deg_to_rad;
    hard_stop_min_rad = min*deg_to_rad;
}

void agree_motor_controller::reset_position_errors(){
    integ_position_error_rad = 0;
    prev_position_error_rad = 0;
    prev_deriv_error_rad_per_s = 0;
}

void agree_motor_controller::set_joint_motor_configuration(joint_motor_configuration_t* motor_config){
  //joint_index                           = motor_config->joint_index;
  incremental_encoder_resolution_cpt    = motor_config->incremental_encoder_resolution_cpt;
  incremental_encoder_sign              = motor_config->incremental_encoder_sign;

  gear_ratio                            = motor_config->gear_ratio;
  gear_ratio_transmission               = motor_config->gear_ratio_transmission;


}

void agree_motor_controller::get_errorcode_hex()
{
  //esmacat_err e = NO_ERR;
  if(input_errorcode != 0)
  {
  std::stringstream ss;
  ss << std::hex << input_errorcode; // int decimal_value
  std::string res ( ss.str() );
  std::cout << "ERROR J" << joint_index+1 << " CODE: 0x" << res <<std::endl;
  }
}
