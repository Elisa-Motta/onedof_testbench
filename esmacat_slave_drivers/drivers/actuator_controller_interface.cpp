/** @file
 * @brief Contains definitions of functions used for the Series-Elastic Actuator (SEA) Driver
*/

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "actuator_controller_interface.h"

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/


/** @brief Constructor for the class that holds the initialization
 *
 * Intializes the raw ADC readings for the analog inputs from both
 * external sources and the ESCON. Also sets the product code and vendor ID.
 * Clears the encoder, disables the ESCON and clears its inputs and fault.
 * It also sets the time interval for the control loop computations.
 */

actuator_controller_interface::actuator_controller_interface()
{
    PLOGI <<" Actuator controller interface object has been created " ;

    /** Contains a product code of the Esmacat slave */
    esmacat_slave_product_id = ACTUATOR_CONTROLLER_PRODUCT_ID;
    /** Contains the vendor ID issued by the EtherCAT Technology group */
    esmacat_slave_vendor_id = ACTUATOR_CONTROLLER_VENDOR_ID;
    /** Contains the firmware version for this Actuator Controller Driver */
    esmacat_slave_revision_number = ACTUATOR_CONTROLLER_REVISION;

}

/** @brief Private Function- Reads the specified raw analog input from external source
 * @param Index of the analog input to be read
 * @return Reading of the specified analog input
 */
uint16_t actuator_controller_interface::get_raw_loadcell_reading(void)
{
    return loadcell_reading_raw;
}


/** @brief Private Function- Reads the specified raw analog input from ESCON
 * @param Index of the analog input to be read
 * @return Reading of the specified analog input
 */
uint16_t actuator_controller_interface::get_raw_escon_analog_input(int ai_index)
{
    //    return raw_escon_analog_input_reading[ai_index];
}

esmacat_err actuator_controller_interface::set_aux_setpoint(float aux)
{
    aux_setpoint=aux;
    return NO_ERR;
}


/** @brief Definition of inherited data exchange function for Esmacat slave
 *
 * Combines data exchange process of the base module and the motor driver slaves
 *
 * @param ec_slave_outputs Pointer to the start of the outputs for slave in consideration
 * @param oloop No. of output bytes
 * @param ec_slave_inputs Pointer to the start of the inputs for slave in consideration
 * @param iloop No. of input bytes
 */
void actuator_controller_interface::ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop)
{

    ecat_data_process_actuator_controller_interface(ec_slave_outputs,ec_slave_inputs);   // for actuator controller interface
}

/** @brief Private function- Data exchange process that is specific to the Esmacat Motor Driver slave
 *
 *  [***hardware dependent - do not change***] This function queues the values to be sent to the
 * firmware, and also decodes packets received. Motor Driver fault, analog input readings from both
 * the external source and the ESCON, and quadrature encoder readings are handled here.
 * @param ec_slave_outputs Pointer to the start of the outputs for slave in consideration
 * @param ec_slave_inputs Pointer to the start of the inputs for slave in consideration
 **/
void actuator_controller_interface::ecat_data_process_actuator_controller_interface(uint8_t* ec_slave_outputs,uint8_t* ec_slave_inputs)
{
//    unsigned char input_variable[15];
//    unsigned char output_variable[12];

//    for (int i=0;i<15;i++) input_variable[i] = *(ec_slave_inputs+i);
//    actuator_controller_status = +(input_variable[0] << 0)+(input_variable[1] << 8);
//    IN_system_parameter_type = +(input_variable[2] << 0)+(input_variable[3] << 8);
//    IN_system_parameter_value = +(input_variable[4] << 0)+(input_variable[5] << 8);
//    loadcell_reading_raw = +(input_variable[6] << 0)+(input_variable[7] << 8);
//    incremental_encoder_reading_raw_cpt = +(input_variable[8] << 0)+(input_variable[9] << 8)+(input_variable[10] << 16)+(input_variable[11] << 24);
//    absolute_encoder_reading_raw_cpt= +(input_variable[12] << 0)+(input_variable[13] << 8);
//    for (int i=0;i<8;i++) general_purpose_input[i] = (((input_variable[14] & (0b00000001 << i)) != 0x00));

//    for (int i=0;i<12;i++) output_variable[i] = 0;
//    output_variable[0] = output_variable[0] | (escon_enable << 0) ;
//    for (int i=0;i<7;i++) output_variable[0] = output_variable[0] | (general_purpose_output[i] << (i+1)) ;  // gpio 0-5

//    uint8_t escon_derating_factor_1to255 = static_cast<uint8_t>(escon_derating_factor * 255);
//    output_variable[1] = escon_derating_factor_1to255;
//    output_variable[2] =  (OUT_system_parameter_type ) & 0x00ff;
//    output_variable[3] =  (OUT_system_parameter_type >> 8) & 0x00ff;
//    output_variable[4] =  (OUT_system_parameter_value >> 0) & 0x00ff;
//    output_variable[5] =  (OUT_system_parameter_value >> 8) & 0x00ff;
//    uint32_t transmit = convert_float_for_tx(control_setpoint);
//    output_variable[6]  = (transmit & 0x000000ff) >> 0;
//    output_variable[7]  = (transmit & 0x0000ff00) >> 8;
//    output_variable[8]  = (transmit & 0x00ff0000) >> 16;
//    output_variable[9]  = (transmit & 0xff000000) >> 24;
//    output_variable[10] = (communication_timeout_counter >> 0) & 0x00ff;
//    output_variable[11] = (communication_timeout_counter >> 8) & 0x00ff;
//    for (int i=0;i<12;i++) *(ec_slave_outputs+i) = output_variable[i];

    unsigned char input_variable[15];
    unsigned char output_variable[16];

    for (int i=0;i<15;i++) input_variable[i] = *(ec_slave_inputs+i);
    actuator_controller_status = +(input_variable[0] << 0)+(input_variable[1] << 8);
    IN_system_parameter_type = +(input_variable[2] << 0)+(input_variable[3] << 8);
    IN_system_parameter_value = +(input_variable[4] << 0)+(input_variable[5] << 8);
    loadcell_reading_raw = +(input_variable[6] << 0)+(input_variable[7] << 8);
    incremental_encoder_reading_raw_cpt = +(input_variable[8] << 0)+(input_variable[9] << 8)+(input_variable[10] << 16)+(input_variable[11] << 24);
    absolute_encoder_reading_raw_cpt= +(input_variable[12] << 0)+(input_variable[13] << 8);
    for (int i=0;i<8;i++) general_purpose_input[i] = (((input_variable[14] & (0b00000001 << i)) != 0x00));

    for (int i=0;i<16;i++) output_variable[i] = 0;
    output_variable[0] = output_variable[0] | (escon_enable << 0) ;
    for (int i=0;i<7;i++) output_variable[0] = output_variable[0] | (general_purpose_output[i] << (i+1)) ;  // gpio 0-5

    uint8_t escon_derating_factor_1to255 = static_cast<uint8_t>(escon_derating_factor * 255);
    output_variable[1] = escon_derating_factor_1to255;
    output_variable[2] =  (OUT_system_parameter_type ) & 0x00ff;
    output_variable[3] =  (OUT_system_parameter_type >> 8) & 0x00ff;
    output_variable[4] =  (OUT_system_parameter_value >> 0) & 0x00ff;
    output_variable[5] =  (OUT_system_parameter_value >> 8) & 0x00ff;
    uint32_t transmit = convert_float_for_tx(control_setpoint);
    output_variable[6]  = (transmit & 0x000000ff) >> 0;
    output_variable[7]  = (transmit & 0x0000ff00) >> 8;
    output_variable[8]  = (transmit & 0x00ff0000) >> 16;
    output_variable[9]  = (transmit & 0xff000000) >> 24;
    output_variable[10] = (communication_timeout_counter >> 0) & 0x00ff;
    output_variable[11] = (communication_timeout_counter >> 8) & 0x00ff;
    transmit = convert_float_for_tx(aux_setpoint);
    output_variable[12]  = (transmit & 0x000000ff) >> 0;
    output_variable[13]  = (transmit & 0x0000ff00) >> 8;
    output_variable[14]  = (transmit & 0x00ff0000) >> 16;
    output_variable[15]  = (transmit & 0xff000000) >> 24;
    for (int i=0;i<16;i++) *(ec_slave_outputs+i) = output_variable[i];
}

/** @brief Reads analog input value in mV and provides error handling
 *
 * @param analog_input_index Index of the external source analog input to be read
 * @param error Contains the error message to demonstrate success of data acquisition
  * (ERR_SEA_DRIVER_EXT_ANALOG_INPUT_INDEX_OUT_OF_RANGE or NO_ERR)
 * @return Voltage reading (mV) of the selected external source analog input
 */
float actuator_controller_interface::get_loadcell_reading_mV(esmacat_err& error)
{
    float adc_reading_mV = 0;
    error = NO_ERR;
    adc_reading_mV = static_cast<float>(SEA_ADC_OFFSET_MV + SEA_ADC_FSR_MV * static_cast<double>(get_raw_loadcell_reading())/static_cast<double>(UINT16_MAX));
    return adc_reading_mV;
}





/** @brief Obtains the fault status of the ESCON
 * @return ESCON fault status
 */
bool actuator_controller_interface::get_escon_fault()
{
    return general_purpose_input[5]; // the firmware 3.25 uses general_purpose_input[5] for escon_fault
}


/** @brief Computes the signed raw absolute encoder reading of the joint
 * in counts/turn
 *
 * Applies the sign to the raw reading from the absolute encoder
 * @return Absolute encoder reading in counts/turn
 */
uint16_t actuator_controller_interface::get_raw_absolute_encoder_reading_cpt()
{
    return absolute_encoder_reading_raw_cpt;
}


/** @brief Computes the signed raw incremental encoder reading of the joint
 * in counts/turn
 *
 * Applies the sign to the raw reading from the incremental encoder
 * @return Incremental encoder reading in counts/turn
 */
int32_t actuator_controller_interface::get_raw_incremental_encoder_reading_cpt()
{
    return incremental_encoder_reading_raw_cpt ;
}

/** @brief Obtains the joint acceleration in counts/sec^2
 */
float actuator_controller_interface::get_acceleration_counts_per_sec_square()
{
    return acceleration_counts_per_sec_square ;
}

esmacat_err actuator_controller_interface::read_feedback_variables_sequentially(uint64_t loop_cnt, bool flag_print)
{
    const int index_start_feedback = LOOP_INDEX_FOR_STARTING_TO_READ_FEEDBACK_VALUES_SEQUENCIALLY;
    if (loop_cnt < index_start_feedback) return NO_ERR;

    OUT_system_parameter_type = feedback_list[ loop_cnt%feedback_list.size() ].feedback_type;
    OUT_system_parameter_value = 0x0000;

    for (unsigned int i=0;i<feedback_list.size();i++)
    {
        if(IN_system_parameter_type == feedback_list[i].feedback_type )
        {
            feedback_list[i].feedback_value = IN_system_parameter_value;
            if (flag_print == true)
            {
                if ( i== 0) cout << endl;
                cout << feedback_list[i].feedback_label << " " ;
                if ( feedback_list[i].feedback_datatype_0int_1float == 0) cout << feedback_list[i].feedback_value;
                else cout << convert_ecat_format_to_float(feedback_list[i].feedback_value);
            }
        }
    }
    if (flag_print == true) cout << "\t";
    return NO_ERR;
}

/** @brief Allow access to private variable for the one cycle time period
 * @return One cycle time period
 */
float actuator_controller_interface::get_esmacat_app_one_cycle_time_sec()
{
    return esmacat_app_one_cycle_time_sec;
}
/** @brief Enables/disables the escon
 * @param enable 1= Enable, 0 = Disable
 */
void actuator_controller_interface::set_escon_enable(bool enable)
{
    escon_enable  = enable;
}

void actuator_controller_interface::set_user_status_led(bool turn_on)
{
    user_status_led = turn_on;
    general_purpose_output[5] = turn_on;
}


/** @brief Provides current setpoint to the ESCON
 *
 * Verifies that the setpoint is within the specified bounds. Limits the value
 * to the bounds specified. The ESCON in only driven from 10% to 90% of its
 * usable range
 * @param setpoint Current setpoint value for the ESCON
 * (Range = [ESMACAT_MD_MIN_SET_VALUE, ESMACAT_MD_MAX_SET_VALUE])
 * @return Status of the function. ERR_SEA_DRIVER_SETPOINT_OUT_OF_RANGE
 * if the setpoint was out of range, else NO_ERR
 */
esmacat_err actuator_controller_interface::set_control_setpoint_0to1(float setpoint)
{
    esmacat_err e = NO_ERR;
    if (setpoint > ACTUATOR_MAX_CURRENT_SETPOINT ){
        setpoint = ACTUATOR_MAX_CURRENT_SETPOINT;
        e= ERR_SEA_DRIVER_SETPOINT_OUT_OF_RANGE;
    }
    if (setpoint < ACTUATOR_MIN_CURRENT_SETPOINT ){
        setpoint = ACTUATOR_MIN_CURRENT_SETPOINT ;
        e = ERR_SEA_DRIVER_SETPOINT_OUT_OF_RANGE;
    }
    set_control_setpoint(setpoint);
    return e;
}

esmacat_err actuator_controller_interface::set_control_setpoint(float setpoint)
{
    control_setpoint = setpoint;
    return NO_ERR;
}

/** Allows for a derating fraction between 0 and 1 to be set
 */
esmacat_err actuator_controller_interface::set_escon_derating_factor(float factor)
{
    if (factor > 1.0 )
    {
        factor = 1.0;
        PLOGW << "ESCON derating factor ,"<< factor << ", is higher than 1.0";
    }
    if (factor < 0.0 )
    {
        factor = 0.0;
        PLOGW << "ESCON derating factor ,"<< factor << ", is lower than 0.0";
    }
    escon_derating_factor = factor;
    return NO_ERR;
}

/** @brief Sets and Queues the conversion parameter of the escon to configure the slave */
void actuator_controller_interface::set_current_conversion_factor(float conversion_factor)
{
    current_conversion_factor_mA_to_setpoint = conversion_factor;
    configure_escon_conversion_factor(conversion_factor);
}


///** @brief Sets the control setpoint to configure the slave */
//void actuator_controller_interface::set_escon_setpoint(float setpoint)
//{
//    control_setpoint = setpoint;
//}

/** @brief Sets the direction of the digital I/O on the slave
 *
 * Also queues the values to be sent to the firmware [** do not change**]
 * @param dio_direction Array of ESMACAT_BASE_NUMBER_OF_DIO number of values specifying
 * which signals are inputs and which are set as outputs
 */
esmacat_err actuator_controller_interface::configure_dio_direction(IO_Direction* direction)
{
    esmacat_err e = ERR_UNKNOWN;
    uint16_t dio_direction_bit_array = 0;
    for (int i=0;i<ACTUATOR_CONTROLLER_NUMBER_OF_DIO;i++)
    {
        dio_direction[i] = direction[i];
        dio_direction_bit_array = dio_direction_bit_array | (direction[i] << i);
    }
    e = add_system_parameters_in_queue(0x0011,dio_direction_bit_array);
    return e;
}

esmacat_err actuator_controller_interface::configure_pwm_frequency_2nd_actuator(uint16_t pwm_frq)
{
    esmacat_err e = ERR_UNKNOWN;
    e = add_system_parameters_in_queue(0x0201,pwm_frq);
    return e;
}

esmacat_err actuator_controller_interface::configure_pwm_duty_cycle_2nd_actuator(float pwm_duty_cycle_0_to_1)
{
    esmacat_err e,e1,e2,e3 = ERR_UNKNOWN;
    uint16_t pwm_duty_cycle_0_to_10000;
    if (pwm_duty_cycle_0_to_1 > 1.0 )   // if pwm is higher than 1
    {
        pwm_duty_cycle_0_to_1 = 1.0;
        e1 = ERR_SEA_SECOND_ACTUATOR_MAX_PWM_REACHED;
    }
    else e1 = NO_ERR;

    if ( pwm_duty_cycle_0_to_1 < 0.0)   // if pwm is lower than 0
    {
        pwm_duty_cycle_0_to_1 = 0.0;
        e2 = ERR_SEA_SECOND_ACTUATOR_MIN_PWM_REACHED;
    }
    else e2 = NO_ERR;

    pwm_duty_cycle_0_to_10000 = static_cast<uint16_t>(pwm_duty_cycle_0_to_1 * ACTUATOR_MAX_PWM_OUTPUT);   // convert its range 0-1 to 0-1000
    e3 = add_system_parameters_in_queue(0x0613,pwm_duty_cycle_0_to_10000);

    // find the combination of the errors.
    if (e1 != NO_ERR )
    {
        e=e1;
    }
    if (e2 != NO_ERR)
    {
        if( e == ERR_UNKNOWN) e=e2;
        else e = ERR_SEA_SECOND_ACTUATOR_MULTIPLE_PWM_ERROR;
    }
    if (e3 != NO_ERR)
    {
        if( e == ERR_UNKNOWN) e=e3;
        else e = ERR_SEA_SECOND_ACTUATOR_MULTIPLE_PWM_ERROR;
    }
    if (e1 == NO_ERR && e2== NO_ERR && e3==NO_ERR ) e=NO_ERR;
    return e;
}

esmacat_err actuator_controller_interface::configure_incremental_encoder_resolution_cpt(uint16_t resolution_cpt)
{
    return add_system_parameters_in_queue(0x0619,resolution_cpt);
}

esmacat_err actuator_controller_interface::configure_motor_rotor_inertia_g_per_cm2(float inertia)
{
    return add_system_parameters_in_queue(0x0620,convert_float_to_ecat_format(inertia));
}

esmacat_err actuator_controller_interface::configure_gearhead_rotor_inertia_g_per_cm2(float inertia)
{
    return add_system_parameters_in_queue(0x0621,convert_float_to_ecat_format(inertia));
}

esmacat_err actuator_controller_interface::configure_velocity_low_pass_filter_weight_for_current_measure(float weight_for_now)
{
    if (weight_for_now > 1.0 )
    {
        weight_for_now = 1.0;
        PLOGW << "velocity_low_pass_filter_weight_for_current_measure needs to be between 0 and 1";
    }
    if (weight_for_now < 0.0 )
    {
        weight_for_now = 0.0;
        PLOGW << "velocity_low_pass_filter_weight_for_current_measure needs to be between 0 and 1";
    }

    return add_system_parameters_in_queue(0x0622,convert_float_to_ecat_format(weight_for_now));
}

esmacat_err actuator_controller_interface::configure_loadcell_low_pass_filter_weight_for_current_measure(float weight_for_now)
{
    if (weight_for_now > 1.0 )
    {
        weight_for_now = 1.0;
        PLOGW << "velocity_low_pass_filter_weight_for_current_measure needs to be between 0 and 1";
    }
    if (weight_for_now < 0.0 )
    {
        weight_for_now = 0.0;
        PLOGW << "velocity_low_pass_filter_weight_for_current_measure needs to be between 0 and 1";
    }

    return add_system_parameters_in_queue(0x0611,convert_float_to_ecat_format(weight_for_now));
}

esmacat_err actuator_controller_interface::configure_gain_inertia_dynamics_compensation(float gain)
{
    return add_system_parameters_in_queue(0x0623,convert_float_to_ecat_format(gain));
}

esmacat_err actuator_controller_interface::configure_max_integrated_torque_error_mNm(float max_ie)
{
    return add_system_parameters_in_queue(0x0625,convert_float_to_ecat_format(max_ie));
}

esmacat_err actuator_controller_interface::configure_escon_analog_output0_voltage_V_to_current_A_offset(float offset)
{
    return add_system_parameters_in_queue(0x0626,convert_float_to_ecat_format(offset));
}

esmacat_err actuator_controller_interface::configure_escon_analog_output0_voltage_V_to_current_A_slope(float slope)
{
    return add_system_parameters_in_queue(0x0627,convert_float_to_ecat_format(slope));
}

esmacat_err actuator_controller_interface::configure_escon_analog_output1_velocity_V_to_current_rpm_offset(float offset)
{
    return add_system_parameters_in_queue(0x0628,convert_float_to_ecat_format(offset));
}

esmacat_err actuator_controller_interface::configure_escon_analog_output1_velocity_V_to_current_rpm_slope(float slope)
{
    return add_system_parameters_in_queue(0x0629,convert_float_to_ecat_format(slope));
}

esmacat_err actuator_controller_interface::configure_max_allowable_redundancy_error_for_motor_current_mA(float slope)
{
    return add_system_parameters_in_queue(0x062A,convert_float_to_ecat_format(slope));
}

esmacat_err actuator_controller_interface::configure_max_allowable_redundancy_error_for_motor_velocity_rad_per_sec(float slope)
{
    return add_system_parameters_in_queue(0x062B,convert_float_to_ecat_format(slope));
}


/** @brief Clears the encoder [Do not change]
 *
 * This function should not be modified since it is tied to the firmware
 * for the slave
 */
void actuator_controller_interface::configure_slave_encoder_clear()
{
    add_system_parameters_in_queue(0x0500,0);
}

/** @brief Queues the escon setpoint sign to configure the slave */
void actuator_controller_interface::configure_escon_setpoint_sign (int sign)
{
    add_system_parameters_in_queue(0x0602,static_cast<uint16_t>(sign));
}
/** @brief Queues the Zero offset of the loadcell to configure the slave */
void actuator_controller_interface::configure_loadcell_sign (int loadcell_sign)
{
    add_system_parameters_in_queue(0x0603,static_cast<uint16_t>(loadcell_sign));
}

/** @brief Queues the Zero offset of the loadcell to configure the slave */
void actuator_controller_interface::configure_loadcell_zero_offset (float loadcell_offset_mV)
{
    add_system_parameters_in_queue(0x0605,convert_float_to_ecat_format(loadcell_offset_mV));
}

/** @brief Queues the calibration parameter of the loadcell to configure the slave */
void actuator_controller_interface::configure_loadcell_calibration(float loadcell_calibration_mV_to_mNm)
{
    add_system_parameters_in_queue(0x0606,convert_float_to_ecat_format(loadcell_calibration_mV_to_mNm));
}

/** @brief Queues the Torque constant of the motor to configure the slave */
void actuator_controller_interface::configure_torque_constant (float torque_constant_mNm_per_mA)
{
    add_system_parameters_in_queue(0x0607,convert_float_to_ecat_format(torque_constant_mNm_per_mA));
}

/** @brief Queues the gear ratio of the motor to configure the slave */
void actuator_controller_interface::configure_gear_ratio (uint16_t gear_ratio)
{
    add_system_parameters_in_queue(0x0608,gear_ratio);
}

/** @brief Queues the gear power efficiency of the motor to configure the slave */
void actuator_controller_interface::configure_gear_power_efficiency (float gear_power_efficiency)
{
    add_system_parameters_in_queue(0x0609,convert_float_to_ecat_format(gear_power_efficiency));
}

/** @brief Queues the conversion factor of the ESCON (mA to setpoint) to configure the slave */
void actuator_controller_interface::configure_escon_conversion_factor(float current_conversion_factor_mA_to_setpoint)
{
    add_system_parameters_in_queue(0x060A,convert_float_to_ecat_format(current_conversion_factor_mA_to_setpoint));
}

/** @brief Queues the control mode of the slave */
void actuator_controller_interface::configure_control_type (control_mode_t mode)
{
    add_system_parameters_in_queue(0x0601,static_cast<uint16_t>(mode));
}

/** @brief Queues the maximum allowable torque  */
void actuator_controller_interface::configure_max_torque_change_mNm_per_ms(uint16_t max_torque_change_mNm_per_interval)
{
    add_system_parameters_in_queue(0x0604,max_torque_change_mNm_per_interval);
}

/** @brief Queues the proportional gain of the torque control to be sent to the slave */
void actuator_controller_interface::configure_torque_control_p_gain(float torque_control_p_gain)
{
    add_system_parameters_in_queue(0x060B,convert_float_to_ecat_format(torque_control_p_gain));
}

/** @brief Queues the Integral gain of the torque control to be sent to the slave */
void actuator_controller_interface::configure_torque_control_i_gain(float torque_control_i_gain)
{
    add_system_parameters_in_queue(0x060C,convert_float_to_ecat_format(torque_control_i_gain));
}

/** @brief Queues the Derivative gain of the torque control to be sent to the slave */
void actuator_controller_interface::configure_torque_control_d_gain(float torque_control_d_gain)
{
    add_system_parameters_in_queue(0x060D,convert_float_to_ecat_format(torque_control_d_gain));
}
/** @brief Sets the time period for the slave application */
void actuator_controller_interface::set_one_cycle_time_s(float time_period_s)
{
    esmacat_app_one_cycle_time_sec = time_period_s;
}


/** @brief Reads the value of the specified input signal
 *
 * Has error handling- ERR_BASE_MODULE_DIO_INDEX_OUT_OF_RANGE if input index is out of range,
 * ERR_BASE_MODULE_DIO_DIRECTION_NOT_input if value is being read is not an input,
 * NO_ERR indicates successful completion
 *
 * @param index_of_digital_input Index of the digital output signal to be assigned
 * @param error Status of the function (ERR_BASE_MODULE_DIO_INDEX_OUT_OF_RANGE,
 * ERR_BASE_MODULE_DIO_DIRECTION_NOT_INPUT, NO_ERR)
 * @return Boolean value read at the specified index
 */
bool actuator_controller_interface::get_digital_input(int index_of_digital_input, esmacat_err& error)
{
    if (index_of_digital_input < 0 || index_of_digital_input >ACTUATOR_CONTROLLER_NUMBER_OF_DIO)
    {
        error = ERR_BASE_MODULE_DIO_INDEX_OUT_OF_RANGE;
        return 0;
    }
    else
    {
        if (dio_direction[index_of_digital_input] != IO_INPUT )
        {
            error = ERR_BASE_MODULE_DIO_DIRECTION_NOT_INPUT;
            return 0;
        }
        else
        {
            return general_purpose_input[index_of_digital_input];
            error = NO_ERR;
        }
    }
}


/** @brief Reads the value of the specified input signal
 *
 * No error handling
 *
 * @param index_of_digital_input Index of the digital output signal to be assigned
 * @return Boolean value read at the specified index
 */
bool actuator_controller_interface::get_digital_input(int index_of_digital_input)
{
    esmacat_err error = NO_ERR;
    return get_digital_input(index_of_digital_input, error);
}

esmacat_err actuator_controller_interface::set_current_loop_cnt(uint64_t current_loop_cnt)
{
    communication_timeout_counter = (current_loop_cnt & 0xffff); // send only the 16 bit LSB
    return NO_ERR;
}

/** @brief Assigns the the specified digital output boolean value to the specified output signal
 *
 * Has error handling- ERR_BASE_MODULE_DIO_INDEX_OUT_OF_RANGE if input index is out of range,
 * ERR_BASE_MODULE_DIO_DIRECTION_NOT_OUTPUT if value is being assigned to an input not output,
 * NO_ERR indicates successful completion
 *
 * @param index_of_digital_output Index of the digital output signal to be assigned
 * @param digital_output_value Boolean value to be assigned to the digital output signal
 * @return Error status of the function (ERR_BASE_MODULE_DIO_INDEX_OUT_OF_RANGE,
 * ERR_BASE_MODULE_DIO_DIRECTION_NOT_OUTPUT, NO_ERR)
 */
esmacat_err actuator_controller_interface::set_digital_output(int index_of_digital_output, bool digital_output_value)
{
    esmacat_err error;
    if (index_of_digital_output < 0 || index_of_digital_output>ACTUATOR_CONTROLLER_NUMBER_OF_DIO)
    {
        error = ERR_BASE_MODULE_DIO_INDEX_OUT_OF_RANGE;
    }
    else
    {
        if (dio_direction[index_of_digital_output] != IO_OUTPUT )
        {
            error = ERR_BASE_MODULE_DIO_DIRECTION_NOT_OUTPUT;
        }
        else
        {
            general_purpose_output[index_of_digital_output] = digital_output_value;
            error = NO_ERR;
        }
    }
    return error;
}

// to be deleted
/** @brief Private Function- Reads the specified raw analog input from external source
 * @param Index of the analog input to be read
 * @return Reading of the specified analog input
 */
uint16_t actuator_controller_interface::get_raw_linear_actuator_reading(void)
{
    return linear_actuator_reading_raw;
}

/** @brief Reads analog input value in mV and provides error handling
 *
 * @param analog_input_index Index of the ESCON analog input to be read
 * @param error Contains the error message to demonstrate success of data acquisition
  * (ERR_SEA_DRIVER_ESCON_ANALOG_INPUT_INDEX_OUT_OF_RANGE or NO_ERR)
 * @return Voltage reading (mV) of the selected ESCON analog input
 */
float actuator_controller_interface::get_analog_input_from_escon_mV(int analog_input_index, esmacat_err& error)
{
    float adc_reading_mV = 0;
    if (analog_input_index > ACTUATOR_CONTROLLER_NUM_ANALOG_INPUT || analog_input_index < 0)
    {
        error = ERR_SEA_DRIVER_ESCON_ANALOG_INPUT_INDEX_OUT_OF_RANGE;
        adc_reading_mV = 0;
    }
    else
    {
        error = NO_ERR;
        adc_reading_mV = static_cast<float>(SEA_ADC_OFFSET_MV + SEA_ADC_FSR_MV * static_cast<double>(get_raw_escon_analog_input(analog_input_index))/static_cast<uint16_t>(UINT16_MAX));
    }
    return adc_reading_mV;
}

/** @brief Reads analog input value in mV and provides error handling
 *
 * @param analog_input_index Index of the external source analog input to be read
 * @param error Contains the error message to demonstrate success of data acquisition
  * (ERR_SEA_DRIVER_EXT_ANALOG_INPUT_INDEX_OUT_OF_RANGE or NO_ERR)
 * @return Voltage reading (mV) of the selected external source analog input
 */
float actuator_controller_interface::get_linear_actuator_reading_mV(esmacat_err& error)
{
    float adc_reading_mV = 0;
    error = NO_ERR;
    adc_reading_mV = static_cast<float>(SEA_ADC_OFFSET_MV + SEA_ADC_FSR_MV * static_cast<double>(get_raw_linear_actuator_reading())/static_cast<double>(UINT16_MAX));
    return adc_reading_mV;
}
