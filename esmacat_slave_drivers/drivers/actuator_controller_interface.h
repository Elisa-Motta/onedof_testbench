/** @file
 * @brief Contains declarations required for the Acutator controller interface. actuator_controller_interface is an EtherCAT interface for Actuator controller
*/
#ifndef ACTUATOR_CONTROLLER_INTERFACE_H
#define ACTUATOR_CONTROLLER_INTERFACE_H

/*****************************************************************************************
 * MACROS
 ****************************************************************************************/
/** @brief Unique vendor ID assigned by the EtherCAT Technology Group to Harmonic Bionics Inc.*/
#define ACTUATOR_CONTROLLER_VENDOR_ID        0x0062696f

/** @brief Unique product code for the SEA slave v2 required for EtherCAT */
#define ACTUATOR_CONTROLLER_PRODUCT_ID       static_cast<uint32_t>(0x00020401)

/** @brief Unique revision number for the Actuator Controller required for EtherCAT */
#define ACTUATOR_CONTROLLER_REVISION       static_cast<uint32_t>(0x3a30)

/** @brief Unique product code for the SEA legacy slave required for EtherCAT */
#define ACTUATOR_CONTROLLER_LEGACY_PRODUCT_ID       static_cast<uint32_t>(0x00020301)

/** @brief Offset voltage (in mV) for the ADC on the SEA */
#define SEA_ADC_OFFSET_MV   -1.5*4096.0

/** @brief Full-scale range (in mV) of the ADC on the SEA */
#define SEA_ADC_FSR_MV      3.0*4096.0

/** @brief Minimum value for the PWM output */
#define ACTUATOR_MIN_PWM_OUTPUT      0

/** @brief Maximum value for the PWM output */
#define ACTUATOR_MAX_PWM_OUTPUT      10000

/** @brief Minimum ESCON current setpoint value */
#define ACTUATOR_MIN_CURRENT_SETPOINT       -1

/** @brief Maximum ESCON current setpoint value */
#define ACTUATOR_MAX_CURRENT_SETPOINT       1

/** @brief Maximum number of analog input sources (ESCON) for the slave */
#define ACTUATOR_CONTROLLER_NUM_ANALOG_INPUT     2

#define ACTUATOR_CONTROLLER_NUMBER_OF_DIO   5

#define LOOP_INDEX_FOR_STARTING_TO_READ_FEEDBACK_VALUES_SEQUENCIALLY 2000

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "application.h"
#include "error_list.h"
#include <vector>

/*****************************************************************************************
 * STRUCTS
 ****************************************************************************************/
struct actuator_controller_feedback{
    uint16_t feedback_type = 0;
    string feedback_label = "";
    uint16_t feedback_value = 0;
    int feedback_datatype_0int_1float = 0;
};

/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/
/** @brief EtherCAT interfce for actuator controller*/
class actuator_controller_interface : public esmacat_slave
{
public:
    /** enum for control mode of actuator controller */
    enum class control_mode_t { direct_escon_control, torque_control =  2};
private:
    // EtherCAT Packet input
    /** EtherCAT packet input, actuator controller status */
    uint16_t actuator_controller_status = 0;
    /** EtherCAT packet input, loadcell reading unsigned16 bit raw value*/
    uint16_t loadcell_reading_raw = 0;
    /** EtherCAT packet input, Input raw readings from the absolute encoder in counts per turn:
     * absolute encoder data, range is 0 - 4095 for 360 deg */
    uint16_t absolute_encoder_reading_raw_cpt  = 0;
    /** EtherCAT packet input, Input from the LS7366 quadrature encoder reader */
    int32_t incremental_encoder_reading_raw_cpt = 0;
    /** EtherCAT packet input, General purpose input. general_purpose_input[0-4] are reserved for GPIO 0-4, currently 0-1 are only available */
    bool general_purpose_input[8] = {0,0,0,0,0,0,0,0};

    // EtherCAT Packet output
    /** EtherCAT packet output, escon enable */
    bool escon_enable = 0;
    /** EtherCAT packet output, general purpose output [0-4] are reserved for GPIO[0-4]. Currently 0-1 are only available. General_purpose_output[5] is reserved to control user status led */
    bool general_purpose_output[7] = {0,0,0,0,0,0,0};
    /** EtherCAT packet output, escon derating factor derates the setpoint value of ESCON*/
    float escon_derating_factor = 0;
    /** EtherCAT packet output, master software its loop_cnt via communication_timeout_counter */
    uint16_t communication_timeout_counter = 0;
    /** EtherCAT packet output, Contains the setpoint commanded for torque/position/current
     * control; interpreted based on control_mode */
    float control_setpoint = 0;

    // internal state variables
    /** DIO directions for actuator controller digital signals*/
    IO_Direction dio_direction[ACTUATOR_CONTROLLER_NUMBER_OF_DIO] = {IO_OUTPUT,IO_OUTPUT,IO_OUTPUT,IO_OUTPUT,IO_OUTPUT};
    /** Time interval for the loop computations */
    float esmacat_app_one_cycle_time_sec = 0;
    /** The setpoint for the ESCON is normalized, this is the conversion factor to
     * convert the setpoint back to mA*/
    float current_conversion_factor_mA_to_setpoint = 0;
    /** Current state of the status led  */
    bool user_status_led = false;
    /** Process converting the eterhcat packet to a human-readable variables */
    void ecat_data_process_actuator_controller_interface(uint8_t* ec_slave_outputs,uint8_t* ec_slave_inputs);

//    std::vector<actuator_controller_feedback> feedback_list = { {0x0614,"raw adc potentiometer linear act",0,0},
//                                                                {0x0615,"raw adc escon AO0",0,0},
//                                                                {0x0616,"raw adc escon AO1",0,0},
//                                                                {0x0617,"motor velocity rad/s",0,1},
//                                                                {0x0618,"motor acceleration rad/s2",0,1},
//                                                                {0x0624,"escon setvalue 0to1",0,1},
//                                                                {0x062C,"torque error mNm",0,1},
//                                                                {0x062D,"differntial torque error mNm/s",0,1},
//                                                                {0x062E,"integrated error mNm sec",0,1},
//                                                              };

    std::vector<actuator_controller_feedback> feedback_list;

    // to be deleted
    /** Potential meter value in linear actuator. Unit is raw*/
    uint16_t linear_actuator_reading_raw = 0;
    /** get raw potentimetor value of the linear actuator */
    uint16_t get_raw_linear_actuator_reading();
    /** Array containing raw readings for 2 analog inputs from ESCON;
     *     /** get raw escon analog input value. choose between two channels of escon output */
    uint16_t get_raw_escon_analog_input(int ai_index);
    float acceleration_counts_per_sec_square;
    float aux_setpoint = 0;

public:

    esmacat_err set_aux_setpoint(float aux);

    /** constructor of actuator controller interface */
    actuator_controller_interface();

    /** Process converting the eterhcat packet to a human-readable variables */
    void ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop);

    // get EtherCAT packet input
    /** get raw loadcell reading from actuator controller */
    uint16_t get_raw_loadcell_reading();
    /** read the load cell reading in mV */
    float get_loadcell_reading_mV(esmacat_err& error);
    uint16_t get_raw_absolute_encoder_reading_cpt();
    int32_t get_raw_incremental_encoder_reading_cpt();
    /** Reads in the fault value output of the ESCON received over EtherCAT*/
    bool get_escon_fault();
    bool get_digital_input(int index_of_digital_input, esmacat_err& error);
    bool get_digital_input(int index_of_digital_input);
    float get_esmacat_app_one_cycle_time_sec();

    // public set functions
    void set_escon_enable(bool enable_on);
    void set_user_status_led(bool turn_on);
    void set_current_conversion_factor(float conversion_factor);
    void set_one_cycle_time_s(float time_period_s);
    esmacat_err set_digital_output(int digital_output_index, bool digital_output_value);
    esmacat_err set_control_setpoint_0to1(float setpoint);
    esmacat_err set_control_setpoint(float setpoint);
    esmacat_err set_escon_derating_factor(float factor);
    esmacat_err set_current_loop_cnt(uint64_t current_loop_cnt);

    // public configure functions
    esmacat_err configure_dio_direction(IO_Direction* direction);
    esmacat_err configure_pwm_frequency_2nd_actuator(uint16_t pwm_frq);
    esmacat_err configure_pwm_duty_cycle_2nd_actuator(float pwm_duty_cycle_0_to_1);
    esmacat_err configure_incremental_encoder_resolution_cpt(uint16_t resolution_cpt);
    esmacat_err configure_velocity_low_pass_filter_weight_for_current_measure(float weight_for_now);
    esmacat_err configure_loadcell_low_pass_filter_weight_for_current_measure(float weight_for_now);
    esmacat_err configure_gain_inertia_dynamics_compensation(float gain);
    esmacat_err configure_max_integrated_torque_error_mNm(float max_ie);
    esmacat_err configure_escon_analog_output0_voltage_V_to_current_A_offset(float offset);
    esmacat_err configure_escon_analog_output0_voltage_V_to_current_A_slope(float slope);
    esmacat_err configure_escon_analog_output1_velocity_V_to_current_rpm_offset(float offset);
    esmacat_err configure_escon_analog_output1_velocity_V_to_current_rpm_slope(float slope);
    esmacat_err configure_max_allowable_redundancy_error_for_motor_current_mA(float slope);
    esmacat_err configure_max_allowable_redundancy_error_for_motor_velocity_rad_per_sec(float slope);

    void configure_slave_encoder_clear();
    void configure_escon_setpoint_sign (int sign);
    void configure_loadcell_sign (int loadcell_sign);
    void configure_loadcell_zero_offset (float loadcell_offset_mV);
    void configure_loadcell_calibration(float loadcell_calibration_mV_to_mNm);
    void configure_torque_constant (float torque_constant_mNm_per_mA);
    void configure_gear_ratio (uint16_t gear_ratio);
    void configure_gear_power_efficiency (float gear_power_efficiency);
    void configure_escon_conversion_factor(float current_conversion_factor_mA_to_setpoint);
    void configure_control_type(control_mode_t mode);
    void configure_max_torque_change_mNm_per_ms(uint16_t max_torque_change_mNm_per_interval);
    void configure_torque_control_p_gain (float torque_control_p_gain);
    void configure_torque_control_i_gain (float torque_control_i_gain);
    void configure_torque_control_d_gain (float torque_control_d_gain);

    // not being used in the firmware controller, but having functions for future
    esmacat_err configure_motor_rotor_inertia_g_per_cm2(float inertia);
    esmacat_err configure_gearhead_rotor_inertia_g_per_cm2(float inertia);

    esmacat_err read_feedback_variables_sequentially(uint64_t loop_cnt, bool flag_print);

    /** read the potentiometer value of linear actuator in mV */
    float get_linear_actuator_reading_mV(esmacat_err& error);
    float get_velocity_counts_per_sec();
    float get_acceleration_counts_per_sec_square();
    float  get_analog_input_from_escon_mV(int analog_input_index, esmacat_err& error);


};
#endif // ACTUATOR_CONTROLLER_INTERFACE_H
