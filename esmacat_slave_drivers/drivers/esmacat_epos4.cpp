
/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "esmacat_epos4.h"
#include "master.h"
#include <string>
#include <sstream>

esmacat_epos4::esmacat_epos4(){

    std::cout <<"Esmacat EPOS 4 Driver Object is created" << std::endl;

    esmacat_slave_vendor_id = ESMACAT_EPOS4_VENDOR_ID;
    esmacat_slave_product_id = ESMACAT_EPOS4_PRODUCT_ID;

    input_statusword = 0;
    input_encoder_counter = 0;
    output_mode_operation = 0;

//    esmacat_app_one_cycle_time_sec = (float)/1000000000; // need to be updated
//    std::cout << "Control Period: " << esmacat_app_one_cycle_time_sec << " s" << std::endl;

}

/***************************
 *        INPUTS           *
 ***************************/


uint16_t esmacat_epos4::get_statusword()
{
    return input_statusword;
}

int32_t esmacat_epos4::get_encoder_counter()
{
    return input_encoder_counter;
}

int32_t esmacat_epos4::get_encoder_filt_speed()
{
    return input_encoder_velocity;
}

int16_t esmacat_epos4::get_motor_filt_torque()
{
    return input_motor_torque;
}

double esmacat_epos4::get_position()
{
    return (double)(input_encoder_counter/8192.0*360.0);
}

/***************************
 *        OUTPUTS          *
 ***************************/

esmacat_err esmacat_epos4::set_controlword(uint16_t controlword){
    output_controlword = controlword;
    return NO_ERR;
}

esmacat_err esmacat_epos4::set_mode_operation(int8_t mode_operation){
    output_mode_operation = mode_operation;
    return NO_ERR;
}

esmacat_err esmacat_epos4::set_target_torque(int16_t target_torque){
    output_motor_target_torque = target_torque;
    return NO_ERR;
}

esmacat_err esmacat_epos4::set_offset_torque(int16_t offset_torque){
    output_motor_offset_torque = offset_torque;
    return NO_ERR;
}

esmacat_err esmacat_epos4::reset_fault(){
    set_controlword(128);
    return NO_ERR;

}

esmacat_err esmacat_epos4::stop_motor(){
    set_controlword(6);
    return NO_ERR;

}

esmacat_err esmacat_epos4::start_motor(){
    set_controlword(15);
    return NO_ERR;

}

void esmacat_epos4::ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop){

    unsigned char input_variable[17];
    unsigned char output_variable[11];

    /* TxPDO CST */

    // Statusword
    // Index: 0x6041-00
    // Type: UNSIGNED16
    input_variable[0] = *(ec_slave_inputs+0);
    input_variable[1] = *(ec_slave_inputs+1);
    input_statusword = +(input_variable[0] << 0)+(input_variable[1] << 8);

    // Position actual value
    // Index: 0x6064-00
    // Type: INTEGER32
    input_variable[2] = *(ec_slave_inputs+2);
    input_variable[3] = *(ec_slave_inputs+3);
    input_variable[4] = *(ec_slave_inputs+4);
    input_variable[5] = *(ec_slave_inputs+5);
    input_encoder_counter = +(input_variable[2] << 0)+(input_variable[3] << 8)+(input_variable[4] << 16)+(input_variable[5] << 24);

    // Velocity actual value
    // Index: 0x606C-00
    // Type: INTEGER32
    input_variable[6] = *(ec_slave_inputs+6);
    input_variable[7] = *(ec_slave_inputs+7);
    input_variable[8] = *(ec_slave_inputs+8);
    input_variable[9] = *(ec_slave_inputs+9);
    input_encoder_velocity = +(input_variable[6] << 0)+(input_variable[7] << 8)+(input_variable[8] << 16)+(input_variable[9] << 24);

    // Torque actual value (MotorRatedTorque/1000)
    // Index: 0x6077-00
    // Type: INTEGER16
    input_variable[10] = *(ec_slave_inputs+10);
    input_variable[11] = *(ec_slave_inputs+11);
    input_motor_torque = +(input_variable[10] << 0)+(input_variable[11] << 8);

    // Mode of Operation
    // Index: 0x6061-00
    // Type: INTEGER8
    input_variable[12] = *(ec_slave_inputs+12);
    input_mode_operation = +(input_variable[12] << 0);

    // Digital Inputs
    // Index: 0x60FD-00
    // Type: UNSIGNED32
    input_variable[13] = *(ec_slave_inputs+13);
    input_variable[14] = *(ec_slave_inputs+14);
    input_variable[15] = *(ec_slave_inputs+15);
    input_variable[16] = *(ec_slave_inputs+16);

    /* RxPDO CST */

    // Set Controlword
    // Index: 0x6040-00
    *(ec_slave_outputs+0)  = (output_controlword & 0x000000ff) >> 0;
    *(ec_slave_outputs+1)  = (output_controlword & 0x0000ff00) >> 8;

    // Set Target torque
    // Index: 0x6071-00
    *(ec_slave_outputs+2) = (output_motor_target_torque & 0x000000ff) >> 0;
    *(ec_slave_outputs+3) = (output_motor_target_torque & 0x0000ff00) >> 8;

    // Set Offset torque
    // Index: 0x60B2-00
    *(ec_slave_outputs+4) = (output_motor_offset_torque & 0x000000ff) >> 0;
    *(ec_slave_outputs+5) = (output_motor_offset_torque & 0x0000ff00) >> 8;

    // Set Mode of Operation
    // Index: 0x6060-00
    *(ec_slave_outputs+6) = (output_mode_operation & 0x000000ff) >> 0;

    // Set Digital Outputs
    // Index: 0x60FE-01
    *(ec_slave_outputs+7) = 0;
    *(ec_slave_outputs+8) = 0;
    *(ec_slave_outputs+9) = 0;
    *(ec_slave_outputs+10) = 0;
}

