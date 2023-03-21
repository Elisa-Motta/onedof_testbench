#ifndef ESMACAT_EPOS4_H
#define ESMACAT_EPOS4_H

#define	ESMACAT_EPOS4_VENDOR_ID	0x000000fb
#define	ESMACAT_EPOS4_PRODUCT_ID	0x61500000

#include "slave.h"
#include <iostream>
#include <math.h>

class esmacat_epos4: public esmacat_slave
{
private:

    // PDO mapping according to CST mode

    // TxPDO from driver
    uint16_t input_statusword;
    int32_t input_encoder_counter;
    int32_t input_encoder_velocity;
    int16_t input_motor_torque;
    int8_t input_mode_operation;

    // RxPDO to driver
    uint16_t output_controlword = 0;
    int16_t output_motor_target_torque = 0;
    int16_t output_motor_offset_torque = 0;
    int8_t output_mode_operation = 0;

protected:
    float esmacat_app_one_cycle_time_sec;

public:
    /** Contains a product code of the EPOS4 slave */
    uint32_t esmacat_slave_product_id = ESMACAT_EPOS4_PRODUCT_ID;
    /** Contains a vendor ID assigned by the EtherCAT Technology Group*/
    uint32_t esmacat_slave_vendor_id = ESMACAT_EPOS4_VENDOR_ID;
    esmacat_epos4();

    uint32_t get_esmacat_product_id(){return esmacat_slave_product_id;}
    uint32_t get_esmacat_vendor_id(){return esmacat_slave_vendor_id;}
    void ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop);

    // TxPDO from driver
    uint16_t  get_statusword();
    int32_t   get_encoder_counter();
    int32_t   get_encoder_filt_speed();
    int16_t   get_motor_filt_torque();

    double    get_position();

    // RxPDO to driver
    esmacat_err set_controlword(uint16_t controlword);
    esmacat_err set_target_torque(int16_t target_torque);
    esmacat_err set_offset_torque(int16_t offset_torque);
    esmacat_err set_mode_operation(int8_t mode_operation);

    // High Level Functions

    esmacat_err reset_fault();
    esmacat_err stop_motor();
    esmacat_err start_motor();

}; 

#endif
