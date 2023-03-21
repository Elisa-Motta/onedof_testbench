/** @file
 * @brief Contains declarations required for the Series-Elastic Actuator (SEA) Driver
*/
#ifndef LENGTH_ACQUISITION_H
#define LENGTH_ACQUISITION_H

/*****************************************************************************************
 * MACROS
 ****************************************************************************************/
/** @brief Unique vendor ID assigned by the EtherCAT Technology Group to Harmonic Bionics Inc.*/
#define SEA_DRIVER_VENDOR_ID        0x0062696f

/** @brief Unique product code for the SEA slave v2 required for EtherCAT */
#define SEA_DRIVER_PRODUCT_ID       static_cast<uint32_t>(0x00020401)

/** @brief Unique product code for the SEA legacy slave required for EtherCAT */
#define SEA_DRIVER_LEGACY_PRODUCT_ID       static_cast<uint32_t>(0x00020301)

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
#define ACTUATOR_NUM_ANALOG_INPUT     2

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "error_list.h"
#include "actuator_controller_interface.h"


/*****************************************************************************************
 * STRUCTS
 ****************************************************************************************/

/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/

class length_acquisition
{
private:
    /** Offset to be applied to the linear actuator reading in mV */
    float linear_actuator_offset_mV;
    /** Factor that translates the linear actuator reading in mV to load reading in mm */
    float linear_actuator_calibration_mV_to_mm;

public:
    length_acquisition();
    float get_linear_actuator_length_mm(esmacat_err& error, actuator_controller_interface* controller);
    void set_linear_actuator_zero_offset (float offset);
    void set_linear_actuator_calibration (float calibration);

};
#endif // LENGTH_ACQUISITION_H
