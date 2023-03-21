/** @file
 * @brief Contains definitions of functions used for the Series-Elastic Actuator (SEA) Driver
*/

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "length_acquisition.h"

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

length_acquisition::length_acquisition()
{
    linear_actuator_offset_mV = 0;
    linear_actuator_calibration_mV_to_mm = 0;
}


/** @brief Computes the length traveled by the linear actuator in mm
 * Uses the offset and calibration settings to convert the adc input reading in mV
 * to a length in mm
 *
 * @return Length traveled by the linear actuator in mm
 */
float length_acquisition::get_linear_actuator_length_mm(esmacat_err& error, actuator_controller_interface* controller)
{
    float linear_actuator_length_mm = (controller->get_linear_actuator_reading_mV(error)-\
                                 linear_actuator_offset_mV)*linear_actuator_calibration_mV_to_mm;
    return linear_actuator_length_mm;
}

void length_acquisition::set_linear_actuator_zero_offset (float offset)
{
    linear_actuator_offset_mV = offset;
}
void length_acquisition::set_linear_actuator_calibration (float calibration)
{
    linear_actuator_calibration_mV_to_mm = calibration;
}

