/** @file
 * @brief Contains definitions of functions used for the Series-Elastic Actuator (SEA) Driver
*/

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "load_acquisition.h"

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

load_acquisition::load_acquisition()
{

    loadcell_offset_mV = 0;
    loadcell_calibration_mV_to_mNm = 0;
    loadcell_sign = 1;
    loadcell_loop_flag = 0;
    filtered_load_mNm = 0;
}
/** @brief Computes the loadcell reading of the joint in milli-Nm
 *
 * @param Contains the error message to demonstrate success of data acquisition
 * If a loadcell calibration value was not set, ERR_SEA_LOADCELL_PARAMETER_NOT_LOADED
 * is returned; else NO_ERR is returned to indicate successful calculation
 * @return Loadcell reading in milli-Nm
 */
float load_acquisition::get_load_mNm(esmacat_err& error, actuator_controller_interface* controller)
{
    error = NO_ERR;
    float measured_load_cell_voltage_mV = loadcell_sign* controller->get_loadcell_reading_mV(error);
    if ( loadcell_calibration_mV_to_mNm == static_cast<float>(0.0))
    {
        error = ERR_SEA_LOADCELL_PARAMETER_NOT_LOADED;
    }
    float load_mNm = (measured_load_cell_voltage_mV -  loadcell_offset_mV) * loadcell_calibration_mV_to_mNm;
    return load_mNm;
}

/** @brief Computes the filtered loadcell reading of the joint in milli-Nm
 *
 * Obtains the current unfiltered loadcell reading in milli-Nm and applies an
 * IIR low-pass-filter to it. The equation for a LPF with input x and output y is
 *
 * y [n] = m * y[n-1] + (1-m) * x[n]
 * @param Contains the error message to demonstrate success of data acquisition
 * The error from reading the unfiltered loadcell reading is passed through. Set to
 * NO_ERR for successful computation.
 * @return Filtered Loadcell reading in milli-Nm
 */
float load_acquisition::get_filtered_load_mNm(esmacat_err& error, actuator_controller_interface* controller)
{
    float unfiltered_load_reading_mNm = get_load_mNm(error, controller);
    if (error == NO_ERR)
    {
        if (loadcell_loop_flag == 0)
        {
            filtered_load_mNm = unfiltered_load_reading_mNm;
        }
        else
        {
            filtered_load_mNm = filtered_load_mNm*static_cast<float>(0.99) + unfiltered_load_reading_mNm*static_cast<float>(0.01);
        }
        loadcell_loop_flag++;
    }
    return filtered_load_mNm;
}
/** @brief Queues the Zero offset of the loadcell to configure the slave */
void load_acquisition::set_loadcell_sign (int sign, actuator_controller_interface* controller)
{
    loadcell_sign = sign;
    controller->configure_loadcell_sign(loadcell_sign);
}

/** @brief Queues the Zero offset of the loadcell to configure the slave */
void load_acquisition::set_loadcell_zero_offset (float offset, actuator_controller_interface* controller)
{
    loadcell_offset_mV = offset;
    controller->configure_loadcell_zero_offset(loadcell_offset_mV);
}

/** @brief Queues the calibration parameter of the loadcell to configure the slave */
void load_acquisition::set_loadcell_calibration(float calibration, actuator_controller_interface* controller)
{
    loadcell_calibration_mV_to_mNm = calibration;
    controller->configure_loadcell_calibration(loadcell_calibration_mV_to_mNm);
}
