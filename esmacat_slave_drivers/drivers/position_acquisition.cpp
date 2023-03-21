/** @file
 * @brief Contains definitions of functions used for the Series-Elastic Actuator (SEA) Driver
*/

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "position_acquisition.h"

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

position_acquisition::position_acquisition()
{

    absolute_encoder_offset_counts = 0;
    absolute_encoder_sign = 1;
    ramp_baseline_absolute_enc_val = 0;
    absolute_enc_loop_flag = 0;
    absolute_enc_ramp_flag =0;
    prev_absolute_enc_err_flag =0;

    incremental_encoder_resolution_cpt =0 ;
    incremental_encoder_offset_counts = 0;
    incremental_encoder_sign = 1;
}

/** @brief Computes the signed raw absolute encoder reading of the joint
 * in counts/turn
 *
 * Applies the sign to the raw reading from the absolute encoder
 * @return Absolute encoder reading in counts/turn
 */
int16_t position_acquisition::get_signed_raw_absolute_encoder_reading_cpt(actuator_controller_interface* controller)
{
    return  static_cast<int16_t>(controller->get_raw_absolute_encoder_reading_cpt() * absolute_encoder_sign);
}

/** @brief Computes the unfiltered absolute encoder reading in counts/turn
 *
 * Applies the offset to the reading normalizes the values to a range of
 * [-2048, 2048] where -2048  => -180 degrees and +2048 => +180 degrees.
 * The rate of increase of the reading is also limited to a steady rate to
 * filter out noisy spikes. This rate is limited to 'baseline' +/- 10
 * @return Unfiltered absolute encoder reading in counts/turn
 */
int16_t position_acquisition::get_unfiltered_absolute_encoder_reading_cpt(actuator_controller_interface* controller)
{
    //applies offset
    int16_t reading_with_offset_cpt = get_signed_raw_absolute_encoder_reading_cpt(controller) - absolute_encoder_offset_counts;
    // Setting value to +-180 degrees
    if (reading_with_offset_cpt> 2048)
    {
        reading_with_offset_cpt = reading_with_offset_cpt - 4096;
    }
    if (reading_with_offset_cpt<= -2048)
    {
        reading_with_offset_cpt = reading_with_offset_cpt + 4096;
    }
    // Setting filtered value for first loop cycle
    if (absolute_enc_ramp_flag ==0)
    {
        ramp_baseline_absolute_enc_val = reading_with_offset_cpt;
        absolute_enc_ramp_flag =1;
    }
    // removing outlier noise values (This could be accomplished with a better filter or
    // eliminated if the communication errors are solved.)
    //limits the rate of increase of the absolute encoder reading to
    // baseline +/- 10 only
    float ramp_limited_reading_with_offset_cpt;
    if(reading_with_offset_cpt-ramp_baseline_absolute_enc_val >= 10)
    {
        ramp_limited_reading_with_offset_cpt = ramp_baseline_absolute_enc_val+10;
    }
    else if(reading_with_offset_cpt-ramp_baseline_absolute_enc_val <= -10)
    {
        ramp_limited_reading_with_offset_cpt = ramp_baseline_absolute_enc_val-10;
    }
    else
    {
        ramp_limited_reading_with_offset_cpt = reading_with_offset_cpt;
    }

    ramp_baseline_absolute_enc_val = static_cast<float>(0.97)*ramp_baseline_absolute_enc_val+static_cast<float>(0.03)*ramp_limited_reading_with_offset_cpt;
    return  static_cast<int16_t>(ramp_limited_reading_with_offset_cpt);
}

/** @brief Computes the filtered absolute encoder reading in counts/turn
 *
 * Obtains the current unfiltered absolute encoder reading in counts/turn and applies an
 * IIR low-pass-filter to it. The equation for a LPF with input x and output y is
 *
 * y [n] = m * y[n-1] + (1-m) * x[n]
 * @return Filtered Absolute encoder reading in counts/turn
 */
float position_acquisition::get_filtered_absolute_encoder_reading_cpt(actuator_controller_interface* controller)
{
    float filtered_reading = 0;
    float unfiltered_reading = get_unfiltered_absolute_encoder_reading_cpt(controller);
    if (absolute_enc_loop_flag ==0)
    {
        filtered_reading = unfiltered_reading;
        absolute_enc_loop_flag =1;
    }
    else
    {
        filtered_reading = filtered_absolute_encoder_reading_cpt;
        filtered_reading = static_cast<float>(0.95)*filtered_reading+static_cast<float>(0.05)*unfiltered_reading;
    }
    return filtered_reading;
}

/** @brief Computes the filtered absolute encoder reading in radians
 * This is computed as: reading * 2 * pi/ no. of counts per rev
 * @return Filtered absolute encoder reading in radians
 */
float position_acquisition::get_filtered_absolute_encoder_reading_radians(actuator_controller_interface* controller)
{
    return get_filtered_absolute_encoder_reading_cpt(controller)*static_cast<float>(2*M_PI/4096);
}

/** @brief Sets the absolute encoder offset configured in counts
 *
 * @param offset The absolute encoder offset in counts
 */
void position_acquisition::set_absolute_encoder_offset_counts(int16_t offset)
{
    absolute_encoder_offset_counts = offset;
}

/** @brief Sets the absolute encoder offset sign
 *
 * @param offset The absolute encoder offset sign
 */
void position_acquisition::set_absolute_encoder_sign(int sign)
{
    absolute_encoder_sign = sign;
}

/** @brief Computes the signed raw incremental encoder reading of the joint
 * in counts/turn
 *
 * Applies the sign to the raw reading from the incremental encoder
 * @return Incremental encoder reading in counts/turn
 */
int32_t position_acquisition::get_signed_raw_incremental_encoder_reading_cpt(actuator_controller_interface* controller)
{
    return controller->get_raw_incremental_encoder_reading_cpt() * incremental_encoder_sign;
}


/** @brief Computes the  incremental encoder reading of the joint
 * in counts/turn with the offset considered
 *
 * @return Incremental encoder reading in counts/turn
 */
int position_acquisition::get_incremental_encoder_reading_cpt(actuator_controller_interface* controller)
{
    int32_t reading = get_signed_raw_incremental_encoder_reading_cpt(controller)  - incremental_encoder_offset_counts;
    return  reading;
}

/** @brief Computes the  incremental encoder reading of the joint
 * in radians
 *
 * Encoder resolution is in PPR which is = CPT/4. Since the readings
 * are measured in counts, the resolution has to be scaled appropriately.
 * The gear ratio is also applied since the reading has to be converted from
 * the motor shaft to the output shaft.
 * @return Incremental encoder reading in radians
 */
float position_acquisition::get_incremental_encoder_reading_radians(actuator_controller_interface* controller, uint16_t gear_ratio)
{
    return get_incremental_encoder_reading_cpt(controller) *  static_cast<float>(2*M_PI/(incremental_encoder_resolution_cpt*4*gear_ratio));
}

/** @brief Gets the incremental encoder offset configured in counts
 *
 * @return Incremental encoder offset in counts
 */
int32_t position_acquisition::get_incremental_encoder_offset_counts(actuator_controller_interface* controller)
{
    return incremental_encoder_offset_counts;
}

/** @brief Sets the incremental encoder offset configured in counts
 *
 * @param offset The incremental encoder offset in counts
 */
void position_acquisition::set_incremental_encoder_resolution_cpt(int resolution)
{
    incremental_encoder_resolution_cpt = resolution;
}

/** @brief Sets the incremental encoder offset configured in counts
 *
 * @param offset The incremental encoder offset in counts
 */
void position_acquisition::set_incremental_encoder_offset_counts(int32_t offset)
{
    incremental_encoder_offset_counts = offset;
}

/** @brief Sets the incremental encoder offset sign
 *
 * @param offset The incremental encoder offset sign
 */
void position_acquisition::set_incremental_encoder_sign(int sign)
{
    incremental_encoder_sign = sign;
}

/** @brief Computes the incremental encoder offset from the absolute encoder reading
 *
 * The position of the joint calculated by the incremental encoder and absolute encoder
 * should be the same. The difference is the offset that is to be applied to the incremental
 * encoder reading
 *
 * expected reading =
 * absolute encoder reading * (incremental encoder resolution * 4 * gear ratio)/absolute encoder resolution
 * @return Offset of the incremental encoder reading from the absolute encoder reading in counts
 */
int32_t position_acquisition::get_incremental_encoder_offset_from_absolute_encoder_counts(actuator_controller_interface* controller, uint16_t gear_ratio)
{
    int32_t abs_reading = get_unfiltered_absolute_encoder_reading_cpt(controller);
    int32_t inc_reading = get_signed_raw_incremental_encoder_reading_cpt(controller);
    int32_t offset =  inc_reading - abs_reading * ((incremental_encoder_resolution_cpt*4)*gear_ratio/4096);
    return offset;
}

/** @brief Calibrates the incremental encoder based on the offset observed between the
 * absolute encoder reading and the incremental encoder reading
 *
 * @param elapsed_time_ms Time elapsed since application was initiated (in ms)
 */
void position_acquisition::incremental_encoder_calibration (float elapsed_time_ms, bool is_init, actuator_controller_interface* controller, uint16_t gear_ratio)
{
    double observed_offset = get_incremental_encoder_offset_from_absolute_encoder_counts(controller, gear_ratio);
    double set_offset = get_incremental_encoder_offset_counts(controller);
    double measured_diff = observed_offset - set_offset;

    if (is_init == true)
    {
        set_incremental_encoder_offset_counts(get_incremental_encoder_offset_from_absolute_encoder_counts(controller,gear_ratio));
        return;
    }
    if (elapsed_time_ms < 2000)
    {
        int32_t offset_setting= static_cast<int32_t>(0.01* observed_offset + 0.99* set_offset);
        set_incremental_encoder_offset_counts(offset_setting);
    }
    else
    {
        if(measured_diff >700)
        {   //One revolution in incremental encoder is 120*4096, so this activates if our error is ~0.5 degrees
            prev_absolute_enc_err_flag++;
            if(prev_absolute_enc_err_flag < 2){
                //sea->set_incremental_encoder_offset_counts(sea->get_incremental_encoder_offset_from_absolute_encoder_counts()+100);
                //cout<<"ERROR in J"<<slave_index<<" incremental encoder offset. +ve corrective action taken"<<endl; err_msg_count++;
            }
        }
        else if(measured_diff < -700)
        {
            prev_absolute_enc_err_flag++;
            if(prev_absolute_enc_err_flag < 2)
            {
                //sea->set_incremental_encoder_offset_counts(sea->get_incremental_encoder_offset_from_absolute_encoder_counts()-100);
                //cout<<"ERROR in J"<<slave_index<<" incremental encoder offset. -ve corrective action taken"<<endl; err_msg_count++;
            }
        }
        else
        {
            prev_absolute_enc_err_flag=0;
        }

    }
}
/** @brief Clears the encoder [Do not change]
 *
 * This function should not be modified since it is tied to the firmware
 * for the slave
 */
void position_acquisition::clear_incremental_encoder(actuator_controller_interface* controller)
{
    controller->configure_slave_encoder_clear();
}
