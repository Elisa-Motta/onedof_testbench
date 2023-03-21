/** @file
 * @brief Contains declarations required for the Series-Elastic Actuator (SEA) Driver
*/
#ifndef POSITION_ACQUISITION_H
#define POSITION_ACQUISITION_H

/*****************************************************************************************
 * MACROS
 ****************************************************************************************/

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "joint_structs.h"
#include "actuator_controller_interface.h"

/*****************************************************************************************
 * STRUCTS
 ****************************************************************************************/


/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/

class position_acquisition
{
private:

    /** Offset of the absolute encoder in counts */
    int16_t absolute_encoder_offset_counts;
    /** Sign of the readings from the absolute encoder with positive position */
    int absolute_encoder_sign;
    /** Filtered reading from the absolute encoder in counts per turn */
    float filtered_absolute_encoder_reading_cpt;

    /** Resolution of the incremental encoder in counts per turn */
    int incremental_encoder_resolution_cpt;
    /** Offset of the incremental encoder in counts */
    int32_t incremental_encoder_offset_counts;
    /** Sign of the readings from the incremental encoder with positive position */
    int incremental_encoder_sign;
    /** Holds the updated value of the baseline from which the absolute
     * encoder reading will be ramp limited to ensure sudden spikes are
     * filtered out */
    float ramp_baseline_absolute_enc_val;
    /** Flag is true if the loop has been called more than once. It is used to track the first
     * time the loop is called for absolute encoder ramp baseline */
    int absolute_enc_ramp_flag;
    /** Flag is true if the loop has been called more than once. It is used to track the first
     * time the loop is called for absolute encoder filter */
    int absolute_enc_loop_flag;
    /** Flag is true if the loop has been called more than once. It is used to track how many times
     * the calibration algorithm has been called */
    int prev_absolute_enc_err_flag;

public:

    position_acquisition();
    int16_t get_signed_raw_absolute_encoder_reading_cpt(actuator_controller_interface* controller);
    int16_t get_unfiltered_absolute_encoder_reading_cpt(actuator_controller_interface* controller);
    float get_filtered_absolute_encoder_reading_cpt(actuator_controller_interface* controller);
    float get_filtered_absolute_encoder_reading_radians(actuator_controller_interface* controller);
    void set_absolute_encoder_offset_counts(int16_t offset);
    void set_absolute_encoder_sign (int sign);
    int get_incremental_encoder_reading_cpt(actuator_controller_interface* controller);
    int32_t get_signed_raw_incremental_encoder_reading_cpt(actuator_controller_interface* controller);
    int32_t get_incremental_encoder_offset_counts(actuator_controller_interface* controller);
    float get_incremental_encoder_reading_radians(actuator_controller_interface* controller, uint16_t gear_ratio);
    void set_incremental_encoder_resolution_cpt(int resolution);
    void set_incremental_encoder_offset_counts(int32_t offset);
    void set_incremental_encoder_sign (int sign);
    int32_t get_incremental_encoder_offset_from_absolute_encoder_counts(actuator_controller_interface* controller, uint16_t gear_ratio);
    void incremental_encoder_calibration (float elapsed_time_ms, bool is_init, actuator_controller_interface* controller, uint16_t gear_ratio);
    void clear_incremental_encoder(actuator_controller_interface* controller);

};
#endif // POSITION_ACQUISITION_H
