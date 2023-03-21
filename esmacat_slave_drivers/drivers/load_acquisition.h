/** @file
 * @brief Contains declarations required for the Series-Elastic Actuator (SEA) Driver
*/
#ifndef LOAD_ACQUISITION_H
#define LOAD_ACQUISITION_H

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

class load_acquisition
{
private:
    /** Offset to be applied to the loadcell reading in mV */
    float loadcell_offset_mV;
    /** Factor that translates the loadcell reading in mV to load reading in milli-Nm */
    float loadcell_calibration_mV_to_mNm;
    /** Sign of the readings with positive load */
    int loadcell_sign;
    /** Filtered loadcell reading in milli-Nm*/
    float filtered_load_mNm;
    /** Flag is true if the loop has been called more than once. It is used to track the first
     * time the loop is called for the loadcell filter */
    int loadcell_loop_flag;

public:
    load_acquisition();
    float get_load_mNm(esmacat_err& error, actuator_controller_interface* controller);
    float get_filtered_load_mNm(esmacat_err& error, actuator_controller_interface* controller);
    void set_loadcell_sign (int sign, actuator_controller_interface* controller);
    void set_loadcell_zero_offset (float offset, actuator_controller_interface* controller);
    void set_loadcell_calibration (float calibration, actuator_controller_interface* controller);

};
#endif // LOAD_ACQUISITION_H
