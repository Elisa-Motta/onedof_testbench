/** @file
 * @brief Contains declaration required to read and load parameters for all Harmony joints and repsective joints of the 1DoF setup
*/


#ifndef AGREE_PARAMETERS_H
#define AGREE_PARAMETERS_H

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "application.h"
#include "agree_motor_controller.h"

#define N_DOFS 1
#define N_DOFS_MAX 5
#define N_FRAMES 8

#define RIGHT 0
#define LEFT 1

#define CYCLIC_SYNCHRONOUS_TORQUE 10
#define HOMING_MODE  6

/*****************************************************************************************
 * CLASSES
 ****************************************************************************************/

class agree_parameters_class
{
public:
    agree_parameters_class();

    void read_agree_parameters();
    void set_agree_parameters(agree_motor_controller *agree_joints);

    /** Flag if Testbed (1DoF) is used **/
    bool     use_testbed;
    bool     save_data;

    /** Select robot side **/
    int     side;

private:

        int32_t incremental_encoder_resolution_cpt[N_DOFS_MAX];
        int     incremental_encoder_sign[N_DOFS_MAX];
        float   incremental_encoder_offset_rad[N_DOFS_MAX];

        float   gear_ratio[N_DOFS_MAX];
        float   gear_ratio_transmission[N_DOFS_MAX];
        float   gear_power_efficiency[N_DOFS_MAX];
        
        float   viscous_friction_coefficient[N_DOFS_MAX];
        float   couloumb_friction_mNm[N_DOFS_MAX];

        float hard_stop_max_deg[N_DOFS_MAX];
        float hard_stop_min_deg[N_DOFS_MAX];

        float torque_constant_mNm_per_mA[N_DOFS_MAX];
        float torque_sign[N_DOFS_MAX];
        float nominal_current_A[N_DOFS_MAX];
        float max_current_mA[N_DOFS_MAX];


        float loadcell_calibration_mV_to_mNm[9];

        int   loadcell_sign[N_DOFS_MAX];
        float loadcell_fullscale_in_lbs[N_DOFS_MAX];
        float loadcell_offset_mNm[N_DOFS_MAX];


        int torque_ctrl_ESCON_setpoint_sign[9];

        float torque_control_p_gain[9];
        float torque_control_i_gain[9];
        float torque_control_d_gain[9];

        float position_control_p_gain[9];
        float position_control_i_gain[9];
        float position_control_d_gain[9];


        float impedance_control_k_mNm_per_rad[9];
        float impedance_control_d_mNm_s_per_rad[9];

        float impedence_control_max_err_rad[9];

        float friction_comp_max_torque_mNm[9];
        float friction_torque_threshold_rad_per_s[9];

        float soft_to_hard_stop_offset_deg[9];
        float soft_stop_max_torque_mNm[9];

        float velocity_stop_threshold_rad_per_sec[9];
        float max_load_stop_threshold_mNm[9];



};

#endif // AGREE_PARAMETERS_H
