/** @file
 * @brief Contains definitions of functions used to read and load parameters for all Harmony joints and repsective joints of the 1DoF setup
*/


/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "agree_parameters.h"

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/

agree_parameters_class::agree_parameters_class()
{
    save_data = false;
}

void agree_parameters_class::read_agree_parameters(){



    // Set AGREE Configuration
    for (int index=0;index<N_DOFS;index++) {

        // Encoder Configuration
        incremental_encoder_resolution_cpt[index]       = 2048;
        incremental_encoder_sign[index]                 = 1;

        // Gearbox Configuration
        gear_ratio[index]                               = 156.0;
        gear_ratio_transmission[index]                  = 1.0;
        viscous_friction_coefficient[index]             = 200.0; // [mNm/rad]

        // Motor Configuration
        torque_constant_mNm_per_mA[index]               = 36.9;
        torque_sign[index]                              = 1;
        nominal_current_A[index]                        = 3.21;
        max_current_mA[index]                           = 5000;

        // Loadcell Configuration
        loadcell_fullscale_in_lbs[index]                = 200.0;
        loadcell_sign[index]                            = 1.0;

        // Torque Controller
        torque_control_p_gain[index]                    = 4.5;
        torque_control_d_gain[index]                    = 1.5;
        torque_control_i_gain[index]                    = 0.02;

        // Impedance Controller
        impedance_control_k_mNm_per_rad[index]          = 0.0;
        impedance_control_d_mNm_s_per_rad[index]        = 0.25;

        // Limits
        hard_stop_max_deg[index]                        = 45.0;
        hard_stop_min_deg[index]                        = -45.0;
    }

    // Set J1 Configuration
    int joint_index = 0;
    torque_control_p_gain[joint_index]                            = 3.5;
    torque_control_d_gain[joint_index]                            = 2.0;
    torque_control_i_gain[joint_index]                            = 0.1;
    gear_ratio[joint_index]                                       = 156.0;
    gear_ratio_transmission[joint_index]                          = 28.0/16.0;
    loadcell_fullscale_in_lbs[joint_index]                        = 50.0;
    loadcell_offset_mNm[joint_index]                              = 0.0;
    loadcell_sign[joint_index]                                    = 1;
    incremental_encoder_offset_rad[joint_index]                   = 0;
    incremental_encoder_sign[joint_index]                         = 1;
    viscous_friction_coefficient[joint_index]                     = 800.0; // [mNm/rad]
    hard_stop_max_deg[joint_index]                                = 31.5;
    hard_stop_min_deg[joint_index]                                = -20.0;

    // Set J2 Configuration
    joint_index = 1;
    torque_control_p_gain[joint_index]                            = 2.5;
    torque_control_d_gain[joint_index]                            = 2.0;
    torque_control_i_gain[joint_index]                            = 0.0;
    gear_ratio[joint_index]                                       = 156.0;
    gear_ratio_transmission[joint_index]                          = 1.0;
    loadcell_fullscale_in_lbs[joint_index]                        = 200.0;
    loadcell_offset_mNm[joint_index]                              = 0.0;
    loadcell_sign[joint_index]                                    = -1;
    incremental_encoder_offset_rad[joint_index]                   = 0;
    incremental_encoder_sign[joint_index]                         = 1;
    viscous_friction_coefficient[joint_index]                     = 200.0; // [mNm/rad]
    hard_stop_max_deg[joint_index]                                = 55.0;
    hard_stop_min_deg[joint_index]                                = 0.0;


    // Set J3 Configuration
    joint_index = 3;
    torque_control_p_gain[joint_index]                            = 7.5;
    torque_control_d_gain[joint_index]                            = 2.5;
    torque_control_i_gain[joint_index]                            = 0.02;
    gear_ratio[joint_index]                                       = 156;
    gear_ratio_transmission[joint_index]                          = 110.0/15.0;
    loadcell_fullscale_in_lbs[joint_index]                        = 50.0;
    loadcell_offset_mNm[joint_index]                              = -250.0;
    loadcell_sign[joint_index]                                    = 1;
    incremental_encoder_offset_rad[joint_index]                   = 0;
    incremental_encoder_sign[joint_index]                         = 1;
    viscous_friction_coefficient[joint_index]                     = 400.0; // [mNm/rad]
    hard_stop_max_deg[joint_index]                                = 55.0;
    hard_stop_min_deg[joint_index]                                = 0.0;

    // Set J4 Configuration
    joint_index = 2;
    torque_control_p_gain[joint_index]                            = 3.5;
    torque_control_d_gain[joint_index]                            = 2.0;
    torque_control_i_gain[joint_index]                            = 0.0;
    gear_ratio[joint_index]                                       = 156;
    gear_ratio_transmission[joint_index]                          = 1;
    loadcell_fullscale_in_lbs[joint_index]                        = 200.0;
    loadcell_offset_mNm[joint_index]                              = 0.0;
    loadcell_sign[joint_index]                                    = 1;
    incremental_encoder_offset_rad[joint_index]                   = 0;
    incremental_encoder_sign[joint_index]                         = 1;
    viscous_friction_coefficient[joint_index]                     = 200.0; // [mNm/rad]
    hard_stop_max_deg[joint_index]                                = 0.0;
    hard_stop_min_deg[joint_index]                                = -75.0; // -135 con glifo


    if(use_testbed){
        // Set Testbed Configuration
        joint_index = 0;
        torque_control_p_gain[joint_index]                          = 2.5; // 4.5;
        torque_control_d_gain[joint_index]                          = 1.5;
        torque_control_i_gain[joint_index]                          = 0.2;
        gear_ratio[joint_index]                                     = 156;
        gear_ratio_transmission[joint_index]                        = 1.0;
        loadcell_fullscale_in_lbs[joint_index]                      = 200.0;
        loadcell_offset_mNm[joint_index]                            = 0.0;
        loadcell_sign[joint_index]                                  = 1;
        incremental_encoder_offset_rad[joint_index]                 = 0; //M_PI/2.0;
        incremental_encoder_sign[joint_index]                       = 1;
        viscous_friction_coefficient[joint_index]                   = 0.0; // [mNm/rad]
        hard_stop_min_deg[joint_index]                              = -100.0;
        hard_stop_max_deg[joint_index]                              = 45.0;
    }

}





void agree_parameters_class::set_agree_parameters(agree_motor_controller *agree_joints){

    joint_motor_configuration_t motor_config;

    for (int joint_index = 0 ; joint_index < N_DOFS ; joint_index++ )
    {
        // Joint Index
        agree_joints[joint_index].set_joint_index(joint_index);

        // Encoder Parameters
        agree_joints[joint_index].set_encoder_cpt(incremental_encoder_resolution_cpt[joint_index]);
        agree_joints[joint_index].set_encoder_sign(incremental_encoder_sign[joint_index]);
        agree_joints[joint_index].set_encoder_offset(incremental_encoder_offset_rad[joint_index]);

        // Loadcell Parameters
        agree_joints[joint_index].set_loadcell_fullscale_lbs(loadcell_fullscale_in_lbs[joint_index]);
        agree_joints[joint_index].set_loadcell_offset_mNm(loadcell_offset_mNm[joint_index]);
        agree_joints[joint_index].set_loadcell_sign(loadcell_sign[joint_index]);

        // Gearbox Parameters
        agree_joints[joint_index].set_gear_ratio(gear_ratio[joint_index]);
        agree_joints[joint_index].set_gear_ratio_transmission(gear_ratio_transmission[joint_index]);
        agree_joints[joint_index].set_viscous_friction_coefficient(viscous_friction_coefficient[joint_index]);
        //agree_joints[joint_index].set_coulomb_friction_mNm(couloumb_friction_mNm[joint_index]);

        // Controller Parameters
        agree_joints[joint_index].set_current_limit(max_current_mA[joint_index]);
        agree_joints[joint_index].set_torque_control_pid_gain(torque_control_p_gain[joint_index],torque_control_d_gain[joint_index],torque_control_i_gain[joint_index]);

        // Range of Motion Parameters
        agree_joints[joint_index].set_joint_limits(hard_stop_min_deg[joint_index], hard_stop_max_deg[joint_index]);
    }
}
