#include "onedof_robot.h"

using namespace std;

/*****************************************************************************************
 * JOINT CONTROLLER POINTERS ASSIGNMENT
 ****************************************************************************************/
/// \brief agree_robot::assign_esmacat_slave_index
/// \param slave_in_app
/// \param slave_index
/// \return
///
void agree_robot_class::assign_esmacat_slave_index(agree_joint_controller* slave_in_app, int slave_index){
    agree_joints[slave_index] = slave_in_app;
    return;
}

/*****************************************************************************************
 * ROBOT CONSTRUCTOR
 ****************************************************************************************/
/// \brief Robot::Robot
///
agree_robot_class::agree_robot_class(){

    // Joint Sign
    DH_to_motor_sign.resize(N_DOFS_MAX);

    // Computed Variables
    gravity_torques_mNm.resize(N_DOFS_MAX);
    signed_gravity_torques_mNm.resize(N_DOFS_MAX);
    weight_compensation_torques_mNm.resize(N_DOFS_MAX);
    signed_weight_compensation_torques_mNm.resize(N_DOFS_MAX);

    // Measured Variables
    position_rad.resize(N_DOFS_MAX);
    speed_rad_s.resize(N_DOFS_MAX);
    loadcell_torque_mNm.resize(N_DOFS_MAX);
    impedance_control_setpoint_rad.resize(N_DOFS_MAX);
    impedance_control_k_gain_mNm_per_rad.resize(N_DOFS_MAX);
    impedance_control_d_gain_mNm_per_rad_per_sec.resize(N_DOFS_MAX);
    impedance_control_feedforward_torque_mNm.resize(N_DOFS_MAX);

    // Set all joint to false
    joint_is_referenced = Eigen::Array<bool,1,N_DOFS_MAX>::Zero(N_DOFS_MAX);
    joint_is_positioned = Eigen::Array<bool,1,N_DOFS_MAX>::Zero(N_DOFS_MAX);

    robot_is_referenced = false;
    robot_is_positioned = false;
}

/*****************************************************************************************
 * GET ROBOT PARAMETERS
 ****************************************************************************************/
/// \brief agree_robot::read_agree_system_parameters
///
void agree_robot_class::read_agree_robot_parameters(int side){

    // Centers of mass properties
    link_com_m      = 0.140;
    shell_com_m     = 0.170;

    // Mass properties
    link_mass_kg    = 0.350;
    shell_mass_kg   = 0.220;

    if(side==RIGHT) motor_to_model_sign = 1;
    else            motor_to_model_sign = -1;


}

/*****************************************************************************************
 * GET USER PARAMETERS
 ****************************************************************************************/
/// \brief agree_robot::read_agree_user_parameters
/// User weight compensation parameters
void agree_robot_class::read_agree_user_parameters(){

    // Length properties
    forearm_length_m = 0.146*weight_compensation_config.human_height_m;
    hand_length_m     = 0.108*weight_compensation_config.human_height_m;

    // Centers of mass properties
    forearm_com_m   = 0.43*forearm_length_m;
    hand_com_m      = 0.506*hand_length_m;

    // Mass properties
    forearm_mass_kg = 0.016*weight_compensation_config.human_weight_kg;
    hand_mass_kg    = 0.006*weight_compensation_config.human_weight_kg;

}

/*****************************************************************************************
 * GRAVITY COMPENSATION ALGORITHM
 ****************************************************************************************/
/// \brief agree_robot_class::get_inverse_dynamics_torque_onedof
/// \param weight_assistance
/// \param position
/// \return
///
double agree_robot_class::get_inverse_dynamics_torque_onedof(float weight_assistance, double position_rad){

    double gravity_torque_mNm;
    double weight_compensation_mNm;
    double g = -9.81;

    gravity_torque_mNm  = link_mass_kg*g*sin(position_rad)*link_com_m*Nm_to_mNm +
                        + shell_mass_kg*g*sin(position_rad)*(shell_com_m)*Nm_to_mNm   ;

    weight_compensation_mNm = forearm_mass_kg*g*sin(position_rad)*forearm_com_m*Nm_to_mNm +
                            + hand_mass_kg*g*sin(position_rad)*(forearm_length_m+hand_com_m)*Nm_to_mNm;

    return gravity_torque_mNm + weight_assistance*weight_compensation_mNm;
}

/*****************************************************************************************
 * GET JOINT ANGLES
 ****************************************************************************************/
///
/// \brief agree_robot_class::get_current_joint_angles_rad
/// \return
///
Eigen::VectorXd agree_robot_class::get_current_joint_angles_rad()
{
    Eigen::VectorXd joint_angles_rad(N_DOFS_MAX);
    for (int i=0;i<N_DOFS;i++){
        joint_angles_rad(i)           = agree_joints[i]->joint_values.incremental_encoder_reading_radians;
    }
    for (int i=N_DOFS;i<N_DOFS_MAX;i++){
        /** TODO: Just for debug */
        joint_angles_rad(i) = 0.0;

    }
    return joint_angles_rad;
}

/*****************************************************************************************
 * GET JOINT VELOCITIES
 ****************************************************************************************/
///
/// \brief agree_robot_class::get_current_joint_velocity_rad
/// \return
///
Eigen::VectorXd agree_robot_class::get_current_joint_velocity_rad_s()
{
    Eigen::VectorXd joint_velocity_rad_s(N_DOFS_MAX);
    for (int i=0;i<N_DOFS;i++){
        joint_velocity_rad_s(i) = agree_joints[i]->joint_values.incremental_encoder_speed_radians_sec;
    }
    for (int i=N_DOFS;i<N_DOFS_MAX;i++){
        joint_velocity_rad_s(i) = 0.0;
    }
    return joint_velocity_rad_s;
}

/*****************************************************************************************
 * GET JOINT TORQUES
 ****************************************************************************************/
///
/// \brief agree_robot_class::get_current_loadcell_torque_mNm
/// \return joint loadcell torque in milli-Nm
///
Eigen::VectorXd agree_robot_class::get_current_loadcell_torque_mNm()
{
    Eigen::VectorXd joint_torque_mNm(N_DOFS_MAX);
    for (int i=0;i<N_DOFS;i++){
        joint_torque_mNm(i) = agree_joints[i]->joint_values.filtered_load_mNm;
    }
    for (int i=N_DOFS;i<N_DOFS_MAX;i++){
        joint_torque_mNm(i) = 0.0;
    }
    return joint_torque_mNm;
}

/*****************************************************************************************
 * GET JOINT VARIABLES
 ****************************************************************************************/
void agree_robot_class::update_joints_variables(double elapsed_time_ms, uint64_t loop_cnt)
{
    position_rad = get_current_joint_angles_rad();
    speed_rad_s = get_current_joint_velocity_rad_s();
    loadcell_torque_mNm = get_current_loadcell_torque_mNm();
}

/**
esmacat_err agree_robot_class::get_epos4_errorcode(){

    esmacat_err error = NO_ERR;
    int errorcode = 0;

    for (int i=0;i<N_DOFS;i++){
        errorcode = agree_joints[i]->ethercat_interface.get_escon_fault() ;
        if(errorcode != 0)
        {
            std::stringstream ss;
            ss << std::hex << errorcode; // int decimal_value
            std::string res ( ss.str() );
            PLOGE << "EPOS4 Error J" << i+1 << " CODE: 0x" << res;
            error = ERR_UNKNOWN;
        }
    }
    return error;
}


///
/// \brief agree_robot_class::reset_joint_angles
///
void agree_robot_class::reset_joint_angles()
{
    for (int joint_index=0;joint_index<N_DOFS;joint_index++)
    {
        agree_joints[joint_index]->clear_encoder_counter();
    }
}

///
/// \brief agree_robot_class::reset_fault
///
void agree_robot_class::reset_fault()
{
    for (int joint_index=0;joint_index<N_DOFS;joint_index++)
    {
        agree_joints[joint_index]->reset_fault();
    }
}

///
/// \brief agree_robot_class::start_motor
///
void agree_robot_class::start_motor()
{
    for (int joint_index=0;joint_index<N_DOFS;joint_index++)
    {
        agree_joints[joint_index]->start_motor();
    }
}

///
/// \brief agree_robot_class::stop_motor
///
void agree_robot_class::stop_motor()
{
    for (int joint_index=0;joint_index<N_DOFS;joint_index++)
    {
        agree_joints[joint_index]->stop_motor();
    }
}

///
/// \brief agree_robot_class::set_mode_operation
/// \param mode of operation of EPOS4
///
void agree_robot_class::set_mode_operation(int mode)
{
    for (int joint_index=0;joint_index<N_DOFS;joint_index++)
    {
        agree_joints[joint_index]->set_mode_operation(mode);
    }
}

///
void agree_robot_class::set_elapsed_time_ms(double elapsed_time_ms){
    for (int joint_index=0;joint_index<N_DOFS;joint_index++)
    {
        agree_joints[joint_index]->elapsed_time = elapsed_time_ms;
    }
}

///
/// \brief agree_robot_class::save_joint_status
///
void agree_robot_class::save_joint_status(){
    for (int joint_index=0;joint_index<N_DOFS;joint_index++)
    {
        agree_joints[joint_index]->save_joint_status();
    }
}

///
/// \brief agree_robot_class::set_impedance_control_gain
///
void agree_robot_class::set_impedance_control_gain(){
    for (int joint_index=0;joint_index<N_DOFS;joint_index++)
    {
        agree_joints[joint_index]->set_impedance_control_gain(agree_joints[joint_index]->joint_task_control_command.impedance_control_k_gain_mNm_per_rad,
                                                              agree_joints[joint_index]->joint_task_control_command.impedance_control_d_gain_mNm_per_rad_per_sec);
    }
}

*/
