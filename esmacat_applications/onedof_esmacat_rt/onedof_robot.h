#ifndef AGREE_ROBOT_H
#define AGREE_ROBOT_H

/*****************************************************************************************
* DEFINES
*****************************************************************************************/

#define N_DOFS 1
#define N_DOFS_MAX 5
#define N_FRAMES 8

#define RIGHT 0
#define LEFT 1

/*****************************************************************************************
* INCLUDES
*****************************************************************************************/

#include <eigen3/Eigen/Geometry>

//#include "application.h"
//#include "agree_common.h"
#include "agree_joint_controller.h"

#include <iostream>

using namespace std;

class agree_motor_controller;

class agree_robot_class {

private:
    double g;

public:

    agree_robot_class();

    agree_joint_controller* agree_joints[N_DOFS];

    /** Robot status mode */
    uint16_t status;
    /** Robot commanded mode */
    uint16_t command, prev_command;

    /** Kinematic DH model table*/
    Eigen::MatrixXd DH_model;
    /** Joint signs according to DH model */
    Eigen::VectorXd DH_to_motor_sign;

    /** Anthropometry data - Winter et al. 2009 */
    // Length properties
    double forearm_length_m;
    double hand_length_m;

    // Centers of mass properties
    double forearm_com_m;
    double hand_com_m;

    // Mass properties
    double forearm_mass_kg;
    double hand_mass_kg;

    /** Robot data - from SolidWorks */
    // Length properties
    double link_length_m;

    // Centers of mass properties
    double link_com_m;
    double shell_com_m;

    // Mass properties
    double link_mass_kg;
    double shell_mass_kg;

    // Sign for conversion from motor reference to model reference
    double motor_to_model_sign;


    /** Joint angle positions measured with incremental encoder */
    Eigen::VectorXd position_rad;
    /** Joint angle velocities measured by differentiating the incremental encoder*/
    Eigen::VectorXd speed_rad_s;
    /** Torques measured at each joint frame */
    Eigen::VectorXd loadcell_torque_mNm;

    /** Torques required at each joint frame to compensate for gravity */
    Eigen::VectorXd gravity_torques_mNm;
    /** Torques required at each joint frame to compensate for gravity, with DH signs accounted for */
    Eigen::VectorXd signed_gravity_torques_mNm;

    /** Torques required at each joint frame to compensate for gravity of robot+human*/
    Eigen::VectorXd weight_compensation_torques_mNm;
    /** Torques required at each joint frame to compensate for gravity of robot+human, with DH signs accounted for */
    Eigen::VectorXd signed_weight_compensation_torques_mNm;

    /** Joint angle reference position for impedance controller */
    Eigen::VectorXd impedance_control_setpoint_rad;
    /** Joint virtual stiffness for impedance controller */
    Eigen::VectorXd impedance_control_k_gain_mNm_per_rad;
    /** Joint virtual stiffness for impedance controller */
    Eigen::VectorXd impedance_control_d_gain_mNm_per_rad_per_sec;
    /** Joint feedforward torque setpoint */
    Eigen::VectorXd impedance_control_feedforward_torque_mNm;



    /** Joint handler */
    esmacat_err     joint_error[N_DOFS_MAX];

    /** Array of booleans to store referenced joints **/
    Eigen::Array<bool,1,5> joint_is_referenced;
    /** Array of booleans to store boolean **/
    Eigen::Array<bool,1,5> joint_is_positioned;

    /** Boolean for initialization done **/
    bool                    robot_is_referenced = 0;
    /** Boolean for initialization done **/
    bool                    robot_is_positioned = 0;

    int16_t                 joint_reference[5];
    /** Subject and robot configuration */
    arm_weight_compensation_config_t weight_compensation_config;

    //// ////////////////////////////////////////////////////////////////
    //// /////////////////// Robot Class Methods/////////////////////////
    //// ////////////////////////////////////////////////////////////////

    void assign_esmacat_slave_index(agree_joint_controller* slave_in_app, int slave_index);

    void direct_kinematics();
    void get_Jacobian();

    Eigen::MatrixXd get_dh_table();

    Eigen::VectorXd get_inverse_dynamics_torques(float a);
    double          get_inverse_dynamics_torque_onedof(float a, double position);

    Eigen::VectorXd cartesian_impedance_torque_LGR();
    Eigen::VectorXd friction_compensation_torque();

    Eigen::VectorXd get_current_joint_angles_rad();
    Eigen::VectorXd get_current_joint_velocity_rad_s();
    Eigen::VectorXd get_current_loadcell_torque_mNm();

    void            update_joints_variables(double elapsed_time_ms, uint64_t loop_cnt);

    esmacat_err     get_epos4_errorcode();
    void            reset_joint_angles();
    void            reset_fault();
    void            start_motor();
    void            stop_motor();
    void            set_mode_operation(int mode);
    void            set_current_limit();
    void            set_impedance_control_gain();
    void            set_elapsed_time_ms(double elapsed_time_ms);
    void            save_joint_status();

    void            run_homing_mode();
    void            run_impedance_control(double elapsed_time_ms);

    Eigen::VectorXd motor_to_model_angles_conversion(Eigen::VectorXd motor_angles_rad);
    Eigen::VectorXd model_to_motor_angles_conversion(Eigen::VectorXd model_angles_rad);

    void read_agree_robot_parameters(int side);
    void read_agree_user_parameters();

    void set_cartesian_impedance_parameters(double Ksx, double Kdx,  double impedance_setpoint_x, double Ksz, double Kdz,double impedance_setpoint_z);
};

#endif
