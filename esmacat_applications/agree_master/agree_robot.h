#ifndef AGREE_ROBOT_H
#define AGREE_ROBOT_H

/*****************************************************************************************
* INCLUDES
****************************************************************************************/

#include <eigen3/Eigen/Geometry>

//#include "application.h"
#include "agree_common.h"
#include "agree_parameters.h"
//#include "agree_manager.h"
//#include "agree_motor_controller.h"

#include <iostream>

using namespace std;

class agree_motor_controller;

class agree_robot_class {

private:
    double g;

public:

    agree_robot_class();

    agree_motor_controller* agree_joints[N_DOFS];

    uint16_t status;
    uint16_t command, prev_command;

    /** Kinematic DH model table*/
    Eigen::MatrixXd dh_model;
    /** Joint signs according to DH model */
    Eigen::VectorXd joint_sign;

    /** Anthropometry data - Winter et al. 2009 */
    // Human properties
    float  human_mass_kg;
    float  human_height_m;
    // Length properties
    float upperarm_length_m;
    float lowerarm_length_m;
    float hand_length_m;
    // Mass properties
    float arm_mass_kg[N_FRAMES];
    Eigen::Vector3d arm_com_m[N_FRAMES];

    /** Robot data - from SolidWorks AGREE V1 */
    // Length properties
    float link_length[N_DOFS];
    float trunk_height_m;
    float trunk_width_m;
    // Mass Properties
    float link_mass_kg[N_FRAMES];
    Eigen::Vector3d link_com_m[N_FRAMES];
    Eigen::Matrix3d I_tensor[N_FRAMES];




    Eigen::MatrixXd motorToModel;

    /** Homogeneous transformation matrix, from 0 to n */
    Frame T_total[N_FRAMES];
    /** Homogeneous transformation matrix, from n to n+1 */
    Frame T_matrix[N_FRAMES];
    /** Orientation of the gravity frame */
    Eigen::Vector3d g_hat[N_FRAMES];
    /** Robot Jacobian Matrix */
    Eigen::MatrixXd Jacobian;

    /** Joint angle positions measured with incremental encoder */
    Eigen::VectorXd position_rad;
    /** Joint angle velocities measured by differentiating the incremental encoder*/
    Eigen::VectorXd velocity_rad_s;
    /** Torques measured at each joint frame */
    Eigen::VectorXd loadcell_torque_mNm;

    /** Torques required at each joint frame to compensate for gravity */
    Eigen::VectorXd gravity_torques_Nm;
    /** Torques required at each joint frame to compensate for gravity, with DH signs accounted for */
    Eigen::VectorXd signed_gravity_torques_mNm;

    /** Torques required at each joint frame to compensate for gravity of robot+human*/
    Eigen::VectorXd weight_compensation_torques_Nm;
    /** Torques required at each joint frame to compensate for gravity of robot+human, with DH signs accounted for */
    Eigen::VectorXd signed_weight_compensation_torques_mNm;

    /** Joint angle reference position for impedance controller */
    Eigen::VectorXd impedance_control_setpoint_rad;
    /** Joint virtual stiffness for impedance controller */
    Eigen::VectorXd impedance_control_k_gain_mNm_per_rad;
    /** Joint virtual stiffness for impedance controller */
    Eigen::VectorXd impedance_control_d_gain_mNm_per_rad_per_sec;



    /** Joint handler */
    esmacat_err     joint_error[N_DOFS_MAX];

    /** Array of booleans to store referenced joints **/
    Eigen::Array<bool,1,5> joint_is_referenced;
    /** Boolean for initialization done **/
    bool                    robot_is_ready;
    /** Subject and robot configuration */
    robot_configuration_t robot_config;

    // Cartesian Impedance
//    CartesianDirection cartesian_direction;
//    double cartesian_Ksx,cartesian_Kdx, cartesian_Ksz,cartesian_Kdz;
//    double cartesian_setpoint_x,cartesian_setpoint_z,cartesian_error_x,cartesian_error_z, cartesian_max_error;
//    Eigen::Vector3d cartesian_force;

    //// ////////////////////////////////////////////////////////////////
    //// /////////////////// Robot Class Methods/////////////////////////
    //// ////////////////////////////////////////////////////////////////

    void assign_esmacat_slave_index(agree_motor_controller* slave_in_app, int slave_index);

    void direct_kinematics();
    void get_Jacobian();

    Eigen::MatrixXd get_dh_table();

    Eigen::VectorXd get_inverse_dynamics_torques();
    double          get_inverse_dynamics_torque_onedof(float a, double position);

    Eigen::VectorXd cartesian_impedance_torque_LGR();
    Eigen::VectorXd friction_compensation_torque();

    Eigen::VectorXd get_current_joint_angles_rad();
    Eigen::VectorXd get_current_joint_velocity_rad_s();
    Eigen::VectorXd get_current_loadcell_torque_mNm();

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
    void            run_impedance_control();




    Eigen::VectorXd motor_to_model_angles_conversion(Eigen::VectorXd motor_angles_rad);
    Eigen::VectorXd model_to_motor_angles_conversion(Eigen::VectorXd model_angles_rad);
    Eigen::VectorXd ConvertJointToMotorTorque(Eigen::VectorXd jointTorque);

    void read_agree_robot_parameters(int side);
    Eigen::MatrixXd get_dh_model(int side);
    int get_joint_sign(int joint_index){return joint_sign(joint_index);}


    void set_cartesian_impedance_parameters(double Ksx, double Kdx,  double impedance_setpoint_x, double Ksz, double Kdz,double impedance_setpoint_z);
};

#endif
