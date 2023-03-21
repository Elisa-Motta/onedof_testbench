#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "agree_structs.h"
#include "esmacat_epos4_mod.h"
#include "agree_common.h"
#include "application.h"
#include <cstdlib>

using namespace std;

class agree_motor_controller : public esmacat_epos4
{
    //////////////////////////////////////////////////////////////////
    //////////////// POSITION VARIABLES //////////////////////////////
    //////////////////////////////////////////////////////////////////

    /** Resolution of the incremental encoder in counts per turn */
    int32_t incremental_encoder_resolution_cpt;
    /** Sign of the readings from the incremental encoder with positive position */
    int     incremental_encoder_sign;
    /** Offset of incremental encoder wrt DH model */
    float   incremental_encoder_offset_rad;


    /** Gear ratio of the motor (output shaft to motor shaft) */
    float   gear_ratio;
    /** Gear ratio of the joint transmission (output joint to output shaft) */
    float   gear_ratio_transmission;

    /** Torque constant of the motor in milli-Nm per mA */
    float torque_constant_mNm_per_mA  = 36.9;
    /** Motor Nominal Current in A */
    float nominal_current_A         = 3.21;

    /** Viscous friction coefficient */
    float viscous_friction_coeff = 200; //100
    /** Coulomb friction torque */
    float coulomb_friction_mNm;

  /** Position PID parameters */
  double position_control_rad_p_gain;
  double position_control_rad_i_gain;
  double position_control_rad_d_gain;
  /** Max Position integration error */
  double max_integ_position_error_rad = 10*M_PI/180;

  /** Velocity PID parameters */
  float speed_control_rad_p_gain;
  float speed_control_rad_i_gain;
  float speed_control_rad_d_gain;
  /** Max Velocity integration error */
  float max_integ_speed_error_rad = 10*M_PI/180;

  // Temporary variables
  float prev_position_rad = 0;
  float prev_velocity_rad = 0;

  double prev_loadcell_torque = 0;

  double prev_position_error_rad = 0;
  double integ_position_error_rad = 0;

  float prev_speed_error_rad = 0;
  float prev_deriv_error_rad_per_s = 0;
  float speed_integ_error_rad;





  /*************************
   *  MOTOR     VARIABLES  *
   *************************/



  /*************************
   *  TORQUE    VARIABLES  *
   *************************/

  /** Loadcell fullscale reading in inch-lbs */
  float loadcell_fullscale_lbs;
  /** Offset to be applied to the loadcell reading in mNm */
  float loadcell_offset_mNm = -45.0;
  /** Loadcell sign wrt motor direction */
  int loadcell_sign         = 1;


  // Filtered desired torque mNm
  // used to smooth the torque response if desired torque is far from actuaL
  // Temporary variables

  float filtered_setpoint;
  float filtered_deriv_torque_error;
  float prev_loadcell_error_mNm;
  float prev_deriv_loadcell_error_mNm;
  float integ_loadcell_error_mNm;
  double filtered_setpoint_torque_mNm = 0;

  float filtered_stiffness_k;
  float filtered_damping_d;


  /** @brief Sets the loaded joint parameters into the respective actuator controller and driver objects
   *
   * @param Pointer to the sea_controller object
   * @param Index of the joint (Note that J[0] and J[1] are unused)
   */
public:
  agree_motor_controller();

  /** Index of the joint */
  int joint_index = 0;


  /** Offset of the incremental encoder in counts */
  int32_t incremental_encoder_offset_counts   = 0;

  /** Position in radians of the hard stop in the positive direction */
  float hard_stop_max_rad = 55.0/180*M_PI;//31.5/180.0*M_PI;
  /** Position in radians of the hard stop in the negative direction */
  float hard_stop_min_rad = 0.0;//-20.0/180.0*M_PI;
  /** Position offset in radians from soft to hard end-stops */
  float soft_to_hard_stop_offset_rad = M_PI/32;
  /** Maximum soft end-stop torque in mNm */
  float soft_stop_max_torque_mNm = 20.0*1000;
  /** Soft end-stop damping */
  float soft_stop_damping = 5.0*1000;
  /** Max Torque Integral error mNm */
  float max_integ_torque_error_mNm = 2000;



  /** Holds all the readings for each joint **/
  joint_status_t                    joint_status;
  /** Holds the status of the torque control loop */
  joint_torque_control_status_t     joint_torque_control_status;
  /** Holds the status of the impedance control loop */
  joint_impedance_control_status_t  joint_impedance_control_status;
  /** Holds the configuration of the joint controller */
  joint_controller_configuration_t  joint_impedance_control_config;

  joint_task_control_command_t      joint_task_control_command;

  /** Holds motor configuration */
  joint_motor_configuration_t       joint_motor_config;


  /*************************
   *  POSITION FUNCTIONS    *
   *************************/

  esmacat_err clear_encoder_counter();
  int32_t get_encoder_counter();
  int32_t get_encoder_position();

  float get_encoder_position_radians();           // Need to be virtual to be executed within robot class methods
  float get_encoder_computed_speed_radians();     // Need to be virtual to be executed within robot class methods
  float get_encoder_filt_speed_radians();

  // set position control
  esmacat_err set_position_control_rad(float desired_position_rad, float elapsed_time_ms, float setpoint_feedforward);
  // set speed control
  esmacat_err set_speed_control(float desired_speed, float setpoint_feedforward);

  // set PID parameters position control
  void set_position_control_rad_pid_gain(float p, float d, float i);
  void set_speed_control_rad_pid_gain(float p, float d, float i);

  bool set_homing_mode();


  /*************************
   *  TORQUE FUNCTIONS    *
   *************************/

  /** Read feedback torque from loadcell in mNm */
  double get_loadcell_torque_mNm();
  /** Set desired torque in feedforward current control */
  esmacat_err set_feedforward_torque_mNm(float desired_torque_mNm);
  /** Set desired torque in feedback torque control in mNm, the reference is the loadcell value */
  esmacat_err set_torque_control_mNm(double desired_torque_mNm);
  /** Set PID parameters torque control */
  void set_torque_control_pid_gain(float p, float d, float i);
  /** Set desired position in impedance control loop, feedforward torque term is also fed to impedance controller */
  esmacat_err set_impedance_control_rad(float impedance_control_setpoint_rad,float gravity_torque_mNm);
  /** Set impedance control parameters */
  void set_impedance_control_gain(float k, float d);


  /*************************
   *  COMMON  FUNCTIONS    *
   *************************/

  float mNm_to_setpoint(float torque_mNm);
  void  get_errorcode_hex();

  void set_joint_index(int j);
  void set_torque_constant(float t);
  void set_nominal_current(float i);
  void set_encoder_cpt(int32_t cpt);
  void set_encoder_sign(int s);
  void set_encoder_offset(float offset);
  void set_gear_ratio(float g);
  void set_gear_ratio_transmission(float g);
  void set_joint_limits(float min,float max);

  void set_loadcell_fullscale_lbs(float f);
  void set_loadcell_offset_mNm(float o);
  void set_loadcell_sign(int sign);
  void set_viscous_friction_coefficient(float v);
  void set_coulomb_friction_mNm(float c);

  void reset_position_errors();
  float sigmoid(float x){return 1/(1+exp(x));}

  void set_joint_motor_configuration(joint_motor_configuration_t* motor_config);

  // Stores variables in motor_controller class
  /** Joint position in radians */
  float position_rad = 0;
  /** Joint velocity in radians per second */
  float velocity_rad_per_s = 0;
  /** Joint loadcell torque in mNm */
  float loadcell_torque_mNm = 0;
  /** Homing procedure status */
  bool encoder_is_referenced;

  void save_joint_status();


};

#endif // MOTOR_CONTROLLER_H
