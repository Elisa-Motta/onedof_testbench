#ifndef ONEDOF_REHAMOVE_APP_H
#define ONEDOF_REHAMOVE_APP_H

#include "headers.h"

// Stimulation headers
#include "smpt_ll_client.h"
#include "smpt_client.h"
#include "smpt_ml_client.h"
#include "smpt_messages.h"
#include "smpt_packet_number_generator.h"
#include "smpt_ll_packet_validity.h"

#include "feshandler.h"

#define EXERCISE_START              (M_PI/180.0)*150
#define EXERCISE_DURATION           6000
#define EXERCISE_AMPLITUDE          -(M_PI/180.0)*90
#define FLEXION_DURATION            3000

#define SOGLIA_POS_ERR 0.4

#define Number_of_points 3
#define Ramp 3
#define Period 25 //mettere periodo 40 se voglio 25Hz
#define DELTA_TORQUE 100



class onedof_rehamove_app
{

private:

public:

    onedof_rehamove_app(){}
    ~onedof_rehamove_app(){}

    /* Finite state machine*/
    void FSM();
    uint16_t rehamove_mode = 1;
    uint16_t last_rehamove_mode = 99;

    bool change_mode = true;
    onedof_shared_memory_comm c;

   /* Rehamove stimulation */
    void stimulation_calibration();
    void stimulation_calibration_bis(vector <double> &calib);
    void stimulation_initialization();
    void stimulation_pos(vector <double> &calib);
    void stimulation_torque(vector <double> &calib);
    double calcoloPID( double setpoint, double pv, double charge );
    void stimulation_ramp();
    void stimulation_pos_iterative(vector <double> &calib);
    void modulate_coefficient();

    void std_calculation(double data, int i);

    /* Stimulator variables */
    const char *port_name= "/dev/ttyUSB0";
    Smpt_device device= {0};
    Smpt_ml_init ml_init = {0};
    Smpt_ml_update ml_update = {0};
    Smpt_ml_get_current_data ml_get_current_data = {0};

    /* Motimove struct*/
    FesSerial fs;

    /* PID variables */
    double integral = 0;
    double derivative = 0;
    double kp=0.95;
    double ki=0.3;
    double kd=0.3;
    double output=0;
    double charge = 0;
    double dt=0.001;
    double error = 0;
    double pre_error = 0;
    double max = 1;
    double min = 0;
    double qold=0;
    double error_PI=0;

    double q_old=0;

    double q_calib= 0.0;
    

    /* Mean and STD variables */
    double mean_position_error=0;
    double mean_velocity=0.0;
    double std_velocity=0.0;
    double mean_pos_err_block=0.0;
    double old_pos_err=0;
    double sum_pos_err=0;
    double sum_vel=0.0;
    double sum_mean_pos_err =0.0;
    double sum_mean_vel=0.0;
    double avg_mean_velocity=0.0;
    double sum_pos_err_block=0.0;
    double sum_block_vel=0.0;
    double mean_block_vel = 0.0;
    double block_delta = 0.0;
    double block_mean = 0.0;
    double block_M2 = 0.0;
    double std_block_vel=0.0;
    double sum_alloc_arm_tor =0.0;
    double mean_alloc_arm_tor =0.0;
    double mean_vel_block=0.0;
    double pos_err=0;
    int num_iter = 0;
    int num_block = 0;
    double delta = 0.0;
    double delta_block = 0.0;
    double delta_vel=0.0;
    double avg_delta = 0.0;
    double avg_delta_vel=0.0;
    double mean = 0.0;
    double mean_block = 0.0;
    double mean_vel = 0.0;
    double avg_mean = 0.0;
    double avg_mean_vel=0.0;
    double M2 = 0.0;
    double M2_block = 0.0;
    double M2_vel=0.0;
    double avg_M2 = 0.0;
    double avg_M2_vel=0.0;
    double avg_std_vel=0.0;
    double m = 0.0;
    double up = 0.0;
    double down = 0.0;

    /* Stimulation Calibration variables */
    bool initialize=0;
    int stop_calibration=0;
    double corrente=2;
    double durata=200;
    double torque;
    double prev_torque;
    bool flag_min=0;
    double Imin=0;
    double PWmin=0;
    double Imax=0;
    double PWmax=0;

    double IMIN = 0;
    double IMAX = 0;
    double PWMIN = 0;
    double PWMAX = 0;


    /* Running Stimulation variables */
    double stim_current=0;
    double stim_PW=0;
    double q=0;
    double q0 = 0;
    double mod_coefficient=0;
    double coeff_allocation = 0.0;
    double coeff_weight_assistance = 0.0;
    double coeff_weight_assistance_calibrated = 0.0;
    double q_squared=0;
    double q_amp=0;
    double q_adjusted = 0.0;
    double des_pos;
    double pid_des_pos = 0.0;
    double act_pos;
    double des_torque;
    double act_velocity;
    double act_torque;
    double loadcell_reading;
    double pos_error = 0.0;
    double torque_impedance;
    double torque_setpoint;
    vector <double> calib ={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    double torque_fes;
    double torque_motor;
    double torque_tot;
    double des_pos2=0;
    double arm_weigth_comp_torq;
    int phase;
    double arm_torque;
    double linear_coefficient=0; /*relationship between charge and torque*/
    double std_position_error = 0.0;
    double sum_std_mean_position_error = 0.0;
    double avg_std_position_error = 0.0;
    double sum_mean_position_error = 0.0;
    double avg_mean_position_error = 0.0;
    double vector_position_error[FLEXION_DURATION] = {0.0};
    double tau_feedback = 0.0;
    double stiffness_nm_rad = 0.0;
    double damping_nms_rad = 0.0;
    double PW_min_ramp=200;
    double I_min_ramp=2;
    double PW_max_ramp=500;
    double I_max_ramp=20;
    int flag_boh = 0;
    int change_alloc = 0;
    int counter_alloc = 0;
    int vel_out = 0;

    double threshold_pos_error     = 0.0;
    double threshold_std_pos_error = 0.0;


    /* Onedof control */
    double trajectory_generation(int counter);
    double charge_generation(double q_adj, int iteration);

    // Charge waveform generation
    double charge_waveform_generation(double iteration, double counter_waveform);
    double charge_waveform_generation_new(double iteration, double counter_waveform);
    double charge_waveform_generation_tri(double iteration, double counter_waveform);


    int counter_t = 0;
    int counter_supp = 0;
    int counter_rep = 0;
    int counter_no_change = 0;
    int counter_q_adj = 0;

    // Initialize counter for waveform generation
    int counter_waveform = 0;
    int counter_iteration = 0;

    double read_ref_torque(int size, double *reference);
    double set_ref_torque();
    void configure_write();
    void configure_read(vector <double> &calib);

    double  elapsed_time_ms_offset_exercise   = 0;
    int iter=0;
    double torque_ref[100];


    /* Allocation */
    double FES_allocation = 1;
    double motor_allocation= 0;
    double torque_ref_fes[100];
    double torque_ref_motor[100];
    int campioni;
    int iter_seno;
    double delta_allocation = 0.0;
    double K_fatigue = 0.0;

    double charge_old= 0.0;
    double charge_dot = 0.0;
    double charge_dot_old= 0.0;
    double T_limit = 0.0;

    // for trials
    int count_good = 0;
    int count_bad = 0;

    // for triceps
    double q_adjusted_triceps = 0;
    double sum_pos_err_t = 0;
    double mean_position_error_t=0;
    double std_position_error_t = 0;
    double sum_mean_position_error_t = 0;
    double avg_mean_position_error_t = 0;
    double sum_std_mean_position_error_t = 0;
    double avg_std_position_error_t = 0;
    double avg_delta_t = 0.0;
    double avg_mean_t = 0.0;
    double avg_M2_t = 0.0;
    double q_t = 0;
     void stimulation_torque_triceps(vector <double> &calib);
     double IMIN_t = 0;
     double IMAX_t = 0;
     double PWMIN_t = 0;
     double PWMAX_t = 0;
     double stim_current_t = 0;
     double stim_PW_t = 0;
     double delta_t = 0;
     double mean_t = 0;
     double M2_t = 0;
     int count_good_t = 0;


    unsigned long int loop_count = 0;
    unsigned long int t_overflow = 0;

    void write_rehamove_file();
    void open_rehamove_file();
    void close_rehamove_file();
    void open_calibration_file();
    void write_calibration_file();
    void write_empty_calibration_file();
    void close_calibration_file();


    //MOTIMOVE functions
    void stimulation_initialization_motimove();
    void stimulation_calibration_motimove();
    void stimulation_torque_motimove(vector <double> &calib);


};

#endif // ONEDOF_REHAMOVE_APP_H








