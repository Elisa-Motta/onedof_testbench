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

#define EXERCISE_START              (M_PI/18.0)*15
#define EXERCISE_DURATION           6000.0
#define EXERCISE_AMPLITUDE          -(M_PI/18.0)*9
#define FLEXION_DURATION            3000


#define Number_of_points 3
#define Ramp 3
#define Period 40 //mettere periodo 40 se voglio 25Hz
#define DELTA_TORQUE 200

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
    void stimulation_initialization();
    void stimulation_pos(vector <double> &calib);
    void stimulation_torque(vector <double> &calib);
    double calcoloPID( double setpoint, double pv, double charge );
    void stimulation_ramp();
    void stimulation_pos_iterative(vector <double> &calib);
    // funzioni nuove Rebecca
    void update_allocation_factor();
    double update_charge_percentage(vector <double> &GS_params, double value);
    double torque_based_charge_generation(double iteration,  double counter_supp);
    double charge_waveform_generation(double iteration, double t);
    const char *port_name= "/dev/ttyUSB0";
    Smpt_device device= {0};
    Smpt_ml_init ml_init = {0};
    Smpt_ml_update ml_update = {0};
    Smpt_ml_get_current_data ml_get_current_data = {0};

    /* PID variables */
    double integral = 0;
    double derivative = 0;
    double kp=0.3;
    double ki=0.003;
    //double kappap=0.005;
    double output=0;
//    double kd;
    double charge = 0;
    double dt=0.001;
    double error = 0;
    double pre_error = 0;
    double max = 1;
    double min = 0;
    double qold=0;
    double error_PI=0;

    
    double mean_pos_err=0;
    double old_pos_err=0;
    double sum_pos_err=0;
    double pos_err=0;


    /* Calibration variables */
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
    double Qmax=0;

    double IMIN = 0;
    double IMAX = 0;
    double PWMIN = 0;
    double PWMAX = 0;


    double charge_old= 0.0;
    double charge_dot = 0.0;
    double charge_dot_old= 0.0;
    double T_limit = 0.0;

    /* Stimulation variables */
    double stim_current=0;
    double stim_PW=0;
    double q=0;
    double coeff_allocation = 0.0;
    double coeff_weight_assistance = 0.0;
    double coeff_weight_assistance_calibrated = 0.0;
    double q_squared=0;
    double q_amp=0;
    double q_adjusted = 0.0;
    double q_adj = 0.0;
    double des_pos;
    double act_pos;
    double delta = 0.0;
    double mean = 0.0;
    double M2 = 0.0;
    int rep = 0;
    double std_pos_err = 0.0;
     double act_velocity;

    /* Charge Updating variables */
    double sum_mean_pos_err= 0.0;
    double sum_std_pos_err =0.0;
    double std_pos_err_GS = 0.0;
    double mean_pos_err_GS =0.0;
    double delta_percentage = 0.0;
    double K_learning = 0.0;
    double delta_q=0.0;
    int num_iter = 0;
    double std_position_error = 0.0;
    double sum_std_mean_position_error = 0.0;
    double mean_std_mean_position_error = 0.0;
    double sum_mean_position_error = 0.0;
    double mean_mean_position_error = 0.0;
    double mean_position_error = 0.0;


    /* Charge generation variables*/

    double q_old = 0.0;
    double q_dot = 0.0;
    double q_dot_old= 0.0;
    int count_back = 0;


    // Initialize counter for waveform generation
    int counter_waveform = 0;
    int counter_iteration = 0;

    /**/
    double des_torque;
    double act_torque;
    double torque_impedance;
    double torque_setpoint;
    vector <double> calib_r ={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    double torque_fes;
    double torque_motor;
    double torque_tot;
    double des_pos2=0;
    double loadcell_reading;
    double arm_weigth_comp_torq;
    int phase;
    double arm_torque;
    double linear_coefficient=0; /*relationship between charge and torque*/
    double alfa = 0.5;
    bool first_rep_flag=0;
    double tau_feedback=0;
    double stiffness_nm_rad = 0.0;
    double damping_nms_rad = 0.0;
    double threshold_pos_error     = 0.0;
    double threshold_std_pos_error = 0.0;
    int counter_t =0;
    int counter_supp=0;





    double PW_min_ramp=200;
    double I_min_ramp=2;
    double PW_max_ramp=500;
    double I_max_ramp=20;



    /* Onedof control */
    void trajectory_generation();
    double charge_generation(double q_adj, int iteration);
    double read_ref_torque(int size, double *reference);
    double set_ref_torque();
    void configure_write();
    void configure_read(vector <double> &calib_r);

    void computemean();

    int    support_repetitions = 0;
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
    // parametri funzione rebecca
    double sum_tau_imp = 0.0;
    double mean_tau_imp = 0.0;
    double old_tau_imp = 0.0;
    double var=0.0;

    unsigned long int loop_count = 0;
    unsigned long int t_overflow = 0;

    void write_rehamove_file();
    void open_rehamove_file();
    void close_rehamove_file();
    void open_calibration_file();
    void write_calibration_file();
    void close_calibration_file();






};

#endif // ONEDOF_REHAMOVE_APP_H








