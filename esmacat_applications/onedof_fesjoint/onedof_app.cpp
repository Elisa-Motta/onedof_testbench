/**
 * @file
  *
 * @brief This application demonstrates a minimal example of joint_controller application.
 *
 */

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include <math.h>
#include "onedof_app.h"

double interim_acceleration;

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/
/** @brief Constructor for the class
 *
 * Initializes and loads parameters essential for the functioning of the robot
 *
 */
onedof_manager::onedof_manager()
{
    // Loops that goes along the agree joint controllers
    for(int slave_index=0;slave_index<N_DOFS_MAX;slave_index++)
    {
        // Assign cycle time in seconds to agree joint controller objects
        agree_joints[slave_index].set_one_cycle_time_s(static_cast<float>(get_one_cycle_time_ns()/1000000000.0));
        agree_joint_error[slave_index] = NO_ERR;
        agree_control_error[slave_index] = NO_ERR;
    }
    PLOG_INFO << "Onedof Manager Constructed at " << 1000000.0/get_one_cycle_time_ns() << "kHz";
}

onedof_manager::~onedof_manager()
{
    PLOG_INFO << "Onedof Manager Destructed";
}

/** @brief Method to assign slave index to each of the EtherCAT slaves on the device
 *
 * Connects the etherCAT slaves to their respective pointers that were definedin onedof_app.h
 * Has options to connect with the EtherCAT switch or to a single arm directly
 * Has options to connect an EASE for use as a controller
 *
 */
void  onedof_manager::assign_slave_sequence()
{
    int first_driver_index = 0;

    if(USE_EASE){
        onedof_manager::assign_esmacat_slave_index(&(agree_arduino),(0));
        first_driver_index = 1;
    }

    // tell the master what type of slave is at which point in the chain
    for(int slave_index=0;slave_index<N_DOFS;slave_index++)
    {
        // Assign slave index for desired slaves
        assign_esmacat_slave_index(&agree_joints[slave_index].ethercat_interface,slave_index+first_driver_index);
        // Pass agree_joints class to agree_robot class
        agree_robot.assign_esmacat_slave_index(&agree_joints[slave_index],slave_index);
    }

}
/** @brief Method to configure slaves
 *
 * Configures pin input/output direction
 * Configures control mode
 * Configures driver parameters from .json files.
 *
 */
void onedof_manager::configure_slaves()
{
    // Set input/output direction
    IO_Direction d[]={IO_INPUT,IO_INPUT,IO_OUTPUT,IO_OUTPUT,IO_OUTPUT};

    // Loops that goes along the agree joint controllers
    for(int slave_index=0;slave_index<N_DOFS;slave_index++)
    {
        // Configure input/output direction
        agree_joints[slave_index].ethercat_interface.configure_dio_direction(d);
        // Set mode (torque vs current)
        agree_joints[slave_index].set_control_mode(actuator_controller_interface::control_mode_t::torque_control);
        // Configure driver parameters
        agree_joints[slave_index].configure_joint_controller_from_file("../../../onedof_config_files/J0_actuator_maxon_ec45_156.json");
    }
}

/** @brief Commands that are run after EtherCAT has been setup, but before the first loop
 *
 * Used to run encoder calibration sequences, and will eventually have pre-startup checks.
 * Option to write to file are also here
 *
 */
void onedof_manager::init()
{
    // Set conversiona factor for arduino stored variables
    if(USE_EASE) agree_arduino.set_conversion_factor(20);

    // Initializing the shared memory
    if (onedof_shared_memory.init())
    {
        PLOGI << "User Interface shared memory initialized with key " << hex << static_cast<int>(onedof_shared_memory.get_shared_memory_key()) << dec ;    // start the shared memory communication
    }
    else
    {
        PLOGE << "User Interface shared memory initialization has been failed";
        onedof_shared_memory.detach_shared_memory();
    }

    // Initializes shared memory mode/command/status
    onedof_shared_memory.set_esmacat_command(robot_control_mode_t::standby);
    onedof_shared_memory.set_esmacat_status(robot_control_mode_t::standby);

    onedof_manager::readsharedMemory();

    agree_robot.read_agree_user_parameters();
    agree_robot.read_agree_robot_parameters(RIGHT);

    if(USE_LOGGER) openfile();
}

/** @brief This method is the primary loop for the 1Dof application
 *
 * All of the computation required to run the Onedof setup will be done within this
 *
 */
void onedof_manager::loop()
{

    // Initialize error for each cycle loop
    esmacat_err agree_control_error[5] = {NO_ERR, NO_ERR,NO_ERR,NO_ERR,NO_ERR};

    // Read from shared-memory-segment
    onedof_manager::readsharedMemory();

    // Update joints variable in joint class
    for(int j=0;j<N_DOFS;j++){
        agree_joint_error[j] = agree_joints[j].update_joint_variables(elapsed_time_ms,loop_cnt);
    }

    // Update joints variables in robot class
    agree_robot.update_joints_variables(elapsed_time_ms,loop_cnt);

    // Update user-dependent parameters
    // Needed in the loop to on-line update weight_compensation_config
    agree_robot.read_agree_user_parameters();


    // Compute beta function
    float P0,P1,P2,P3,P4,P5;
    double interim_time_exercise;
    double interim_setpoint_exercise;
    double interim_acceleration_exercise;

    P0 = EXERCISE_START;
    P2 = 0.0;
    P3 = 5.0; //5ht Order
    P4 = P2+EXERCISE_DURATION;
    P5 = 5.0; //5th Order
    P1 = EXERCISE_AMPLITUDE/pow(EXERCISE_DURATION/2,(P3+P5));

    // Compute time variable
    interim_time_exercise = (elapsed_time_ms-elapsed_time_ms_offset_exercise);

    // Beta-Function
    interim_setpoint_exercise = P0+P1*pow((interim_time_exercise-P2),P3)*pow((P4-interim_time_exercise),P5);

//        // Compute acceleration
    interim_acceleration_exercise = (P1*(P3-1)*P3*pow((interim_time_exercise-P2),(P3-2))*pow((P4-interim_time_exercise),P5)-2*P1*P3*P5*pow((interim_time_exercise-P2),(P3-1))*pow((P4-interim_time_exercise),(P5-1))
            + P1*(P5-1)*P5*pow((interim_time_exercise-P2),P3)*pow((P4-interim_time_exercise),(P5-2)))*1000.0*1000.0;


    interim_acceleration=interim_acceleration_exercise;
   // agree_robot.impedance_control_acc << interim_acceleration_exercise;


    // Compute Direct Kinematics and Inverse Dynamics
    // Robot torques
    agree_robot.robot_dynamic_torque_mNm        << agree_robot.get_inverse_dynamics_torque_onedof(0.0,interim_setpoint_exercise,0.0 ), 0, 0, 0, 0;
    agree_robot.robot_weight_torque_mNm        << agree_robot.get_inverse_dynamics_torque_onedof(0.0,agree_robot.position_rad[0],0.0 ), 0, 0, 0, 0;

    //Total torques
    agree_robot.total_dynamic_torque_mNm        << agree_robot.get_inverse_dynamics_torque_onedof(agree_robot.weight_compensation_config.weight_assistance, interim_setpoint_exercise, 0.0 ), 0, 0, 0, 0;
    agree_robot.total_weight_torque_mNm         << agree_robot.get_inverse_dynamics_torque_onedof(agree_robot.weight_compensation_config.weight_assistance, agree_robot.position_rad[0], 0.0 ), 0, 0, 0, 0;
    //??????: CHE DIFFERENZA C'È FRA QUESTE DUE?


    //Net arm torques
    agree_robot.arm_dynamic_torque_mNm          = agree_robot.total_dynamic_torque_mNm - agree_robot.robot_dynamic_torque_mNm;
    agree_robot.arm_weight_torque_mNm          = agree_robot.total_weight_torque_mNm - agree_robot.robot_weight_torque_mNm;

    for(int joint_index=0;joint_index<N_DOFS;joint_index++)
    {
        // Compute net arm weight compensation torque
        // For shared memory coomunication
        agree_joints[joint_index].joint_impedance_control_terms.arm_dynamic_torque_mNm = agree_robot.arm_dynamic_torque_mNm[joint_index];
        agree_joints[joint_index].joint_impedance_control_terms.robot_weight_torque_mNm = agree_robot.robot_weight_torque_mNm[joint_index];
        agree_joints[joint_index].joint_impedance_control_terms.robot_dynamic_torque_mNm = agree_robot.robot_dynamic_torque_mNm[joint_index];
        agree_joints[joint_index].joint_impedance_control_terms.total_dynamic_torque_mNm = agree_robot.total_dynamic_torque_mNm[joint_index];
    }

    // Swap control mode function
    if(control_mode != prev_mode){
        PLOGW << "Robot: " << robot_control_labels[onedof_shared_memory.data->control_mode_command] << " Control: " << control_mode_labels[static_cast<int>(agree_joints[0].get_current_control_mode())] << endl;
        elapsed_time_ms_offset = elapsed_time_ms;
        if(prev_mode != adaptive_control && prev_mode != passive_control && prev_mode != anti_g_control && prev_mode != transparent_control && prev_mode != resistive_control && prev_mode != challenging_control )
        elapsed_time_ms_offset_exercise = elapsed_time_ms;
    }

    // Switch control mode
    switch (control_mode)
    {
    /*********************/
    /*   Stop + Quit     */
    /*********************/
    case robot_control_mode_t::quit: case robot_control_mode_t::standby:
    {
        for(int joint_index=0;joint_index<N_DOFS;joint_index++)
        {
            // Disable drivers
            agree_joints[joint_index].ethercat_interface.set_escon_enable(false);
        }
        // Update Status
        control_mode_status = control_mode;
        break;
    }

        // Current Control - NOTE: This mode needs direct_escon_control to be enabled
        case robot_control_mode_t::current_control:
        {

            agree_joints[0].set_position_control_pid_gain(1500.0,0.0,50.0);

            float t = M_PI/4*sin(2*M_PI*0.1*elapsed_time_ms/1000);
//            agree_joints[0].set_position_control_pid_gain(0.3*1, 0.01*1, 0.013);

            agree_joints[0].control_position_rad(t,elapsed_time_ms);
            // Set External Torque Control parameters
            // agree_joints[0].set_torque_gain(4.5,0.0025,0.0);
            // Run External Torque Control
            //agree_control_error[0] = agree_joints[0].control_torque_directly_mA(100,static_cast<float>(elapsed_time_ms - elapsed_time_ms_offset));
            //agree_joints[0].control_torque_external_mNm(0.0,static_cast<float>(elapsed_time_ms - mode_start_elapsed_time_ms));
            // Update Status
            control_mode_status = control_mode;

            break;
        }

        // Zero-Torque Control
    case robot_control_mode_t::zerotorque_control:
    {
        for(int j=0;j<N_DOFS;j++){

            // Enable Inner Control
            agree_joints[j].ethercat_interface.set_escon_enable(true);
            // Run Inner Torque Control
            agree_control_error[j] = agree_joints[j].control_torque_with_soft_stop_mNm(0.0, static_cast<float>(elapsed_time_ms - elapsed_time_ms_offset));
        }
        // Update Status
        control_mode_status = control_mode;
        //            "torque_control_p_gain": 0.6,
        //            "torque_control_i_gain": 0.025,
        //            "torque_control_d_gain": 0.0025,

        break;
    }

        // Torque Control
    case robot_control_mode_t::torque_control:
    {
        for(int joint_index=0;joint_index<N_DOFS;joint_index++){

            // Compute desired torque profile in mNm
            support_torq[joint_index] = 0.0;
            // Enable Inner Control
            agree_joints[joint_index].ethercat_interface.set_escon_enable(true);
            // Run Inner Torque Control
            agree_control_error[joint_index] = agree_joints[joint_index].control_torque_with_soft_stop_mNm(onedof_shared_memory.data->impedance_control_command->impedance_control_feedforward_torque_mNm, static_cast<float>(elapsed_time_ms - elapsed_time_ms_offset));
        }

        // Update Status
        control_mode_status = control_mode;

        break;
    }

    /*********************/
    /* Impedance Control */
    /*********************/
    case impedance_ext_control: case freeze_control: case impedance_control: case gravity_control: case weight_comp_control: case passive_control: case adaptive_control: case anti_g_control: case transparent_control: case resistive_control: case challenging_control:
    {
        // First Iteration
        if(control_mode != prev_mode){

            // Set starting/offset transition values
            for(int j=0;j<N_DOFS;j++){
                // Starting position
                agree_joints[j].impedance_control_offset_pos_rad = agree_joints[j].joint_values.incremental_encoder_reading_radians;
                // Starting torque
                //agree_joints[j].impedance_control_offset_torque = support_torq[j]; // BUG: avoid abrupt changes during transition. agree_joints[j].joint_values.filtered_load_mNm;
                // Starting time
                agree_joints[j].impedance_control_offset_time_ms = elapsed_time_ms;
                // Starting impedance
                if(agree_joints[j].joint_impedance_control_terms.impedance_control_first_run)
                {
                    agree_joints[j].impedance_control_offset_setpoint_rad = agree_joints[j].get_incremental_encoder_reading_radians();
                    agree_joints[j].impedance_control_offset_torque = agree_joints[j].joint_values.filtered_load_mNm;

                }
                else
                {
                    agree_joints[j].impedance_control_offset_setpoint_rad = agree_joints[j].get_impedance_control_setpoint_rad();
                    agree_joints[j].impedance_control_offset_torque = support_torq[j];
                }
                // Starting stiffness
                agree_joints[j].impedance_control_offset_k = agree_joints[j].get_impedance_control_K_mNm_per_rad();
                // Starting damping
                agree_joints[j].impedance_control_offset_d = agree_joints[j].get_impedance_control_d_mNm_per_rad_per_sec();

            }

            support_repetitions = 0;
        }


        // Restart Beta-Function
        if(interim_time_exercise > EXERCISE_DURATION) {
            elapsed_time_ms_offset_exercise = elapsed_time_ms;
            support_repetitions++;
            PLOGW << "Repetition #" << support_repetitions;
        }

//        if(control_mode == impedance_control){ //desired position
//            agree_robot.weight_compensation_torques_mNm  << agree_robot.get_inverse_dynamics_torque_onedof(agree_robot.weight_compensation_config.weight_assistance,interim_setpoint_exercise, interim_acceleration_exercise), 0, 0, 0, 0;
//        }
//        else if(control_mode == impedance_ext_control) { //actual position
//            agree_robot.weight_compensation_torques_mNm  << agree_robot.get_inverse_dynamics_torque_onedof(agree_robot.weight_compensation_config.weight_assistance, interim_setpoint_exercise, interim_acceleration_exercise), 0, 0, 0, 0;
//        }

        // Compute trajectory and run outer impedance control
        for(int j=0;j<N_DOFS;j++)
        {

            if(control_mode == adaptive_control || control_mode == passive_control || control_mode == anti_g_control || control_mode==transparent_control ||  control_mode == resistive_control || control_mode == challenging_control ){
                target_k[j]     = agree_robot.impedance_control_k_gain_mNm_per_rad[j];
                target_d[j]     = agree_robot.impedance_control_d_gain_mNm_per_rad_per_sec[j];
                target_traj[j]  = interim_setpoint_exercise; // WARNING: Trajectory generated in the real-time code to avoid quantization errors.
                target_torq[j]  = agree_robot.total_weight_torque_mNm[j];
            }

            else if(control_mode == freeze_control){ //WARNING: freeze position is computed in real-time. FIXME: freeze position from shm?
                target_k[j]     = agree_robot.impedance_control_k_gain_mNm_per_rad[j];
                target_d[j]     = agree_robot.impedance_control_d_gain_mNm_per_rad_per_sec[j];
                target_traj[j]  = agree_joints[j].impedance_control_offset_pos_rad;
                target_torq[j]  = agree_robot.robot_weight_torque_mNm[j];

            }


            else if(control_mode == impedance_ext_control){

                target_k[j]     = agree_robot.impedance_control_k_gain_mNm_per_rad[j];
                target_d[j]     = agree_robot.impedance_control_d_gain_mNm_per_rad_per_sec[j];
                // Sinusoidal-Function
                target_traj[j] = interim_setpoint_exercise;
                target_torq[j]  = agree_robot.total_weight_torque_mNm[j];
            }
            else if(control_mode == impedance_control){



                target_k[j]     = agree_robot.impedance_control_k_gain_mNm_per_rad[j];
                target_d[j]     = agree_robot.impedance_control_d_gain_mNm_per_rad_per_sec[j];
                // Sinusoidal-Function
                target_traj[j] = interim_setpoint_exercise;
                //target_torq[j] = agree_robot.arm_dynamic_torque_mNm[j]+agree_robot.robot_dynamic_torque_mNm[j];
                target_torq[j] = agree_robot.impedance_control_feedforward_allocation_factor*agree_robot.arm_dynamic_torque_mNm[j]+
                                 agree_robot.robot_dynamic_torque_mNm[j];
                PLOGW << "MOTOR ALLOC FACTOR:  " << agree_robot.impedance_control_feedforward_allocation_factor;
            }
            else if(control_mode == gravity_control){
                target_k[j] = 0.0;
                target_d[j] = 0.1; // TODO: Tune damping
                target_traj[j] = agree_joints[j].get_incremental_encoder_reading_radians();
                target_torq[j] = agree_robot.robot_weight_torque_mNm[j];
            }
            else if(control_mode == weight_comp_control){
                target_k[j] = 0.0;
                target_d[j] = 0.5;
                target_traj[j] = agree_joints[j].get_incremental_encoder_reading_radians();
                target_torq[j] = agree_robot.total_weight_torque_mNm[j];
            }
            else if(control_mode == zerotorque_control){
                target_k[j] = 0.0;
                target_d[j] = 0.0;
                target_traj[j] = agree_joints[j].get_incremental_encoder_reading_radians();
                target_torq[j] = 0.0;
            }

            support_k[j]        = agree_joints[j].impedance_control_offset_k + (target_k[j]*1000 - agree_joints[j].impedance_control_offset_k)
                    * smooth_start_func_tanh((elapsed_time_ms-agree_joints[j].impedance_control_offset_time_ms)/2000.0);

            support_d[j]        = agree_joints[j].impedance_control_offset_d + (target_d[j]*1000 - agree_joints[j].impedance_control_offset_d)
                    * smooth_start_func_tanh((elapsed_time_ms-agree_joints[j].impedance_control_offset_time_ms)/2000.0);

            support_traj[j]     = agree_joints[j].impedance_control_offset_setpoint_rad + (target_traj[j] - agree_joints[j].impedance_control_offset_setpoint_rad)
                    * smooth_start_func_tanh((elapsed_time_ms-agree_joints[j].impedance_control_offset_time_ms)/500.0);

            support_torq[j]     = agree_joints[j].impedance_control_offset_torque + (target_torq[j] - agree_joints[j].impedance_control_offset_torque)
                    * smooth_start_func_tanh((elapsed_time_ms-agree_joints[j].impedance_control_offset_time_ms)/2000.0);

            // NOTE: Alternative function for smooth function
            // /2.0*(tanh((elapsed_time_ms-agree_joints[j].impedance_control_offset_time_ms)/10000.0-exp(1.0))+1.0);

            // Set Impedance parameters
            agree_joints[j].set_impedance_control_K_mNm_per_rad(support_k[j]);
            agree_joints[j].set_impedance_control_d_mNm_per_rad_per_sec(support_d[j]);

            // Run Outer Impedance Control
            agree_control_error[j]= agree_joints[j].impedance_control( static_cast<float>(support_traj[j]), static_cast<float>(support_torq[j]), static_cast<float>(elapsed_time_ms), true);
        }

        // Update Status
        control_mode_status = control_mode;

        break;
    }

    /**************************/
    /* Go-To-Position Control */
    /**************************/
    case robot_control_mode_t::go_position_control:
    {
        // First Iteration
        if(control_mode != prev_mode){

            // Set starting/offset transition values
            for(int j=0;j<N_DOFS;j++){
                // Starting position
                agree_joints[j].impedance_control_offset_pos_rad = agree_joints[j].joint_values.incremental_encoder_reading_radians;
                // Starting torque
                agree_joints[j].impedance_control_offset_torque = agree_joints[j].joint_values.filtered_load_mNm;
                // Starting time
                agree_joints[j].impedance_control_offset_time_ms = elapsed_time_ms;
                // Starting impedance
                if(agree_joints[j].joint_impedance_control_terms.impedance_control_first_run)
                {
                    agree_joints[j].impedance_control_offset_setpoint_rad = agree_joints[j].get_incremental_encoder_reading_radians();
                }
                else
                {
                    agree_joints[j].impedance_control_offset_setpoint_rad = agree_joints[j].get_impedance_control_setpoint_rad();
                }
                // Starting stiffness
                agree_joints[j].impedance_control_offset_k = agree_joints[j].get_impedance_control_K_mNm_per_rad();
                // Starting damping
                agree_joints[j].impedance_control_offset_d = agree_joints[j].get_impedance_control_d_mNm_per_rad_per_sec();

            }
        }


        // Compute trajectory and run outer impedance control
        for(int joint_index=0;joint_index<N_DOFS;joint_index++)
        {
            target_k[joint_index]     = agree_robot.impedance_control_k_gain_mNm_per_rad[joint_index];
            target_d[joint_index]     = agree_robot.impedance_control_d_gain_mNm_per_rad_per_sec[joint_index];

            //target_traj[joint_index]  =agree_robot.impedance_control_setpoint_rad[joint_index];
            target_traj[joint_index]  = EXERCISE_START; //WARNING: Start position is computed in real-time. FIXME: agree_robot.impedance_control_setpoint_rad[joint_index];

            target_torq[joint_index]  = agree_robot.robot_weight_torque_mNm[joint_index]; //TODO: Check sign with testbed

            // Support trajectory
            support_traj[joint_index] = agree_joints[joint_index].impedance_control_offset_pos_rad + ( target_traj[joint_index]  - agree_joints[joint_index].impedance_control_offset_pos_rad)
                    /2.0*(tanh((elapsed_time_ms-agree_joints[joint_index].impedance_control_offset_time_ms)/1000.0-exp(1.0))+1.0);

            // Support k-d parameters
            support_k[joint_index]        = agree_joints[joint_index].impedance_control_offset_k + (target_k[joint_index]*1000 - agree_joints[joint_index].impedance_control_offset_k)
                    * smooth_start_func_tanh((elapsed_time_ms-agree_joints[joint_index].impedance_control_offset_time_ms)/2000.0);

            support_d[joint_index]        = agree_joints[joint_index].impedance_control_offset_d + (target_d[joint_index]*1000 - agree_joints[joint_index].impedance_control_offset_d)
                    * smooth_start_func_tanh((elapsed_time_ms-agree_joints[joint_index].impedance_control_offset_time_ms)/2000.0);

            support_torq[joint_index]     = target_torq[joint_index];

            //                agree_joints[joint_index].set_impedance_control_K_mNm_per_rad(agree_robot.impedance_control_k_gain_mNm_per_rad[joint_index]);
            //                agree_joints[joint_index].set_impedance_control_d_mNm_per_rad_per_sec(agree_robot.impedance_control_d_gain_mNm_per_rad_per_sec[joint_index]);

            // Run Outer Impedance Control
            // Set Impedance parameters
            agree_joints[joint_index].set_impedance_control_K_mNm_per_rad(support_k[joint_index]);
            agree_joints[joint_index].set_impedance_control_d_mNm_per_rad_per_sec(support_d[joint_index]);

            // Run Outer Impedance Control
            agree_control_error[joint_index]= agree_joints[joint_index].impedance_control( static_cast<float>(support_traj[joint_index]), static_cast<float>(support_torq[joint_index]), static_cast<float>(elapsed_time_ms), false);



            if(abs(agree_robot.impedance_control_setpoint_rad[joint_index] - agree_joints[joint_index].joint_values.incremental_encoder_reading_radians)<=M_PI/32.0)
            {
                agree_joints[joint_index].joint_is_positioned = true;

            }
            else{
                agree_joints[joint_index].joint_is_positioned = false;
            }

            agree_robot.joint_is_positioned[joint_index] = agree_joints[joint_index].joint_is_positioned;
        }

        // For all non-active joints set joint_is_referenced flag to true.
        for (int joint_index=N_DOFS;joint_index<N_DOFS_MAX;joint_index++){
            agree_robot.joint_is_positioned[joint_index] = true;
        }

        // Update Status
        if(agree_robot.joint_is_positioned.prod()){
            agree_robot.robot_is_positioned = true;
            control_mode_status = robot_control_mode_t::go_position_done;
        }
        else{
            agree_robot.robot_is_positioned = false;
            control_mode_status = robot_control_mode_t::go_position_control;
        }
        break;
    }

    /******************/
    /* Homing Control */
    /******************/
    case robot_control_mode_t::homing_control:
    {
        if(control_mode != prev_mode){
            // For all active joints run homing control
            for (int joint_index=0;joint_index<N_DOFS;joint_index++){
                agree_joints[joint_index].calibration_status = 100;
                agree_joints[joint_index].incremental_encoder_calibration_counter = 0;
                                    agree_joints[joint_index].impedance_control_offset_pos_rad = agree_joints[joint_index].get_incremental_encoder_reading_radians();
            }
        }

        // If calibration was previously completed (EASE 6th variable set to true)
        if(agree_arduino.is_robot_referenced()){

            // For all active joints calibrate incremental encoder with values from EASE
            for (int joint_index=0;joint_index<N_DOFS;joint_index++){
                agree_joints[joint_index].calibration_reference_counts = agree_arduino.get_calibration_reference_counts(joint_index);
                agree_joints[joint_index].calibrate_incremental_encoder();
                agree_robot.joint_is_referenced[joint_index] = true;
            }

            // For all non-active joints set joint_is_referenced flag to true.
            for (int joint_index=N_DOFS;joint_index<N_DOFS_MAX;joint_index++){
                agree_robot.joint_is_referenced[joint_index] = true;
            }

            agree_robot.robot_is_referenced = true;


        }
        // If calibration was not previously completed (EASE 6th variable set to zero)
        else{

            // For all active joints run homing control
            for (int joint_index=0;joint_index<N_DOFS;joint_index++){
                agree_control_error[joint_index] = agree_joints[joint_index].homing_control(elapsed_time_ms);
                agree_robot.joint_is_referenced[joint_index] = agree_joints[joint_index].joint_is_referenced;
            }
            // For all non-active joints set joint_is_referenced flag to true.
            for (int joint_index=N_DOFS;joint_index<N_DOFS_MAX;joint_index++){
                agree_robot.joint_is_referenced[joint_index] = true;
            }

            agree_robot.joint_is_referenced[2] = true;

            // If all active joints are referenced
            if(agree_robot.joint_is_referenced.prod()){
                agree_robot.robot_is_referenced = true;

                // Set calibration references in arduino
                for (int joint_index=0;joint_index<N_DOFS;joint_index++){
                    agree_arduino.set_calibration_reference_counts(joint_index,agree_joints[joint_index].calibration_reference_counts);
                }
                // Set calibration boolean in arduino
                agree_arduino.set_robot_referenced(agree_robot.robot_is_referenced);
                // Data ready to be saved in EASE;
                agree_arduino.set_calibration_ready(true);

            }
            else
            {
                agree_robot.robot_is_referenced = false;
            }
        }

        if(agree_robot.robot_is_referenced){
            control_mode_status = robot_control_mode_t::homing_done;
            for (int joint_index=0;joint_index<N_DOFS;joint_index++){
            agree_joints[joint_index].joint_impedance_control_terms.impedance_control_first_run = true;
//                agree_control_error[joint_index] = agree_joints[joint_index].impedance_control(0.0,
//                                                                                               0.0,
//                                                                                               (elapsed_time_ms - elapsed_time_ms_offset),
//                                                                                               false);
            }
        }
        else {
            control_mode_status = robot_control_mode_t::homing_control;
            elapsed_time_ms_offset = elapsed_time_ms;
        }

        break;

    }

    /*******************/
    /* Default Control */
    /*******************/
    default:
    {
        // Stop motors
        for(int j=0;j<N_DOFS;j++)
        {
            agree_joints[j].ethercat_interface.set_escon_enable(false);
        }
        // Update status
        control_mode_status = control_mode;

        break;
    }
    }

    // Update previous control mode
    prev_mode = control_mode;
    // Write shared-memory-segment
    write2sharedMemory();

    // Save Log in file at 1kHz
    if(USE_LOGGER) writefile();

    // Print feedback at 1Hz
    if(loop_cnt%1000==0) print_feedback();

    // Print sequential feedback from mini-torque-driver at 1kHz
    if(PRINT_DEBUG) print_sequential_feedback();


    if (loop_cnt > APPLICATION_TIMEOUT_MS || onedof_shared_memory.data->control_mode_command == robot_control_mode_t::quit )
    {
        PLOGW << "Exit conditions met. AGREE Manager is closing.";
        stop();
        onedof_shared_memory.detach_shared_memory();
        if(USE_LOGGER) closefile();
    } // Shared memory trigger to stop
}

void onedof_manager::write2sharedMemory()
{
    for(int joint_index=0;joint_index<N_DOFS_MAX;joint_index++){

        onedof_shared_memory.data->joint_values[joint_index]                    = agree_joints[joint_index].get_joint_vals();
        onedof_shared_memory.data->torque_control_terms[joint_index]            = agree_joints[joint_index].get_torque_control_terms();
        onedof_shared_memory.data->impedance_control_terms[joint_index]         = agree_joints[joint_index].get_impedance_control_terms();

    }


    onedof_shared_memory.data->elapsed_time_ms               = elapsed_time_ms;
    onedof_shared_memory.data->loop_cnt                      = loop_cnt;
    onedof_shared_memory.data->control_mode_status           = control_mode_status;
}

void onedof_manager::readsharedMemory()
{
    control_mode                            = onedof_shared_memory.data->control_mode_command;

    agree_robot.weight_compensation_config  = onedof_shared_memory.data->arm_weight_compensation_config;

    for(int joint_index=0;joint_index<N_DOFS_MAX;joint_index++){
        agree_robot.impedance_control_setpoint_rad[joint_index]                     = onedof_shared_memory.data->impedance_control_command[joint_index].impedance_control_setpoint_rad;
        agree_robot.impedance_control_k_gain_mNm_per_rad[joint_index]               = onedof_shared_memory.data->impedance_control_command[joint_index].impedance_control_k_gain_mNm_per_rad;
        agree_robot.impedance_control_d_gain_mNm_per_rad_per_sec[joint_index]       = onedof_shared_memory.data->impedance_control_command[joint_index].impedance_control_d_gain_mNm_per_rad_per_sec;
        agree_robot.impedance_control_feedforward_torque_mNm[joint_index]           = onedof_shared_memory.data->impedance_control_command[joint_index].impedance_control_feedforward_torque_mNm;
// TODO: Move this somewhere else
//            agree_joints[joint_index].set_joint_config_soft_stop(   agree_shared_memory.data->impedance_control_command->soft_stop_lower_limit_rad,
//                                                                    agree_shared_memory.data->impedance_control_command->soft_stop_upper_limit_rad );
    }

    agree_robot.impedance_control_feedforward_allocation_factor=onedof_shared_memory.data->impedance_control_command->impedance_control_feedforward_allocation_factor;

    // boolean for ROS command use
    if(onedof_shared_memory.data->use_ros)
    {
        agree_robot.weight_compensation_config  = onedof_shared_memory.data->arm_weight_compensation_config;


    }
}

void onedof_manager::openfile(){

    // Get system  time
    time_t t = time(nullptr);
    struct tm * now = localtime( & t );
    char buffer [80];

    // Log directory
    strftime (buffer,80,"/home/esmacat/esmacat_rt/onedof_log/onedof-esmacat-%Y-%m-%d-%H-%M-%S.csv",now);
    CSVfile.open (buffer);
    if(CSVfile.is_open())
    {
        PLOGI << "AGREE Log File Created Successfully";

        CSVfile << endl

                << "J_command" << ","
                << "J_status" << ","
                << "J_elapsed_time_ms" << ",";

        for(int joint_index=0;joint_index<N_DOFS;joint_index++)
        {
            CSVfile << "J_" << joint_index << "_position_rad" ","
                    << "J_" << joint_index << "_position_deg" ","
                    << "J_" << joint_index << "_velocity_rad_s" << ","
                    << "J_" << joint_index << "_velocity_rad_s_driver" << ","

                    << "T_" << joint_index << "_torque_loadcell" << "," 
                    << "T_" << joint_index << "_raw_loadcell" << ","
                    << "T_" << joint_index << "_torque_des" << ","
                    << "T_" << joint_index << "_torque_grav" << ","
                    << "T_" << joint_index << "_torque_weight" << ","

                    << "I_" << joint_index << "_position_des_rad" << ","
                    << "I_" << joint_index << "_acceleration_des_rad" << ","
                    << "I_" << joint_index << "_stiffness" << ","
                    << "I_" << joint_index << "_damping" << ","

                    << "E_" << joint_index << "_position_des_rad" << ","
                    << "E_" << joint_index << "_stiffness" << ","
                    << "E_" << joint_index << "_damping" << ","
                    << "E_" << joint_index << "_weight_assistance";
        }
    }
    else{
        PLOGE << "AGREE Log File Error";
    }
}

void onedof_manager::writefile(){

    joint_values_t joint_values;
    impedance_control_terms_t impedance_values;

    CSVfile << endl

            << control_mode << ","
            << control_mode_status << ","
            << elapsed_time_ms << ",";

    for(int joint_index=0;joint_index<N_DOFS;joint_index++)
    {
        joint_values = agree_joints[joint_index].get_joint_vals();
        impedance_values = agree_joints[joint_index].get_impedance_control_terms();

        CSVfile << joint_values.incremental_encoder_reading_radians << ","
                << agree_robot.position_rad(0) *rad_to_deg  << ","
                << joint_values.incremental_encoder_speed_radians_sec << ","
                << joint_values.incremental_encoder_speed_radians_sec_driver << ","

                << agree_robot.loadcell_torque_mNm(0)  << ","
                <<  agree_joints[0].ethercat_interface.get_loadcell_reading_mV(agree_joint_error[0])  << ","
                << joint_values.torque_setpoint_mNm << ","
                << agree_robot.robot_weight_torque_mNm[joint_index] <<","
                << agree_robot.total_weight_torque_mNm[joint_index] <<","


                << impedance_values.impedance_control_setpoint_rad << ","
                << interim_acceleration << ","
                << impedance_values.impedance_control_k_mNm_rad << ","
                << impedance_values.impedance_control_d_mNm_rad_per_sec << ","

                << agree_robot.impedance_control_setpoint_rad[joint_index] << ","
                << agree_robot.impedance_control_k_gain_mNm_per_rad[joint_index] << ","
                << agree_robot.impedance_control_d_gain_mNm_per_rad_per_sec[joint_index] << ","
                << agree_robot.weight_compensation_config.weight_assistance;
    }

}

void onedof_manager::closefile(){
    CSVfile.close();
    PLOGI << "AGREE Log File Closed Successfully";
}

///
/// \brief onedof_manager::print_sequential_feedback
///
/// This functions needs to be called at each cycle of the main loop and it sequentially reads the desired parameters from the mini-torque-driver.
/// In this case parameters are received at 250Hz.
///
void onedof_manager::print_sequential_feedback()
{
    //    {0x0614,"raw adc potentiometer linear act",0,0},
    //    {0x0615,"raw adc escon AO0",0,0},
    //    {0x0616,"raw adc escon AO1",0,0},
    //    {0x0617,"motor velocity rad/s",0,1},
    //    {0x0618,"motor acceleration rad/s2",0,1},
    //    {0x0624,"escon setvalue 0to1",0,1},
    //    {0x062C,"torque error mNm",0,1},
    //    {0x062D,"differntial torque error mNm/s",0,1},
    //    {0x062E,"integrated error mNm sec",0,1},

    if (loop_cnt > 5000)
    {
        if (loop_cnt%4 == 0)
        {
            agree_joints[0].ethercat_interface.OUT_system_parameter_type = 0x0624;
            agree_joints[0].ethercat_interface.OUT_system_parameter_value = 0x0000;
        }
        if (loop_cnt%4 == 1)
        {
            agree_joints[0].ethercat_interface.OUT_system_parameter_type = 0x062C;
            agree_joints[0].ethercat_interface.OUT_system_parameter_value = 0x0000;
        }
        if (loop_cnt%4 == 2)
        {
            agree_joints[0].ethercat_interface.OUT_system_parameter_type = 0x062D;
            agree_joints[0].ethercat_interface.OUT_system_parameter_value = 0x0000;
        }
        if (loop_cnt%4 == 3)
        {
            agree_joints[0].ethercat_interface.OUT_system_parameter_type = 0x062E;
            agree_joints[0].ethercat_interface.OUT_system_parameter_value = 0x0000;
        }

        if (agree_joints[0].ethercat_interface.IN_system_parameter_type == 0x0624)  cout << fixed << "T_setpoint: " << convert_ecat_format_to_float(agree_joints[0].ethercat_interface.IN_system_parameter_value) << "\t";
        if (agree_joints[0].ethercat_interface.IN_system_parameter_type == 0x062C)  cout << "T_error: " << convert_ecat_format_to_float(agree_joints[0].ethercat_interface.IN_system_parameter_value) << "\t";
        if (agree_joints[0].ethercat_interface.IN_system_parameter_type == 0x062D)  cout << "T_deriv: " << convert_ecat_format_to_float(agree_joints[0].ethercat_interface.IN_system_parameter_value) << "\t";
        if (agree_joints[0].ethercat_interface.IN_system_parameter_type == 0x062E)  cout << "T_integ: " << convert_ecat_format_to_float(agree_joints[0].ethercat_interface.IN_system_parameter_value) << endl;
    }
}

void onedof_manager::quit(){
    onedof_shared_memory.~onedof_shared_memory_comm();
    stop();
}

void onedof_manager::print_feedback(){
    for (int joint_index=0;joint_index<N_DOFS_MAX;joint_index++) {

        if(agree_joint_error[joint_index])  {PLOGE << "ESMACAT_JOINT_ERR_J"<<joint_index+1  <<": " << agree_joint_error[joint_index]; }
        if(agree_control_error[joint_index]){PLOGE << "ESMACAT_CONTROL_ERR_J"<<joint_index+1<<": " << agree_control_error[joint_index]; }
    }
    std::cout  << "Status: " << dec << robot_control_labels[control_mode] << ": " << control_mode << ">" << control_mode_status << endl
//              << "Torque      [mV]: "   << fixed << setprecision(3)  << agree_joints[0].ethercat_interface.get_loadcell_reading_mV(agree_joint_error[0]) << endl
//               << "Raw value      [mV]: "   << fixed << setprecision(3)  << agree_joints[0].ethercat_interface.get_loadcell_reading_mV(agree_joint_error[0]) << endl
//               << "Bridge voltage      [mV]: "   << fixed << setprecision(3)  << (agree_joints[0].ethercat_interface.get_loadcell_reading_mV(agree_joint_error[0]))/250 << endl
               << "Torque     [mNm]: "   << fixed << setprecision(3)  << agree_robot.loadcell_torque_mNm(0)                               << endl
               << "Gravity:   [mNm]: "   << fixed << setprecision(3)  << agree_robot.robot_weight_torque_mNm(0)                              << endl
               << "Weight:    [mNm]: "   << fixed << setprecision(3)  << agree_robot.total_weight_torque_mNm(0)                    << endl
//               << "Speed:   [rad/s]: "   << fixed << setprecision(3)  << agree_robot.speed_rad_s(0) *rad_to_deg                            << endl
                << "Position   [deg]: "   << fixed << setprecision(3)  << agree_robot.position_rad(0) *rad_to_deg                           << endl
                << "Setpoint   [deg]: "   << fixed << setprecision(3)  << agree_joints[0].get_impedance_control_setpoint_rad() *rad_to_deg << " << " << agree_robot.impedance_control_setpoint_rad(0) *rad_to_deg         << endl
//                << "Feedforward[mNm]: "   << fixed << setprecision(3)  << agree_joints[0].get_impedance_control_torque_mNm() << " << " << agree_robot.weight_compensation_torques_mNm(0)        << endl
                << "K_s:    [Nm/rad]: "   << fixed << setprecision(3)  << agree_joints[0].get_impedance_control_K_mNm_per_rad()*mNm_to_Nm        << " << " << agree_robot.impedance_control_k_gain_mNm_per_rad(0)            << endl
                << "D_d:  [Nm/rad*s]: "   << fixed << setprecision(3)  << agree_joints[0].get_impedance_control_d_mNm_per_rad_per_sec()*mNm_to_Nm << " << " << agree_robot.impedance_control_d_gain_mNm_per_rad_per_sec(0)      << endl
                << "User parameters : "   << agree_robot.weight_compensation_config.human_height_m << " [m] " << agree_robot.weight_compensation_config.human_weight_kg << " [kg] " << agree_robot.weight_compensation_config.weight_assistance*100.0 << " [%]" << endl
               << "Homing offset   : "     << agree_arduino.get_calibration_reference_counts(0) << ": "  << boolalpha << agree_arduino.is_robot_referenced()  << dec << endl
                << "Allocation factor: "   << agree_robot.impedance_control_feedforward_allocation_factor << endl
                  //               << agree_arduino.is_calibration_ready() << endl << endl
//               << "Exercise status : " << exercise_status << endl
//               << "Joint referenced: "   <<
//                  agree_robot.joint_is_referenced << " >> " <<
//                  agree_robot.robot_is_referenced << endl

                  //                << "Hand Position: " << agree_robot.T_total[N_FRAMES-1].position << endl


               << "Use ROS         : "   << onedof_shared_memory.data->use_ros<< endl
                  //                << "End-Stop  [bool]: "   << agree_joints[0].ethercat_interface.get_digital_input(0) << " " << agree_joints[0].ethercat_interface.get_digital_input(1) << endl
                  //                << agree_joints[0].joint_config.hard_stop_lower_limit_degrees << " " << agree_joints[0].joint_config.hard_stop_upper_limit_degrees
                  //                << "Joint positioned: "   << agree_robot.joint_is_positioned << " >> " << agree_robot.joint_is_positioned.prod() << endl
                  //                << "Spring Counterweight: " <<   (-20.75*pow(-agree_joints[1].joint_values.incremental_encoder_reading_radians,2)-14.0*-agree_joints[1].joint_values.incremental_encoder_reading_radians+4.3)*Nm_to_mNm << endl
               << endl;
}

