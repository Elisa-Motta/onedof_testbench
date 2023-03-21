
/** @file
 * @brief This file contains the definition of the functions associated with the user-defined
 * application for the Esmacat slave project */
/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include "agree_manager.h"
#include <math.h>

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/

/**
 * @brief Constructor for the class that holds the initialization
 */
agree_manager::agree_manager(){

    // Flag for testbed
    agree_parameters.use_testbed = false;
    // Flag for data record
    agree_parameters.save_data   = true;
    // Set Side of the AGREE robot
    agree_parameters.side        = LEFT;
    // Set Error of the AGREE robot
    agree_error                  = NO_ERR;

    // Read parameters
    agree_parameters.read_agree_parameters();
    // Set parameters to motor controller class
    agree_parameters.set_agree_parameters(agree_joints);



    PLOGW << "AGREE Manager started";
}

/**
 * @brief Destructor for the class
 */
agree_manager::~agree_manager(){

   PLOGW << "AGREE Manager destroyed";
}



/**
 * @brief Identifies the actual Esmacat slave sequence in the EtherCAT communication chain.
 */
void agree_manager::assign_slave_sequence(){

    // tell the master what type of slave is at which point in the chain
    for(int slave_index=0;slave_index<N_DOFS;slave_index++)
    {
        // Assign slave index for desired slaves
        assign_esmacat_slave_index(&agree_joints[slave_index],slave_index);
        // Pass agree_joints class to agree_robot class
        agree_robot.assign_esmacat_slave_index(&agree_joints[slave_index],slave_index);
    }
}

/**
 * @brief Configure your Esmacat slave.
 * Link Esmacat slave object with the actual Esmacat slave in the EtherCAT communication chain.
 * Functions beginning with 'configure_slave' must only be executed in this function
 */
void agree_manager::configure_slaves(){

    // add initialization code here
    // Functions starting with "configure_slave" work only in configure_slaves() function
}

/**
 * @brief Initialization that needs to happen on the first iteration of the loop
 */
void agree_manager::init()
{
    //Initialize Shared Memory for ROS Interface
    agree_shared_memory.init();

    // Reset EPOS4 Errors
    agree_robot.reset_fault();

    // Set Mode of Operation (6 = Homing Mode, 10 = Cyclic Synchronous Torque)
    agree_robot.set_mode_operation(CYCLIC_SYNCHRONOUS_TORQUE);

    // Set initial state of the Real-Time Controller
    agree_robot.status = STOP;
    agree_shared_memory.data->command = STOP;

    //Read joint sensors and save variables
    agree_robot.position_rad            = agree_robot.get_current_joint_angles_rad();
    agree_robot.velocity_rad_s          = agree_robot.get_current_joint_velocity_rad_s();
    agree_robot.loadcell_torque_mNm     = agree_robot.get_current_loadcell_torque_mNm();

    // First iteration functions
    agree_robot.read_agree_robot_parameters(agree_parameters.side);

    agree_parameters.set_agree_parameters(agree_joints);


    // Write2file initialization
    initialize_write2file();

    PLOGI << "AGREE Manager initialized";

}

/**
 * @brief Executes functions at the defined loop rate: 1kHz
 */
void agree_manager::loop(){

    // Update variables from shared memory
    readsharedmemory();

    //Read joint sensors and save variables
    agree_robot.position_rad            = agree_robot.get_current_joint_angles_rad();
    agree_robot.velocity_rad_s          = agree_robot.get_current_joint_velocity_rad_s();
    agree_robot.loadcell_torque_mNm     = agree_robot.get_current_loadcell_torque_mNm();


    // Compute Direct Kinematics and Inverse Dynamics
    if(agree_parameters.use_testbed){
        agree_robot.gravity_torques_Nm[0]             = agree_robot.get_inverse_dynamics_torque_onedof(0.0,agree_robot.position_rad[0]);
        agree_robot.weight_compensation_torques_Nm[0] = agree_robot.get_inverse_dynamics_torque_onedof(agree_shared_memory.data->robot_config.weight_compensation_level,agree_robot.position_rad[0]);
    }
    else{
        agree_robot.direct_kinematics();
        agree_robot.gravity_torques_Nm = agree_robot.get_inverse_dynamics_torques();
    }
    // Save sensors data into joint_status class, for shared memory communication
    agree_robot.save_joint_status();

    // Set application elapsed time to motor controller classes
    agree_robot.set_elapsed_time_ms(elapsed_time_ms);

    double support_trajectory;
    double spring_counterweight_mNm;

    // Set Open-Loop Torque Controller Parameters
    float feedforward_vibration_mNm = sin(2*M_PI*elapsed_time_ms/20.0)*2500.0;
    float coulomb_torque_mNm = 450.0;
    float coulomb_velocity_rad_s = static_cast<float>(1E-3);
    float feedforward_friction_compensation_mNm = coulomb_torque_mNm*tanh(agree_joints[0].velocity_rad_per_s/coulomb_velocity_rad_s);
    float gearbox_efficiency = 0.9;


    // To be executed during first 500ms
    if (loop_cnt < 250 ) {

        // Reset EPOS4 Errors
        agree_robot.reset_fault();

    }
    // Check for residual errors on EPOS4
    else if (loop_cnt == 250) {
        agree_error = agree_robot.get_epos4_errorcode();
        if(agree_error)
        {
            PLOGE << "EPOS4 Errors not cleared";
            quit();
        }
        else{
            PLOGW << "Reset EPOS4 Driver";
        }
    }
    // To be executed from 500ms to 1000ms
    else if (loop_cnt < 1000) {

        // Stop motors
        agree_robot.stop_motor();

        // Reset joint angles of the robot
        //agree_robot.reset_joint_angles();
    }
    // Stop driver
    else if (loop_cnt == 1000) {
        PLOGW << "Stop EPOS4 Driver";
    }
    else {

        if(agree_robot.command == HOMING){
            // Set Homing Mode of Operation
            agree_robot.set_mode_operation(HOMING_MODE);
            // TODO: add check. When using exo, joint need to be referenced before starting!

            // Run homing mode on all active joints
            agree_robot.run_homing_mode();

            // Check if all active joints are correctly referenced by checking product along boolean array
            if (agree_robot.joint_is_referenced.prod())
            {
                agree_robot.status = HOMING_DONE;
                agree_robot.robot_is_ready = true;
            }
            else
            {
                agree_robot.status = HOMING;
                agree_robot.robot_is_ready = false;
            }
        }
        else {
            // Set Cyclic Synchronous Torque Mode
            agree_robot.set_mode_operation(CYCLIC_SYNCHRONOUS_TORQUE);
        }
        if(agree_robot.robot_is_ready){

         switch(agree_robot.command){

        case STOP:

            // Stop motors
            agree_robot.stop_motor();

            // Update robot status
            agree_robot.status = STOP;

            break;

        case NULLTORQUE:

            // Start motors
            agree_robot.start_motor();

            // Set Impedance Controller Gains and Impedance Controller
            for (int joint_index=0;joint_index<N_DOFS;joint_index++) {
            agree_joints[joint_index].set_impedance_control_gain(0.0,0.1);
            agree_joints[joint_index].set_impedance_control_rad(0.0,0.0);
            }

            // Start motor
            agree_robot.start_motor();

            // Update status
            agree_robot.status = NULLTORQUE;

            break;

        case POSITION:

            // At first iteration save starting time and position
            if(agree_robot.command != agree_robot.prev_command)
            {
                elapsed_time_ms_offset = elapsed_time_ms;
                encoder_position_offset = agree_robot.position_rad(0);
            }

            // TODO: TAKE K AND D FROM SHARED MEMORY (COMING FROM SMARTBOX_ROS_INTERFACE FSM)
            agree_joints[0].set_impedance_control_gain(50.0,10.0);

            agree_joints[0].start_motor();

            // Final position received from shared-memory setpoint
            support_trajectory = encoder_position_offset+(0.0/180.0*M_PI - encoder_position_offset)/2.0*(tanh((elapsed_time_ms-elapsed_time_ms_offset)/1000.0-exp(1.0))+1.0);

            agree_joints[0].set_impedance_control_rad(support_trajectory,0.0);

//            if(abs(agree_joints[0].joint_task_control_command.impedance_control_setpoint_rad - agree_joints[0].position_rad)>=M_PI/64.0)
//            {
//                agree_robot.status = POSITION;
//            }
//            else {
//                agree_robot.status = POSITION_DONE;
//            }
//            break;
            if(abs(0.0/180.0*M_PI - agree_joints[0].position_rad)>=M_PI/64.0)
            {
                agree_robot.status = POSITION;
            }
            else {
                agree_robot.status = POSITION_DONE;
            }
             break;


        case GRAVITY:
            // TODO: Never checked with J2 Exo!
            spring_counterweight_mNm = (-20.75*pow(agree_joints[1].position_rad,2)-14.0*agree_joints[1].position_rad+4.3)*Nm_to_mNm;
            // Set Impedance Controller Gains
            for (int joint_index=0;joint_index<N_DOFS;joint_index++) {
            agree_joints[joint_index].set_impedance_control_gain(0.0,agree_joints[joint_index].joint_task_control_command.impedance_control_d_gain_mNm_per_rad_per_sec);
            agree_joints[0].set_impedance_control_rad(0.0,-agree_robot.gravity_torques_Nm[0]);
            }

//            agree_joints[1].set_impedance_control_gain(0.0,agree_joints[1].joint_task_control_command.impedance_control_d_gain_mNm_per_rad_per_sec);
//            agree_joints[1].set_impedance_control_rad(0.0,-spring_counterweight_mNm);
            //agree_joints[1].start_motor();
            // Start motor
//            agree_robot.start_motor();
            // Update status
            agree_robot.status = GRAVITY;

            break;

        case FREEZE: case FREEZE_SOFT:
            // TODO: check if loadcell is symmetric and re-calibrate.

            // Set Impedance Controller Gains
            agree_robot.set_impedance_control_gain();
            // Set Impedance Controller
            agree_robot.run_impedance_control();
            // Start motor
            agree_robot.start_motor();
            // Update status
            agree_robot.status = FREEZE;
            break;

        case IMPEDANCE:

            if(agree_robot.command != agree_robot.prev_command) elapsed_time_ms_offset = elapsed_time_ms;

            // Set Impedance Controller Gains
            agree_joints[0].set_impedance_control_gain( agree_joints[0].joint_task_control_command.impedance_control_k_gain_mNm_per_rad,
                                                        agree_joints[0].joint_task_control_command.impedance_control_d_gain_mNm_per_rad_per_sec);
            agree_robot.start_motor();
            support_trajectory = sin(2*3.1415*(elapsed_time_ms-elapsed_time_ms_offset)/5000.0);
            agree_joints[0].set_impedance_control_rad(support_trajectory*M_PI/4,0.0);
            agree_robot.status = IMPEDANCE;
            break;

         case TORQUE:

             if(agree_robot.command != agree_robot.prev_command) elapsed_time_ms_offset = elapsed_time_ms;

             // Start motors
             agree_robot.start_motor();
             // Create sine wave
             support_trajectory = sin(2*3.1415*elapsed_time_ms/2000.0);
             // Set Torque Controller
             agree_joints[0].set_torque_control_mNm(0.1*(elapsed_time_ms-elapsed_time_ms_offset));
//             agree_joints[0].set_torque_control_mNm(2000.0);//+1000*support_trajectory);

             break;

         case CURRENT:

             if(agree_robot.command != agree_robot.prev_command) elapsed_time_ms_offset = elapsed_time_ms;

             // Start motors
             agree_robot.start_motor();

             agree_joints[0].set_feedforward_torque_mNm((0.1*(elapsed_time_ms-elapsed_time_ms_offset)+feedforward_vibration_mNm+feedforward_friction_compensation_mNm)/gearbox_efficiency);

             // Update status
             agree_robot.status = CURRENT;

             break;

        case IMPEDANCE_EXT: case PASSIVE:

            // Set Impedance Controller Gains
            agree_robot.set_impedance_control_gain();
            // Set Impedance Controller
            agree_joints[0].set_impedance_control_rad(agree_joints[0].joint_task_control_command.impedance_control_setpoint_rad,-agree_robot.gravity_torques_Nm[0]);
            // Start motor
            agree_robot.start_motor();
            // Set Status
            agree_robot.status = agree_robot.command;
            break;

        case EXIT:
            PLOGE << "AGREE Manager shutting down ...";
            agree_robot.status = EXIT;
            break;

        }

    }
    }

    if (loop_cnt%1000 == 0)
    {
        // Get driver errors
        agree_error = agree_robot.get_epos4_errorcode();
        // Print debug stuff on screen
        print2screen();
    }



    if (loop_cnt%10 == 0 && agree_parameters.save_data)
    {
        // Write to Log file
        write2file();
    }

    prev_state = state;
    agree_robot.prev_command = agree_robot.command;

    if (loop_cnt > APPLICATION_TIMEOUT_MS)
    {
        agree_robot.stop_motor();
        agree_robot.status = STOP;
    }

    // Write to Shared memory
    write2sharedmemory();

    if(loop_cnt > APPLICATION_TIMEOUT_MS+500 || agree_robot.command == EXIT)
    {
        agree_shared_memory.data->status = EXIT;
        CSVfile.close();
        agree_manager::quit();
    }

}

void agree_manager::initialize_write2file(){
    time_t t = time(nullptr);   // get time now
    struct tm * now = localtime( & t );
    char buffer [80];
    strftime (buffer,80,"AGREE-Manager-%Y-%m-%d-%H-%M-%S.csv",now);
    CSVfile.open (buffer);
    if(CSVfile.is_open())
    {
        PLOGI << "Log File Created Successfully";

        CSVfile << endl

                << "J_status" << ","
                << "J_command" << ","
                << "J_elapsed_time_ms" << ",";

        for(int joint_index=0;joint_index<N_DOFS;joint_index++)
        {
            CSVfile << "J_" << joint_index << "_position_rad_" ","
                    << "J_" << joint_index << "_velocity_rad_s" << ","

                    << "T_" << joint_index << "_torque_loadcell" << ","
                    << "T_" << joint_index << "_torque_des" << ","

                    << "I_" << joint_index << "_position_des_rad" << ","
                    << "I_" << joint_index << "_stiffness" << ","
                    << "I_" << joint_index << "damping" << ",";
        }
    }
    else{
        PLOGE << "Log File Error";
    }
}

///
////// \brief agree_manager::write2file
/// This functions appends sensors signals, control variables and setpoints to the .csv log file at each timestamp (i.e. 1kHz)
///
void agree_manager::write2file(){

    CSVfile << endl

            << agree_robot.status << ","
            << agree_robot.command << ","
            << elapsed_time_ms << ",";

    for(int joint_index=0;joint_index<N_DOFS;joint_index++)
    {

        CSVfile << agree_joints[joint_index].joint_status.incremental_encoder_position_radians << ","
                << agree_joints[joint_index].joint_status.velocity_rad_per_s << ","

                << agree_joints[joint_index].joint_status.loadcell_torque_mNm << ","
                << agree_joints[joint_index].joint_torque_control_status.desired_torque_mNm << ","

                << agree_joints[joint_index].joint_impedance_control_status.impedance_control_setpoint_rad << ","
                << agree_joints[joint_index].joint_impedance_control_status.impedance_control_k_mNm_per_rad << ","
                << agree_joints[joint_index].joint_impedance_control_status.impedance_control_d_mNm_s_per_rad << ",";
    }

}

// TODO: ADD TORQUE STATUS + ROBOT STATUS

void agree_manager::write2sharedmemory(){

    agree_shared_memory.data->status        = agree_robot.status;
    agree_shared_memory.data->elapsed_time  = elapsed_time_ms;
    agree_shared_memory.data->loop_cnt      = loop_cnt;

    for(int joint_index=0;joint_index<N_DOFS;joint_index++){
        agree_shared_memory.data->joint_status[joint_index]                   = agree_joints[joint_index].joint_status;
        agree_shared_memory.data->joint_impedance_control_status[joint_index] = agree_joints[joint_index].joint_impedance_control_status;
    }
}

void agree_manager::readsharedmemory(){

    agree_robot.command                     = agree_shared_memory.data->command;
//    agree_robot.robot_config                = agree_shared_memory.data->robot_config;

    for(int joint_index=0;joint_index<N_DOFS_MAX;joint_index++){
        agree_joints[joint_index].joint_task_control_command.impedance_control_k_gain_mNm_per_rad           = agree_shared_memory.data->joint_task_control_parameters[joint_index].impedance_control_k_gain_mNm_per_rad;
        agree_joints[joint_index].joint_task_control_command.impedance_control_d_gain_mNm_per_rad_per_sec   = agree_shared_memory.data->joint_task_control_parameters[joint_index].impedance_control_d_gain_mNm_per_rad_per_sec;
        agree_joints[joint_index].joint_task_control_command.impedance_control_setpoint_rad                 = agree_shared_memory.data->joint_task_control_parameters[joint_index].impedance_control_setpoint_rad;

        agree_robot.impedance_control_k_gain_mNm_per_rad[joint_index]                                       = agree_joints[joint_index].joint_task_control_command.impedance_control_k_gain_mNm_per_rad;
        agree_robot.impedance_control_d_gain_mNm_per_rad_per_sec[joint_index]                               = agree_joints[joint_index].joint_task_control_command.impedance_control_d_gain_mNm_per_rad_per_sec;
        agree_robot.impedance_control_setpoint_rad[joint_index]                                             = agree_joints[joint_index].joint_task_control_command.impedance_control_setpoint_rad;
    }
}

void agree_manager::print2screen(){
    std::cout   << "Status: " << agree_robot.status << endl
                << "Torque     [mNm]: "   << fixed << setprecision(3)  << agree_robot.loadcell_torque_mNm.transpose()                   << endl
                << "Position   [deg]: "   << fixed << setprecision(3)  << agree_robot.position_rad.transpose()*rad_to_deg               << endl
                << "Setpoint   [rad]: "   << fixed << setprecision(3)  << agree_robot.impedance_control_setpoint_rad.transpose()*rad_to_deg        << endl
                << "Gravity:   [mNm]: "   << fixed << setprecision(3)  << agree_robot.gravity_torques_Nm.transpose()                    << endl
                << "K_stif: [Nm/rad]: "   << fixed << setprecision(3)  << agree_robot.impedance_control_k_gain_mNm_per_rad.transpose()  << endl
                << "Joint referenced: "   << agree_robot.joint_is_referenced << " " << agree_robot.joint_is_referenced.prod()           << endl
                << "Spring Counterweight: " <<   (-20.75*pow(-agree_joints[1].position_rad,2)-14.0*-agree_joints[1].position_rad+4.3)*Nm_to_mNm << endl
                << endl;
}

void agree_manager::quit(){
    agree_shared_memory.~esmacat_shared_memory_comm();
    stop();
}
