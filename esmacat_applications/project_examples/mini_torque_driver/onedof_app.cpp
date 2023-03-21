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

/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/
/** @brief Constructor for the class
 *
 * Initializes and loads parameters essential for the functioning of the robot
 *
 */
onedof_app::onedof_app()
{
    actuator.set_one_cycle_time_s (get_one_cycle_time_ns()/1000000000.0);
    actuator2.set_one_cycle_time_s (get_one_cycle_time_ns()/1000000000.0);
    actuator3.set_one_cycle_time_s (get_one_cycle_time_ns()/1000000000.0);

}

onedof_app::~onedof_app()
{
    comm.detach_shared_memory();
    std::cout << "object is destructed";
}

/** @brief Method to assign slave index to each of the EtherCAT slaves on the device
 *
 * Connects the etherCAT slaves to their respective pointers that were definedin onedof_app.h
 * Has options to connect with the EtherCAT switch or to a single arm directly
 * Has options to connect an EASE for use as a controller
 *
 */
void onedof_app::assign_slave_sequence()
{
    onedof_app::assign_esmacat_slave_index(&(ease0),(0));
    onedof_app::assign_esmacat_slave_index(&(actuator.controller_interface),(1));
    onedof_app::assign_esmacat_slave_index(&(actuator2.controller_interface),(2));
    onedof_app::assign_esmacat_slave_index(&(actuator3.controller_interface),(3));

}
/** @brief Method to configure slaves
 *
 * Currently blank
 */
void onedof_app::configure_slaves()
{
    IO_Direction d[]={IO_OUTPUT,IO_OUTPUT,IO_OUTPUT,IO_OUTPUT,IO_OUTPUT};
    actuator.controller_interface.configure_dio_direction(d);
    actuator.set_control_mode(actuator_controller_interface::control_mode_t::torque_control);
    actuator.configure_joint_controller_from_file("../../../../onedof_config_files/actuator_series200.json");
    actuator2.configure_joint_controller_from_file("../../../../onedof_config_files/actuator_series200.json");
    actuator3.configure_joint_controller_from_file("../../../../onedof_config_files/actuator_series200.json");

}

/** @brief Commands that are run after EtherCAT has been setup, but before the first loop
 *
 * Used to run encoder calibration sequences, and will eventually have pre-startup checks.
 * Option to write to file are also here
 *
 */
void onedof_app::init()
{
    // Initializing the shared memory
    if (comm.init())
    {
        PLOGI << "User Interface shared memory initialized with key " << static_cast<int>(comm.get_shared_memory_key());    // start the shared memory communication
    }
    else
    {
        PLOGE << "User Interface shared memory initialization has been failed";
        comm.detach_shared_memory();
    }

}

/** @brief This method is the primary loop for the 1Dof application
 *
 * All of the computation required to run the 1Dof setup will be done within this
 *
 */
void onedof_app::loop()
{
    esmacat_err err;

    //actuator.update_joint_variables(elapsed_time_ms,loop_cnt);
//    for (int i=0;i<5;i++)   actuator.controller_interface.set_digital_output(i,true);
//    actuator.controller_interface.get_raw_loadcell_reading();
    //write2sharedMemory();

    if (loop_cnt%1000 == 0){
        cout << "\nLoop count : " << elapsed_time_ms << endl;
        cout << "Encoder reading [cpt] : " <<     actuator.get_incremental_encoder_reading_radians() << " " << actuator2.get_incremental_encoder_reading_radians() << " " << actuator3.get_incremental_encoder_reading_radians() << endl;

//        cout << "Setpoint " << endl;
//        cout << "Computed velocity [rad/s] : " << endl;
//        cout << "Setpoint [mA] : " <<  << endl;
//        cout << "Loadcell reading [raw] : " << ecat_slave.controller_interface.get_raw_loadcell_reading() << endl;
//          cout << "Photocell reading :" << home << endl;

    }
    /**
    switch (comm.data->mode)
    {
    case 1:
    {
        actuator.controller_interface.set_escon_enable(true);
//        actuator.control_torque_with_soft_stop_mNm(0.0,elapsed_time_ms);
        actuator.control_torque_with_aux_input_mNm(0.0,30000,elapsed_time_ms);
        write2sharedMemory(); //Outputting data to shared memory for display by separate executable
        break;
    }
    case 2:
    {
        //Setpoint for motion
        float oscillation_period_ms = 8000.0;
        control_setpoint_rad = ( 0.0 + 10.0*sin(2*M_PI * elapsed_time_ms/oscillation_period_ms)) *M_PI/180; //Sinusoidal motion of actuator
        //Passing to Controller
        actuator.impedance_control(static_cast<float>(control_setpoint_rad),gravity_torque_mNm,static_cast<float>(elapsed_time_ms));
        write2sharedMemory(); //Outputting data to shared memory for display by separate executable
        break;
    }
    default:
    {
        if (comm.data->stop == 1)
        {
            stop();
        } // Shared memory trigger to stop
        break;
    }
    }
    **/
//    if (loop_cnt > 100000 || comm.data->stop == 1)
//    {
//        stop();
//    } // Shared memory trigger to stop
}
/** @brief This method outputs collected and calculated data to shared memory for other executables that may read and use them
 *
 * The primary users of this data are the display executable and the GUI
 *
 */
void onedof_app::write2sharedMemory()
{
    comm.data->R_J_vals[0] = actuator.get_joint_vals();
    comm.data->R_T_ctrl_vals[0] = actuator.get_torque_control_terms();
    comm.data->R_I_ctrl_vals[0] = actuator.get_impedance_control_terms();
    comm.data->elapsed_time_ms = elapsed_time_ms;
    comm.data->last_joint_index = 2;
    comm.data->loop_cnt = loop_cnt;
    comm.data->elapsed_time_ms = elapsed_time_ms;
}

void onedof_app::print_feedback()
{

    if (loop_cnt > 5000)
    {
        if (loop_cnt%3 == 0)
        {
            actuator.controller_interface.OUT_system_parameter_type = 0x0624;
            actuator.controller_interface.OUT_system_parameter_value = 0x0000;
        }
        if (loop_cnt%3 == 1)
        {
            actuator.controller_interface.OUT_system_parameter_type = 0x0617;
            actuator.controller_interface.OUT_system_parameter_value = 0x0000;
        }
        if (loop_cnt%3 == 2)
        {
            actuator.controller_interface.OUT_system_parameter_type = 0x0618;
            actuator.controller_interface.OUT_system_parameter_value = 0x0000;
        }
        if (actuator.controller_interface.IN_system_parameter_type == 0x0624)  cout << "sp: " << convert_ecat_format_to_float(actuator.controller_interface.IN_system_parameter_value) << "\t";
        if (actuator.controller_interface.IN_system_parameter_type == 0x0617)  cout << "v: " << convert_ecat_format_to_float(actuator.controller_interface.IN_system_parameter_value) << "\t";
        if (actuator.controller_interface.IN_system_parameter_type == 0x0618)  cout << "a: " << convert_ecat_format_to_float(actuator.controller_interface.IN_system_parameter_value) << endl;
    }

}
