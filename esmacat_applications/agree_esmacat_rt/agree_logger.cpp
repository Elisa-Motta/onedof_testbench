#include "onedof_logger.h"

namespace logger{

std::string create_header(std::vector<std::string> joints_labels)
{
    std::stringstream header;
    header.str("");
    header << "Elapsed Time(ms)" << ",";

    header << "R_" << joints_labels[0] << "_fault" << ",";
    header << "R_" << joints_labels[0] << "_Load" << ",";
    header << "R_" << joints_labels[0] << "_fLoad" << ",";
    header << "R_" << joints_labels[0] << "_deg" << ",";
    header << "R_" << joints_labels[0] << "_abs_r" << ",";
    header << "R_" << joints_labels[0] << "_fAbs" << ",";
    header << "R_" << joints_labels[0] << "_enc" << ",";
    header << "R_" << joints_labels[0] << "_T_des" << ",";
//    header << "R_" << joints_labels[0] << "_fT_des" << ",";
    header << "R_" << joints_labels[0] << "_T_ff" << ",";
    header << "R_" << joints_labels[0] << "_T_L_err" << ",";
    header << "R_" << joints_labels[0] << "_T_p" << ",";
    header << "R_" << joints_labels[0] << "_T_i" << ",";
    header << "R_" << joints_labels[0] << "_T_d" << ",";
    header << "R_" << joints_labels[0] << "_T_fb" << ",";
    header << "R_" << joints_labels[0] << "_T_tot" << ",";
    header << "R_" << joints_labels[0] << "_t_con" << ",";
    header << "R_" << joints_labels[0] << "_t_g" << ",";
    header << "R_" << joints_labels[0] << "_t_f" << ",";
    header << "R_" << joints_labels[0] << "_t_st" << ",";
    header << "R_" << joints_labels[0] << "_t_des" << ",";
    return header.str();

}

std::string create_dataline(const esmacat_shared_memory_comm& comm, const double elapsed_time)
{
    std::stringstream header;
    header.str("");

    header << elapsed_time << ",";

    header << comm.data->R_J_vals[0].escon_fault << ",";
    header << comm.data->R_J_vals[0].loadcell_reading_mNm << ",";
    header << comm.data->R_J_vals[0].filtered_load_mNm << ",";
    header << comm.data->R_J_vals[0].incremental_encoder_reading_degrees << ",";
    header << comm.data->R_J_vals[0].unfiltered_absolute_encoder_reading_cpt << ",";
    header << comm.data->R_J_vals[0].filtered_absolute_encoder_reading_cpt << ",";
    header << comm.data->R_J_vals[0].incremental_encoder_reading_cpt << ",";
    header << comm.data->R_T_ctrl_vals[0].torque_setpoint_mNm << ",";
//    header << comm.data->R_T_ctrl_vals[0].filt_desired_torque_mNm << ",";
    header << comm.data->R_T_ctrl_vals[0].feedforward_demanded_torque_mNm << ",";
    header << comm.data->R_T_ctrl_vals[0].load_err << ",";
    header << comm.data->R_T_ctrl_vals[0].pterm_mNm << ",";
    header << comm.data->R_T_ctrl_vals[0].iterm_mNm << ",";
    header << comm.data->R_T_ctrl_vals[0].dterm_mNm << ",";
    header << comm.data->R_T_ctrl_vals[0].pid_output_torque_mNm << ",";
    header << comm.data->R_T_ctrl_vals[0].pid_output_torque_mNm + comm.data->R_T_ctrl_vals[0].feedforward_demanded_torque_mNm << ",";
    header << comm.data->R_I_ctrl_vals[0].controller_torque_mNm << ",";
    header << comm.data->R_I_ctrl_vals[0].gravity_torque_mNm << ",";
    header << comm.data->R_I_ctrl_vals[0].friction_comp_torque_mNm << ",";
    header << comm.data->R_I_ctrl_vals[0].soft_stop_torque_mNm << ",";
    header << comm.data->R_I_ctrl_vals[0].torque_setpoint_mNm << ",";

    return header.str();
}

}

