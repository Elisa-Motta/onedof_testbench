
#include "onedof_rehamove_app.h"

#include <stdlib.h>
#include <cstdio>
#include <string>
#include <sstream>

#include <libconfig.h++>

#define CALIBRATION_VALUES  "Calibrationvalues"
#define L0_NODE_NAME "Imin"
#define L1_NODE_NAME "PWmin"
#define L2_NODE_NAME "Imin"
#define L3_NODE_NAME "Imin"

#define CHARGE_DURATION 4000

// USER calibration parameters
float A=0.0;
float B=0.0;
float C=0.0;
float D=0.0;
float H=1.70;
float W=60.0;
float E=0.0;
float F=0.0;
float G=0.0;

// USER ID
char userID[10];

int g;
ofstream CSV_calibration_file;
ifstream CSV_calibration_file_read;
ofstream CSV_user_file;
ifstream CSV_user_file_read;
//ofstream CSV_rehamove_file;

//channel MOTIMOVE
int ch_motimove = 0;

using namespace std;

void read_calibration_file(vector <double> &calib);


void onedof_rehamove_app::FSM(){

    write_rehamove_file();

    //TODO: read shared memory (before compensation?)
    act_pos = c.data->joint_values->incremental_encoder_reading_radians;
    des_pos = static_cast<double>(c.data->impedance_control_terms->impedance_control_setpoint_rad);
    act_torque = c.data->joint_values->filtered_load_mNm;
    des_torque = c.data->impedance_control_terms->torque_setpoint_mNm;
    act_velocity = c.data->joint_values->incremental_encoder_speed_radians_sec;

    // torque setpoint is computer as: gravity_torque_mNm + controller_torque_mNm + friction_comp_torque_mNm + soft_stop_torque_mNm
    torque_impedance=c.data->impedance_control_terms->torque_setpoint_mNm;

//    c.data->impedance_control_command->impedance_control_k_gain_mNm_per_rad = stiffness_nm_rad;
//    c.data->impedance_control_command->impedance_control_d_gain_mNm_per_rad_per_sec = damping_nms_rad;

    if(change_mode && rehamove_mode!=602 && rehamove_mode!=601 ){
        cout << " Mode changed to: ";
    }

    switch (rehamove_mode)
    {
    case 0:
    {
        if(change_mode){
            change_mode=false;
            cout << " EXIT mode" << endl;
            close_rehamove_file();
            close_calibration_file();
        }
        c.data->control_mode_command = robot_control_mode_t::quit;
        ml_update.enable_channel[Smpt_Channel_Red] = false;
        ml_update.enable_channel[Smpt_Channel_Blue] = false;
        fs.fes_State.active[ch_motimove] = false;
        c.data->stop = true;
        break;
    }

    case 1:
    {
        // Executed only at mode change
        if( change_mode){
            change_mode=false;
            cout << " STOP mode" << endl;
            read_calibration_file(calib);
            coeff_weight_assistance = E;
        }
        stiffness_nm_rad = 5.0;
        damping_nms_rad = 1.0;
        coeff_allocation = 0.0;
        c.data->control_mode_command = robot_control_mode_t::standby;
        ml_update.enable_channel[Smpt_Channel_Red] = false;
        ml_update.enable_channel[Smpt_Channel_Blue] = false;
        fs.fes_State.active[ch_motimove] = false;
        break;
    }

    case 2:
    {
        // Executed only at mode change
        if( change_mode){
            change_mode=false;
            cout << " HOMING mode" << endl;
            stiffness_nm_rad = 5.0;
            damping_nms_rad = 1.0;
        }

           c.data->impedance_control_command->impedance_control_k_gain_mNm_per_rad = stiffness_nm_rad;
           c.data->impedance_control_command->impedance_control_d_gain_mNm_per_rad_per_sec = damping_nms_rad;
//        stiffness_nm_rad = 5.0;
//        damping_nms_rad = 1.0;
        c.data->control_mode_command = robot_control_mode_t::homing_control;
        ml_update.enable_channel[Smpt_Channel_Red] = false;
        ml_update.enable_channel[Smpt_Channel_Blue] = false;
        fs.fes_State.active[ch_motimove] = false;

        break;
    }

    case 3:
    {
        if(change_mode){
            change_mode=false;
            cout << " FES CALIBRATION COMPLETED" << endl;
            q=0;
            corrente=0;
            durata=0;
            ml_update.enable_channel[Smpt_Channel_Red] = false;
            ml_update.enable_channel[Smpt_Channel_Blue] = false;
            fs.fes_State.active[ch_motimove] = false;
        }
        c.data->control_mode_command = robot_control_mode_t::standby;
        //if(loop_count%100==0){
        //    stimulation_calibration();
        //}
        break;
    }

    case 103:
    {
         if(change_mode){
             change_mode=false;
             cout << " END Calibration mode" << endl;
//             ml_update.enable_channel[Smpt_Channel_Red] = false;
         }
        //Imin=7;
        //PWmin=265;
        //Imax=16;
        //PWmax=332;

        read_calibration_file(calib);

        A = Imin;
        B = PWmin;
        C = Imax;
        D = PWmax;

        open_calibration_file();
        write_calibration_file();
        close_calibration_file();

        cout << " MIN CURRENT" << Imin << endl;
        cout << " MIN PULSEWIDTH" <<PWmin << endl;
        cout << " MAX CURRENT" << Imax << endl;
        cout << " MAX PULSEWIDTH" << PWmax << endl;

        rehamove_mode = 3;
        change_mode = true;
        break;
    }

    case 101:
    {
        if(change_mode){
            change_mode=false;
            cout << " STIM CALIBRATION mode" << endl;
            stimulation_initialization_motimove();
            corrente=5;
            durata=200;
            q_calib=0.0;
            campioni=0;

            stiffness_nm_rad = 60.0;
            damping_nms_rad = 6.0;

        }

        // STANDARD CALIBRATION IN FREEZE
        c.data->control_mode_command = robot_control_mode_t::freeze_control;
        // Giunto molto rigido 60.0/40.0 - 6.0/4.0
        c.data->impedance_control_command->impedance_control_k_gain_mNm_per_rad = stiffness_nm_rad;
        c.data->impedance_control_command->impedance_control_d_gain_mNm_per_rad_per_sec = damping_nms_rad;
        if(campioni>3000){
            if(loop_count%100==0){
                stimulation_calibration_motimove();
            }
        }

        campioni++;
        break;
    }

    case 102:
    {
        if(change_mode){
            change_mode=false;
            cout << " STOP CALIBRATION mode" << endl;
        }
        Imax=round(corrente);
        PWmax=round(durata);
        rehamove_mode=103;
        change_mode = true;
        ml_update.enable_channel[Smpt_Channel_Red] = false;
        ml_update.enable_channel[Smpt_Channel_Blue] = false;
        fs.fes_State.active[ch_motimove] = false;

        break;
    }

    case 55:
    {
      if(change_mode){
          change_mode=false;
          cout << "Trial mode" << endl;

          stimulation_initialization_motimove();

          // Read calibration file for FES parameters
          read_calibration_file(calib);

//            coeff_weight_assistance = E;

          // Insert values manually for the moment
            threshold_pos_error=-2.0;
            threshold_std_pos_error=2.0;
            coeff_weight_assistance = 0.25;

            coeff_allocation=0.0;


//            close_calibration_file();

          campioni=0;
          num_iter=0;
          counter_t = 0;
          counter_supp = 0;
          q_adjusted=0.2;

          // Stiffness and damping are set a priori
          stiffness_nm_rad = 5.0;
          damping_nms_rad = 1.0;

          // Fatigue parameter
//            K_fatigue = 30.0;

          // Initialize waveform counter
          counter_waveform = 0;
          counter_iteration = 0;
//            change_alloc = 0;
//            counter_q_adj=0;

          // Reset errors & velocity single repetition
          sum_pos_err = 0;
//            sum_vel = 0;
          mean_position_error=0;
//            mean_velocity = 0;
          std_position_error = 0;
//            std_velocity = 0;

          // Reset errors across repetitions
          sum_mean_position_error = 0;
          avg_mean_position_error = 0;
          sum_std_mean_position_error = 0;
          avg_std_position_error = 0;

          avg_delta = 0.0;
          avg_mean = 0.0;
          avg_M2 = 0.0;

          // Velocity across blocks
//            sum_block_vel = 0;
//            mean_block_vel = 0;
//            std_block_vel = 0;

//            block_delta =0.0;
//            block_mean = 0.0;
//            block_M2 = 0.0;

          // (Re)set parameters for charge waveform
          charge_old= 0.0;
          charge_dot = 0.0;
          charge_dot_old= 0.0;
          T_limit = 0.65;

          count_good = 0;
          count_bad = 0;

      }

      // Write shared memory
      c.data->control_mode_command = robot_control_mode_t::impedance_control;
      c.data->impedance_control_command->impedance_control_k_gain_mNm_per_rad         = stiffness_nm_rad;
      c.data->impedance_control_command->impedance_control_d_gain_mNm_per_rad_per_sec = damping_nms_rad;
//        c.data->rehamove_mean_mean_position_error = avg_mean_position_error;
      c.data->rehamove_mean_std_mean_position_error = threshold_std_pos_error;

      c.data->impedance_control_command->impedance_control_feedforward_allocation_factor=coeff_allocation;
      c.data->arm_weight_compensation_config.weight_assistance = coeff_weight_assistance;

      c.data->rehamove_mean_position_error=mean_position_error;
      c.data->rehamove_std_mean_position_error=std_position_error;
//        c.data->rehamove_mean_velocity = mean_velocity;
      c.data->rehamove_iteration = num_iter;

      c.data->rehamove_charge_adj = q_adjusted;
      c.data->rehamove_charge = q;

      //q = charge_waveform_generation_new(counter_iteration,counter_waveform)*q_adjusted; //se forma d'onda secondo traiettoria
      //q = charge_waveform_generation(counter_iteration,counter_waveform)*q_adjusted; //se forma d'onda secondo torque con inizio non nullo
      //q = ((charge_waveform_generation(counter_iteration,counter_waveform)-q0)*2)*q_adjusted;  //se forma d'onda secondo torque con inizio nullo
      q = (charge_waveform_generation(counter_iteration,counter_waveform)-q0/2)*q_adjusted*1.2;
      q = std::max(0.0,std::min(1.0,q));

      if(loop_count%10==0){
                  stimulation_torque_motimove(calib);
              }

              // Compute trajectory error with/without sign
              if(counter_waveform<EXERCISE_DURATION/2){

                  // Compute position error
                  pos_err=des_pos-act_pos;
                  // cumulate position error
                  sum_pos_err+=pos_err;

                  // Compute Standard Deviation with streaming data
                  delta = abs(pos_err) - mean;
                  mean = mean + delta/static_cast<double>(counter_waveform+1);
                  M2 = M2+delta*(abs(pos_err)-mean);
             }

              // COMPUTE MEAN POS ERR AND STD WHEN FLEXION IS OVER
              if(counter_waveform==EXERCISE_DURATION/2){

                  // Count all iterations
                  num_iter++;

                  // Count iterations up to ten (OR FIVE???)
                  counter_rep++;

                  // Compute mean pos err and std of a single iteration
                  mean_position_error=sum_pos_err/counter_waveform;
                  std_position_error = sqrt(M2/(EXERCISE_DURATION/2-1));

                  // Compute averaged position error across rep
                  sum_std_mean_position_error+=std_position_error;
                  avg_std_position_error = sum_std_mean_position_error/num_iter;

                  // Compute averaged std across rep (standard deviation delle medie)
                  avg_delta = abs(mean_position_error) - avg_mean;
                  avg_mean = avg_mean + avg_delta/static_cast<double>(num_iter);
                  avg_M2 = avg_M2+avg_delta*(abs(mean_position_error) - avg_mean);
                  avg_std_position_error = sqrt(avg_M2/(num_iter/2));


                  // Reset parameters for charge waveform
                  charge_old= 0.0;
                  charge_dot = 0.0;
                  charge_dot_old= 0.0;

        }

              if(num_iter > 1){

                  // Iterative learning

                  if (counter_waveform>=4500 && counter_waveform<4501){

                      //            double threshold_pos_error = -3.7;
                      //            double threshold_std_pos_error = 2.5;
                      double K_adjusted = 0.02;
                      double K_vol = 0.05;
                      cout << "**************************" << endl;
                      cout << "Iterative Learning Control" << endl;
                      cout << "Mean error: " << mean_position_error*rad_to_deg << endl;
                      cout << "Std  error: " << std_position_error*rad_to_deg << endl;
                      cout << "Thr error: " << threshold_pos_error<< endl;
                      cout << "Thr std: " << threshold_std_pos_error << endl;

                      if((mean_position_error-std_position_error)*rad_to_deg < (threshold_pos_error-threshold_std_pos_error)){


                          cout << "Slow movement: increase charge" << endl;
                          q_adjusted += -K_adjusted*((mean_position_error-std_position_error)*rad_to_deg-(threshold_pos_error-threshold_std_pos_error));
                          count_good = 0;

                      }
                      else if((mean_position_error+std_position_error)*rad_to_deg > (-threshold_pos_error+threshold_std_pos_error)){
                          cout << "Fast movement: decrease charge" << endl;

                          q_adjusted += -K_adjusted*(((mean_position_error+std_position_error)*rad_to_deg-(-threshold_pos_error+threshold_std_pos_error)));
                          //count_good = 0;

                      }
                      else {
                          count_good += 1;
                          if (count_good >= 5)
                          {
                            q_adjusted = q_adjusted - K_vol;
                            cout << "Good performance, let's try to decrease charge" << endl;
                          }
                          else {
                            cout << "Ok movement: Ok charge" << endl;
                          }
                      }



                      // Copyright SDG please.
      //                q_adjusted = std::max(0.0,std::min(1.0,q_adjusted));
                      // OPTION 2:
                       q_adjusted = std::max(0.0,std::min(1.5,q_adjusted));

                       cout << "Charge adjusted: " << q_adjusted << endl;

                       if (q_adjusted == 1.5)
                       {
                          count_bad += 1;
                          if (count_bad >= 5 && (mean_position_error-std_position_error)*rad_to_deg < (threshold_pos_error-threshold_std_pos_error))
                          {
                            if (coeff_allocation == 1.0)
                            {
                              cout << "Subject is not able to perform movement without full motor action" << endl;
                            }
                            else {
                              coeff_allocation += 0.25;
                              count_bad = 0;
                              q_adjusted = 0.3;
                              cout << "Bad performance: increase allocation" << endl;
                            }

                          }
                       }




                  }




              }






              // KEEP COUNTING...
              if(counter_waveform < EXERCISE_DURATION){
                  counter_waveform++;
                  campioni++;
              }





              // Reset counters, some variables and update allocation factor
              if (counter_waveform==EXERCISE_DURATION) {

                  sum_pos_err=0;
      //            sum_vel=0;
                  delta = 0.0;
                  mean = 0.0;
                  M2 = 0.0;
      //            delta_vel=0;
      //            mean_vel=0;
      //            M2_vel=0;
                  counter_waveform=0;
                  campioni=0;
                  T_limit=1.0;
                  counter_iteration++;

              }


              // Set shared memory
              c.data->rehamove_mode=rehamove_mode;
              c.data->rehamove_mean_position_error=mean_position_error;
              c.data->rehamove_mean_velocity=mean_velocity;
              c.data->rehamove_std_mean_position_error=std_position_error;
              c.data->rehamove_iteration=num_iter;
              c.data->rehamove_charge=q;
              c.data->rehamove_charge_adj=q_adjusted;
              c.data->rehamove_mean_mean_position_error=avg_mean_position_error;
              c.data->rehamove_mean_std_mean_position_error=avg_std_position_error;


              // Save parameters for feedforward motor torque
              c.data->arm_weight_compensation_config.human_height_m = H;
              c.data->arm_weight_compensation_config.human_weight_kg = W;
              c.data->arm_weight_compensation_config.weight_assistance = coeff_weight_assistance;
              c.data->impedance_control_command->impedance_control_feedforward_allocation_factor=coeff_allocation;
              c.data->impedance_control_command->impedance_control_k_gain_mNm_per_rad = stiffness_nm_rad;
              c.data->impedance_control_command->impedance_control_d_gain_mNm_per_rad_per_sec = damping_nms_rad;

      break;
    }

    case 56:
    {
      if(change_mode){
          change_mode=false;
          cout << "Flex+Ext mode" << endl;

          stimulation_initialization();

          // Read calibration file for FES parameters
          read_calibration_file(calib);

//            coeff_weight_assistance = E;

          // Insert values manually for the moment
            threshold_pos_error=-2.0;
            threshold_std_pos_error=1.0;
            coeff_weight_assistance = 0.25;

            coeff_allocation=0.0;


//            close_calibration_file();

          campioni=0;
          num_iter=0;
          counter_t = 0;
          counter_supp = 0;
          q_adjusted=0.2;

          // Stiffness and damping are set a priori
          stiffness_nm_rad = 5.0;
          damping_nms_rad = 1.0;

          // Fatigue parameter
//            K_fatigue = 30.0;

          // Initialize waveform counter
          counter_waveform = 0;
          counter_iteration = 0;
//            change_alloc = 0;
//            counter_q_adj=0;

          // Reset errors & velocity single repetition
          sum_pos_err = 0;
//            sum_vel = 0;
          mean_position_error=0;
//            mean_velocity = 0;
          std_position_error = 0;
//            std_velocity = 0;

          // Reset errors across repetitions
          sum_mean_position_error = 0;
          avg_mean_position_error = 0;
          sum_std_mean_position_error = 0;
          avg_std_position_error = 0;

          avg_delta = 0.0;
          avg_mean = 0.0;
          avg_M2 = 0.0;

          // Velocity across blocks
//            sum_block_vel = 0;
//            mean_block_vel = 0;
//            std_block_vel = 0;

//            block_delta =0.0;
//            block_mean = 0.0;
//            block_M2 = 0.0;

          // (Re)set parameters for charge waveform
          charge_old= 0.0;
          charge_dot = 0.0;
          charge_dot_old= 0.0;
          T_limit = 0.65;

          count_good = 0;
          count_bad = 0;


          //FOR TRICEPS
          q_adjusted_triceps = 0.2;
          sum_pos_err_t = 0;
//            sum_vel = 0;
          mean_position_error_t=0;
//            mean_velocity = 0;
          std_position_error_t = 0;
//            std_velocity = 0;

          // Reset errors across repetitions
          sum_mean_position_error_t = 0;
          avg_mean_position_error_t = 0;
          sum_std_mean_position_error_t = 0;
          avg_std_position_error_t = 0;

          avg_delta_t = 0.0;
          avg_mean_t = 0.0;
          avg_M2_t = 0.0;

          count_good_t = 0.0;

      }

      // Write shared memory
      c.data->control_mode_command = robot_control_mode_t::impedance_control;
      c.data->impedance_control_command->impedance_control_k_gain_mNm_per_rad         = stiffness_nm_rad;
      c.data->impedance_control_command->impedance_control_d_gain_mNm_per_rad_per_sec = damping_nms_rad;
//        c.data->rehamove_mean_mean_position_error = avg_mean_position_error;
      c.data->rehamove_mean_std_mean_position_error = threshold_std_pos_error;

      c.data->impedance_control_command->impedance_control_feedforward_allocation_factor=coeff_allocation;
      c.data->arm_weight_compensation_config.weight_assistance = coeff_weight_assistance;

      c.data->rehamove_mean_position_error=mean_position_error;
      c.data->rehamove_std_mean_position_error=std_position_error;
//        c.data->rehamove_mean_velocity = mean_velocity;
      c.data->rehamove_iteration = num_iter;

      c.data->rehamove_charge_adj = q_adjusted;
      c.data->rehamove_charge = q;

      //q = charge_waveform_generation_new(counter_iteration,counter_waveform)*q_adjusted; //se forma d'onda secondo traiettoria
      q = charge_waveform_generation(counter_iteration,counter_waveform)*q_adjusted; //se forma d'onda secondo torque con inizio non nullo
      //q = ((charge_waveform_generation(counter_iteration,counter_waveform)-q0)*2)*q_adjusted;  //se forma d'onda secondo torque con inizio nullo
      q = std::max(0.0,std::min(1.0,q));


      q_t = charge_waveform_generation_tri(counter_iteration,counter_waveform)*q_adjusted_triceps;
      q_t = std::max(0.0,std::min(1.0,q_t));

      if(loop_count%10==0){
                  stimulation_torque_triceps(calib);
              }

              // Compute trajectory error with/without sign
              if(counter_waveform<EXERCISE_DURATION/2){

                  // Compute position error
                  pos_err=des_pos-act_pos;
                  // cumulate position error
                  sum_pos_err+=pos_err;

                  // Compute Standard Deviation with streaming data
                  delta = abs(pos_err) - mean;
                  mean = mean + delta/static_cast<double>(counter_waveform+1);
                  M2 = M2+delta*(abs(pos_err)-mean);
             }

              // COMPUTE MEAN POS ERR AND STD WHEN FLEXION IS OVER
              if(counter_waveform==EXERCISE_DURATION/2){

                  // Count all iterations
                  num_iter++;

                  // Count iterations up to ten (OR FIVE???)
                  counter_rep++;

                  // Compute mean pos err and std of a single iteration
                  mean_position_error=sum_pos_err/counter_waveform;
                  std_position_error = sqrt(M2/(EXERCISE_DURATION/2-1));

                  // Compute averaged position error across rep
                  sum_std_mean_position_error+=std_position_error;
                  avg_std_position_error = sum_std_mean_position_error/num_iter;

                  // Compute averaged std across rep (standard deviation delle medie)
                  avg_delta = abs(mean_position_error) - avg_mean;
                  avg_mean = avg_mean + avg_delta/static_cast<double>(num_iter);
                  avg_M2 = avg_M2+avg_delta*(abs(mean_position_error) - avg_mean);
                  avg_std_position_error = sqrt(avg_M2/(num_iter/2));


                  // Reset parameters for charge waveform
                  charge_old= 0.0;
                  charge_dot = 0.0;
                  charge_dot_old= 0.0;

        }
              if(counter_waveform>=EXERCISE_DURATION/2){

                  // Compute position error
                  pos_err=des_pos-act_pos;
                  // cumulate position error
                  sum_pos_err_t+=pos_err;

                  // Compute Standard Deviation with streaming data
                  delta_t = abs(pos_err) - mean_t;
                  mean_t = mean_t + delta_t/static_cast<double>((counter_waveform-EXERCISE_DURATION/2)+1);
                  M2_t = M2_t+delta_t*(abs(pos_err)-mean_t);
             }

              if(counter_waveform==EXERCISE_DURATION-1){

                  // Compute mean pos err and std of a single iteration
                  mean_position_error_t=sum_pos_err_t/(counter_waveform-EXERCISE_DURATION/2+1);
                  std_position_error_t = sqrt(M2_t/(EXERCISE_DURATION/2-1));

                  // Compute averaged position error across rep
                  sum_std_mean_position_error_t+=std_position_error_t;
                  avg_std_position_error_t = sum_std_mean_position_error_t/num_iter;

                  // Compute averaged std across rep (standard deviation delle medie)
                  avg_delta_t = abs(mean_position_error_t) - avg_mean_t;
                  avg_mean_t = avg_mean_t + avg_delta_t/static_cast<double>(num_iter);
                  avg_M2_t = avg_M2_t+avg_delta_t*(abs(mean_position_error_t) - avg_mean_t);
                  avg_std_position_error_t = sqrt(avg_M2_t/(num_iter/2));
              }

              if(num_iter > 1){

                  // Iterative learning

                  if (counter_waveform>=4500 && counter_waveform<4501){

                      //            double threshold_pos_error = -3.7;
                      //            double threshold_std_pos_error = 2.5;
                      double K_adjusted = 0.02;
                      double K_vol = 0.05;
                      cout << "**************************" << endl;
                      cout << "Iterative Learning Control Biceps" << endl;
                      cout << "Mean error: " << mean_position_error*rad_to_deg << endl;
                      cout << "Std  error: " << std_position_error*rad_to_deg << endl;
                      cout << "Thr error: " << threshold_pos_error<< endl;
                      cout << "Thr std: " << threshold_std_pos_error << endl;

                      if((mean_position_error-std_position_error)*rad_to_deg < (threshold_pos_error-threshold_std_pos_error)){


                          cout << "Slow movement: increase charge" << endl;
                          q_adjusted += -K_adjusted*((mean_position_error-std_position_error)*rad_to_deg-(threshold_pos_error-threshold_std_pos_error));
                          count_good = 0;

                      }
                      else if((mean_position_error+std_position_error)*rad_to_deg > (-threshold_pos_error+threshold_std_pos_error)){
                          cout << "Fast movement: decrease charge" << endl;

                          q_adjusted += -K_adjusted*(((mean_position_error+std_position_error)*rad_to_deg-(-threshold_pos_error+threshold_std_pos_error)));
                          //count_good = 0;

                      }

                      else {
                          cout << "Ok movement: Ok charge" << endl;
                          count_good += 1;
                          if (count_good >= 5)
                          {
                            q_adjusted = q_adjusted - K_vol;
                            cout << "Voluntary???" << endl;
                          }
                      }



                      // Copyright SDG please.
      //                q_adjusted = std::max(0.0,std::min(1.0,q_adjusted));
                      // OPTION 2:
                       q_adjusted = std::max(0.0,std::min(1.5,q_adjusted));

                       cout << "Charge adjusted: " << q_adjusted << endl;
                       /*
                       if (q_adjusted == 1.5)
                       {
                          count_bad += 1;
                          if (count_bad >= 5 && (mean_position_error-std_position_error)*rad_to_deg < (threshold_pos_error-threshold_std_pos_error))
                          {
                            if (coeff_allocation == 1.0)
                            {
                              cout << "Subject is not able to perform movement without full motor action" << endl;
                            }
                            else {
                              coeff_allocation += 0.25;
                              count_bad = 0;
                              q_adjusted = 0.3;
                              cout << "Bad performance: increase allocation" << endl;
                            }

                          }
                       }
                      */



                  }




              }
              if(num_iter > 1){

                  // Iterative learning

                  if (counter_waveform>=1500 && counter_waveform<1501){

                      //            double threshold_pos_error = -3.7;
                      //            double threshold_std_pos_error = 2.5;
                      double K_adjusted = 0.015;
                      double K_vol = 0.05;
                      cout << "**************************" << endl;
                      cout << "Iterative Learning Control Triceps" << endl;
                      cout << "Mean error triceps: " << mean_position_error_t*rad_to_deg << endl;
                      cout << "Std  error triceps: " << std_position_error_t*rad_to_deg << endl;
                      cout << "Thr error: " << threshold_pos_error<< endl;
                      cout << "Thr std: " << threshold_std_pos_error << endl;

                      if((mean_position_error_t+std_position_error_t)*rad_to_deg > (-threshold_pos_error-threshold_std_pos_error)){


                          cout << "Slow movement: increase charge" << endl;
                          q_adjusted_triceps += K_adjusted*((mean_position_error_t+std_position_error_t)*rad_to_deg-(-threshold_pos_error-threshold_std_pos_error));
                          count_good = 0;

                      }
                      else if((mean_position_error_t-std_position_error_t)*rad_to_deg < (threshold_pos_error-threshold_std_pos_error)){
                          cout << "Fast movement: decrease charge" << endl;

                          q_adjusted_triceps += K_adjusted*(((mean_position_error_t-std_position_error_t)*rad_to_deg-(threshold_pos_error-threshold_std_pos_error)));
                          //count_good = 0;

                      }
                      else {
                          cout << "Ok movement: Ok charge" << endl;
                          count_good_t += 1;
                          if (count_good_t >= 5)
                          {
                            q_adjusted_triceps = q_adjusted_triceps - K_vol;
                            cout << "Voluntary???" << endl;
                          }
                      }



                      // Copyright SDG please.
      //                q_adjusted = std::max(0.0,std::min(1.0,q_adjusted));
                      // OPTION 2:
                       q_adjusted_triceps = std::max(0.0,std::min(1.5,q_adjusted_triceps));

                       cout << "Charge adjusted triceps: " << q_adjusted_triceps << endl;
                       /*
                       if (q_adjusted == 1.5)
                       {
                          count_bad += 1;
                          if (count_bad >= 5 && (mean_position_error-std_position_error)*rad_to_deg < (threshold_pos_error-threshold_std_pos_error))
                          {
                            if (coeff_allocation == 1.0)
                            {
                              cout << "Subject is not able to perform movement without full motor action" << endl;
                            }
                            else {
                              coeff_allocation += 0.25;
                              count_bad = 0;
                              q_adjusted = 0.3;
                              cout << "Bad performance: increase allocation" << endl;
                            }

                          }
                       }
                      */



                  }




              }






              // KEEP COUNTING...
              if(counter_waveform < EXERCISE_DURATION){
                  counter_waveform++;
                  campioni++;
              }





              // Reset counters, some variables and update allocation factor
              if (counter_waveform==EXERCISE_DURATION) {

                  sum_pos_err=0;
      //            sum_vel=0;
                  delta = 0.0;
                  mean = 0.0;
                  M2 = 0.0;
      //            delta_vel=0;
      //            mean_vel=0;
      //            M2_vel=0;
                  counter_waveform=0;
                  campioni=0;
                  T_limit=1.0;
                  counter_iteration++;

                  sum_pos_err_t=0;
      //            sum_vel=0;
                  delta_t = 0.0;
                  mean_t = 0.0;
                  M2_t = 0.0;

              }


              // Set shared memory
              c.data->rehamove_mode=rehamove_mode;
              c.data->rehamove_mean_position_error=mean_position_error;
              c.data->rehamove_mean_velocity=mean_velocity;
              c.data->rehamove_std_mean_position_error=std_position_error;
              c.data->rehamove_iteration=num_iter;
              c.data->rehamove_charge=q;
              c.data->rehamove_charge_adj=q_adjusted;
              c.data->rehamove_mean_mean_position_error=avg_mean_position_error;
              c.data->rehamove_mean_std_mean_position_error=avg_std_position_error;


              // Save parameters for feedforward motor torque
              c.data->arm_weight_compensation_config.human_height_m = H;
              c.data->arm_weight_compensation_config.human_weight_kg = W;
              c.data->arm_weight_compensation_config.weight_assistance = coeff_weight_assistance;
              c.data->impedance_control_command->impedance_control_feedforward_allocation_factor=coeff_allocation;
              c.data->impedance_control_command->impedance_control_k_gain_mNm_per_rad = stiffness_nm_rad;
              c.data->impedance_control_command->impedance_control_d_gain_mNm_per_rad_per_sec = damping_nms_rad;

      break;
    }

    case 10:{
        // ILC ONLY MODALITY
        if(change_mode){
            change_mode=false;
            cout << "ILC ONLY MODALITY" << endl;

            stimulation_initialization_motimove();

            // Read calibration file for FES parameters
            read_calibration_file(calib);

//            coeff_weight_assistance = E;

            // Insert values manually for the moment
              threshold_pos_error=-2.0;
              threshold_std_pos_error=2.0;
              //coeff_weight_assistance = E;
              coeff_weight_assistance = 0.25;
              coeff_allocation=0.0;


//            close_calibration_file();

            campioni=0;
            num_iter=0;
            counter_t = 0;
            counter_supp = 0;
            q_adjusted=0.2;

            // Stiffness and damping are set a priori
            stiffness_nm_rad = 5.0;
            damping_nms_rad = 1.0;

            // Fatigue parameter
//            K_fatigue = 30.0;

            // Initialize waveform counter
            counter_waveform = 0;
            counter_iteration = 0;
//            change_alloc = 0;
//            counter_q_adj=0;

            // Reset errors & velocity single repetition
            sum_pos_err = 0;
//            sum_vel = 0;
            mean_position_error=0;
//            mean_velocity = 0;
            std_position_error = 0;
//            std_velocity = 0;

            // Reset errors across repetitions
            sum_mean_position_error = 0;
            avg_mean_position_error = 0;
            sum_std_mean_position_error = 0;
            avg_std_position_error = 0;

            avg_delta = 0.0;
            avg_mean = 0.0;
            avg_M2 = 0.0;

            // Velocity across blocks
//            sum_block_vel = 0;
//            mean_block_vel = 0;
//            std_block_vel = 0;

//            block_delta =0.0;
//            block_mean = 0.0;
//            block_M2 = 0.0;

            // (Re)set parameters for charge waveform
            charge_old= 0.0;
            charge_dot = 0.0;
            charge_dot_old= 0.0;
            T_limit = 0.65;

        }

        // Write shared memory
        c.data->control_mode_command = robot_control_mode_t::impedance_control;
        c.data->impedance_control_command->impedance_control_k_gain_mNm_per_rad         = stiffness_nm_rad;
        c.data->impedance_control_command->impedance_control_d_gain_mNm_per_rad_per_sec = damping_nms_rad;
//        c.data->rehamove_mean_mean_position_error = avg_mean_position_error;
        c.data->rehamove_mean_std_mean_position_error = threshold_std_pos_error;

        c.data->impedance_control_command->impedance_control_feedforward_allocation_factor=coeff_allocation;
        c.data->arm_weight_compensation_config.weight_assistance = coeff_weight_assistance;

        c.data->rehamove_mean_position_error=mean_position_error;
        c.data->rehamove_std_mean_position_error=std_position_error;
//        c.data->rehamove_mean_velocity = mean_velocity;
        c.data->rehamove_iteration = num_iter;

        c.data->rehamove_charge_adj = q_adjusted;
        c.data->rehamove_charge = q;
//        c.data ->rehamove_counter_q_adj = counter_q_adj;


        // TODO: do not stimulate while keeping 100% motor allocation
//        if(coeff_allocation<1){
//            q = charge_waveform_generation(counter_iteration,counter_waveform)*q_adjusted;

            // OPTION 2

            q = (charge_waveform_generation(counter_iteration,counter_waveform)-q0/2)*q_adjusted*1.2;
            q = std::max(0.0,std::min(1.0,q));
//        }
//        else {
//            q_adjusted=0.2;
//            q = (charge_waveform_generation(counter_iteration,counter_waveform)-q0/2)*q_adjusted*1.2;
//            q = std::max(0.0,std::min(1.0,q));

//            q=0;
//        }

        if(loop_count%10==0){
            stimulation_torque_motimove(calib);
        }

        // Compute trajectory error with/without sign
        if(counter_waveform<EXERCISE_DURATION/2){

            // Compute position error
            pos_err=des_pos-act_pos;
            // cumulate position error
            sum_pos_err+=pos_err;

            // Compute Standard Deviation with streaming data
            delta = abs(pos_err) - mean;
            mean = mean + delta/static_cast<double>(counter_waveform+1);
            M2 = M2+delta*(abs(pos_err)-mean);
       }

        // COMPUTE MEAN POS ERR AND STD WHEN FLEXION IS OVER
        if(counter_waveform==EXERCISE_DURATION/2){

            // Count all iterations
            num_iter++;

            // Count iterations up to ten (OR FIVE???)
            counter_rep++;

            // Compute mean pos err and std of a single iteration
            mean_position_error=sum_pos_err/counter_waveform;
            std_position_error = sqrt(M2/(EXERCISE_DURATION/2-1));

            // Compute averaged position error across rep
            sum_std_mean_position_error+=std_position_error;
            avg_std_position_error = sum_std_mean_position_error/num_iter;

            // Compute averaged std across rep (standard deviation delle medie)
            avg_delta = abs(mean_position_error) - avg_mean;
            avg_mean = avg_mean + avg_delta/static_cast<double>(num_iter);
            avg_M2 = avg_M2+avg_delta*(abs(mean_position_error) - avg_mean);
            avg_std_position_error = sqrt(avg_M2/(num_iter/2));


            // Reset parameters for charge waveform
            charge_old= 0.0;
            charge_dot = 0.0;
            charge_dot_old= 0.0;

  }

        if(num_iter > 1){

            // Iterative learning

            if (counter_waveform>=4500 && counter_waveform<4501){

                //            double threshold_pos_error = -3.7;
                //            double threshold_std_pos_error = 2.5;
                double K_adjusted = 0.01;
                cout << "**************************" << endl;
                cout << "Iterative Learning Control" << endl;
                cout << "Mean error: " << mean_position_error*rad_to_deg << endl;
                cout << "Std  error: " << std_position_error*rad_to_deg << endl;
                cout << "Thr error: " << threshold_pos_error<< endl;
                cout << "Thr std: " << threshold_std_pos_error << endl;

                if(mean_position_error*rad_to_deg < (threshold_pos_error-threshold_std_pos_error)){


                    cout << "Slow movement: increase charge" << endl;
                    q_adjusted += -K_adjusted*((mean_position_error*rad_to_deg-threshold_pos_error));

                }
                else if(mean_position_error*rad_to_deg > (threshold_pos_error+threshold_std_pos_error)){
                    cout << "Fast movement: decrease charge" << endl;

                    q_adjusted += -K_adjusted*((mean_position_error*rad_to_deg-threshold_pos_error));

                }
                else {
                    cout << "Ok movement: Ok charge" << endl;
                }



                // Copyright SDG please.
//                q_adjusted = std::max(0.0,std::min(1.0,q_adjusted));
                // OPTION 2:
                 q_adjusted = std::max(0.0,std::min(1.5,q_adjusted));

                 cout << "Charge adjusted: " << q_adjusted << endl;




            }




        }






        // KEEP COUNTING...
        if(counter_waveform < EXERCISE_DURATION){
            counter_waveform++;
            campioni++;
        }





        // Reset counters, some variables and update allocation factor
        if (counter_waveform==EXERCISE_DURATION) {

            sum_pos_err=0;
//            sum_vel=0;
            delta = 0.0;
            mean = 0.0;
            M2 = 0.0;
//            delta_vel=0;
//            mean_vel=0;
//            M2_vel=0;
            counter_waveform=0;
            campioni=0;
            T_limit=1.0;
            counter_iteration++;

            /*
            if(num_iter%35==0){
                coeff_allocation+=0.25;
                q_adjusted=0.2;
            }
            */



        }


        // Set shared memory
        c.data->rehamove_mode=rehamove_mode;
        c.data->rehamove_mean_position_error=mean_position_error;
        c.data->rehamove_mean_velocity=mean_velocity;
        c.data->rehamove_std_mean_position_error=std_position_error;
        c.data->rehamove_iteration=num_iter;
        c.data->rehamove_charge=q;
        c.data->rehamove_charge_adj=q_adjusted;
        c.data->rehamove_mean_mean_position_error=avg_mean_position_error;
        c.data->rehamove_mean_std_mean_position_error=avg_std_position_error;


        // Save parameters for feedforward motor torque
        c.data->arm_weight_compensation_config.human_height_m = H;
        c.data->arm_weight_compensation_config.human_weight_kg = W;
        c.data->arm_weight_compensation_config.weight_assistance = coeff_weight_assistance;
        c.data->impedance_control_command->impedance_control_feedforward_allocation_factor=coeff_allocation;
        c.data->impedance_control_command->impedance_control_k_gain_mNm_per_rad = stiffness_nm_rad;
        c.data->impedance_control_command->impedance_control_d_gain_mNm_per_rad_per_sec = damping_nms_rad;

        break;


    }

    case 4:





    {
        if( change_mode){
            change_mode=false;
            cout << " WEIGHT ASSISTANCE CALIBRATION " << endl;
        }

        c.data->control_mode_command = robot_control_mode_t::impedance_ext_control;
        stiffness_nm_rad        = 0.0;
        damping_nms_rad         = 0.5;
        break;
    }

    // Calibration Motor Phase 2: 100% Motor allocation + compute mean and std
    case 5:
    {
        // Increase/Decrease charge amplitude + increase/decrease allocation modulation
        if(change_mode){
            change_mode=false;
            cout << "IMPEDANCE + CHARGE MODULATION + ALLOCATION MODULATION" << endl;

            read_calibration_file(calib);
            threshold_pos_error=-2.5;
            threshold_std_pos_error=2.5;

            campioni=0;
            num_iter=0;
            counter_t = 0;
            counter_supp = 0;
//            q_adjusted=0.2;

            delta = 0.0;
            mean = 0.0;
            M2 = 0.0;

            // Initialize waveform counter
            counter_waveform = 0;
            counter_iteration = 0;

            coeff_allocation = 1.0;
            coeff_weight_assistance = E;

            stiffness_nm_rad = 5.0;
            damping_nms_rad = 1.0;

            // Reset errors single repetition
            sum_pos_err = 0;
            mean_position_error = 0;
            std_position_error = 0;

            // Reset errors across repetitions
            sum_mean_position_error = 0;
            avg_mean_position_error = 0;
            sum_std_mean_position_error = 0;
            avg_std_position_error = 0;
            avg_M2 = 0.0;
            avg_delta = 0.0;
            avg_mean = 0.0;

        }

        // Write shared memory
        c.data->control_mode_command = robot_control_mode_t::impedance_control;
        c.data->impedance_control_command->impedance_control_k_gain_mNm_per_rad         = stiffness_nm_rad;
        c.data->impedance_control_command->impedance_control_d_gain_mNm_per_rad_per_sec = damping_nms_rad;
        c.data->rehamove_mean_position_error=mean_position_error;
        c.data->rehamove_std_mean_position_error=std_position_error;
        c.data->rehamove_iteration = num_iter;
        c.data->impedance_control_command->impedance_control_feedforward_allocation_factor=coeff_allocation;
        c.data->arm_weight_compensation_config.weight_assistance = coeff_weight_assistance;
        c.data->rehamove_mean_alloc_tor = mean_alloc_arm_tor;


        cout << "mean position error" << mean_position_error*rad_to_deg<< endl;


        if(counter_iteration > 0){
            // Compute trajectory error with/without sign
            if(counter_waveform<EXERCISE_DURATION/2){

                // Compute position error
                pos_err=des_pos-act_pos;
                // cumulate position error
                sum_pos_err+=pos_err;

                // Compute Standard Deviation with streaming data
                delta = abs(pos_err) - mean;
                mean = mean + delta/static_cast<double>(counter_waveform+1);
                M2 = M2+delta*(abs(pos_err)-mean);


                // Compute mean allocated arm torque
                sum_alloc_arm_tor+=c.data->impedance_control_terms->arm_dynamic_torque_mNm;



            }

            // After elbow flexion compute mean position error and mean std
            if(counter_waveform==EXERCISE_DURATION/2){

                //cout << "mean position error" << mean_position_error << endl;


                num_iter++;
                mean_position_error=sum_pos_err/counter_waveform;
                mean_alloc_arm_tor=sum_alloc_arm_tor/counter_waveform;
                std_position_error = sqrt(M2/(EXERCISE_DURATION/2-1));

                // Compute averaged position error across iterations
                sum_mean_position_error+=mean_position_error;
                avg_mean_position_error=sum_mean_position_error/num_iter;

                // Compute averaged Standard Deviation across iterations
                avg_delta = abs(mean_position_error) - avg_mean;
                avg_mean = avg_mean + avg_delta/static_cast<double>(num_iter);
                avg_M2 = avg_M2+avg_delta*(abs(mean_position_error) - avg_mean);
                avg_std_position_error = sqrt(avg_M2/(num_iter/2));

            }
        }

        if(counter_waveform < EXERCISE_DURATION){
            counter_waveform++;
            campioni++;
        }

        // After elbow flexion/extension reset counters
        if (counter_waveform==EXERCISE_DURATION) {

            if(num_iter>1 && num_iter%100==0){
                coeff_allocation-=0.25;
            }

            sum_pos_err=0;
            sum_alloc_arm_tor=0;
            delta = 0.0;
            mean = 0.0;
            M2 = 0.0;
            counter_waveform=0;
            campioni=0;
            T_limit=1.0;
            counter_iteration++;
//            if(num_iter > 0 && num_iter%10==0){
//                coeff_allocation+=0.5;

//            }
        }


            if(num_iter > 1){

                read_calibration_file(calib);

                E = coeff_weight_assistance; //<< "," //coeff_weight_assistance << ","
                F = avg_mean_position_error; // << F << "," //<< avg_mean_position_error<< ","
                G = avg_std_position_error; // << G << "," //<< avg_std_position_error << ","

                open_calibration_file();
                write_calibration_file();
                close_calibration_file();
            }




        break;
    }

    case 6:
    {

        if( change_mode){
            change_mode=false;
            cout << " PI FES control" << endl;
            stimulation_initialization();
            read_calibration_file(calib);

            // Initialize waveform counter
            counter_waveform = 0;
            campioni = 0;
            integral = 0;
            derivative = 0;

            q=0;


        }

        c.data->impedance_control_command->impedance_control_k_gain_mNm_per_rad = stiffness_nm_rad;
        c.data->impedance_control_command->impedance_control_d_gain_mNm_per_rad_per_sec = damping_nms_rad;
        c.data->arm_weight_compensation_config.weight_assistance = coeff_weight_assistance;


        if(loop_count%10==0){
            stimulation_torque(calib);
        }



        if(counter_waveform<EXERCISE_DURATION/2)
        {

            campioni ++;
            pid_des_pos=trajectory_generation(campioni);
            q=calcoloPID(pid_des_pos,act_pos,q_old);
            q_old=q;

//            q=q+(des_pos*rad_to_deg-act_pos*rad_to_deg)*0.003;
            // TODO: tune k and i parameters

            stiffness_nm_rad = 0.0;
            damping_nms_rad = 0.0;
            coeff_weight_assistance = 0.0;
            c.data->control_mode_command = robot_control_mode_t::gravity_control;
            c.data->rehamove_desired_position_pid = pid_des_pos;
//            c.data->control_mode_command = robot_control_mode_t::transparent_control;
        }
        if(counter_waveform == EXERCISE_DURATION/2) {

            q=0;
            q_old=0;
            num_iter++;
            campioni = 0;
            pid_des_pos = des_pos;
           // Try with/without integral
//            integral = 0;
            derivative = 0;

        }


        if(counter_waveform>EXERCISE_DURATION/2){
            stiffness_nm_rad = 5.0;
            damping_nms_rad = 1.5;
            campioni = 0;
            c.data->control_mode_command = robot_control_mode_t::go_position_control;
            q=0;
            q_old=0;
            if (act_pos*rad_to_deg>148.5) {
                counter_waveform=0;
            }

            pid_des_pos = des_pos;
        }


        counter_waveform++;

        cout << "counter waveform " << counter_waveform << endl;
        cout << "campioni " << campioni << endl;


        break;
    }

    case 61:
    {

        // Increase/Decrease charge amplitude + increase/decrease allocation modulation
        if(change_mode){
            change_mode=false;
            cout << "100% FES Mode - no impedance-based correction" << endl;

            stimulation_initialization();

            // Read calibration file for FES parameters
            read_calibration_file(calib);

            coeff_weight_assistance = E;
            coeff_allocation = 0;

            // Insert threshold values manually for the moment
            threshold_pos_error=-2.5;
            threshold_std_pos_error=2.5;

            // Initialize counters
            campioni=0;
            num_iter=0;
            counter_t = 0;
            counter_supp = 0;
            q_adjusted=0.0;

            // Stiffness and damping are set a priori
            stiffness_nm_rad = 0.0;
            damping_nms_rad = 1.0;

            // Initialize waveform counter
            counter_waveform = 0;
            counter_iteration = 0;

            // Reset errors single repetition
            sum_pos_err = 0;
            mean_position_error=0;
            std_position_error = 0;

            // Reset errors across repetitions
            sum_mean_position_error = 0;
            avg_mean_position_error = 0;
            sum_std_mean_position_error = 0;
            avg_std_position_error = 0;

            avg_delta = 0.0;
            avg_mean = 0.0;
            avg_M2 = 0.0;

            // (Re)set parameters for charge waveform
            charge_old= 0.0;
            charge_dot = 0.0;
            charge_dot_old= 0.0;
            T_limit = 0.65;

        }

        // do not stimulate while keeping 100% motor allocation
        coeff_allocation=0;

        q = (charge_waveform_generation(counter_iteration,counter_waveform)-q0/2)*q_adjusted*1.2;
        q = std::max(0.0,std::min(1.0,q));


        // Command stimulator
        if(loop_count%10==0){
            stimulation_torque(calib);
        }

        // Flexion phase
        if(counter_waveform<EXERCISE_DURATION/2){

            // Stiffness and damping are set a priori
            stiffness_nm_rad = 0.0;
            damping_nms_rad = 1.5;

            // Compute position error
            pos_err=des_pos-act_pos;
            // cumulate position error
            sum_pos_err+=pos_err;

            // Compute Standard Deviation with streaming data
            delta = abs(pos_err) - mean;
            mean = mean + delta/static_cast<double>(counter_waveform+1);
            M2 = M2+delta*(abs(pos_err)-mean);
        }
        // Extension phase
        else {


            q_old=0;

            if( counter_waveform < EXERCISE_DURATION/4.0*3.0)
            {
                stiffness_nm_rad += 2.0/1000;
            }
            else {
                stiffness_nm_rad -= 2.0/1000;
            }
            // Stiffness and damping are set a priori
//            stiffness_nm_rad = 0.0;
            damping_nms_rad = 1.5;

//            if (c.data->control_mode_status==robot_control_mode_t::go_position_done) {
//                sum_pos_err=0;
//                delta = 0.0;
//                mean = 0.0;
//                M2 = 0.0;
//                campioni=0;
//                T_limit=1.0;
//                counter_iteration++;
//                stiffness_nm_rad = 0.0;
//                damping_nms_rad = 0.5;
//                c.data->control_mode_command = robot_control_mode_t::gravity_control;
//            }
//            if (c.data->control_mode_status==robot_control_mode_t::gravity_control){
//                if(counter_waveform > EXERCISE_DURATION){
//                    counter_waveform=0;
//                    cout << "Restart" << endl;
//                }
//            }
        }

        // Write shared memory
        c.data->control_mode_command = robot_control_mode_t::impedance_control;
        c.data->impedance_control_command->impedance_control_k_gain_mNm_per_rad         = stiffness_nm_rad;
        c.data->impedance_control_command->impedance_control_d_gain_mNm_per_rad_per_sec = damping_nms_rad;
        c.data->rehamove_mean_mean_position_error                                       = avg_mean_position_error;
        c.data->rehamove_mean_std_mean_position_error                                   = avg_std_position_error;

        // After elbow flexion compute mean position error and mean std
        if(counter_waveform==EXERCISE_DURATION/2){

//            // Start elbow extension
//            stiffness_nm_rad = 1.0;
//            damping_nms_rad = 1.5;
//            c.data->impedance_control_command->impedance_control_setpoint_rad = EXERCISE_START;
//            c.data->control_mode_command = robot_control_mode_t::go_position_control;

            num_iter++;
            mean_position_error=sum_pos_err/counter_waveform;
            std_position_error = sqrt(M2/(EXERCISE_DURATION/2-1));

            // Compute averaged position error across rep
            sum_std_mean_position_error+=std_position_error;
            avg_std_position_error = sum_std_mean_position_error/num_iter;

            // Compute averaged std across rep (standard deviation delle medie)
            avg_delta = abs(mean_position_error) - avg_mean;
            avg_mean = avg_mean + avg_delta/static_cast<double>(num_iter);
            avg_M2 = avg_M2+avg_delta*(abs(mean_position_error) - avg_mean);
            avg_std_position_error = sqrt(avg_M2/(num_iter/2));

            // Reset parameters for charge waveform
            charge_old= 0.0;
            charge_dot = 0.0;
            charge_dot_old= 0.0;

            c.data->rehamove_final_reaching_error = pos_err;
        }

        if(num_iter > 1){

            // Iterative learning

            if (counter_waveform>=4500 && counter_waveform<4501){

                //            double threshold_pos_error = -3.7;
                //            double threshold_std_pos_error = 2.5;
                double K_adjusted = 0.006;
                cout << "**************************" << endl;
                cout << "Iterative Learning Control" << endl;
                cout << "Mean error: " << mean_position_error*rad_to_deg << endl;
                cout << "Std  error: " << std_position_error*rad_to_deg << endl;
                cout << "Thr error: " << threshold_pos_error<< endl;
                cout << "Thr std: " << threshold_std_pos_error << endl;

                if(pos_err*rad_to_deg < (threshold_pos_error-threshold_std_pos_error)){


                    cout << "Slow movement: increase charge" << endl;
                    q_adjusted += -K_adjusted*((pos_err*rad_to_deg-threshold_pos_error));

                }
                else if(pos_err*rad_to_deg > (threshold_pos_error+threshold_std_pos_error)){
                    cout << "Fast movement: decrease charge" << endl;

                    q_adjusted += -K_adjusted*((pos_err*rad_to_deg-threshold_pos_error));

                }
                else {
                    cout << "Ok movement: Ok charge" << endl;
                }

                // Copyright SDG please.
                q_adjusted = std::max(0.0,std::min(1.5,q_adjusted));
                cout << "Charge adjusted: " << q_adjusted << endl;
            }
        }

        if(counter_waveform < EXERCISE_DURATION){
            counter_waveform++;
            campioni++;
        }

        // After elbow flexion/extension reset counters
        if (counter_waveform==EXERCISE_DURATION) {
            sum_pos_err=0;
            delta = 0.0;
            mean = 0.0;
            M2 = 0.0;
            counter_waveform=0;
            campioni=0;
            T_limit=1.0;
            counter_iteration++;
        }
        break;
    }

    case 7 :  // TO DO: add MOTIMOVE functions if needed
    {
        if(change_mode){
            change_mode=false;
            cout << " FATIGUE mode" << endl;

            stiffness_nm_rad = 60.0;
            damping_nms_rad = 6.0;


            stimulation_initialization();
            read_calibration_file(calib);
            counter_waveform=0;
        }


        // STANDARD CALIBRATION IN FREEZE
        c.data->control_mode_command = robot_control_mode_t::freeze_control;
        // Giunto molto rigido 60.0/40.0 - 6.0/4.0
        c.data->impedance_control_command->impedance_control_k_gain_mNm_per_rad = stiffness_nm_rad;
        c.data->impedance_control_command->impedance_control_d_gain_mNm_per_rad_per_sec = damping_nms_rad;


        #define FATIGUE_TEST_DURATION 3000


        double current_stim, pw_stim;

        if(counter_waveform < 500){
            current_stim = static_cast<double>(counter_waveform)/500.0*0.8*static_cast<double>(C);
            pw_stim = static_cast<double>(counter_waveform)/500.0*0.8*static_cast<double>(D);

        }
        else if (counter_waveform < FATIGUE_TEST_DURATION) {
            current_stim = 0.8*C;
            pw_stim     = 0.8*D;
        }
        else{
            current_stim = 0;
            pw_stim = 0;
        }

        if(counter_waveform%100==0){
            ml_update.channel_config[Smpt_Channel_Red].number_of_points = Number_of_points;
            ml_update.channel_config[Smpt_Channel_Red].ramp = Ramp;
            ml_update.channel_config[Smpt_Channel_Red].period = Period; //mettere periodo 40 se voglio 25Hz

            ml_update.channel_config[Smpt_Channel_Red].points[0].current = current_stim;
            ml_update.channel_config[Smpt_Channel_Red].points[1].current = 0;
            ml_update.channel_config[Smpt_Channel_Red].points[2].current = -current_stim;

            ml_update.channel_config[Smpt_Channel_Red].points[0].time = pw_stim;
            ml_update.channel_config[Smpt_Channel_Red].points[1].time = 0;
            ml_update.channel_config[Smpt_Channel_Red].points[2].time = pw_stim;

            smpt_send_ml_update(&device, &ml_update);

            // Send packet
            Smpt_ml_get_current_data ml_get_current_data = {0};
            ml_get_current_data.packet_number = smpt_packet_number_generator_next(&device);
            ml_get_current_data.data_selection[Smpt_Ml_Data_Stimulation] = true;
            smpt_send_ml_get_current_data(&device, &ml_get_current_data);
        }

        counter_waveform++;

        if(counter_waveform%(FATIGUE_TEST_DURATION*2)==0){
            counter_waveform=0;
        }

        if(counter_waveform==1500){
            cout << "Torque: " << c.data->joint_values->loadcell_reading_mNm << endl;
        }

        stim_current=current_stim;
        stim_PW=pw_stim;

        break;
    }

    case 701:
    {
        // ** OLD VERSION OF COOPERATIVE CONTROL **
        // Start the impedance control with parameters set above

        if(change_mode){
            change_mode=false;
            cout << " IMPEDANCE mode + FES - ALLOCATION" << endl;
            stimulation_initialization();

            // Read CONFIGURATION file (Imin, PWmin, Imax, PWmax) obtained with CALIBRATION
            read_calibration_file(calib);
            cout << "il vettore di calibrazione è:" << calib[0] << "," << calib[1] << ","<< calib[2] << "," << calib[3] <<  endl;
            close_calibration_file();
            campioni=0;

            delta = 0.0;
            mean = 0.0;
            M2 = 0.0;
            avg_std_position_error = 0.0;
            sum_std_mean_position_error = 0.0;
            sum_mean_position_error = 0.0;
            avg_mean_position_error = 0.0;


        }

        c.data->control_mode_command = robot_control_mode_t::impedance_control;
        c.data->impedance_control_command->impedance_control_k_gain_mNm_per_rad         = 30.0;
        c.data->impedance_control_command->impedance_control_d_gain_mNm_per_rad_per_sec = 2.0;

        // TODO: UPDATE ALLOCATION FROM USER INPUT
        // c.data->impedance_control_command->impedancCSV_calibration_filee_control_feedforward_allocation_factor=motor_allocation;
        coeff_allocation = 1.0;


        if(loop_count%10){
            stimulation_torque(calib);
        }



        campioni++; stiffness_nm_rad = 10.0;
        damping_nms_rad = 1.0;

        // Compute trajectory error with sign
        pos_err=des_pos-act_pos;
        sum_pos_err+=pos_err;

        // Compute Standard Deviation with streaming data
        delta = pos_err - mean;
        mean = mean + delta/campioni;
        M2 = M2+delta*(pos_err-mean);

        // After one elbow flexion compute mean position error
        if(campioni==4000){
            num_iter++;
            mean_position_error=sum_pos_err/4000;
            std_position_error = sqrt(M2/(4000-1));
            sum_std_mean_position_error+=std_position_error;
            avg_std_position_error = sum_std_mean_position_error/num_iter;
            sum_mean_position_error += mean_position_error;
            avg_mean_position_error = sum_mean_position_error/num_iter;

        }


        // After elbow flexion/extension reset counters
        if (campioni==8000) {
            campioni=0;
            sum_pos_err= 0.0;
            mean_position_error = 0.0;
            std_position_error = 0.0;
            delta = 0.0;
            mean = 0.0;
            M2 = 0.0;

            // TODO: save the mean of the std in the 100% motor case in a variable

            // use it to modify charge and/or allocation in mode 9
        }

        iter_seno++;
        if(campioni==1000) {iter_seno=0;}
        if(campioni==1001) {q=0;}
        if(campioni==4000) iter_seno=0;
        break;
    }

    case 8:
    {
        // POSITION mode
        if( change_mode){
            change_mode=false;
            cout << " Go to position mode" << endl;
            q = 0.2;
        }
        c.data->control_mode_command = robot_control_mode_t::go_position_control;

        break;
    }

    case 9:
    {

        // Increase/Decrease charge amplitude + increase/decrease allocation modulation
        if(change_mode){
            change_mode=false;
            cout << "IMPEDANCE + CHARGE MODULATION + ALLOCATION MODULATION" << endl;

            stimulation_initialization();

            // Read calibration file for FES parameters
            read_calibration_file(calib);

//            coeff_weight_assistance = E;

            // Insert values manually for the moment
              threshold_pos_error=0.0;
              threshold_std_pos_error=2.0;
              coeff_weight_assistance = E;

              coeff_allocation=0.0;


//            close_calibration_file();

            campioni=0;
            num_iter=0;
            counter_t = 0;
            counter_supp = 0;
            q_adjusted=0.2;

            // Stiffness and damping are set a priori
            stiffness_nm_rad = 5.0;
            damping_nms_rad = 1.0;

            // Fatigue parameter
            K_fatigue = 30.0;

            // Initialize waveform counter
            counter_waveform = 0;
            counter_iteration = 0;
            change_alloc = 0;
            counter_q_adj=0;

            // Reset errors & velocity single repetition
            sum_pos_err = 0;
            sum_vel = 0;
            mean_position_error=0;
            mean_velocity = 0;
            std_position_error = 0;
            std_velocity = 0;

            // Reset errors across repetitions
            sum_mean_position_error = 0;
            avg_mean_position_error = 0;
            sum_std_mean_position_error = 0;
            avg_std_position_error = 0;

            avg_delta = 0.0;
            avg_mean = 0.0;
            avg_M2 = 0.0;

            // Velocity across blocks
            sum_block_vel = 0;
            mean_block_vel = 0;
            std_block_vel = 0;

            block_delta =0.0;
            block_mean = 0.0;
            block_M2 = 0.0;

            // (Re)set parameters for charge waveform
            charge_old= 0.0;
            charge_dot = 0.0;
            charge_dot_old= 0.0;
            T_limit = 0.65;

        }

        // Write shared memory
        c.data->control_mode_command = robot_control_mode_t::impedance_control;
        c.data->impedance_control_command->impedance_control_k_gain_mNm_per_rad         = stiffness_nm_rad;
        c.data->impedance_control_command->impedance_control_d_gain_mNm_per_rad_per_sec = damping_nms_rad;
//        c.data->rehamove_mean_mean_position_error = avg_mean_position_error;
        c.data->rehamove_mean_std_mean_position_error = threshold_std_pos_error;

        c.data->impedance_control_command->impedance_control_feedforward_allocation_factor=coeff_allocation;
        c.data->arm_weight_compensation_config.weight_assistance = coeff_weight_assistance;

        c.data->rehamove_mean_position_error=mean_position_error;
        c.data->rehamove_std_mean_position_error=std_position_error;
        c.data->rehamove_mean_velocity = mean_velocity;
        c.data->rehamove_iteration = num_iter;

        c.data->rehamove_charge_adj = q_adjusted;
        c.data->rehamove_charge = q;
        c.data ->rehamove_counter_q_adj = counter_q_adj;


        // TODO: do not stimulate while keeping 100% motor allocation
//        if(coeff_allocation<1){
//            q = charge_waveform_generation(counter_iteration,counter_waveform)*q_adjusted;

            // OPTION 2

            q = (charge_waveform_generation(counter_iteration,counter_waveform)-q0/2)*q_adjusted*1.2;
            q = std::max(0.0,std::min(1.0,q));
//        }
//        else {
//            q_adjusted=0.2;
//            q = (charge_waveform_generation(counter_iteration,counter_waveform)-q0/2)*q_adjusted*1.2;
//            q = std::max(0.0,std::min(1.0,q));

//            q=0;
//        }

        if(loop_count%10==0){
            stimulation_torque(calib);
        }

        // Compute trajectory error with/without sign
        if(counter_waveform<EXERCISE_DURATION/2){

            // Compute position error
            pos_err=des_pos-act_pos;
            // cumulate position error
            sum_pos_err+=pos_err;

            // Compute Standard Deviation with streaming data
            delta = abs(pos_err) - mean;
            mean = mean + delta/static_cast<double>(counter_waveform+1);
            M2 = M2+delta*(abs(pos_err)-mean);


            // Cumulate velocity
            sum_vel+=act_velocity;

            // Compute STD of velocities with streaming data
            delta_vel = act_velocity - mean_vel;
            mean_vel = mean_vel + delta_vel/static_cast<double>(counter_waveform+1);
            M2_vel = M2_vel + delta_vel*(act_velocity-mean_vel);

        }

        // After elbow flexion compute mean position error and mean std
        if(counter_waveform==EXERCISE_DURATION/2){

            num_iter++;
            if(change_alloc == 0){
                counter_rep++;
            } else {
                counter_alloc ++;
            }

            mean_position_error=sum_pos_err/counter_waveform;
            std_position_error = sqrt(M2/(EXERCISE_DURATION/2-1));

            // Compute averaged position error across rep
            sum_std_mean_position_error+=std_position_error;
            avg_std_position_error = sum_std_mean_position_error/num_iter;

            // Compute averaged std across rep (standard deviation delle medie)
            avg_delta = abs(mean_position_error) - avg_mean;
            avg_mean = avg_mean + avg_delta/static_cast<double>(num_iter);
            avg_M2 = avg_M2+avg_delta*(abs(mean_position_error) - avg_mean);
            avg_std_position_error = sqrt(avg_M2/(num_iter/2));

//             Compute mean flexion velocity
            mean_velocity = sum_vel / counter_waveform;
//            std_velocity = sqrt(M2_vel/(EXERCISE_DURATION/2-1)); // non so se serva...

//            // Compute average velocity and std across a block of ten (FIVE?????) iterations
//            // mean
            sum_block_vel+=mean_velocity;
            if(change_alloc == 0){
                mean_block_vel=sum_block_vel/counter_rep;
                // std
                delta_block=abs(mean_velocity)-mean_block;
                mean_block=mean_block+delta_block/static_cast<double>(counter_rep);
                M2_block=M2_block+delta_block*(abs(mean_velocity)-mean_block);
                std_block_vel=sqrt(M2_block/(counter_rep/2));

            } else {
                mean_block_vel=sum_block_vel/counter_alloc;
                // std
                delta_block=abs(mean_velocity)-mean_block;
                mean_block=mean_block+delta_block/static_cast<double>(counter_alloc);
                M2_block=M2_block+delta_block*(abs(mean_velocity)-mean_block);
                std_block_vel=sqrt(M2_block/(counter_alloc/2));

            }




            // Reset parameters for charge waveform
            charge_old= 0.0;
            charge_dot = 0.0;
            charge_dot_old= 0.0;


            // Compare mean velocity across block with current boundaries
            if (counter_alloc == 10){
                m = mean_block_vel;
                up = mean_block_vel+std_block_vel;
                down = mean_block_vel - std_block_vel;
                change_alloc = 0;
                counter_alloc=0;
//                num_block++;
            }

           if(counter_rep == 10){
               // if it's lower than the lower bound then raise allocation
               num_block++;

               if(num_block > 2){
                    // change allocation factor if one of these 2 conditions is satisfied
                   if(mean_block_vel > up || counter_q_adj > 40){


                           delta_allocation=K_fatigue*(mean_block_vel-up); // TODO: TUNE K_FATIGUE !!
                           coeff_allocation+=delta_allocation;
                           coeff_allocation = std::max(0.0,std::min(1.0,coeff_allocation));

                           change_alloc=1;

                           counter_no_change=0;

                           if(counter_q_adj>40){
                               counter_q_adj=0;
                           }

//                       num_block++;

                   }
                       else {

                           counter_no_change++;
//                           num_block++;

                       }



                } else if (num_block == 2){

                    m=mean_block_vel;
                    up=mean_block_vel+std_block_vel;
                    down=mean_block_vel-std_block_vel;
//                    num_block ++ ;

               }



               // if it keeps on staying inside the boundaries or it's higher than upper bound, count the blocks until
               // it gets lower than lower bound again
               if (counter_no_change > 4){
                  coeff_allocation -= 0.15; // TODO: TUNING/DECIDE WHAT TO DO !!
                  coeff_allocation = std::max(0.0,std::min(1.0,q_adjusted));

                  change_alloc = 1;
               }



            }

        }

        if(num_iter > 1){

            // Iterative learning

            if (counter_waveform>=4500 && counter_waveform<4501){

                //            double threshold_pos_error = -3.7;
                //            double threshold_std_pos_error = 2.5;
                double K_adjusted = 0.01;
                cout << "**************************" << endl;
                cout << "Iterative Learning Control" << endl;
                cout << "Mean error: " << mean_position_error*rad_to_deg << endl;
                cout << "Std  error: " << std_position_error*rad_to_deg << endl;
                cout << "Thr error: " << threshold_pos_error<< endl;
                cout << "Thr std: " << threshold_std_pos_error << endl;

                if(mean_position_error*rad_to_deg < (threshold_pos_error-threshold_std_pos_error)){


                    cout << "Slow movement: increase charge" << endl;
                    q_adjusted += -K_adjusted*((mean_position_error*rad_to_deg-threshold_pos_error));

                }
                else if(mean_position_error*rad_to_deg > (threshold_pos_error+threshold_std_pos_error)){
                    cout << "Fast movement: decrease charge" << endl;

                    q_adjusted += -K_adjusted*((mean_position_error*rad_to_deg-threshold_pos_error));

                }
                else {
                    cout << "Ok movement: Ok charge" << endl;
                }

//                if(num_iter%10==0){
//                     coeff_allocation+=0.25;

//                }

                // Copyright SDG please.
//                q_adjusted = std::max(0.0,std::min(1.0,q_adjusted));
                // OPTION 2:
                 q_adjusted = std::max(0.0,std::min(1.5,q_adjusted));

                 if(q_adjusted == 1.5){

                     counter_q_adj ++;

                 }


                cout << "Charge adjusted: " << q_adjusted << endl;


//                if(num_iter%10==0){
//                    coeff_allocation+=0.25;

//                }

            }




        }


        if(counter_waveform < EXERCISE_DURATION){
            counter_waveform++;
            campioni++;
        }



        // After elbow flexion/extension reset counters
        if (counter_waveform==EXERCISE_DURATION) {


//            if(num_iter%30==0){

//                if(num_iter%90==0){
//                    flag_boh=1;
//                }
//                if(flag_boh){
//                    coeff_allocation-=0.5;
//                }
//                if(flag_boh==0){
//                    coeff_allocation+=0.5;
//                }

//            }

//            if(num_iter%30==0){
//                threshold_std_pos_error+=0.5;

//            }

            sum_pos_err=0;
            sum_vel=0;
            delta = 0.0;
            mean = 0.0;
            M2 = 0.0;
            delta_vel=0;
            mean_vel=0;
            M2_vel=0;
            counter_waveform=0;
            campioni=0;
            T_limit=1.0;
            counter_iteration++;
            if(counter_rep%10==0 ){
                counter_rep=0;
                c.data->rehamove_mean_vel_block=mean_block_vel;
                c.data->rehamove_std_vel_block=std_block_vel;
                c.data->rehamove_delta_alloc=delta_allocation;
                c.data->impedance_control_command->impedance_control_feedforward_allocation_factor=coeff_allocation;
                c.data ->rehamove_counter_no_change = counter_no_change;
//                c.data ->rehamove_counter_q_adj = counter_q_adj;
                c.data->rehamove_num_block=num_block;
                c.data->rehamove_m=m;
                c.data->rehamove_up=up;
                c.data->rehamove_down=down;
                mean_block_vel=0.0;
                sum_block_vel = 0.0;
                std_block_vel=0.0;
                sum_pos_err_block=0.0;
                mean_block= 0.0;
                delta_block= 0.0;
                M2_block= 0.0;


            }
            if(counter_alloc==5){
                c.data->rehamove_m=m;
                c.data->rehamove_up=up;
                c.data->rehamove_down=down;
                mean_block_vel=0.0;
                sum_block_vel = 0.0;
                std_block_vel=0.0;
                sum_pos_err_block=0.0;
                mean_block= 0.0;
                delta_block= 0.0;
                M2_block= 0.0;
            }




        }

        // Increase waveform counter for next iteration until exercise duration
        break;
    }

//    default:
//        cout << "I should not be here" << endl;
//        break;
//    }

//    if(change_mode) change_mode=false;

//    // TODO: set shared memory
//    c.data->rehamove_mode=rehamove_mode;
//    c.data->rehamove_mean_position_error=mean_position_error;
//    c.data->rehamove_std_mean_position_error=std_position_error;
//    c.data->rehamove_iteration=num_iter;
//    c.data->rehamove_charge=q;
//    c.data->rehamove_charge_adj=q_adjusted;
//    c.data->rehamove_mean_mean_position_error=avg_mean_position_error;
//    c.data->rehamove_mean_std_mean_position_error=avg_std_position_error;

//    // Save parameters for feedforward motor torque
//    c.data->arm_weight_compensation_config.human_height_m = H;
//    c.data->arm_weight_compensation_config.human_weight_kg = W;
//    c.data->arm_weight_compensation_config.weight_assistance = coeff_weight_assistance;
//    c.data->impedance_control_command->impedance_control_feedforward_allocation_factor=coeff_allocation;
//    c.data->impedance_control_command->impedance_control_k_gain_mNm_per_rad = stiffness_nm_rad;
//    c.data->impedance_control_command->impedance_control_d_gain_mNm_per_rad_per_sec = damping_nms_rad;




}

}





void onedof_rehamove_app::stimulation_initialization(){

    smpt_open_serial_port(&device, port_name);
    smpt_clear_ml_init(&ml_init);
    ml_init.packet_number = smpt_packet_number_generator_next(&device);
    smpt_send_ml_init(&device, &ml_init);
    smpt_clear_ml_update(&ml_update);

    ml_update.enable_channel[Smpt_Channel_Red] = true;
    ml_update.enable_channel[Smpt_Channel_Blue] = true;

    ml_update.packet_number = smpt_packet_number_generator_next(&device);

    PLOGW << "Rehamove initialization done";
}

void onedof_rehamove_app::stimulation_initialization_motimove(){

    //bool sit = true;
    bool sit = fs.configTermios();  //TO DO: check not working
    fs.fes_State.active[ch_motimove] = true;


    if (sit){
      PLOGW << "Motimove initialization done";}
    else {
      PLOGW << "Motimove initialization error!!!!";
       rehamove_mode = 1;
        change_mode = true;}


}

void onedof_rehamove_app::stimulation_calibration(){

    torque=c.data->joint_values->loadcell_reading_mNm;

    ml_update.channel_config[Smpt_Channel_Red].number_of_points = Number_of_points;
    ml_update.channel_config[Smpt_Channel_Red].ramp = Ramp;
    ml_update.channel_config[Smpt_Channel_Red].period = Period; //mettere periodo 40 se voglio 25Hz

    ml_update.channel_config[Smpt_Channel_Red].points[0].current = corrente;
    ml_update.channel_config[Smpt_Channel_Red].points[1].current = 0;
    ml_update.channel_config[Smpt_Channel_Red].points[2].current = -corrente;

    ml_update.channel_config[Smpt_Channel_Red].points[0].time = durata;
    ml_update.channel_config[Smpt_Channel_Red].points[1].time = 0;
    ml_update.channel_config[Smpt_Channel_Red].points[2].time =durata;

    smpt_send_ml_update(&device, &ml_update);

    // Send packet
    Smpt_ml_get_current_data ml_get_current_data = {0};
    ml_get_current_data.packet_number = smpt_packet_number_generator_next(&device);
    ml_get_current_data.data_selection[Smpt_Ml_Data_Stimulation] = true;
    smpt_send_ml_get_current_data(&device, &ml_get_current_data);


    if(torque>prev_torque+DELTA_TORQUE && flag_min==0){
        // IDENTIFICAZIONE MINIMO
        Imin=round(corrente);
        PWmin=round(durata);

        flag_min=1;
    }

    if(corrente>24 || durata>1000 ){
        ml_update.enable_channel[Smpt_Channel_Red] = false;
        rehamove_mode = 102;
        change_mode = true;
        Imax=round(corrente);
        PWmax=round(durata);
    }

    corrente+=0.12;
    durata+=1.5;
    prev_torque=torque;

}

void onedof_rehamove_app::stimulation_calibration_motimove(){

    torque=c.data->joint_values->loadcell_reading_mNm;


    fs.fes_State.stim_value[ch_motimove] = corrente;
    fs.fes_State.pulse_width[ch_motimove] = durata;

    //cout << "Vado" << endl;

    fs.createChannelTrajectory();
    //cout << "Channel Traj fatto" << endl;
    fs.setChannels();
    //cout << "Channels fatto" << endl;
    fs.createMsg();
    //cout << "Create message fatto" << endl;

    if(torque>prev_torque+DELTA_TORQUE && flag_min==0){
        // IDENTIFICAZIONE MINIMO
        Imin=round(corrente);
        PWmin=round(durata);

        flag_min=1;
    }

    if(corrente>30 || durata>1000 ){
        ml_update.enable_channel[Smpt_Channel_Red] = false;
        rehamove_mode = 102;
        change_mode = true;
        Imax=round(corrente);
        PWmax=round(durata);
    }

    corrente+=0.12;
    durata+=1.5;
    prev_torque=torque;

}

void onedof_rehamove_app::stimulation_torque(vector <double> &calib){

    IMIN= calib[0];
    PWMIN= calib[1];
    IMAX= calib[2];
    PWMAX= calib[3];

    if(campioni>0 && campioni<EXERCISE_DURATION/2){

        // SINUSOID
        //q=q_adj*(1-c.data->impedance_control_command->impedance_control_feedforward_allocation_factor)*sin((2*M_PI*(1.0/8000.0)*campioni));


        // BETA FUNCTION
        //      q=charge_generation(q_adjusted*(1-c.data->impedance_control_command->impedance_control_feedforward_allocation_factor),campioni);
        
        // TODO: change charge with the new q
        // TODO: change q_adj based on mean of std found in 201 (100% motor)
        //        q=charge_generation(q_adj*(1-coeff_allocation),campioni);
        //        q=torque_based_charge_generation(campioni, counter_t, q_adj, counter_supp);

        // EMG BASED
        //        modulate_coefficient();
        //        q=q_adj*mod_coefficient;

        stim_current=IMIN+sqrt(q)*(IMAX-IMIN);
        stim_PW=PWMIN+sqrt(q)*(PWMAX-PWMIN);

        if (q == 0.0)
        {
          stim_current=0;
          stim_PW=0;
        }

    }

    else {

        stim_current=0;
        stim_PW=0;

    }

    ml_update.channel_config[Smpt_Channel_Red].number_of_points = Number_of_points;
    ml_update.channel_config[Smpt_Channel_Red].ramp = Ramp;
    ml_update.channel_config[Smpt_Channel_Red].period = Period; //mettere periodo 40 se voglio 25Hz

    ml_update.channel_config[Smpt_Channel_Red].points[0].current = stim_current;
    ml_update.channel_config[Smpt_Channel_Red].points[1].current = 0;
    ml_update.channel_config[Smpt_Channel_Red].points[2].current = -stim_current;

    ml_update.channel_config[Smpt_Channel_Red].points[0].time = stim_PW;
    ml_update.channel_config[Smpt_Channel_Red].points[1].time = 0;
    ml_update.channel_config[Smpt_Channel_Red].points[2].time =stim_PW;

    smpt_send_ml_update(&device, &ml_update);

    // Send packet
    Smpt_ml_get_current_data ml_get_current_data = {0};
    ml_get_current_data.packet_number = smpt_packet_number_generator_next(&device);
    ml_get_current_data.data_selection[Smpt_Ml_Data_Stimulation] = true;
    smpt_send_ml_get_current_data(&device, &ml_get_current_data);

    tau_feedback = (c.data->impedance_control_terms->torque_setpoint_mNm)-(c.data->impedance_control_terms->robot_dynamic_torque_mNm+c.data->impedance_control_terms->arm_dynamic_torque_mNm*coeff_allocation);

}

void onedof_rehamove_app::stimulation_torque_triceps(vector <double> &calib){

    IMIN= calib[0];
    PWMIN= calib[1];
    IMAX= calib[2];
    PWMAX= calib[3];

    IMIN_t = 3;
    IMAX_t = 9;
    PWMIN_t = 239;
    PWMAX_t = 304;

    if(campioni>0 && campioni<EXERCISE_DURATION/2){

        // SINUSOID
        //q=q_adj*(1-c.data->impedance_control_command->impedance_control_feedforward_allocation_factor)*sin((2*M_PI*(1.0/8000.0)*campioni));


        // BETA FUNCTION
        //      q=charge_generation(q_adjusted*(1-c.data->impedance_control_command->impedance_control_feedforward_allocation_factor),campioni);

        // TODO: change charge with the new q
        // TODO: change q_adj based on mean of std found in 201 (100% motor)
        //        q=charge_generation(q_adj*(1-coeff_allocation),campioni);
        //        q=torque_based_charge_generation(campioni, counter_t, q_adj, counter_supp);

        // EMG BASED
        //        modulate_coefficient();
        //        q=q_adj*mod_coefficient;

        stim_current=IMIN+sqrt(q)*(IMAX-IMIN);
        stim_PW=PWMIN+sqrt(q)*(PWMAX-PWMIN);

        if (q == 0.0)
        {
          stim_current=0;
          stim_PW=0;
        }

        stim_current_t=0;
        stim_PW_t=0;

    }

    else {

        stim_current=0;
        stim_PW=0;

        stim_current_t=IMIN_t+sqrt(q_t)*(IMAX_t-IMIN_t);
        stim_PW_t=PWMIN_t+sqrt(q_t)*(PWMAX_t-PWMIN_t);

        if (q_t == 0.0)
        {
          stim_current_t=0;
          stim_PW_t=0;
        }

    }

    ml_update.channel_config[Smpt_Channel_Red].number_of_points = Number_of_points;
    ml_update.channel_config[Smpt_Channel_Red].ramp = Ramp;
    ml_update.channel_config[Smpt_Channel_Red].period = Period; //mettere periodo 40 se voglio 25Hz

    ml_update.channel_config[Smpt_Channel_Red].points[0].current = stim_current;
    ml_update.channel_config[Smpt_Channel_Red].points[1].current = 0;
    ml_update.channel_config[Smpt_Channel_Red].points[2].current = -stim_current;

    ml_update.channel_config[Smpt_Channel_Red].points[0].time = stim_PW;
    ml_update.channel_config[Smpt_Channel_Red].points[1].time = 0;
    ml_update.channel_config[Smpt_Channel_Red].points[2].time =stim_PW;

    ml_update.channel_config[Smpt_Channel_Blue].number_of_points = Number_of_points;
    ml_update.channel_config[Smpt_Channel_Blue].ramp = Ramp;
    ml_update.channel_config[Smpt_Channel_Blue].period = Period; //mettere periodo 40 se voglio 25Hz

    ml_update.channel_config[Smpt_Channel_Blue].points[0].current = stim_current_t;
    ml_update.channel_config[Smpt_Channel_Blue].points[1].current = 0;
    ml_update.channel_config[Smpt_Channel_Blue].points[2].current = -stim_current_t;

    ml_update.channel_config[Smpt_Channel_Blue].points[0].time = stim_PW_t;
    ml_update.channel_config[Smpt_Channel_Blue].points[1].time = 0;
    ml_update.channel_config[Smpt_Channel_Blue].points[2].time =stim_PW_t;

    smpt_send_ml_update(&device, &ml_update);

    // Send packet
    Smpt_ml_get_current_data ml_get_current_data = {0};
    ml_get_current_data.packet_number = smpt_packet_number_generator_next(&device);
    ml_get_current_data.data_selection[Smpt_Ml_Data_Stimulation] = true;
    smpt_send_ml_get_current_data(&device, &ml_get_current_data);

    tau_feedback = (c.data->impedance_control_terms->torque_setpoint_mNm)-(c.data->impedance_control_terms->robot_dynamic_torque_mNm+c.data->impedance_control_terms->arm_dynamic_torque_mNm*coeff_allocation);

}

void onedof_rehamove_app::stimulation_torque_motimove(vector <double> &calib){

    IMIN= calib[0];
    PWMIN= calib[1];
    IMAX= calib[2];
    PWMAX= calib[3];

    if(campioni>0 && campioni<EXERCISE_DURATION/2){

        // SINUSOID
        //q=q_adj*(1-c.data->impedance_control_command->impedance_control_feedforward_allocation_factor)*sin((2*M_PI*(1.0/8000.0)*campioni));


        // BETA FUNCTION
        //      q=charge_generation(q_adjusted*(1-c.data->impedance_control_command->impedance_control_feedforward_allocation_factor),campioni);

        // TODO: change charge with the new q
        // TODO: change q_adj based on mean of std found in 201 (100% motor)
        //        q=charge_generation(q_adj*(1-coeff_allocation),campioni);
        //        q=torque_based_charge_generation(campioni, counter_t, q_adj, counter_supp);

        // EMG BASED
        //        modulate_coefficient();
        //        q=q_adj*mod_coefficient;

        stim_current=IMIN+sqrt(q)*(IMAX-IMIN);
        stim_PW=PWMIN+sqrt(q)*(PWMAX-PWMIN);
        if (q == 0.0)
        {
          stim_current=0;
          stim_PW=0;
        }

    }

    else {

        stim_current=0;
        stim_PW=0;

    }

    fs.fes_State.stim_value[ch_motimove] = stim_current;
    fs.fes_State.pulse_width[ch_motimove] = stim_PW;

    fs.createChannelTrajectory();
    fs.setChannels();
    fs.createMsg();

    tau_feedback = (c.data->impedance_control_terms->torque_setpoint_mNm)-(c.data->impedance_control_terms->robot_dynamic_torque_mNm+c.data->impedance_control_terms->arm_dynamic_torque_mNm*coeff_allocation);

}

double onedof_rehamove_app::trajectory_generation(int counter){

    // Compute beta function
    float P0,P1,P2,P3,P4,P5;
    double interim_time_exercise;
    double interim_setpoint_exercise;

    P0 = EXERCISE_START;
    P2 = 0.0;
    P3 = 5.0; //5ht Order
    P4 = P2+EXERCISE_DURATION;
    P5 = 5.0; //5th Order
    P1 = EXERCISE_AMPLITUDE/pow(EXERCISE_DURATION/2,(P3+P5));

    // Compute time variable
//    interim_time_exercise = (c.data->elapsed_time_ms-elapsed_time_ms_offset_exercise);
    interim_time_exercise=counter;

    // Beta-Function
    return interim_setpoint_exercise = P0+P1*pow((interim_time_exercise-P2),P3)*pow((P4-interim_time_exercise),P5);
}

double onedof_rehamove_app::calcoloPID( double setpoint, double pv, double charge )
{

    // Calculate error
    error = -(setpoint - pv);
    integral += error*dt;
    derivative = (error-pre_error)*dt;

    // Proportional term
    double Pout = kp * (error);
    double Iout = ki * integral;
    double Dout = kd * derivative;

    double output= Pout + Iout + Dout;


    // Restrict to max/min (si parla dell'incremento)
    if( output > max )
        output = max;
    else if( output < min )
        output = min;

    // Save error to previous error
    pre_error = error;

//    charge=charge+output;
    charge=charge+output;

    if(charge>1){charge=1;}
    else if(charge<0){charge=0;}

    return output;
//    return charge;
}

void onedof_rehamove_app:: open_calibration_file(){


    // Get system  time
    time_t t = time(nullptr);
    struct tm * now = localtime( & t );
    char buffer [80];

    // Open user-specific calibration file
    snprintf (buffer,80,"/home/esmacat/esmacat_rt/onedof_log/onedof_subjects/calib_user_%s.csv", userID);
    CSV_calibration_file.open(buffer);

    if(CSV_calibration_file.is_open()){
        cout << "USER Calibration file opened." << endl;
    }
}

void onedof_rehamove_app::close_calibration_file(){

    if(CSV_calibration_file.is_open()){
        CSV_calibration_file.close();
        cout << "USER Calibration file closed correctly." << endl;
    }

}

void onedof_rehamove_app::write_calibration_file(){

    CSV_calibration_file

            << 1 << ","
            << A << ","
            << B << ","
            << C << ","
            << D << "," 
            << H << ","
            << W << ","
            << E << "," //coeff_weight_assistance << ","
            << F << "," //<< avg_mean_position_error<< ","
            << G << "," //<< avg_std_position_error << ","
            << userID ;

}

void onedof_rehamove_app::write_empty_calibration_file(){

    CSV_calibration_file

            << 1 << ","
            << 0 << ","
            << 0 << ","
            << 0 << ","
            << 0 << ","
            << 0 << ","
            << 0 << ","
            << 0 << "," //coeff_weight_assistance << ","
            << 0 << "," //<< avg_mean_position_error<< ","
            << 0 << "," //<< avg_std_position_error << ","
            << userID ;

}

void read_calibration_file( vector <double> &calib){

// TENTATIVO 1
//    std::string user = userID;
//    std::stringstream ss;
//    ss<<"/home/esmacat/esmacat_rt/onedof_log/onedof_log_rehamove/_user"<< userID <<".csv";
//    std::ifstream file(ss.str());


    // Open user-specific calibration file
    char buffer [80];
    snprintf (buffer,80,"/home/esmacat/esmacat_rt/onedof_log/onedof_subjects/calib_user_%s.csv", userID);
    CSV_calibration_file_read.open(buffer, ios::in);

    if(CSV_calibration_file_read.is_open()){
        cout << "USER Calibration file opened." << endl;
    }

    // Get the roll number
    // of which the data is required
    int rollnum=1, roll2, count = 0;


    // Read the Data from the file
    // as String Vector
    vector<string> row;
    string line, word, temp;

    row.clear();

    // read an entire row and
    // store it in a string variable 'line'
    getline(CSV_calibration_file_read, line);


    // used for breaking words
    stringstream s(line);

    // read every column data of a row and
    // store it in a string variable, 'word'
    while (getline(s, word, ',')) {

        // add all the column data
        // of a row to a vector
        row.push_back(word);
    }

    // convert string to integer for comparision
    roll2 = stoi(row[0]);

    // Compare the roll number
    if (roll2 == rollnum) {

        // Print the found data
        count = 1;
//        cout << "Imin   " << row[1] << "\n";
//        cout << "PWmin  " << row[2] << "\n";
//        cout << "Imax   " << row[3] << "\n";
//        cout << "PWmax  " << row[4] << "\n";
//        cout << "Weight Ass" << row[7] << "\n";
//        cout << "Avg Pos Err" << row[8] << "\n";
//        cout << "Avg std Pos Err" << row[9]<< "\n";
//        cout << "UserID " << row[10] << endl;

        calib[0]=stod(row[1]);
        A = calib[0];
        calib[1]=stod(row[2]);
        B = calib[1];
        calib[2]=stod(row[3]);
        C = calib[2];
        calib[3]=stod(row[4]);
        D = calib[3];
        calib[4]=stod(row[5]);
        H = calib[4];
        calib[5]=stod(row[6]);
        W = calib[5];
        calib[6]=stod(row[7]);
        E = calib[6];
        calib[7]=stod(row[8]);
        F = calib[7];
        calib[8]=stod(row[9]);
        G = calib[8];
        //        calib[9]=stod(row[10]);



    }

    if (count == 0)

        cout << "Record not found\n";// File pointer


    if(CSV_calibration_file_read.is_open())
    {
        CSV_calibration_file_read.close();
        cout << "USER Calibration file closed correctly" << endl;
    }

}

/**
 * @brief onedof_rehamove_app::open_rehamove_file
 */
void onedof_rehamove_app::open_rehamove_file(){

    // Get system  time
    time_t t = time(nullptr);
    struct tm * now = localtime( & t );
    char buffer [100];
    char time_c [20];

    // Open user-specific data file
    // Open general data file
    strftime (time_c,80,"%Y-%m-%d-%H-%M-%S.csv",now);
    snprintf (buffer,100,"/home/esmacat/esmacat_rt/onedof_log/onedof_subjects/data_user_%s_%s", userID, time_c);
    CSV_user_file.open (buffer);
    if(CSV_user_file.is_open())
    {
        PLOGI << "USER Log File Created";
        CSV_user_file << endl

                          << "mode" << ","
                          << "time" << ","
                          << "biceps_current" << ","
                          << "biceps_pulsewidth" << ","
                          << "biceps_charge" << ","
                          << "biceps_charge_correction" << ","
                          //<< "triceps_current" << ","
                          //<< "triceps_charge_correction" << ","
                          << "actual_position" << ","
                          << "desired_position" << ","
                          << "actual_torque" << ","
                          << "desired_torque" << ","
                          << "actual_velocity" << ","
                          << "stiffness" << ","
                          << "damping" << ","
                          << "allocation_factor" << ","
                          << "coeff_weight_assistance" << ","
                          << "total_dynamic_torque" << ","
                          << "robot_dynamic_torque" << ","
                          << "arm_dynamic_torque" << ","
                          << "robot_allocated_dynamic_torque" << ","
                          << "fes_allocated_dynamic_torque" << ","
                          << "counter_waveform" << ","
                          << "mean_position_error" << ","
                          << "std_position_error" << ","
                          //<< "mean_position_error_t" << ","
                          //<< "std_position_error_t" << ","
                          << "delta_allocation" << ","
                          << "block" << ","
                          << "mean_velocity_block" <<","
                          << "std_velocity_block" << ","
                          << "current_m" << ","
                          << "current_upper_bound" << ","
                          << "current_lower_bound" << ","
                          << "flag allocation" << ",";


    }

    else{
        PLOGE << "USER Log File Error";
    }


}

/**
 * @brief onedof_rehamove_app::write_rehamove_file
 */
void onedof_rehamove_app::write_rehamove_file(){

    CSV_user_file << endl

               << rehamove_mode << ","
               << loop_count << ","
               << stim_current << ","
               << stim_PW << ","
               << q << ","
               << q_adjusted <<","
               //<< q_t << ","
               //<< q_adjusted_triceps <<","
               << act_pos << ","
               << des_pos << ","
               << act_torque << ","
               << des_torque << ","
               << act_velocity << ","
               << stiffness_nm_rad << ","
               << damping_nms_rad << ","
               << coeff_allocation << ","
               << coeff_weight_assistance << ","
               << c.data->impedance_control_terms->total_dynamic_torque_mNm << ","
               << c.data->impedance_control_terms->robot_dynamic_torque_mNm << ","
               << c.data->impedance_control_terms->arm_dynamic_torque_mNm << ","
               << c.data->impedance_control_terms->robot_dynamic_torque_mNm+c.data->impedance_control_terms->arm_dynamic_torque_mNm*coeff_allocation << ","
               << c.data->impedance_control_terms->arm_dynamic_torque_mNm*(1.0 - coeff_allocation) << ","
               << counter_waveform << ","
               << mean_position_error << ","
               << std_position_error << ","
               //<< mean_position_error_t << ","
               //<< std_position_error_t << ","
               << delta_allocation << ","
               << num_block << ","
               << mean_block_vel<< ","
               << std_block_vel << ","
               << m << ","
               << up << ","
               << down << ","
               << change_alloc << ",";

}

/**
 * @brief onedof_rehamove_app::close_rehamove_file
 */
void onedof_rehamove_app::close_rehamove_file(){

    if(CSV_user_file.is_open())
    {
    CSV_user_file.close();
    cout << "USER data file closed correctly." << endl;
    }
}

/**
 * @brief onedof_rehamove_app::charge_waveform_generation
 * @param iteration
 * @param t
 * @return
 */
double onedof_rehamove_app::charge_waveform_generation(double iteration, double t){

    // Declare beta function parameters
    double P0,P1,P2,P3,P4,P5;
    double T = EXERCISE_DURATION/2.0; // ms
    double T_d=200.0;                 // ms
    double K= 0.5;
    double Q_slope = 0.002; // aumento lo slope se diminuisco il q_slope

    // support variables
    double beta_function;
    double charge;

    // Rising edge
    P0 = EXERCISE_START;
    P2 = 0.0;
    P3 = 5.0; //5ht Order

    //Falling edge
    P4 = P2+EXERCISE_DURATION;
    P5 = 5.0; //5th Order

    P1 = EXERCISE_AMPLITUDE/pow(EXERCISE_DURATION/2,(P3+P5));

    q0 = sin(EXERCISE_START);

    // Compute beta-function

//    if(iteration > 0){
        // 1. First beta-function
        if(t >= 0 && t < (T_limit*T - T_d)  ){
            // Compute charge during first phase
            charge = sin(P0+P1*pow((t+T_d-P2),P3)*pow((P4-(t+T_d)),P5));

            // Compute differential charge
            charge_dot = charge - charge_old;

            // Compute max or min (inversion of sign)
            if((charge_dot<0 && charge_dot_old>=0 && t>100)||(charge_dot>0 && charge_dot_old<=0 && t>100)){
                T_limit = t/T;
                //cout << "Max found at "<< T_limit << endl;
            }

            // Store values
            charge_old = charge;
            charge_dot_old = charge_dot;
        }

        // 2. Plateau
        else if(t < (T_limit*T +T_d)){
            charge = 1;
        }

        // 3. Second beta-function
        else if(t <= T){
            charge = sin(P0+P1*pow((t-P2),P3)*pow((P4-t),P5));
        }

        // 4. Descending slope
        else if(t >= T && t < 1.5*T){
            charge = sin(EXERCISE_AMPLITUDE+EXERCISE_START) - Q_slope*(t-T);
            if(charge < 0 ) charge = 0;
        }

        // 5. Ascending slope
        else if(t >= 1.5*T && t <= 2*T){
            if(t > 2*T - sin(EXERCISE_START)/Q_slope){
                charge = (t-2*T+sin(EXERCISE_START)/Q_slope)*Q_slope;
            }
            else{
                charge = 0;
            }
        }

        // X. else
        else{
            charge = 0;
        }

//    }


//    else {

//        // 0. First ascending slope
//        if(t >= 1.5*T && t <= 2*T){
//            if(t > - sin(EXERCISE_START/Q_slope)){
//                charge = (t+EXERCISE_START/Q_slope)*Q_slope;
//            }
//            else{
//                charge = 0;
//            }
//        }
//        // X. else
//        else{
//            charge = 0;
//        }

//    }

    //cout << "charge = " << charge << "\n" << endl;
    return charge;

}

double onedof_rehamove_app::charge_waveform_generation_tri(double iteration, double t){

    // Declare beta function parameters
    double P0,P1,P2,P3,P4,P5;
    double T = EXERCISE_DURATION/2.0; // ms
    double T_d=200.0;                 // ms
    double K= 0.5;
    double Q_slope = 0.002; // aumento lo slope se diminuisco il q_slope

    // support variables
    double beta_function;
    double charge;

    // Rising edge
    P0 = EXERCISE_START;
    P2 = 0.0;
    P3 = 5.0; //5ht Order

    //Falling edge
    P4 = P2+EXERCISE_DURATION;
    P5 = 5.0; //5th Order

    P1 = EXERCISE_AMPLITUDE/pow(EXERCISE_DURATION/2,(P3+P5));

    q0 = sin(EXERCISE_START);

    if (t>=0 && t<=0.5*T)
    {
      charge = sin(EXERCISE_START) - Q_slope*t;
      if (charge <0)
      {
        charge = 0;
      }
    }
    else if (t>0.5*T && t<=T)
    {
      charge = sin(EXERCISE_AMPLITUDE+EXERCISE_START) + Q_slope*(t-T);
      if(charge<0)
      {
        charge = 0;
      }
    }
    else if (t>T)
    {
      charge =  sin(P0+P1*pow((t-P2),P3)*pow((P4-(t)),P5));
    }
    else {
      charge = 0;
    }

    //cout << "charge = " << charge << "\n" << endl;
    return charge;

}

/**
 * @brief onedof_rehamove_app::charge_waveform_generation
 * @param iteration
 * @param t
 * @return
 */
double onedof_rehamove_app::charge_waveform_generation_new(double iteration, double t){

    // Declare beta function parameters
    double P0,P1,P2,P3,P4,P5;
    double T = EXERCISE_DURATION/2.0; // ms
    double T_d=0.0;                 // ms
    double K= 0.5;
    double Q_slope = 0.002; // aumento lo slope se diminuisco il q_slope

    // support variables
    double beta_function;
    double charge;

    // Rising edge
    P0 = EXERCISE_START;
    P2 = 0.0;
    P3 = 5.0; //5ht Order

    //Falling edge
    P4 = P2+EXERCISE_DURATION;
    P5 = 5.0; //5th Order

    P1 = EXERCISE_AMPLITUDE/pow(EXERCISE_DURATION/2,(P3+P5));

    q0 = sin(EXERCISE_START);

    double scaling = ((P0 + P1*pow(EXERCISE_DURATION/3-P2,P3)*pow(P4-EXERCISE_DURATION/3,P5)-EXERCISE_START)/EXERCISE_AMPLITUDE)-((P0 + P1*pow(EXERCISE_DURATION/3-1-P2,P3)*pow(P4-EXERCISE_DURATION/3+1,P5)-EXERCISE_START)/EXERCISE_AMPLITUDE);

    // Compute beta-function
    if (t>=0 && t<T)
    {
      charge = ((P0+P1*pow((t+T_d-P2),P3)*pow((P4-(t+T_d)),P5)-EXERCISE_START)/EXERCISE_AMPLITUDE)-((P0+P1*pow((t-1+T_d-P2),P3)*pow((P4-(t-1+T_d)),P5)-EXERCISE_START)/EXERCISE_AMPLITUDE);
      charge = charge/scaling;
    }
    else {
      charge = 0;
    }
    //cout << "charge = " << charge << "\n" << endl;
//

    return charge;

}




