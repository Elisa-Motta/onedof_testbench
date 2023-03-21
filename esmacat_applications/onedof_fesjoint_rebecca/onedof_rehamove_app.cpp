#include "onedof_rehamove_app.h"

#include <stdlib.h>
#include <libconfig.h++>

#define CALIBRATION_VALUES  "Calibrationvalues"
#define L0_NODE_NAME "Imin"
#define L1_NODE_NAME "PWmin"
#define L2_NODE_NAME "Imin"
#define L3_NODE_NAME "Imin"

#define CHARGE_DURATION 4000

float A=0;
float B=0;
float C=0;
float D=0;
float H=0;
float W=0;
int g;
ofstream CSVfile;
ifstream CSVfile_read;
ofstream CSV_rehamove_newfile;
ifstream CSVnewfile_read;
//ofstream CSV_GS_file;
//ifstream CSV_GS_file_read;
//double M_GS=0.0;
//double STD=0.0;

//void openfile();
//void openfile2();
//void closefile();
//void writefile();
//void readfile(vector <double> &calib);
//void read_GS_file (vector <double> &GS_params);
void read_calibration_file(vector <double> &calib_r);
//void writenewfile();
//void opennewfile();


void onedof_rehamove_app::FSM(){
    write_rehamove_file();

    //TODO: read shared memory
    // torque setpoint is computer as: gravity_torque_mNm + controller_torque_mNm + friction_comp_torque_mNm + soft_stop_torque_mNm
    torque_impedance=c.data->impedance_control_terms->torque_setpoint_mNm;
    act_pos = c.data->joint_values->incremental_encoder_reading_radians;
    des_pos = static_cast<double>(c.data->impedance_control_terms->impedance_control_setpoint_rad);
    act_torque = c.data->joint_values->loadcell_reading_mNm;
    des_torque = c.data->impedance_control_terms->torque_setpoint_mNm;
    act_velocity = c.data->joint_values->incremental_encoder_speed_radians_sec;

    // torque setpoint is computer as: gravity_torque_mNm + controller_torque_mNm + friction_comp_torque_mNm + soft_stop_torque_mNm
    //rehamove_mode = 1;

    H=1.70;
    W=55.0;


    if(change_mode && rehamove_mode!=602 && rehamove_mode!=601 ){
        cout << " Mode changed to: ";
    }

    switch (rehamove_mode)
    {
    case 0:
        if(change_mode){
            cout << " EXIT mode" << endl;
            close_rehamove_file();}
        c.data->control_mode_command = robot_control_mode_t::quit;
        ml_update.enable_channel[Smpt_Channel_Red] = false;
        c.data->stop = true;
        break;
    case 1:
        // Executed only at mode change
        if( change_mode){
            cout << " STOP mode" << endl;
        }
        stiffness_nm_rad = 5.0;
        damping_nms_rad = 1.0;
        coeff_allocation = 0.0;
        c.data->control_mode_command = robot_control_mode_t::standby;
        ml_update.enable_channel[Smpt_Channel_Red] = false;
        break;
    case 2:
        // Executed only at mode change
        if( change_mode){
            cout << " HOMING mode" << endl;
        }
        c.data->control_mode_command = robot_control_mode_t::homing_control;
        ml_update.enable_channel[Smpt_Channel_Red] = false;
        break;

    case 101:

        if(change_mode){
            cout << " STIM CALIBRATION mode" << endl;
            stimulation_initialization();
            corrente=5;
            durata=200;
//            q_calib=0.0;
            campioni=0;

            // For dynamic calibration
            //readfile(calib);

        }



        // STANDARD CALIBRATION IN FREEZE
        c.data->control_mode_command = robot_control_mode_t::freeze_control;
        // Giunto molto rigido 60.0/40.0 - 6.0/4.0
        c.data->impedance_control_command->impedance_control_k_gain_mNm_per_rad = 60.0;
        c.data->impedance_control_command->impedance_control_d_gain_mNm_per_rad_per_sec = 6.0;
        if(loop_count%100==0){
        stimulation_calibration();
      }

        // DYNAMIC CALIBRATION IN TRANSPARENT
//        c.data->control_mode_command = robot_control_mode_t::gravity_control;
//        trajectory_generation();

//        if(loop_count%100==0){
//            stimulation_calibration_bis(calib);
//        }
//        campioni++;
//        if (campioni==8000) {
//            q_calib+=0.2;
//            campioni=0;
//            q=0;}
//        if(q_calib>=1) q_calib=1;
        break;

    case 102:

        cout << " STOP CALIBRATION mode" << endl;

        C=corrente;
        D=durata;
        rehamove_mode=103;
        ml_update.enable_channel[Smpt_Channel_Red] = false;

        break;

    case 103:

        cout << " END Calibration" << endl;

        open_calibration_file();
        write_calibration_file();
        close_calibration_file();

        Imin=A;
        PWmin=B;
        Imax=C;
        PWmax=D;

        cout << " MIN CURRENT" << Imin << endl;
        cout << " MIN PULSEWIDTH" <<PWmin << endl;
        cout << " MAX CURRENT" << Imax << endl;
        cout << " MAX PULSEWIDTH" << PWmax << endl;

        ml_update.enable_channel[Smpt_Channel_Red] = false;

        rehamove_mode=1;

        break;



    case 4:
        if( change_mode){
            cout << " WEIGHT ASSISTANCE CALIBRATION " << endl;
            coeff_weight_assistance=0.4;

        }

        c.data->control_mode_command = robot_control_mode_t::impedance_ext_control;
        stiffness_nm_rad        = 0.0;
        damping_nms_rad = 0.5;
        break;


    case 5:
        if(change_mode){
            cout << " 100% MOTOR ALLOCATION" << endl;
            campioni=0;
            coeff_allocation = 1.0; /* % MOTOR ALLOCATION */
            //            coeff_weight_assistance= 0.4;
//            stiffness_nm_rad = 10.0;
//            damping_nms_rad = 1.0;

            delta = 0.0;
            mean = 0.0;
            M2 = 0.0;
            mean_std_mean_position_error = 0.0;
            sum_std_mean_position_error = 0.0;
            sum_mean_position_error = 0.0;
            mean_mean_position_error = 0.0;

        }

        c.data->control_mode_command = robot_control_mode_t::impedance_control;
        c.data->impedance_control_command->impedance_control_k_gain_mNm_per_rad         = stiffness_nm_rad;
        c.data->impedance_control_command->impedance_control_d_gain_mNm_per_rad_per_sec = damping_nms_rad;

        campioni++;

        // Compute trajectory error with sign
        pos_err=des_pos-act_pos;
        sum_pos_err+=pos_err;

        // Compute Standard Deviation with streaming data
        delta = pos_err - mean;
        mean = mean + delta/campioni;
        M2 = M2+delta*(pos_err-mean);

        // After one elbow flexion compute mean position error
        if(campioni==FLEXION_DURATION){
            num_iter++;
            mean_position_error=sum_pos_err/FLEXION_DURATION;
            std_position_error = sqrt(M2/(FLEXION_DURATION-1));
            sum_std_mean_position_error+=std_position_error;
            mean_std_mean_position_error = sum_std_mean_position_error/num_iter;
            sum_mean_position_error += mean_position_error;
            mean_mean_position_error = sum_mean_position_error/num_iter;

        }

        // After elbow flexion/extension reset counters
        if (campioni==EXERCISE_DURATION) {

            c.data->rehamove_mean_mean_position_error=mean_mean_position_error;
            c.data->rehamove_mean_std_mean_position_error=mean_std_mean_position_error;


            campioni=0;
            sum_pos_err= 0.0;
            mean_position_error = 0.0;
            std_position_error = 0.0;
            delta = 0.0;
            mean = 0.0;
            M2 = 0.0;

        }

        break;









    case 6:
        if( change_mode){

        }


        break;





    case 7:

        cout << "USER ID:";
        char user;
        user=getchar();

        break;

//    case 7:
//        // Start the impedance control with calibration parameters set above

//        if(change_mode){
//            cout << " IMPEDANCE mode + FES - ALLOCATION" << endl;
//            stimulation_initialization();

//            // Read CONFIGURATION file (Imin, PWmin, Imax, PWmax) obtained with CALIBRATION
//            readfile(calib);
//           cout << "il vettore di calibrazione è:" << calib[0] << "," << calib[1] << ","<< calib[2] << "," << calib[3] <<  endl;
//            closefile();
//            campioni=0;
//            first_rep_flag=0;

//        }

//        // linear coefficient calculation tau-q

//        c.data->control_mode_command = robot_control_mode_t::impedance_control;
//        c.data->impedance_control_command->impedance_control_k_gain_mNm_per_rad         = 30.0;
//        c.data->impedance_control_command->impedance_control_d_gain_mNm_per_rad_per_sec = 2.0;
//        c.data->arm_weight_compensation_config.human_height_m = 1.70;
//        c.data->arm_weight_compensation_config.human_weight_kg = 60;
//        c.data->arm_weight_compensation_config.weight_assistance = 0.5;


//        campioni++; // dove li metto i campioni conteggiati????

//        //c.data->impedance_control_command->impedance_control_feedforward_allocation_factor=motor_allocation;
//        c.data->impedance_control_command->impedance_control_feedforward_allocation_factor=1.0;
//        // provo a usare fattore allocazione fisso





//        // ... fuori a 1000Hz?


//        if(loop_count%10){
//    stimulation_torque(calib);

//        }
        
//        //update_allocation_factor(); // lo metto qui perchè l'alfa mi serve per la ripetizione successiva, e alfa viene calcolato
//        // dalla funzione quando nella ripetizione ormai non serve più, quindi va per la


//        if (campioni==8000) {
//            campioni=0;
//        }

//        //if(campioni==1000) {iter_seno=0;}
//        //if(campioni==1001) {q=0;}
//       //if(campioni>=4000) {
//          //  q=0;
//        //}  // annullo carica anche qui per sicurezza....

//        break;


    case 8:
        // POSITION mode
        if( change_mode){
            cout << " Go to position mode" << endl;

        }
        c.data->control_mode_command = robot_control_mode_t::go_position_control;


        break;





    case 9:

        // Increase/Decrease charge amplitude + increase/decrease allocation modulation
        if(change_mode){
            cout << "IMPEDANCE + CHARGE MODULATION + ALLOCATION MODULATION" << endl;
            stimulation_initialization();
//            read_calibration_file(calib_r);
//            threshold_pos_error=calib_r[7];
//            threshold_std_pos_error=calib_r[8];
//            coeff_weight_assistance = calib_r[6];
//            close_calibration_file();

            threshold_pos_error=3;
            threshold_std_pos_error=2;
            coeff_weight_assistance = 0.3;

            campioni=0;
            num_iter=0;
            counter_t=0;
            counter_supp=0;
            q_adjusted=0.2;

            //coeff_allocation=1.0;
            //q_adjusted=1.0-coeff_allocation;
//            coeff_allocation = 1.0; /* % MOTOR ALLOCATION */
            // da inserire dopo sua calibrazione

            stiffness_nm_rad = 5;
            damping_nms_rad = 1;
          //  q_adj=q_adjusted;

//            delta = 0.0;
//            mean = 0.0;
//            M2 = 0.0;

            K_learning = 0.1;


            delta = 0.0;
            mean = 0.0;
            M2 = 0.0;

            charge_old= 0.0;
            charge_dot = 0.0;
            charge_dot_old= 0.0;
             T_limit = 0.65;
             q_old=0.0;

             // Initialize waveform counter
             counter_waveform = 0;
             counter_iteration = 0;
          }

        c.data->control_mode_command = robot_control_mode_t::impedance_control;
        c.data->impedance_control_command->impedance_control_k_gain_mNm_per_rad         = stiffness_nm_rad;
        c.data->impedance_control_command->impedance_control_d_gain_mNm_per_rad_per_sec = damping_nms_rad;


        q=charge_waveform_generation(counter_iteration, counter_waveform)*q_adjusted;

        if(loop_count%10==0){
        stimulation_torque(calib_r);
      }

        // Compute trajectory error with/without sign
        if(counter_waveform<=EXERCISE_DURATION/2){

            // Compute position error
            pos_err=des_pos-act_pos;
            // cumulate position error
             sum_pos_err+=pos_err;

            // Compute Standard Deviation with streaming data
            delta = abs(pos_err) - mean;
            mean = mean + delta/static_cast<double>(counter_waveform+1);
            M2 = M2+delta*(abs(pos_err)-mean);
        }

        // After elbow flexion compute mean position error and mean std
        if(counter_waveform==EXERCISE_DURATION/2){

            num_iter++;
            mean_position_error=sum_pos_err/counter_waveform;

            std_position_error = sqrt(M2/(EXERCISE_DURATION/2-1));

            sum_std_mean_position_error+=std_position_error;
            mean_std_mean_position_error = sum_std_mean_position_error/num_iter;

            sum_mean_position_error+=mean_position_error;
            mean_mean_position_error=sum_mean_position_error/counter_waveform;

            charge_old= 0.0;
            charge_dot = 0.0;
            charge_dot_old= 0.0;

        }


        if(num_iter > 1){

            // Iterative learning

            if (counter_waveform>=4500 && counter_waveform<4501){



                cout << "**************************" << endl;
                cout << "Iterative Learning Control" << endl;
                cout << "Mean error: " << mean_position_error << endl;
                cout << "Std  error: " << std_position_error<< endl;
                cout << "Thr error: " << threshold_pos_error<< endl;
//                cout << "Thr std: " << threshold_std_pos_error << endl;

                if(mean_position_error <= (0-abs(threshold_pos_error))){

                    cout << "Slow movement: increase charge" << endl;

                    q_adjusted += K_learning*(0-mean_position_error);
                    // Provare anche con distanza dallo zero
                    //q_adjusted += K_learning*(-threshold_pos_error-mean_position_error);



                }
                if(mean_position_error>= (0+abs(threshold_std_pos_error))){

                    cout << "Fast movement: decrease charge" << endl;
                    q_adjusted += K_learning*(0+mean_position_error);
                    // Provare anche con distanza dallo zero
                    // q_adjusted += K_learning*(+threshold_pos_err-mean_position_error);
                }
                if(mean_position_error < abs(threshold_pos_error) && mean_position_error > -abs(threshold_pos_error)){
                    cout << "Ok movement: Ok charge" << endl;
                }
                // Copyright SDG please.
                q_adjusted = std::max(0.0,std::min(1.0,q_adjusted));
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

            // Allocazione cambia automaticamente (per le prove)
            if(num_iter%10==0){
                coeff_allocation+=0.2;
                q_adjusted=0.2;

            }

        }


        break;

    default:
        cout << "I should not be here" << endl;
        break;

    }

    if(change_mode)        change_mode=false;

    // TODO: set shared memory
    c.data->rehamove_mode=rehamove_mode;
    c.data->rehamove_mean_position_error=mean_position_error;
    c.data->rehamove_iteration=num_iter;
    c.data->rehamove_charge=q;
    c.data->rehamove_charge_adj=q_adjusted;
    c.data->rehamove_std_mean_position_error=std_position_error;
    c.data->rehamove_mean_mean_position_error=mean_mean_position_error;
    c.data->rehamove_mean_std_mean_position_error=mean_std_mean_position_error;

    // Save parameters for feedforward motor torque
    c.data->arm_weight_compensation_config.human_height_m = H;
    c.data->arm_weight_compensation_config.human_weight_kg = W;
    c.data->arm_weight_compensation_config.weight_assistance = coeff_weight_assistance;
    c.data->impedance_control_command->impedance_control_feedforward_allocation_factor=coeff_allocation;
    c.data->impedance_control_command->impedance_control_k_gain_mNm_per_rad = stiffness_nm_rad;
    c.data->impedance_control_command->impedance_control_d_gain_mNm_per_rad_per_sec = damping_nms_rad;



}



void onedof_rehamove_app::stimulation_initialization(){

    smpt_open_serial_port(&device, port_name);
    smpt_clear_ml_init(&ml_init);
    ml_init.packet_number = smpt_packet_number_generator_next(&device);
    smpt_send_ml_init(&device, &ml_init);
    smpt_clear_ml_update(&ml_update);

    ml_update.enable_channel[Smpt_Channel_Red] = true;
    ml_update.packet_number = smpt_packet_number_generator_next(&device);

    PLOGW << "Rehamove initialization done";
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

    cout << "corrente " << corrente << endl;
    cout << "pulsewidth " << durata << endl;
    cout << "torque" << torque << endl;
//    cout << "prev_torque" << prev_torque << endl;


    if(torque>prev_torque+DELTA_TORQUE && flag_min==0){
        // IDENTIFICAZIONE MINIMO
        A=corrente;
        B=durata;

        flag_min=1;
    }

    if(corrente>24 || durata>500 ){
        ml_update.enable_channel[Smpt_Channel_Red] = false;
        rehamove_mode = 102;
        change_mode = true;
        C=corrente;
        D=durata;

    }

    corrente+=0.12;
    durata+=1.5;

    prev_torque=torque;

}


void onedof_rehamove_app::stimulation_torque(vector <double> &calib_r){


    IMIN= calib_r[0];
    PWMIN= calib_r[1];
    IMAX= calib_r[2];
    PWMAX= calib_r[3];

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

    if(campioni>0 && campioni<EXERCISE_DURATION){


        // SINUSOID
        //q=q_adj*(1-c.data->impedance_control_command->impedance_control_feedforward_allocation_factor)*sin((2*M_PI*(1.0/8000.0)*campioni));


        // BETA FUNCTION
//      q=charge_generation(q_adjusted*(1-c.data->impedance_control_command->impedance_control_feedforward_allocation_factor),campioni);
//        q=torque_based_charge_generation(campioni,count_back); // non sono sicura di q_adjusted...

        // LINEAR RELATIONSHIP WITH TORQUE
//        q = charge_waveform_generation(counter_iteration,counter_waveform);


        // EMG BASED
//        modulate_coefficient();
//        q=q_adj*mod_coefficient;

        stim_current=IMIN+sqrt(q)*(IMAX-IMIN);
        stim_PW=PWMIN+sqrt(q)*(PWMAX-PWMIN);

    }



    tau_feedback = (c.data->impedance_control_terms->torque_setpoint_mNm)-(c.data->impedance_control_terms->robot_dynamic_torque_mNm+c.data->impedance_control_terms->arm_dynamic_torque_mNm*c.data->impedance_control_command->impedance_control_feedforward_allocation_factor);

}



double onedof_rehamove_app::calcoloPID( double setpoint, double pv, double charge )
{

    // Calculate error
    error = -(setpoint - pv);

    // Proportional term
    double Pout = kp * (error);

    double Iout = ki * integral;

      double output= Pout + Iout;


    // Restrict to max/min (si parla dell'incremento)
    if( output > max )
        output = max;
    else if( output < min )
        output = min;

    // Save error to previous error
    pre_error = (-error);

    charge=charge+output;


    if(charge>1){charge=1;}
    else if(charge<0){charge=0;}

    //return output;
    return charge;
}

void onedof_rehamove_app:: open_calibration_file(){


    // Get system  time
    time_t t = time(nullptr);
    struct tm * now = localtime( & t );
    char buffer [80];

    // Log directory

        strftime (buffer,80,"/home/esmacat/esmacat_rt/onedof_log/onedof_log_rehamove/RP.csv",now);
        CSVfile.open (buffer);



}


void onedof_rehamove_app::close_calibration_file(){

    CSVfile.close();

}

void onedof_rehamove_app::write_calibration_file(){



    CSVfile

             << 1 << ","
             << A << ","
             << B << ","
             << C << ","
             << D << ","
             << H << ","
             << W << ","
             << coeff_weight_assistance << ","
             << mean_mean_position_error<< ","
             << mean_std_mean_position_error;


}

void read_calibration_file( vector <double> &calib_r){

    fstream fin;

       // Open an existing file
       fin.open("/home/esmacat/esmacat_rt/onedof_log/onedof_log_rehamove/RP.csv", ios::in);

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
       getline(fin, line);

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
           cout << "Imin " << row[1] << "\n";
           cout << "PWmin" << row[2] << "\n";
           cout << "Imax " << row[3] << "\n";
           cout << "PWmax" << row[4] << "\n";


           calib_r[0]=stod(row[1]);
           calib_r[1]=stod(row[2]);
           calib_r[2]=stod(row[3]);
           calib_r[3]=stod(row[4]);
           calib_r[4]=stod(row[5]);
           calib_r[5]=stod(row[6]);
           calib_r[6]=stod(row[7]);
           calib_r[7]=stod(row[8]);
           calib_r[8]=stod(row[9]);


       }

       if (count == 0)

           cout << "Record not found\n";// File pointer



}

void onedof_rehamove_app::open_rehamove_file(){

    // Get system  time
    time_t t = time(nullptr);
    struct tm * now = localtime( & t );
    char buffer [80];

    // Log directory
    strftime (buffer,80,"/home/esmacat/esmacat_rt/onedof_log/onedof-rehamove-RP-%Y-%m-%d-%H-%M-%S.csv",now);
    CSV_rehamove_newfile.open (buffer);
    if(CSV_rehamove_newfile.is_open())
    {
        //PLOGI << "AGREE Log File Created Successfully";

        CSV_rehamove_newfile << endl

                             << "mode" << ","
                             << "time" << ","
                             << "biceps_current" << ","
                             << "biceps_pulsewidth" << ","
                             << "biceps_charge" << ","
                             << "biceps_charge_correction" << ","
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
                             << "avg_mean_position_error" <<","
                             << "avg_std_position_error" << ",";

    }

    else{
        PLOGE << "REHAMOVE Log File Error";
    }
}

void onedof_rehamove_app::write_rehamove_file(){

    CSV_rehamove_newfile << endl

                         << rehamove_mode << ","
                         << loop_count << ","
                         << stim_current << ","
                         << stim_PW << ","
                         << q << ","
                         << q_adjusted <<","
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
                         << threshold_pos_error << ","
                         << threshold_std_pos_error << ",";


}

void onedof_rehamove_app::close_rehamove_file(){

    CSV_rehamove_newfile.close();

}


double onedof_rehamove_app::update_charge_percentage(vector <double> &GS_params, double value){  //mettere dentro PE medio ??


    if (value > GS_params[0]+GS_params[1] ){
        delta_percentage=K_learning*((GS_params[0]+GS_params[1])-value);
    }
    if(value < GS_params[0]-GS_params[1]){
        delta_percentage=K_learning*((GS_params[0]-GS_params[1])- value);
    }
   if(value < GS_params[0]+GS_params[1]  && value > GS_params[0]-GS_params[1]){
       delta_percentage=0;
   }



    return delta_percentage;


}



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
                cout << "Max found at "<< T_limit << endl;
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

    return charge;

}



void onedof_rehamove_app::update_allocation_factor(){

    //double interim_time_exercise;



    //interim_time_exercise = (c.data->elapsed_time_ms-elapsed_time_ms_offset_exercise);


    if(campioni < EXERCISE_DURATION/2) {

        sum_tau_imp = sum_tau_imp + (c.data->impedance_control_terms->torque_setpoint_mNm)-(c.data->impedance_control_terms->robot_dynamic_torque_mNm+c.data->impedance_control_terms->arm_dynamic_torque_mNm*c.data->impedance_control_command->impedance_control_feedforward_allocation_factor);
    }


    if (campioni > EXERCISE_DURATION - 1 && first_rep_flag==1){

        mean_tau_imp=sum_tau_imp/(EXERCISE_DURATION/2);

        if( abs(mean_tau_imp) >  abs(old_tau_imp) ){     // probabilmente ci andrà una soglia...

            var=(abs(mean_tau_imp)-abs(old_tau_imp))/abs(old_tau_imp); // probabilmente fuzzy logic, non resterà diretto var

             if(mean_tau_imp > 0){  //

                 alfa= alfa+var;

              }

              else if (mean_tau_imp < 0) {

                 alfa = alfa - var;


                }

                if(alfa>1.0){

                    alfa=1.0;

                }

                else if (alfa<0.0) {

                   alfa=0.0;

               }


           }

            else {

             var=(abs(mean_tau_imp)-abs(old_tau_imp))/abs(old_tau_imp);

            }

            old_tau_imp=mean_tau_imp;

            sum_tau_imp = 0;


    }


    if(campioni > EXERCISE_DURATION -1 && first_rep_flag==0)  {

            old_tau_imp=sum_tau_imp/(EXERCISE_DURATION/2);

            first_rep_flag=1;

            sum_tau_imp=0;

        }


}






