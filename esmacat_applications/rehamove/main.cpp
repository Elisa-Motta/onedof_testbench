//*****************************************************

//----------------------------------
//--------HEADERS STIMOLAZIONE------
//----------------------------------


#include "smpt_ll_client.h"
#include "smpt_client.h"
#include "smpt_ml_client.h"
#include "smpt_messages.h"
#include "smpt_packet_number_generator.h"
#include "smpt_ll_packet_validity.h"

#include "pid.h"
#include "onedof_shared_memory_comm.h"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <sys/ioctl.h>
#include <termios.h>

using namespace std;

#define DIM 10
#define NITER 100

//----------------------------------
//------VARIABILI STIMOLAZIONE------
//----------------------------------
int iterazione=0;
double avg_tau_R_fes=0; //moving average
double avg_tau_imp=0;
double tau_imp=0;
double tau_R_fes;
double tau_R_fes_vect[DIM]={0};
double tau_imp_vect[DIM]={0};
double sum_fes;
double sum_imp;
int i;
double print_alpha[NITER]={0}; //la dimensione è il numero di iterazioni
double print_tau_R_fes[NITER]={0};
double print_tau_imp[NITER]={0}; // questi tre vettori print mi servono per salvare i valori
double tau_R_exo; // lo invio a motore
double tau_R; // arriva da motore
int corrente_stimolazione;
bool check_stop;
bool check_close;
//int i;
int check_send;
int check_data;
int check_open;
int corr;
int numero_imp;
int numero_impulsi =0;
double timp=5;
bool mStop=false;
double alpha;
bool mUpdate=false;
int mode;
char input;
double charge_new;

onedof_shared_memory_comm c;
void set_impedance();


// inizializzazione stimolazione

const char *port_name= "/dev/ttyUSB0";
Smpt_device device= {0};
Smpt_ml_init ml_init = {0};
Smpt_ml_update ml_update = {0};

//*****************************************************

//----------------------------------
//-----HEADERS REALTIME TEMPLATE----
//----------------------------------

#include <iostream>
#include <stdio.h>              // printf
#include <string.h>
#include <time.h>               // cyclic loop
#include <pthread.h>            // multithread
#include <algorithm>            // std::copy
#include <signal.h>             // to catch ctrl-c
#include <unistd.h>

#include <plog/Log.h>
#include <plog/Appenders/ColorConsoleAppender.h>
#include "time_spec_operation.h"
#include "loop_time_stats.h"

#define DEFAULT_LOOP_TIME_NS 1000000L
#define DEFAULT_APP_DURATION_COUNTS 10000
#define NSEC_PER_SEC 1000000000L
#define ALLOWED_LOOPTIME_OVERFLOW_NS 200000L

//----------------------------------
//------TEXT COLOR IDENTIFIERS------
//----------------------------------

const string boldred_key = "\033[1;31m";
const string red_key = "\033[31m";
const string boldpurple_key = "\033[1;35m";
const string yellow_key = "\033[33m";
const string blue_key = "\033[36m";
const string green_key = "\033[32m";
const string color_key = "\033[0m";

//----------------------------------
//------------PROTOTYPES------------
//----------------------------------

bool kbhit();
void print_command_keys();


//***************************************************

// my task  la funzione che viene inserita nel LOOP real time
// viene quindi generata in loop fino a che questo non decade

void my_task(){



                    ml_update.channel_config[Smpt_Channel_Red].number_of_points = 3;
                    ml_update.channel_config[Smpt_Channel_Red].ramp = 3;
                    ml_update.channel_config[Smpt_Channel_Red].period = 30;

                    ml_update.channel_config[Smpt_Channel_Red].points[0].current = corrente_stimolazione;
                    ml_update.channel_config[Smpt_Channel_Red].points[1].current = 0;
                    ml_update.channel_config[Smpt_Channel_Red].points[2].current = -corrente_stimolazione;

                    corr= ml_update.channel_config[Smpt_Channel_Red].points[0].current;
                    cout << "corrente attuale" << corr << endl;

                    ml_update.channel_config[Smpt_Channel_Red].points[0].time = 300;
                    ml_update.channel_config[Smpt_Channel_Red].points[1].time = 100;
                    ml_update.channel_config[Smpt_Channel_Red].points[2].time = 300;

                    //    check_send=smpt_send_ml_update(&device, &ml_update);
                    //    qDebug() << "check send" << check_send;
                    // check_send=smpt_send_ml_update(&device, &ml_update);
                    smpt_send_ml_update(&device, &ml_update);
                    //cout << "check send" << check_send;

                    Smpt_ml_get_current_data ml_get_current_data = {0};
                    //fill_ml_get_current_data()
                    ml_get_current_data.packet_number = smpt_packet_number_generator_next(&device);
                    ml_get_current_data.data_selection[Smpt_Ml_Data_Stimulation] = true;

                    //check_data=smpt_send_ml_get_current_data(&device, &ml_get_current_data);
                    smpt_send_ml_get_current_data(&device, &ml_get_current_data);
                   // cout << "check data" << check_data;


                    // ad ogni ciclo voglio salvare un vettore per le tau_impedence (ad ogni ciclo significa ogni 0.04 sec)
                    // un vettore delle tau_ref_fes

                    //prendere valore della tau_imp da shared memory

//                    if(iterazione>1){
//                        for(i=1; i<DIM; i++)
//                        {
//                            //a ogni iterazione li sposto e salvo nell'ultimo posto dell'array il nuovo valore che ottengo dalla memory
//                            tau_R_fes_vect[i-1]=tau_R_fes_vect[i];
//                            tau_R_fes_vect[DIM-1]=abs(tau_R_fes); //arriva dall'allocatore

//                            tau_imp_vect[i-1]=tau_imp_vect[i];
//                            tau_imp_vect[DIM-1]=abs(timp);
//                            //tau_imp_vect[DIM-1]=abs(tau_imp); //arriva dal motore

//                        }}

//                    //---------------------
//                    //------ALLOCATOR------
//                    //---------------------

//                    if(iterazione==0)
//                    {
//                        alpha=0.8;
//                    }
//                    else if(avg_tau_R_fes==0) //siamo alla prima iter oppure in altri casi particolari
//                    {
//                        alpha=0;
//                    }

//                    else if(iterazione>0){

//                        // ALLOCAZIONE CONTINUA - CALCOLO MEDIA MOBILE SU 0.4 SEC
//                        // calcolo average del tau impedenza in valore assoluto
//                        // calcolo average del taur_r_fes in valore assoluto
//                        //sui vettori che si aggiornano a ogni iterazione
//                        for(i=0; i<DIM; i++)
//                        {
//                            sum_fes=sum_fes+tau_R_fes_vect[i];
//                            sum_imp=sum_imp+tau_imp_vect[i];

//                        }
//                        avg_tau_R_fes=sum_fes/DIM; // finestre di 10 iterazioni da 0.04 sec = finestre da 0.4 secondi
//                        avg_tau_imp=sum_imp/DIM;

//                        // calcolo alpha
//                        alpha=avg_tau_imp/avg_tau_R_fes;
//                        cout << "alpha" << alpha << endl;

//                        //salvo gli alpha per tenerne traccia
//                        print_alpha[iterazione]=alpha;

//                        tau_R_fes=tau_R*(1-alpha); // da inviare al PID
//                        tau_R_exo=tau_R-tau_R_fes; // da inviare a motore
//                        //c.data->impedance_control_command->impedance_control_feedforward_torque_mNm=tau_R_exo;

//                        print_tau_R_fes[iterazione]=tau_R_fes;
//                        print_tau_imp[iterazione]=timp;

//                    }


                    //---------------------
                    //---------PID---------
                    //---------------------

                    PID pid = PID(0.1, 100, -100, 0.1, 0.01, 0.5);


                    if(iterazione==0)
                    {corrente_stimolazione=8;}
                    else if (iterazione>1){
                        corrente_stimolazione= ml_update.channel_config[Smpt_Channel_Red].points[0].current;} //di fatto sto prendendo quella dell'iterazione precedente
                    //qDebug() << "se la stimolazione è > 1 la corrente è: " << corrente_stimolazione;
                    //       tau_imp=torque_imp;
                    cout << "tau imp nel PID è " << timp << endl;

                    double set_point= tau_R_fes;

                    double feedback=(tau_R_fes - timp); // di fatto il tau muscles è la tauRfes - tauimp


                    //        for (int i = 0; i < 1; i++) {

                    //qDebug() << "taum imp " << torque_imp;
                    set_point=10; // setto la tau r fes a 10 fissa


                    double charge = pid.calculate(set_point, feedback, charge_new);

                    charge_new=charge;

                    //se setto che la tau_r_fes deve essere 10 e la tau_imp è 2 significa che devo dare e=10-2 al PID per avere una corrente tale da generare 10 di torque quindi sarà una corrente
                    // più grande volta per volta. se invece faccio che la tau_imp diminuisce avrò che voglio 10, genero 15, avrò un contributo finchè non ho che la torque imp è =10 quindi no error.
                    // inc è l'incremento positivo o negativo che devo apportare alla corrente per fare in modo che sia corretta a seconda dell'errore
                    // sulla torque (tau_R_fes-tau_muscles=tau_R_fes-(tau_R_fes-tau_imp)=tau_imp)

                    // in realtà qui andrebbe settata la charge che è rapporto tra la corrente e la pulsewidth


                    cout<< "corrente_stimolazione da PID "<<corrente_stimolazione << endl;
                    cout << "charge" << charge << "\n";

                    //---------------------------------------------------------------------------------------------------------------


                    if(mUpdate==true) {


                        cout << "sto aggiornando a 25Hz" << endl;
                        ml_update.channel_config[Smpt_Channel_Red].points[0].current = corrente_stimolazione;
                        ml_update.channel_config[Smpt_Channel_Red].points[1].current = 0;
                        ml_update.channel_config[Smpt_Channel_Red].points[2].current = -corrente_stimolazione;


                        corr= ml_update.channel_config[Smpt_Channel_Red].points[0].current;
                        cout << "corrente attuale" << corr << endl;

                        smpt_send_ml_update(&device, &ml_update);
                      // cout << "check send update" << check_send <<endl;


                        Smpt_ml_get_current_data ml_get_current_data = {0};
                        //fill_ml_get_current_data()
                        ml_get_current_data.packet_number = smpt_packet_number_generator_next(&device);
                        ml_get_current_data.data_selection[Smpt_Ml_Data_Stimulation] = true;

                        //check_data=smpt_send_ml_get_current_data(&device, &ml_get_current_data);
                        smpt_send_ml_get_current_data(&device, &ml_get_current_data);
                      //  cout << "check data update" << check_data << endl;
                         mUpdate=false;

                    }

                    //                check_send=smpt_send_ml_update(&device, &ml_update);
                    //                cout << "check send" << check_send;

                    //        Smpt_ml_get_current_data ml_get_current_data = {0};
                    //        //fill_ml_get_current_data()
                    //        ml_get_current_data.packet_number = smpt_packet_number_generator_next(&device);
                    //        ml_get_current_data.data_selection[Smpt_Ml_Data_Stimulation] = true;

                    //        check_data=smpt_send_ml_get_current_data(&device, &ml_get_current_data);
                    //        cout << "check data" << check_data;

                    //        iterazione++;
                    //        cout << "iterazione" << iterazione;

                    if(timp<10)
                    {    timp=timp+1;

                    }


//                    if(mStop==true) {



//                        ml_update.channel_config[Smpt_Channel_Red].points[0].current = 0;
//                        ml_update.channel_config[Smpt_Channel_Red].points[1].current = 0;
//                        ml_update.channel_config[Smpt_Channel_Red].points[2].current = 0;


//                        corr= ml_update.channel_config[Smpt_Channel_Red].points[0].current;
////                        cout << "corrente attuale" << corr;

//                        smpt_send_ml_update(&device, &ml_update);
////                        cout << "check send" << check_send;
//                        mUpdate=false;

//                        Smpt_ml_get_current_data ml_get_current_data = {0};
//                        //fill_ml_get_current_data()
//                        ml_get_current_data.packet_number = smpt_packet_number_generator_next(&device);
//                        ml_get_current_data.data_selection[Smpt_Ml_Data_Stimulation] = true;

//                        check_data=smpt_send_ml_get_current_data(&device, &ml_get_current_data);
////                        cout << "check data" << check_data;

//                        smpt_send_ml_stop(&device, smpt_packet_number_generator_next(&device));
//                        smpt_close_serial_port(&device);


//                    }

//                    sleep(2);

//                    break;
//                }

//                case 2:
//                {
//                    cout << "stop stimolazione";
//                    smpt_send_ml_stop(&device, smpt_packet_number_generator_next(&device));
//                    smpt_close_serial_port(&device);

//                    break;

        }

//    }

//}

void* rt(void* cookie){

    loop_time_stats realtime_loop_time_stats("realtime_loop_time_stats.txt",loop_time_stats::output_mode::screenout_only);

    unsigned int period = DEFAULT_LOOP_TIME_NS;
    struct timespec t_start, t_now, t_next,t_period,t_result;
    unsigned long int t_overflow = 0;   // measure the overflowed time for each cycle
    unsigned long int loop_count = 0;
    /* ... */
    /* Initialization code */
    clock_gettime(CLOCK_MONOTONIC, &t_start);
    clock_gettime( CLOCK_MONOTONIC, &t_now);
    t_next = t_now;


    while(loop_count<DEFAULT_APP_DURATION_COUNTS){

        realtime_loop_time_stats.loop_starting_point();

        /* Calculate the time for the execution of this task*/
        t_period.tv_sec = 0;
        t_period.tv_nsec = DEFAULT_LOOP_TIME_NS;
        TIMESPEC_INCREMENT ( t_next, t_period );

                clock_gettime ( CLOCK_MONOTONIC, &t_now);

        if(loop_count%1000==0){
            timespec_sub(&t_result,&t_now,&t_start);
            PLOGI << green_key << "RT Clock 100: " << (double) (timespec_to_nsec(&t_result)/1e9) << color_key << endl;
            if( c.data->control_mode_command== 7){
                my_task();
                iterazione++;
                cout << "iterazione" << iterazione << endl;
            }


//            if(timp>10)
//            {    timp=timp-0.5;
//            }

        }
        if(loop_count%4000==0){
            timespec_sub(&t_result,&t_now,&t_start);
            PLOGI << green_key << "RT Clock 25: " << (double) (timespec_to_nsec(&t_result)/1e9) << color_key << endl;
            mUpdate=true;

        }
        loop_count++;

        /* Sleep until the next execution*/
        clock_nanosleep ( CLOCK_MONOTONIC, TIMER_ABSTIME, &t_next, nullptr );
        clock_gettime ( CLOCK_MONOTONIC, &t_now);

        t_overflow = (t_now.tv_sec*1e9 + t_now.tv_nsec) - (t_next.tv_sec*1e9 + t_next.tv_nsec);
        if(t_overflow > ALLOWED_LOOPTIME_OVERFLOW_NS)
        {
            cout << red_key << "RT Overflow: " << t_overflow << color_key << endl;
        }

    }

    realtime_loop_time_stats.store_loop_time_stats();
}

void *nrt(void * federica){
    /* NRT code */
    loop_time_stats non_realtime_loop_time_stats("non_realtime_loop_time_stats.txt",loop_time_stats::output_mode::screenout_only);
    static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender; // Create the 2nd appender.

    plog::init(plog::info,&consoleAppender);

    struct timespec t_start, t_now, t_next, t_period, t_prev,t_result;
    clock_gettime(CLOCK_REALTIME, &t_start);
    t_next = t_start;
    unsigned long int loop_count = 0;
    unsigned long int t_overflow = 0;

    /* Calculate the time for the execution of this task*/
    t_period.tv_sec = 0;
    t_period.tv_nsec = DEFAULT_LOOP_TIME_NS;

    while(loop_count<DEFAULT_APP_DURATION_COUNTS*2)
    {
        t_prev = t_next;
        non_realtime_loop_time_stats.loop_starting_point();
        if(loop_count%1000==0){
            timespec_sub(&t_result,&t_now,&t_start);
            PLOGI << yellow_key << "NRT Clock: " << (double) (timespec_to_nsec(&t_result)/1e9) << color_key << endl;
        }
        loop_count++;
        timespec_add_nsec(&t_next, &t_prev, DEFAULT_LOOP_TIME_NS);
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &t_next, NULL);
        clock_gettime ( CLOCK_REALTIME, &t_now);

        t_overflow = (t_now.tv_sec*1e9 + t_now.tv_nsec) - (t_next.tv_sec*1e9 + t_next.tv_nsec);
        if(t_overflow > ALLOWED_LOOPTIME_OVERFLOW_NS)
        {
            cout << red_key << "NRT Overflow: " << t_overflow << color_key << endl;
        }
    }

    non_realtime_loop_time_stats.store_loop_time_stats();

}

void set_impedance()
{
    c.data->impedance_control_command->soft_stop_lower_limit_rad=-180;
    c.data->impedance_control_command->soft_stop_upper_limit_rad=180;
    c.data->impedance_control_command->impedance_control_k_gain_mNm_per_rad=0;
    c.data->impedance_control_command->impedance_control_d_gain_mNm_per_rad_per_sec=0;
    c.data->impedance_control_command->impedance_control_setpoint_rad=-1;

}

int main()
{

    set_impedance();

    int policy;
    struct sched_param prio;
    pthread_attr_t attr;

    pthread_t rt_loop;
    pthread_t nrt_loop;

    // open serial port
    check_open=smpt_open_serial_port(&device, port_name);
    cout << "check_open " << check_open <<endl;

    //initialize mid level stimulation
    smpt_clear_ml_init(&ml_init);
    ml_init.packet_number = smpt_packet_number_generator_next(&device);
    smpt_send_ml_init(&device, &ml_init); /* Send the ml_init command to the stimulation unit */

    smpt_clear_ml_update(&ml_update);
    ml_update.enable_channel[Smpt_Channel_Red] = true;
    ml_update.packet_number = smpt_packet_number_generator_next(&device);



    //--------initialize shared memory---------
    //-----------------------------------------
        static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender; //include application.h to get the plog libraries

        c.init();
        //plog::init(plog::info, &consoleAppender);

        if (c.init())
        {
            cout << "User Interface shared memory initialized with key: " << hex << c.get_shared_memory_key() << endl;    // start the shared memory communication
        }
        else
        {
            cout << "User Interface shared memory initialization has been failed";
            c.detach_shared_memory();
            return 0;
        }

        char input_char = '0';
        std::string mode_name; //This section should ideally be done with an enum

    //c.restore_default_values();
//    print_command_keys();

    //-----------------------------------------
    //-----------------------------------------




//    while(mStop==false)
//    {


//        if(kbhit())
//        {

//            cin >> input;

//            switch(input)
//            {
//            case ' ':
//                print_command_keys();
//                break;
//            case '0':

//                c.data->control_mode_command=robot_control_mode_t::quit;
//                c.data->stop = true;
//                mStop=true;
//                mode=2;
//                cout << "stop stimulation ";




//                break;

//            case '7':

//                mode = 1;
//                iterazione=0;

//                if(mode==1){
//                for(i=0; i<10; i++)
//                {
//                    cout << i << endl;
//                    sleep(1);
//                }}

                /** REAL-TIME THREAD */
                pthread_attr_init( &attr);
                pthread_attr_setinheritsched( &attr, PTHREAD_EXPLICIT_SCHED);
                policy = SCHED_RR;
                pthread_attr_setschedpolicy( &attr, policy);
                prio.sched_priority = 1; // priority range should be btw -20 to +19
                pthread_attr_setschedparam(&attr,&prio);

                if ( pthread_create(&rt_loop, &attr, rt, nullptr ) ){
                    PLOGE <<  "Error: rt not created" ;
                    return 1;
                }


                /** NON REAL-TIME THREAD */

            //    pthread_attr_init( &attr);
            //    pthread_attr_setinheritsched( &attr, PTHREAD_EXPLICIT_SCHED);
            //    policy = SCHED_OTHER;
            //    pthread_attr_setschedpolicy( &attr, policy);
            //    prio.sched_priority = 1; // priority range should be btw -20 to +19
            //    pthread_attr_setschedparam(&attr,&prio);

            //    if ( pthread_create(&nrt_loop, &attr, nrt, nullptr) ){
            //        PLOGE << "Error: nrt not created" ;
            //        return 1;
            //    }



//                /* wait for threads to finish */
//                pthread_join(rt_loop,NULL);
//                cout << ": Running " << robot_mode_labels[11] << " mode!" << endl;
//                c.data->control_mode_command = static_cast<robot_control_mode_t>(11);


//                break;

//            default:
//                cout << "Invalid Input - stopping test"<< endl;
//                c.data->control_mode_command =robot_control_mode_t::quit;
//                break;


//            }

//        }




//    /** NON REAL-TIME THREAD */

//    pthread_attr_init( &attr);
//    pthread_attr_setinheritsched( &attr, PTHREAD_EXPLICIT_SCHED);
//    policy = SCHED_OTHER;
//    pthread_attr_setschedpolicy( &attr, policy);
//    prio.sched_priority = 1; // priority range should be btw -20 to +19
//    pthread_attr_setschedparam(&attr,&prio);

//    if ( pthread_create(&nrt_loop, &attr, nrt, nullptr) ){
//        PLOGE << "Error: nrt not created" ;
//        return 1;
//    }



//    /* wait for threads to finish */
      pthread_join(rt_loop,NULL);
//   // pthread_join(nrt_loop,NULL);

    return 0;
}





//----------------------------------
//-------------FUNCTIONS------------
//----------------------------------

// wait for keyboard interrupt
bool kbhit()
{
    termios term;
    tcgetattr(0, &term);

    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);

    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);

    tcsetattr(0, TCSANOW, &term);

    return byteswaiting > 0;
}


// Print commands on terminal
void print_command_keys()
{
    std::cout   << ": Enter command for test"  << endl;
  std::cout << boldred_key << "\nCOMMAND KEYS:"<< color_key << std::endl;
  std::cout << blue_key << "\'0\'" << color_key << ": EXIT" << "\n";
  std::cout << blue_key << "\'1\'" << color_key << ": STOP mode"<< "\n";

  std::cout << blue_key << "\'2\'" << color_key << ": CURRENT mode"<< "\n";
  std::cout << blue_key << "\'3\'" << color_key << ": TORQUE mode" << "\n";
  std::cout << blue_key << "\'4\'" << color_key << ": NULL-TORQUE mode" << "\n";

  std::cout << blue_key << "\'5\'" << color_key << ": ANTI-G mode"<< "\n";
  std::cout << blue_key << "\'6\'" << color_key << ": FREEZE mode"<< "\n";
  std::cout << blue_key << "\'7\'" << color_key << ": IMPEDANCE mode"<< "\n";


//  std::cout << blue_key << "\'5\'" << color_key << ": FREEZE mode"<< "\n";
//  std::cout << blue_key << "\'6\'" << color_key << ": HOMING mode"<< "\n";
//  std::cout << blue_key << "\'7\'" << color_key << ": POSITION mode"<< "\n";

//  std::cout << blue_key << "\'t\'" << color_key << ": PASSIVE mode"<< "\n";
//  std::cout << blue_key << "\'i\'" << color_key << ": IMPEDANCE EXTERNAL mode"<< "\n";
//  std::cout << blue_key << "\'w\'" << color_key << ": ANTI-G mode"<< "\n";
//  std::cout << blue_key << "\'g\'" << color_key << ": TRANSPARENT mode"<< "\n";
//  std::cout << blue_key << "\'r\'" << color_key << ": RESISTIVE mode"<< "\n";
//  std::cout << blue_key << "\'c\'" << color_key << ": CHALLENGING mode"<< "\n";

//  cout << endl;
//  std::cout << blue_key << "\'d+\'" << color_key << ": increase DAMPING"<< "\n";
//  std::cout << blue_key << "\'d-\'" << color_key << ": decrease DAMPING"<< "\n";
//  std::cout << blue_key << "\'k+\'" << color_key << ": increase STIFFNESS"<< "\n";
//  std::cout << blue_key << "\'k-\'" << color_key << ": decrease STIFFNESS"<< "\n";
//  std::cout << blue_key << "\'a+\'" << color_key << ": increase WEIGHT assistance"<< "\n";
//  std::cout << blue_key << "\'a-\'" << color_key << ": decrease WEIGHT assistance"<< "\n";
  cout << endl;
  std::cout << blue_key << "\'M\'" << color_key << ": SHOW current settings and command keys\n"<< "\n";
}

