#include "onedof_rehamove_class.h"
#include <stdio.h>


bool kbhit();


onedof_rehamove_class::onedof_rehamove_class(){

    // Initializing the shared memory
    if (app.c.init())
    {
        PLOGI << "User Interface shared memory initialized with key: " << hex << app.c.get_shared_memory_key();    // start the shared memory communication
    }
    else
    {
        PLOGE << "User Interface shared memory initialization has been failed";
        app.c.detach_shared_memory();
    }

}

onedof_rehamove_class::~onedof_rehamove_class(){
    //join_thread();
    //join_rt_thread();
}

void onedof_rehamove_class::getinfo()
{
    struct sched_param param;
    int policy;

    sched_getparam(0, &param);
    printf("Priority of this process: %d\n\r", param.sched_priority);

    pthread_getschedparam(pthread_self(), &policy, &param);

    printf("Priority of the thread: %d, current policy is: %d\n\r",
              param.sched_priority, policy);
}


// Thread functions

void onedof_rehamove_class::nrt_thread(){
    /* NRT code */

    loop_time_stats non_realtime_loop_time_stats("non_realtime_loop_time_stats.txt",loop_time_stats::output_mode::screenout_only);

    struct timespec t_start, t_now, t_next, t_period, t_prev,t_result;

    PLOGW << "NRT Thread";

    getinfo();

    clock_gettime(CLOCK_REALTIME, &t_start);
    t_next = t_start;
    unsigned long int loop_count = 0;
    unsigned long int t_overflow = 0;

    /* Calculate the time for the execution of this task*/
    t_period.tv_sec = 0;
    t_period.tv_nsec = DEFAULT_LOOP_TIME_NS;

   print_command_keys();


//   while(loop_count<20000 && c.data->stop==false)
        while(app.c.data->stop==false)
    {
        t_prev = t_next;
        non_realtime_loop_time_stats.loop_starting_point();

        interface();

        if(loop_count%1000==0){
            timespec_sub(&t_result,&t_now,&t_start);
            //PLOGI << yellow_key << "NRT Clock: " << (double) (timespec_to_nsec(&t_result)/1e9) << color_key << endl;
        }
        loop_count++;
        timespec_add_nsec(&t_next, &t_prev, DEFAULT_LOOP_TIME_NS*10);
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &t_next, NULL);
        clock_gettime ( CLOCK_REALTIME, &t_now);

        t_overflow = (t_now.tv_sec*1e9 + t_now.tv_nsec) - (t_next.tv_sec*1e9 + t_next.tv_nsec);
        if(t_overflow > ALLOWED_LOOPTIME_OVERFLOW_NS)
        {
//            cout << red_key << "NRT Overflow: " << t_overflow << color_key  << endl;
        }
    }

        stop_thread();

}

void onedof_rehamove_class::rt_thread(){

    loop_time_stats realtime_loop_time_stats("realtime_loop_time_stats.txt",loop_time_stats::output_mode::screenout_only);
    struct timespec t_start, t_now, t_next,t_period,t_result; // timespec variable to handle timing

    PLOGW << "RT Thread";
    getinfo();

    clock_gettime(CLOCK_MONOTONIC, &t_start);
    clock_gettime( CLOCK_MONOTONIC, &t_now);
    t_next = t_now;
//    unsigned long int t_overflow = 0;   // measure the overflowed time for each cycle
//    unsigned long int loop_count = 0;

    /* Non-cyclic task */


    /* Cyclic Loop */
   while(app.c.data->stop==false){

        // Save timing stats
        realtime_loop_time_stats.loop_starting_point();

        // Calculate the time period for the execution of this task
        t_period.tv_sec = 0;
        t_period.tv_nsec = DEFAULT_LOOP_TIME_NS;

        // Compute next cycle deadline
        TIMESPEC_INCREMENT ( t_next, t_period );

//        // Debug cycle at 1 Hz
//        if(app.loop_count%1000==0){

        // Debug cycle at 1 kHz
        if(app.loop_count%1==0){

            // Compute time since start
            timespec_sub(&t_result,&t_now,&t_start);
            //PLOGI << green_key << "RT Clock: " << (double) (timespec_to_nsec(&t_result)/1e9) << color_key << endl;

//            cout << stim.variable << endl;
            // Run my cyclic task
            app.FSM();


            //interface();

        }
        app.loop_count++;

        // Sleep until the next execution time
        clock_nanosleep ( CLOCK_MONOTONIC, TIMER_ABSTIME, &t_next, nullptr );
        // Get time after sleep
        clock_gettime ( CLOCK_MONOTONIC, &t_now);
        // Compute overflow time
        app.t_overflow = (t_now.tv_sec*1e9 + t_now.tv_nsec) - (t_next.tv_sec*1e9 + t_next.tv_nsec);
        // If overflow is too big, print overrun
        if(app.t_overflow > ALLOWED_LOOPTIME_OVERFLOW_NS)
        {
//            cout << red_key << "RT Overflow: " << t_overflow << color_key << endl;
        }

    }

    // Print histogram on screen
    realtime_loop_time_stats.store_loop_time_stats();

//    stop_rt_thread();
}

void onedof_rehamove_class::stop_thread(){

    //    non_realtime_loop_time_stats.store_loop_time_stats();

}

void onedof_rehamove_class::stop_rt_thread(){



}

bool onedof_rehamove_class::start_thread(){
    // Scheduler variables
    int policy;
    struct sched_param prio;
    pthread_attr_t attr;

    pthread_attr_init( &attr);
    pthread_attr_setinheritsched( &attr, PTHREAD_EXPLICIT_SCHED);

    policy = SCHED_OTHER;

    pthread_attr_setschedpolicy( &attr, policy);
    prio.sched_priority = 1; // priority range should be btw -20 to +19
    pthread_attr_setschedparam(&attr,&prio);

    if ( pthread_create(&pthread_nrt_input, &attr, internal_nrt, this) ){
        PLOGE << "Error: nrt thread" ;
        return 1;
    }
    return 0;
}

bool onedof_rehamove_class::start_rt_thread(){

    // Scheduler variables
    int policy;
    struct sched_param prio;
    pthread_attr_t attr;

    pthread_attr_init( &attr);
    pthread_attr_setinheritsched( &attr, PTHREAD_EXPLICIT_SCHED);

    policy = SCHED_RR;
    pthread_attr_setschedpolicy( &attr, policy);
    prio.sched_priority = 1 ; // priority range should be btw 0 and 99
    pthread_attr_setschedparam(&attr,&prio);

    if ( pthread_create(&pthread_rt_rehamove, &attr, internal_rt, this) ){
        PLOGE << "Error: rt thread" ;
        return 1;
    }
    return 0;
}

void onedof_rehamove_class::join_thread()
{
    (void) pthread_join(pthread_nrt_input, NULL);

}

void onedof_rehamove_class::join_rt_thread(){

        (void) pthread_join(pthread_rt_rehamove, NULL);
}






// User interface (switch between modalities)
void onedof_rehamove_class::interface()
{

    // Non-blocking function, if keyboard hit is detected, do something.
    if(kbhit())

    {

        char input_char = '0';
        uint16_t mode = 1;

        // Get character
//        cin >> input_char;
        input_char = getchar();

        switch (input_char){


        case ' ':
            print_command_keys();
            break;
        case '0':
            // EXIT
            mode=0;
            app.close_rehamove_file();
            break;
        case '1':
            // STOP mode
            mode=1;
            break;
        case '2':
            // HOMING mode
            mode=2;
            break;
        case '3':
            // FES CALIBRATION
            mode=101;
            break;

        case '4':
            // MOTOR CALIBRATION 1: WEIGHT ASSISTANCE
            //in quale configurazione abbiamo deciso di metterlo??
            mode=4;
            break;

        case '5':
            // MOTOR CALIBRATION 2: Compute mean and std only motor
            mode=5;
            break;


        case '6':
            // IMPEDANCE CONTROL + ALLOCATION ** parameter setting
            mode=6;
            break;
//        case '+':
//            cout << ": Allocation Increased" << endl;
//            mode=601;
//            if(app.q_adjusted<0) app.q_adjusted=0.0;
//            if(app.q_adjusted>1) app.q_adjusted=1.0;
//            else app.q_adjusted += 0.05;
//            break;
//        case '-':
//            cout << ": Allocation Decreased" << endl;
//            mode=602;

//            if(app.q_adjusted<0) app.q_adjusted=0.0;
//            if(app.q_adjusted>1) app.q_adjusted=1.0;
//            else app.q_adjusted -= 0.05;

//            break;
        case 'k':  case 'K':
            while(kbhit()==false);
            input_char=getchar();
            if(input_char=='+'){
                app.stiffness_nm_rad += 2.0;
                 cout << "Stiffness : "<<  app.stiffness_nm_rad << endl;
            }
            else if(input_char=='-'){
                app.stiffness_nm_rad -= 2.0;
                cout << "Stiffness : "<<  app.stiffness_nm_rad << endl;
            }
            else{
               cout << "Unknown command" << endl;
            }
            break;

        case 'd': case 'D':
            while(kbhit()==false);
                input_char=getchar();
                if(input_char=='+'){
                    app.damping_nms_rad += 0.5;
                    cout << "Damping : "<<  app.damping_nms_rad << endl;
                }
                else if(input_char=='-'){
                    app.damping_nms_rad -= 0.5;
                    cout << "Damping : "<<  app.damping_nms_rad << endl;
                }
                else{
                    cout << "Unknown command" << endl;
                }

            break;
        case 'w': case 'W':
            while(kbhit()==false);
            input_char=getchar();
            if(input_char=='-'){
                app.coeff_weight_assistance -= 0.05;
                if(app.coeff_weight_assistance<0) app.coeff_weight_assistance=0;
                cout << "Weight assistance : "<<  app.coeff_weight_assistance << endl;
            }
            else if(input_char=='+'){
                app.coeff_weight_assistance += 0.05;
                if(app.coeff_weight_assistance>1) app.coeff_weight_assistance=1;
                cout << "Weight assistance : "<<  app.coeff_weight_assistance << endl;
            }
            else{
               cout << "Unknown command" << endl;
            }
            break;

        case 'a':  case 'A':
            while(kbhit()==false);
            input_char=getchar();
            if(input_char=='+'){
                app.coeff_allocation += 0.2;
                if(app.coeff_allocation>1) app.coeff_allocation=1;
                cout << "Motor allocation : "<<  app.coeff_allocation << endl;
            }
            else if(input_char=='-'){
                app.coeff_allocation -= 0.2;
                if(app.coeff_allocation<0) app.coeff_allocation=0;
                cout << "Motor allocation : "<<  app.coeff_allocation << endl;
            }
            else{
               cout << "Unknown command" << endl;
            }
            break;

        case 'q':  case 'Q':
            while(kbhit()==false);
            input_char=getchar();
            if(input_char=='+'){
                app.q_adjusted += 0.1;
                if(app.q_adjusted>1) app.q_adjusted=1;
                cout << "Charge : "<<  app.q_adjusted << endl;
            }
            else if(input_char=='-'){
                app.q_adjusted -= 0.1;
                if(app.q_adjusted<0) app.q_adjusted=0;
                cout << "Charge : "<<  app.q_adjusted << endl;
            }
            else{
               cout << "Unknown command" << endl;
            }
            break;
        case '7':
            //  INSERT USER DATA
            mode=7;
            break;
        case '8':
            // POSITION MODE
            mode=8;
            break;
        case '9':
            // ITERATIVE
            mode=9;
            break;
        case 'z':
            // se schiaccio z passo nella fase di stop della calibrazione
            mode=102;
            break;

        default:
            cout << red_key << "INVALID INPUT - stopping test"<< color_key << endl;
            //c.data->control_mode_command =robot_control_mode_t::standby;
            break;
        }
         //app.last_rehamove_mode = app.rehamove_mode;
        if(input_char!='+' && input_char!='-' && input_char!='d'  && input_char!='k' && input_char!='K'  && input_char!='D'
                 && input_char!='Q'  && input_char!='q'  && input_char!='A'  && input_char!='a'  && input_char!='W'  && input_char!='w'){
         app.rehamove_mode = mode;
         app.change_mode = true;
         }

    }


}


// Wait for keyboard interrupt
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
void onedof_rehamove_class::print_command_keys()
{
    std::cout   << ": Enter command for test"  << endl;
  std::cout << boldred_key << "\nCOMMAND KEYS:"<< color_key << std::endl;
  std::cout << blue_key << "\'0\'" << color_key << ": EXIT mode" << "\n";
  std::cout << blue_key << "\'1\'" << color_key << ": STOP mode"<< "\n";
  std::cout << blue_key << "\'2\'" << color_key << ": HOMING mode"<< "\n";


  std::cout << blue_key << "\'3\'" << color_key << ": FES CALIBRATION mode"<< "\n";
  std::cout << blue_key << "\'4\'" << color_key << ": WEIGHT ASSITANCE CALIBRATION mode" << "\n";
  std::cout << blue_key << "\'5\'" << color_key << ": 100% MOTOR ALLOCATION: THRESHOLD CALIBRATION" << "\n";

  std::cout << blue_key << "\'6\'" << color_key << ": IMPEDANCE mode + ALLOCATION ** PARAMETERS SETTING"<< "\n";
  std::cout << blue_key << "\'7\'" << color_key << ": INSERT USER DATA"<< "\n";
  std::cout << blue_key << "\'8\'" << color_key << ": POSITION mode"<< "\n";
  std::cout << blue_key << "\'9\'" << color_key << ": ITERATIVE mode"<< "\n";



  std::cout << green_key << "\'K +-\'" << color_key << ": Kd STIFFNESS"<< "\n";
  std::cout << green_key << "\'D +-\'" << color_key << ": DKd DAMPING"<< "\n";
  std::cout << green_key << "\'Q +-\'" << color_key << ": q CHARGE"<< "\n";
  std::cout << green_key << "\'W +-\'" << color_key << ": w WEIGTH assistance"<< "\n";
  std::cout << green_key << "\'A +-\'" << color_key << ": ALLOCATION"<< "\n";

  cout << endl;

}






