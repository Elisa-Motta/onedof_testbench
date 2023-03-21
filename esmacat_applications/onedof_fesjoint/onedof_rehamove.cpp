

#include "onedof_rehamove.h"
#include "onedof_rehamove_app.h"
#include "feshandler.h"
#include "stdio.h"

onedof_rehamove_app app;

// Rebecca: creo file da nominare con user ID inserito
extern char userID[10];

static void getinfo ()
{
    struct sched_param param;
    int policy;

    sched_getparam(0, &param);
    printf("Priority of this process: %d\n\r", param.sched_priority);

    pthread_getschedparam(pthread_self(), &policy, &param);

    printf("Priority of the thread: %d, current policy is: %d\n\r",
              param.sched_priority, policy);
}


int main()
{

    // Read reference torque for impedance control
    //app.read_ref_torque(100, app.torque_ref);

    //Print reference torque to check saving
    //    for (int i=0; i<100; i++){

    //        cout << app.torque_ref[i] << endl;
    //    }

    // Plogger initialization
    static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender; // Create appender.
    plog::init(plog::info,&consoleAppender);
    PLOGI << "Version " << POSIX;

    /*
    printf("Max priority for SCHED_OTHER %d\n", sched_get_priority_max(SCHED_OTHER));
    printf("Min priority for SCHED_OTHER %d\n", sched_get_priority_min(SCHED_OTHER));
    printf("Max priority for SCHED_FIFO %d\n", sched_get_priority_max(SCHED_FIFO));
    printf("Min priority for SCHED_FIFO %d\n", sched_get_priority_min(SCHED_FIFO));
    printf("Max priority for SCHED_RR %d\n", sched_get_priority_max(SCHED_RR));
    printf("Min priority for SCHED_RR %d\n", sched_get_priority_min(SCHED_RR));
    */
    PLOGW << "Main thread";
    getinfo();

    // Scheduler variables
    int policy;
    struct sched_param prio;
    pthread_attr_t attr;

    policy = SCHED_OTHER;
    if (pthread_setschedparam( pthread_self(),policy, &prio )){
            perror ("Error: check pthread_setschedparam (root permission?)");
            exit(1);
        }

    onedof_rehamove_class rehamove;
    onedof_rehamove_app app;

    // Open file with user ID
    cout << red_key << "Insert user ID: " << color_key;
    cin >> userID;
    cout << "your USER ID is: " << userID << "\n" ;


    char buffer [80];
    FILE *file;

    // Open user-specific calibration file
    snprintf (buffer,80,"/home/esmacat/esmacat_rt/onedof_log/onedof_subjects/calib_user_%s.csv", userID);

    if (file = fopen(buffer,"r")){
        fclose(file);
        cout << red_key << "USER calibration file exists" << color_key << endl;
    } else {
        cout << yellow_key << "USER calibration file does not exist" << color_key << endl;
        app.open_calibration_file();
        app.write_calibration_file();
        app.close_calibration_file();
    }

    app.open_rehamove_file();

    // Start threads (real-time and non-real-time)
    rehamove.start_thread();
    rehamove.start_rt_thread();

    // Threads need to join for clean quit
    rehamove.join_thread();
    rehamove.join_rt_thread();

    return 0;

}






































