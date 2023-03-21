//#include "interface.h"

////#include <QCoreApplication>
////#include <QtCore/QCoreApplication>


//// HEADERS
//#include "smpt_ll_client.h"
//#include "smpt_client.h"
//#include "smpt_ml_client.h"
//#include "smpt_messages.h"
//#include "smpt_packet_number_generator.h"
//#include "smpt_ll_packet_validity.h"
//#include <iostream>
//#include <stdint.h>
//#include <ostream>
////#include <QtDebug>
//#include <thread>
//#include <chrono>
//#include <string>
//#include <stdio.h>
//using namespace std;
//#ifdef __unix__
//# include <unistd.h>
//#elif defined _WIN32
//# include <windows.h>
//#define sleep(x) Sleep(1000 * (x))
//#endif
//#include <time.h>
//#include <sys/time.h>
//#include <stdlib.h>
//#include <unistd.h>
//#include <pthread.h>
////#include <QtCore/QCoreApplication>
//#include <termios.h>
//#include <fcntl.h>
//int usleep(useconds_t usec);

//bool kbhit();
//void print_command_keys();


//#include <iostream>
//#include <string>
//#include <thread>
//#include <chrono>
//#include "onedof_shared_memory_comm.h"
//#include <sys/ioctl.h>
//#include <termios.h>


////mi serve "application.h" da includere per avere la libreria plog




//// Text Color Identifiers
//const string boldred_key = "\033[1;31m";
//const string red_key = "\033[31m";
//const string boldpurple_key = "\033[1;35m";
//const string yellow_key = "\033[33m";
//const string blue_key = "\033[36m";
//const string green_key = "\033[32m";
//const string color_key = "\033[0m";

//char input = '0';

//interface::interface()
//{
//    mStop=false;
//}

//void interface::run()
//{
//    onedof_shared_memory_comm c;


//    //--------initialize shared memory---------
//    //-----------------------------------------
//       // static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender; //include application.h to get the plog libraries

////        c.init();
////        //plog::init(plog::info, &consoleAppender);

////        if (c.init())
////        {
////            cout << "User Interface shared memory initialized with key: " << hex << c.get_shared_memory_key();    // start the shared memory communication
////        }
////        else
////        {
////            cout << "User Interface shared memory initialization has been failed";
////            c.detach_shared_memory();
////            return ;
////        }

////        char input_char = '0';
////        std::string mode_name; //This section should ideally be done with an enum

//   //  c.restore_default_values();
//    print_command_keys();

//    //-----------------------------------------
//    //-----------------------------------------





//    while(mStimulation.finish==false)
//    {
//        //mStimulation.start();




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
//                qDebug() << "CASE 0";
//                mStop=true;

//                mStimulation.mode=2;
//                qDebug() << "stop stimulation ";
//                break;

//            case '1':
//                cout << "ciao\n";
//                mStimulation.mode = 1;
//                cout << ": Running " << robot_mode_labels[11] << " mode!" << endl;
//                c.data->control_mode_command = static_cast<robot_control_mode_t>(11);
//                mStimulation.mode = 1;

//                break;

//            case '2':
//                cout << "\n";
//                cout << "Running controllo\n";

//                mStimulation.controllo();
//                mStimulation.torque();



//                break;

//            default:
//                cout << "Invalid Input - stopping test"<< endl;
//                c.data->control_mode_command =robot_control_mode_t::quit;
//                break;


//            }


//        }
//    }
//    return;
//}



//bool kbhit()
//{
//    termios term;
//    tcgetattr(0, &term);

//    termios term2 = term;
//    term2.c_lflag &= ~ICANON;
//    tcsetattr(0, TCSANOW, &term2);

//    int byteswaiting;
//    ioctl(0, FIONREAD, &byteswaiting);

//    tcsetattr(0, TCSANOW, &term);

//    return byteswaiting > 0;
//}


//// Print commands on terminal
//void print_command_keys()
//{
//    std::cout   << ": Enter command for test"  << endl;
//  std::cout << boldred_key << "\nCOMMAND KEYS:"<< color_key << std::endl;
//  std::cout << blue_key << "\'0\'" << color_key << ": EXIT" << "\n";
//  std::cout << blue_key << "\'1\'" << color_key << ": STOP mode"<< "\n";

//  std::cout << blue_key << "\'2\'" << color_key << ": CURRENT mode"<< "\n";
//  std::cout << blue_key << "\'3\'" << color_key << ": TORQUE mode" << "\n";
//  std::cout << blue_key << "\'4\'" << color_key << ": NULL-TORQUE mode" << "\n";

//  std::cout << blue_key << "\'5\'" << color_key << ": ANTI-G mode"<< "\n";
//  std::cout << blue_key << "\'6\'" << color_key << ": FREEZE mode"<< "\n";
//  std::cout << blue_key << "\'7\'" << color_key << ": IMPEDANCE mode"<< "\n";


////  std::cout << blue_key << "\'5\'" << color_key << ": FREEZE mode"<< "\n";
////  std::cout << blue_key << "\'6\'" << color_key << ": HOMING mode"<< "\n";
////  std::cout << blue_key << "\'7\'" << color_key << ": POSITION mode"<< "\n";

////  std::cout << blue_key << "\'t\'" << color_key << ": PASSIVE mode"<< "\n";
////  std::cout << blue_key << "\'i\'" << color_key << ": IMPEDANCE EXTERNAL mode"<< "\n";
////  std::cout << blue_key << "\'w\'" << color_key << ": ANTI-G mode"<< "\n";
////  std::cout << blue_key << "\'g\'" << color_key << ": TRANSPARENT mode"<< "\n";
////  std::cout << blue_key << "\'r\'" << color_key << ": RESISTIVE mode"<< "\n";
////  std::cout << blue_key << "\'c\'" << color_key << ": CHALLENGING mode"<< "\n";

////  cout << endl;
////  std::cout << blue_key << "\'d+\'" << color_key << ": increase DAMPING"<< "\n";
////  std::cout << blue_key << "\'d-\'" << color_key << ": decrease DAMPING"<< "\n";
////  std::cout << blue_key << "\'k+\'" << color_key << ": increase STIFFNESS"<< "\n";
////  std::cout << blue_key << "\'k-\'" << color_key << ": decrease STIFFNESS"<< "\n";
////  std::cout << blue_key << "\'a+\'" << color_key << ": increase WEIGHT assistance"<< "\n";
////  std::cout << blue_key << "\'a-\'" << color_key << ": decrease WEIGHT assistance"<< "\n";
//  cout << endl;
//  std::cout << blue_key << "\'M\'" << color_key << ": SHOW current settings and command keys\n"<< "\n";
//}




