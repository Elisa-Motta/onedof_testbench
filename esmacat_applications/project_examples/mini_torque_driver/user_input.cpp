/** @file
 * @brief Contains definitions of functions used for the primary executable of Harmony SHR
 * User Input
 *
*/
/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include <iostream>
#include <thread>
#include <chrono>
#include "shared_memory_comm.h"

#include <sys/ioctl.h>
#include <termios.h>
#include "application.h"


using namespace  std;
bool kbhit();


/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/
int main()
{
    static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender; //include application.h to get the plog libraries

    plog::init(plog::info, &consoleAppender); // Initialize the logger
    esmacat_shared_memory_comm c;

    // Initializing the shared memory
    if (c.init())
    {
        PLOGI << "User Interface shared memory initialized with key " << static_cast<int>(c.get_shared_memory_key());    // start the shared memory communication
    }
    else
    {
        PLOGE << "User Interface shared memory initialization has been failed";
        c.detach_shared_memory();
        return 0;
    }

    char input_char = '0';
    std::string mode_name; //This section should ideally be done with an enum


    std::cout << "\n\n\n\n"
         << "*************************************************************"
         << "\n\nEnter command for test"  << endl
         << "0 to Stop test"<<endl
         << "1 to run Zero torque Control"<<endl
         << "2 to run Impedance Control"<<endl
         << endl;

    while( c.data->stop == 0)
    {
        if(kbhit())
        {
            cin >> input_char;

            switch (input_char)
            {
            case '0':
                cout<<"\n\nStopping test\n\n";
                c.data->stop =1;
                break;
            case '1':
                cout << "\n Running Impedance Control\n";
                c.data->mode = static_cast<robot_control_mode_t>(1);
                break;
            case '2':
                cout << "\n Running Calibration\n";
                c.data->mode = static_cast<robot_control_mode_t>(2);
                break;
            default:
                cout<<"\n Invalid Input - stopping test\n\n";
                c.data->stop =1;
                break;
            }
        }
    }


    return 0;
}




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
