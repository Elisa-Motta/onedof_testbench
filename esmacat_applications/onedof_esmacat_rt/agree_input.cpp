/** @file
 * @brief Contains definitions of functions used for the primary executable of Harmony SHR
 * User Input
 *
*/
/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include "onedof_shared_memory_comm.h"

#include <sys/ioctl.h>
#include <termios.h>
#include "application.h"

// Text Color Identifiers
const string boldred_key = "\033[1;31m";
const string red_key = "\033[31m";
const string boldpurple_key = "\033[1;35m";
const string yellow_key = "\033[33m";
const string blue_key = "\033[36m";
const string green_key = "\033[32m";
const string color_key = "\033[0m";

using namespace  std;
bool kbhit();
void print_command_keys();


/*****************************************************************************************
 * FUNCTIONS
 ****************************************************************************************/
int main()
{
    static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender; //include application.h to get the plog libraries

    plog::init(plog::info, &consoleAppender); // Initialize the logger
    onedof_shared_memory_comm c;

    // Initializing the shared memory
    if (c.init())
    {
        PLOGI << "User Interface shared memory initialized with key: " << hex << c.get_shared_memory_key();    // start the shared memory communication
    }
    else
    {
        PLOGE << "User Interface shared memory initialization has been failed";
        c.detach_shared_memory();
        return 0;
    }

    char input_char = '0';
    std::string mode_name; //This section should ideally be done with an enum

    //c.restore_default_values();
    print_command_keys();


    while( c.data->stop == 0)
    {
        if(kbhit())
        {
            cin >> input_char;

            switch (input_char)
            {
            case ' ':
                print_command_keys();
                break;
            case '0':
                c.data->control_mode_command = robot_control_mode_t::quit;
                c.data->stop = true;
                break;
            case '1':  case '2': case '3': case '4': case '5': case '6': case '7': case '8':  case '9':
                cout << ": Running " << robot_mode_labels[stoi(&input_char)] << " mode!" << endl;
                c.data->control_mode_command = static_cast<robot_control_mode_t>(stoi(&input_char));
                break;

            case 'l': case 'L':
                cout << ": Running Low Torque Mode" << endl;
                c.data->control_mode_command = robot_control_mode_t::torque_control;
                c.data->impedance_control_command->impedance_control_feedforward_torque_mNm = 250.0;
                break;
            case 'm': case 'M':
                cout << ": Running Mid Torque Mode" << endl;
                c.data->control_mode_command = robot_control_mode_t::torque_control;
                c.data->impedance_control_command->impedance_control_feedforward_torque_mNm = 500.0;
                c.data->impedance_control_command->impedance_control_k_gain_mNm_per_rad         = 3.3;
                c.data->impedance_control_command->impedance_control_d_gain_mNm_per_rad_per_sec = 6.6;
                c.data->impedance_control_command->impedance_control_setpoint_rad               = 9.9;
                break;
            case 'h': case 'H':
                cout << ": Running High Torque Mode" << endl;
                c.data->control_mode_command = robot_control_mode_t::torque_control;
                c.data->impedance_control_command->impedance_control_feedforward_torque_mNm = 1000.0;
                break;
            default:
                cout << "Invalid Input - stopping test"<< endl;
                c.data->control_mode_command =robot_control_mode_t::quit;
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
