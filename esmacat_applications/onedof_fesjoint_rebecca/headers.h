#ifndef HEADERS_H
#define HEADERS_H

#include "global_variables.h"
#include "application.h"
#include "onedof_shared_memory_comm.h"
#include "onedof_shared_memory_comm.h"
#include "onedof_rehamove_app.h"
#include "onedof_app.h"
#include "agree_joint_controller.h"
#include "onedof_app.h"
#include "ethercat_arduino_shield_by_esmacat.h"
#include "onedof_ease.h"
#include "time_spec_operation.h"
#include "loop_time_stats.h"


// Stimulation headers
#include "smpt_ll_client.h"
#include "smpt_client.h"
#include "smpt_ml_client.h"
#include "smpt_messages.h"
#include "smpt_packet_number_generator.h"
#include "smpt_ll_packet_validity.h"

#include <plog/Log.h>
#include <plog/Appenders/ColorConsoleAppender.h>

#include <sys/ioctl.h>
#include <termios.h>
#include <csignal>
using namespace std;
using namespace Eigen;

#include <iostream>
using std::cerr;
using std::endl;
#include <fstream>
using std::ofstream;
#include <cstdlib>
#include <math.h>
#include <string>
#include <thread>
#include <chrono>
#include <pthread.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <stdio.h>              // printf
#include <string.h>
#include <time.h>               // cyclic loop
#include <algorithm>            // std::copy
#include <signal.h>             // to catch ctrl-c
#include <unistd.h>
#include <cmath>
#include <matio.h>
#include <vector>
#include <utility> // std::pair
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream
#include <MATio/MATio.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>



#endif // HEADERS_H
