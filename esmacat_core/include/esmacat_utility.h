#ifndef ESMACAT_UTILITY_H
#define ESMACAT_UTILITY_H

#include "time_spec_operation.h"
#include "loop_time_stats.h"
#include <plog/Log.h>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <math.h>

// flow_control waits until "current_status" arrives "arrive_here". When waiting time exceeds "wait_us", string "s" will be logged as an error
template <class T>
bool flow_control(T &current_status, T arrive_here, int wait_us, string s)
{
    struct timespec t_1us; t_1us.tv_sec = 0; t_1us.tv_nsec=1000;       // define  1us
    while( current_status < arrive_here ){
        clock_nanosleep ( CLOCK_MONOTONIC, 0, &t_1us, NULL );   // wait 1us for second app till it set up the t_ref;
        if (wait_us-- == 0) {
            PLOGE << s;
            PLOGE.printf("current status %d couldn't pass the desired status %d",static_cast<int>(current_status),static_cast<int>(arrive_here));
            return true;
        }
    }
    return false;
}

float sigmoid(float x);
float smooth_start_func_tanh(float x, float scale = 1.0);
float convert_ecat_format_to_float(uint16_t uint_number);
uint16_t convert_float_to_ecat_format(float float_number);
uint32_t convert_float_for_tx(float float_number);
float convert_rx_to_float(uint32_t uint_number);

#endif // ESMACAT_UTILITY_H

