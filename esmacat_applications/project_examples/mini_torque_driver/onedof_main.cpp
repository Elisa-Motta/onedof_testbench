/** @file
 * @brief This application shows a minimal example of one dof joint control using Joint_controller class.
 *
*/

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/
#include <iostream>
#include "onedof_app.h"

using namespace std;

int main(){
    //this is defined in my_app.cpp and my_app.h
    static plog::RollingFileAppender<plog::CsvFormatter> fileAppender("esmacat_log.csv", 80000, 10); // Create the 1st appender.
    static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender; // Create the 2nd appender.

    plog::init(plog::warning, &fileAppender).addAppender(&consoleAppender); // Initialize the logger with the both appenders.
    onedof_app app;
    // set the cycle time in ns
    app.set_one_cycle_time_ns(1000000L);

    // start the esmacat application customized for your slave
    app.start();

    //the application runs as long as the esmacat master and slave are in communication
    while (app.is_esmacat_app_closed() == false );
    app.close();
    return 0;
}

