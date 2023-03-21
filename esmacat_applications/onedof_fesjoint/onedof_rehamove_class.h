#ifndef ONEDOF_REHAMOVE_INPUT_H
#define ONEDOF_REHAMOVE_INPUT_H

#include "headers.h"

#define DEFAULT_LOOP_TIME_NS 1000000L
#define DEFAULT_APP_DURATION_COUNTS 10000


// Text Color Identifiers
const string boldred_key = "\033[1;31m";
const string red_key = "\033[31m";
const string boldpurple_key = "\033[1;35m";
const string yellow_key = "\033[33m";
const string blue_key = "\033[36m";
const string green_key = "\033[32m";
const string color_key = "\033[0m";

using namespace  std;


class onedof_rehamove_class
{

private:
    static void *internal_nrt(void * This) {((onedof_rehamove_class *)This)->nrt_thread(); return nullptr;}
    static void *internal_rt(void * This) {((onedof_rehamove_class *)This)->rt_thread(); return nullptr;}
    static void getinfo();

    // Pthreads variables
    pthread_t pthread_nrt_input;
    pthread_t pthread_rt_rehamove;

public:

    onedof_rehamove_app app;


    onedof_rehamove_class();
    ~onedof_rehamove_class();
    bool start_thread();
    bool start_rt_thread();
    void stop_rt_thread();
    void nrt_thread();
    void rt_thread();
    void stop_thread();
    void join_thread();
    void join_rt_thread();
    void interface();
    void print_command_keys();


};

#endif // ONEDOF_REHAMOVE_INPUT_H
