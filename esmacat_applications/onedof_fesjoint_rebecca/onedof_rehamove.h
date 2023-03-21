#ifndef ONEDOF_REHAMOVE_H
#define ONEDOF_REHAMOVE_H

#endif // ONEDOF_REHAMOVE_H


/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/

#include "onedof_rehamove_class.h"
#include "onedof_rehamove_app.h"
#include "headers.h"

/*****************************************************************************************
 * REHAMOVE INCLUDES
 ****************************************************************************************/

#ifdef _POSIX_PRIORITY_SCHEDULING
#define POSIX "POSIX 1003.1b\n";
#endif
#ifdef _POSIX_THREADS
#ifdef _POSIX_THREAD_PRIORITY_SCHEDULING
#define POSIX "POSIX 1003.1c\n";
#endif
#endif

#define DEFAULT_LOOP_TIME_NS 1000000L
#define DEFAULT_APP_DURATION_COUNTS 10000
#define ALLOWED_LOOPTIME_OVERFLOW_NS 200000L

using namespace std;
using std::ofstream;
using std::cerr;
using std::endl;
