#include "esmacat_parameter.h"

std::string get_ethercat_adapter_name(){
    return ETHERCAT_ADAPTER_NAME;
}

unsigned long get_esmacat_default_loop_time_ns()
{
    return ESMACAT_DEFAULT_LOOP_TIME_NS;
}
