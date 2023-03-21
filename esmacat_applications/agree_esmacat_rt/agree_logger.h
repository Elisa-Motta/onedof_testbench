#ifndef ONEDOF_LOGGER_H
#define ONEDOF_LOGGER_H

#include "agree_shared_memory_comm.h"
//#include "harmony_common.h"
#include "csv_handling.h"

namespace logger{

std::string create_header(std::vector<std::string> joints_labels);
std::string create_dataline(const agree_shared_memory_comm& comm, const double elapsed_time);
}

#endif // ONEDOF_LOGGER_H
