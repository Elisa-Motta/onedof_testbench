#ifndef ERROR_LOGGER_H
#define ERROR_LOGGER_H

#include <inttypes.h>
#include <string>
#include <vector>
#include <iostream>

struct error_code_description
{
    uint32_t error_code;
    std::string err_description;
};

class error_logger
{
    std::vector <error_code_description> error_list;
public:
    void push(uint32_t err_code, std::string err_str);
    bool error_found();
    void print_error();
    error_logger();
};

#endif // ERROR_LOGGER_H
