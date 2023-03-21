#ifndef SHARED_MEMORY_ECAT_H
#define SHARED_MEMORY_ECAT_H

#include <sys/ipc.h>
#include <sys/shm.h>
#include <time.h>
#include <inttypes.h>
#include "esmacat_parameter.h"

#define DEFAULT_KEY_ID  5700        // default key for shared memory

enum class ecat_flow_sequence{ ecat_app_off, ecat_app_on, ecat_setup_done, all_states_OP_arrived, cycle_running, wait_next_cycle, ecat_terminated};
enum class esmacat_flow_sequence{ esmacat_app_off, esmacat_app_on, esmacat_setup_done, one_cycle_period_set, shadow_copy_done_cycle_running, wait_next_cycle, esmacat_terminated};

class shared_memory_ecat
{
    struct ecat_slave{
        uint32_t vendor_id = 0;
        uint32_t eep_id = 0;
        uint32_t eep_rev = 0;
        uint8_t input_byte_size = 0;
        uint8_t output_byte_size = 0;
        uint16_t input_data_starting_index = 0;
        uint16_t output_data_starting_index = 0;
        char name[41];
    };

    struct shared_memory_packet {
        ecat_flow_sequence ecat_main_current_sequence= ecat_flow_sequence::ecat_app_off;
        esmacat_flow_sequence esmacat_main_current_sequence = esmacat_flow_sequence::esmacat_app_off;
        uint64_t esmacat_loop_cnt = 0;
        uint64_t ecat_loop_cnt = 0;
        struct timespec t_ref;
        unsigned long one_cycle_time_ns = get_esmacat_default_loop_time_ns();
        int number_of_ecat_slaves = 0;
        struct ecat_slave slave[MAX_NUMBER_OF_ETHERCAT_SLAVE];
        uint8_t ecat_input_data_stream[MAX_ECAT_INPUT_DATA_SIZE_BYTE];
        uint8_t ecat_output_data_stream[MAX_ECAT_OUTPUT_DATA_SIZE_BYTE];
    };

private:
    key_t key;
    int shmid = 0;
public:
    shared_memory_ecat();
    ~shared_memory_ecat();

    shared_memory_packet* data;
    bool init();
    key_t get_shared_memory_key(){return key;}
    void change_shared_memory_key(key_t k){key= k;} // only use this function before init
    void detach_shared_memory();
};

#endif // SHARED_MEMORY_ECAT_H
