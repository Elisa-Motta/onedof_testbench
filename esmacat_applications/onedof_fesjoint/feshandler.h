#pragma once

#include "singlepulse.h"


extern "C"{
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <linux/serial.h>
}
#include<math.h>
#include<iostream>
#include <algorithm>
#include<string.h>
#include <sstream>
#include <bitset>
#include <iomanip>
#include<thread>
#include<chrono>


        enum MsgType{
            pulse_by_pulse=8,
            train_of_pulse_start=2,
            train_of_pulse_stop =3,
        };
        enum HighVoltage{
             On=1,
             Off=0,
             No_charge=2,
        };


        const uint8_t NUMBER_OF_CHANNELS = 8;/*FES DEVICE HAS 8 CHANNELS*/
        const uint8_t T_BUFFER_SIZE = 35;/*WE SEND 35 BYTES OF DATA TO FES DEVICE*/
        const uint8_t R_BUFFER_SIZE = 16;/*FES RESPONDS BACK WITH 16 BYTES OF DATA*/
        using channelid = uint8_t; //qui definisco l'ID
        using byte = float;
        using hexabyte = std::string;

        struct channel{                     //EXA!!!
            byte channel_id;
            hexabyte msg_type;
            hexabyte delay;
            hexabyte period;
            hexabyte power_modulation;
            hexabyte stim_value;
            hexabyte pulse_width;
            hexabyte pre_scaler;
            bool doublet;
            hexabyte double_delay;
            hexabyte sensor_input;
            hexabyte high_voltage;
            hexabyte checksum;

        };

        struct external_channel{            //BYTE!!

            std::array<byte,NUMBER_OF_CHANNELS> channel_id;
            //byte msg_type;
            byte msg_type;
//            MsgType msg_type;
            std::array<byte,NUMBER_OF_CHANNELS> delay;
            byte period;
            std::array<byte,NUMBER_OF_CHANNELS> power_modulation;
            std::array<byte,NUMBER_OF_CHANNELS> stim_value;
            std::array<byte,NUMBER_OF_CHANNELS> pulse_width;
            std::array<byte,NUMBER_OF_CHANNELS> pre_scaler;
            std::array<bool,NUMBER_OF_CHANNELS> doublet;
            std::array<byte,NUMBER_OF_CHANNELS> double_delay;
            byte sensor_input;
            byte high_voltage;
            byte checksum;
            std::array<bool,NUMBER_OF_CHANNELS> active;
            std::array<byte,NUMBER_OF_CHANNELS> min_stim;
            std::array<byte,NUMBER_OF_CHANNELS> max_stim;

        };
        using fes_channels = channel[NUMBER_OF_CHANNELS];
        using fes_state = external_channel;
        //using fes_state = channel;


        class FesSerial{
        public:

            bool configTermios();
            std::string getSerialAddress()const;
            void closeConnection();

            std::string decimalToHexaString(byte n);

            void createChannelTrajectory();
            void setChannels();
            void createMsg();
            void sendMsg();


//        private:

            int baud_rate_;
            int file_descriptor_;
            struct termios termiosconfig,termios_save_;
            char *send_buffer_;
            int sent_bytes_;
            const char *pos_;
            byte delay_;
            //byte period_;
            byte period_;
            byte power_modulation_;
            byte doublet_delay_;
            char serial_address_[1024] = "/dev/ttyUSB0";
            bool doublets_[NUMBER_OF_CHANNELS];
            fes_channels channels_;
            MsgType msg_type_;
            hexabyte msg_length_;
            byte stim_values_[NUMBER_OF_CHANNELS];
            byte pulse_widths_[NUMBER_OF_CHANNELS];
            byte prescalers_[NUMBER_OF_CHANNELS];
            hexabyte doublethexa_;
            HighVoltage high_voltage_;
            hexabyte sensor_input_hex_;
            byte sensor_input_;
            hexabyte checksum_;
            hexabyte whole_msg_;
            hexabyte msg_type_hexa_;
            hexabyte high_volt_hexa_;
            hexabyte train_start_msg_;
            hexabyte train_stop_msg_;
            fes_state fes_State;

        };//Fesserial class


       class FesMonitor{
        public:
            bool startStim();
            bool initStim = false;

        };
