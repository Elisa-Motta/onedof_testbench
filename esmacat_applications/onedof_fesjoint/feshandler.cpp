#include "feshandler.h"
#include "singlepulse.h"

#include <stdint.h>
#include <stdio.h>
#include <termios.h>
#include <iomanip>

#include <pthread.h>

#include "signal.h"

//#define STIM_VALUE 30
//#define PULSE_WIDTH 40
//#define PERIOD 40


using namespace std;

//#define DEFAULT_LOOP_TIME_NS 1000000L

//int fesT=1000;
//double durationFES = 8000;

//bool runningFES =true;
//bool stimulationON2 = true;

//using namespace fes;
//using namespace fes_communication;
//using namespace fes::fes_communication;

bool FesSerial::configTermios(){
    bool ret;

    //closeConnection();
    file_descriptor_ = open(serial_address_, O_RDWR | O_NOCTTY | O_NDELAY);
    //file_descriptor_ = -1;

    if(file_descriptor_ == -1){
        //printf("\n %s  problem with Opening\n",serial_address_);
        return false;
    }

    else{
        //printf("\n %s Opened Successfully: file descriptor value %d \n",serial_address_,file_descriptor_);
        ret = true;
    }

    tcgetattr(file_descriptor_, &termiosconfig);
    memcpy(&termios_save_, &termiosconfig, sizeof(struct termios));

    termiosconfig.c_cflag = B115200;
    termiosconfig.c_cflag |=  CS8;
    termiosconfig.c_cflag &= ~PARENB;
    termiosconfig.c_cflag |= CLOCAL;
    termiosconfig.c_oflag = 0;
    termiosconfig.c_lflag = 0;
    termiosconfig.c_cc[VTIME] = 0;
    termiosconfig.c_cc[VMIN] =  1;
    tcsetattr(file_descriptor_, TCSANOW, &termiosconfig);
    tcflush(file_descriptor_, TCOFLUSH);
    tcflush(file_descriptor_, TCIFLUSH);
    int RTS_flag,DTR_flag;
    RTS_flag = TIOCM_RTS;
    DTR_flag = TIOCM_DTR;
    ioctl(file_descriptor_,TIOCMSET,&RTS_flag);
    ioctl(file_descriptor_,TIOCMSET,&DTR_flag);


    return ret;
}

std::string FesSerial::getSerialAddress()const{
    std::string res(serial_address_);
    return res;
}


void FesSerial::closeConnection(){
    if(file_descriptor_!=-1){
        //tcsetattr(file_descriptor_, TCSANOW, &termios_save_);
            tcflush(file_descriptor_, TCOFLUSH);
            tcflush(file_descriptor_, TCIFLUSH);
        close(file_descriptor_);
        file_descriptor_=-1;
    }



}


void FesSerial::createChannelTrajectory(){  //ho preso multiplechannels!

    for(int i=0;i<NUMBER_OF_CHANNELS;i++){
        if(fes_State.active[i]){
            fes_State.msg_type = pulse_by_pulse ;
            fes_State.period= 25;
            fes_State.high_voltage= 1;

            fes_State.doublet[i] = false;
            fes_State.double_delay[i] = 30; //30 * 100 uSec
            fes_State.delay[i] = 0;
            fes_State.sensor_input = 0;
            fes_State.power_modulation[i] = 100;

            //fes_State.stim_value[i] = 7;
            fes_State.pre_scaler[i]=1;
            //fes_State.pulse_width[i] = 400; //400us di PW in multiple of 10us, viene diviso poi
        }
        else {
            fes_State.msg_type = pulse_by_pulse ;
            fes_State.period= 25;  //25 ms--> 40 Hz          --> MESSI ALL'ESTERNO ALL'INIZIO
            fes_State.high_voltage= 1;
            fes_State.doublet[i] = false;
            fes_State.double_delay[i] = 30; //30 * 100 uSec
            fes_State.delay[i] = 0;
            fes_State.sensor_input = 0;
            fes_State.power_modulation[i] = 100;

            fes_State.stim_value[i] = 0;
            fes_State.pre_scaler[i]=1;
            fes_State.pulse_width[i] = 0; //400us di PW in multiple of 10us, viene diviso poi
        }

//        std::cout << "\ramp (dec) " << fes_State.stim_value[0] << "  "<< endl;
//        std::cout <<  "\rPW (dec)" << fes_State.pulse_width[0] << "  " << endl;
//        std::cout << "\rPERIOD (dec)" << fes_State.period << "  " << endl;
        //cout << "\rmsg type" << fesState->msg_type << endl;
}
    //std::cout << "\ramp (dec) " << fes_State.stim_value[0] << "  "<< endl;
    //std::cout <<  "\rPW (dec)" << fes_State.pulse_width[0] << "  " << endl;
    //std::cout << "\rPERIOD (dec)" << fes_State.period << "  " << endl;
}

std::string FesSerial::decimalToHexaString(byte n)
{
    std::stringstream stream;
    stream<<std::setfill('0')<<std::setw(2)<<std::uppercase<<std::hex<<static_cast<int>(n);
    std::string result(stream.str());
    return result;
}

void FesSerial::setChannels(){
    msg_type_ = static_cast<MsgType>(fes_State.msg_type);
    if(msg_type_==MsgType::pulse_by_pulse){
        //msg_length_=22;
        msg_length_ = decimalToHexaString(34);
        msg_type_hexa_ = decimalToHexaString(8);
        //std::cout<<"\rmsg_length_: "<<msg_length_<<std::endl;
    }
    else {
        //msg_length_ = 03;
        msg_length_ = decimalToHexaString(3);
        if(msg_type_==MsgType::train_of_pulse_start){
            msg_type_hexa_ = decimalToHexaString(2);
        }
        else {
            msg_type_hexa_ = decimalToHexaString(3);
        }
    }
    high_volt_hexa_ = decimalToHexaString(fes_State.high_voltage);
    byte doublevalue = 0;
    sensor_input_ = fes_State.sensor_input;
    for(std::size_t i=0;i<NUMBER_OF_CHANNELS;i++){

        channels_[i].stim_value = decimalToHexaString(fes_State.stim_value[i]);
        //convert pulse width valuesstim_value
        channels_[i].pulse_width = decimalToHexaString(std::min((int)fes_State.pulse_width[i]/10,80));
        //convert prescaler values
        channels_[i].pre_scaler = decimalToHexaString(fes_State.pre_scaler[i]);
        //convert power modulation values
        channels_[i].power_modulation = decimalToHexaString(fes_State.power_modulation[i]);
        //convert period values
        channels_[i].period = decimalToHexaString(fes_State.period);
        //cout << "\nPeriod era " << channels[0].period << "   ora è HEXA " << channels_[0].period << "  " << endl;

        //convert delay values
        channels_[i].delay = decimalToHexaString(fes_State.delay[i]);
        //convert doublet delay values
        channels_[i].double_delay = decimalToHexaString(doublet_delay_);
        channels_[i].doublet = fes_State.doublet[i];
        if(channels_[i].doublet){
            doublevalue += pow(2,i);
        }
    }
    doublethexa_ = decimalToHexaString(doublevalue);

    sensor_input_hex_ =  decimalToHexaString(sensor_input_);

    //cout << "\ramp (exa) " << channels_[0].stim_value << "  "<< endl;
    //cout <<  "\rPW (exa)" << channels_[0].pulse_width << "  " << endl;
    //cout << "\rPERIOD (exa)" << channels_[0].period << "  " << endl;

}



void FesSerial::createMsg(){
    whole_msg_ = "FF";
   // std::cout<<"whole_msg_: "<<whole_msg_<<std::endl;
    whole_msg_ += msg_length_;
    //std::cout<<"msg_length_: "<<whole_msg_<<std::endl;
    whole_msg_ += msg_type_hexa_;
    //std::cout<<"msg_type_: "<<whole_msg_<<std::endl;
    whole_msg_ += channels_[0].delay;
    //std::cout<<"whole_msg_: "<<whole_msg_<<std::endl;
    whole_msg_ += channels_[0].period;
   // std::cout<<"whole_msg_: "<<whole_msg_<<std::endl;
    whole_msg_ += channels_[0].power_modulation;
    //std::cout<<"whole_msg_: "<<whole_msg_<<std::endl;
    for(std::size_t i=0;i<NUMBER_OF_CHANNELS;i++){
        whole_msg_ += channels_[i].stim_value;
    }
    for(std::size_t i=0;i<NUMBER_OF_CHANNELS;i++){
        whole_msg_ += channels_[i].pulse_width;
    }
    for(std::size_t i=0;i<NUMBER_OF_CHANNELS;i++){
        whole_msg_ += channels_[i].pre_scaler;
    }
    whole_msg_ += doublethexa_;
    whole_msg_ += channels_[0].double_delay;
    whole_msg_ += sensor_input_hex_;
    whole_msg_ += high_volt_hexa_;
    whole_msg_ += checksum_;
    //std::cout<<"\rCreated Msg: "<<whole_msg_<<std::endl;
    sendMsg();

}

void FesSerial::sendMsg(){
  //std::cout<<"\rLength of string: "<<whole_msg_.length()<<std::endl;

  const char *pos = whole_msg_.c_str();
   unsigned char val[35];
   for (std::size_t count = 0; count < sizeof val/sizeof *val; count++) {
       sscanf(pos, "%02hhX", &val[count]);
       //printf("\r%02hhX; ",val[count]);
       pos += 2;
   }


   int sschecksum =0;
   for (unsigned int count = 1; count < T_BUFFER_SIZE-1; count++) {

       sschecksum += val[count];


   }
   char  sum = sschecksum & 0x7f;

//   printf("\n\rchecksum final is:, %02hhX\n ",sum);
//   printf("\n\rvalue[34] before is : %02hhX",val[34]);
   val[34] = sum;

//   printf("\n\rvalue[34] after is : %02hhX",val[34]);
//   printf("\n\r");


   int  bytes_written  = 0;
   bytes_written = write(file_descriptor_,val,sizeof(val));
   //for(int i=0;i<bytes_written;i++)
       //printf("%02hhX;",val[i]);

   //printf("\n\r%dBytes written to ttyUSB0", bytes_written);
   //printf("\n\r+----------------------------------+\n\n");

}




bool FesMonitor::startStim(){
    string mystr;
    int onset;
    //cout << "Enter 1 for starting stimulation" << endl;
    getline (cin,mystr);
    stringstream(mystr)>>onset;
    //cout << onset;

    if (onset==1)
    {
        initStim=true;
    }
    else
    {
         initStim=false;
    }

    return initStim;
}


