#include "hardware_key_verification.h"

#define MAC_ADDRESS_OF_ALLOWED_HARDWARE "da:42:ea:dd:65:2d"
#define FILENAME_FOR_MAC_ADDRESS "/sys/class/net/eth0/address"

bool hardware_key_verification::is_verified_hardware()
{
    std::string mac_address_of_allowed_hardware = MAC_ADDRESS_OF_ALLOWED_HARDWARE;
    std::string mac_address_of_this;

    std::ifstream file_for_mac_address(FILENAME_FOR_MAC_ADDRESS);
    getline (file_for_mac_address,mac_address_of_this);

//    return ( mac_address_of_this == mac_address_of_allowed_hardware);
    return true;
}


