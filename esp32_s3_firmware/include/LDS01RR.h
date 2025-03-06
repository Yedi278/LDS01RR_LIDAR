#ifndef LDS01RR_H
#define LDS01RR_H

#include <Arduino.h>
#include <HardwareSerial.h>

void dump_packet(unsigned char* packet);

/**
 * @brief Extracts the angle from the 22 byte packet
 * @details the angle is in degrees from 0 to 89
 * to get the actual angle multiply by 4
 */
int angle(unsigned char *packet);

/**
 * @brief Extracts the rpm from the 22 byte packet
 * @details the rpm is in float
 */
float rpm(unsigned char *packet);

/**
 * @brief Verifies the checksum of the packet
 * @details the checksum is the last 2 bytes of the packet
 */
bool verify_packet_checksum(unsigned char *packet);

/**
 * @brief Checks if the data is invalid
 * @details the data is invalid if the object is too close or the max range is reached 
 */
bool invalid_data_flag(unsigned char *data) { // 4 bytes in the data buffer
    return (data[1] & 0x80) >> 7;
}

/**
 * @brief Checks if the signal strength is too low
 */
bool strength_warning_flag(unsigned char *data) { // 4 bytes in the data buffer
    return (data[1] & 0x40) >> 6;
}

/**
 * @brief Extracts the distance in mm from the 4 byte data buffer
 * @details the distance is in mm
 * @warning if the data is invalid the distance is not reliable: use invalid_data_flag
 * @warning if the signal strength is too low the distance is not reliable
 */
int dist_mm(unsigned char *data) { // 4 bytes in the data buffer
    return data[0] | (( data[1] & 0x3F) << 8); // 14 bits for the distance
}

/**
 * @brief Extracts the signal strength from the 4 byte data buffer
 * @details the signal strength is in arbitrary units
 * @warning if the signal strength is too low the distance is not reliable
 */
int signal_strength(unsigned char *data) { // 4 bytes in the data buffer
    return data[2] | (data[3] << 8); // 16 bits for the signal strength
}

#endif // LDS01RR_H