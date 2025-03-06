#include "LDS01RR.h"


void dump_packet(unsigned char* packet){
    for(int i=0; i<22; ++i){
        Serial.print(packet[i],HEX);
        Serial.print(" ");
    }
    Serial.println();
}

int angle(unsigned char *packet) { // 22 bytes in the packet
    return packet[1] - 0xA0; // 16 bits for the angle
}

float rpm(unsigned char *packet) { // 22 bytes in the packet
    return float(packet[2] | ((packet[3]<<8))) / 64.f;
}

bool verify_packet_checksum(unsigned char *packet) { // 22 bytes in the packet
    int checksum32 = 0;
    for (int i=0; i<10; i++)
    checksum32 = (checksum32<<1) + packet[2*i] + (packet[2*i+1]<<8);
    return packet[20]+(packet[21]<<8) == (((checksum32 & 0x7FFF) + (checksum32 >> 15)) & 0x7FFF);
}

// no return/max range/too low of reflectivity
bool invalid_data_flag(unsigned char *data) { // 4 bytes in the data buffer
    return (data[1] & 0x80) >> 7;
}

// object too close, possible poor reading due to proximity; kicks in at < 0.6m
bool strength_warning_flag(unsigned char *data) { // 4 bytes in the data buffer
    return (data[1] & 0x40) >> 6;
}

int dist_mm(unsigned char *data) { // 4 bytes in the data buffer
    return data[0] | (( data[1] & 0x3F) << 8); // 14 bits for the distance
}

int signal_strength(unsigned char *data) { // 4 bytes in the data buffer
    return data[2] | (data[3] << 8); // 16 bits for the signal strength
}
