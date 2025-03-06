/**
 * @author Yehan Edirisinghe
 * @date 2021-06-06
 * 
 * @file main.cpp
 * @brief This file contains the main code for the ESP32 microcontroller.
 * It reads data from the lidar sensor and sends it to the PC via serial connection.
 * The data is sent in JSON format.
 * 
 */


// specifications for UART data trasmission
#define RXPIN 3 // RX pin for lidar (CHANGE IT)
#define TXPIN 18  // TX pin for lidar (not used here)
#define LIDAR_BAUDRATE 115200 //transmission speed for lidar (fixed, don't change)
#define BAUDRATE 460800       //transmission speed for pc serial connection

#define START_BIT 0xFA  // start bit for lidar data packet
#define BUFFER_SIZE 22  // size of the data packet


#include <Arduino.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <fstream>
#include <string.h>

#include "LDS01RR.h"


// Lidar serial object on USART 1
HardwareSerial Lidar(1);

// reading variables
int buffer_index = 0;
unsigned char temp, raw_data_curr, raw_data_old;
unsigned char packet[BUFFER_SIZE];

// JSON variables
JsonDocument jdoc;
JsonArray data = jdoc["data"].to<JsonArray>();
JsonArray strength = jdoc["str"].to<JsonArray>();

void setup(){
  // initialise serial comunication
  Serial.begin(BAUDRATE);
  Serial.println();

  // ensure TXPIN is only for receiving data
  pinMode(TXPIN, INPUT);
  // initialise lidar serial communication
  Lidar.begin(LIDAR_BAUDRATE, SERIAL_8N1, RXPIN, TXPIN);
}


void loop() {

  if(Lidar.read(&temp, 1) > 0){
    
    raw_data_old = raw_data_curr;
    raw_data_curr = temp;

    if(raw_data_old == START_BIT && raw_data_curr <= 0xf9 && raw_data_curr >= 0xa0){

      if(verify_packet_checksum(packet)){
        
        jdoc["ang"] = angle(packet);
        // jdoc["rpm"] = rpm(packet);
        data.add(dist_mm(packet+4));
        data.add(dist_mm(packet+8));
        data.add(dist_mm(packet+12));
        data.add(dist_mm(packet+16));

        strength.add(signal_strength(packet+6));
        strength.add(signal_strength(packet+10));
        strength.add(signal_strength(packet+14));
        strength.add(signal_strength(packet+18));

        serializeJson(jdoc, Serial);
        Serial.println();

        data.clear();
        strength.clear();

        // dump_packet(packet);
      }

      packet[0] = START_BIT;
      packet[1] = raw_data_curr;
      buffer_index = 2;

      return;
    }

    if(buffer_index > BUFFER_SIZE){
      buffer_index = 0;
      return;
    }

    packet[buffer_index] = raw_data_curr;
    buffer_index++;
  
  }
}