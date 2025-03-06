# LDS01RR LIDAR DATA STRUCTURE

https://github.com/user-attachments/assets/adeb7e57-0af2-43ff-b6a1-c8d1a08f5a2c

This is a Lidar Sersor from a Roborock vacuum cleaner. In this page we try to unravel the working principles for the ones that would like to use it in their project.

Data is sent through UART transmission at 115200 baud rate.
The data is composed of hexadecimal values (unsigned char).

Lidar data are sent in 2 modes:
* Calibration initial mode
* Data Mode

## Calibration Mode:

the data sent is structured as:

...

## Data Mode

The data has structure:

|   | **HEADER**    | **ANGLE[Grad]**   | **RPM** | **DATA 1**     | **DATA 2**     | **DATA 2**     | **DATA 4**     | **CHECKSUM** |   |
|---|---------------|-------------------|---------|----------------|----------------|----------------|----------------|--------------|---|
|   | 1 byte        | 2 byte            | 2 byte  | 4 byte         | 4 byte         | 4 byte         | 4 byte         | 2 byte       |   |
|   | constant **0xfa** | from **0xa0** to **0xfa** | float   | Data Structure | Data Structure | Data Structure | Data Structure | int          |   |

### HEADER

The header is a constat value *0XFA* that is always sent at the start of a scan message

### ANGLE

The angle is in a range from 0 to 89 degrees (90 tot). Every scan message is composed of 4 scanned angles(360 degrees tot): 
taken **i** as the angle given by the message the actual angles measured are: {4***i**, 4***i**+1, 4***i**+2, 4***i**+3}

```c++
unsigned char packet[22];       // whole data packet from lidar
int angle = packet[1] - 0xa0;  // number from 0 to 89
int ang1 = angle*4;             // actual angles in degrees
int ang2 = angle*4 +1;
int ang3 = angle*4 +2;
int ang4 = angle*4 +3;
```

### RPM

The Lidar returns the rpm measured as a float value.

```c++
unsigned char packet[22];       // whole data packet from lidar
float rpm = float( packet[2] | ( packet[3] << 8 )) / 64.f;
```


### Data Strucure

The data is composed of a distance in mm, a signal strength and error correction data bits for data and strength:

**Data:**

| first 2 bytes | last 2 bytes |
| -- | -- |
| distance | signal strength |

```c++
unsigned char packet[22]; // whole data packet from lidar
// pointers to data packets of 4 bytes each
unsigned char* data_1 = packet+4;
unsigned char* data_2 = packet+8;
unsigned char* data_3 = packet+12;
unsigned char* data_4 = packet+16;

void get_dist(unsigned char* data){
    return data[0] | (( data[1] & 0x3F) << 8); // 14 bits for the distance
}

void get_strength(unsigned char* data){
    return data[2] | (data[3] << 8); // 16 bits for the signal strength
}
```

**Error Bits:**

```c++
// object too close, possible poor reading due to proximity; kicks in at < 0.6m
bool strength_warning_flag(unsigned char *data) { // 4 bytes in the data buffer
    return (data[1] & 0x40) >> 6;
}

// no return/max range/too low of reflectivity
bool invalid_data_flag(unsigned char *data) { // 4 bytes in the data buffer
    return (data[1] & 0x80) >> 7;
}
```

### CHECKSUM

At the end of every packet 2 bytes are reserved for the error correction:

```c++
bool verify_packet_checksum(unsigned char *packet) { // 22 bytes in the packet
    int checksum32 = 0;
    for (int i=0; i<10; i++)
    checksum32 = (checksum32<<1) + packet[2*i] + (packet[2*i+1]<<8);
    return packet[20]+(packet[21]<<8) == (((checksum32 & 0x7FFF) + (checksum32 >> 15)) & 0x7FFF);
}
```
