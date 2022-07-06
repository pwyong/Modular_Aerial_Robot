//Arduino Mega based 6 channel R/C Reader + Signal Writer to 6 ESCs
//Modified to
//Arduino Due based 6 channel R/C Reader + Signal Writer to 6 ESCs
//Modified to
//Arduino Due based 6 channel R/C Reader + Signal Writer to 4 ESCs for T3_Multirotor
//Modified for
//ROS competible system
//Modified for 
//Arduino Due based 8 channel Sbus + Signal Writer to 4 ESCs for T4_Multirotor
//Modified for 
//Arduino Due based 8 channel Sbus Receiver

//reference_publish:  http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World
//reference_subscribe:http://wiki.ros.org/rosserial_arduino/Tutorials/Blink
//reference_array:    http://wiki.ros.org/rosserial_arduino/Tutorials/Servo%20Controller

//To interact w. ROS on computer, run rosrun rosserial_python serial_node.py /dev/"@devicename@"


// Update Logs.
// 2016.05.04 First build
// 2016.05.11 Changed SW functions ex)calcSW
// 2017.01.18 Code modified for Arduino Due
// 2018.08.09 Code modified to operate under the ROS system w. rosserial
// 2018.08.15 Remapped the analogwrite->PWM relationship
// 2022.06.24 Applied voltage sensor

// Seung Jae Lee
// Seoul National University
// Department of Mechanical & Aerospace Engineering

// Loop Frequency : around 500 Hz
// Generated PWM Frequency : 455.846 Hz

// For ROS==============================================================================
#define USE_USBCON //use the following line if you have an arduino w. native port

#include <ros.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
//--------------------------------------------------------------------------------------

#if defined(ARDUINO) && ARDUINO >=100
#include "Arduino.h"
#else
#include <WProgram.h>
#endif

// For ROS==============================================================================
ros::NodeHandle nh;

std_msgs::Int16MultiArray sbus_msg;
std_msgs::Int16 voltage_msg;
ros::Publisher sbus("sbus",&sbus_msg);
ros::Publisher battery("battery",&voltage_msg);
//--------------------------------------------------------------------------------------

// Set S.BUS=============================================================
#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03
//-----------------------------------------------------------------------
// Battery Voltage Check=================================================
#define BATTERY_V_PIN            A11

uint8_t sbus_data[25] = {0x0f,0x01,0x04,0x20,0x00,0xff,0x07,0x40,0x00,0x02,0x10,0x80,0x2c,0x64,0x21,0x0b,0x59,0x08,0x40,0x00,0x02,0x10,0x80,0x00,0x00};
int16_t channels[18]  = {1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,0,0};
uint8_t  failsafe_status = SBUS_SIGNAL_FAILSAFE;

uint8_t failsafe(void) {return failsafe_status;}
//-------------------------------------------------------------------------------------


//for Read S.BUS data===========================================================
int16_t channel(uint8_t ch) {
    // Read channel data
    if ((ch>0)&&(ch<=16)){
        return channels[ch-1];
    }else{
        return 1023;
    }
}

 

void update_channels(void) {
    // Read all received data and calculate channel data
    uint8_t i;
    uint8_t sbus_pointer = 0;
    while (Serial1.available()) {
        uint8_t data = Serial1.read(); // get data from serial rx buffer
        switch (sbus_pointer) {
            case 0: // Byte 1
                if (data==0x0f) {
                    sbus_data[sbus_pointer] = data;
                    sbus_pointer++;
                }
                break;
            case 24:    // Byte 25 >> if last byte == 0x00 >> convert data
                if (data==0x00) {
                    sbus_data[sbus_pointer] = data;
                    // clear channels[]
                    for (i=0; i<16; i++) {channels[i] = 0;}
 
                    // reset counters
                    uint8_t byte_in_sbus = 1;
                    uint8_t bit_in_sbus = 0;
                    uint8_t ch = 0;
                    uint8_t bit_in_channel = 0;
 
                    // process actual sbus data
                    for (i=0; i<176; i++) {
                        if (sbus_data[byte_in_sbus] & (1<<bit_in_sbus)) {
                            channels[ch] |= (1<<bit_in_channel);
                        }
                        bit_in_sbus++;
                        bit_in_channel++;
 
                        if (bit_in_sbus == 8) {
                            bit_in_sbus =0;
                            byte_in_sbus++;
                        }
                        if (bit_in_channel == 11) {
                            bit_in_channel =0;
                            ch++;
                        }
                    }
                    // DigiChannel 1
                    if (sbus_data[23] & (1<<0)) {
                        channels[16] = 1;
                    }else{
                        channels[16] = 0;
                    }
                    // DigiChannel 2
                    if (sbus_data[23] & (1<<1)) {
                        channels[17] = 1;
                    }else{
                        channels[17] = 0;
                    }
                    // Failsafe
                    failsafe_status = SBUS_SIGNAL_OK;
                    if (sbus_data[23] & (1<<2)) {
                        failsafe_status = SBUS_SIGNAL_LOST;
                    }
                    if (sbus_data[23] & (1<<3)) {
                        failsafe_status = SBUS_SIGNAL_FAILSAFE;
                    }
                }
                break;
 
            default:  // collect Channel data (11bit) / Failsafe information
                sbus_data[sbus_pointer] = data;
                sbus_pointer++;
        }
    }
}
//------------------------------------------------------------------------------------------

void setup() {
  // For ROS========================================================================
  nh.getHardware()->setBaud(57600);
//  Serial.begin(250000);
  Serial1.begin(100000,SERIAL_8E2);
  pinMode(BATTERY_V_PIN,INPUT);
  
  sbus_msg.data=(short int *)malloc(sizeof(short int) * 9);
  sbus_msg.data_length = 9;
  nh.initNode();
  nh.advertise(sbus);
  nh.advertise(battery);
  //--------------------------------------------------------------------------------
  analogReadResolution(12);
  analogWriteResolution(12);
}


int32_t sbus_time=0, update_time=0,start_time=0;
void loop() {
  start_time = micros();
  
  update_channels();
  sbus_msg.data[0] = channel(1);
  sbus_msg.data[1] = channel(2);
  sbus_msg.data[2] = channel(3);
  sbus_msg.data[3] = channel(4);
  sbus_msg.data[4] = channel(5);
  sbus_msg.data[5] = channel(6);
  sbus_msg.data[6] = channel(7);
  sbus_msg.data[7] = channel(8);
  sbus_msg.data[8] = channel(10);

  voltage_msg.data = analogRead(BATTERY_V_PIN);
  sbus.publish(&sbus_msg);
  battery.publish(&voltage_msg);
  nh.spinOnce();
  
  while(micros()-start_time<20000);
}
