//Arduino Mega based 6 channel R/C Reader + Signal Writer to 6 ESCs
//Modified to
//Arduino Due based 6 channel R/C Reader + Signal Writer to 6 ESCs
//Modified to
//Arduino Due based 6 channel R/C Reader + Signal Writer to 4 ESCs for T3_Multirotor
//Modified for
//ROS competible system
//Modified for
//Arduino Due based 8 channel Sbus + Signal Writer to 4 ESCs for T4_Multirotor

//reference_publish:  http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World
//reference_subscribe:http://wiki.ros.org/rosserial_arduino/Tutorials/Blink
//reference_array:    http://wiki.ros.org/rosserial_arduino/Tutorials/Servo%20Controller

//To interact w. ROS on computer, run rosrun rosserial_python serial_node.py /dev/"@devicename@"

// Arduino Due can change its frequency by modifying the varient.h file
// C:\Users\sjlaz\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.10\variants\arduino_due_x
// Reference:         https://folk.uio.no/jeanra/Microelectronics/GettingTheBestOutOfPWMDue.html
// What to change----------------------------------------------------------------------------------------
///*
// * PWM
// */
//#define PWM_INTERFACE    PWM
//#define PWM_INTERFACE_ID  ID_PWM
//#define PWM_FREQUENCY   456
//#define PWM_MAX_DUTY_CYCLE  1023
//#define PWM_MIN_DUTY_CYCLE  0
//#define PWM_RESOLUTION    10
//
///*
// * TC
// */
//#define TC_INTERFACE        TC0
//#define TC_INTERFACE_ID     ID_TC0
//#define TC_FREQUENCY        456
//#define TC_MAX_DUTY_CYCLE   1023
//#define TC_MIN_DUTY_CYCLE   0
//#define TC_RESOLUTION      10
//-------------------------------------------------------------------------------------------------------

// Update Logs.
// 2016.05.04 First build
// 2016.05.11 Changed SW functions ex)calcSW
// 2017.01.18 Code modified for Arduino Due
// 2018.08.09 Code modified to operate under the ROS system w. rosserial
// 2018.08.15 Remapped the analogwrite->PWM relationship

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

#include <Servo.h>

// For ROS==============================================================================
ros::NodeHandle nh;

std_msgs::Int16MultiArray sbus_msg;
ros::Publisher sbus("sbus", &sbus_msg);

std_msgs::Int16MultiArray pwm_msg;
ros::Publisher PWM_ard("PWM_ard", &pwm_msg);

std_msgs::Int16 loop_time;
ros::Publisher loop_timer("loop", &loop_time);

//--------------------------------------------------------------------------------------


// Assign channel out pins=============================================================
#define PROP_ONE_OUT_PIN 10
#define PROP_TWO_OUT_PIN 11
#define PROP_THR_OUT_PIN 12
#define PROP_FOU_OUT_PIN 13
//-------------------------------------------------------------------------------------

// Set S.BUS=============================================================
#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03

uint8_t sbus_data[25] = {0x0f, 0x01, 0x04, 0x20, 0x00, 0xff, 0x07, 0x40, 0x00, 0x02, 0x10, 0x80, 0x2c, 0x64, 0x21, 0x0b, 0x59, 0x08, 0x40, 0x00, 0x02, 0x10, 0x80, 0x00, 0x00};
int16_t channels[18]  = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 0, 0};
uint8_t  failsafe_status = SBUS_SIGNAL_FAILSAFE;

uint8_t failsafe(void) {
  return failsafe_status;
}
//-------------------------------------------------------------------------------------


//Input variables====================================================================
int u1 = 1000, u2 = 1000, u3 = 1000, u4 = 1000;
//-----------------------------------------------------------------------------------

//Write PWM signals to motor ESCs===========================================================

void writePWM(int ua1, int ua2, int ua3, int ua4) {

  ua1 = constrain( ua1, 1000, 2000 );
  ua2 = constrain( ua2, 1000, 2000 );
  ua3 = constrain( ua3, 1000, 2000 );
  ua4 = constrain( ua4, 1000, 2000 );

  //Write PWM Values to motor=============================================================
  //  analogWriteResolution(12);
  analogWrite(PROP_ONE_OUT_PIN, (int)(ua1 * 1.866891 + 0.868104));
  analogWrite(PROP_TWO_OUT_PIN, (int)(ua2 * 1.866891 + 0.868104));
  analogWrite(PROP_THR_OUT_PIN, (int)(ua3 * 1.866891 + 0.868104));
  analogWrite(PROP_FOU_OUT_PIN, (int)(ua4 * 1.866891 + 0.868104));
  //  analogWrite(PROP_ONE_OUT_PIN, (int)(1.022704 * (double)(ua1 + 2) + 1.120065));
  //  analogWrite(PROP_TWO_OUT_PIN, (int)(1.022704 * (double)(ua2 + 2) + 1.120065));
  //  analogWrite(PROP_THR_OUT_PIN, (int)(1.022704 * (double)(ua3 + 2) + 1.120065));
  //  analogWrite(PROP_FOU_OUT_PIN, (int)(1.022704 * (double)(ua4 + 2) + 1.120065));
  //--------------------------------------------------------------------------------------
  pwm_msg.data[0] = (int)(ua1 * 1.866891 + 0.868104);
  pwm_msg.data[1] = (int)(ua2 * 1.866891 + 0.868104);
  pwm_msg.data[2] = (int)(ua3 * 1.866891 + 0.868104);
  pwm_msg.data[3] = (int)(ua4 * 1.866891 + 0.868104);
  PWM_ard.publish(&pwm_msg);

}

void writePWM_Kill() {
  u1 = 1000;
  u2 = 1000;
  u3 = 1000;
  u4 = 1000;

  u1 = constrain( u1, 1000, 2000 );
  u2 = constrain( u2, 1000, 2000 );
  u3 = constrain( u3, 1000, 2000 );
  u4 = constrain( u4, 1000, 2000 );

  writePWM(u1, u2, u3, u4);
}
void writePWM_Max() {
  u1 = 2000;
  u2 = 2000;
  u3 = 2000;
  u4 = 2000;

  u1 = constrain( u1, 1000, 2000 );
  u2 = constrain( u2, 1000, 2000 );
  u3 = constrain( u3, 1000, 2000 );
  u4 = constrain( u4, 1000, 2000 );

  writePWM(u1, u2, u3, u4);
}

void writePWM_Upboard(const std_msgs::Int16MultiArray &cmd_msg) {
  u1 = cmd_msg.data[0];
  u2 = cmd_msg.data[1];
  u3 = cmd_msg.data[2];
  u4 = cmd_msg.data[3];

  u1 = constrain( u1, 1000, 2000 );
  u2 = constrain( u2, 1000, 2000 );
  u3 = constrain( u3, 1000, 2000 );
  u4 = constrain( u4, 1000, 2000 );
  //Kill UAV================================================================================
  //    if ( channel(5) < 1023 ) {
  //    writePWM_Kill();
  //  }
  //  //----------------------------------------------------------------------------------------
  //  //Under Upboard's command==================================================================
  //  else {
  //    writePWM(u1, u2, u3, u4);
  //  }
  //----------------------------------------------------------------------------------------
}
//------------------------------------------------------------------------------------------


//for Read S.BUS data===========================================================
int16_t channel(uint8_t ch) {
  // Read channel data
  if ((ch > 0) && (ch <= 16)) {
    return channels[ch - 1];
  } else {
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
        if (data == 0x0f) {
          sbus_data[sbus_pointer] = data;
          sbus_pointer++;
        }
        break;
      case 24:    // Byte 25 >> if last byte == 0x00 >> convert data
        if (data == 0x00) {
          sbus_data[sbus_pointer] = data;
          // clear channels[]
          for (i = 0; i < 16; i++) {
            channels[i] = 0;
          }

          // reset counters
          uint8_t byte_in_sbus = 1;
          uint8_t bit_in_sbus = 0;
          uint8_t ch = 0;
          uint8_t bit_in_channel = 0;

          // process actual sbus data
          for (i = 0; i < 176; i++) {
            if (sbus_data[byte_in_sbus] & (1 << bit_in_sbus)) {
              channels[ch] |= (1 << bit_in_channel);
            }
            bit_in_sbus++;
            bit_in_channel++;

            if (bit_in_sbus == 8) {
              bit_in_sbus = 0;
              byte_in_sbus++;
            }
            if (bit_in_channel == 11) {
              bit_in_channel = 0;
              ch++;
            }
          }
          // DigiChannel 1
          if (sbus_data[23] & (1 << 0)) {
            channels[16] = 1;
          } else {
            channels[16] = 0;
          }
          // DigiChannel 2
          if (sbus_data[23] & (1 << 1)) {
            channels[17] = 1;
          } else {
            channels[17] = 0;
          }
          // Failsafe
          failsafe_status = SBUS_SIGNAL_OK;
          if (sbus_data[23] & (1 << 2)) {
            failsafe_status = SBUS_SIGNAL_LOST;
          }
          if (sbus_data[23] & (1 << 3)) {
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

// For ROS==================================================================================
ros::Subscriber<std_msgs::Int16MultiArray> PWMs("PWMs", writePWM_Upboard);
//------------------------------------------------------------------------------------------


void setup() {
  // For ROS========================================================================
  nh.getHardware()->setBaud(57600);
  //Serial.begin(57600);
  Serial1.begin(100000, SERIAL_8E2);
  sbus_msg.data = (short int *)malloc(sizeof(short int) * 8);
  sbus_msg.data_length = 8;
  pwm_msg.data = (short int *)malloc(sizeof(short int) * 4);
  pwm_msg.data_length = 4;
  nh.initNode();
  nh.advertise(sbus);
  nh.advertise(loop_timer);
  nh.advertise(PWM_ard);
  nh.subscribe(PWMs);
  //--------------------------------------------------------------------------------


  //Read&Write Resolution Setting==================================================
  analogWriteResolution(12);

  //-------------------------------------------------------------------------------

  //PinMode Setting==================================================
  pinMode(PROP_ONE_OUT_PIN, OUTPUT);
  pinMode(PROP_TWO_OUT_PIN, OUTPUT);
  pinMode(PROP_THR_OUT_PIN, OUTPUT);
  pinMode(PROP_FOU_OUT_PIN, OUTPUT);
  //------------------------------------------------------------------------------
  writePWM_Kill();

}


int32_t update_time = 0, start_time = 0, sbus_time = 0, iteration = 0;
void loop() {
  start_time = micros();
  if (sbus_time - update_time > 20000) {
    update_channels();
    sbus_msg.data[0] = channel(1);
    sbus_msg.data[1] = channel(2);
    sbus_msg.data[2] = channel(3);
    sbus_msg.data[3] = channel(4);
    sbus_msg.data[4] = channel(5);
    sbus_msg.data[5] = channel(6);
    sbus_msg.data[6] = channel(7);
    sbus_msg.data[7] = channel(8);
    sbus.publish(&sbus_msg);
    update_time = micros();
    sbus_time = micros();
  }
  else {
    sbus_time = micros();
  }
  if ( channel(5) < 1024 ) {
    writePWM_Kill();
  }
  else {
    writePWM(u1, u2, u3, u4);
    //        if(iteration<500)    writePWM(1200, 1000, 1000, 1000);
    //        else{
    //          writePWM(1800,1000,1000,1000);
    //          if(iteration>1000) iteration=0;
    //        }
  }
  //The following two lines are for DJI Snail ESC Calibration
  // Single beep: Min-Max setting
  // Double beep: Rotation direction
  //  if (sbus_msg.data[4] < 1024)  writePWM_Kill();
  //  else writePWM_Max();


  // For ROS (R/W to Up-board ROS)============================================================

  nh.spinOnce();
  while (micros() - start_time < 2000) {};
  loop_time.data = (int)(micros() - start_time);
  loop_timer.publish(&loop_time);
  //    iteration++;

}
