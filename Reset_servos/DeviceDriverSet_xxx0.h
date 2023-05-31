#ifndef _DeviceDriverSet_xxx0_H_
#define _DeviceDriverSet_xxx0_H_


#include <arduino.h>

/*Motor*/
class DeviceDriverSet_Motor
{
public:
  void DeviceDriverSet_Motor_Init(void);
#if _Test_DeviceDriverSet
  void DeviceDriverSet_Motor_Test(void);
#endif
  void DeviceDriverSet_Motor_control(boolean direction_A, uint8_t speed_A, //A组电机参数
                                     boolean direction_B, uint8_t speed_B, //B组电机参数
                                     boolean controlED                     //AB使能允许 true
  );                                                                       //电机控制
private:

#define PIN_Motor_PWMA 5
#define PIN_Motor_PWMB 6
#define PIN_Motor_BIN_1 8
#define PIN_Motor_AIN_1 7
#define PIN_Motor_STBY 3

public:
#define speed_Max 255
#define direction_just true
#define direction_back false
#define direction_void 3

#define Duration_enable true
#define Duration_disable false
#define control_enable true
#define control_disable false

// IR sensors pin
#define L_S A2
#define M_S A1
#define R_S A0
// #define Turn_S A3

//ultrasonic pins
#define ultrasonic_ping 13 // Trigger Pin of Ultrasonic Sensor
#define ultraonisc_echo 12 // Echo Pin of Ultrasonic Sensor


//servo for ultrasonic pins (S4)
//#define ultrasonicServoPin 11 // servo pin for ultrasonic servo(S4)

//Servo 1 (slider)
#define servoSlider 44

//Servo 2 (lifting system)
#define servoLift 45

//Servo 3 (gripper)
#define servoGripper 46

// pressure sensor
#define pressurePin A11

// ir_sensor for number of seeing black line
#define ir_sensor_L 26
#define ir_sensor_R 27

};

/*IRrecv*/
#include <IRremote.h>

class DeviceDriverSet_IRrecv
{
public:
  void DeviceDriverSet_IRrecv_Init(void);
  bool DeviceDriverSet_IRrecv_Get(uint8_t *IRrecv_Get /*out*/);
  void DeviceDriverSet_IRrecv_Test(void);

public:
  unsigned long IR_PreMillis;

private:
#define RECV_PIN 9

/*A:4294967295*/
#define aRECV_upper 16736925
#define aRECV_lower 16754775
#define aRECV_Left 16720605
#define aRECV_right 16761405
#define aRECV_ok 16712445
#define aRECV_1 16738455
#define aRECV_2 16750695
#define aRECV_3 16756815
#define aRECV_4 16724175
#define aRECV_5 16718055
#define aRECV_6 16743045
#define aRECV_7 16716015
#define aRECV_8 16726215
#define aRECV_9 16734885
// #define aRECV_ *16728765
// #define aRECV_0 16730805
// #define aRECV_ # 16732845
/*B:*/
#define bRECV_upper 5316027
#define bRECV_lower 2747854299
#define bRECV_Left 1386468383
#define bRECV_right 553536955
#define bRECV_ok 3622325019
#define bRECV_1 3238126971
#define bRECV_2 2538093563
#define bRECV_3 4039382595
#define bRECV_4 2534850111
#define bRECV_5 1033561079
#define bRECV_6 1635910171
#define bRECV_7 2351064443
#define bRECV_8 1217346747
#define bRECV_9 71952287
  // #define bRECV_ *851901943
  // #define bRECV_0 465573243
  // #define bRECV_ # 1053031451
};

#endif
