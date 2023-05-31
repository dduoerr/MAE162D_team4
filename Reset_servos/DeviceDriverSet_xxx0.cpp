#include "DeviceDriverSet_xxx0.h"

/*Motor control*/
void DeviceDriverSet_Motor::DeviceDriverSet_Motor_Init(void)
{
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);

}


void DeviceDriverSet_Motor::DeviceDriverSet_Motor_control(boolean direction_A, uint8_t speed_A, //A组电机参数
                                                          boolean direction_B, uint8_t speed_B, //B组电机参数
                                                          boolean controlED                     //AB使能允许 true
                                                          )                                     //电机控制
{

  if (controlED == control_enable) //使能允许？
  {
    digitalWrite(PIN_Motor_STBY, HIGH);
    { //A...Right

      switch (direction_A) //方向控制
      {
      case direction_just:
        digitalWrite(PIN_Motor_AIN_1, HIGH);
        analogWrite(PIN_Motor_PWMA, speed_A);
        break;
      case direction_back:

        digitalWrite(PIN_Motor_AIN_1, LOW);
        analogWrite(PIN_Motor_PWMA, speed_A);
        break;
      case direction_void:
        analogWrite(PIN_Motor_PWMA, 0);
        digitalWrite(PIN_Motor_STBY, LOW);
        break;
      default:
        analogWrite(PIN_Motor_PWMA, 0);
        digitalWrite(PIN_Motor_STBY, LOW);
        break;
      }
    }

    { //B...Left
      switch (direction_B)
      {
      case direction_just:
        digitalWrite(PIN_Motor_BIN_1, HIGH);
        analogWrite(PIN_Motor_PWMB, speed_B);
        break;
      case direction_back:
        digitalWrite(PIN_Motor_BIN_1, LOW);
        analogWrite(PIN_Motor_PWMB, speed_B);
        break;
      case direction_void:
        analogWrite(PIN_Motor_PWMB, 0);
        digitalWrite(PIN_Motor_STBY, LOW);
        break;
      default:
        analogWrite(PIN_Motor_PWMB, 0);
        digitalWrite(PIN_Motor_STBY, LOW);
        break;
      }
    }
  }
  else
  {
    digitalWrite(PIN_Motor_STBY, LOW);
    return;
  }
};

/*IRrecv*/
IRrecv irrecv(RECV_PIN); //  Create an infrared receive drive object
decode_results results;  //  Create decoding object
void DeviceDriverSet_IRrecv::DeviceDriverSet_IRrecv_Init(void)
{
  irrecv.enableIRIn(); //Enable infrared communication NEC
}
bool DeviceDriverSet_IRrecv::DeviceDriverSet_IRrecv_Get(uint8_t *IRrecv_Get /*out*/)
{
  if (irrecv.decode(&results))
  {
    IR_PreMillis = millis();
    switch (results.value)
    {
    case /* constant-expression */ aRECV_upper:
    case /* constant-expression */ bRECV_upper:
      /* code */ *IRrecv_Get = 11;
      break;
    case /* constant-expression */ aRECV_lower:
    case /* constant-expression */ bRECV_lower:
      /* code */ *IRrecv_Get = 12;
      break;
    case /* constant-expression */ aRECV_Left:
    case /* constant-expression */ bRECV_Left:
      /* code */ *IRrecv_Get = 13;
      break;
    case /* constant-expression */ aRECV_right:
    case /* constant-expression */ bRECV_right:
      /* code */ *IRrecv_Get = 14;
      break;
    case /* constant-expression */ aRECV_ok:
    case /* constant-expression */ bRECV_ok:
      /* code */ *IRrecv_Get = 15;
      break;

    case /* constant-expression */ aRECV_1:
    case /* constant-expression */ bRECV_1:
      /* code */ *IRrecv_Get = 1;
      break;
    case /* constant-expression */ aRECV_2:
    case /* constant-expression */ bRECV_2:
      /* code */ *IRrecv_Get = 2;
      break;
    case /* constant-expression */ aRECV_3:
    case /* constant-expression */ bRECV_3:
      /* code */ *IRrecv_Get = 3;
      break;
    case /* constant-expression */ aRECV_4:
    case /* constant-expression */ bRECV_4:
      /* code */ *IRrecv_Get = 4;
      break;
    case /* constant-expression */ aRECV_5:
    case /* constant-expression */ bRECV_5:
      /* code */ *IRrecv_Get = 5;
      break;
    case /* constant-expression */ aRECV_6:
    case /* constant-expression */ bRECV_6:
      /* code */ *IRrecv_Get = 6;
      break;
    default:
      // *IRrecv_Get = 5;
      irrecv.resume();
      return false;
      break;
    }
    irrecv.resume();
    return true;
  }
  else
  {
    return false;
  }
}
