#include <Arduino.h>
#include "RobotMotor.h"

#define SPEED_MAX 255
#define SERIAL_DEBUG

/* ---------------------------------- 
   --- TB6612FNG Motor controller ---
   ---------------------------------- */
void RobotMotor::init(uint8_t pwma, uint8_t pwmb, uint8_t ain1, uint8_t bin1, uint8_t stby)
{
  pinMode(pwma, OUTPUT);
  pinMode(pwmb, OUTPUT);
  pinMode(ain1, OUTPUT);
  pinMode(bin1, OUTPUT);
  pinMode(stby, OUTPUT);
  pin_pwma = pwma;
  pin_pwmb = pwmb;
  pin_ain1 = ain1;
  pin_bin1 = bin1;
  pin_stby = stby;
}

void RobotMotor::control(uint8_t direction_group_left, uint8_t speed_group_left, uint8_t direction_group_right, uint8_t speed_group_right)
{
  static uint8_t prev_direction_left = 255;
  static uint8_t prev_direction_right = 255;
  static uint8_t prev_speed_left = 255;
  static uint8_t prev_speed_right = 255;

  /* If no change in direction or speed, do nothing */
  if (prev_direction_left == direction_group_left &&
      prev_direction_right == direction_group_right &&
      prev_speed_left == speed_group_left &&
      prev_speed_right == speed_group_right) return;

#ifdef SERIAL_DEBUG
  Serial.println("RobotMotor::control()");
#endif
  digitalWrite(pin_stby, HIGH);
  switch (direction_group_right) //movement direction control
  {
  case MOTOR_FORWARD:
    digitalWrite(pin_ain1, HIGH);
    analogWrite(pin_pwma, speed_group_left);
    break;

  case MOTOR_BACKWARD:
    digitalWrite(pin_ain1, LOW);
    analogWrite(pin_pwma, speed_group_left);
    break;

  case MOTOR_STOP:
    analogWrite(pin_pwma, 0);
    digitalWrite(pin_stby, LOW);
    break;

  default:
    analogWrite(pin_pwma, 0);
    digitalWrite(pin_stby, LOW);
    break;
  }

  switch (direction_group_left)
  {
  case MOTOR_FORWARD:
    digitalWrite(pin_bin1, HIGH);
    analogWrite(pin_pwmb, speed_group_right);
    break;

  case MOTOR_BACKWARD:
    digitalWrite(pin_bin1, LOW);
    analogWrite(pin_pwmb, speed_group_right);
    break;

  case MOTOR_STOP:
    analogWrite(pin_pwmb, 0);
    digitalWrite(pin_stby, LOW);
    break;

  default:
    analogWrite(pin_pwmb, 0);
    digitalWrite(pin_stby, LOW);
    break;
  }
  prev_direction_left = direction_group_left;
  prev_direction_right = direction_group_right;
  prev_speed_left = speed_group_left;
  prev_speed_right = speed_group_right;
}
