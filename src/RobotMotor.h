#ifndef _ROBOTMOTOR_H_
#define _ROBOTMOTOR_H_

#include <stdint.h>

/* ---------------------------------- 
   --- TB6612FNG Motor controller ---
   ---------------------------------- */

#define MOTOR_STOP      0
#define MOTOR_FORWARD   1
#define MOTOR_BACKWARD  2

#define MOTOR_SPEED_LOW     80
#define MOTOR_SPEED_NORMAL  100
#define MOTOR_SPEED_HIGH    200

class RobotMotor
{
private:
  uint8_t pin_pwma;
  uint8_t pin_pwmb;
  uint8_t pin_ain1;
  uint8_t pin_bin1;
  uint8_t pin_stby;
public:
  void init(uint8_t pwma, uint8_t pwmb, uint8_t ain1, uint8_t bin1, uint8_t stby);
  void control(uint8_t direction_group_left, uint8_t speed_group_left, uint8_t direction_group_right, uint8_t speed_group_right);
};

#endif