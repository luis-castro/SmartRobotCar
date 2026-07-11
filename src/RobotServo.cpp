#ifndef _ROBOTSERVO_H_
#define _ROBOTSERVO_H_

#include <Arduino.h>
#include "RobotServo.h"

/* -------------------------
   --- Camera Servo SG90 ---
   ------------------------- */
void RobotServo::init(int pinX, int pinY)
{
  servoX.attach(pinX, 500, 2400); //500: 0 degree  2400: 180 degree
  servoX.attach(pinX);
  servoX.write(90); //sets the servo position according to the 90（middle）
  delay(500);

  servoY.attach(pinY, 500, 2400); //500: 0 degree  2400: 180 degree
  servoY.attach(pinY);
  servoY.write(90); //sets the servo position according to the 90（middle）
  delay(500);
}

void RobotServo::controlX(unsigned int angle)
{  
  if (angle > 180) angle = 180;
  servoX.write(angle);
  delay(500);
}


void RobotServo::controlY(unsigned int angle)
{  
  if (angle > 180) angle = 180;
  servoY.write(angle);
  delay(500);
}

#endif