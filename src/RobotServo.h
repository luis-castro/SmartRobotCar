#include <Arduino.h>
#include <Servo.h>

/* -------------------------
   --- Camera Servo SG90 ---
   ------------------------- */
class RobotServo
{
public:
  void init(int pinX, int pinY);
  void controlX(unsigned int angle);
  void controlY(unsigned int angle);

private:
  Servo servoX;
  Servo servoY;
};
