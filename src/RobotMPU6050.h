#ifndef _MPU6050_getdata_H_
#define _MPU6050_getdata_H_
#include <Arduino.h>
#include <MPU6050.h>

class RobotMPU6050
{
private:
  MPU6050 sensor;

public:
  bool init(void);
  bool calibrate(void);
  bool getEulerAngles(float *Yaw);
  bool updateValues(void);

  int16_t ax, ay, az; // acceleration
  int16_t gx, gy, gz; // rotation
  float   pitch, roll, yaw;
  unsigned long now, lastTime = 0;
  float dt;      //Derivative time
  float agz = 0; //Angle variable
  long gzo = 0;  //Gyro offset
};
#endif
