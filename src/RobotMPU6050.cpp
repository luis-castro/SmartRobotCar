#include <I2Cdev.h>
#include <MPU6050.h>
#include "RobotMPU6050.h"
#include <stdio.h>
#include <math.h>

bool RobotMPU6050::init(void)
{
  int ax_o,ay_o,az_o;
  int gx_o,gy_o,gz_o;

  Serial.print("Initializing MPU6050... ");
  Wire.begin();
  sensor.initialize();
  if (!sensor.testConnection()) {
    Serial.println("Error connecting to MPU6050");
    return false;
  }
  ax_o=sensor.getXAccelOffset();
  ay_o=sensor.getYAccelOffset();
  az_o=sensor.getZAccelOffset();
  gx_o=sensor.getXGyroOffset();
  gy_o=sensor.getYGyroOffset();
  gz_o=sensor.getZGyroOffset();

  Serial.print("OK\nOffsets [ax ay_o az gz gy gz]     :\t");
  Serial.print(ax_o); Serial.print("\t"); 
  Serial.print(ay_o); Serial.print("\t"); 
  Serial.print(az_o); Serial.print("\t"); 
  Serial.print(gx_o); Serial.print("\t"); 
  Serial.print(gy_o); Serial.print("\t");
  Serial.print(gz_o); Serial.println("");

  return true;
}

bool RobotMPU6050::calibrate(void)
{
  long f_ax = 0, f_ay = 0, f_az = 0; // Acceleration filter
  long f_gx = 0, f_gy = 0, f_gz = 0; // Rotation filter
  int a_ax = 0, a_ay = 0, a_az = 0;  // Acceleration average value
  int a_gx = 0, a_gy = 0, a_gz = 0;  // Rotation average value
  int ax_o = 0, ay_o = 0, az_o = 0;  // Acceleration offset
  int gx_o = 0, gy_o = 0, gz_o = 0;  // Rotation offset
  int i, j;

  ax_o = sensor.getXAccelOffset();
  ay_o = sensor.getYAccelOffset();
  az_o = sensor.getZAccelOffset();
  gx_o = sensor.getXGyroOffset();
  gy_o = sensor.getYGyroOffset();
  gz_o = sensor.getZGyroOffset();

  Serial.print("Calibrating MPU... ");
  RobotMPU6050::updateValues();
 
  for (i = 0; i < 100; i++) {
    for(j = 0; j < 100; j++) {
      // Exponential Moving Average (EMA)
      //   New Filter = Old Filter - Old Filter / 32 + Currrent
      //   Average = New Filter / 32
      f_ax = f_ax - (f_ax >> 5) + ax;
      a_ax = f_ax >> 5;
      f_ay = f_ay - (f_ay >> 5) + ay;
      a_ay = f_ay >> 5;
      f_az = f_az - (f_az >> 5) + az;
      a_az = f_az >> 5;

      f_gx = f_gx - (f_gx >> 3) + gx;
      a_gx = f_gx >> 3;
      f_gy = f_gy - (f_gy >> 3) + gy;
      a_gy = f_gy >> 3;
      f_gz = f_gz - (f_gz >> 3) + gz;
      a_gz = f_gz >> 3;
    }

    // Calibrate accelerometer
    if (a_ax > 0) ax_o--; else ax_o++;
    if (a_ay > 0) ay_o--; else ay_o++;
    if (a_az - 16384 > 0) az_o--; else az_o++; // 1G on Z axis
    sensor.setXAccelOffset(ax_o);
    sensor.setYAccelOffset(ay_o);
    sensor.setZAccelOffset(az_o);

    // Calibrate gyro to 0 deg/s
    if (a_gx > 0) gx_o--; else gx_o++;
    if (a_gy > 0) gy_o--; else gy_o++;
    if (a_gz > 0) gz_o--; else gz_o++;
    sensor.setXGyroOffset(gx_o);
    sensor.setYGyroOffset(gy_o);
    sensor.setZGyroOffset(gz_o); 
  }
  Serial.print("OK\nAverage values [ax ay az gz gy gz]:\t");
  Serial.print(a_ax); Serial.print("\t");
  Serial.print(a_ay); Serial.print("\t");
  Serial.print(a_az); Serial.print("\t");
  Serial.print(a_gx); Serial.print("\t");
  Serial.print(a_gy); Serial.print("\t");
  Serial.print(a_gz); Serial.print("\n");
  Serial.print("Offsets [ax ay_o az gz gy gz]     :\t");
  Serial.print(ax_o); Serial.print("\t");
  Serial.print(ay_o); Serial.print("\t");
  Serial.print(az_o); Serial.print("\t");
  Serial.print(gx_o); Serial.print("\t");
  Serial.print(gy_o); Serial.print("\t");
  Serial.print(gz_o); Serial.print("\n");

  return true;
}

bool RobotMPU6050::updateValues()
{
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

return true;
}

bool RobotMPU6050::getEulerAngles(float *Yaw)
{
  unsigned long now = millis();           //Record the current time(ms)
  dt = (now - lastTime) / 1000.0;         //Caculate the derivative time(s)
  lastTime = now;                         //Record the last sampling time(ms)
  gz = sensor.getRotationZ();          //Read the raw values of the six axes
  float gyroz = -(gz - gzo) / 131.0 * dt; //z-axis angular velocity
  if (fabs(gyroz) < 0.05)                 //Clear instant zero drift signal
  {
    gyroz = 0.00;
  }
  agz += gyroz; //z-axis angular velocity integral
  *Yaw = agz;
  return true;
}
