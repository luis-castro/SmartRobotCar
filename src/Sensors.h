#ifndef _SENSORS_H_
#define _SENSORS_H_

#include <Arduino.h>
#include <Wire.h>

/* ------------------------- 
   --- Voltage Detection ---
   ------------------------- */
class RobotVoltage {
private:
  uint8_t pin;
public:
  void init(uint8_t pin);
  float getVoltage(void);
};


/* ----------------------
   --- Onboard Button ---
   ---------------------- */
class RobotButton {
private:
  uint8_t pin;

public:
  void init(uint8_t pin);
  void get(uint8_t *get_keyValue);

  private:
  volatile static uint8_t keyValue;
  static void interruptHandler();
};


/* -------------------
   --- IR receiver ---
   ------------------- */
class RobotIR {
  public:
    void init(uint8_t pin);
    bool getCode(int *value);
};


/* ---------------------------------
   --- Ultrasonic sensor HC-SR04 ---
   --------------------------------- */
class RobotUltrasonic
{
private:
  uint8_t pin_trigger;
  uint8_t pin_echo;
public:
  void init(uint8_t pinTrigger, uint8_t pinEcho);
  long getRange(void);
};


/* ---------------------------------
   --- ITR20001 Opto interrupter ---
   --------------------------------- */
class RobotOpto
{
private:
  uint8_t pin_l;
  uint8_t pin_m;
  uint8_t pin_r;
public:
  bool init(uint8_t pinLeft, uint8_t pinMiddle, uint8_t pinRight);
  int readSensor_L(void);
  int readSensor_M(void);
  int readSensor_R(void);
};

void safe_delay(uint16_t ms);

#endif