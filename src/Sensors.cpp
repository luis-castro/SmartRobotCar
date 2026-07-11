#include <Arduino.h>
#include <avr/wdt.h>
#define DECODE_NEC 
#include <IRremote.hpp>
#include "Sensors.h"

#define SERIAL_DEBUG

/* Voltage divisor */
#define ANALOG_MAX_VAL 1024
#define DIVIDER_R1  10
#define DIVIDER_R2  1.5
#define VOLTAGE_MAX 5.00

/* Ultra sound ranger */
#define MAX_DISTANCE 200 // Maximum distance (in centimeters). Maximum sensor distance is rated at 400-500cm.

/* Key modes */
#define MAX_KEYVALUE 4

void safe_delay(uint16_t ms) {
  // clear the Watchdog Timer back to zero just before the delay
  wdt_reset();
  delay(ms);
}

/* ------------------------- 
   --- Voltage Detection ---
   ------------------------- */
void RobotVoltage::init(uint8_t pin_voltage)
{
  pin = pin_voltage;
  pinMode(pin_voltage, INPUT);
}

float RobotVoltage::getVoltage(void)
{
  // Convert 0-1023 digital to Voltage:
  //    Vout = Val * MAX_V / 1024
  // Get Vin from voltage divider:
  //    Vin = Vout (R1 + R2) / R2  
  float V = (analogRead(pin) * VOLTAGE_MAX / ANALOG_MAX_VAL) * ((DIVIDER_R1 + DIVIDER_R2) / DIVIDER_R2);
  V = V + (V * 0.08); //Compensation 8%
  return V;
}


/* ----------------------
   --- Onboard Button ---
   ---------------------- */
volatile uint8_t RobotButton::keyValue = 0; /* Allocate memory for keyValue */

void RobotButton::init(uint8_t pin_button)
{
  pin = pin_button;
  keyValue = 0;
  pinMode(pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin), RobotButton::interruptHandler, FALLING);
}

void RobotButton::interruptHandler(void)
{
  static uint32_t keyValue_time = 0;
  static uint8_t keyValue_temp = 0;

  if ((millis() - keyValue_time) > 500)
  {
    keyValue_time = millis();
    keyValue_temp++;
    if (keyValue_temp > MAX_KEYVALUE)
      keyValue_temp = 0;
    keyValue = keyValue_temp;
  }
}

void RobotButton::get(uint8_t *get_keyValue)
{
  *get_keyValue = keyValue;
}


/* -------------------
   --- IR receiver ---
   ------------------- */
void RobotIR::init(uint8_t pin)
{
  IrReceiver.begin(pin, ENABLE_LED_FEEDBACK);
}

bool RobotIR::getCode(int *data) {
  if (IrReceiver.decode())
  {
#ifdef SERIAL_DEBUG    
    IrReceiver.printIRResultShort(&Serial); // Print complete received data in one line
#endif
    *data = IrReceiver.decodedIRData.command;    
    IrReceiver.resume(); // Enable receiving of the next value
    return true;
  }
  return false;
}


/* ---------------------------------
   --- Ultrasonic sensor HC-SR04 ---
   --------------------------------- */
void RobotUltrasonic::init(uint8_t pinTrigger, uint8_t pinEcho)
{
  pin_trigger = pinTrigger;
  pin_echo = pinEcho;
  pinMode(pin_echo, INPUT);
  pinMode(pin_trigger, OUTPUT);
}

long RobotUltrasonic::getRange()
{
  static long last_read = 0;
  const long min_interval = 50; // minimum interval in ms
  long time;
  static long distance = 0;

  // if too short interval, return cached distance
  if (millis() - last_read < min_interval) {
    return distance;
  }

  // Clear the trigPin by setting it LOW.
  digitalWrite(pin_trigger, LOW);
  delayMicroseconds(5);

  digitalWrite(pin_trigger, HIGH);
  delayMicroseconds(10); // 10us pulse
  digitalWrite(pin_trigger, LOW);
  time = pulseIn(pin_echo, HIGH); // time in microseconds
  distance = time * 0.034 / 2; // sound speed is 0.034 cm/us
  last_read = millis();
  return distance;
}


/* ---------------------------------
   --- ITR20001 Opto interrupter ---
   --------------------------------- */
bool RobotOpto::init(uint8_t pinLeft, uint8_t pinMiddle, uint8_t pinRight)
{
  pin_l = pinLeft;
  pin_m = pinMiddle;
  pin_r = pinRight;
  pinMode(pin_l, INPUT);
  pinMode(pin_m, INPUT);
  pinMode(pin_r, INPUT);
  return false;
}

int RobotOpto::readSensor_L(void)
{
  return analogRead(pin_l);
}

int RobotOpto::readSensor_M(void)
{
  return analogRead(pin_m);
}

int RobotOpto::readSensor_R(void)
{
  return analogRead(pin_r);
}
