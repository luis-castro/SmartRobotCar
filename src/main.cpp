#include <Arduino.h>
#include <avr/wdt.h>
#include "Sensors.h"
#include "RobotLed.h"
#include "RobotMPU6050.h"
#include "RobotMotor.h"
#include "RobotServo.h"

#define SERIAL_DEBUG 1

// Robot PIN assignment
#define PIN_BUTTON        2 // Pin for button
#define PIN_MOTOR_STBY    3 // Motor STBY
#define PIN_RGBLED        4 // Onboard LED
#define PIN_MOTOR_PWMA    5 // Motor PWM right
#define PIN_MOTOR_PWMB    6 // Motor PWM left
#define PIN_MOTOR_AIN1    7 // Motor  IN right
#define PIN_MOTOR_BIN1    8 // Motor  IN left
#define PIN_IR_RECV       9 // Pin for Infrared receiver
#define PIN_RANGING_ECHO 12 // Arduino pin tied to echo pin on the ultrasonic sensor.
#define PIN_RANGING_TRIG 13 // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define PIN_ITR20001_R   A0 // Opto interrupter Right
#define PIN_ITR20001_M   A1 // Opto interrupter Center
#define PIN_ITR20001_L   A2 // Opto interrupter Left
#define PIN_VOLTAGE      A3 // Pin for voltage sensor

// Note: controller pins AIN2 and BIN2 are not connected
#define PIN_SERVO1     10
#define PIN_SERVO2     11


#define DISTANCE_COLLISION  30
#define MIN_VOLTAGE 6.5

enum robot_operation_mode_t {
  MODE_STANDBY = 0,
  MODE_LINETRACKING,                 /* Line Tracking Mode */
  MODE_OBSTACLEAVOIDANCE,            /* Obstacle Avoidance Mode */
  MODE_FOLLOW,                       /* Following Mode */
  MODE_ROCKERCTRL
};

enum robot_direction_t {
  DIRECTION_STOP = 0,
  DIRECTION_FORWARD,
  DIRECTION_BACKWARD,
  DIRECTION_RIGHT,
  DIRECTION_LEFT
};

// Sensors
RobotVoltage myVoltage;
RobotButton myButton;
RobotMPU6050 myMPU;
RobotIR myIR;
RobotOpto myOpto;
RobotUltrasonic myRanging;
RobotServo myServo;

// Devices
RobotLed myLed;
RobotMotor myMotor;

// Commands codes from remote
#define REMOTE_OK   0x40
#define REMOTE_UP   0x46
#define REMOTE_DN   0x15
#define REMOTE_LT   0x44
#define REMOTE_RT   0x43
#define REMOTE_N1   0x16
#define REMOTE_N2   0x19
#define REMOTE_N3   0x0d
#define REMOTE_N4   0x0c
#define REMOTE_N5   0x1B
#define REMOTE_N6   0x5e
#define REMOTE_N7   0x08
#define REMOTE_N8   0x1c
#define REMOTE_N9   0x5a
#define REMOTE_STAR 0x42
#define REMOTE_HASH 0x4a

// Global flags
robot_operation_mode_t mode;
robot_direction_t robot_direction;
int robot_speed;

void setup(void) {
  mode = MODE_STANDBY;
  robot_direction = DIRECTION_STOP;
  robot_speed = 0;

  Serial.begin(57600);
  Serial.println("\n\n\nSmart Robot Car V4.0 - Firmware 3.0\n");
  myLed.init(PIN_RGBLED);
  myVoltage.init(PIN_VOLTAGE);
  myButton.init(PIN_BUTTON);
  myMPU.init();
  myIR.init(PIN_IR_RECV);
  myOpto.init(PIN_ITR20001_L, PIN_ITR20001_M, PIN_ITR20001_R);
  myRanging.init(PIN_RANGING_TRIG, PIN_RANGING_ECHO);
  myServo.init(PIN_SERVO1, PIN_SERVO2);
  myMotor.init(PIN_MOTOR_PWMA, PIN_MOTOR_PWMB, PIN_MOTOR_AIN1, PIN_MOTOR_BIN1, PIN_MOTOR_STBY);
}


void set_motor(int direction, int speed)
{
  static int prev_direction = 255;
  static int prev_speed = 8192;

  if ((prev_direction == direction) && (prev_speed == speed)) return;

  switch(direction) {
    case DIRECTION_STOP:
      myMotor.control(MOTOR_STOP, 0, MOTOR_STOP, 0);
      break;

    case DIRECTION_FORWARD:
      myMotor.control(MOTOR_FORWARD, MOTOR_SPEED_NORMAL, MOTOR_FORWARD, MOTOR_SPEED_NORMAL);
      break;

    case DIRECTION_BACKWARD:
      myMotor.control(MOTOR_BACKWARD, MOTOR_SPEED_NORMAL, MOTOR_BACKWARD, MOTOR_SPEED_NORMAL);
      break;

    case DIRECTION_LEFT:
      myMotor.control(MOTOR_BACKWARD, MOTOR_SPEED_NORMAL, MOTOR_FORWARD, MOTOR_SPEED_NORMAL);
      break;

    case DIRECTION_RIGHT:
      myMotor.control(MOTOR_FORWARD, MOTOR_SPEED_NORMAL, MOTOR_BACKWARD, MOTOR_SPEED_NORMAL);
      break;
  }
}


void action_ir_command(int cmd) {
  switch (cmd) {
    case REMOTE_OK:
      robot_direction = DIRECTION_STOP;
      robot_speed = 0;
      mode = MODE_STANDBY;
      break;

    case REMOTE_UP:
      if (mode == MODE_STANDBY || mode == MODE_ROCKERCTRL) {
        robot_direction = DIRECTION_FORWARD;
        robot_speed = MOTOR_SPEED_NORMAL;
        mode = MODE_ROCKERCTRL;
      }
      break;

    case REMOTE_DN:
      if (mode == MODE_STANDBY || mode == MODE_ROCKERCTRL) {
        robot_direction = DIRECTION_BACKWARD;
        robot_speed = MOTOR_SPEED_NORMAL;
        mode = MODE_ROCKERCTRL;
      }
      break;

    case REMOTE_LT:
      if (mode == MODE_STANDBY || mode == MODE_ROCKERCTRL) {
        robot_direction = DIRECTION_LEFT;
        robot_speed = MOTOR_SPEED_NORMAL;
        mode = MODE_ROCKERCTRL;
      }
      break;

    case REMOTE_RT:
      if (mode == MODE_STANDBY || mode == MODE_ROCKERCTRL) {
        robot_direction = DIRECTION_RIGHT;
        robot_speed = MOTOR_SPEED_NORMAL;
        mode = MODE_ROCKERCTRL;
      }
      break;

    case REMOTE_HASH:
      switch(mode)
      {
        case MODE_STANDBY:           mode = MODE_FOLLOW; break;
        case MODE_FOLLOW:            mode = MODE_LINETRACKING; break;
        case MODE_LINETRACKING:      mode = MODE_OBSTACLEAVOIDANCE; break;
        case MODE_OBSTACLEAVOIDANCE: mode = MODE_ROCKERCTRL; break;
        case MODE_ROCKERCTRL:        mode = MODE_STANDBY; break;
      }
  }
}


bool on_ground()
{
  const int max_v = 800;
  int l, m, r;
  char str[80];

  l = myOpto.readSensor_L();
  m = myOpto.readSensor_M();
  r = myOpto.readSensor_R();

  if (l > max_v && m > max_v && r > max_v) {
#ifdef SERIAL_DEBUG
        sprintf(str, "INFO: Device not on ground [L: %d, M: %d, R: %d]\n", l, m, r);
        Serial.print(str);
#endif
        return false;
      }
  return true;
}


static boolean isInRange(long x, long min, long max) {
  if (min <= x && x <= max)
    return true;
  else
    return false;
}


void update_led_color()
{
  switch (mode)
  {
    case MODE_STANDBY:
      myLed.slowBlink(0, 255, 0);
      break;

    case MODE_ROCKERCTRL:
      myLed.slowBlink(128, 128, 0);
      break;

    case MODE_LINETRACKING:
      myLed.slowBlink(128, 0, 128);
      break;

    case MODE_FOLLOW:
      myLed.slowBlink(128, 128, 0);
      break;

    case MODE_OBSTACLEAVOIDANCE:
      myLed.slowBlink(0, 0, 128);
      break;
  }
}


void obstacle_avoidance()
{
  const int OBSTACLE_THRESHOLD = 30; // Minimum safe distance in cm
  const int NORMAL_SPEED = 100;      // Standard cruising speed
  const int SCAN_DELAY = 400;        // Time (ms) to wait for the servo to physically move
  const int TURN_TIME = 250;         // Time (ms) to allow the robot to turn
  const int LEFT_ANGLE = 150;
  const int RIGHT_ANGLE = 30;
  const int FORWARD_ANGLE = 90;
  long distanceForward = 0;
  long distanceLeft = 0;
  long distanceRight = 0;

  // Ensure sensor is facing forward
  myServo.controlX(FORWARD_ANGLE);
  
  // Read the distance straight ahead
  distanceForward = myRanging.getRange();

  // Check for obstacles
  if (isInRange(distanceForward, 0, OBSTACLE_THRESHOLD)) {
    
    // OBSTACLE DETECTED: Stop the robot immediately
    set_motor(DIRECTION_STOP, 0);    
    myLed.setColor(0, 128, 128);

    // Scan Left
    myServo.controlX(LEFT_ANGLE);
    safe_delay(SCAN_DELAY); // Wait for servo to move
    distanceLeft = myRanging.getRange();
    
    // Scan Right
    myServo.controlX(RIGHT_ANGLE);
    safe_delay(SCAN_DELAY); // Wait for servo to move
    distanceRight = myRanging.getRange();
    
    // Return sensor to center
    myServo.controlX(FORWARD_ANGLE);
    safe_delay(SCAN_DELAY);

    // Make a decision based on scans
    if (distanceLeft > distanceRight && distanceLeft > OBSTACLE_THRESHOLD) {
      // Path left is clearer
      set_motor(DIRECTION_LEFT, NORMAL_SPEED);
      safe_delay(TURN_TIME); 
    } 
    else if (distanceRight > distanceLeft && distanceRight > OBSTACLE_THRESHOLD) {
      // Path right is clearer
      set_motor(DIRECTION_RIGHT, NORMAL_SPEED);
      safe_delay(TURN_TIME);
    } 
    else {
      // Trapped (both sides blocked)! Back up and spin
      set_motor(DIRECTION_BACKWARD, NORMAL_SPEED);
      safe_delay(800);
      set_motor(DIRECTION_LEFT, NORMAL_SPEED);
      safe_delay(TURN_TIME * 2); // Spin around further
    }

    // Stop turning; the next loop iteration will resume forward movement
    set_motor(DIRECTION_STOP, 0);

  } else {
    // PATH IS CLEAR: Keep moving forward
    //update_motor(DIRECTION_FORWARD, NORMAL_SPEED);
    robot_direction = DIRECTION_FORWARD;
    robot_speed = NORMAL_SPEED;
  }
}


// Standard C++ program entry point
int main(void) {
    int ir_code;
    long distance;
    static long prev_distance = 0;
#ifdef SERIAL_DEBUG
    static robot_operation_mode_t prev_mode = MODE_STANDBY;
#endif

    init(); // Arduino hardware initialization
    setup();
    myMPU.calibrate();
    while (true) {
#ifdef SERIAL_DEBUG
      if (mode != prev_mode) {
        Serial.print("mode changed to: ");
        Serial.println(mode);
        prev_mode = mode;
      }
#endif
      if (myVoltage.getVoltage() < MIN_VOLTAGE) {
        myMotor.control(MOTOR_STOP, 0, MOTOR_STOP, 0);
        myLed.flash(255, 0, 0);
        continue;
      }

      if (!on_ground()) {
        // stop motors, but do not change mode
        myLed.flash(0, 255, 0);
        myMotor.control(MOTOR_STOP, 0, MOTOR_STOP, 0);
        continue;
      }

      // Update mode, direction and speed based on IR
      if (myIR.getCode(&ir_code))
        action_ir_command(ir_code);

      // Update LED color
      update_led_color();

      // Action according to current mode
      switch (mode)
      {
        case MODE_STANDBY:          
          break;

        case MODE_ROCKERCTRL:
          distance = myRanging.getRange();
          if (distance != prev_distance) {
            prev_distance = distance;
          }

          if ((distance < DISTANCE_COLLISION) && (robot_direction == DIRECTION_FORWARD)) {
            Serial.println("Imminent collision deteced");
            mode = MODE_STANDBY;
            robot_direction = DIRECTION_STOP;
            robot_speed = 0;
          }
          break;

        case MODE_LINETRACKING:
          break;

        case MODE_FOLLOW:
          break;

        case MODE_OBSTACLEAVOIDANCE:
          obstacle_avoidance();
          break;
      }

      // Apply changes to the motors
      set_motor(robot_direction, robot_speed);
      wdt_reset();
    }
    return 0; // Standard C++ practice, though execution never reaches here
}
