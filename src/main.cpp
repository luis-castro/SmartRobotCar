#include <Arduino.h>
#include <avr/wdt.h>

#include "RobotLed.h"
#include "RobotMPU6050.h"
#include "RobotMotor.h"
#include "RobotServo.h"
#include "Sensors.h"

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
#define PIN_SERVO1       10 // Servo for ultrasonic sensor and camera
#define PIN_SERVO2       11 // Not used, but required for the library to work
#define PIN_RANGING_ECHO 12 // Arduino pin tied to echo pin on the ultrasonic sensor.
#define PIN_RANGING_TRIG 13 // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define PIN_ITR20001_R   A0 // Opto interrupter Right
#define PIN_ITR20001_M   A1 // Opto interrupter Center
#define PIN_ITR20001_L   A2 // Opto interrupter Left
#define PIN_VOLTAGE      A3 // Pin for voltage sensor

// Note: Motor controller pins AIN2 and BIN2 are not connected

#define DISTANCE_COLLISION 30
#define MIN_VOLTAGE        6.5

// Check if a value is within a specified range (inclusive)
static boolean isInRange(long x, long min, long max) {
  if (min <= x && x <= max)
    return true;
  else
    return false;
}

enum robot_operation_mode_t {
  MODE_STANDBY = 0,
  MODE_LINETRACKING,      /* Line Tracking Mode */
  MODE_OBSTACLEAVOIDANCE, /* Obstacle Avoidance Mode */
  MODE_FOLLOW,            /* Following Mode */
  MODE_ROCKERCTRL
};

enum robot_direction_t {
  DIRECTION_STOP = 0,
  DIRECTION_FORWARD,
  DIRECTION_BACKWARD,
  DIRECTION_RIGHT,
  DIRECTION_LEFT
};

enum avoidance_mode_t {
  AVOIDANCE_MODE_CLEAR = 0,
  AVOIDANCE_MODE_SCANNING_LEFT,
  AVOIDANCE_MODE_SCANNING_RIGHT,
  AVOIDANCE_MODE_DECISION,
  AVOIDANCE_MODE_TURN,
  AVOIDANCE_MODE_BACKUP
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
#define REMOTE_OK 0x40
#define REMOTE_UP 0x46
#define REMOTE_DN 0x15
#define REMOTE_LT 0x44
#define REMOTE_RT 0x43
#define REMOTE_N1 0x16
#define REMOTE_N2 0x19
#define REMOTE_N3 0x0d
#define REMOTE_N4 0x0c
#define REMOTE_N5 0x1B
#define REMOTE_N6 0x5e
#define REMOTE_N7 0x08
#define REMOTE_N8 0x1c
#define REMOTE_N9 0x5a
#define REMOTE_STAR 0x42
#define REMOTE_HASH 0x4a

// Global flags
robot_operation_mode_t mode;
robot_direction_t robot_direction;
int robot_speed;

// Initialize each device object and set the initial state of the robot
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

/* Send commands to the 4 motors according to the specified direction and speed */
void set_motor(int direction, int speed) {
  static int prev_direction = 255;
  static int prev_speed = 8192;

  if ((prev_direction == direction) && (prev_speed == speed))
    return;

  switch (direction) {
    case DIRECTION_STOP:
      myMotor.control(MOTOR_STOP, 0, MOTOR_STOP, 0);
      break;

    case DIRECTION_FORWARD:
      myMotor.control(MOTOR_FORWARD, speed, MOTOR_FORWARD, speed);
      break;

    case DIRECTION_BACKWARD:
      myMotor.control(MOTOR_BACKWARD, speed, MOTOR_BACKWARD, speed);
      break;

    case DIRECTION_LEFT:
      myMotor.control(MOTOR_BACKWARD, speed, MOTOR_FORWARD, speed);
      break;

    case DIRECTION_RIGHT:
      myMotor.control(MOTOR_FORWARD, speed, MOTOR_BACKWARD, speed);
      break;
  }
}

/* Handles IR remote control commands by updating robot_direction,
   robot_speed and mode according to the received command */
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
      switch (mode) {
        case MODE_STANDBY:
          mode = MODE_FOLLOW;
          break;
        case MODE_FOLLOW:
          mode = MODE_LINETRACKING;
          break;
        case MODE_LINETRACKING:
          mode = MODE_OBSTACLEAVOIDANCE;
          break;
        case MODE_OBSTACLEAVOIDANCE:
          mode = MODE_ROCKERCTRL;
          break;
        case MODE_ROCKERCTRL:
          mode = MODE_STANDBY;
          break;
      }
  }
}

// Check if the robot is on the ground by reading the opto interrupter sensors
bool on_ground() {
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

// Update the LED color based on the current mode
void update_led_color() {
  switch (mode) {
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

// Perform obstacle avoidance by scanning for obstacles and adjusting the
// robot's movement accordingly
void obstacle_avoidance() {
  static avoidance_mode_t avoidance_mode = AVOIDANCE_MODE_CLEAR;
  static unsigned long last_servo_move_timestamp = 0;
  static unsigned long last_turn_timestamp = 0;

  const int OBSTACLE_THRESHOLD = 30;  // Minimum safe distance in cm
  const int SCAN_DELAY = 400;         // Time (ms) to wait for the servo to physically move
  const int TURN_TIME = 300;          // Time (ms) to allow the robot to turn
  const int BACKUP_TIME = 1000;
  const int LEFT_ANGLE = 150;
  const int RIGHT_ANGLE = 30;
  const int FORWARD_ANGLE = 90;
  long distanceForward = 0;
  long distanceLeft = 0;
  long distanceRight = 0;

  switch (avoidance_mode) {
    case AVOIDANCE_MODE_CLEAR:
      // In this mode we advance forward until we detect an obstacle
      // Read the distance straight ahead
      robot_direction = DIRECTION_FORWARD;
      robot_speed = MOTOR_SPEED_NORMAL;
      distanceForward = myRanging.getRange();

      if (isInRange(distanceForward, 0, OBSTACLE_THRESHOLD)) {
        // Obstacle detected, stop the robot and switch to scanning mode
        robot_direction = DIRECTION_STOP;
        robot_speed = 0;
        avoidance_mode = AVOIDANCE_MODE_SCANNING_LEFT;
        myServo.controlX(LEFT_ANGLE);
        last_servo_move_timestamp = millis();
      }
      break;

    case AVOIDANCE_MODE_SCANNING_LEFT:
      // Scan Left
      if (millis() - last_servo_move_timestamp > SCAN_DELAY) {
        distanceLeft = myRanging.getRange();
        avoidance_mode = AVOIDANCE_MODE_SCANNING_RIGHT;
        myServo.controlX(RIGHT_ANGLE);
        last_servo_move_timestamp = millis();
      }
      break;

    case AVOIDANCE_MODE_SCANNING_RIGHT:
      // Scan Right
      if (millis() - last_servo_move_timestamp > SCAN_DELAY) {
        distanceRight = myRanging.getRange();
        avoidance_mode = AVOIDANCE_MODE_DECISION;
        myServo.controlX(FORWARD_ANGLE);
        last_servo_move_timestamp = millis();
      }
      break;

    case AVOIDANCE_MODE_DECISION:
      if (millis() - last_servo_move_timestamp > SCAN_DELAY) {
        // Calculate distances
        if (distanceLeft > distanceRight && distanceLeft > OBSTACLE_THRESHOLD) {
          // Path left is clearer
          robot_direction = DIRECTION_LEFT;
          robot_speed = MOTOR_SPEED_NORMAL;
          last_turn_timestamp = millis();
          avoidance_mode = AVOIDANCE_MODE_TURN;
        }
        else if (distanceRight > distanceLeft && distanceRight > OBSTACLE_THRESHOLD) {
          // Path right is clearer
          robot_direction = DIRECTION_RIGHT;
          robot_speed = MOTOR_SPEED_NORMAL;
          last_turn_timestamp = millis();
          avoidance_mode = AVOIDANCE_MODE_TURN;
        }
        else {
          // Trapped (both sides blocked)! Back up and spin
          robot_direction = DIRECTION_BACKWARD;
          robot_speed = MOTOR_SPEED_LOW;
          last_turn_timestamp = millis();
          avoidance_mode = AVOIDANCE_MODE_BACKUP;
        }
      }
      break;

    case AVOIDANCE_MODE_BACKUP:
      if (millis() - last_turn_timestamp > BACKUP_TIME) {
        robot_direction = DIRECTION_LEFT;
        robot_speed = MOTOR_SPEED_NORMAL;
        last_turn_timestamp = millis();
        avoidance_mode = AVOIDANCE_MODE_TURN;
      }
      break;

    case AVOIDANCE_MODE_TURN:
      if (millis() - last_turn_timestamp > TURN_TIME) {
        avoidance_mode = AVOIDANCE_MODE_CLEAR;
        robot_direction = DIRECTION_FORWARD;
        robot_speed = MOTOR_SPEED_NORMAL;
      }
      break;
  }
}

// Following mode
void follow_mode() {
  const int MAX_FOLLOW_DISTANCE = 100;
  const int MIN_FOLLOW_DISTANCE = 10;
  const long SERVO_TIME = 250;      // time to wait for servo to move
  const long MAX_STEER_TIME = 200;  // time to steer when target not centered

  enum ScanState {
    SCAN_FULL_LEFT,
    SCAN_LEFT,
    SCAN_CENTER,
    SCAN_RIGHT,
    SCAN_FULL_RIGHT,
    FOLLOWING
  };
  enum ServoPosition {
    SERVO_FULL_LEFT = 150,
    SERVO_LEFT = 120,
    SERVO_CENTER = 90,
    SERVO_RIGHT = 60,
    SERVO_FULL_RIGHT = 30
  };
  static ScanState currentState = FOLLOWING;
  uint16_t distance;
  static unsigned long lastServoMoveTimeStamp = 0;
  static unsigned long targetAcquiredTimeStamp = 0;

  // Safety fallback: Stop moving if wheels lose contact with the ground
  if (!on_ground()) {
    set_motor(DIRECTION_STOP, 0);
    myServo.controlX(SERVO_CENTER);
    safe_delay(SERVO_TIME);
    currentState = FOLLOWING;
    lastServoMoveTimeStamp = 0;
    targetAcquiredTimeStamp = 0;
    return;
  }
  switch (currentState) {
    case FOLLOWING:
      if (millis() - targetAcquiredTimeStamp >= SERVO_TIME) {
        distance = myRanging.getRange();
#ifdef SERIAL_DEBUG
        Serial.print("Distance: ");
        Serial.println(distance);
#endif
        if (distance < MIN_FOLLOW_DISTANCE) {
          set_motor(DIRECTION_STOP, 0);
          targetAcquiredTimeStamp = millis();  // Force a delay
        }
        else if (distance > MAX_FOLLOW_DISTANCE) {
          // Target lost. Move sensor to full left and start scanning for target
          set_motor(DIRECTION_STOP, 0);
          myServo.controlX(SERVO_FULL_LEFT);
          lastServoMoveTimeStamp = millis();
          currentState = SCAN_FULL_LEFT;
        }
        else {
          if ((millis() - targetAcquiredTimeStamp) >= MAX_STEER_TIME + SERVO_TIME)
            set_motor(DIRECTION_FORWARD, MOTOR_SPEED_NORMAL);
        }
      }
      break;

    case SCAN_FULL_LEFT:
    case SCAN_LEFT:
    case SCAN_CENTER:
    case SCAN_RIGHT:
    case SCAN_FULL_RIGHT:
      // Check if enough time has passed for the servo gear to finish rotating
      if ((millis() - lastServoMoveTimeStamp) >= SERVO_TIME) {
        distance = myRanging.getRange();

        if (isInRange(distance, 0, MAX_FOLLOW_DISTANCE)) {
          // Engage motors if target re-acquired
          targetAcquiredTimeStamp = millis();
          switch (currentState) {
            case SCAN_FULL_LEFT:
              set_motor(DIRECTION_LEFT, MOTOR_SPEED_NORMAL);
              break;
            case SCAN_LEFT:
              set_motor(DIRECTION_FORWARD, MOTOR_SPEED_NORMAL);
              break;
            case SCAN_CENTER:
              set_motor(DIRECTION_FORWARD, MOTOR_SPEED_NORMAL);
              break;
            case SCAN_RIGHT:
              set_motor(DIRECTION_FORWARD, MOTOR_SPEED_NORMAL);
              break;
            case SCAN_FULL_RIGHT:
              set_motor(DIRECTION_RIGHT, MOTOR_SPEED_NORMAL);
              break;
          }
          // Re-center the sensor radar
          myServo.controlX(SERVO_CENTER);
          currentState = FOLLOWING;
        }
        else {
          switch (currentState) {
            case SCAN_FULL_LEFT:
              myServo.controlX(SERVO_LEFT);
              lastServoMoveTimeStamp = millis();
              currentState = SCAN_LEFT;
              break;

            case SCAN_LEFT:
              myServo.controlX(SERVO_CENTER);
              lastServoMoveTimeStamp = millis();
              currentState = SCAN_CENTER;
              break;

            case SCAN_CENTER:
              myServo.controlX(SERVO_RIGHT);
              lastServoMoveTimeStamp = millis();
              currentState = SCAN_RIGHT;
              break;

            case SCAN_RIGHT:
              myServo.controlX(SERVO_FULL_RIGHT);
              lastServoMoveTimeStamp = millis();
              currentState = SCAN_FULL_RIGHT;
              break;

            case SCAN_FULL_RIGHT:
              // If we reached this state, our target is lost.
              // We give up and stop.
              myServo.controlX(SERVO_CENTER);
              safe_delay(SERVO_TIME);
              mode = MODE_STANDBY;
              currentState = FOLLOWING;
              lastServoMoveTimeStamp = 0;
              targetAcquiredTimeStamp = 0;
              break;
          }
        }
      }
  }
}

// Perform line tracking by reading the opto interrupter sensors and adjusting
// the robot's movement accordingly
void line_tracking() {
  const uint16_t TRACKING_LOW_THRESHOLD = 250;
  const uint16_t TRACKING_HIGH_THRESHOLD = 850;

  if (isInRange(myOpto.readSensor_M(), TRACKING_LOW_THRESHOLD, TRACKING_HIGH_THRESHOLD)) {
    // Center sensor detects the line: move forward
    robot_direction = DIRECTION_FORWARD;
    robot_speed = MOTOR_SPEED_NORMAL;
  }
  else if (isInRange(myOpto.readSensor_L(), TRACKING_LOW_THRESHOLD, TRACKING_HIGH_THRESHOLD)) {
    // Left sensor detects the line: turn left
    robot_direction = DIRECTION_LEFT;
    robot_speed = MOTOR_SPEED_NORMAL;
  }
  else if (isInRange(myOpto.readSensor_R(), TRACKING_LOW_THRESHOLD, TRACKING_HIGH_THRESHOLD)) {
    // Right sensor detects the line: turn right
    robot_direction = DIRECTION_RIGHT;
    robot_speed = MOTOR_SPEED_NORMAL;
  }
  else {
    // No sensors detect the line: stop
    robot_direction = DIRECTION_STOP;
    robot_speed = 0;
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

  init();  // Arduino hardware initialization
  setup();
  myMPU.calibrate();

  // Main loop: continuously check sensors, update mode, and control motors
  while (true) {
#ifdef SERIAL_DEBUG
    if (mode != prev_mode) {
      Serial.print("mode changed to: ");
      Serial.println(mode);
      prev_mode = mode;
    }
#endif
    if (myVoltage.getVoltage() < MIN_VOLTAGE) {
#ifdef SERIAL_DEBUG
      Serial.println("Low voltage detected");
#endif
      set_motor(DIRECTION_STOP, 0);
      myLed.flash(255, 0, 0);
      continue;
    }

    if (!on_ground()) {
      // stop motors, but do not change mode
      myLed.flash(0, 255, 0);
      set_motor(DIRECTION_STOP, 0);
      continue;
    }

    // Update mode, direction and speed based on IR
    if (myIR.getCode(&ir_code))
      action_ir_command(ir_code);

    // Update LED color
    update_led_color();

    // Action according to current mode
    switch (mode) {
      case MODE_STANDBY:
        break;

      case MODE_ROCKERCTRL:
        distance = myRanging.getRange();
        if (distance != prev_distance) {
          prev_distance = distance;
        }

        if ((distance < DISTANCE_COLLISION) && (robot_direction == DIRECTION_FORWARD)) {
          Serial.println("Imminent collision detected");
          mode = MODE_STANDBY;
          robot_direction = DIRECTION_STOP;
          robot_speed = 0;
        }
        break;

      case MODE_LINETRACKING:
        line_tracking();
        break;

      case MODE_FOLLOW:
        follow_mode();
        break;

      case MODE_OBSTACLEAVOIDANCE:
        obstacle_avoidance();
        break;
    }

    // Apply changes to the motors
    set_motor(robot_direction, robot_speed);
    wdt_reset();
  }
  return 0;  // Standard C++ practice, though execution never reaches here
}
