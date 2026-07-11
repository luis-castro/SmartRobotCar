# Elegoo SmartRobotCar V4.0 alternative firmware

This is an alternate version of the [software included](https://download.elegoo.com/?t=RobotCarV4.0)
with the [Elegoo SmartRobotCar V4.0](https://www.elegoo.com/blogs/arduino-projects/elegoo-smart-robot-car-kit-v4-0-tutorial).

Just an experimental version for fun. I didn't like to source code included with the robot, as it was too difficult to read and was a mix
of different coding styles.

I tried to use a more uniform style and better abstraction and an improved object detection algorithm.

This code will compiled under VScode with the [PlatformIO](https://platformio.org) plugin.

## Basic operation
The robot has the following operation modes, indicated by the LED color:

- Standby (slow flashing green): Stopped and waiting.
- Following mode (slow flashing yellow): Not implemented yet.
- Line tracking mode (slow flashing violet): Not implemented yet.
- Obstacle avoidance mode (slow flashing blue): Robot will advance until an obstacle is detected. Then it will use its ultrasonic ranger (left and right) and continue in the direction that has the most available space. If no space is detected, it will reverse and turn left.

There are 2 special modes:
- Low Battery (flashing red)
- Gound not detected (flashing green)

## Remote control operation

The use of direction keys will put the robot in Rocker control mode.
The robot will advance in the direction of the key and automatically stop if it detects an obstacle, in such a case it will go back to Standby mode.
