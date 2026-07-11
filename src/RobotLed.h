#ifndef _ROBOTLED_H_
#define _ROBOTLED_H_

/* -----------------------
   --- Onboard RBG LED ---
   ----------------------- */
class RobotLed
{
public:
  void init(int pin);
  void slowBlink(int r, int g, int b);
  void flash(int r, int g, int b);
  void setColor(int r, int g, int b);
};

#endif