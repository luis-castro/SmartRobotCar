#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "RobotLed.h"

#define NUM_LEDS   1  // Configured specifically for your 1-LED expansion board

// Declare the NeoPixel object:
// Parameter 1 = Number of pixels in the strip (1)
// Parameter 2 = Arduino pin number (PIN_RGBLED)
// Parameter 3 = Pixel type flags, add together as needed (NEO_GRB + NEO_KHZ800 is standard for WS2812B)

//static Adafruit_NeoPixel rgbLed(NUM_LEDS, PIN_RGBLED, NEO_GRB + NEO_KHZ800);
static Adafruit_NeoPixel rgbLed;

void RobotLed::init(int pin) {
  // Initialize the library data pin structures
  rgbLed.setPin(pin);
  rgbLed.updateType(NEO_GRB + NEO_KHZ800);
  rgbLed.updateLength(NUM_LEDS);
  rgbLed.begin();
  
  // Turn off the LED initially
  rgbLed.show();
}

void RobotLed::flash(int r, int g, int b) {
  static unsigned long last_time = 0;
  unsigned long current_time;
  static bool led_on = false;

  current_time = millis();
  if ((current_time - last_time) > 250) {
    led_on = !led_on;
    if (led_on) {
      rgbLed.setBrightness(255);
      rgbLed.setPixelColor(0, r, g, b);
    }
    else rgbLed.setPixelColor(0, 0, 0, 0);
    rgbLed.show();
    last_time = current_time;
  }
}

void RobotLed::slowBlink(int r, int g, int b) {
  static unsigned long time = 0;
  static bool increase = true;
  static int brightness = 0;

  if ((millis() - time) > 5)
  {
    time = millis();
    if (increase) {
        brightness++;
        if (brightness == 128) increase = false;
    }
    else {
        brightness--;
        if (!brightness) increase = true;
    }
    rgbLed.setBrightness(brightness);
    rgbLed.setPixelColor(0, r, g, b);
    rgbLed.show();
  }
}

void RobotLed::setColor(int r, int g, int b) {
  rgbLed.setBrightness(255);
  rgbLed.setPixelColor(0, r, g, b);
  rgbLed.show();
}