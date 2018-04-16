/**
 * \file settings.h
 * \brief Program constants, pins, parameters
 * 
 * \author Enrico Miglino <balearicdynamics@gmail.com>
 * \version 1.0.0
 * \date April 2019
 */
#include <Adafruit_NeoPixel.h>
#include <Servo.h> 

// Neopixel
#define BTN_START 22     ///< Start/Stop the system
#define BTN_ROSE 24      ///< Rose interaction button
#define BTN_DUCK 26      ///< Duck interaction button

#define DEBOUNCE_DELAY 20 ///< Button debounce delay (ms)

#define NEOPIXEL_PIN 8     ///< Neopixel array control channel
#define NEOPIXEL_COUNT 28  ///< Number of chained neopixel LEDs

// DC Motor controller
#define enA 7     ///< Rose Motor 1 enable signal
#define in1 5     ///< Rose Motor 1 control signal 1
#define in2 6     ///< Rose Motor 1 control signal 2

#define enB 2     ///< Train Motor 2 enable signal
#define in3 3     ///< Train Motor 2 control signal 1
#define in4 4     ///< Train Motor 2 control signal 2

//! Rose motor speed. The lowest possible so the motor will start rotating.
#define ROSE_SPEED 17

//! Full rotation servo control (Duck mechanism)
#define SERVO_DUCK 9 
//! Duck servo speed and direction
#define DUCK_MOTION 0

//! Minimum PWM frequency value to start traing motor
#define MIN_TRAIN_SPEED 64
//! Maximum PWM frequency value to reach the train max speed
#define MAX_TRAIN_SPEED 255
//! Minimum RGB color values for train status
#define MIN_TRAIN_COLOR 0
//! Maximum PWM frequency value to reach the train max speed
#define MAX_TRAIN_COLOR 128
//! Max accepted analog read value to be considered zero. In this case
//! Motor does not moves
#define ZERO_TOLERANCE 50

// Lighting schemes ID
#define LIGHT_ROSE 1    ///< The default scheme wen the rose is running
#define LIGHT_DUCK 2    ///< The light scheme when the duck is running
#define LIGHT_STATIC 3  ///< The light scheme when nothing moves
#define LIGHT_TRAIN 4   ///< The light when the train runs

// Predefeined light delays
#define LIGHT_ROSE_DELAY 50       ///< Delay duration every color change
#define LIGHT_DUCK_DELAY 500      ///< Delay duration every color change
#define LIGHT_STATIC_DELAY 50     ///< Delay duration every color change
#define LIGHT_TRAIN_DELAY 10       ///< Delay duration every color change

//! Progressive fill color delay (ms)
#define FILL_COLOR_DELAY 10

/** 
 * \brief Defines all the parameters to control a light loop externally 
 * 
 * The stepDelay is  value in ms that should thick the next step of the 
 * loop. To void compromising the main application flow, the loop is replaced
 * by a time counter so the next step is launched when the timer reach the expected
 * value
 */
typedef struct {
  uint32_t color;             ///< The current color number in the loop
  int colorRGB;          ///< Train speed-color
  int stepDelay;              ///< The delay (ms) between every color shift operation
  unsigned long timerCounter; ///< The delay timer counter
  bool swapColor;             ///< If true, current color is swapped with white
} neoPixParameters;

//! Environment status flags and variables
typedef struct {
  bool isDuckOn;    ///< Duck enabled status
  bool isRoseOn;    ///< Rose enabled
  bool isTrainOn;   ///< True when the train motor is over zero
  bool isRunning;   ///< Global application status
  bool isDuckStarted;   ///< Duck servo has been started
  bool isRoseStarted;   ///< Rose motor has been started
  int lightScheme;      ///< The current lightinh scheme running
} environment;

