/**
 * \file unreal.h
 * \brief Main program application (ino file)
 * 
 * Based on the Adafruit NeoPixel library specifications consider the
 * following notes related to the mode the library instance is created:\n
 * Parameter 1 = number of pixels in strip\n
 * Parameter 2 = pin number (most are valid)\n
 * Parameter 3 = pixel type flags, add together as needed:\n
 * NEO_RGB     Pixels are wired for RGB bitstream\n
 * NEO_GRB     Pixels are wired for GRB bitstream, correct for neopixel stick\n
 * NEO_KHZ400  400 KHz bitstream (e.g. FLORA pixels)\n
 * NEO_KHZ800  800 KHz bitstream (e.g. High Density LED strip), correct for neopixel stick\n
 * 
 * In our case there are two 8 LEDs strips connected in cascade and the last is connected
 * to a 12 LEDs ring
 * 
 * \author Enrico Miglino <balearicdynamics@gmail.com>
 * \version 1.0.0
 * \date April 2019
 */
#include "settings.h"

#undef _DEBUG // Undef to avoid serial commnication

#ifdef _DEBUG
#include <Streaming.h>
#endif

Servo duck;         ///< Duck servo engine
environment unreal;   ///< Environmental parameters

//! Interactive Neopixel control parameters
neoPixParameters lightsControl;

//! Creates the Neopixel strip instance
Adafruit_NeoPixel lights = Adafruit_NeoPixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

/**
 * \brief Application initialization
 * 
 * Motors are set to the initial direction. Note that the button
 * pins are initialized as INPUT_PULLUP so no external pull-up
 * resistor is needed
 */
void setup() {
#ifdef _DEBUG
Serial.begin(38400);
#endif
  // Initialize motor controller pins
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Preset motors direction
  // Motors initially is disabled
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_ROSE, INPUT_PULLUP);
  pinMode(BTN_DUCK, INPUT_PULLUP);

  lights.begin();
  lights.show(); // Initialize all pixels to 'off'

  // Initialize parameters
  unreal.isDuckOn = false;
  unreal.isRoseOn = false;
  unreal.isRunning = false;
  unreal.isTrainOn = false;
  unreal.isDuckStarted = false;
  unreal.isRoseStarted = false;
  unreal.lightScheme = LIGHT_STATIC;

  lightsControl.color = 0;
  lightsControl.stepDelay = LIGHT_STATIC_DELAY;
  lightsControl.timerCounter = 0;
}

/**
 * \brief Main application loop
 */
void loop() {

  // Only if the system is running the interaction buttons
  // are checked
  checkButton(BTN_START);
  if(unreal.isRunning) {
    // Check the buttons status (if pressed)
    checkButton(BTN_DUCK);
    checkButton(BTN_ROSE);
    checkTrainSpeed();
    interactiveLights();
  }
}

//! Manage the light effects depending on the last user interaction
void interactiveLights(){

  switch(unreal.lightScheme){
    case LIGHT_STATIC:
      interactiveRotatingRainbow();
    break;
    case LIGHT_ROSE:
      interactiveColorCycle();
    break;
    case LIGHT_DUCK:
      interactiveFlashColor(lights.Color(0, 16, 128));
    break;
    case LIGHT_TRAIN:
      if(unreal.isTrainOn){
        interactiveColor(lights.Color(lightsControl.colorRGB, 256 - lightsControl.colorRGB, 128 - lightsControl.colorRGB));
        #ifdef _DEBUG
        Serial << "TrainOn - colorRGB " << lightsControl.colorRGB << endl;
        #endif
      }
    break;
  }
}

/**
 * \brief Check the train speed based on the potentiometer value
 * and apply it to the train motor
 * 
 * To avoid too low PWM frequencies that does not start the train, 
 * the potentiometer returned value is checked in two ways.
 * If the minimum value is in the "zero" range (to manage a minimum of tolerance)
 * the motor is kept stopped, else the minimum mapped value to the motor controller
 * PWM frequency if the value that start running the motor.
 */
void checkTrainSpeed(){
    
    int pwmOutput = analogRead(A0);
    int rgb;

    // Check for zero point
    if(pwmOutput <= ZERO_TOLERANCE) {
      analogWrite(enB, 0); // Motor stopped
      unreal.isTrainOn = false;
    }
    else {
      // Map the potentiometer to the train motor PWM frequencies range
      map(pwmOutput, 0, 1023, MIN_TRAIN_SPEED, MAX_TRAIN_SPEED); 
      analogWrite(enB, pwmOutput); // Send PWM signal to L298N Enable pin
      // Enable the train flag
      unreal.isTrainOn = true;
      unreal.lightScheme = LIGHT_TRAIN;
      // Map the potentiometer to the speed-related light color
      lightsControl.colorRGB = analogRead(A0);
      map(lightsControl.colorRGB, 0, 1023, MIN_TRAIN_COLOR, MAX_TRAIN_COLOR);
      lightsControl.stepDelay = LIGHT_TRAIN_DELAY;
      lightsControl.timerCounter = 0;
    }
}

/**
 * \brief Check the desired button status and set the status flag
 * 
 * Uses a simple debouncing method
 * 
 * \param button Button pin
 */
 void checkButton(int button) {
  // Check if state changed from high to low (button press).
  if(digitalRead(button) == LOW) {
    delay(20);  // debounce delay
    // Check if button is still low after debounce.
    if(digitalRead(button) == LOW) {
      switch(button) {
        case BTN_START:
          unreal.isRunning = !unreal.isRunning;
          changeRunningStatus();
        break;
        case BTN_DUCK:
          unreal.isDuckOn = !unreal.isDuckOn;
          changeDuckStatus();
        break;
        case BTN_ROSE:
          unreal.isRoseOn = !unreal.isRoseOn;
          changeRoseStatus();
        break;
        } // Flag setting
      } // Debounce button
  } // Button pressed
}

/**
 * \brief Change the duck running status
 * 
 * Depending on the current status of the duck servo it starts
 * or stop it
 */
void changeDuckStatus() {
  if(unreal.isDuckStarted) {
    unreal.isDuckStarted = false;
    duck.detach();  // Stop duck servo
    // Initialise the light scheme
    unreal.lightScheme = LIGHT_STATIC;
    lightsControl.color = 0;
    lightsControl.stepDelay = LIGHT_STATIC_DELAY;
    lightsControl.timerCounter = 0;
  }
  else {
    unreal.isDuckStarted = true;
    duck.attach(SERVO_DUCK);  // Start duck servo
    duck.write(DUCK_MOTION);  // Set speed and directio
    // Initialise the light scheme
    unreal.lightScheme = LIGHT_DUCK;
    lightsControl.color = 0;
    lightsControl.stepDelay = LIGHT_DUCK_DELAY;
    lightsControl.timerCounter = 0;
    #ifdef _DEBUG
    Serial << "changeDuckStatus " << endl;
    #endif
  }
}

/**
 * \brief Change the rose running status
 * 
 * Depending on the current status of the rose motor is started
 * or stoped
 */
void changeRoseStatus() {
  if(unreal.isRoseStarted) {
    unreal.isRoseStarted = false;
    analogWrite(enA, 0); // Send PWM signal to L298N Enable pin
    // Initialise the light scheme
    unreal.lightScheme = LIGHT_STATIC;
    lightsControl.color = 0;
    lightsControl.stepDelay = LIGHT_STATIC_DELAY;
    lightsControl.timerCounter = 0;
  }
  else {
    unreal.isRoseStarted = true;
    analogWrite(enA, ROSE_SPEED); // Send PWM signal to L298N Enable pin
    // Initialise the light scheme
    unreal.lightScheme = LIGHT_ROSE;
    lightsControl.color = 0;
    lightsControl.stepDelay = LIGHT_ROSE_DELAY;
    lightsControl.timerCounter = 0;
  }
}

/**
 * \brief Change the running status
 * 
 * Stop all the activities and return to non started mode or start the
 * system.
 * 
 * \note the 500 ms dealy pause after the system has been stopped is needed
 * to avoid reading the button state immediately after che command has been 
 * processed as it is very fast.
 */
void changeRunningStatus() {
  if(unreal.isRunning) {
    rainbow(32, 128, 5);
    delay(500);
    color(lights.Color(0, 0, 0), 1);
    // Initialise the light scheme
    unreal.lightScheme = LIGHT_STATIC;
    lightsControl.color = 0;
    lightsControl.stepDelay = LIGHT_STATIC_DELAY;
    lightsControl.timerCounter = 0;
  }
  else {
    analogWrite(enA, 0); // Send PWM signal to L298N Enable pin
    color(lights.Color(255, 0, 0), 50);
    duck.detach();
    color(lights.Color(0, 0, 0), 1);
  }
}

// Fill the dots one after the other with a color
/**
 * \brief Progressive fill all the LEDs with the same color
 * 
 * \param c The 320-bit encoded color
 * \param wait The step delay (ms)
 */
void color(uint32_t c, uint8_t wait) {
  
  for(uint16_t i = 0; i < lights.numPixels(); i++) {
    lights.setPixelColor(i, c);
    lights.show();
    delay(wait);
  }
}

/**
 * Cycle all the lights LEDS in sequence over all the 256 RGB combinations
 * 
 * \param low The lowest color value of the cycle range
 * \param high The highest color values of the cycle range
 * \param delay The delay cycle changing colors
 */
void rainbow(uint8_t low, uint8_t high, uint8_t wait) {
  uint16_t i, j;

  for(j = low; j < high; j++) {
    for(i = 0; i < lights.numPixels(); i++) {
      lights.setPixelColor(i, calcColor((i+j) & 255));
    }
    lights.show();
    delay(wait);
  }
}

/**
 * \brief Progressive fill all the LEDs with the same color, then alternatively
 * flash the sequence with white
 * 
 * This methodd uses the global lightsControl structure and should be called
 * during the loop cycle
 */
void interactiveFlashColor(uint32_t c) {
  
    unsigned long now = millis();
    if((now - lightsControl.timerCounter) > lightsControl.stepDelay){
      lightsControl.timerCounter = now;

      for(uint16_t i = 0; i < lights.numPixels(); i++) {
        if(lightsControl.swapColor){
          lights.setPixelColor(i, lights.Color(127, 127, 127));
        }
        else{
          lights.setPixelColor(i, c);
        }
        lights.show();
      }
      lightsControl.swapColor = !lightsControl.swapColor;
    }
}

/**
 * \brief Progressive fill all the LEDs with the same color
 * 
 * This methodd uses the global lightsControl structure and should be called
 * during the loop cycle
 */
void interactiveColor(uint32_t c) {
  
  for(uint16_t i = 0; i < lights.numPixels(); i++) {
      lights.setPixelColor(i, c);
    }
    lights.show();
}

/**
 * \brief Rotating rainbow interactive (no parameters from external)
 * 
 * This methodd uses the global lightsControl structure and should be called
 * during the loop cycle
 */
void interactiveRotatingRainbow() {
  
  for (int q=0; q < 3; q++) {
    for (int i=0; i < lights.numPixels(); i=i+3) {
      lights.setPixelColor(i + q, calcColor( (i + lightsControl.color) % 255));
    }
    lights.show();

      unsigned long now = millis();
      if((now - lightsControl.timerCounter) > lightsControl.stepDelay){
        lightsControl.timerCounter = now;
        
        for (int i = 0; i < lights.numPixels(); i = i + 3) {
          lights.setPixelColor(i + q, 0);        //turn every third pixel off

        lightsControl.color++;
        if(lightsControl.color > 255)
          lightsControl.color = 0;
      }
    }
    else {
      return;
    }
  }
}

/**
 * \brief Continuous color shaing
 * 
 * This methodd uses the global lightsControl structure and should be called
 * during the loop cycle
 */
void interactiveColorCycle() {
  uint16_t i;

  unsigned long now = millis();
  if((now - lightsControl.timerCounter) > lightsControl.stepDelay){
    lightsControl.timerCounter = now;

    for(i=0; i< lights.numPixels(); i++) {
      lights.setPixelColor(i, calcColor(((i * 256 / lights.numPixels()) + lightsControl.color) & 255));
    }

    lights.show();

    lightsControl.color++;
    if(lightsControl.color > (256 * 5))
      lightsControl.color = 0;
  } // Timer tick
}

/**
 * \brief Utility function to manage the neopixels lights
 * 
 * Using the Neopixels library calculates the color of a generic
 * LED position. The 32 bit unsigned integer is the resulting color
 * that can be used to set a LED of the Neopixel stream
 * 
 * \param position The LED position in the stream
 * \return The uint32_t 32 bit integer with the color value
 */
uint32_t calcColor(byte pos) {
  pos = 255 - pos;
  if(pos < 85) {
    return lights.Color(255 - pos * 3, 0, pos * 3);
  }
  if(pos < 170) {
    pos -= 85;
    return lights.Color(0, pos * 3, 255 - pos * 3);
  }
  pos -= 170;
  return lights.Color(pos * 3, 255 - pos * 3, 0);
}


