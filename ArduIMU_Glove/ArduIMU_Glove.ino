/*
    ArduIMU_Glove.ino
    Author: Seb Madgwick
*/

//------------------------------------------------------------------------------
// Includes

//#include "AHRS.h"
//#include "Calibration.h"
#include "Adafruit_NeoPixel.h"
#include "FlexSensors.h"
#include "MPU60X0.h"
#include "Receive.h"
#include "SimpleTimer.h"
#include "Send.h"
#include <SPI.h>    // required by MPU6000::cpp
#include <Wire.h>   // required by I2CBus.cpp
#include <EEPROM.h> // required by Calibration.cpp
#include "LEDs.h"
#include "VibrationMotor.h"

// FreeIMU
#include "FreeIMU.h"


//------------------------------------------------------------------------------
// Definitions


#define SENSOR_SEND_RATE    10  // sensor readout rate in ms
#define FLEX_SEND_RATE      20  // flex sensor send rates
#define QUATERION_SEND_RATE 20  // quaternion sen rates
#define SAMPLE_TIMER_RATE   10  // sample IMU rates

//------------------------------------------------------------------------------
// Variables

//AHRS ahrs;
SimpleTimer timer;
// Set the FreeIMU object
FreeIMU imu = FreeIMU();
float q[4];


#define BUTTON_PIN   9    // Digital IO pin connected to the button.  This will be
                          // driven with a pull-up resistor so the switch should
                          // pull the pin to ground momentarily.  On a high -> low
                          // transition the button press logic will execute.


#define PIXEL_PIN    13    // Digital IO pin connected to the NeoPixels.

#define PIXEL_COUNT 1

// Parameter 1 = number of pixels in strip,  neopixel stick has 8
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_RGB     Pixels are wired for RGB bitstream
//   NEO_GRB     Pixels are wired for GRB bitstream, correct for neopixel stick
//   NEO_KHZ400  400 KHz bitstream (e.g. FLORA pixels)
//   NEO_KHZ800  800 KHz bitstream (e.g. High Density LED strip), correct for neopixel stick
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

bool oldState = HIGH;
int showType = 0;


//------------------------------------------------------------------------------
// Functions

void setup() {

    // Init Serial for use by Send.cpp and Receive.cpp
    Serial.begin(115200);
    Wire.begin();

    // Init modules
    LEDs::init();
    FlexSensors::init();
    //Calibration::init();
    VibrationMotor::init();

    //LEDs::setBlue(1);
    LEDs::setRed(1);
    // RGB LED
    pinMode(BUTTON_PIN, INPUT);
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
    startShow(7);

    imu.init();

    // Init timer
     timer.setInterval(QUATERION_SEND_RATE, sendIMUData);
     
    /*delay(5);
      timer.setInterval(SAMPLE_TIMER_RATE, sampleTimerTasks);    
      timer.setInterval(FLEX_SEND_RATE, Send::flexSensorData);
      timer.setInterval(SAMPLE_TIMER_RATE, getButtonState)
    }*/

    
    // timer.setInterval(SENSOR_SEND_RATE, Send::sensorData);
    // Indicate init complete
}

void loop() {
  timer.run();
  //Receive::doTasks();
}

void getButtonState() {
  // Read button state.
  bool newState = digitalRead(BUTTON_PIN);
  
  // Check if state changed from high to low (button press).
  if (newState == LOW && oldState == HIGH) {
    // Check if button is still low after debounce.
    newState = digitalRead(BUTTON_PIN);
    if (newState == LOW) {
      Send::buttonState(1);
    }
  }
  oldState = newState;
}

void sendIMUData() {
  imu.getQ(q);
  serialPrintFloatArr(q, 4);
  Serial.println("");
  //Send::quaternionData(q);
}

// old code
/*
void sampleTimerTasks() {

    // Read Sensor data
    //MPU6000::read();
    //I2CBus::readMagnetometer();

    // Calibrate data
    //Calibration::update();

    // Update AHRS algorithm
    //ahrs.update(DegToRad(0.1f * Calibration::gyrX), DegToRad(0.1f * Calibration::gyrY), DegToRad(0.1f * Calibration::gyrZ), Calibration::accX, Calibration::accY, Calibration::accZ);
}*/

void startShow(int i) {
  switch(i){
    case 0: colorWipe(strip.Color(0, 0, 0), 50);    // Black/off
            break;
    case 1: colorWipe(strip.Color(255, 0, 0), 50);  // Red
            break;
    case 2: colorWipe(strip.Color(0, 255, 0), 50);  // Green
            break;
    case 3: colorWipe(strip.Color(0, 0, 255), 50);  // Blue
            break;
    case 4: theaterChase(strip.Color(127, 127, 127), 50); // White
            break;
    case 5: theaterChase(strip.Color(127,   0,   0), 50); // Red
            break;
    case 6: theaterChase(strip.Color(  0,   0, 127), 50); // Blue
            break;
    case 7: rainbow(20);
            break;
    case 8: rainbowCycle(20);
            break;
  }
}


// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, c);    //turn every third pixel on
      }
      strip.show();
     
      delay(wait);
     
      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if(WheelPos < 170) {
    WheelPos -= 85;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}

void serialPrintFloatArr(float * arr, int length) {
  for(int i=0; i<length; i++) {
    serialFloatPrint(arr[i]);
    Serial.print(",");
  }
}


void serialFloatPrint(float f) {
  byte * b = (byte *) &f;
  for(int i=0; i<4; i++) {
    
    byte b1 = (b[i] >> 4) & 0x0f;
    byte b2 = (b[i] & 0x0f);
    
    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;
    
    Serial.print(c1);
    Serial.print(c2);
  }
}

//------------------------------------------------------------------------------
// End of file
