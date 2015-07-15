/*
    ArduIMU_Glove.ino
    Author: Seb Madgwick
*/

//------------------------------------------------------------------------------
// Includes

//#include "AHRS.h"
//#include "Calibration.h"
#include "FlexSensors.h"
//#include "I2CBus.h"
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

#define FLEX_SEND_RATE      20  // packet send rates in ms
#define SENSOR_SEND_RATE    10
#define QUATERION_SEND_RATE 20
#define MISC_SEND_RATE      1000

//------------------------------------------------------------------------------
// Variables

//AHRS ahrs;
SimpleTimer timer;
// Set the FreeIMU object
FreeIMU imu = FreeIMU();
float q[4];

//------------------------------------------------------------------------------
// Functions

void setup() {

    // Init Serial for use by Send.cpp and Receive.cpp
    Serial.begin(115200);
    Wire.begin();

    // Init modules
    LEDs::init();
    FlexSensors::init();
    //MPU6000::init();
    //I2CBus::init();
    //Calibration::init();
    VibrationMotor::init();

    LEDs::setBlue(1);
    delay(5);
    imu.init();
    delay(5);

    // Init timer
    //timer.setInterval(10, sampleTimerTasks);
    timer.setInterval(FLEX_SEND_RATE, Send::flexSensorData);
    //timer.setInterval(SENSOR_SEND_RATE, Send::sensorData);
    timer.setInterval(QUATERION_SEND_RATE, sendIMUData);

    // Indicate init complete
    LEDs::setRed(1);
}

void loop() {
    timer.run();
    Receive::doTasks();
}

void sendIMUData() {
  imu.getQ(q);
  Send::quaternionData(q);
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

//------------------------------------------------------------------------------
// End of file
