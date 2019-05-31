/*
    Author: Jerome Hallett
    Board: Arduino Uno or ESP32
    Code's purpose is to read sensor values from the INA219 in the receiver, checking I2C connection available to avoid process stalling.
    Measures the rectifier voltage, current, and state of charge of the receiver.

    INA219 sensor code:
    Based from https://learn.adafruit.com/adafruit-ina219-current-sensor-breakout/arduino-code
*/

#include <Arduino.h>
//Import required libraries for INA219 sensor & I2C
#include <Wire.h>
#include <Adafruit_INA219.h>
////EPS32 OLED Screen
#include <OLEDDisplayFonts.h>
#include <OLEDDisplay.h>
#include <OLEDDisplayUi.h>
#include <SSD1306Wire.h>
#include <SSD1306.h>
#include "images.h"
#include "fonts.h"
#include <string.h>

//User specfied inputs
uint8_t chargeThreshold = 14.5; //Threshold of capacitor voltage to consider receiver as charged

//Device setup
unsigned long timer; //Time since programme started running
bool charged = false; //Boolean of whether the receiver has been fully charged
uint8_t ledPin = 16; // Onboard LED reference

//Initialise two instances of the INA219
byte addressRect = 0x40;
byte addressCap = 0x41;
Adafruit_INA219 ina219Rectifier(addressRect);
Adafruit_INA219 ina219_capacitor(addressCap);

// The built-in OLED is a 128*64 mono pixel display
// i2c address = 0x3c
// SDA = 5
// SCL = 4
SSD1306 display(0x3c, 5, 4);

// Function to check availability of I2C data bus
byte checkI2C (byte &address) {
  byte error;
  Wire.beginTransmission(address); // checks I2C connection is available
  error = Wire.endTransmission(); //If is available, error = 0.
  if (error > 0) {
    // Reset I2C
    ina219Rectifier.begin();
    ina219_capacitor.begin();
  }
  return error;
}

void setup() {
  pinMode(ledPin, OUTPUT); // Onboard LED reference
  //   Initialize the OLED display
  display.init();
  display.flipScreenVertically();
  display.setFont(Roboto_Medium_14);
  display.drawString(0, 0, "Data Logger");
  display.display();
  digitalWrite(ledPin, LOW); //NB LED pin is active low
  delay(1500);
  display.clear();

  Serial.begin(115200);
  Serial.println("Hello!");
  uint32_t currentFrequency;
  Wire.begin(21, 22);

  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  ina219Rectifier.begin();
  ina219_capacitor.begin();
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  //ina219.setCalibration_16V_400mA();

  Serial.println("Measuring voltage and current with INA219 ...");
}

void loop() {
  // Initialise rectifier sensor values
  float shuntVoltageRectifier = 0;
  float busVoltageRectifier = 0;
  float current_mA_rectifier = 0;
  float loadVoltageRectifier = 0;
  float powerRectifier_mW = 0;
  float actualVoltageRectifier = 0;

  // Initialise capacitor sensor values
  float shuntVoltageCapacitor = 0;
  float busVoltageCapacitor = 0;
  float currentCapacitor_mA = 0;
  float loadVoltageCapacitor = 0;
  float powerCapacitor_mW = 0;

  bool validRead = false; //flag to determine if data successfully read

  /*
    Now to read values from the INA219 sensors.
    If I2C unavailable due to interference, breaks the while loop to avoid process stalling.
  */
  while (checkI2C (addressRect) == 0 && checkI2C (addressCap) == 0 ) {
    //rectifier sensor read. Measures voltage from the potential divider and scales it up
    shuntVoltageRectifier = ina219Rectifier.getShuntVoltage_mV();
    busVoltageRectifier = ina219Rectifier.getBusVoltage_V();
    loadVoltageRectifier = busVoltageRectifier + (shuntVoltageRectifier / 1000);
    actualVoltageRectifier = loadVoltageRectifier * 4; // Potential divider ratio scaling
    // timer = millis();
    // Serial.print("Rectifier Values:   "); Serial.print(timer); Serial.print(","); Serial.println(actualVoltageRectifier);

    //Capacitor bank sensor read. Measures voltage and current
    shuntVoltageCapacitor = ina219_capacitor.getShuntVoltage_mV();
    busVoltageCapacitor = ina219_capacitor.getBusVoltage_V();
    currentCapacitor_mA = ina219_capacitor.getCurrent_mA();
    loadVoltageCapacitor = busVoltageCapacitor + (shuntVoltageCapacitor / 1000);
    powerCapacitor_mW = loadVoltageCapacitor * currentCapacitor_mA;
    // timer = millis();
    // Serial.print("Capacitor Values:   "); Serial.print(timer); Serial.print(","); Serial.print(loadVoltageCapacitor); Serial.print(","); Serial.print(powerCapacitor_mW); Serial.print(","); Serial.println(currentCapacitor_mA);

    //  Serial.print("Bus Voltage:   "); Serial.print(busVoltageRectifier); Serial.println(" V");
    //  Serial.print("Shunt Voltage: "); Serial.print(shuntVoltageRectifier); Serial.println(" mV");
    //  Serial.print("Load Voltage:  "); Serial.print(loadVoltageRectifier); Serial.println(" V");
    //  Serial.print("Current:       "); Serial.print(current_mA_rectifier); Serial.println(" mA");
    //  Serial.print("Power:         "); Serial.print(powerRectifier_mW); Serial.println(" mW");
    //  Serial.println("");

    // If capacitor voltage exceeds threshold, consider receiver charged
    if (loadVoltageCapacitor > chargeThreshold) {
      charged = true;
    }

    if (actualVoltageRectifier < 31 && loadVoltageCapacitor < 31) {
      validRead = true; //flag to determine if data successfully read
      //Remove comments to output in CSV format to serial. Only prints if both measurements are successful
      timer = millis();
      Serial.printf("%d,%f,%f,%f,%f\n", timer, actualVoltageRectifier, loadVoltageCapacitor, currentCapacitor_mA, powerCapacitor_mW);
    }
    break; //break while loop
  }
  delay(500);
}


