/*
    Author: Jerome Hallett
    Board: Arduino Uno
    Code's purpose is to read sensor values from the INA219 in the receiver
    Measures the rectifier voltage, current, and state of charge of the receiver.

    INA219 sensor code:
    Based from https://learn.adafruit.com/adafruit-ina219-current-sensor-breakout/arduino-code
*/

#include <Wire.h>
#include <Adafruit_INA219.h>

//Initialise two instances of the INA219
byte addressRect = 0x40;
byte addressCap = 0x41;
Adafruit_INA219 ina219Rectifier(addressRect);
Adafruit_INA219 ina219Capacitor(addressCap);

unsigned long timer;

void setup(void)
{
  Serial.begin(9600);
  while (!Serial) {
    // will pause Zero, Leonardo, etc until serial console opens
    delay(1);
  }

  uint32_t currentFrequency;

  Serial.println("Hello!");

  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  ina219Rectifier.begin();
  ina219Capacitor.begin();
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  //ina219.setCalibration_16V_400mA();

  Serial.println("Measuring voltage and current with INA219 ...");
}

void loop(void)
{
  // Initialise rectifier values
  float shuntVoltageRectifier = 0;
  float busVoltageRectifier = 0;
  float currentRectifier_mA = 0;
  float loadVoltageRectifier = 0;
  float powerRectifier_mW = 0;
  float actualVoltageRectifier = 0;

  //Rectifier output
  shuntVoltageRectifier = ina219Rectifier.getShuntVoltage_mV();
  busVoltageRectifier = ina219Rectifier.getBusVoltage_V();
  currentRectifier_mA = ina219Rectifier.getCurrent_mA();
  powerRectifier_mW = ina219Rectifier.getPower_mW();
  loadVoltageRectifier = busVoltageRectifier + (shuntVoltageRectifier / 1000);
  actualVoltageRectifier = loadVoltageRectifier * 4; // divider ratio scaling

  // Initialise capacitor values
  float shuntVoltageCapacitor = 0;
  float busVoltageCapacitor = 0;
  float currentCapacitor_mA = 0;
  float loadVoltageCapacitor = 0;
  float powerCapacitor_mW = 0;

  // Across the capacitor bank
  shuntVoltageCapacitor = ina219Capacitor.getShuntVoltage_mV();
  busVoltageCapacitor = ina219Capacitor.getBusVoltage_V();
  currentCapacitor_mA = ina219Capacitor.getCurrent_mA();
  powerCapacitor_mW = ina219Capacitor.getPower_mW();
  loadVoltageCapacitor = busVoltageCapacitor + (shuntVoltageCapacitor / 1000);

  //  Serial.print("Bus Voltage:   "); Serial.print(busVoltageRectifier); Serial.println(" V");
  //  Serial.print("Shunt Voltage: "); Serial.print(shuntVoltageRectifier); Serial.println(" mV");
  //  Serial.print("Load Voltage:  "); Serial.print(loadVoltageRectifier); Serial.println(" V");
  //  Serial.print("Current:       "); Serial.print(currentRectifier_mA); Serial.println(" mA");
  //  Serial.print("Power:         "); Serial.print(powerRectifier_mW); Serial.println(" mW");
  //  Serial.println("");

  //  Remove comments to output in CSV format
  timer = millis();
  Serial.printf("%d,%f,%f,%f,%f\n", timer, actualVoltageRectifier, loadVoltageCapacitor, currentCapacitor_mA, powerCapacitor_mW);
  delay(500);
}
