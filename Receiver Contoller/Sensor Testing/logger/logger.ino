/*
    Author: Jerome Hallett
    Board: Arduino Uno
    Code's purpose is to read sensor values from the INA219 in the receiver
    Measures the rectifier voltage, current, and state of charge of the receiver.

    INA219 sensor code:
    Based from https://learn.adafruit.com/adafruit-ina219-current-sensor-breakout/arduino-code
*/

//Import required libraries for INA219 & I2C
#include <Wire.h>
#include <Adafruit_INA219.h>

//Initialise two instances of the INA219
byte address_rect = 0x40;
byte address_cap = 0x41;
Adafruit_INA219 ina219_rectifier(address_rect);
Adafruit_INA219 ina219_capacitor(address_cap);

unsigned long timer; //Time since programme started running
bool charged = false; //Boolean of whether the receiver has been fully charged

// Checks availability of I2C data bus
byte checkI2C (byte &address) {
  byte error;
  Wire.beginTransmission(address); // checks I2C connection is available
  error = Wire.endTransmission(); //If is available, error = 0.
  return error;
}

void setup(void) {
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
  ina219_rectifier.begin();
  ina219_capacitor.begin();
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  //ina219.setCalibration_16V_400mA();

  Serial.println("Measuring voltage and current with INA219 ...");
}

void loop(void) {
  // Initialise rectifier sensor values
  float shuntvoltage_rectifier = 0;
  float busvoltage_rectifier = 0;
  float current_mA_rectifier = 0;
  float loadvoltage_rectifier = 0;
  float power_mW_rectifier = 0;
  float actualVoltage_rectifier = 0;

  // Initialise capacitor sensor values
  float shuntvoltage_capacitor = 0;
  float busvoltage_capacitor = 0;
  float current_mA_capacitor = 0;
  float loadvoltage_capacitor = 0;
  float power_mW_capacitor = 0;

  /*
    Now to read values from the INA219 sensors.
    If I2C unavailable due to interference, breaks the while loop to avoid process stalling.
    Read seperatley so valid values are independent
  */

  //Rectifier sensor read. Measures voltage from the potential divider and scales it up
  while (checkI2C (address_rect) == 0) {
    //Rectifier output
    shuntvoltage_rectifier = ina219_rectifier.getShuntVoltage_mV();
    busvoltage_rectifier = ina219_rectifier.getBusVoltage_V();
    loadvoltage_rectifier = busvoltage_rectifier + (shuntvoltage_rectifier / 1000);
    actualVoltage_rectifier = loadvoltage_rectifier * 4; // Potential divider ratio scaling
    timer = millis();
    Serial.print("Rectifier Values:   "); Serial.print(timer); Serial.print(","); Serial.println(actualVoltage_rectifier);
    break;
  }
  //Capacitor bank sensor read. Measures voltage and current
  while (checkI2C (address_cap) == 0) {
    // Across the capacitor bank
    shuntvoltage_capacitor = ina219_capacitor.getShuntVoltage_mV();
    busvoltage_capacitor = ina219_capacitor.getBusVoltage_V();
    current_mA_capacitor = ina219_capacitor.getCurrent_mA();
    power_mW_capacitor = ina219_capacitor.getPower_mW();
    loadvoltage_capacitor = busvoltage_capacitor + (shuntvoltage_capacitor / 1000);
    timer = millis();
    Serial.print("Capacitor Values:   "); Serial.print(timer); Serial.print(","); Serial.print(loadvoltage_capacitor); Serial.print(","); Serial.print(power_mW_capacitor); Serial.print(","); Serial.println(current_mA_capacitor);
    break;
  }

  //  Serial.print("Bus Voltage:   "); Serial.print(busvoltage_rectifier); Serial.println(" V");
  //  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage_rectifier); Serial.println(" mV");
  //  Serial.print("Load Voltage:  "); Serial.print(loadvoltage_rectifier); Serial.println(" V");
  //  Serial.print("Current:       "); Serial.print(current_mA_rectifier); Serial.println(" mA");
  //  Serial.print("Power:         "); Serial.print(power_mW_rectifier); Serial.println(" mW");
  //  Serial.println("");

  // If capacitor voltage exceeds threshold, consider receiver charged
  if (loadvoltage_capacitor > 14.5) {
    charged = true;
  }
  delay(500);
}


