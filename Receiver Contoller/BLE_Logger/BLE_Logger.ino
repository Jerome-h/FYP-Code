/*
    Author: Jerome Hallett
    Board: ESP32
    Requires files in same directory for OLED screen: fonts.h, images.h
    Code's purpose is to read sensor values from the INA219 in the receiver, checking I2C connection available to avoid process stalling.
    Then broadcasts data via a BLE server. Data broadcasted is the rectifier voltage, current, and state of charge of the receiver.
    Also displays information through the ESP32's OLED screen.

    INA219 sensor code:
    Based from https://learn.adafruit.com/adafruit-ina219-current-sensor-breakout/arduino-code
    BLE code:
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updates by chegewara
    With code from https://github.com/nkolban/esp32-snippets/issues/175
*/

// Import required libraries
#include <Arduino.h>
//INA219 sensor & I2C
#include <Wire.h>
#include <Adafruit_INA219.h>
//BLE
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
//EPS32 OLED Screen
#include <OLEDDisplayFonts.h>
#include <OLEDDisplay.h>
#include <OLEDDisplayUi.h>
#include <SSD1306Wire.h>
#include <SSD1306.h>
#include "images.h"
#include "fonts.h"
#include <string.h>

//User specfied inputs
uint8_t ID = 1; //ID of receiver device
uint8_t chargeThreshold = 14.5; //Threshold of capacitor voltage to consider receiver as deviceCharged

//Device setup
unsigned long timer; //Time since programme started running
uint8_t deviceCharged = 0; //Boolean of whether the receiver has been fully deviceCharged
uint8_t ledPin = 16; // Onboard LED reference

// The built-in OLED is a 128*64 mono pixel display
// i2c address = 0x3c
// SDA = 5
// SCL = 4
SSD1306 display(0x3c, 5, 4);

//Initialise two instances of the INA219
byte addressRect = 0x40;
byte addressCap = 0x41;
Adafruit_INA219 ina219_rectifier(addressRect);
Adafruit_INA219 ina219_capacitor(addressCap);

//BLE setup
bool deviceConnected = false;
uint8_t value = 0;

// https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.day_date_time.xml
// https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Specifications/Mesh/Xml/Characteristics/org.bluetooth.characteristic.voltage.xml
// https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.battery_level_state.xml

BLEDescriptor dateTimeDescriptor(BLEUUID((uint16_t)0x290C));
BLEDescriptor voltageDescriptor(BLEUUID((uint16_t)0x290C));
BLEDescriptor batteryLevelStateDescriptor(BLEUUID((uint16_t)0x290C));

BLEDescriptor dateTimeMeasurement(BLEUUID((uint16_t)0x2901));
BLEDescriptor voltageMeasurement(BLEUUID((uint16_t)0x2901));
BLEDescriptor batteryLevelStateMeasurement(BLEUUID((uint16_t)0x2901));

BLECharacteristic dateTimeCharacteristic(BLEUUID((uint16_t)0x2A0A),  // standard 16-bit characteristic UUID
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic voltageCharacteristic(BLEUUID((uint16_t)0x2B18),  // standard 16-bit characteristic UUID
                                        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic batteryLevelStateCharacteristic(BLEUUID((uint16_t)0x2A1B),  // standard 16-bit characteristic UUID
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

//Function to initialise BLE server
void InitBLE() {
  BLEDevice::init("Receiver1");

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  Serial.println("Added callbacks");
  // Create the BLE Service
  BLEService *pService = pServer->createService(BLEUUID((uint16_t)0x181A));
  Serial.println("Created service");
  // Add Charcteristics
  pService->addCharacteristic(&dateTimeCharacteristic);
  pService->addCharacteristic(&voltageCharacteristic);
  pService->addCharacteristic(&batteryLevelStateCharacteristic);
  Serial.println("Added characteristics");

  dateTimeDescriptor.setValue("Date Time eg 29-May-2019 12:00:00");
  dateTimeCharacteristic.addDescriptor(&dateTimeDescriptor);
  voltageDescriptor.setValue("Voltage eg 10");
  voltageCharacteristic.addDescriptor(&voltageDescriptor);
  batteryLevelStateDescriptor.setValue("Battery Level State eg 0/1");
  batteryLevelStateCharacteristic.addDescriptor(&batteryLevelStateDescriptor);
  Serial.println("Added descriptors");
  dateTimeCharacteristic.addDescriptor(new BLE2902());
  voltageCharacteristic.addDescriptor(new BLE2902());
  batteryLevelStateCharacteristic.addDescriptor(new BLE2902());
  Serial.println("Added 2902");

  pService->start();
  Serial.println("Service started");
  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Advertising started");
}

// Function to check availability of I2C data bus
byte checkI2C (byte &address) {
  byte error;
  Wire.beginTransmission(address); // checks I2C connection is available
  error = Wire.endTransmission(); //If is available, error = 0.
  return error;
}

void setup() {
  pinMode(ledPin, OUTPUT); // Onboard LED reference

  // Initialize the OLED display
  display.init();
  display.flipScreenVertically();
  display.setFont(Roboto_Medium_14);
  display.drawString(0, 0, "BLE Server");
  display.display();
  digitalWrite(ledPin, LOW); //NB ESP LED pin is active low
  delay(1500);
  display.clear();

  Serial.begin(115200);
  Serial.println("Starting BLE");

  // Create the BLE Device
  InitBLE();
  Serial.println("Waiting for a client connection to notify...");

  // Initialize the INA219.
  uint32_t currentFrequency;
  Wire.begin(21, 22);
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

void loop() {
  // Initialise rectifier sensor values
  float shuntVoltageRectifier = 0;
  float busVoltageRectifier = 0;
  float currentRectifier_mA = 0;
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
    shuntVoltageRectifier = ina219_rectifier.getShuntVoltage_mV();
    busVoltageRectifier = ina219_rectifier.getBusVoltage_V();
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
    //  Serial.print("Current:       "); Serial.print(currentRectifier_mA); Serial.println(" mA");
    //  Serial.print("Power:         "); Serial.print(powerRectifier_mW); Serial.println(" mW");
    //  Serial.println("");

    // If capacitor voltage exceeds threshold, consider receiver deviceCharged
    if (loadVoltageCapacitor > chargeThreshold) {
      deviceCharged = 1;
    }
    if (actualVoltageRectifier < 31 && loadVoltageCapacitor < 31) {
      validRead = true; //flag to determine if data successfully read
      //  Remove comment to output in CSV format. Only prints if both measurements are successful
      timer = millis();
      Serial.print(timer); Serial.print(","); Serial.print(actualVoltageRectifier); Serial.print(","); Serial.print(loadVoltageCapacitor); Serial.print(","); Serial.print(currentCapacitor_mA); Serial.print(","); Serial.println(powerCapacitor_mW);
    }
    break; //break while loop
  }

  /*
    BLE server update of new valid measurements and state of charge
  */
  if (deviceConnected && validRead == true) {
    digitalWrite(ledPin, LOW); //NB ESP LED pin is active low
    Serial.printf("*** Notify: %d ***\n", value);

    char cTimeStr[30]; //dummy timeStamp
    sprintf(cTimeStr, "%d-%s-%d %d:%d:%d", 29, "May", 2019, 12, 00, 00); // Format time for suitable use in thingSpeak MATLAB visualisation (ISO 8601) http://www.cplusplus.com/reference/ctime/strftime/ . dummy timeStamp. To update with RTC measurement
    dateTimeCharacteristic.setValue(cTimeStr);
    dateTimeCharacteristic.notify();
    char cVolt[6]; //rectifier voltage
    sprintf(cVolt, "%0.2f", actualVoltageRectifier);
    voltageCharacteristic.setValue(cVolt);
    voltageCharacteristic.notify();
    char cState[6];
    sprintf(cState, "%d", deviceCharged);
    batteryLevelStateCharacteristic.setValue(cState);
    batteryLevelStateCharacteristic.notify();
    Serial.printf("    Values: %s %0.2f %0.2f\n", cTimeStr, actualVoltageRectifier, deviceCharged);
    display.drawString(0, 0, "Data Sent!");
    display.display();
    value++;
  }

  digitalWrite(ledPin, HIGH); //NB ESP LED pin is active low
  delay(500);
  display.clear();
}
