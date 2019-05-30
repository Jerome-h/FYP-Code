/*
    Author: Jerome Hallett
    Board: ESP32
    Code's purpose is to read sensor values from the INA219 in the receiver, and then broadcast via a BLE server.
    Data broadcasted is the rectifier voltage, current, and state of charge of the receiver.

    INA219 sensor code:
    Based from https://learn.adafruit.com/adafruit-ina219-current-sensor-breakout/arduino-code
    BLE code:
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updates by chegewara
    With code from https://github.com/nkolban/esp32-snippets/issues/175
*/

// Import required libraries
//INA219 Sensor & I2C
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
uint8_t chargeThreshold = 14.5; //Threshold of capacitor voltage to consider receiver as charged

//Device setup
unsigned long timer; //Time since programme started running
bool charged = false; //Boolean of whether the receiver has been fully charged
uint8_t ledPin = 16; // Onboard LED reference

// The built-in OLED is a 128*64 mono pixel display
// i2c address = 0x3c
// SDA = 5
// SCL = 4
SSD1306 display(0x3c, 5, 4);

//Initialise two instances of the INA219
byte address_rect = 0x40;
byte address_cap = 0x41;
Adafruit_INA219 ina219_rectifier(address_rect);
Adafruit_INA219 ina219_capacitor(address_cap);

//BLE setup
bool deviceConnected = false;
uint8_t value = 0;

// https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.temperature_measurement.xml
// https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.humidity.xml

BLEDescriptor dateTimeDescriptor(BLEUUID((uint16_t)0x290C));
BLEDescriptor temperatureDescriptor(BLEUUID((uint16_t)0x290C));
BLEDescriptor humidityDescriptor(BLEUUID((uint16_t)0x290C));
BLEDescriptor dateTimeMeasurement(BLEUUID((uint16_t)0x2901));
BLEDescriptor temperatureMeasurement(BLEUUID((uint16_t)0x2901));
BLEDescriptor humidityMeasurement(BLEUUID((uint16_t)0x2901));

BLECharacteristic dateTimeCharacteristic(BLEUUID((uint16_t)0x2A0A),  // standard 16-bit characteristic UUID
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic temperatureCharacteristic(BLEUUID((uint16_t)0x2A6E),  // standard 16-bit characteristic UUID
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic humidityCharacteristic(BLEUUID((uint16_t)0x2A6F),  // standard 16-bit characteristic UUID
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
  pService->addCharacteristic(&temperatureCharacteristic);
  pService->addCharacteristic(&humidityCharacteristic);
  Serial.println("Added characteristics");

  dateTimeDescriptor.setValue("Date Time eg 29-May-2019 12:00:00");
  dateTimeCharacteristic.addDescriptor(&dateTimeDescriptor);
  temperatureDescriptor.setValue("Temperature -50-50°C");
  temperatureCharacteristic.addDescriptor(&temperatureDescriptor);
  humidityDescriptor.setValue("Humidity 0-100%");
  humidityCharacteristic.addDescriptor(&humidityDescriptor);
  Serial.println("Added descriptors");
  dateTimeCharacteristic.addDescriptor(new BLE2902());
  temperatureCharacteristic.addDescriptor(new BLE2902());
  humidityCharacteristic.addDescriptor(new BLE2902());
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
  digitalWrite(ledPin, LOW); //NB LED pin is active low
  delay(1500);
  display.clear();

  Serial.begin(115200);
  Serial.println("Starting BLE");

  // Create the BLE Device
  InitBLE();
  Serial.println("Waiting for a client connection to notify...");
  
  // Initialize the INA219.
  uint32_t currentFrequency;
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
float fTemp = 0;
float fHumidity = 40;

void loop() {
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
    Read seperatley so valid values can be independently broadcasted
  */

  //Rectifier sensor read. Measures voltage from the potential divider and scales it up
  while (checkI2C (address_rect) == 0) {
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
    shuntvoltage_capacitor = ina219_capacitor.getShuntVoltage_mV();
    busvoltage_capacitor = ina219_capacitor.getBusVoltage_V();
    current_mA_capacitor = ina219_capacitor.getCurrent_mA();
    power_mW_capacitor = ina219_capacitor.getPower_mW();
    loadvoltage_capacitor = busvoltage_capacitor + (shuntvoltage_capacitor / 1000);
    timer = millis();
    Serial.print("Capacitor Values:   "); Serial.print(timer); Serial.print(","); Serial.print(loadvoltage_capacitor); Serial.print(","); Serial.print(power_mW_capacitor); Serial.print(","); Serial.println(current_mA_capacitor);
    break;
  }
    // If capacitor voltage exceeds threshold, consider receiver charged
  if (loadvoltage_capacitor > chargeThreshold) {
    charged = true;
  }

  /*
    BLE server update of new valid measurements and state of charge
  */
  if (deviceConnected) {
    digitalWrite(ledPin, LOW); //NB LED pin is active low
    Serial.printf("*** Notify: %d ***\n", value);
    fTemp += 1.1;
    if (fTemp > 50) {
      fTemp = 0;
    }
    fHumidity += 1.1;
    if (fHumidity > 100) {
      fHumidity = 40;
    }

    char cTimeStr[30]; // dummy timeStamp
    sprintf(cTimeStr, "%d-%s-%d %d:%d:%d", 29, "May", 2019, 12, 00, 00); // dummy timeStamp. To update with RTC measurement
    dateTimeCharacteristic.setValue(cTimeStr);
    dateTimeCharacteristic.notify();
    char cTemp[6];
    sprintf(cTemp, "%0.2f", fTemp);
    temperatureCharacteristic.setValue(cTemp);
    temperatureCharacteristic.notify();
    char cHumidity[6];
    sprintf(cHumidity, "%0.2f", fHumidity);
    humidityCharacteristic.setValue(cHumidity);
    humidityCharacteristic.notify();
    Serial.printf("    Values: %s %0.2f %0.2f\n", cTimeStr, fTemp, fHumidity);
  }
  digitalWrite(ledPin, HIGH); //NB LED pin is active low
  delay(2000);
  value++;
}
