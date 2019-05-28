/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updates by chegewara
*/
// Import required libraries
#include <Wire.h>
#include <OLEDDisplayFonts.h>
#include <OLEDDisplay.h>
#include <OLEDDisplayUi.h>
#include <SSD1306Wire.h>
#include <SSD1306.h>
#include "images.h"
#include "fonts.h"

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

int ID = 1; //ID of sensor device

// The built-in OLED is a 128*64 mono pixel display
// i2c address = 0x3c
// SDA = 5
// SCL = 4
SSD1306 display(0x3c, 5, 4);

uint8_t ledPin = 16; // Onboard LED reference

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLECharacteristic pCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

void setup() {

  pinMode(ledPin, OUTPUT);
  // Initialize the display
  display.init();
  display.flipScreenVertically();
  display.setFont(Roboto_Medium_14);
  display.drawString(10, 20, "BLE Server");
  display.display();
  delay(1500);
  display.clear();

  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  //Name device
  BLEDevice::init("Drone");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  //  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
  //                                         CHARACTERISTIC_UUID,
  //                                         BLECharacteristic::PROPERTY_READ |
  //                                         BLECharacteristic::PROPERTY_WRITE
  //                                       );
  pService->addCharacteristic(&pCharacteristic);
  pCharacteristic.addDescriptor(new BLE2902());
  //  pCharacteristic->setValue("Hello World!");
  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}

void loop() {
  char TxStr[10];
  sprintf(TxStr, "%d", ID);
  pCharacteristic.setValue(TxStr);

  //For display, LED and serial monitor indication
  digitalWrite(ledPin, LOW); //NB LED pin is active low
  delay(100);
  char dispStr[10];
  sprintf(dispStr, "ID: %d", ID);
  display.drawString(10, 20, dispStr);
  display.display();
  Serial.println(dispStr);

  digitalWrite(ledPin, HIGH); //NB LED pin is active low
  delay(2000);
  display.clear();
}
