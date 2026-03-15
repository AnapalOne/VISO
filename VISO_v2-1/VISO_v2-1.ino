/* VISO v2.1 Source Code
 *
 * Current prototype build supports BPM, temperature (in C) monitoring.
 *    and c
 * Values are transmitted to a bluetooth-connected device using VISOApp.
 *
 * Sensors used: MAX30102 for BPM and SPO2, and MLX90614 for temperature
 * NOTE: Accuracy and reliablity of sensors are not guaranteed since these are not
 *        meant for medical use. User discretion is advised when used in clinical
 *        settings.
 */

/*
 * Copyright (C) 2025 Andrei Jose R. Embarque
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <Wire.h>
#include <stdio.h>
#include <string.h>
#include "stdlib_noniso.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <ArduinoJson.h>

#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>

#include <Adafruit_MLX90614.h>
#include <MAX30105.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2901.h>

#define SDA_PIN 8
#define SCL_PIN 9
#define OLED_ADDRESS 0x3C

#define BUFFER_SIZE 125
#define SAMPLE_RATE 100
#define TEMP_CALIBRATION 2

#define VISO_SERVICE_UUID "e6886439-57e7-4e12-bf9d-de7321c71ae5"
#define VISO_CHARACTERISTIC_UUID "650bf973-937b-4731-a6e6-a87db2f234e5"

BLEServer *VISO_Server = NULL;
BLEService *VISO_Service = NULL;

BLECharacteristic *VISO_Characteristic = NULL;
BLE2901 *VISODescriptor = NULL;
BLEAdvertising *VISO_Advertising = NULL;

Adafruit_MLX90614 mlx = Adafruit_MLX90614();
MAX30105 max30102;
SSD1306AsciiWire oled;

const char TITLE[20] = "VISO v2.1";
char VISO_Address[50];

float bodyTemp = 0.0;
float bpm = 0.0;
float spo2 = 0.0;

bool tempReady = false;
bool jsonReady = false;

int bufferIndex = 0;
char jsonOutput[256];  // BLE JSON output

unsigned long lastTempTime = 0;
unsigned long lastDisplayTime = 0;
unsigned long lastJsonTime = 0;

int32_t spo2Calc;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;

uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];

bool deviceConnected = false;
bool oldDeviceConnected = false;
int beatThresh = 0; 

unsigned long bluetoothDelay = 0;

class VISOCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *VISO_Server) {
    Serial.printf("Connected!\n");
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *VISO_Server) {
    Serial.printf("Disconnected.\n Restart advertising device %s\n", TITLE);
    deviceConnected = false;
  }
};

/*
 * Start bluetooth process to become available using the device name in TITLE.
 * Bluetooth service includes a characteristic to send JSON packets to VISOApp
 *  with the notify property.
 */
void startBT(const char btName[50]) {
  BLEDevice::init(btName);
  String VISO_Address = BLEDevice::getAddress().toString();

  VISO_Server = BLEDevice::createServer();
  VISO_Server->setCallbacks(new VISOCallbacks());

  VISO_Service = VISO_Server->createService(VISO_SERVICE_UUID);

  VISO_Characteristic = VISO_Service->createCharacteristic(
    VISO_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY);

  VISODescriptor = new BLE2901();
  VISODescriptor->setDescription("VISO v2.1 - SPO2, BPM, Temperature");
  VISODescriptor->setAccessPermissions(ESP_GATT_PERM_READ);

  VISO_Characteristic->addDescriptor(VISODescriptor);

  VISO_Service->start();

  VISO_Advertising = BLEDevice::getAdvertising();
  VISO_Advertising->addServiceUUID(VISO_SERVICE_UUID);
  VISO_Advertising->setScanResponse(false);
  VISO_Advertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.printf("Started advertising %s bluetooth device with address %s.\n", btName, VISO_Address.c_str());
}

/*
 * Display format for the LCD monitor.
 */
void updateDisplay() {

  oled.setCursor(4, 0);
  oled.print("VISO v2.1");

  oled.setCursor(4, 2);
  oled.print("Temp:               ");
  oled.setCursor(50, 2);
  if (tempReady) {
    oled.print(bodyTemp, 2);
    oled.print(" C");
  } else {
    oled.print("N/A  ");
  }

  oled.setCursor(4, 4);
  oled.print("BPM:               ");
  oled.setCursor(50, 4);
  if (validHeartRate)
    oled.print(bpm, 0);
  else
    oled.print("N/A");

  oled.setCursor(4, 6);
  oled.print("SpO2:               ");
  oled.setCursor(50, 6);
  if (validSPO2)
    oled.print(spo2);
  else
    oled.print("N/A");
}

/*
 * Transmit values to connected ESP32 device thru bluetooth.
 * Also checks if the device is connected, is connecting, or disconnected.
 */
void updateBT() {
  // if esp32 is connected
  if (deviceConnected) {
    Serial.printf("Sending data to VISOApp... %s\n", jsonOutput);
    VISO_Characteristic->setValue(jsonOutput);
    VISO_Characteristic->notify();
  }

  // if esp32 has disconnected
  if (!deviceConnected && oldDeviceConnected) {
    VISO_Server->startAdvertising();  // restart advertising
    oldDeviceConnected = deviceConnected;
  }

  // if esp32 is connecting to a device
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
}

/*
 * Get temperature values in celsius from MLX90614 temperature sensor.
 * Temperature, when measured using the forehead, does not indicate
 *   the actual core temperature, so add 1.6 Celsius to temperature readings.
 */
void readTemperature() {
  float temp = mlx.readObjectTempC();

  if (!isnan(temp) && temp >= 32 - TEMP_CALIBRATION && temp <= 43 - TEMP_CALIBRATION) {
    bodyTemp = temp + TEMP_CALIBRATION;
    tempReady = true;
  } else {
    tempReady = false;
  }
}

/*
 * Get BPM from MAX30102 integrated pulse oximetry and heart-rate monitor module.
 * Values extracted from the sensor module are in raw infrared (IR) values, which are
 *   converted to (almost) accurate BPM and SPO2 values.
 */
void readMAX30102() {
  max30102.check();

  while (max30102.available()) {
    uint32_t irValue = max30102.getIR();
    uint32_t redValue = max30102.getRed();
    max30102.nextSample();

    // Is finger detected? Uses raw IR data to determine, with a threshold of 50000. 
    if (irValue < 50000) {
      validHeartRate = 0;
      validSPO2 = 0;
      bpm = 0;
      spo2 = 0;
      bufferIndex = 0;
      return;
    }

    irBuffer[bufferIndex] = irValue;
    redBuffer[bufferIndex] = redValue;
    bufferIndex++;

    if (bufferIndex >= BUFFER_SIZE) {
      maxim_heart_rate_and_oxygen_saturation(
        irBuffer,
        BUFFER_SIZE,
        redBuffer,
        &spo2Calc,
        &validSPO2,
        &heartRate,
        &validHeartRate);

      if (validHeartRate && heartRate > 50 && heartRate < 180)
        bpm = (bpm * 0.65) + (heartRate * 0.35);

      if (validSPO2 && spo2Calc > 80 && spo2Calc <= 100)
        spo2 = spo2Calc;

      bufferIndex = 0;
      jsonReady = true;
    }
  }
}

void createJson() {
  StaticJsonDocument<256> doc;

  doc["temperature"] = tempReady ? bodyTemp : 0;
  doc["bpm"] = validHeartRate ? bpm : 0;
  doc["spo2"] = validSPO2 ? spo2 : 0;

  serializeJson(doc, jsonOutput);
}

void setup() {
  Serial.begin(115200);

  delay(1000);  // ESP32 is not able to print info during setup, add a 1s delay before doing anything
  Serial.printf("VISO v2.1\n\n");
  Serial.printf("This program is licensed under GNU GPL v3.0 and comes with ABSOLUTELY NO WARRANTY.\n");
  Serial.printf("This is free software, and you are welcome to redistribute it, under certain conditions.\n\n");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);  // MLX safe speed

  // OLED init
  oled.begin(&Adafruit128x64, OLED_ADDRESS);
  oled.setFont(System5x7);
  oled.clear();
  oled.println("Starting...");
  delay(1000);

  // MLX90614 init
  if (!mlx.begin()) {
    oled.setCursor(4, 4);
    oled.println("MLX ERROR!");
    Serial.println("MLX ERROR!");
    while (1);
  }

  // MAX30102 init
  if (!max30102.begin(Wire, I2C_SPEED_STANDARD)) {
    oled.setCursor(4, 4);
    oled.println("MAX ERROR!");
    Serial.println("MAX ERROR!");
    while (1);
  }

  max30102.setup();
  max30102.setPulseAmplitudeRed(0x24);
  max30102.setPulseAmplitudeIR(0x24);

  oled.clear();

  startBT(TITLE);
}

void loop() {
  unsigned long time = millis();

  // Temp every 500ms
  if (time - lastTempTime >= 500) {
    lastTempTime = time;
    readTemperature();
  }

  // Continuous MAX reading
  readMAX30102();

  // Display every 500ms
  if (time - lastDisplayTime >= 500) {
    lastDisplayTime = time;
    updateDisplay();
  }

  // Transmit values to connected bluetooth device every 7s
  if (time - bluetoothDelay >= 1000) {
    bluetoothDelay = time;
    createJson();
    updateBT();
  }
}