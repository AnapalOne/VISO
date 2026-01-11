/* VISO v1.2 Source Code for Device 2
 *
 * Current prototype build supports BPM and temperature (in C) monitoring.
 * Values are transmitted to a bluetooth-connected device using VISOApp.
 * Future releases will include SpO2, blood pressure, and IoT functionality.
 *
 * Sensors used: KY-039 for BPM and DS18B20 for temperature
 * NOTE: Accuracy and reliablity of sensors are not guaranteed since these are not
 *        meant for medical use. Future prototypes will use more accurate and
 *        established sensors, like the MAX30102 and MAX30205.
 *
 * TODO: reset graphs from VISOApp during disconnect and reconnect.
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
#include <U8g2lib.h>
#include <stdio.h>
#include <string.h>
#include "stdlib_noniso.h"
#include <OneWire.h>
#include <DallasTemperature.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2901.h>

#define samp_siz 4
#define rise_threshold 5

#define ONE_WIRE_BUS 1
#define DEBUG 0

#define VISO_SERVICE_UUID "e6886439-57e7-4e12-bf9d-de7321c71ae5"
#define VISO_BPM_CHARACTERISTIC_UUID "650bf973-937b-4731-a6e6-a87db2f234e5"
#define VISO_TEMP_CHARACTERISTIC_UUID "bce56131-4fcf-48a9-a0c9-f93144337c19"

BLEServer *VISO_Server = NULL;
BLEService *VISO_Service = NULL;

BLECharacteristic *bpmCharacteristic = NULL;
BLECharacteristic *tempCharacteristic = NULL;

BLE2901 *bpmDescriptor = NULL;
BLE2901 *tempDescriptor = NULL;

BLEAdvertising *VISO_Advertising = NULL;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

char btMessage[50];
char bpm_strBT[50];
char tempC_strBT[50];

int pinBPM = 2;
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

const char TITLE[20] = "VISO v1.2 Dev 2";
char VISO_Address[50];

char bpm_str[50];
char temp_str[50];
char rawIR_str[50];
char temp_strPrev[50] = "N/A";
char bpm_strPrev[50] = "N/A";
bool hasValidTemp = false;
bool hasValidBPM = false;

float reads[samp_siz], sum;
float avgBPM;
long int ptr;
float last, reader, start;
float first, second, third, before;
bool rising;
int rise_count;
long int last_beat;
bool heartbeat_detected;
bool deviceConnected = false;
bool oldDeviceConnected = false;

float bpm, tempC;

unsigned long bpmDelay = 0;
unsigned long tempDelay = 0;
unsigned long displayDelay = 0;
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
 * Bluetooth service includes two characteristics with the notify property:
 *  1. BPM Characteristic; and
 *  2. Temperature Characteristic  
 */
void startBT (const char btName[50]) 
{
  BLEDevice::init(btName);
  String VISO_Address = BLEDevice::getAddress().toString();

  VISO_Server = BLEDevice::createServer();
  VISO_Server->setCallbacks(new VISOCallbacks());

  VISO_Service = VISO_Server->createService(VISO_SERVICE_UUID);

  bpmCharacteristic = VISO_Service->createCharacteristic(
    VISO_BPM_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY);

  bpmDescriptor = new BLE2901();
  bpmDescriptor->setDescription("BPM Sensor");
  bpmDescriptor->setAccessPermissions(ESP_GATT_PERM_READ);

  bpmCharacteristic->addDescriptor(bpmDescriptor);

  tempCharacteristic = VISO_Service->createCharacteristic(
    VISO_TEMP_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY);

  tempDescriptor = new BLE2901();
  tempDescriptor->setDescription("Temperature Sensor");
  tempDescriptor->setAccessPermissions(ESP_GATT_PERM_READ);

  tempCharacteristic->addDescriptor(tempDescriptor);

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
 * Converts value arguments to string first, then transmits to the display.
 */
void display (const char name[50], float bpm, float temp, double rawIR) 
{
  dtostrf(bpm, 1, 2, bpm_str);
  dtostrf(temp, 1, 2, temp_str);
  dtostrf(rawIR, 1, 2, rawIR_str);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);

  int halfwidth = u8g2.getDisplayWidth() / 2;

  // Print device name in line 1, centered.
  u8g2.drawStr(halfwidth - 36, 12, name);
  
  // Draw horizontal line in line 2
  u8g2.drawLine(0, 12, 128, 12);

  // Display temperature in line 3
  u8g2.drawStr(0, 24, "Temp:");
  if (temp == -127.00) {
    if (hasValidTemp) u8g2.drawStr(halfwidth, 24, temp_strPrev);
    else u8g2.drawStr(halfwidth, 24, "N/A");
  } else {
    strcpy(temp_strPrev, temp_str);
    hasValidTemp = true;
    u8g2.drawStr(halfwidth, 24, temp_str);
  }

  // Display BPM in line 4
  u8g2.drawStr(0, 36, "BPM:");
  if (heartbeat_detected) {
    if (hasValidBPM) u8g2.drawStr(halfwidth, 36, bpm_strPrev);
    else u8g2.drawStr(halfwidth, 36, "N/A");
  } else {
    strcpy(bpm_strPrev, bpm_str);
    hasValidBPM = true;
    u8g2.drawStr(halfwidth, 36, bpm_str);
  }

  // Display raw IR to line 5
  u8g2.drawStr(0, 48, "Raw IR:");
  u8g2.drawStr(halfwidth, 48, rawIR_str);

  u8g2.sendBuffer();
}

/*
 * Transmit values to connected ESP32 device thru bluetooth. 
 * Also checks if the device is connected, is connecting, or disconnected.
 */
void updateBT(char bpm_strPrev[50], char temp_strPrev[50])
{
  // if esp32 is connected
  if (deviceConnected) { 
    Serial.printf("Sending temp value... %s\n", temp_strPrev);
    tempCharacteristic->setValue(temp_strPrev);
    tempCharacteristic->notify();

    Serial.printf("Sending bpm value... %s\n", bpm_strPrev);
    bpmCharacteristic->setValue(bpm_strPrev);
    bpmCharacteristic->notify();
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
 * Get temperature values in celsius from DS18B20 temperature sensor.
 * Signal pin is connected to pin 1, and values are fetched using
 *  OneWire and DallasTemperature libraries.
 */
float getTempC () 
{
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
}

/*
 * Get BPM from KY-039 Infrared Sensor/Heart Rate Monitor.
 * Values extracted from the sensor module are in raw infrared (IR) values.
 * This funciton converts raw IR to a stable BPM value using Johan_Ha's algorithm
 *  found at https://projecthub.arduino.cc/Johan_Ha/from-ky-039-to-heart-rate-8c660b.
 */
float getBPM()
{
  int n = 0;
  start = millis();
  reader = 0.;

  // Get the average IR reading during a 16ms period
  //  to filter external light noise
  while (millis() - start < 16) {
    reader += analogRead(pinBPM);
    n++;
  }
  reader /= n;

  sum -= reads[ptr];
  sum += reader;
  reads[ptr] = reader;
  last = sum / samp_siz;

  // Check for a rising curve. If not, then it's falling.
  if (last > before) {
    rise_count++;

    // If a rising curve is detected, which implies a valid heart beat, 
    //  record the time since the last heart beat, while keeping track 
    //  of the two previous recordings.
    if (!rising && rise_count > rise_threshold) {
      heartbeat_detected = true;
      rising = true;
      float beatTime = millis() - last_beat;
      last_beat = millis();

      bpm = 60000. / (0.4 * beatTime + 0.3 * second + 0.3 * third); // calculate weighted avg of BPM
      avgBPM = (0.1 * bpm) + ((1 - 0.1) * avgBPM); // exponential smoothing of average BPM

      third = second;
      second = beatTime;
    } else {
      heartbeat_detected = false;
    }
  } 
  else {
    rising = false;
    rise_count = 0;
  }

  before = last;
  ptr++;
  ptr %= samp_siz;

  return avgBPM;
}

void setup() 
{
  Serial.begin(115200);

  delay(1000); // ESP32 is not able to print info during setup, add a 1s delay before doing anything   
  Serial.printf("VISO v1.2 Device 2\n\n");
  Serial.printf("This program is licensed under GNU GPL v3.0 and comes with ABSOLUTELY NO WARRANTY.\n");
  Serial.printf("This is free software, and you are welcome to redistribute it, under certain conditions.\n\n");

  // Setup pins for the LCD display and start U8G2lib. 
  // SDA is connected to pin 8 and SCL is connected to pin 4.
  Wire.begin(8, 4);
  u8g2.begin();
  
  sensors.begin();
  for (int i = 0; i < samp_siz; i++) reads[i] = 0;

  startBT(TITLE);
}

void loop() {
  unsigned long time = millis();
  double rawIR = analogRead(pinBPM);

  // Get BPM every 16ms
  if (time - bpmDelay >= 16) {
    bpmDelay = time;
    avgBPM = getBPM();
  }

  // Get temperature in celsius every 1s
  if (time - tempDelay >= 1000) {
    tempDelay = time;

    tempC = getTempC();
  }

  // Display values to the LCD screen every 200ms
  if (time - displayDelay >= 200) {
    displayDelay = time;

    display(TITLE, avgBPM, tempC, rawIR);
  }

  // Transmit values to connected bluetooth device every 500ms
  if (time - bluetoothDelay >= 500) {
    bluetoothDelay = time;

    updateBT(bpm_strPrev, temp_strPrev);
  }
}
