/* VISO v1.0 Source Code
 *
 * Current prototype build supports BPM and temperature (in C) monitoring.
 * Future releases will include SpO2, blood pressure, and IoT functionality.
 *
 * Sensors used: KY-039 for BPM and DS18B20 for temperature
 * NOTE: Accuracy and reliablity of sensors are not guaranteed since these are not
 *        meant for medical use. Future prototypes will use more accurate and
 *        established sensors, like the MAX30102 and MAX30205.
 *
 * TODO: Bluetooth functionality
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

#define samp_siz 4
#define rise_threshold 5
#define ONE_WIRE_BUS 1
#define DEBUG 0

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

char btMessage[50];
char bpm_strBT[50];
char tempC_strBT[50];

int pinBPM = 2;
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

const char TITLE[20] = "VISO v1.0"; 

char bpm_str[50];
char tempC_str[50];
char rawIR_str[50];
char tempC_str_prev[50] = "N/A";
char bpm_str_prev[50] = "N/A";
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

float bpm, tempC;

unsigned long bpmDelay = 0;
unsigned long tempDelay = 0;
unsigned long displayDelay = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 4);
  u8g2.begin();
  sensors.begin();
  for (int i = 0; i < samp_siz; i++) reads[i] = 0;
}

void loop() {
  unsigned long now = millis();
  double rawIR = analogRead(pinBPM);

  if (now - bpmDelay >= 16) {
    bpmDelay = now;
    int n = 0;
    start = millis();
    reader = 0.;
    while (millis() - start < 16) {
      reader += analogRead(pinBPM);
      n++;
    }
    reader /= n;

    sum -= reads[ptr];
    sum += reader;
    reads[ptr] = reader;
    last = sum / samp_siz;

    if (last > before) {
      rise_count++;
      if (!rising && rise_count > rise_threshold) {
        rising = true;
        float beatTime = millis() - last_beat;
        last_beat = millis();

        bpm = 60000. / (0.4 * beatTime + 0.3 * second + 0.3 * third);
        avgBPM = (0.1 * bpm) + ((1 - 0.1) * avgBPM);

        third = second;
        second = beatTime;
      }
    } else {
      rising = false;
      rise_count = 0;
    }

    before = last;
    ptr++;
    ptr %= samp_siz;
  }

  if (now - tempDelay >= 1000) {
    tempDelay = now;
    sensors.requestTemperatures();

    tempC = sensors.getTempCByIndex(0);
  }

  if (now - displayDelay >= 200) {
    displayDelay = now;

    dtostrf(avgBPM, 1, 2, bpm_str);
    dtostrf(tempC, 1, 2, tempC_str);
    dtostrf(rawIR, 1, 2, rawIR_str);

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);

    int halfwidth = u8g2.getDisplayWidth() / 2;

    u8g2.drawStr(halfwidth - 24, 12, TITLE);
    u8g2.drawLine(0, 12, 128, 12);

    u8g2.drawStr(0, 24, "Temp:");
    if (tempC == -127.00) {
      if (hasValidTemp) u8g2.drawStr(halfwidth, 24, tempC_str_prev);
      else u8g2.drawStr(halfwidth, 24, "N/A");
    } else {
      strcpy(tempC_str_prev, tempC_str);
      hasValidTemp = true;
      u8g2.drawStr(halfwidth, 24, tempC_str);
    }

    u8g2.drawStr(0, 36, "BPM:");
    if (heartbeat_detected) {
      if (hasValidBPM) u8g2.drawStr(halfwidth, 24, bpm_str_prev);
      else u8g2.drawStr(halfwidth, 36, "N/A");
    } else {
      strcpy(bpm_str_prev, bpm_str);
      hasValidBPM = true;
      u8g2.drawStr(halfwidth, 36, bpm_str);
    }

    u8g2.drawStr(0, 48, "Raw IR:");
    u8g2.drawStr(halfwidth, 48, rawIR_str);

    u8g2.sendBuffer();
  }
}
