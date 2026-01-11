/* VISO v0.1 Source Code
 *
 * Current prototype build supports BPM and temperature (in C) monitoring.
 * Future releases will include SpO2, blood pressure, and IoT functionality.
 *
 * Sensors used: KY-039 for BPM and DS18B20 for temperature
 * NOTE: Accuracy and reliablity of sensors are not guaranteed since these are not
 *        meant for medical use. Future prototypes will use more accurate and
 *        established sensors, like the MAX30102 and MAX30205.
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

// Data wire is plugged into digital pin 2 on the Arduino
#define ONE_WIRE_BUS 1

#define DEBUG 0

// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(ONE_WIRE_BUS);

// Pass oneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);

// Pulse Monitor  Test Script
int pinBPM = 2;

// U8G2 constructor for SH1106 I2C 128x64 display
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);


  char bpm_str[50];
  char tempC_str[50];     
  char rawIR_str[50];
  char tempC_str_prev[50] = "N/A";
  bool hasValidTemp = false;

float reads[samp_siz], sum;
    long int now, ptr;
    float  last, reader, start;
    float first, second, third, before, print_value;
    bool rising;
    int rise_count;
    int n;
    long int last_beat;
    bool heartbeat_detected;

      float bpm, tempC;

unsigned long bpmDelay = 0;
unsigned long tempDelay = 0;
unsigned long displayDelay = 0;

void setup() {
  Serial.begin(115200);

  // Discplay
  Wire.begin(8, 4);   // SDA = 6, SCL = 7
  u8g2.begin();

  //temp
    sensors.begin();

  for (int i = 0; i < samp_siz; i++)
      reads[i] = 0;
    sum = 0;
    ptr = 0;
}

void loop() {
    unsigned long now = millis();
    double rawIR = analogRead(pinBPM);

    if (now - bpmDelay >= 16.67) {
      bpmDelay = now;
      // calculate an average of the  sensor
      // during a 20 ms period (this will eliminate
      // the 50  Hz noise caused by electric light
      n = 0;
      start = millis();
      reader = 0.;
      do
      {
        reader += rawIR;
        n++;
        now = millis();
      }
      while (now < start + 16.67);  
      
      reader /= n;  // we got an average
#ifdef DEBUG
      Serial.print("avgIR:");
      Serial.println(reader);
#endif    
      // Add the  newest measurement to an array
      // and subtract the oldest measurement from  the array
      // to maintain a sum of last measurements
      sum -= reads[ptr];
      sum += reader;
      reads[ptr] = reader;
      last = sum / samp_siz;

      // now last holds the average of the values in the array

      // check  for a rising curve (= a heart beat)
      if (last > before)
      {
        rise_count++;
        if (!rising && rise_count > rise_threshold)
        {
          //  Ok, we have detected a rising curve, which implies a heartbeat.
          //  Record the time since last beat, keep track of the two previous
          //  times (first, second, third) to get a weighed average.
          // The rising  flag prevents us from detecting the same rise more than once.
          rising  = true;
          first = millis() - last_beat;
          last_beat = millis();

          // Calculate the weighed average of heartbeat rate
          // according  to the three last beats
          bpm = 60000. / (0.4 * first + 0.3 *  second + 0.3 * third);
#ifdef DEBUG
          Serial.print("bpm:");
          Serial.println(bpm);
#endif      
          third = second;
          second  = first;
          heartbeat_detected = true;
        }
        else
          heartbeat_detected = false;
      }
      else
      {
        //  Ok, the curve is falling
        rising = false;
        rise_count = 0;
      }
      before = last;
      ptr++;
      ptr  %= samp_siz;
  }
  
  if(now - tempDelay >= 1000) {
    tempDelay = now;
    sensors.requestTemperatures();
    tempC = sensors.getTempCByIndex(0);
#ifdef DEBUG
    Serial.print("tempC:");
    Serial.println(tempC);
#endif 
  }

  if (now - displayDelay >= 1000) {
    displayDelay = now;
    dtostrf(bpm, 1, 2, bpm_str);
    dtostrf(tempC, 1, 2, tempC_str);
    dtostrf(rawIR, 1, 2, rawIR_str);

    u8g2.clearBuffer();

    u8g2.setFont(u8g2_font_ncenB08_tr);  // Choose a font

    int halfwidth = u8g2.getDisplayWidth() / 2;

    u8g2.drawStr(halfwidth-24, 12, "VISO v0.1");
    u8g2.drawLine(0, 12, 128, 12);
    
    u8g2.drawStr(0, 24, "Temp: ");

      if (tempC == -127.00) {
          if (hasValidTemp) {
              u8g2.drawStr(halfwidth, 24, tempC_str_prev);
          } else {
              u8g2.drawStr(halfwidth, 24, "N/A");
          }
      }
      else {
          strcpy(tempC_str_prev, tempC_str);
          hasValidTemp = true;
          u8g2.drawStr(halfwidth, 24, tempC_str);
#ifdef DEBUG
          Serial.print("tempC_str_prev:");
          Serial.println(tempC_str_prev);
#endif
      }
        

    u8g2.drawStr(0, 36, "BPM: ");
    if (heartbeat_detected)
      u8g2.drawStr(halfwidth, 36, bpm_str);
    else
      u8g2.drawStr(halfwidth, 36, "N/A");

    u8g2.drawStr(0, 48, "Raw IR: ");
    u8g2.drawStr(halfwidth, 48, rawIR_str);

    u8g2.sendBuffer();
  }
}
