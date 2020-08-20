// calibration sensor

#include <Arduino.h>

#define pinAdc A0
#define baudrate 9600
#define filterSample 15

int filterBuffer[filterSample];

void setup(void) {
  pinMode(pinAdc, 0);
  pinMode(7, 1);
  digitalWrite(7, 1);
  Serial.begin(baudrate);
}

void loop(void) {
  static int index = 0, total = 0, average = 0;
  float adc = 0, adcMax = 0, adcMin = 1023;
  for (int i = 0; i < 200; i++) {
    total = total - filterBuffer[index];
    filterBuffer[index] = analogRead(pinAdc);
    total = total + filterBuffer[index];
    index = index + 1;
    index = index < filterSample ? index : 0;
    average = total / filterSample;

    adcMax = average > adcMax ? average : adcMax;
    adcMin = average < adcMin ? average : adcMin;
  }
  adc = (adcMax - adcMin) * (5 / 1023);
  Serial.println(adc);
}
