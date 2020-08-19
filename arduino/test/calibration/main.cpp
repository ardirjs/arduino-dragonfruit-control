// calibration sensor

#include <Arduino.h>

#define pinAdc A0
#define baudrate 9600
#define filterSample 15

int filterBuffer[filterSample];

void setup(void) {
  pinMode(pinAdc, 0);
  Serial.begin(baudrate);
}

void loop(void) {
  // float adc = 0, adcMax = 0, adcMin = 1023;
  static int index = 0, total = 0, average = 0;

  total = total - filterBuffer[index];
  filterBuffer[index] = analogRead(pinAdc);
  total = total + filterBuffer[index];
  index = index + 1;
  index = index < filterSample ? index : 0;
  average = total / filterSample;

  Serial.println(average);
}
