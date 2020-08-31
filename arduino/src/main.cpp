#include <Arduino.h>

#define pinVoltage A1
#define sample 150

float maps(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup(void) {
  Serial.begin(9600);
  pinMode(pinVoltage, false);
}

void loop(void) {

  /*
  -* read adc value without delay and filtering
  for (int i = 0; i < sample; i ++) {
    Serial.println(analogRead(pinVoltage));
  }
  */

  /*
  -* read adc value with delay and without filtering
  for (int i = 0; i < sample; i ++) {
    Serial.println(analogRead(pinVoltage));
    delay(50);
  }
  */

  /*
  -* read adc value with delay and filtering
  const int filter = 10;
  unsigned int bufferFiltering[filter];
  static int index = 0, total = 0;
  for (int i = 0; i < filter; i ++) {
    bufferFiltering[i] = 0;
  }
  for (int i = 0; i < sample; i ++) {
    total -= bufferFiltering[index];
    bufferFiltering[index] = analogRead(pinVoltage);
    total += bufferFiltering[index];
    index = index < (filter - 1) ? index + 1 : 0;
    Serial.println(total / filter);
    delay(100);
  }
  */

  int maxVoltage = 220;
  for (int i = 0; i <= maxVoltage; i += 5) {
    int r = random(0, 2);
    int rf = random(0, 1001);
    float v = 0;
    if (r) {
      v = maps(i, 0, 220, 0, 0.77) + ((float)rf/50000);
    } else {
      v = maps(i, 0, 220, 0, 0.77) - ((float)rf/50000);
    }
    //Serial.println((float)rf/10000, 6);
    Serial.println(v, 6);
  }

  while (true);
}
