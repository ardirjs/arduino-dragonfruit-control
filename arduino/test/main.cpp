#include <Arduino.h>
#include <SIM800L.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>

#define baudrate 9600
#define debugMode 0
#define debugModeSim 0

#define pinSimRs 11
#define pinSimRx 12
#define pinSimTx 13

#define pinSwitchA 8
#define pinSwitchB 7

#define pinAdcCurrent A0
#define pinAdcVoltage A1
#define pinLedStandby A3
#define pinLedNetwork A4
#define pinLedSending A5

#define simBuffer 100
#define simSuccess 200
#define simTimeout 7500

#define adcVref 5.0
#define adcDelay 100
#define adcScale 1023.0
#define adcFilterSample 16
#define adcSampleCounter 200

int statusIndicator = 0;
int cFilterBuffer[adcFilterSample];
int vFilterBuffer[adcFilterSample];
int minCurrent = 0, maxCurrent = 30;
int minVoltage = 195, maxVoltage = 230;
boolean runningSwitching = false;

float current, voltage;
float cAdcCalibration = 1.7949;
float vAdcCalibration = 298.5074;

struct x {int hor, min;} t;
struct y {int lmp, hon, hof, mon, mof;} a, b;

const char *la = "&la=";
const char *lb = "&lb=";
const char *vv = "&vv=";
const char *va = "&va=";

const char *apn = "internet";
const char *host = "http://dragon.rjsdevs.icu/get/save.php?";
const char *deviceId = "id=12345678";

SIM800L *sim;

void pinInitialize(void) {
  int numPins = 8, pins[numPins] = {
    pinSimRs, pinSwitchA, pinSwitchB, pinLedStandby,
    pinLedSending, pinLedNetwork, pinAdcCurrent, pinAdcVoltage
  };

  for (int i = 0; i < numPins; i++) {
    pinMode(pins[i], i < (numPins - 2) ? 1 : 0);
    digitalWrite(pinLedStandby, 1);
    digitalWrite(pinLedSending, 1);
    digitalWrite(pinLedNetwork, 1);
  }
}

void timerInitialize(void) {
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  OCR2A = 0xf9;
  TCCR2A |= 1 << WGM21;
  TCCR2B |= 1 << CS21;
  TIMSK2 |= 1 << OCIE2A;
  sei();
}

void ledIndicator(int pins) {
  digitalWrite(pinLedStandby, pins == pinLedStandby ? 1 : 0);
  digitalWrite(pinLedSending, pins == pinLedSending ? 1 : 0);
  digitalWrite(pinLedNetwork, pins == pinLedNetwork ? 1 : 0);
}

void readingSensorValue(void) {
  // variable current
  float cAdc = 0, cAdcMax = 0, cAdcMin = adcScale;
  static boolean cReset = true;
  static int cIndex = 0, cTotal = 0, cAverage = 0, cValue = 0;

  // variable voltage
  float vAdc = 0, vAdcMax = 0, vAdcMin = adcScale;
  static boolean vReset = true;
  static int vIndex = 0, vTotal = 0, vAverage = 0, vValue = 0;

  // reset buffer sample current
  if (cReset) {
    for (int i = 0; i < adcFilterSample; i++) {
      cReset = false;
      cFilterBuffer[i] = 0;
    }
  }

  // reset buffer sample voltage
  if (vReset) {
    for (int i = 0; i < adcFilterSample; i++) {
      vReset = false;
      vFilterBuffer[i] = 0;
    }
  }

  // filter analog signal for sensor current and voltage
  for (int i = 0; i < adcSampleCounter; i++) {

    // filter current signal
    cTotal = cTotal - cFilterBuffer[cIndex];
    cFilterBuffer[cIndex] = analogRead(pinAdcCurrent);
    cTotal = cTotal + cFilterBuffer[cIndex];
    cIndex = cIndex + 1;
    cIndex = cIndex < adcFilterSample ? cIndex : 0;
    cAverage = (cTotal / adcFilterSample) - 0;

    // filter voltage signal
    vTotal = vTotal - vFilterBuffer[vIndex];
    vFilterBuffer[vIndex] = analogRead(pinAdcVoltage);
    vTotal = vTotal + vFilterBuffer[vIndex];
    vIndex = vIndex + 1;
    vIndex = vIndex < adcFilterSample ? vIndex : 0;
    vAverage = (vTotal / adcFilterSample) + 5;

    // read adc current for min and max value
    cAdcMin = cAverage < cAdcMin ? cAverage : cAdcMin;
    cAdcMax = cAverage > cAdcMax ? cAverage : cAdcMax;

    // read adc voltage for min and max value
    vAdcMin = vAverage < vAdcMin ? vAverage : vAdcMin;
    vAdcMax = vAverage > vAdcMax ? vAverage : vAdcMax;

    delay(adcDelay);
  }

  // calculate calibration sensor current
  cAdc = (cAdcMax - cAdcMin) * (adcVref / adcScale);
  // cAdc = cAdc < 2 ? 0 : cAdc;
  cValue = cAdc * cAdcCalibration;

  // calculate calibration sensor voltage
  vAdc = (vAdcMax - vAdcMin) * (adcVref / adcScale);
  // vAdc = vAdc < 2 ? 0 : vAdc;
  vValue = vAdc * vAdcCalibration;

  #if debugMode
  // print adc current data
  Serial.print("cAdc: " + (String)cAdc + " cAdcMin: " + (String)cAdcMin + " ");
  Serial.print("cAdcMax: " + (String)cAdcMax + " cValue: " + (String)cValue + " \n");

  // print adc voltage data
  Serial.print("vAdc: " + (String)vAdc + " vAdcMin: " + (String)vAdcMin + " ");
  Serial.print("vAdcMax: " + (String)vAdcMax + " vValue: " + (String)vValue + " \n");
  #endif

  current = cValue < minCurrent ? minCurrent : cValue > maxCurrent ? maxCurrent : cValue;
  voltage = vValue < minVoltage ? minVoltage : vValue > maxVoltage ? maxVoltage : vValue;
}

void turningOnOffControl(void) {
  if (runningSwitching) {
    if (!a.hon && !a.hof && !a.mon && !a.mof && !b.hon && !b.hof && !b.mon && !b.mof) {
      // manual mode
      digitalWrite(pinSwitchA, a.lmp);
      digitalWrite(pinSwitchB, b.lmp);
    } else {
      // schedule mode
      if (a.hon || a.hof || a.mon || a.mof) {
        if (((a.hon * 60) + a.mon) < ((a.hof * 60) + a.mof)) {
          digitalWrite(pinSwitchA, (((t.hor * 60) + t.min) > ((a.hon * 60) + a.mon) && \
          ((t.hor * 60) + t.min) < ((a.hof * 60) + a.mof)) ? 1 : 0);
        } else {
          digitalWrite(pinSwitchA, (((t.hor * 60) + t.min) < ((a.hon * 60) + a.mon) && \
          ((t.hor * 60) + t.min) > ((a.hof * 60) + a.mof)) ? 0 : 1);
        }
      }

      if (b.hon || b.hof || b.mon || b.mof) {
        if (((b.hon * 60) + b.mon) < ((b.hof * 60) + b.mof)) {
          digitalWrite(pinSwitchB, (((t.hor * 60) + t.min) > ((b.hon * 60) + b.mon) && \
          ((t.hor * 60) + t.min) < ((b.hof * 60) + b.mof)) ? 1 : 0);
        } else {
          digitalWrite(pinSwitchB, (((t.hor * 60) + t.min) < ((b.hon * 60) + b.mon) && \
          ((t.hor * 60) + t.min) > ((b.hof * 60) + b.mof)) ? 0 : 1);
        }
      }
    }
  }
}

void simIsInitalize(void) {
  SoftwareSerial *stream = new SoftwareSerial(pinSimTx, pinSimRx);
  stream->begin(baudrate);
  #if !debugModeSim
  sim = new SIM800L((Stream*)stream, 0, simBuffer * 2, simBuffer * 2);
  #else
  sim = new SIM800L((Stream*)stream, 0, simBuffer * 2, simBuffer * 2, (Stream*)&Serial);
  #endif
}

void simIsReset(void) {
  for (int i = 0; i < 3; i++) {
    digitalWrite(pinSimRs, !(i % 2));
    delay(100);
  }
}

void simIsConnect(void) {
  simIsReset();
  while (!sim->isReady());
  while (!sim->getSignal());
  ledIndicator(pinLedNetwork);
  while (!sim->getRegistrationStatus());
  while (!sim->setupGPRS(apn));
  while (!sim->connectGPRS());
  ledIndicator(pinLedSending);
}

void simIsProcessed(void) {
  ledIndicator(pinLedStandby);
  readingSensorValue();
  simIsConnect();
  int v = voltage;
  char url[simBuffer], packet[simBuffer], d[simBuffer / 5];
  dtostrf(current, 4, 2, d);
  String jsonObject, urlpath = (String)host;
  urlpath += (String)deviceId;
  urlpath += (String)la + (String)a.lmp + (String)lb + (String)b.lmp;
  urlpath += (String)vv + (String)v + (String)va + (String)d;
  urlpath.toCharArray(url, simBuffer);

  #if debugMode
  Serial.println(url);
  #endif

  if (sim->doGet(url, simTimeout) == simSuccess) {
    jsonObject = sim->getDataReceived();
  }
  // jsonObject = "{a:[1,22,3,22,14],b:[1,0,0,0,0],t:[2,23]}";

  if (jsonObject.length()) {
    jsonObject.toCharArray(packet, simBuffer);
    StaticJsonDocument <200> document;
    deserializeJson(document, packet);

    a.lmp = document["a"][0];
    a.hon = document["a"][1];
    a.hof = document["a"][2];
    a.mon = document["a"][3];
    a.mof = document["a"][4];

    b.lmp = document["b"][0];
    b.hon = document["b"][1];
    b.hof = document["b"][2];
    b.mon = document["b"][3];
    b.mof = document["b"][4];

    t.hor = document["t"][0];
    t.min = document["t"][1];

    runningSwitching = true;

    #if debugMode
    /*
    Serial.print("json:- " + (String)packet + "\n");
    Serial.print("a.lmp: " + (String)a.lmp + " a.hon: " + (String)a.hon + " ");
    Serial.print("a.hof: " + (String)a.hof + " a.mon: " + (String)a.mon + " ");
    Serial.print("a.mof: " + (String)a.mof + "\n");
    Serial.print("b.lmp: " + (String)b.lmp + " b.hon: " + (String)b.hon + " ");
    Serial.print("b.hof: " + (String)b.hof + " b.mon: " + (String)b.mon + " ");
    Serial.print("b.mof: " + (String)b.mof + "\n");
    Serial.print("hours: " + (String)t.hor + " minut: " + (String)t.min + "\n");
    delay(3000);
    */
    #endif

    turningOnOffControl();
  }
}

// ISR(TIMER2_COMPA_vect) {
//   ledIndicator(statusIndicator);
//   turningOnOffControl();
// }

void setup(void) {

  Serial.begin(baudrate);

  pinInitialize();
  simIsInitalize();
  // timerInitialize();
}

void loop(void) {
  simIsProcessed();
}
