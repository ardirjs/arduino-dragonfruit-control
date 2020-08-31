#include <Arduino.h>
#include <SIM800L.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>

// debug serial monitor

#ifndef baudrate
#define baudrate 9600
#endif

#ifndef debugMode
#define debugMode true
#endif

#ifndef debugModeSim
#define debugModeSim false
#endif

// initialize pin sim800l

#define pinSimRs 11
#define pinSimRx 12
#define pinSimTx 13

// initialize pin switch relay

#define pinSwitchChA 8
#define pinSwitchChB 7

// initialize pin sensor

#define pinSensorCurrent A0
#define pinSensorVoltage A1

// initialize pin led indicator

#define pinLedStandby A3
#define pinLedNetwork A4
#define pinLedSending A5

// initialize variable sim800l

const uint8_t simBuffer = 100, simSuccess = 200;
const uint16_t simTimeout = 7500;

// initialize variable adc

float current = 0, voltage = 0;
const float adcVref = 5.0, adcScale = 1023.0;
const uint8_t adcTime = 100, adcFilterSample = 16, adcSampleCounter = 200;

// initialize variable led indicator and output switch relay

static uint8_t statusIndicator = 0;
static boolean runningSwitching = false;

// initialize variable calibration sensor

const uint8_t minCurrent = 0, maxCurrent = 30;
const uint8_t minVoltage = 0, maxVoltage = 130;
const float cAdcCalibration = 1.7949, vAdcCalibration = 298.5074;
int currentFilterBuffer[adcFilterSample], voltageFilterBuffer[adcFilterSample];
