#pragma once

#include <Arduino.h>
#include <AccelStepper.h>
#include <HX711_ADC.h>
#if defined(ESP8266) || defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif
#include <AccelStepper.h>

#define motorPin1 8  // IN1 on the ULN2003 driver
#define motorPin2 9  // IN2 on the ULN2003 driver
#define motorPin3 10 // IN3 on the ULN2003 driver
#define motorPin4 11 // IN4 on the ULN2003 driver

// Define the AccelStepper interface type; 4 wire motor in half step mode:
#define MotorInterfaceType 8

// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper
// library with 28BYJ-48 stepper motor:
AccelStepper stepper = AccelStepper(MotorInterfaceType, motorPin1, motorPin3,
                                    motorPin2, motorPin4);

// HX711 pins:
const int HX711_dout = 4; // mcu > HX711 dout pin
const int HX711_sck = 5;  // mcu > HX711 sck pin

// HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_eepromAdress = 0;
unsigned long t = 0;
uint32_t countTimer = 0;

float currentCellVal = 0;
float previousCellVal = 0;

int count = 0;
boolean coverOpen = false;
const float KonaWeight = 5045;  // Kona Cat weight as of Aug 29, 2023
const float LuckyWeight = 6078; // Lucky Cat weight as of Aug 29, 2023
const double var = .08;         // range variance
const float testWeight = 1420;  // 3Lb weight for testing

