/*
   -------------------------------------------------------------------------------------
   HX711_ADC
   Arduino library for HX711 24-Bit Analog-to-Digital Converter for Weight
   Scales Olav Kallhovd sept2017
   -------------------------------------------------------------------------------------
*/

/*
   Settling time (number of samples) and data filtering can be adjusted in the
   config.h file For calibration and storing the calibration value in eeprom,
   see example file "Calibration.ino"

   The update() function checks for new data and starts the next conversion. In
   order to acheive maximum effective sample rate, update() should be called at
   least as often as the HX711 sample rate; >10Hz@10SPS, >80Hz@80SPS. If you
   have other time consuming code running (i.e. a graphical LCD), consider
   calling update() from an interrupt routine, see example file
   "Read_1x_load_cell_interrupt_driven.ino".

   This is an example sketch on how to use this library
*/

#include <HX711_ADC.h>
#if defined(ESP8266) || defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif
#include <AccelStepper.h>

// Motor pin definitions:
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

// static float previousCellVal = 0;
const float kw = 5045; // Kona Cat weight as of Aug, 2023

void setup() { //********************************************************************************
  Serial.begin(57600);
  delay(10);
  Serial.println();
  Serial.println("Starting...");

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(200);

  LoadCell.begin();
  // LoadCell.setReverseOutput(); //uncomment to turn a negative output value to
  // positive

  float calibrationValue;   // calibration value (see example file
                            // "Calibration.ino")
  calibrationValue = 696.0; // uncomment this if you want to set the calibration
                            // value in the sketch
#if defined(ESP8266) || defined(ESP32)
  // EEPROM.begin(512); // uncomment this if you use ESP8266/ESP32 and want to
  // fetch the calibration value from eeprom
#endif

  // GET EPROM
  // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

  EEPROM.get(calVal_eepromAdress,
             calibrationValue); // uncomment this if you want to fetch the
                                // calibration value from eeprom

  unsigned long stabilizingtime =
      2000; // preciscion right after power-up can be improved by adding a few
            // seconds of stabilizing time

  // NO TARE
  // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
  boolean _tare = false; // set this to false if you don't want tare to be
                         // performed in the next step

  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1)
      ;
  } else {
    LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
    Serial.println("Startup is complete");
  }
}

void loop() { //**************************************************************************************

  static boolean newDataReady = 0;
  const int serialPrintInterval =
      0; // increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell.update())
    newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float i = LoadCell.getData();
      Serial.print("Load_cell output val: ");
      Serial.println(i);
      newDataReady = 0;
      t = millis();
    }
  }

  // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't')
      LoadCell.tareNoDelay();
  }

  // check if last tare operation is complete:
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }
}

// if ((currentCellVal >= (kw - (kw * .05))) &&
//     (currentCellVal <= (kw + (kw * .05))) &&
//     (stepper.currentPosition() == 0)) {

//   Serial.println("Opening COVER!");
//   stepper.moveTo(-1224);   // Set target position OPEN:
//   stepper.runToPosition(); // Run to position with set speed
// }

// else if cat NOT within range and cover opened, then CLOSE cover
// else if (((currentCellVal <= (kw - (kw * .05))) ||
//           (currentCellVal >= (kw + (kw * .05)))) &&
//          (stepper.currentPosition() == -1224)) {

//   Serial.println("CLOSING COVER!");
//   stepper.moveTo(0);       // Set target position "CLOSED"
//   stepper.runToPosition(); // Run to position with set speed
// }