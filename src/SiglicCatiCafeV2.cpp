

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
const float KonaWeight = 5045; // Kona Cat weight as of Aug, 2023
// TODO: const float LuckyWeight = ??

//*****************************************************************************************************
void setup() {
  Serial.begin(57600);
  delay(10);
  Serial.println();
  Serial.println("Starting...");

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

  //!&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
  EEPROM.get(calVal_eepromAdress,
             calibrationValue); //! uncomment this if you want to fetch the
                                //! calibration value from eeprom

  unsigned long stabilizingtime =
      2000; // preciscion right after power-up can be improved by adding a few
            // seconds of stabilizing time

  //!&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
  boolean _tare = true; //! set this to false if you don't want tare to be
                        //! performed in the next step (Load and set 0 weight)

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

//*****************************************************************************************************
void loop() {
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
