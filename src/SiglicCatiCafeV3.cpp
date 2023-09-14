#include "boot.h"


// FUNCTION ################################################################

float diff(float num1, float num2) {
  float result = abs(num2 - num1);
  return result;
}

// FUNCTION ################################################################
void checkWeight(float weight) {

  if ((currentCellVal >= (weight - (weight * var))) &&
      (currentCellVal <= (weight + (weight * var))) && (coverOpen == false)) {
    if (count == 0)
      countTimer = millis();

    Serial.print("Load_cell output val: ");
    Serial.println(currentCellVal);
    count++;
    if (count >= 3) {
      Serial.print("MS is: ");
      Serial.println(millis() - countTimer);
      count = 0;
      Serial.println("Opening COVER!");
      coverOpen = true;
      stepper.moveTo(-1224);   // Set target position OPEN:
      stepper.runToPosition(); // Run to position with set speed
    }

  }

  // else if cat  NOT within range and cover opened then CLOSE cover

  else if ((currentCellVal <= 100) && (coverOpen == true)) {
    if (count == 0)
      countTimer = millis();
    Serial.print("Load_cell output val: ");
    Serial.println(currentCellVal);
    count++;

    if (count >= 3) {
      Serial.print("MS is: ");
      Serial.println(millis() - countTimer);
      count = 0;
      Serial.println("Closing COVER!");
      coverOpen = false;
      stepper.moveTo(0);       // Set target position "CLOSED"
      stepper.runToPosition(); // Run to position with set speed
    }
  }
}

//*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void setup() {
  Serial.begin(57600);
  delay(10);
  Serial.println();
  Serial.println("Starting...");

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(200);
  stepper.setCurrentPosition(0);

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

//*====================================================================================================
void loop() {

  if ((!countTimer == 0) && (millis() - countTimer >= 1000)) {
    count = 0;
  }
  static boolean newDataReady = 0;
  const int serialPrintInterval =
      0; // increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell.update())
    newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      currentCellVal = LoadCell.getData();
      if (diff(previousCellVal, currentCellVal) < 10) {
        checkWeight(testWeight);
      }
      previousCellVal = currentCellVal;
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
