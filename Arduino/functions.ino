// Menu callback function
void on_menu_1_1_1_1_selected(MenuComponent* p_menu_component) {
  OLED_PORT.clear();
  delay(2000); // so we can look the result on the display
}
void on_menu_1_1_1_2_selected(MenuComponent* p_menu_component) {
  OLED_PORT.clear();
  delay(2000); // so we can look the result on the display
}
void on_menu_1_1_1_3_selected(MenuComponent* p_menu_component) {
  OLED_PORT.clear();
  delay(2000); // so we can look the result on the display
}
void on_menu_1_1_2_1_selected(MenuComponent* p_menu_component) {
  OLED_PORT.clear();
  delay(2000); // so we can look the result on the display
}
void on_menu_1_1_2_2_selected(MenuComponent* p_menu_component) {
  OLED_PORT.clear();
  delay(2000); // so we can look the result on the display
}
void on_menu_1_1_2_3_selected(MenuComponent* p_menu_component) {
  OLED_PORT.clear();
  delay(2000); // so we can look the result on the display
}
void on_menu_1_1_2_4_selected(MenuComponent* p_menu_component) {
  OLED_PORT.clear();
  delay(2000); // so we can look the result on the display
}

void on_menu_1_2_1_1_selected(MenuComponent* p_menu_component) {
  OLED_PORT.clear();
  showScreenId = 2;
}

void on_menu_1_2_1_2_selected(MenuComponent* p_menu_component) {
  OLED_PORT.clear();
  fDetectMlx(OLED_PORT);
  delay(2000); // so we can look the result on the display
}

void on_menu_1_2_2_1_selected(MenuComponent* p_menu_component) {
  OLED_PORT.clear();
  showScreenId = 3;
}

void on_menu_1_2_2_2_selected(MenuComponent* p_menu_component) {
  OLED_PORT.clear();
  fCalGear(OLED_PORT);
  delay(2000); // so we can look the result on the display
}

void on_menu_1_2_3_1_selected(MenuComponent* p_menu_component) {
  OLED_PORT.clear();
  showScreenId = 4;
}

void on_menu_1_2_3_2_selected(MenuComponent* p_menu_component) {
  OLED_PORT.clear();
  fCalThrottle(OLED_PORT);
  delay(2000); // so we can look the result on the display
}

void on_menu_1_2_4_1_selected(MenuComponent* p_menu_component) {
  OLED_PORT.clear();
  showScreenId = 5;
}

void on_menu_1_2_4_2_selected(MenuComponent* p_menu_component) {
  NumericMenuItem* p_nmi = (NumericMenuItem*) p_menu_component;
  rpmFlywheelTeeth = (uint8_t) p_nmi->get_value();
  OLED_PORT.clear();
  fSaveFlywheel2Eeprom(OLED_PORT);
  delay(2000);
}

void on_menu_1_2_4_3_selected(MenuComponent* p_menu_component) {
  NumericMenuItem* p_nmi = (NumericMenuItem*) p_menu_component;
  rpmCorrectionRatio = (uint8_t) p_nmi->get_value();
  OLED_PORT.clear();
  fSaveRpmCorr2Eeprom(OLED_PORT);
  delay(2000);
}

void on_menu_1_3_1_1_selected(MenuComponent* p_menu_component) {
  OLED_PORT.clear();
  OLED_PORT.print("on_menu_1_3_1_1_selected");
  delay(1500); // so we can look the result on the display
}

void on_menu_1_3_2_1_selected(MenuComponent* p_menu_component) {
  OLED_PORT.clear();
  OLED_PORT.print("on_menu_1_3_2_1_selected");
  delay(1500); // so we can look the result on the display
}

void on_menu_1_3_3_1_selected(MenuComponent* p_menu_component) {
  OLED_PORT.clear();
  OLED_PORT.print("on_menu_1_3_3_1_selected");
  delay(1500); // so we can look the result on the display
}

void on_menu_1_3_4_1_selected(MenuComponent* p_menu_component) {
  OLED_PORT.clear();
  OLED_PORT.print("on_menu_1_3_4_1_selected");
  delay(1500); // so we can look the result on the display
}

void on_menu_2_1_selected(MenuComponent* p_menu_component) {
  OLED_PORT.clear();
  OLED_PORT.print("TRACKS");
  delay(1500); // so we can look the result on the display
}

void on_menu_3_selected(MenuComponent* p_menu_component) {
  OLED_PORT.clear();
  showScreenId = 99;
}

void on_menu_4_selected(MenuComponent* p_menu_component) {
  showScreenId = 0; // Back to first screen
}


/**************************************************************
  #printNav > Print navigation menu
**************************************************************/
void printNav(uint8_t typeId = 0) {
  OLED_PORT.setCursor(0, 7);
  switch (typeId) {
    case 1:
      OLED_PORT.print(F("MENU"));
      break;
    case 2:
      OLED_PORT.print(F("           BACK"));
      break;
    default:
      OLED_PORT.print(F(" UP | DW | BACK | OK "));
      break;
  }
}

/**************************************************************
  #showData > Print data on OLED screen
**************************************************************/
void showData(uint8_t screenId) {
  OLED_PORT.setCursor(0, 0);
  switch (screenId) {
    case 0:
      //OLED_PORT.println(F("HOME"));
      printDate(fix_data, OLED_PORT);
      printNav(1);
      break;
    case 2:
      for (uint8_t i = 0; i < maxMlx; i++) {
        if (mlxAddresses[i] != 0x00) {
          OLED_PORT.print(F("MLX "));
          OLED_PORT.print(mlxAddresses[i]);
          OLED_PORT.print(F(" : "));
          OLED_PORT.print(mlxValues[i], 0);
          OLED_PORT.println(F("°"));
        }
      }
      printNav(2);
      break;
    case 3:
      for (uint8_t i = 0; i < 7; i++) {
        if (i == 0) {
          OLED_PORT.print(F("N"));
        } else {
          OLED_PORT.print(i);
        }
        OLED_PORT.print(F(" > "));
        OLED_PORT.print(inAnaGearCalib[i]);
        if (i % 2 != 0) {
          OLED_PORT.println();
        } else {
          OLED_PORT.print(F("    "));
        }
      }
      OLED_PORT.println();
      OLED_PORT.print(F("Gear ... "));
      OLED_PORT.println(gear);
      printNav(2);
      break;
    case 4:
      OLED_PORT.print(F("Throttle max ... "));
      OLED_PORT.println(inAnaThrottleMax);
      OLED_PORT.print(F("Throttle ... "));
      OLED_PORT.println(inAnaThrottle);
      printNav(2);
      break;
    case 5:
      OLED_PORT.print(F("Flywheel teeth ... "));
      OLED_PORT.println(rpmFlywheelTeeth);
      OLED_PORT.print(F("RPM ratio ... "));
      OLED_PORT.println(rpmCorrectionRatio);
      OLED_PORT.print(F("RPM ... "));
      OLED_PORT.println(inDigiSqrRpm);
      printNav(2);
      break;
    case 99:
      OLED_PORT.println("D.A.W.A.");
      OLED_PORT.println("HW rev : 6.1");
      OLED_PORT.println("SW rev : 6.1");
      OLED_PORT.println("Author : E.PIGEON");
      OLED_PORT.println("dawa@panik-po.com");
      OLED_PORT.println("dawa.panik-po.com");
      printNav(2);
      break;
  }
}

/**************************************************************
  #handleMenuActions > Handle menu navigation
**************************************************************/
void handleMenuActions(uint8_t buttonTriggered) {
  if (showScreenId == 1) {
    switch (buttonTriggered) {
      case 3: // Previous item
        ms.prev();
        ms.display();
        break;
      case 2: // Next item
        ms.next();
        ms.display();
        break;
      case 1: // Back presed
        ms.back();
        ms.display();
        break;
      case 0: // Select presed
        ms.select();
        ms.display();
        break;
      default:
        break;
    }
  }
}

/**************************************************************
  #callbackMcp1Interrupt > Interrupt when buttons pressed
**************************************************************/
void callbackMcp1Interrupt() {
  mcp1Interrupt = true; // Set boolean to handle actions outside the interrupt
}

/**************************************************************
  #cleanMcp1Interrupt > Read GPIO values of MCP23017
**************************************************************/
void cleanMcp1Interrupt() {
  MCP1.readGPIOAB(); // A read is needed to reset MCP23017 interrupt flags
  mcp1Interrupt = false;
}

/**************************************************************
  #handleMcp1Interrupt > Actions when buttons pressed
**************************************************************/
void handleMcp1Interrupt() {
  mcp1PinTriggeredId = MCP1.getLastInterruptPin(); // Read which pin was triggered
  mcp1PinTriggeredState = MCP1.getLastInterruptPinValue(); // 0 => pressed, 1 => released
  cleanMcp1Interrupt(); // Clean interrupts

#ifdef DEBUG
  DEBUG_PORT.print(F("Button "));
  DEBUG_PORT.print(mcp1PinTriggeredId);
  DEBUG_PORT.print(F(", state "));
  DEBUG_PORT.println(mcp1PinTriggeredState);
#endif

  if (mcp1PinTriggeredState == 1) {
    buttonPressed = millis();
#ifdef DEBUG
    DEBUG_PORT.println(buttonPressed);
#endif
    handleMenuActions(mcp1PinTriggeredId);
  } else {
    buttonPressed = millis() - buttonPressed;
#ifdef DEBUG
    DEBUG_PORT.println(buttonPressed);
#endif
  }
}

/**************************************************************
  #fGetHelp > Print available commands
**************************************************************/
void fGetHelp() {
  BLUETOOTH_PORT.println(F("Available commands"));
  BLUETOOTH_PORT.println(F(">>> General"));
  BLUETOOTH_PORT.println(F("1-Show realtime data"));
  BLUETOOTH_PORT.println(F("2-Show last runs since 12h"));
  BLUETOOTH_PORT.println(F("3-Show last runs since 48h"));
  BLUETOOTH_PORT.println(F("4-Show best laps on this track"));
  BLUETOOTH_PORT.println(F(">>> Calibration"));
  BLUETOOTH_PORT.println(F("20-Calibrate Throttle"));
  BLUETOOTH_PORT.println(F("21-Calibrate Analogic Opt 1"));
  BLUETOOTH_PORT.println(F("22-Calibrate Analogic Opt 2"));
  BLUETOOTH_PORT.println(F("23-Calibrate gears"));
  BLUETOOTH_PORT.println(F(">>> Inputs"));
  BLUETOOTH_PORT.println(F("30-Show input ports state"));
  BLUETOOTH_PORT.println(F("31-Enable or disable port state"));
  BLUETOOTH_PORT.println(F(">>> Infrared temp. sensors"));
  BLUETOOTH_PORT.println(F("40-Show IR sensors"));
  BLUETOOTH_PORT.println(F("41-Detect IR sensors"));
}

/**************************************************************
  #fGetLastRuns > Print sessions in the last xxh
**************************************************************/
void fGetLastRuns(uint16_t sinceHours) {
  char trackName[16], lapTime[10], bestLapTimeStr[10], histDate[14];
  float lapTimeSec, bestLapTime;
  uint16_t bestLapId, histTrackId;
  int32_t histTimestamp;

  historyFile = sd.open("HISTORY.csv", FILE_READ);
  if (historyFile) {
    BLUETOOTH_PORT.println(F("----------------------------------"));
    while (historyFile.available()) {
      csvReadInt32(&historyFile, &histTimestamp, csvDelim);
      csvReadText(&historyFile, histDate, sizeof(histDate), csvDelim);
      csvReadUint16(&historyFile, &histTrackId, csvDelim);
      if (histTimestamp > (fix_data.dateTime - (sinceHours * 3600))) {
        printDateTime(histTimestamp, BLUETOOTH_PORT);
        trackFile = sd.open("TRACKS.csv", FILE_READ); // Read trackfile to get track name
        if (trackFile) {
          while (trackFile.available()) {
            csvReadUint16(&trackFile, &trackId, csvDelim);
            csvReadText(&trackFile, trackName, sizeof(trackName), csvDelim);
            csvReadInt32(&trackFile, &flineLat1, csvDelim);
            csvReadInt32(&trackFile, &flineLon1, csvDelim);
            csvReadInt32(&trackFile, &flineLat2, csvDelim);
            csvReadInt32(&trackFile, &flineLon2, csvDelim);

            if (trackId == histTrackId) {
              trackFile.close();
              BLUETOOTH_PORT.print(F("Track : "));
              BLUETOOTH_PORT.println(trackName);
              break;
            }
          }
          trackFile.close();
        } else {
          BLUETOOTH_PORT.print(F("Can't find any information about track"));
        }
        sprintf(filename, "%s-LAPTIMES.csv", histDate);
        lapFile = sd.open(filename, FILE_READ);
        if (lapFile) {
          bestLapTime = 9999.00;
          while (lapFile.available()) {
            csvReadUint16(&lapFile, &lapId, csvDelim);
            csvReadText(&lapFile, lapTime, sizeof(lapTime), csvDelim);
            csvReadFloat(&lapFile, &lapTimeSec, csvDelim);
            if (lapTimeSec < bestLapTime) {
              bestLapTime = lapTimeSec;
              bestLapId = lapId;
              strcpy(bestLapTimeStr, lapTime);
            }
            BLUETOOTH_PORT.print(F("L"));
            BLUETOOTH_PORT.print(lapId);
            BLUETOOTH_PORT.print(F(" > "));
            BLUETOOTH_PORT.println(lapTime);
          }
          BLUETOOTH_PORT.print(F("Best : "));
          BLUETOOTH_PORT.print(F("L"));
          BLUETOOTH_PORT.print(bestLapId);
          BLUETOOTH_PORT.print(F(" > "));
          BLUETOOTH_PORT.println(bestLapTimeStr);
          BLUETOOTH_PORT.println(F("----------------------------------"));
          lapFile.close();
        } else {
          BLUETOOTH_PORT.println(F("No valid lap"));
        }
      } else {
        BLUETOOTH_PORT.print(F("Can't find any run since the last "));
        BLUETOOTH_PORT.print(sinceHours);
        BLUETOOTH_PORT.println(F(" hours"));
      }
    }
    historyFile.close();
  }
}

/**************************************************************
  #fGetTrackBest > Print best laptimes
**************************************************************/
void fGetTrackBest() {
  // TODO
}

/**************************************************************
  #fGetData > Print realtime data (like on the oled screen)
**************************************************************/
void fGetData() {
  printData1(fix_data, BLUETOOTH_PORT);
  printData2(fix_data, BLUETOOTH_PORT);
}

/**************************************************************
  #fSaveFlywheel2Eeprom > Save new rpmFlywheel value to EEPROM
**************************************************************/
void fSaveFlywheel2Eeprom(Print & OUT_PORT) {
  if (EEPROM_writeAnything(29, rpmFlywheelTeeth) == sizeof(rpmFlywheelTeeth)) {
    OUT_PORT.println(F("New value saved"));
  } else {
    OUT_PORT.println(F("save failed !"));
  }
}

/**************************************************************
  #fSaveRpmCorr2Eeprom > Save new RPM correction value to EEPROM
**************************************************************/
void fSaveRpmCorr2Eeprom(Print & OUT_PORT) {
  if (EEPROM_writeAnything(28, rpmCorrectionRatio) == sizeof(rpmCorrectionRatio)) {
    OUT_PORT.println(F("New value saved"));
  } else {
    OUT_PORT.println(F("save failed !"));
  }
}

/**************************************************************
  #fCalThrottle > Calibrate throttle (max value)
**************************************************************/
void fCalThrottle(Print & OUT_PORT) {
  uint16_t tmpThrottle;
  OUT_PORT.print(F("Accelerate ... "));
  delay(5000);
  tmpThrottle = analogRead(inAnaThrottlePin);
  OUT_PORT.println(tmpThrottle);
  if (EEPROM_writeAnything(31, tmpThrottle) == sizeof(tmpThrottle)) {
    inAnaThrottleMax = tmpThrottle;
    OUT_PORT.println(F("Throttle cal. done !"));
  } else {
    OUT_PORT.println(F("Throttle cal. failed !"));
  }
}

/**************************************************************
  #fCalAnaOpt1 > Calibrate analogic opt 1 (max value)
**************************************************************/
void fCalAnaOpt1() {
  uint16_t tmpAna;
  BLUETOOTH_PORT.print(F("Set analogic opt1 to 100% ... "));
  delay(2000);
  tmpAna = analogRead(inAnaOpt1Pin);
  BLUETOOTH_PORT.println(tmpAna);
  if (EEPROM_writeAnything(33, tmpAna) == sizeof(tmpAna)) {
    inAnaOpt1Max = tmpAna;
    BLUETOOTH_PORT.println(F("Analogic opt1 calibration done !"));
  } else {
    BLUETOOTH_PORT.println(F("Analogic opt1 calibration failed !"));
  }
}

/**************************************************************
  #fCalAnaOpt2 > Calibrate analogic opt 2 (max value)
**************************************************************/
void fCalAnaOpt2() {
  uint16_t tmpAna;
  BLUETOOTH_PORT.print(F("Set analogic opt2 to 100% ... "));
  delay(2000);
  tmpAna = analogRead(inAnaOpt2Pin);
  BLUETOOTH_PORT.println(tmpAna);
  if (EEPROM_writeAnything(35, tmpAna) == sizeof(tmpAna)) {
    inAnaOpt2Max = tmpAna;
    BLUETOOTH_PORT.println(F("Analogic opt2 calibration done !"));
  } else {
    BLUETOOTH_PORT.println(F("Analogic opt2 calibration failed !"));
  }
}

/**************************************************************
  #fCalGear > Calibrate gear (values for each gear)
**************************************************************/
void fCalGear(Print & OUT_PORT) {
  for (uint8_t i = 0; i < 7; i++) {
    if (i == 0) {
      OUT_PORT.print(F("Set GEAR to N"));
    } else {
      OUT_PORT.print(F("Set GEAR to "));
      OUT_PORT.print(i);
    }
    OUT_PORT.print(F(" ... "));
    delay(3000);
    inAnaGearCalib[i] = analogRead(inAnaGearPin);
    OUT_PORT.println(inAnaGearCalib[i]);
  }
  if (EEPROM_writeAnything(37, inAnaGearCalib) == sizeof(inAnaGearCalib)) {
    OUT_PORT.println(F("Gear cal. done !"));
  } else {
    OUT_PORT.println(F("Gear cal. failed !"));
  }
}

/**************************************************************
  #fCal9axis > Calibrate 9-axis sensor
**************************************************************/
void fCal9axis() {
  /*OLED_PORT.clear();
    OLED_PORT.setCursor(0, 0); // Set cursor upper-left
    OLED_PORT.println(F("9-Axis calibration"));
    OLED_PORT.println(F("Move DAWA by doing 8"));
    OLED_PORT.println(F("Place DAWA in 6 pos."));
    OLED_PORT.println(F("+X,-X,+Y,-Y,+Z,-Z"));
    BLUETOOTH_PORT.println(F("9-Axis calibration"));
    BLUETOOTH_PORT.println(F("1) Move DAWA by doing eights"));
    BLUETOOTH_PORT.println(F("2) Place DAWA in 6 standing positions for +X, -X, +Y, -Y, +Z and -Z"));
    while (!bno.isFullyCalibrated()) { // We wait until sensor fully calibrated !! Offsets are returned only if sensor is calibrated !!
    delay(200);
    bno.getCalibration(&calSys, &calGyro, &calAccel, &calMag); // Get calibration state (from 0 to 3 for the 3 sensors + main sensor calibration)
    OLED_PORT.setCursor(42, 5);
    OLED_PORT.print(calSys);
    OLED_PORT.print(F(";"));
    OLED_PORT.print(calGyro);
    OLED_PORT.print(F(";"));
    OLED_PORT.print(calAccel);
    OLED_PORT.print(F(";"));
    OLED_PORT.print(calMag);
    delay(200);
    }
    bno.getSensorOffsets(calibrationData); // Sensor is calibrated, we save offsets to var (calibrationData)
    OLED_PORT.setCursor(36, 7);
    calibResetBit = 0;
    if ((EEPROM_writeAnything(0, calibrationData) == sizeof(adafruit_bno055_offsets_t)) && (EEPROM_writeAnything(25, calibResetBit) == sizeof(calibResetBit))) { // We save the 11 offsets to eeprom
    OLED_PORT.println(F("OK"));
    BLUETOOTH_PORT.println(F("9-axis calibration done !"));
    BLUETOOTH_PORT.print("OFFSETS : ");
    BLUETOOTH_PORT.print(calibrationData.accel_offset_x);
    BLUETOOTH_PORT.print(",");
    BLUETOOTH_PORT.print(calibrationData.accel_offset_y);
    BLUETOOTH_PORT.print(",");
    BLUETOOTH_PORT.print(calibrationData.accel_offset_z);
    BLUETOOTH_PORT.print(",");
    BLUETOOTH_PORT.print(calibrationData.gyro_offset_x);
    BLUETOOTH_PORT.print(",");
    BLUETOOTH_PORT.print(calibrationData.gyro_offset_y);
    BLUETOOTH_PORT.print(",");
    BLUETOOTH_PORT.print(calibrationData.gyro_offset_z);
    BLUETOOTH_PORT.print(",");
    BLUETOOTH_PORT.print(calibrationData.mag_offset_x);
    BLUETOOTH_PORT.print(",");
    BLUETOOTH_PORT.print(calibrationData.mag_offset_y);
    BLUETOOTH_PORT.print(",");
    BLUETOOTH_PORT.print(calibrationData.mag_offset_z);
    BLUETOOTH_PORT.print(",");
    BLUETOOTH_PORT.print(calibrationData.accel_radius);
    BLUETOOTH_PORT.print(",");
    BLUETOOTH_PORT.println(calibrationData.mag_radius);
    } else {
    OLED_PORT.println(F("FAILED"));
    BLUETOOTH_PORT.println(F("9-axis calibration failed !"));
    }
    delay(1000);
    OLED_PORT.clear();*/
}

/**************************************************************
  #fResetCal9axis > Reset 9-axis calibration sensor
**************************************************************/
void fResetCal9axis() {
  /*calibResetBit = 1;
    if (EEPROM_writeAnything(25, calibResetBit) == sizeof(calibResetBit)) { // Set the bit to not load existing offsets at the next restart
    BLUETOOTH_PORT.println(F("Please restart and run a new calibration"));
    } else {
    BLUETOOTH_PORT.println(F("EEPROM write error"));
    }*/
}

/**************************************************************
  #fChangeEnabledInputs > Change enabled inputs
**************************************************************/
void fChangeEnabledInputs(uint8_t inputBits) {
  if (inputBits == 0) {
    BLUETOOTH_PORT.println(ERROR_PROVIDE_NUMBER);
    BLUETOOTH_PORT.println(F("This number shoud be the sum of values of each inputs you want to enable"));
  } else {
    if (EEPROM_writeAnything(30, inputBits) == sizeof(inputBits)) {
      enabledInputsBits = inputBits;
      BLUETOOTH_PORT.println(F("New inputs state defined !"));
      showEnabledInputs(BLUETOOTH_PORT);
    } else {
      BLUETOOTH_PORT.println(F("Error writing inputs state"));
    }
  }
}

/**************************************************************
  #fShowEnabledInputs > Show enabled inputs
**************************************************************/
void fShowEnabledInputs() {
  showEnabledInputs(BLUETOOTH_PORT);
}

/**************************************************************
  #showEnabledInputs > Show enabled inputs
**************************************************************/
void showEnabledInputs(Print & OUT_PORT) {
  uint8_t bitShift = B00000001, tmpComp;
  if (EEPROM_readAnything(30, enabledInputsBits) == sizeof(enabledInputsBits)) {
    OUT_PORT.println(F("Enabled inputs state :"));
    for (uint8_t i = 0; i < 8; i++) {
      OUT_PORT.print(inputsLabel[i]);
      tmpComp = bitShift & enabledInputsBits;
      if (tmpComp == bitShift) {
        OUT_PORT.print(F(" : ENABLED"));
      } else {
        OUT_PORT.print(F(" : DISABLED"));
      }
      OUT_PORT.print(F(" ("));
      OUT_PORT.print(pow(2, i), 0);
      OUT_PORT.println(F(")"));
      bitShift = bitShift << 1;
    }
  } else {
    OUT_PORT.println(F("Error reading inputs state"));
  }
}

/**************************************************************
  #mcp2EnableOneOutput > Control MCP2 outputs : only one HIGH (0 < id < 8) or all (id = 8)
**************************************************************/
void mcp2EnableOneOutput(uint8_t idOutput) {
  for (uint8_t i = 0; i < 8; i++) {
    if (i == idOutput || idOutput == 8) {
      MCP2.digitalWrite(i, HIGH);
    } else {
      MCP2.digitalWrite(i, LOW);
    }
    delay(50);
  }
}

/**************************************************************
  #fListMlxAddr > List saved I2C address for all MLX declared chip
**************************************************************/
void fListMlxAddr() {
  for (uint8_t i = 0; i < maxMlx; i++) {
    BLUETOOTH_PORT.print(F("MLX I2C address : "));
    BLUETOOTH_PORT.println(mlxAddresses[i], HEX);
  }
}

/**************************************************************
  #fDetectMlx > Add new MLX chips
**************************************************************/
void fDetectMlx(Print & OUT_PORT) {
  uint8_t mlxAddress, mlxId = 0;
  for (uint8_t i = 0; i < maxMlx; i++) {
    mlxAddresses[i] = 0x00; // Array which will contains all discovered MLX addresses. This array will be wrote on EEPROM at the end
  }
  for (uint8_t i = 0; i < 8; i++) {
    mcp2EnableOneOutput(i); // Enable only one MCP output, all the others are disabled
    MLX90614 mynewmlx = MLX90614(0); // 0 = broadcast, only one MLX chip shloud be plugged on I2C bus
    mlxAddress = (uint8_t)mynewmlx.readEEProm(mlxEepAddr); // Read address of the only MLX available on I2C bus (others are disabled with MCP)
    if (mlxAddress != 0xFF) {
      mlxAddresses[mlxId] = firstMlxAddress + mlxId;
      mynewmlx.writeEEProm(mlxEepAddr, mlxAddresses[mlxId]);
      OUT_PORT.print(F("New MLX ! (P="));
      OUT_PORT.print(i);
      OUT_PORT.print(F("/ID="));
      OUT_PORT.print((uint8_t)mynewmlx.readEEProm(mlxEepAddr), HEX);
      OUT_PORT.println(F(")"));
      mlxId++;
    }
  }
  if (EEPROM_writeAnything(50, mlxAddresses) == sizeof(mlxAddresses)) { // Write array of addresses on EEPROM
    OUT_PORT.print(mlxId);
    OUT_PORT.println(F(" MLX detected"));
  } else {
    OUT_PORT.println(F("Error adding new MLX sensors"));
  }
  mcp2EnableOneOutput(8); // Re-enable all outputs on MCP2
}

/**************************************************************
  SERCOM5_Handler : Secondary hardware serial for bluetooth
**************************************************************/
void SERCOM5_Handler() {
  BLUETOOTH_PORT.IrqHandler();
}

/**************************************************************
  Output DATE+TIME based on a timestamp
**************************************************************/
void printDateTime(uint32_t timestamp, Print & OUT_PORT) {
  if (day(timestamp) < 10) OUT_PORT.print(F("0")); // Leading zeros
  OUT_PORT.print(day(timestamp));
  OUT_PORT.print(F("/"));
  if (month(timestamp) < 10) OUT_PORT.print(F("0")); // Leading zeros
  OUT_PORT.print(month(timestamp));
  OUT_PORT.print(F("/"));
  OUT_PORT.print(year(timestamp));
  OUT_PORT.print(F(" "));
  if (hour(timestamp) < 10) OUT_PORT.print(F("0")); // Leading zeros
  OUT_PORT.print(hour(timestamp));
  OUT_PORT.print(F(":"));
  if (minute(timestamp) < 10) OUT_PORT.print(F("0")); // Leading zeros
  OUT_PORT.print(minute(timestamp));
  OUT_PORT.print(F(":"));
  if (second(timestamp) < 10) OUT_PORT.print(F("0")); // Leading zeros
  OUT_PORT.println(second(timestamp));
}

/**************************************************************
  #processCharInput > Build a string character by character (serial bluetooth)
**************************************************************/
char processCharInput(char* cmdBuffer, const char c) {
  if (c >= 32 && c <= 126) { // Ignore control and special ascii characters
    if (strlen(cmdBuffer) < 16) {
      strncat(cmdBuffer, &c, 1);
    } else {
      return '\n';
    }
  }
  return c;
}

/**************************************************************
  #initError > Triggered when initialization error (program stop and slow blinking led)
**************************************************************/
void initError() {
  delay(2000); // Pause 3 sec pour lecture infos LCD
  while (1) {
    //digitalWrite(ledPin, digitalRead(ledPin) ^ 1); // Quick LED blink (there's an error)
    MCP1.digitalWrite(mcp1Led1, MCP1.digitalRead(mcp1Led1) ^ 1);
    MCP1.digitalWrite(mcp1Led2, MCP1.digitalRead(mcp1Led2) ^ 1);
    MCP1.digitalWrite(mcp1Led3, MCP1.digitalRead(mcp1Led3) ^ 1);
    MCP1.digitalWrite(mcp1Led4, MCP1.digitalRead(mcp1Led4) ^ 1);
    delay(200);
  }
}

/**************************************************************
  #sendUBX > Send UBX commands to UBLOX GPS
**************************************************************/
void sendUBX(const unsigned char *progmemBytes, size_t len) {
  GPS_PORT.write(0xB5); // SYNC1
  GPS_PORT.write(0x62); // SYNC2

  uint8_t a = 0, b = 0;
  while (len-- > 0) {
    uint8_t c = pgm_read_byte(progmemBytes++);
    a += c;
    b += a;
    GPS_PORT.write(c);
  }
  GPS_PORT.write(a); // CHECKSUM A
  GPS_PORT.write(b); // CHECKSUM B
}

/**************************************************************
  Read CSV file
  https://github.com/greiman/SdFat/blob/master/examples/ReadCsv/ReadCsv.ino

  Read a CSV file one field at a time.
  file - File to read.
  str - Character array for the field.
  size - Size of str array.
  delim - csv delimiter.
  return - negative value for failure.
        delimiter, '\n' or zero(EOF) for success.
**************************************************************/
int csvReadText(File * file, char* str, size_t size, char delim) {
  char ch;
  int rtn;
  size_t n = 0;
  while (true) {
    // check for EOF
    if (!file->available()) {
      rtn = 0;
      break;
    }
    if (file->read(&ch, 1) != 1) {
      // read error
      rtn = -1;
      break;
    }
    // Delete CR.
    if (ch == '\r') {
      continue;
    }
    if (ch == delim || ch == '\n') {
      rtn = ch;
      break;
    }
    if ((n + 1) >= size) {
      // string too long
      rtn = -2;
      n--;
      break;
    }
    str[n++] = ch;
  }
  str[n] = '\0';
  return rtn;
}

int csvReadInt32(File * file, int32_t* num, char delim) {
  char buf[20];
  char* ptr;
  int rtn = csvReadText(file, buf, sizeof(buf), delim);
  if (rtn < 0) return rtn;
  *num = strtol(buf, &ptr, 10);
  if (buf == ptr) return -3;
  while (isspace(*ptr)) ptr++;
  return *ptr == 0 ? rtn : -4;
}

int csvReadInt16(File * file, int16_t* num, char delim) {
  int32_t tmp;
  int rtn = csvReadInt32(file, &tmp, delim);
  if (rtn < 0) return rtn;
  if (tmp < INT_MIN || tmp > INT_MAX) return -5;
  *num = tmp;
  return rtn;
}

int csvReadUint32(File * file, uint32_t* num, char delim) {
  char buf[20];
  char* ptr;
  int rtn = csvReadText(file, buf, sizeof(buf), delim);
  if (rtn < 0) return rtn;
  *num = strtoul(buf, &ptr, 10);
  if (buf == ptr) return -3;
  while (isspace(*ptr)) ptr++;
  return *ptr == 0 ? rtn : -4;
}

int csvReadUint16(File * file, uint16_t* num, char delim) {
  uint32_t tmp;
  int rtn = csvReadUint32(file, &tmp, delim);
  if (rtn < 0) return rtn;
  if (tmp > UINT_MAX) return -5;
  *num = tmp;
  return rtn;
}

int csvReadDouble(File * file, double * num, char delim) {
  char buf[20];
  char* ptr;
  int rtn = csvReadText(file, buf, sizeof(buf), delim);
  if (rtn < 0) return rtn;
  *num = strtod(buf, &ptr);
  if (buf == ptr) return -3;
  while (isspace(*ptr)) ptr++;
  return *ptr == 0 ? rtn : -4;
}

int csvReadFloat(File * file, float * num, char delim) {
  double tmp;
  int rtn = csvReadDouble(file, &tmp, delim);
  if (rtn < 0)return rtn;
  // could test for too large.
  *num = tmp;
  return rtn;
}

/**************************************************************
  #segIntersect > Calculate 2 line segments intersection
**************************************************************/
bool segIntersect(int32_t pos_now_lat, int32_t pos_now_lon, int32_t pos_prev_lat, int32_t pos_prev_lon, int32_t trackLat1, int32_t trackLon1, int32_t trackLat2, int32_t trackLon2, int32_t & pos_cross_lat, int32_t & pos_cross_lon) {
  bool denomPositive;
  float denom, s_numer, t_numer, t;
  int32_t track_pos_x, track_pos_y, pos_x, pos_y, trackLon, trackLat;

  trackLon = trackLon2 - trackLon1;
  trackLat = trackLat2 - trackLat1;
  pos_x = pos_now_lon - pos_prev_lon;
  pos_y = pos_now_lat - pos_prev_lat;
  denom = trackLon * pos_y - pos_x * trackLat;
  if (denom == 0) {
    return 0; // Collinear
  }

  if (denom > 0) {
    denomPositive = true;
  } else {
    denomPositive = false;
  }

  track_pos_x = trackLon1 - pos_prev_lon;
  track_pos_y = trackLat1 - pos_prev_lat;

  s_numer = trackLon * track_pos_y - trackLat * track_pos_x;
  if ((s_numer < 0) == denomPositive) {
    return 0; // No collision
  }

  t_numer = pos_x * track_pos_y - pos_y * track_pos_x;
  if ((t_numer < 0) == denomPositive) {
    return 0; // No collision
  }

  if (((s_numer > denom) == denomPositive) || ((t_numer > denom) == denomPositive)) {
    return 0; // No collision
  }

  // Collision detected
  t = t_numer / denom;
  pos_cross_lat = trackLat1 + (t * trackLat);
  pos_cross_lon = trackLon1 + (t * trackLon);

  return 1;
}

/**************************************************************
  #gpsDistance > Calculate distance between 2 GPS coords (unit = meters) - haversine formula
**************************************************************/
float gpsDistance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2) {
  float dlam, dphi, p = 0.017453292519943295; // (Pi / 180)
  dphi = p * (lat1 + lat2) * 0.5e-7; //average latitude in radians
  float cphi = cos(dphi);
  dphi = p * ( lat2 - lat1) * 1.0e-7; //differences in degrees (to radians)
  dlam = p * ( lon2 - lon1) * 1.0e-7;
  dlam *= cphi;  //correct for latitude
  return 6371000.0 * sqrt(dphi * dphi + dlam * dlam);
}

/**************************************************************
  #timeAdd > Add a time [arg0] (Ex : 5.14) to another which is composed of seconds [arg1] and milliseconds [arg2]. Return seconds [arg3] and milliseconds [arg4]
**************************************************************/
void timeAdd(float timeSecCsec, int32_t endSec, int32_t endCsec, int32_t &returnSec, int32_t &returnCsec) {
  returnSec = endSec + (int32_t)(timeSecCsec + (endCsec / 100.00));
  returnCsec = ((timeSecCsec + (endCsec / 100.00)) - (int32_t)(timeSecCsec + (endCsec / 100.00))) * 100;
}

/**************************************************************
  #timeSubstract > Substract a time [arg0][arg1] to another [arg2][arg3]. Return seconds [arg4] and milliseconds [arg6]
**************************************************************/
void timeSubstract(int32_t s1, int32_t cs1, int32_t s2, int32_t cs2, int32_t &returnSec, int32_t &returnCsec) {
  returnCsec = cs1 - cs2;
  if (returnCsec < 0) {
    returnSec = (s1 - s2) - 1;
    returnCsec += 100;
  } else {
    returnSec = s1 - s2;
  }
}

/**************************************************************
  #printDate > Print date (oled screen or bluetooth serial)
**************************************************************/
void printDate(gps_fix & fix_data, Print & OUT_PORT) {
  if (fix_data.valid.date && fix_data.valid.time) {
    if (fix_data.dateTime.date < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.date);
    OUT_PORT.print(F("/"));
    if (fix_data.dateTime.month < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.month);
    OUT_PORT.print(F("/"));
    if (fix_data.dateTime.year < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.year);
    OUT_PORT.print(F(" "));
    if (fix_data.dateTime.hours < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.hours);
    OUT_PORT.print(F(":"));
    if (fix_data.dateTime.minutes < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.minutes);
    OUT_PORT.print(F(":"));
    if (fix_data.dateTime.seconds < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.println(fix_data.dateTime.seconds);
  } else {
    OUT_PORT.println(F("No GPS signal"));
  }
}

/**************************************************************
  #printData1 > Print realtime data on one output (oled screen or bluetooth serial)
**************************************************************/
void printData1(gps_fix & fix_data, Print & OUT_PORT) {
  if (fix_data.valid.date && fix_data.valid.time) {
    if (fix_data.dateTime.date < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.date);
    OUT_PORT.print(F("/"));
    if (fix_data.dateTime.month < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.month);
    OUT_PORT.print(F("/"));
    if (fix_data.dateTime.year < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.year);
    OUT_PORT.print(F(" "));
    if (fix_data.dateTime.hours < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.hours);
    OUT_PORT.print(F(":"));
    if (fix_data.dateTime.minutes < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.minutes);
    OUT_PORT.print(F(":"));
    if (fix_data.dateTime.seconds < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.println(fix_data.dateTime.seconds);
  } else {
    OUT_PORT.println(F("No GPS signal"));
  }
  /*OUT_PORT.print(F("GPS:"));
    OUT_PORT.print(fix_data.dateTime);
    OUT_PORT.print(F("."));
    if (fix_data.dateTime_cs < 10) OUT_PORT.print(F("0")); // Leading zeros (remember "fix_data.dateTime_cs" is an integer !!)
    OUT_PORT.println(fix_data.dateTime_cs);*/
  OUT_PORT.print(F("RPM:        "));
  OUT_PORT.println(inDigiSqrRpm);
  OUT_PORT.print(F("DIGISQR:    "));
  OUT_PORT.println(inDigiSqrOpt1);
  OUT_PORT.print(F("BRAKE/DIGI1:"));
  OUT_PORT.print(inDigiBrake);
  OUT_PORT.print(F(" / "));
  OUT_PORT.println(inDigiOpt1);
  OUT_PORT.print(F("GEAR:       "));
  OUT_PORT.print(gear);
  OUT_PORT.print(F(" ("));
  if (inAnaGear < 10) OUT_PORT.print(F("000"));
  if (inAnaGear >= 10 && inAnaGear < 100) OUT_PORT.print(F("00"));
  if (inAnaGear >= 100 && inAnaGear < 1000) OUT_PORT.print(F("0"));
  OUT_PORT.print(inAnaGear); //gear
  OUT_PORT.println(F(")"));
  OUT_PORT.print(F("THROTTLE:   "));
  if (inAnaThrottle < 10) OUT_PORT.print(F("00"));
  if (inAnaThrottle >= 10 && inAnaThrottle < 100) OUT_PORT.print(F("0"));
  OUT_PORT.println(inAnaThrottle);
  OUT_PORT.print(F("ANAOPT1/2:  "));
  if (inAnaOpt1 < 10) OUT_PORT.print(F("00"));
  if (inAnaOpt1 >= 10 && inAnaOpt1 < 100) OUT_PORT.print(F("0"));
  OUT_PORT.print(inAnaOpt1);
  OUT_PORT.print(F(" / "));
  if (inAnaOpt2 < 10) OUT_PORT.print(F("00"));
  if (inAnaOpt2 >= 10 && inAnaOpt2 < 100) OUT_PORT.print(F("0"));
  OUT_PORT.println(inAnaOpt2);
  OUT_PORT.print(F("MLX:"));
  for (uint8_t i = 0; i < maxMlx; i++) {
    if (mlxAddresses[i] != 0x00) {
      OUT_PORT.print(mlxValues[i], 0);
      OUT_PORT.print(F(" "));
    }
  }
}

/**************************************************************
  #printData2 > Print realtime data on one output (oled screen or bluetooth serial)
**************************************************************/
void printData2(gps_fix & fix_data, Print & OUT_PORT) {
  uint8_t bitShift = B00000001, tmpComp;
  uint8_t printedValues = 0;
  if (fix_data.valid.date && fix_data.valid.time) {
    if (fix_data.dateTime.date < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.date);
    OUT_PORT.print(F("/"));
    if (fix_data.dateTime.month < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.month);
    OUT_PORT.print(F("/"));
    if (fix_data.dateTime.year < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.year);
    OUT_PORT.print(F(" "));
    if (fix_data.dateTime.hours < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.hours);
    OUT_PORT.print(F(":"));
    if (fix_data.dateTime.minutes < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.minutes);
    OUT_PORT.print(F(":"));
    if (fix_data.dateTime.seconds < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.println(fix_data.dateTime.seconds);
  } else {
    OUT_PORT.println(F("No GPS signal"));
  }
  OUT_PORT.print(F("CALIB (SGAM):"));
  /*OUT_PORT.print(calSys);
    OUT_PORT.print(F(";"));
    OUT_PORT.print(calGyro);
    OUT_PORT.print(F(";"));
    OUT_PORT.print(calAccel);
    OUT_PORT.print(F(";"));
    OUT_PORT.println(calMag);*/

  OUT_PORT.print(F("9AXIS:"));
  /*OUT_PORT.print(event.orientation.x, 0);
    OUT_PORT.print(F(";"));
    OUT_PORT.print(event.orientation.y, 0);
    OUT_PORT.print(F(";"));
    OUT_PORT.println(event.orientation.z, 0);*/

  /*for (uint8_t i = 0; i < 8; i++) { // We parse the 8 analog values
    tmpComp = bitShift & anaDisabledBits;
    if (i == 6 && printedValues > 0) {
      OUT_PORT.println();
    }
    if (tmpComp == bitShift) { // If analog is disabled ...
      if (i == 6) {
        OUT_PORT.println(F("inAnaThrottle:N/C"));
      } else if (i == 7) {
        OUT_PORT.println(F("GEAR:N/C"));
      }
    } else { // If analog port is enabled ...
      if (i == 6) { // If "inAnaThrottle" ...
        OUT_PORT.print(F("inAnaThrottle:"));
        if (inAnaThrottle < 10) OUT_PORT.print(F("00"));
        if (inAnaThrottle >= 10 && inAnaThrottle < 100) OUT_PORT.print(F("0"));
        OUT_PORT.println(inAnaThrottle);
      } else if (i == 7) { // If "Gear" ...
        OUT_PORT.print(F("GEAR:"));
        OUT_PORT.println(gear);
      } else { // If other analog ports ...
        if (printedValues % 3 == 0) {
          if (printedValues > 0) {
            OUT_PORT.println();
          }
          OUT_PORT.print(F("ADC:"));
        }
        if (anaValues[i] < 10) OUT_PORT.print(F("000"));
        if (anaValues[i] >= 10 && anaValues[i] < 100) OUT_PORT.print(F("00"));
        if (anaValues[i] >= 100 && anaValues[i] < 1000) OUT_PORT.print(F("0"));
        OUT_PORT.print(anaValues[i]);
        OUT_PORT.print(F(" "));
        printedValues++;
      }
    }
    bitShift = bitShift << 1;
    }*/
  OUT_PORT.print(F("AMB. T:"));
  //OUT_PORT.println(temperature);
  OUT_PORT.print(F("MLX T1:"));
  for (uint8_t i = 0; i < maxMlx; i++) {
    if (mlxAddresses[i] != 0x00) {
      if (i == 2) {
        OUT_PORT.println(mlxValues[i], 0);
        OUT_PORT.print(F("MLX T2:"));
      } else {
        OUT_PORT.print(mlxValues[i], 0);
        OUT_PORT.print(F(" "));
      }
    }
  }
}

/**************************************************************
  GPS time adjust
**************************************************************/
void adjustTime(NeoGPS::time_t & dt) {
  NeoGPS::clock_t seconds = dt; // convert date/time structure to seconds

#ifdef CALCULATE_DST
  //  Calculate DST changeover times once per reset and year!
  static NeoGPS::time_t  changeover;
  static NeoGPS::clock_t springForward, fallBack;

  if ((springForward == 0) || (changeover.year != dt.year)) {

    //  Calculate the spring changeover time (seconds)
    changeover.year    = dt.year;
    changeover.month   = springMonth;
    changeover.date    = springDate;
    changeover.hours   = springHour;
    changeover.minutes = 0;
    changeover.seconds = 0;
    changeover.set_day();
    // Step back to a Sunday, if day != SUNDAY
    changeover.date -= (changeover.day - NeoGPS::time_t::SUNDAY);
    springForward = (NeoGPS::clock_t) changeover;

    //  Calculate the fall changeover time (seconds)
    changeover.month   = fallMonth;
    changeover.date    = fallDate;
    changeover.hours   = fallHour - 1; // to account for the "apparent" DST +1
    changeover.set_day();
    // Step back to a Sunday, if day != SUNDAY
    changeover.date -= (changeover.day - NeoGPS::time_t::SUNDAY);
    fallBack = (NeoGPS::clock_t) changeover;
  }
#endif

  //  First, offset from UTC to the local timezone
  seconds += zone_offset;

#ifdef CALCULATE_DST
  //  Then add an hour if DST is in effect
  if ((springForward <= seconds) && (seconds < fallBack))
    seconds += NeoGPS::SECONDS_PER_HOUR;
#endif

  dt = seconds; // convert seconds back to a date/time structure
}

/**************************************************************
  Get datetime for files creation/modification on SD Card
**************************************************************/
void dateTimeSd(uint16_t* date, uint16_t* time) {
  *date = FAT_DATE(fix_data.dateTime.year + 2000, fix_data.dateTime.month, fix_data.dateTime.date); // return date using FAT_DATE macro to format fields
  *time = FAT_TIME(fix_data.dateTime.hours, fix_data.dateTime.minutes, fix_data.dateTime.seconds); // return time using FAT_TIME macro to format fields
}
