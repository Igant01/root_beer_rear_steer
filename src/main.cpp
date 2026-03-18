/**
 * @file main.cpp
 * @author Ian Gant (Igant01)
 * @brief 
 * @version 0.1
 * @date 2024-06-18
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <LMT87.h>
#include <current_sense.h>
#include <motor_driver.h>
#include <SAS.h>
#include <PID_v1.h>
#include <EEPROM.h>
#include "PID_params.h"

#define SASRB 18  //rear steering angle b
#define SASRA 17  //rear steering angle a
#define SASFB 16  //front steering angle b
#define SASFA 15  //front steering angle a
#define CURRA 14  //current sense direction a
#define CURRB 19 //current sense direction b


// Hardware objects
LMT87 tempSensor;
current_sense forwardCurrent(CURRA);
current_sense reverseCurrent(CURRB);
motor_driver driver;
SAS frontSAS(SASFA, SASFB, 0);
SAS rearSAS(SASRA, SASRB, 16);

// Task intervals (milliseconds)
const unsigned long interval1 = 1;   // 1000Hz
const unsigned long interval2 = 2;   // 500Hz
const unsigned long interval3 = 5;   // 200Hz

// Time trackers
unsigned long prevTime1 = 0;
unsigned long prevTime2 = 0;
unsigned long prevTime3 = 0;

//master control enable
bool runFlag = true;

// USB connection tracking
bool wasUSBConnected = false;

// Calibration settings
bool allowEEPROMWrite = false;
bool motorEnabled = false; // Track motor enabled state
bool sweepActive = false; // Flag to control continuous sweep
const float currentThreshold = 500.0; // mA - adjust based on your motor
const int calibrationMotorSpeedLeft = 100;  // 0-512 (left side)
const int calibrationMotorSpeedRight = 900; // 512-1023 (right side)

// PID Loop data for plotting (target vs actual)
struct LoopData {
    double target;
    double actual;
    double error;
    unsigned long timestamp;
};

// Circular buffers for loop data (100 samples each)
#define LOOP_DATA_BUFFER_SIZE 100
LoopData torqueLoopData[LOOP_DATA_BUFFER_SIZE];
int torqueLoopIndex = 0;
LoopData velocityLoopData[LOOP_DATA_BUFFER_SIZE];
int velocityLoopIndex = 0;
LoopData positionLoopData[LOOP_DATA_BUFFER_SIZE];
int positionLoopIndex = 0;

// Flags to enable data streaming for plots
bool streamTorqueData = false;
bool streamVelocityData = false;
bool streamPositionData = false;
unsigned long streamStartMicros = 0;  // Time reference for streaming (resets on start)


void loop1();
void loop2();
void loop3();
void usbConnectedFunction();
bool calibrateRearSAS();
void sweepAndLogSAS();

void setup() {
  Serial.begin(115200);
  
  // Load PID parameters from EEPROM
  PIDManager::loadAllPID();
  
  // Validate EEPROM calibration flags - if not exactly 1, clear to 0 (mark as invalid)
  byte rearFlag = EEPROM.read(4);
  byte frontFlag = EEPROM.read(20);
  
  // If flag is garbage (not 0 or 1), clear it
  if (rearFlag != 0 && rearFlag != 1) {
    EEPROM.write(4, 0);
  }
  if (frontFlag != 0 && frontFlag != 1) {
    EEPROM.write(20, 0);
  }
  
  // Load calibration from EEPROM (will only load if flag == 1)
  rearSAS.loadFromEEPROM();
  frontSAS.loadFromEEPROM();
}

void loop() {
  unsigned long currentMillis = millis();

  // Task 1
  if (currentMillis - prevTime1 >= interval1 && (runFlag || streamTorqueData)) {
    prevTime1 = currentMillis;
    loop1();
  }

  // Task 2 
  if (currentMillis - prevTime2 >= interval2 && (runFlag || streamVelocityData)) {
    prevTime2 = currentMillis;
    loop2();
  }

  // Task 3 
  if (currentMillis - prevTime3 >= interval3 && (runFlag || streamPositionData)) {
    prevTime3 = currentMillis;
    loop3();
  }

  // Handle USB connection state changes
  bool isUSBConnected = (bool)Serial;
  
  if(isUSBConnected && !wasUSBConnected) {
    // Just connected - enter USB control mode
    runFlag = false;
    wasUSBConnected = true;
  }
  else if(!isUSBConnected && wasUSBConnected) {
    // Just disconnected - resume normal operation
    runFlag = true;
    wasUSBConnected = false;
  }
  
  // Process USB commands when connected (runFlag toggle is controlled by T command)
  if(isUSBConnected) {
    usbConnectedFunction();
  }
}

void loop1() {  // High speed (torque control loop, safety monitoring)
  // TODO: Implement torque control using motor current feedback
  static double torqueTarget = 0.0;
  static double torqueActual = 0.0;
  
  // Update loop control calculations here
  // torqueTarget = desired torque setpoint
  // torqueActual = measured torque from current sensor feedback
  
  // Stream data if enabled
  if (streamTorqueData) {
    unsigned long now = micros() - streamStartMicros;
    Serial.print("TC,");
    Serial.print(torqueTarget, 3);
    Serial.print(",");
    Serial.print(torqueActual, 3);
    Serial.print(",");
    Serial.println(now / 1000.0, 3);  // Convert micros to ms
  }
}

void loop2() {  // Low speed (velocity control loop, data logging)
  // TODO: Implement velocity control using motor speed feedback
  static double velocityTarget = 0.0;
  static double velocityActual = 0.0;
  
  // Update loop control calculations here
  // velocityTarget = desired velocity setpoint
  // velocityActual = measured velocity from encoder/sensor feedback
  
  // Stream data if enabled
  if (streamVelocityData) {
    unsigned long now = micros() - streamStartMicros;
    Serial.print("VC,");
    Serial.print(velocityTarget, 3);
    Serial.print(",");
    Serial.print(velocityActual, 3);
    Serial.print(",");
    Serial.println(now / 1000.0, 3);  // Convert micros to ms
  }
}

void loop3() {  // Very low speed (steering/position control loop, data logging)
  // TODO: Implement steering position control using steering angle feedback
  static double steeringTarget = 0.0;
  static double steeringActual = 0.0;
  
  // Update loop control calculations here
  // steeringTarget = desired steering angle setpoint
  // steeringActual = measured steering angle from SAS sensor feedback
  
  // Stream data if enabled
  if (streamPositionData) {
    unsigned long now = micros() - streamStartMicros;
    Serial.print("LP,");
    Serial.print(steeringTarget, 3);
    Serial.print(",");
    Serial.print(steeringActual, 3);
    Serial.print(",");
    Serial.println(now / 1000.0, 3);  // Convert micros to ms
  }
}

bool calibrateRearSAS() {
  // Auto-calibrate rear SAS by moving motor and detecting mechanical limits via current spike
  Serial.println("CAL,START");
  
  driver.enable();
  delay(100);
  
  // Calibrate LEFT (move motor to left until current spike on forwardCurrent)
  Serial.println("CAL,LEFT");
  driver.set(calibrationMotorSpeedLeft); // Move left
  delay(1000); // Allow motor to start moving
  
  bool leftFound = false;
  unsigned long timeoutStart = millis();
  while(!leftFound && (millis() - timeoutStart < 10000)) {
    float current = forwardCurrent.read();
    if(current > currentThreshold) {
      leftFound = true;
      rearSAS.setLeft();
      Serial.print("CAL,LEFT,OK,");
      Serial.print(rearSAS.getPotA());
      Serial.print(",");
      Serial.println(rearSAS.getPotB());
    }
    delay(50);
  }
  
  if(!leftFound) {
    Serial.println("CAL,LEFT,FAIL");
    driver.disable();
    return false;
  }
  
  // Move to center before going right
  driver.set(512);
  delay(500);
  
  // Calibrate RIGHT (move motor to right until current spike on reverseCurrent)
  Serial.println("CAL,RIGHT");
  driver.set(calibrationMotorSpeedRight); // Move right
  delay(1000); // Allow motor to start moving
  
  bool rightFound = false;
  timeoutStart = millis();
  while(!rightFound && (millis() - timeoutStart < 10000)) {
    float current = reverseCurrent.read();
    if(current > currentThreshold) {
      rightFound = true;
      rearSAS.setRight();
      Serial.print("CAL,RIGHT,OK,");
      Serial.print(rearSAS.getPotA());
      Serial.print(",");
      Serial.println(rearSAS.getPotB());
    }
    delay(50);
  }
  
  if(!rightFound) {
    Serial.println("CAL,RIGHT,FAIL");
    driver.disable();
    return false;
  }
  
  // Return to center
  driver.set(512);
  delay(200);
  driver.disable();
  
  Serial.println("CAL,OK");
  return true;
}

void sweepAndLogSAS() {
  // Sweep steering through full range and log raw pot values, repeating until stopped
  Serial.println("SWEEP,START");
  
  sweepActive = true;
  driver.enable();
  delay(100);
  
  // Sweep from left to right (0 to 1023)
  Serial.println("SWEEP,DATA,motorVal,rearPotA,rearPotB,frontPotA,frontPotB");
  
  while(sweepActive) {
    for(int motorVal = 0; motorVal <= 1023; motorVal += 10) {
      if(!sweepActive) break; // Check for stop between each step
      
      driver.set(motorVal);
      delay(100); // Give motor time to move and settle
      
      // Log current position
      Serial.print("SWEEP,");
      Serial.print(motorVal);
      Serial.print(",");
      Serial.print(rearSAS.getPotA());
      Serial.print(",");
      Serial.print(rearSAS.getPotB());
      Serial.print(",");
      Serial.print(frontSAS.getPotA());
      Serial.print(",");
      Serial.println(frontSAS.getPotB());
      
      // Check for stop command
      if(Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        if(cmd.trim().equalsIgnoreCase("Z")) {
          Serial.println("SWEEP,STOPPED");
          sweepActive = false;
          break;
        }
      }
    }
  }
  
  // Return to center
  driver.set(512);
  delay(200);
  driver.disable();
  
  Serial.println("SWEEP,OK");
}

void usbConnectedFunction() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.length() == 0) return;

    char cmd = command.charAt(0);
    
    switch(cmd) {
      // Toggle runFlag (normal operation vs USB control)
      case 'T': // Toggle
        runFlag = !runFlag;
        Serial.print("T,");
        Serial.println(runFlag ? "ON" : "OFF");
        break;
      
      // Auto-calibrate rear SAS
      case 'C': // Calibrate
        if (command.length() > 1) {
          char subcmd = command.charAt(1);
          if (subcmd == 'A') { // CA - auto calibrate
            calibrateRearSAS();
          }
        }
        break;
      
      // Sweep through full range and log pot values - toggle on/off
      case 'Z': // Sweep (Z chosen as uncommon letter)
        if(!sweepActive) {
          // Start sweep
          sweepAndLogSAS();
        } else {
          // Stop sweep (flag will be checked in sweepAndLogSAS loop)
          sweepActive = false;
          Serial.println("SWEEP,STOPPING");
        }
        break;
      
      // View limit values
      case 'V': // View calibration limits (V) or EEPROM values (VE)
        if (command.length() > 1 && command.charAt(1) == 'E') {
          // VE - View EEPROM values
          int16_t rearL, rearR, frontL, frontR;
          byte rearFlag = EEPROM.read(4);
          byte frontFlag = EEPROM.read(20);
          
          if (rearFlag == 1) {
            EEPROM.get(0, rearL);
            EEPROM.get(2, rearR);
          } else {
            rearL = 0; rearR = 0;
          }
          
          if (frontFlag == 1) {
            EEPROM.get(16, frontL);
            EEPROM.get(18, frontR);
          } else {
            frontL = 0; frontR = 0;
          }
          
          Serial.print("VE,rear,");
          Serial.print(rearL);
          Serial.print(",");
          Serial.print(rearR);
          Serial.print(",front,");
          Serial.print(frontL);
          Serial.print(",");
          Serial.println(frontR);
        } else {
          // V - View working limits (RAM values)
          Serial.print("V,");
          Serial.print("rear,");
          Serial.print(rearSAS.getLeftLimit());
          Serial.print(",");
          Serial.print(rearSAS.getRightLimit());
          Serial.print(",");
          Serial.print(rearSAS.matchesEEPROM() ? "1" : "0");
          Serial.print(",front,");
          Serial.print(frontSAS.getLeftLimit());
          Serial.print(",");
          Serial.print(frontSAS.getRightLimit());
          Serial.print(",");
          Serial.println(frontSAS.matchesEEPROM() ? "1" : "0");
        }
        break;
      
      // SAS Calibration - Rear
      case 'R': // Rear SAS
        if (command.length() > 1) {
          char subcmd = command.charAt(1);
          if (subcmd == 'L') {
            rearSAS.setLeft();
            Serial.println("RL,OK");
          } else if (subcmd == 'R') {
            rearSAS.setRight();
            Serial.println("RR,OK");
          }
        }
        break;
      
      // SAS Calibration - Front
      case 'F': // Front SAS
        if (command.length() > 1) {
          char subcmd = command.charAt(1);
          if (subcmd == 'L') {
            frontSAS.setLeft();
            Serial.println("FL,OK");
          } else if (subcmd == 'R') {
            frontSAS.setRight();
            Serial.println("FR,OK");
          }
        }
        break;
      
      // Save to EEPROM
      case 'W': // Write/Save
        if (command.length() > 1) {
          char subcmd = command.charAt(1);
          if (subcmd == 'S') { // WS - save
            if (allowEEPROMWrite) {
              rearSAS.saveToEEPROM();
              frontSAS.saveToEEPROM();
              Serial.println("WS,OK");
            } else {
              Serial.println("WS,LOCKED");
            }
          } else if (subcmd == 'L') { // WL - load
            rearSAS.loadFromEEPROM();
            frontSAS.loadFromEEPROM();
            Serial.println("WL,OK");
          } else if (subcmd == 'T') { // WT - toggle write protection
            allowEEPROMWrite = !allowEEPROMWrite;
            Serial.print("WT,");
            Serial.println(allowEEPROMWrite ? "UNLOCKED" : "LOCKED");
          }
        }
        break;
      
      // Motor Enable/Disable
      case 'E': // Enable
        driver.enable();
        motorEnabled = true;
        Serial.println("E,OK");
        break;
      
      case 'D': // Disable
        driver.disable();
        motorEnabled = false;
        Serial.println("D,OK");
        break;
      
      // Motor control: M<0-1023>
      case 'M': // Motor jog
        if (motorEnabled && command.length() > 1) {
          int value = command.substring(1).toInt();
          value = constrain(value, 0, 1023);
          driver.set(value);
          Serial.print("M,");
          Serial.println(value);
        }
        break;
      
      // Get raw pot positions: P
      case 'P': // Raw pot positions
        Serial.print("P,");
        Serial.print(rearSAS.getPotA());
        Serial.print(",");
        Serial.print(rearSAS.getPotB());
        Serial.print(",");
        Serial.print(frontSAS.getPotA());
        Serial.print(",");
        Serial.println(frontSAS.getPotB());
        break;
      
      // Status: S
      case 'S': // Status
        Serial.print("S,");
        Serial.print(rearSAS.positionRaw());
        Serial.print(",");
        Serial.println(frontSAS.positionRaw());
        break;
      
      // Debug: D - show EEPROM contents
      case 'G': // Get EEPROM debug info
        Serial.print("DBG,rearL:");
        Serial.print(rearSAS.getLeftLimit());
        Serial.print(",rearR:");
        Serial.print(rearSAS.getRightLimit());
        Serial.print(",rearMatch:");
        Serial.print(rearSAS.matchesEEPROM() ? "1" : "0");
        Serial.print(",frontL:");
        Serial.print(frontSAS.getLeftLimit());
        Serial.print(",frontR:");
        Serial.print(frontSAS.getRightLimit());
        Serial.print(",frontMatch:");
        Serial.println(frontSAS.matchesEEPROM() ? "1" : "0");
        break;
      
      // PID Parameter Commands
      // IC = Get current torque PID params
      // IV = Get current velocity PID params  
      // IP = Get current position PID params
      case 'I':
        if (command.length() > 1) {
          char subcmd = command.charAt(1);
          if (subcmd == 'C') { // IC - Get torque PID
            Serial.print("IC,");
            Serial.print(PIDManager::torquePID.Kp, 6);
            Serial.print(",");
            Serial.print(PIDManager::torquePID.Ki, 6);
            Serial.print(",");
            Serial.println(PIDManager::torquePID.Kd, 6);
          } else if (subcmd == 'V') { // IV - Get velocity PID
            Serial.print("IV,");
            Serial.print(PIDManager::velocityPID.Kp, 6);
            Serial.print(",");
            Serial.print(PIDManager::velocityPID.Ki, 6);
            Serial.print(",");
            Serial.println(PIDManager::velocityPID.Kd, 6);
          } else if (subcmd == 'P') { // IP - Get position PID
            Serial.print("IP,");
            Serial.print(PIDManager::positionPID.Kp, 6);
            Serial.print(",");
            Serial.print(PIDManager::positionPID.Ki, 6);
            Serial.print(",");
            Serial.println(PIDManager::positionPID.Kd, 6);
          } else if (subcmd == 'A') { // IA - Get all PID params
            Serial.print("IAT,");
            Serial.print(PIDManager::torquePID.Kp, 6);
            Serial.print(",");
            Serial.print(PIDManager::torquePID.Ki, 6);
            Serial.print(",");
            Serial.println(PIDManager::torquePID.Kd, 6);
            Serial.print("IAV,");
            Serial.print(PIDManager::velocityPID.Kp, 6);
            Serial.print(",");
            Serial.print(PIDManager::velocityPID.Ki, 6);
            Serial.print(",");
            Serial.println(PIDManager::velocityPID.Kd, 6);
            Serial.print("IAP,");
            Serial.print(PIDManager::positionPID.Kp, 6);
            Serial.print(",");
            Serial.print(PIDManager::positionPID.Ki, 6);
            Serial.print(",");
            Serial.println(PIDManager::positionPID.Kd, 6);
          }
        }
        break;
      
      // Set PID Parameters
      // HC<kp,ki,kd> = Set torque PID
      // HV<kp,ki,kd> = Set velocity PID
      // HP<kp,ki,kd> = Set position PID
      case 'H':
        if (command.length() > 1) {
          char subcmd = command.charAt(1);
          // Parse format: H[C/V/P]<kp,ki,kd>
          String params = command.substring(2);
          int comma1 = params.indexOf(',');
          int comma2 = params.lastIndexOf(',');
          
          if (comma1 > 0 && comma2 > comma1) {
            double kp = atof(params.substring(0, comma1).c_str());
            double ki = atof(params.substring(comma1 + 1, comma2).c_str());
            double kd = atof(params.substring(comma2 + 1).c_str());
            
            if (subcmd == 'C') { // HC - Set torque PID
              PIDManager::setTorquePID(kp, ki, kd);
              Serial.println("HC,OK");
            } else if (subcmd == 'V') { // HV - Set velocity PID
              PIDManager::setVelocityPID(kp, ki, kd);
              Serial.println("HV,OK");
            } else if (subcmd == 'P') { // HP - Set position PID
              PIDManager::setPositionPID(kp, ki, kd);
              Serial.println("HP,OK");
            }
          } else {
            Serial.println("ERR,InvalidFormat");
          }
        }
        break;
      
      // Data Streaming for plotting
      // DC = Toggle torque loop data streaming
      // DV = Toggle velocity loop data streaming
      // DP = Toggle position loop data streaming
      case 'U': // User data stream
        if (command.length() > 1) {
          char subcmd = command.charAt(1);
          if (subcmd == 'C') { // UC - Toggle torque data streaming
            streamTorqueData = !streamTorqueData;
            if (streamTorqueData) streamStartMicros = micros();  // Reset time on start
            Serial.print("UC,");
            Serial.println(streamTorqueData ? "ON" : "OFF");
          } else if (subcmd == 'V') { // UV - Toggle velocity data streaming
            streamVelocityData = !streamVelocityData;
            Serial.print("UV,");
            Serial.println(streamVelocityData ? "ON" : "OFF");
          } else if (subcmd == 'P') { // UP - Toggle position data streaming
            streamPositionData = !streamPositionData;
            Serial.print("UP,");
            Serial.println(streamPositionData ? "ON" : "OFF");
          } else if (subcmd == 'S') { // US - Stop all streaming immediately
            streamTorqueData = false;
            streamVelocityData = false;
            streamPositionData = false;
            Serial.println("US,OK");
          }
        }
        break;
      
      // WPID = Write PID to EEPROM
      case 'X':
        if (command.length() > 1 && command.substring(1) == "PID") {
          if (allowEEPROMWrite) {
            PIDManager::saveAllPID();
            Serial.println("XPID,OK");
          } else {
            Serial.println("XPID,LOCKED");
          }
        }
        break;
      
      default:
        Serial.println("ERR");
        break;
    }
  }
}
