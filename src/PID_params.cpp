/**
 * @file PID_params.cpp
 * @author Ian Gant (Igant01)
 * @brief PID parameter management implementation
 * @version 0.1
 * @date 2026-03-17
 */

#include "PID_params.h"

// Default PID values - can be tuned as needed
const PIDParameters PIDManager::DEFAULT_TORQUE_PID(1.0, 0.1, 0.01);
const PIDParameters PIDManager::DEFAULT_VELOCITY_PID(0.5, 0.05, 0.005);
const PIDParameters PIDManager::DEFAULT_POSITION_PID(0.3, 0.03, 0.003);

// Static member initialization
PIDParameters PIDManager::torquePID = PIDManager::DEFAULT_TORQUE_PID;
PIDParameters PIDManager::velocityPID = PIDManager::DEFAULT_VELOCITY_PID;
PIDParameters PIDManager::positionPID = PIDManager::DEFAULT_POSITION_PID;

// Torque PID EEPROM functions
void PIDManager::loadTorquePID() {
    if (EEPROM.read(48) == 1) {
        EEPROM.get(24, torquePID.Kp);
        EEPROM.get(32, torquePID.Ki);
        EEPROM.get(40, torquePID.Kd);
    } else {
        torquePID = DEFAULT_TORQUE_PID;
    }
}

void PIDManager::saveTorquePID() {
    EEPROM.put(24, torquePID.Kp);
    EEPROM.put(32, torquePID.Ki);
    EEPROM.put(40, torquePID.Kd);
    EEPROM.write(48, 1);
}

// Velocity PID EEPROM functions
void PIDManager::loadVelocityPID() {
    if (EEPROM.read(80) == 1) {
        EEPROM.get(56, velocityPID.Kp);
        EEPROM.get(64, velocityPID.Ki);
        EEPROM.get(72, velocityPID.Kd);
    } else {
        velocityPID = DEFAULT_VELOCITY_PID;
    }
}

void PIDManager::saveVelocityPID() {
    EEPROM.put(56, velocityPID.Kp);
    EEPROM.put(64, velocityPID.Ki);
    EEPROM.put(72, velocityPID.Kd);
    EEPROM.write(80, 1);
}

// Position PID EEPROM functions
void PIDManager::loadPositionPID() {
    if (EEPROM.read(112) == 1) {
        EEPROM.get(88, positionPID.Kp);
        EEPROM.get(96, positionPID.Ki);
        EEPROM.get(104, positionPID.Kd);
    } else {
        positionPID = DEFAULT_POSITION_PID;
    }
}

void PIDManager::savePositionPID() {
    EEPROM.put(88, positionPID.Kp);
    EEPROM.put(96, positionPID.Ki);
    EEPROM.put(104, positionPID.Kd);
    EEPROM.write(112, 1);
}

// Load all PID parameters
void PIDManager::loadAllPID() {
    loadTorquePID();
    loadVelocityPID();
    loadPositionPID();
}

// Save all PID parameters
void PIDManager::saveAllPID() {
    saveTorquePID();
    saveVelocityPID();
    savePositionPID();
}

// Validity checks
bool PIDManager::torquePIDValid() {
    return EEPROM.read(48) == 1;
}

bool PIDManager::velocityPIDValid() {
    return EEPROM.read(80) == 1;
}

bool PIDManager::positionPIDValid() {
    return EEPROM.read(112) == 1;
}

// Set PID parameters
void PIDManager::setTorquePID(double kp, double ki, double kd) {
    torquePID = PIDParameters(kp, ki, kd);
}

void PIDManager::setVelocityPID(double kp, double ki, double kd) {
    velocityPID = PIDParameters(kp, ki, kd);
}

void PIDManager::setPositionPID(double kp, double ki, double kd) {
    positionPID = PIDParameters(kp, ki, kd);
}
