/**
 * @file SAS.h
 * @author Ian Gant (Igant01)
 * @brief 
 * @version 0.1
 * @date 2024-06-18
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef SAS_H
#define SAS_H

#include <cstdint>

#define EEPROM_REAR_ADDR 0
#define EEPROM_FRONT_ADDR 16

enum SensorFault
{
    NO_FAULT,
    SENSOR_FAILURE,
    CORRELATION_FAULT,
    POT1_RANGE_FAULT,
    POT2_RANGE_FAULT,
    POT1_DRIFT_FAULT,
    POT2_DRIFT_FAULT,
    RATE_FAULT,
    STUCK_FAULT
};

class SAS {

    private:
        int16_t leftLimit, rightLimit; //position limits (0-1023) at left and right extremes
        float leftAngle, rightAngle;
        bool calibratedLeft,calibratedRight;
        int pinA,pinB;
        int eeprom_addr;
        int lastPosition = 0;
        int mismatchCounter = 0;
        int stuckCounter = 0;
        long driftAccumulator1 = 0;
        long driftAccumulator2 = 0;
        int driftSamples = 0;
        int offset = 0;
        bool calibrated = false;
        bool degradedMode = false;
        bool faultActive = false;
        SensorFault faultType = NO_FAULT;
        
    public:
        SAS(int pinA,int pinB,int eeprom_addr);
        void setLeft();
        void setRight(); 
        int getPotA();
        int getPotB();
        void saveToEEPROM();
        void loadFromEEPROM();
        void printCalibration();
        bool isCalibrated();
        int16_t getLeftLimit();
        int16_t getRightLimit();
        bool matchesEEPROM();
        void clearFault();
        int getPosition();
        int positionRaw();
        bool error();
        float angle();

};

#endif