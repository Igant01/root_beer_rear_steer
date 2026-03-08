/**
 * @file SAS.cpp
 * @author Ian Gant (Igant01)
 * @brief 
 * @version 0.1
 * @date 2024-06-18
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <SAS.h>
#include <Arduino.h>
#include <EEPROM.h>

SAS::SAS(int PINA,int PINB, int eeprom_addr){ //A top B bottom this is arbitrary
    leftLimit = 0;
    rightLimit = 0;
    calibratedLeft = false;
    calibratedRight = false;
    pinA = PINA;
    pinB = PINB;
    this->eeprom_addr = eeprom_addr;
    pinMode(pinA,INPUT);
    pinMode(pinB,INPUT);
}

void SAS::setLeft(){
    // Store average of pot values at left extreme
    leftLimit = (analogRead(pinA) + analogRead(pinB)) / 2;
    calibratedLeft = true;
}

void SAS::setRight(){
    // Store average of pot values at right extreme
    rightLimit = (analogRead(pinA) + analogRead(pinB)) / 2;
    calibratedRight = true;
}

int SAS::getPotA(){
    return analogRead(pinA);
}

int SAS::getPotB(){
    return analogRead(pinB);
}

void SAS::saveToEEPROM(){
    if(!calibratedLeft || !calibratedRight) return;
    EEPROM.put(eeprom_addr, leftLimit);
    EEPROM.put(eeprom_addr + 2, rightLimit);
    EEPROM.put(eeprom_addr + 4, (byte)1); // calibration flag
}

void SAS::loadFromEEPROM(){
    byte calibFlag = EEPROM.read(eeprom_addr + 4);
    if(calibFlag == 1) {
        EEPROM.get(eeprom_addr, leftLimit);
        EEPROM.get(eeprom_addr + 2, rightLimit);
        calibratedLeft = true;
        calibratedRight = true;
    }
}

void SAS::printCalibration(){
    Serial.print("Cal: ");
    Serial.print(leftLimit);
    Serial.print(",");
    Serial.println(rightLimit);
}

bool SAS::isCalibrated(){
    return calibratedLeft && calibratedRight;
}

int16_t SAS::getLeftLimit(){
    return leftLimit;
}

int16_t SAS::getRightLimit(){
    return rightLimit;
}

bool SAS::matchesEEPROM(){
    // Read values from EEPROM and compare
    int16_t eepromLeftLimit, eepromRightLimit;
    byte calibFlag = EEPROM.read(eeprom_addr + 4);
    
    if(calibFlag != 1) return false; // No calibration in EEPROM
    
    EEPROM.get(eeprom_addr, eepromLeftLimit);
    EEPROM.get(eeprom_addr + 2, eepromRightLimit);
    
    return (leftLimit == eepromLeftLimit && rightLimit == eepromRightLimit);
}

void SAS::clearFault(){
    faultActive = false;
    faultType = NO_FAULT;
}

int SAS::getPosition(){
    const int MIN_VAL = 10;
    const int MAX_VAL = 1013;

    const int BASE_TOLERANCE = 40;
    const int MAX_RATE = 80;

    const int MISMATCH_LIMIT = 10;
    const int STUCK_LIMIT = 200;

    const int DRIFT_WARNING = 25;

    const int DEAD_BAND = 2;

    // ---------- FILTERED ADC READ ----------
    long sum1 = 0;
    long sum2 = 0;

    for (int i = 0; i < 5; i++)
    {
        sum1 += analogRead(pinA);
        sum2 += analogRead(pinB);
    }

    int pot1 = sum1 / 5;
    int pot2 = sum2 / 5;

    bool pot1Valid = (pot1 > MIN_VAL && pot1 < MAX_VAL);
    bool pot2Valid = (pot2 > MIN_VAL && pot2 < MAX_VAL);

    int newPos = lastPosition;

    // ---------- RANGE CHECK ----------
    if (!pot1Valid && !pot2Valid)
    {
        faultActive = true;
        faultType = SENSOR_FAILURE;
        return lastPosition;
    }

    if (!pot1Valid)
    {
        faultActive = true;
        faultType = POT1_RANGE_FAULT;
        degradedMode = true;
    }

    if (!pot2Valid)
    {
        faultActive = true;
        faultType = POT2_RANGE_FAULT;
        degradedMode = true;
    }

    // ---------- AUTO CALIBRATION ----------
    if (!calibrated && pot1Valid && pot2Valid)
    {
        offset = pot1 - pot2;
        calibrated = true;
    }

    int pot2Corrected = pot2 + offset;

    // ---------- CORRELATION CHECK ----------
    if (pot1Valid && pot2Valid)
    {
        int diff = abs(pot1 - pot2Corrected);

        int positionEstimate = (pot1 + pot2Corrected) / 2;

        int dynamicTolerance = BASE_TOLERANCE + (positionEstimate / 50);

        if (diff <= dynamicTolerance)
        {
            mismatchCounter = 0;
            newPos = positionEstimate;

            driftAccumulator1 += diff;
            driftAccumulator2 += diff;
            driftSamples++;

            if (driftSamples > 1000)
            {
                int avgDrift = (driftAccumulator1 + driftAccumulator2) / (2 * driftSamples);

                if (avgDrift > DRIFT_WARNING && !faultActive)
                {
                    faultActive = true;
                    faultType = CORRELATION_FAULT;
                }

                driftAccumulator1 = 0;
                driftAccumulator2 = 0;
                driftSamples = 0;
            }
        }
        else
        {
            mismatchCounter++;

            if (mismatchCounter > MISMATCH_LIMIT && !faultActive)
            {
                faultActive = true;
                faultType = CORRELATION_FAULT;
                return lastPosition;
            }

            newPos = positionEstimate;
        }
    }
    else if (pot1Valid)
    {
        newPos = pot1;
    }
    else
    {
        newPos = pot2Corrected;
    }

    // ---------- RATE CHECK ----------
    if (abs(newPos - lastPosition) > MAX_RATE)
    {
        if (!faultActive)
        {
            faultActive = true;
            faultType = RATE_FAULT;
        }

        return lastPosition;
    }

    // ---------- STUCK DETECTION ----------
    if (newPos == lastPosition)
    {
        stuckCounter++;

        if (stuckCounter > STUCK_LIMIT && !faultActive)
        {
            faultActive = true;
            faultType = STUCK_FAULT;
        }
    }
    else
    {
        stuckCounter = 0;
    }

    // ---------- DEAD BAND FILTER ----------
    if (abs(newPos - lastPosition) <= DEAD_BAND)
        newPos = lastPosition;

    lastPosition = newPos;

    return newPos;
}

//convert raw position to angle in degrees 
float SAS::angle(){
    
}

int SAS::positionRaw(){
    int outMin = 0; //output minimum value
    int outMax = 1023;  //output maximum value
    if(calibratedLeft && calibratedRight && (!error())){
        int pinA_val = analogRead(pinA);
        int pinB_val = analogRead(pinB);
        int pinAvg = (pinA_val + pinB_val) / 2; //average of current position
        return map(pinAvg, leftLimit, rightLimit, outMin, outMax); //map current position to outMin through outMax 
    }
    else{
        return -1;
    }
}

bool SAS::error(){
    int errorMargin = 10;
    int potAVal = analogRead(pinA);
    int potBVal = analogRead(pinB);
    int x = abs(potAVal - potBVal);
    if(x > errorMargin){
        return true;
    }
    return false;
}