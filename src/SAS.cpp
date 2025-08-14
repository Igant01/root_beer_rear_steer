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

SAS::SAS(int PINA,int PINB){ //A top B bottom this is arbitrary
    topMin = 0;
    topMax = 0;
    bottomMin = 0;
    bottomMax = 0;
    calibratedLeft = false;
    calibratedRight = false;
    pinA = PINA;
    pinB = PINB;
    pinMode(pinA,INPUT);
    pinMode(pinB,INPUT);
}

void SAS::setLeft(){
    topMin = analogRead(pinA);
    bottomMin = analogRead(pinB);
    calibratedLeft = true;
}

void SAS::setRight(){
    topMax = analogRead(pinA);
    bottomMax = analogRead(pinB);
    calibratedRight = true;
}

bool SAS::error(){
    int errorMargin = 10;
    int x = 0;
    x = abs(topMin-bottomMin);
    if(x > errorMargin){
        return true;
    }
    x = abs(topMax-bottomMax);
    if(x > errorMargin){
        return true;
    }
    return false;
}

int SAS::positionNonRedundant(){
    int outMin = 0; //output minimum value
    int outMax = 1023;  //output maximum value
    if(calibratedLeft&&calibratedRight&&(!error())){
        int pinAvg = (analogRead(pinA)+analogRead(pinB))/2; //average of current position
        int leftAvg = (topMin+bottomMin)/2; //average of left/minimum extremes
        int rightAvg = (topMax+bottomMax)/2; //average of right/maximum extremes
        return map(pinAvg,leftAvg,rightAvg,outMin,outMax); //map current position to outMin through outMax 
    }
    else{
        return -1;
    }
}