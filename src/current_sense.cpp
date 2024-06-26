/**
 * @file current_sense.cpp
 * @author Ian Gant (Igant01)
 * @brief 
 * @version 0.1
 * @date 2024-06-18
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <current_sense.h>
#include <Arduino.h>

current_sense::current_sense(int definePin){
    pin = definePin;
    pinMode(pin,INPUT);
}

float current_sense::read(){
    double a = analogRead(pin);
    a = a/1023;
    a = a*3.3;
    a = a/52.36; //amp multiplier
    a = a*1000;
    return a;
}