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

#define SASRB 18  //rear steering angle b
#define SASRA 17  //rear steering angle a
#define SASFB 16  //front steering angle b
#define SASFA 15  //front steering angle a
#define CURRA 14  //current sense direction a
//#define CURRB 19 //current sense direction b

LMT87 tempSensor;
current_sense forwardCurrent(CURRA);
//current_sense(CURRB) reverseCurrent;

SAS frontSAS(SASFA,SASFB);
SAS rearSAS(SASRA,SASRB);


void setup() {
  Serial.begin(115200);


  
}

void loop() {

  delay(100);
  Serial.println(tempSensor.read());

  
}

