/**
 * @file motor_driver.cpp
 * @author Ian Gant (Igant01)
 * @brief 
 * @version 0.1
 * @date 2024-06-18
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <motor_driver.h>
#include <Arduino.h>

motor_driver::motor_driver(){
  pinMode(DirA,OUTPUT);
  pinMode(DirB,OUTPUT);
  pinMode(EN,OUTPUT);

  digitalWrite(EN,HIGH); //disable
}

void motor_driver::enable(){
  digitalWrite(EN,LOW);
  ENABLE = true;
}

void motor_driver::disable(){
  digitalWrite(EN,HIGH);
  ENABLE = false;
}

void motor_driver::set(int input){
  int inMin = 0;
  int inMax = 1023;
  int mid = inMax/2;
  if(input<inMin||input>inMax){
    return;
  }
    if(ENABLE){
      if(input<=mid){
        int x = map(input,mid,inMin,0,255);
        analogWrite(DirB, 0);      // ensure opposite leg is off
        analogWrite(DirA,x);
      }
      else{
        int x = map(input,mid,inMax,0,255);
        analogWrite(DirA, 0);      // ensure opposite leg is off
        analogWrite(DirB,x);
      }
    }
}

