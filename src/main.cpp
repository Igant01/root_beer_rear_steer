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

#define SASRB 18  //rear steering angle b
#define SASRA 17  //rear steering angle a
#define SASFB 16  //front steering angle b
#define SASFA 15  //front steering angle a
#define CURRA 14  //current sense direction a
#define CURRB 19 //current sense direction b


// Hardware objects
LMT87 tempSensor;
current_sense forwardCurrent(CURRA);
motor_driver driver;
SAS frontSAS(SASFA, SASFB);
SAS rearSAS(SASRA, SASRB);


// Velocity PID
double lastPos;
double veloSetpoint, veloInput, veloOutput; // reference variables 
double veloKp=2, veloKi=5, veloKd=1;
PID veloPID(&veloInput, &veloOutput, &veloSetpoint, veloKp, veloKi, veloKd, DIRECT);

// Position PID
double posSetpoint, posInput, posOutput; // reference variables 
double posKp=2, posKi=5, posKd=1;
PID posPID(&posInput, &posOutput, &posSetpoint, posKp, posKi, posKd, DIRECT);

// Task intervals (milliseconds)
const unsigned long interval1 = 1;   // 1000Hz
const unsigned long interval2 = 2;   // 500Hz

// Time trackers
unsigned long prevTime1 = 0;
unsigned long prevTime2 = 0;

void task1();
void task2();


void setup() {
  Serial.begin(115200);

  posPID.SetSampleTime(interval2);
  veloPID.SetSampleTime(interval1);

  veloPID.SetMode(AUTOMATIC);
  posPID.SetMode(AUTOMATIC);

  lastPos = rearSAS.positionNonRedundant();

}

void loop() {
  unsigned long currentMillis = millis();

  // Task 1
  if (currentMillis - prevTime1 >= interval1) {
    prevTime1 = currentMillis;
    task1();
  }

  // Task 2 
  if (currentMillis - prevTime2 >= interval2) {
    prevTime2 = currentMillis;
    task2();
  }

}

void task1() {  // High speed
  double velocity = (rearSAS.positionNonRedundant() - lastPos)/0.001;
  lastPos = rearSAS.positionNonRedundant();
  Serial.println("Task 1 running");
  veloInput = velocity;
  veloPID.Compute();
  driver.set(veloOutput);
}

void task2() {  //Low speed
  posSetpoint = frontSAS.positionNonRedundant();
  Serial.println("Task 2 running");
  posInput = rearSAS.positionNonRedundant();
  posPID.Compute();
  veloSetpoint = posOutput;
}

