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
#include <PID_AutoTune_v0.h>

#define SASRB 18  //rear steering angle b
#define SASRA 17  //rear steering angle a
#define SASFB 16  //front steering angle b
#define SASFA 15  //front steering angle a
#define CURRA 14  //current sense direction a
//#define CURRB 19 //current sense direction b

byte ATuneModeRemember=2;
double input=80, output=50, setpoint=180;
double kp=2,ki=0.5,kd=2;

double kpmodel=1.5, taup=100, theta[50];
double outputStart=5;
double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;
unsigned int aTuneLookBack=20;

boolean tuning = false;
unsigned long  modelTime, serialTime;

PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);
PID_ATune aTune(&input, &output);

//set to false to connect to the real world
boolean useSimulation = true;

LMT87 tempSensor;
current_sense forwardCurrent(CURRA);
//current_sense(CURRB) reverseCurrent;

SAS frontSAS(SASFA,SASFB);
SAS rearSAS(SASRA,SASRB);

void changeAutoTune();
void AutoTuneHelper(boolean start);
void SerialSend();
void SerialReceive();
void DoModel();


void setup(){
  if(useSimulation){
    for(byte i=0;i<50;i++){
      theta[i]=outputStart;
    }
    modelTime = 0;
  }
  //Setup the pid 
  myPID.SetMode(AUTOMATIC);

  if(tuning){
    tuning=false;
    changeAutoTune();
    tuning=true;
  }
  
  serialTime = 0;
  Serial.begin(115200);

}

void loop(){

  Serial.println(tempSensor.read());

  unsigned long now = millis();

  if(!useSimulation){ //pull the input in from the real world
    input = analogRead(0);
  }
  
  if(tuning){
    byte val = (aTune.Runtime());
    if (val!=0){
      tuning = false;
    }
    if(!tuning){ //we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      myPID.SetTunings(kp,ki,kd);
      AutoTuneHelper(false);
    }
  }
  else myPID.Compute();
  
  if(useSimulation){
    theta[30]=output;
    if(now>=modelTime){
      modelTime +=100; 
      DoModel();
    }
  }
  else{
     analogWrite(0,output); 
  }
  
  //send-receive with processing if it's time
  if(millis()>serialTime){
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }
}

void changeAutoTune(){
 if(!tuning){
    //Set the output to the desired starting frequency.
    output=aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else{ //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start){
  if(start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}


void SerialSend(){
  Serial.print("setpoint: ");Serial.print(setpoint); Serial.print(" ");
  Serial.print("input: ");Serial.print(input); Serial.print(" ");
  Serial.print("output: ");Serial.print(output); Serial.print(" ");
  if(tuning){
    Serial.println("tuning mode");
  } else {
    Serial.print("kp: ");Serial.print(myPID.GetKp());Serial.print(" ");
    Serial.print("ki: ");Serial.print(myPID.GetKi());Serial.print(" ");
    Serial.print("kd: ");Serial.print(myPID.GetKd());Serial.println();
  }
}

void SerialReceive(){
  if(Serial.available()){
   char b = Serial.read(); 
   Serial.flush(); 
   if((b=='1' && !tuning) || (b!='1' && tuning))changeAutoTune();
  }
}

void DoModel(){
  //cycle the dead time
  for(byte i=0;i<49;i++){
    theta[i] = theta[i+1];
  }
  //compute the input
  input = (kpmodel / taup) *(theta[0]-outputStart) + input*(1-1/taup) + ((float)random(-10,10))/100;

}
