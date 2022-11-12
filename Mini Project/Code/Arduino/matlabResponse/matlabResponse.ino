#include "DualMC33926MotorShield.h"
#include "Encoder.h"

#include "Wire.h"

#define sampleTime 10
#define ADDR 0x8

DualMC33926MotorShield ms; 
Encoder encoder(2,5);

//const double Ki = 0.156633991202905;
double Ki = .07, Kp = 15.8; //Not the same from matlab
//const double Kp = 3.18865312201695;
double setpoint = 0, I = 0, e = 0, Tc = 0, Ts = 0, u =0;
double curEnc = 0;
double const umax = 7;

void setup() {
  ms.init(); 
  Serial.begin(115200);
  pinMode(4,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  digitalWrite(7,HIGH);
  while(millis() <= 500){
    Serial.print(encoder.read());
    Serial.write(13);
    Serial.write(11);
  }
  setpoint = 6.28;
}

void loop() {
  if(millis() >= Tc + sampleTime){
    //Read the set point here
    curEnc = encoder.read();
    e = setpoint-curEnc/3200*6.28;
      I += Ts*e;
      u = Kp*e+Ki*I;

      if(abs(u) > umax){
        u = sgn(u)*umax;
        I = (u-Kp*e)/Ki;
      }
      if( u < 0){
        analogWrite(9,0);
        digitalWrite(7,LOW);
      }
      if( u >= 0){
        analogWrite(9,0);
        digitalWrite(7,HIGH);
      }
      //analogWrite(9, u*12.34);
      analogWrite(9,abs(u)*255/12.34);
      Ts = millis()-Tc;
      Tc = millis();
    }
    Serial.print(curEnc*6.8/3200);
    Serial.write(13);
    Serial.write(10);
}

int sgn(float num){
  int sign = 1;
  if(num >= 0){
    sign = 1;
  } else {
    sign = -1;
  }
  return sign;
}
