#include "DualMC33926MotorShield.h"
#include "Encoder.h"

#include "Wire.h"

#define sampleTime 10
#define ADDR 0x8

DualMC33926MotorShield ms; 
Encoder encoder(2,5);

//Variables for holding received data
byte data[32] = {0};
byte value = 0;

//const double Ki = 0.156633991202905;
double Ki = .07, Kp = 15.8; //Not the same from matlab
//const double Kp = 3.18865312201695;
double setpoint = 0, I = 0, e = 0, Tc = 0, Ts = 0, u =0;
double curEnc = 0, prevEnc = 0;
double const umax = 7;

void setup() {
  ms.init(); 
  Serial.begin(115200);

  Wire.begin(ADDR);
  delay(1000);
  Serial.print("------------ I am Arduino Uno I2C at address 0x");
  Serial.print(ADDR);
  Serial.println(" ------------");
  delay(1000);

  Wire.onRequest(requestData1);
  Wire.onReceive(receiveData1);
  
  pinMode(4,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  digitalWrite(7,HIGH);
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
}

void requestData1() {
  Serial.println("---> Recieved Requests");
  Serial.println("Sending Back New Position");
  Serial.print("Sending value : ");
  Serial.println(value);
  Wire.write(value);
}

void receiveData1(int numBytes) {
  if(numBytes > 1) {
    int i = 0;
    while (Wire.available()) {
      data[1] = Wire.read();
      i++;
    }
  
    Serial.println(F("---> Recieved Events"));
    Serial.print(F("Recieved value : "));
    Serial.println(data[1]);
  
    value = int(data[1]);

    switch(value){
      case 1:
        setpoint = 0;
        Serial.println(setpoint);
        break;
      case 2:
        setpoint = 1.57;
        Serial.println(setpoint);
        break;
      case 3:
        setpoint = 3.14;
        Serial.println(setpoint);
        break;
      case 4:
        setpoint = 4.71;
        Serial.println(setpoint);
        break;
    } 
  }  
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
