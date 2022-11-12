#include "DualMC33926MotorShield.h"
#include "Encoder.h"

#define sampleTime 10

DualMC33926MotorShield motor;
Encoder encoder(2,5);

double dTheta = 0, dT = 0, dW = 0, prevTime = 0, currTime = 0, angVel = 0;
double currEnc = 0, preEnc = 0;

int CR = 13, LF = 10; //Define the Caridge Return and the Line Feed
double Tc = 0, Ts = 0;
void setup() {
  motor.init();
  Serial.begin(115200);

  //Define the following pinouts for the motor to run correctly
  pinMode(4,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);

  //Define the direction of the motor
  digitalWrite(7, HIGH);
  while(millis() <= 250){
    Serial.print(0);
    Serial.write(13);
    Serial.write(10);
  }
  analogWrite(9,144);
  Tc = millis();
}

void loop() {
  if(millis() >= Tc+sampleTime){
    preEnc = currEnc;
    currEnc = encoder.read();
    angVel = (currEnc - preEnc)*(6.28/3200)*1000/(Tc-Ts);
    Ts = Tc;
    Tc = millis();
  }
  Serial.print(angVel);
  Serial.write(CR);
  Serial.write(LF);
}
