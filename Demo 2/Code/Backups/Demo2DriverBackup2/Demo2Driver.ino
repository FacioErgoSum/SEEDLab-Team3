// SEED Lab - Team 3
// Demo 2 Driving Script
// Fall 2022

// Goal: Drive to a target position based off of a polar coordanite.
// Achieve low error using two PI controllers, one for 
// positional error and another for angular error. Switch
// between two turn modes to only turn or to drive straight
// with some angular correction.

#include "DualMC33926MotorShield.h"
#include "Encoder.h"
#include "Wire.h"

//Set I2C Address
#define ADDR 0x8

//Speed of our loop
#define sampleTime 10

//Set Debug Bool
#define DEBUG false

//Set Mode
#define Mode 2
//  1 - Percise
//  2 - Fast

#if Mode == 1  //Precision Mode
  //Position PID Values
  #define pos_Kp          0.13
  #define pos_Ki          0.0000013
  //Angular PID Values
  #define ang_Kp          0.8
  #define ang_Ki          0.000014
  //Clamp Values
  #define umax_pos        0.35
  #define umax_ang        0.3
  //Turn rate
  #define turnMode        0.1
  //Maximum angular error used in our Hysteresis
  #define angleMaxError 0.4
  #define angleMaxErrorI 0.02
#elif Mode == 2 //Fast Mode
  //Position PID Values
  #define pos_Kp          0.83
  #define pos_Ki          0.0000013
  //Angular PID Values
  #define ang_Kp          2.5
  #define ang_Ki          0.0000014
  //Clamp Values
  #define umax_pos        0.65
  #define umax_ang        0.5
  //Turn rate
  #define turnMode        0.25
  //Maximum angular error used in our Hysteresis
  #define angleMaxError 0.20
  #define angleMaxErrorI 0.1
#endif

//Robot Constants
#define pi                3.1415
#define wheelDiameter     5.9495 //Inches
#define totalEncoderTicks 3200
#define wheelBase         12.3

DualMC33926MotorShield ms; 
Encoder rightMotor(2,5);
Encoder leftMotor(3,6);

//I2C Data in Bytes
byte data[32] = {0};

//double pos_Setpoint = pos_Int_Setpoint * 1.007; //Fudges our target distance to get better results
//Variables to hold all of localization
double Tc = 0, Ts = 0, e = 0;
double currentRightMotorEncoder = 0, prevRightMotorEncoder = 0, differenceEncoderRight = 0, rotationDifferenceRight = 0;
double currentLeftMotorEncoder = 0, prevLeftMotorEncoder = 0, differenceEncoderLeft = 0, rotationDifferenceLeft = 0;
double directionalVelocity = 0, angularVelocity = 0;
double robotX = 0.000001, robotY = 0.000001, robotAngle = 0.000001;
float targetPointX = 0, targetPointY =0;
double positionalError = 10, angularError = 0;
bool CloseToMarker = false;

double sysEnc = 0, pos_I, ang_I;
double sysAngVel = 0, sysPos = 0, sysPosVel = 0, sysAng = 0;
double pos_u = 0, ang_u = 0;

int mode = 0;
//Mode 0, Stop
//Mode 1, Search
//Mode 2, Drive to point

bool systemMode = false; //true is forwad, false is turn

void setup() {
  
  ms.init();
  
  pinMode(4,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  
  digitalWrite(7,HIGH);
  digitalWrite(8,HIGH);
  
  Tc = millis();
  
  Serial.begin(115200);
  Serial.println("Begin");

  Wire.begin(ADDR);
  delay(1000);
  Serial.print("------------ I am Arduino Uno I2C at address 0x");
  Serial.print(ADDR);
  Serial.println(" ------------");
  delay(1000);

  Wire.onRequest(requestData);
  Wire.onReceive(receiveData);
}

void loop() {
  //Defines if we have reached the point or not
  if ((mode != 2) || !CloseToMarker) { //(positionalError > (pos_Setpoint/29))||((positionalError > 0.1)) (mode != 2)
    
    //Creates a loop that runs only during our sample time
    if(millis() >= Tc + sampleTime){
      
      //Read encoder values
      prevRightMotorEncoder = currentRightMotorEncoder;
      currentRightMotorEncoder = rightMotor.read();
      prevLeftMotorEncoder = currentLeftMotorEncoder;
      currentLeftMotorEncoder = leftMotor.read();
      
      //Calculate difference in ticks
      differenceEncoderLeft = -1*(currentLeftMotorEncoder - prevLeftMotorEncoder);
      differenceEncoderRight = -1*(currentRightMotorEncoder - prevRightMotorEncoder);
      
      //Get rotation of wheel
      rotationDifferenceLeft = (pi * wheelDiameter * differenceEncoderLeft) / totalEncoderTicks;
      rotationDifferenceRight = (pi * wheelDiameter * differenceEncoderRight) / totalEncoderTicks;
      
      //Calculate Angular Velocity
      angularVelocity = (rotationDifferenceRight - rotationDifferenceLeft) / wheelBase;
      
      //Calculate Directional velocity
      directionalVelocity = (rotationDifferenceRight + rotationDifferenceLeft) * 0.5;
      
      //Get new positions
      robotX += directionalVelocity * cos(robotAngle);
      robotY += directionalVelocity * sin(robotAngle);
      robotAngle += angularVelocity;
      
      //CorrectAngle
      if (robotAngle > (pi)) {
        robotAngle -= 2*pi;
      }
      if (robotAngle < (-1*pi)) {
        robotAngle += 2*pi;
      }



//------------------------ Finite State Machine ------------------------//


      switch(mode) {
        case 0: // Stop Mode
          positionalError = 0;
          angularError = 0;
          break;
		    case 1: // Search Mode (Turning)
          positionalError = 0;
          angularError = turnMode;
          pos_I = 0.0;
          ang_I = 0.0;
          CloseToMarker = false;
          break;
        case 2: // Drive to Point
          //Calculate Postion Error
          positionalError = sqrt(sq(targetPointX - robotX)+sq(targetPointY - robotY));
          //Calculate Angular Error
          angularError = atan((targetPointY - robotY)/(targetPointX - robotX)) - robotAngle;
          //Check distance away
          if (positionalError < 1) {
            CloseToMarker = true;
          }
          //Correct Tangent
          if ((targetPointX - robotX)<0) {
            angularError += pi;
          }
  
          //Correct Anglular error
          if (angularError > (pi)) {
            angularError -= 2*pi;
          }
          if (angularError < (-1*pi)) {
            angularError += 2*pi;
          }
          break;
        default:
          Serial.println("Finite State Machine Reached Invalid State");
          break;
      }
      
      //Debug print
      if (DEBUG) {
        Serial.print(positionalError);
        Serial.print(" - ");
        Serial.print(angularError);
        Serial.print(" - ");
        Serial.print(mode);
        Serial.print(" - ");
        Serial.print(systemMode);
        Serial.print(" - ");
        Serial.print(targetPointX);
        Serial.print(" - ");
        Serial.println(targetPointY);
      }

      //Switch stament to go between just turning and riving straight
      //SystemMode is set using Hysteresis defined near end of script
      if(systemMode){ //low angle difference
        pos_I += Ts*positionalError;
        pos_u = pos_Kp*positionalError; //pos_u = pos_Kp*positionalError+pos_I*pos_Ki;
        ang_I += Ts*angularError;
        ang_u = ang_Kp*angularError; //ang_u = ang_Kp*angularError+ang_I*ang_Ki;
  
        if(abs(pos_u) > umax_pos){ //low ang error
          pos_u = sgn(pos_u)*umax_pos;
          pos_I = (pos_u-pos_Kp*positionalError)/pos_Ki;
        }
  
        if(abs(ang_u) > umax_ang){
          ang_u = sgn(ang_u)*umax_ang;
          ang_I = (ang_I-ang_Ki*angularError)/ang_Ki;
        }
  
        if(pos_u > 0){
          //analogWrite(9,0);
          //analogWrite(10,0);
          digitalWrite(7,LOW);
          digitalWrite(8,LOW);
          //Serial.print("Drive Forward at ");
        }
  
        else {
          //analogWrite(9,0);
          //analogWrite(10,0);
          digitalWrite(7,HIGH);
          digitalWrite(8,HIGH);
          //Serial.print("Drive Backward at ");
        }
        if(true){
         analogWrite(9,abs(pos_u+(ang_u*1.5))*255); //12.34
         analogWrite(10, abs(pos_u-(ang_u*1.5))*255); 
         //Serial.println(pos_u);
        } 
        
    }
    else { //high angle difference, turn mode
        //ang_u = ang_Kp*angularError;
        ang_I += Ts*angularError;
        ang_u = ang_Kp*angularError+ang_I*ang_Ki;
        if(abs(ang_u) > umax_ang){
          ang_u = sgn(ang_u)*umax_ang;
          ang_I = (ang_I-ang_Ki*angularError)/ang_Ki;
        }
        //Perform different motor commands depending on if error is positive or negative
        //Serial.println(angularError);
        if (angularError < 0){ 
         analogWrite(9,(abs(ang_u)*255)); //12.34
         analogWrite(10,(abs(ang_u)*255)); 
         digitalWrite(7,HIGH);
         digitalWrite(8,LOW);
         //Serial.println("angerror1");
        }
        else {
         analogWrite(9,(abs(ang_u)*255)); //12.34
         analogWrite(10,(abs(ang_u)*255)); 
         digitalWrite(7,LOW);
         digitalWrite(8,HIGH); 
         //Serial.println("angerror-1");
        }

    }
      Ts = millis()-Tc;
      Tc = millis();
      //Serial.print(positionalError);
        //Serial.print(" - ");
        //Serial.println(angularError);
  }
  //Serial.println(angularError);
  } else {
      analogWrite(9,(0)); //12.34
      analogWrite(10,(0)); 
  }

//Hysteresis, 
//Exit turn mode once angular error is smaller than angleMaxErrorI,
//Go back into turn mode once angular error is greater than angleMaxError
  if (systemMode) {
    if ((angularError > angleMaxError) || (angularError < (-1* angleMaxError))) {
      systemMode = false;
    }
  }
  else {
    if ((angularError < angleMaxErrorI) && (angularError > (-1* angleMaxErrorI))) {
      systemMode = true;
      analogWrite(9,0);
      analogWrite(10,0);
      delay(100);
      ang_I = 0;
      ang_u = 0;
    }
  }
  
}

//--------------------------- I2C Functions ---------------------------//

void requestData() {
  //https://arduino.stackexchange.com/questions/84539/convert-float-to-byte-from-arduino-to-raspberry-pi-i2c
  //Serial.println("---> Recieved Requests");
  //Serial.println("Sending Back New Position");
  Wire.write((uint8_t *) &robotX, sizeof robotX);
  Wire.write((uint8_t *) &robotY, sizeof robotY);
  Wire.write((uint8_t *) &robotAngle, sizeof robotAngle);
}

void receiveData(int numBytes) {
  int i = 0;
  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }
  
  //Serial.println(F("---> Recieved Data"));
  //Serial.println(F("Recieved value : "));
  parseValues(data);
  //https://forums.raspberrypi.com/viewtopic.php?t=191208  
}

void parseValues(byte data[]){
  union float_tag{
    byte b[4];
    float fval;
  }targetX;

  targetX.b[0] = data[1];
  targetX.b[1] = data[2];
  targetX.b[2] = data[3];
  targetX.b[3] = data[4];

  union float_tag_2{
    byte b[4];
    float fval;
  }targetY;

  targetY.b[0] = data[5];
  targetY.b[1] = data[6];
  targetY.b[2] = data[7];
  targetY.b[3] = data[8];

  mode = data[9];
  targetPointX = targetX.fval;
  targetPointY = targetY.fval;

  //Serial.println(targetX.fval);
  //Serial.println(targetY.fval);
  //Serial.println(data[9]);
}

void printBinary(byte b) {
  for (int i = 7; i >= 0; i-- )
  {
    Serial.print((b >> i) & 0X01);//shift and select first bit
  }
  Serial.println();
}

//Function to return ths sign of a value
int sgn(float num){
  int sign = 1;
  if(num >= 0){
    sign = 1;
  } else {
    sign = -1;
  }
  return sign;
}
