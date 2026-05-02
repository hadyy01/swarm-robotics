/**
 * Swarm Robotics Research — Active Matter Lab, IIT Bombay
 * Author  : Hady Khan
 * Affil.  : Active Matter Lab, Department of Physics, IIT Bombay
 * Advisor : Prof. Dr. Nitin Kumar, Department of Physics, IIT Bombay
 *
 * Hardware:
 *   - Custom differential-drive robot (75 mm diameter)
 *   - TB6612FNG dual motor driver
 *   - 8× TCRT5000 IR proximity sensors (via CD4051 8:1 MUX)
 *   - nRF24L01+ 2.4 GHz wireless transceiver
 *   - 2× LDR light sensors (for projector-trail detection)
 *
 * Libraries required:
 *   - SparkFun TB6612 Motor Driver  (v1.0.1)
 *   - RF24                          (v1.4.x)
 *   - RF24Network                   (v1.0.x)
 */
 *
 * Module : Run-and-Tumble Motion
 * Summary: Implements run-and-tumble dynamics inspired by bacterial locomotion.
 * The robot alternates between straight 'run' phases (constant velocity)
 * and 'tumble' phases (random angle reorientation drawn from U[0, 2pi]).
 * Boundary reflection is handled via front-facing IR sensors.
 */

//INCLUDE LIBRARY
#include <SparkFun_TB6612.h>


//DEFINE PARAMETERS
//RUN
unsigned int runVelocity = 100; 
unsigned int runTime  = 1600; //1.1s
//TUMBLE
unsigned int tumbleVelocity = 255.00;
float fullRotationTime = 825.00; //New FULL rotation timee measured for the white cap robot
//float restTime = 400;
//float tumbleTime = 400.00;
       
//OTHER VARIABLES
int runDelay;


//MOTOR PINS
#define PWMA 3
#define AIN2 4
#define AIN1 5
#define STBY 1
#define BIN1 7
#define BIN2 8
#define PWMB 9

//MOTOR OFFSET
const int offsetA = 1;
const int offsetB = 1;

//DEFINING MOTORS
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

//Mux control pins
int s0 = A5;
int s1 = A2;
int s2 = A1;

//Mux in "Z" pin
int Z_pin = A4;

int IR_LED_pin = 10;

void setup() {
  Serial.begin(9600);
  //Define random seed
  randomSeed(analogRead(A0));

  pinMode(s0, OUTPUT); 
  pinMode(s1, OUTPUT); 
  pinMode(s2, OUTPUT);
  pinMode(IR_LED_pin,OUTPUT);
  
  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(IR_LED_pin,HIGH);

  delay(5000);
}

void loop() {

/////////////////////RUN//////////////////////////////////
    //run
    for (int i = 1; i < runTime; i++){
        int IR_N = readMux(4);
        int IR_NW = readMux(5);
        int IR_NE = readMux(3);
//        Serial.println(IR_NE);
        if (IR_N > 150 || IR_NW > 550 || IR_NE > 550){// Close to the boundary - stop do not hit the boundary //
          brake(motor1,motor2);
//          delay(runTime - i);
          break;
        }
        else{
          motor1.drive(runVelocity);
          motor2.drive(runVelocity + 1);
          delay(1);
        }
    }  
  ///////////////////TUMBLE/////////////////////////////////
     //DECIDING TUMBLETIME/ANGLE
        int tumbleDelay = generateTumbleTime();
//   Serial.println(tumbleDelay);
    //TUMBLE
    if (tumbleDelay > 0){
      motor1.drive(tumbleVelocity);
      motor2.drive(-tumbleVelocity);
      delay(tumbleDelay);
    }
    else{
      motor1.drive(-tumbleVelocity);
      motor2.drive(tumbleVelocity);
      delay(abs(tumbleDelay));
    }
        
}

//int generateRunTime()  //Generate runTime according its propability in the mention distribution (higher propability means letter is generated more often)
//{
//    int pointer;
//    int totProbabilities = 0;
//    int sum = 0;
//    int index = 0;
//
//    for  (int i = 0; i < runTimeNumber; i++) totProbabilities = totProbabilities + runTimeDist[i]; // tot = sum of all probabilities
//
//    pointer = random(0, totProbabilities);
//
//    while (sum <= pointer)    {
//        sum = sum + runTimeDist[index];
//        index++;
//    }
//    return runTime[index -1];
//}

int generateTumbleTime () //Generate tumble time with equal probality of every angle 0 - 360
{
  int angle = random(0, 360);
  
  if (angle > 180){
    angle = angle - 360;
  }
//  Serial.println(angle);
  float ratio = fullRotationTime/360.00;
  int tumbleTime = angle*ratio;
//  Serial.print(angle);
//  Serial.print('\t');
//  Serial.println(tumbleTime);
  return tumbleTime;
}

//int generateTumbleSpeed () //Generate tumble speed for constant tumble time for different tumble angle acording to the dist
//{
//  int tumbleAngle = random(0, 360);
//  if (tumbleAngle > 180){
//    tumbleAngle = tumbleAngle - 360;
//  }
//  //Find velocity in deg/sec
//  float degsec = (tumbleAngle * 1000.00)/tumbleTime;
//
//  //Convert velocity from deg/sec to analog value
//  float analog = (degsec + 15.64)/1.97;
//  
//  return analog;
//}

float readMux(int channel){
  int controlPin[] = {s0, s1, s2};

  int muxChannel[8][3]={
    {0,0,0}, //channel 0
    {1,0,0}, //channel 1
    {0,1,0}, //channel 2
    {1,1,0}, //channel 3
    {0,0,1}, //channel 4
    {1,0,1}, //channel 5
    {0,1,1}, //channel 6
    {1,1,1}, //channel 7
  };

  //loop through the 3 sig
  for(int i = 0; i < 3; i ++){
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }

  //read the value at the Z pin
  int val = analogRead(Z_pin);

//  if(val>200){
//    val = 1;
//  }
//  else{
//    val = 0;
//  }
  //return the value
  return val;
}
