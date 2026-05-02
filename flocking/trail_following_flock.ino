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
 * Module : Trail-Following Flocking with LDR Sensors
 * Summary: Robots follow a light trail projected onto the arena floor by an
 * overhead projector + camera system. LDR differential intensity
 * steers the robot onto the trail; IR sensors handle boundary
 * avoidance when the robot exits the trail region.
 * Trailing induces emergent flocking without explicit communication.
 */

#include <SparkFun_TB6612.h>

//Defining pins connected to the motor driver
#define PWMA 3
#define AIN2 4
#define AIN1 5
#define STBY 1
#define BIN1 7
#define BIN2 8
#define PWMB 9

// these constants are used to allow you to make your motor configuration
// line up with function names like forward. Value can be 1 or -1
const int offsetA = -1;
const int offsetB = -1;

// Initializing motors.
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetA, STBY);

//Mux control pins
int s0 = A5;
int s1 = A2;
int s2 = A1;

//Mux in "Z" pin
int Z_pin = A4;

int IR_LED_pin = 10;

// Global variables
double Vr, Vl, eta;

// Input parameters
double Vnot = 6.00;      // Constant ABP velocity in cm/s
double etaMax = 1.00;  // Strenght of rotational noise in rad/s
int dT = 50;          // Delay time in ms
int I_L = 410;         // Intensity I_L = (Iwhite_L + Itrail_L)/2
int I_R = 580;         // Intensity I_R = (Iwhite_R + Itrail_R)/2

/*
Robot No    I_L   I_R
  5         480   530
  6         500   610
  7         465   555
  10        410   580
*/

// Other parameters
int LDR_R, LDR_L, IR_F, IR_FL, IR_FR;
int V = 110;  // Boundary rotation speed 180 deg/s

void setup() {
  // put your setup code here, to run once:
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(IR_LED_pin, OUTPUT);

  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);

  pinMode(A7, INPUT);
  pinMode(A6, INPUT);

  digitalWrite(IR_LED_pin, HIGH);

  Serial.begin(9600);

  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH); //MAKES LDR ON
}


void loop() {
  ///////// READ SENSOR DATA /////////////////////
  LDR_R = analogRead(A6);
  LDR_L = analogRead(A7);
  IR_FL = readMux(1);
  IR_FR = readMux(7);
  IR_F = readMux(0);
  //      brake(motor1,motor2);
  if (LDR_L > I_L && LDR_R < I_R) { // Rotate right
    motor1.drive(V);
    motor2.drive(-V);
    delay(50);
  }
  else if (LDR_L < I_L && LDR_R > I_R) { // Rotate left
    motor1.drive(-V);
    motor2.drive(+V);
    delay(50);
  }
    //IF IR DETECT ANYTHING
  else if(IR_FL > 500 || IR_FR > 500 || IR_F > 500){
    //TURN OFF LED
    digitalWrite(IR_LED_pin,LOW);
    //READ DATA AGAIN
    IR_FL = readMux(1);
    IR_FR = readMux(7);
    IR_F = readMux(0);
    //IF IR DETECT ANYTHING
    if(IR_FL > 800 || IR_FR > 800 || IR_F > 800){
      //TURN ON LED
      digitalWrite(IR_LED_pin,HIGH);
    }
    else{
      //TURN ON LED
      digitalWrite(IR_LED_pin,HIGH);
      //READ DATA AGAIN
      IR_FL = readMux(1);
      IR_FR = readMux(7);
      IR_F = readMux(0);
      //AVOID BOUNDARY
      if (IR_F > 250 && IR_FR > 250) {// Rotate left
      motor1.drive(-V);
      motor2.drive(+V);
      delay(375);
      }
      else if (IR_F > 250 && IR_FL > 250) {// Rotate right
      motor1.drive(+V);
      motor2.drive(-V);
      delay(375);
      }
      else if (IR_F > 500) {// Rotate right
      motor1.drive(V);
      motor2.drive(-V);
      delay(500);
      }
      else if (IR_FR > 500) {// Rotate left
        motor1.drive(-V);
        motor2.drive(+V);
        delay(250);
      }
      else if (IR_FL > 500) {// Rotate right
        motor1.drive(V);
        motor2.drive(-V);
        delay(250);
      }
    }
  }
  else { // ABP
    eta = random(-100, 101) / (100.00 / etaMax);
    Vr = ((eta * 7.50) + (2.00 * Vnot)) / 2.00;  // In cm/s
    Vl = (2.00 * Vnot) - Vr;                     // In cm/s

    Vr = (Vr + 1.0015) / 0.1059;    // Analog value
    Vl = ((Vl + 1.0015) / 0.1059);  // Analog value

    motor1.drive(Vr);
    motor2.drive(Vl);
    delay(dT);
  }
}

//FUNCTION TO READ DATA FROM IR SENSORS
float readMux(int channel) {
  int controlPin[] = { s0, s1, s2 };

  int muxChannel[8][3] = {
    { 0, 0, 0 },  //channel 0
    { 1, 0, 0 },  //channel 1
    { 0, 1, 0 },  //channel 2
    { 1, 1, 0 },  //channel 3
    { 0, 0, 1 },  //channel 4
    { 1, 0, 1 },  //channel 5
    { 0, 1, 1 },  //channel 6
    { 1, 1, 1 },  //channel 7
  };

  //loop through the 3 sig
  for (int i = 0; i < 3; i++) {
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }

  //read the value at the Z pin
  int val = analogRead(Z_pin);
  return val;
}
