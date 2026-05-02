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
 * Module : Sheepdog Flocking with Yellow-Trail Projector
 * Summary: Extends trail-following with a yellow projector trail (detected by
 * LDR threshold > yellow_trail). Robots that enter the projected
 * trail transition from free ABP motion to coordinated trail-following,
 * replicating sheepdog-style herding dynamics.
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
double Vnot = 6.00;    // Constant velocity in cm/s
double etaMax = 5.00;  // Strenght of rotational noise in rad/s
int dT = 250;          // Delay time in ms
int I_L = 400;         // Intensity between black and white for left LDR
int I_R = 550;         // Intensity between black and white for right LDR

int LDR_R, LDR_L, dI;
//float count=0.00;
int V = 100;
int count = 0;

int yellow_trail = 100;  // ANALOG VALUE FOR THE LIGHT INTENSITY OF PROJECTOR

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
}
void loop() {
  // put your main code here, to run repeatedly:
  int IR_SE = readMux(1);
  int IR_E = readMux(2);
  int IR_NE = readMux(3);
  int IR_N = readMux(4);
  int IR_NW = readMux(5);
  int IR_W = readMux(6);
  int IR_SW = readMux(7);
  int IR_S = readMux(0);

  LDR_R = analogRead(A6);  // Light Intensity Sensor1
  LDR_L = analogRead(A7);  // Light Intensity Sensor2
  dI = LDR_L - LDR_R;
  Serial.println(LDR_R);
  Serial.print("           ");
  Serial.print(LDR_L);
  if (LDR_R > yellow_trail && LDR_L > yellow_trail) {
    LDR_R = analogRead(A6);  // Light Intensity Sensor1
    LDR_L = analogRead(A7);  // Light Intensity Sensor2
    dI = LDR_L - LDR_R;

    if (dI < -170) {  //Rotate towards black
      motor1.drive(-V);
      motor2.drive(+V);
      delay(10);
    }

    else if (dI > -120) {  //Rotate towards black
      motor1.drive(+V);
      motor2.drive(-V);
      delay(10);
    } else if (LDR_L > I_L && LDR_R > I_R) {  //MOVE BACK
      motor1.drive(-V);
      motor2.drive(-V);
    } else {  //ABP motion
      eta = random(-100, 101) / (100.00 / etaMax);
      Vr = ((eta * 7.00) + (2.00 * Vnot)) / 2.00;  // In cm/s
      Vl = (2.00 * Vnot) - Vr;                     // In cm/s

      Vr = (Vr + 1.0015) / 0.1059;    // Analog value
      Vl = ((Vl + 1.0015) / 0.1059);  // Analog value

      motor1.drive(Vr);
      motor2.drive(Vl);
      delay(dT/2);

      int LDR_R = analogRead(A6);  // Light Intensity Sensor1
      int LDR_L = analogRead(A7);  // Light Intensity Sensor2

      if (dI < -170) {  //Rotate towards black
        motor1.drive(-V);
        motor2.drive(+V);
        delay(10);
      }

      else if (dI > -120) {  //Rotate towards black
        motor1.drive(+V);
        motor2.drive(-V);
        delay(10);
      } else if (LDR_L > I_L && LDR_R > I_R) {  //MOVE BACK
        motor1.drive(-V);
        motor2.drive(-V);
        count = count + 1;
        delay(100);

        //Serial.println(count);
      } else {
        motor1.drive(Vr);
        motor2.drive(Vl);
        delay(dT / 2);
      }
    }
  } else {
    //READ DATA FROM THE IR SENSORS
    int IR_SE = readMux(1);
    int IR_E = readMux(2);
    int IR_NE = readMux(3);
    int IR_N = readMux(4);
    int IR_NW = readMux(5);
    int IR_W = readMux(6);
    int IR_SW = readMux(7);
    int IR_S = readMux(0);

    //IF IR DETECT ANYTHING
    if (IR_NW > 500 || IR_NE > 500 || IR_N > 500 || IR_W > 500 || IR_SW > 500 || IR_S > 500 || IR_SE > 500 || IR_E > 500) {
      //TURN OFF LED
      digitalWrite(IR_LED_pin, LOW);
      //READ DATA AGAIN
      int IR_SE = readMux(1);
      int IR_E = readMux(2);
      int IR_NE = readMux(3);
      int IR_N = readMux(4);
      int IR_NW = readMux(5);
      int IR_W = readMux(6);
      int IR_SW = readMux(7);
      int IR_S = readMux(0);
      //IF IR DETECT ANYTHING
      if (IR_NW > 900 || IR_NE > 900 || IR_N > 900 || IR_W > 900 || IR_SW > 900 || IR_S > 900 || IR_SE > 900 || IR_E > 900) {



        // //TURN ON LED
        // digitalWrite(IR_LED_pin,HIGH);
        // //ROBOT NEARBY-SO STOPS
        // brake(motor1,motor2);
        // delay(2000);
        // //RANDOM ROTATION
        // int ran = random(1,3);
        // if(ran == 1){
        //   left(motor1,motor2,200);
        //   delay(300);
        // }
        // else{
        //   right(motor1,motor2,200);
        //   delay(300);
        // }
        // //BLIND RUN
        // forward(motor1,motor2,80);
        // delay(500);
      } else {
        //TURN ON LED
        digitalWrite(IR_LED_pin, HIGH);
        //READ DATA AGAIN
        int IR_SE = readMux(1);
        int IR_S = readMux(0);
        int IR_SW = readMux(7);
        //AVOID BOUNDARY
        if (IR_SW > 500) {
          left(motor1, motor2, 200);
          delay(400);
          //BLIND RUN
          forward(motor1, motor2, 80);
          delay(500);
        }

        else if (IR_SE > 500) {
          right(motor1, motor2, 200);
          delay(400);
          //BLIND RUN
          forward(motor1, motor2, 80);
          delay(500);
        }

        else if (IR_S > 500) {
          right(motor1, motor2, 200);
          delay(900);
          //BLIND RUN
          forward(motor1, motor2, 80);
          delay(500);
        }
      }

    }
    //ELSE ABP
    else {
      eta = random(-100, 101) / (100.00 / etaMax);
      Vr = ((eta * 7.00) + (2.00 * Vnot)) / 2.00;  // In cm/s
      Vl = (2.00 * Vnot) - Vr;                     // In cm/s

      Vr = (Vr + 1.0015) / 0.1059;    // Analog value
      Vl = ((Vl + 1.0015) / 0.1059);  // Analog value

      motor1.drive(Vr);
      motor2.drive(Vl);
      delay(dT);
    }
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
