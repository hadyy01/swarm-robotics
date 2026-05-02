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
 * Module : NRF24L01 — Transmitter (Multi-pipe)
 * Summary: Transmitter that writes eta to two pipes (address1, address2) in a
 * single radio cycle, enabling synchronised motion of two receivers.
 */

//const int offsetA = 1;
//const int offsetB = 1;
//
//double Vr, Vl, eta;
//double Vnot = 10.00;     // Constant velocity in cm/s
//double etaMax = 5.00;    // Strenght of rotational noise in rad/s
//int    dT = 250;        // Delay time in ms
//
//RF24 radio(6, A0);
//RF24 radio2(6, A0);
//
//const byte address1[] = "nodel1";
//const byte address2[] = "nodel2";
//
//Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
//Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);
//
//void setup()
//{
//  Serial.begin(9600);
//  radio.begin();
//  radio.openWritingPipe(address1);
//  radio.stopListening();
//  radio2.openWritingPipe(address2);
//  radio2.stopListening();
//}
//
//
//void loop() {
//  eta = random(-100, 101) / (100.00 / etaMax);
//  Serial.println(eta);
//  radio.write(&eta, sizeof(eta));
//  radio2.write(&eta, sizeof(eta));
//  Vr = ((eta * 7.00) + (2.00 * Vnot)) / 2.00; // In cm/s
//  Vl = (2.00 * Vnot) - Vr;              // In cm/s
//  Vr = (Vr + 0.729) / 0.1114;           // Analog value
//  Vl = ((Vl + 0.729) / 0.1114);         // Analog value
//  motor1.drive(Vr);
//  motor2.drive(Vl);
//  delay(dT);
//}




#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SparkFun_TB6612.h>

// Defining motor parameters
#define PWMA 3
#define AIN2 4
#define AIN1 5
#define STBY 1
#define BIN1 7
#define BIN2 8
#define PWMB 9

const int offsetA = 1;
const int offsetB = 1;

double Vr, Vl, eta;
double Vnot = 10.00;    // Constant velocity in cm/s
double etaMax = 5.00;   // Strenght of rotational noise in rad/s
int dT = 250;           // Delay time in ms

const int CE = 6;
const int CSN = A0;
const byte address1[6] = "1Node";
const byte address2[6] = "2Node";

RF24 radio(CE, CSN);
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.setRetries(15, 15);
  radio.setPayloadSize(32);
  radio.openWritingPipe(address1);
  radio.openWritingPipe(address2);
  radio.startListening();
}

void loop() {
  radio.stopListening();
  eta = random(-100, 101) / (100.00 / etaMax);
  Serial.println(eta);
  radio.write(&eta, sizeof(eta), address1);
  radio.write(&eta, sizeof(eta), address2);
  Vr = ((eta * 7.00) + (2.00 * Vnot))/ 2.00; // In cm/s
  Vl = (2.00 * Vnot) - Vr;                   // In cm/s
  Vr = (Vr + 0.729) / 0.1114;                // Analog value
  Vl = ((Vl + 0.729) / 0.1114);              // Analog value
  radio.startListening();
  motor1.drive(Vr);
  motor2.drive(Vl);
  delay(dT);
}
