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
 * Module : NRF24L01 — Master Node (Tree Network, v2)
 * Summary: Revised master node with corrected motor calibration constants
 * (Vr/Vl conversion: +0.729 / 0.1114 instead of +1.0015 / 0.1059).
 */

#include <RF24Network.h>
#include <SparkFun_TB6612.h>
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

// Defining motor parameters
#define PWMA 3
#define AIN2 4
#define AIN1 5
#define STBY 1
#define BIN1 7
#define BIN2 8
#define PWMB 9

const uint16_t this_node = 00;
const uint16_t node01 = 01;
const uint16_t node02 = 02;
const uint16_t node03 = 03;
const uint16_t node04 = 04;
const uint16_t node05 = 05;

const int offsetA = 1;
const int offsetB = 1;

double Vr, Vl, eta;
double Vnot = 10.00;     // Constant velocity in cm/s
double etaMax = 5.00;    // Strenght of rotational noise in rad/s
int    dT = 250;         // Delay time in ms

RF24 radio(6, A0);       // nRF24L01 (CE,CSN)

RF24Network network(radio);

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

RF24NetworkHeader header2(node01);
RF24NetworkHeader header3(node02);
RF24NetworkHeader header4(node03);
//RF24NetworkHeader header5(node04);
//RF24NetworkHeader header6(node05);

void setup() {
  SPI.begin();
  radio.begin();
  network.begin(90, this_node);  //(channel, node address)
  radio.setDataRate(RF24_2MBPS);
  Serial.begin(9600);
}

void loop() {
  //  unsigned long t1 = micros();
  eta = random(-100, 101) / (100.00 / etaMax);

  //  //  SEND TO NODE 1
  //  unsigned long t1 = micros();
  network.write(header2, &eta, sizeof(eta));
  //  unsigned long t2 = micros();
  //  Serial.println(t2 - t1);
  //
  //  //  SEND TO NODE
  //  unsigned long t3 = micros();
  network.write(header3, &eta, sizeof(eta));
  //  unsigned long t4 = micros();
  //  Serial.println(t4 - t3);



  //  SEND TO NODE 3
  //  unsigned long t5 = micros();
  network.write(header4, &eta, sizeof(eta));
  //  unsigned long t6 = micros();
  //  Serial.println(t6 - t5);

  //  SEND TO NODE 4
  //  network.write(header5, &eta, sizeof(eta));

  //  SEND TO NODE 5
  //  network.write(header6, &eta, sizeof(eta));


  //  REGULAR VELOCITY CODE
  Vr = ((eta * 7.00) + (2.00 * Vnot)) / 2.00;  // In cm/s
  Vl = (2.00 * Vnot) - Vr;                     // In cm/s
  Vr = (Vr + 0.729) / 0.1114;                  // Analog value
  Vl = ((Vl + 0.729) / 0.1114);                // Analog value
  motor1.drive(Vr);
  motor2.drive(Vl);
  delay(dT);
  //  unsigned long t2 = micros();
  //  Serial.println((t2-t1));
}
