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
 * Module : NRF24L01 — Slave Node (Tree Network, v2)
 * Summary: Revised slave with updated motor calibration constants matching v2 master.
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

const uint16_t this_node = 03;   // Address of our node in Octal format ( 04,031, etc)
const uint16_t master00 = 00;    // Address of the other node in Octal format

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

void setup() {

  SPI.begin();
  radio.begin();
  network.begin(90, this_node); //(channel, node address)
  radio.setDataRate(RF24_2MBPS);
}
void loop() {
  network.update();
//  Serial.println(network.available());
  if (network.available()) {
    while ( network.available() ) {
      RF24NetworkHeader header;
      int data;
      network.read(header, &eta, sizeof(eta)); // Read the incoming data
      Vr = ((eta * 7.00) + (2.00 * Vnot)) / 2.00; // In cm/s
      Vl = (2.00 * Vnot) - Vr;              // In cm/s
      Vr = (Vr + 0.729) / 0.1114;           // Analog value
      Vl = ((Vl + 0.729) / 0.1114);           // Analog value
      motor1.drive(Vr);
      motor2.drive(Vl);
//      delay(dT);
    }
  }
  else {
    motor1.drive(0);
    motor2.drive(0);
  }
}
