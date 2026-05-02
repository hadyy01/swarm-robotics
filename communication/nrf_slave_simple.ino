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
 * Module : NRF24L01 — Slave (Point-to-Point)
 * Summary: Slave node: listens for eta from the master and drives its motors
 * using the same ABP velocity equations, achieving synchronised motion.
 */

#include <SparkFun_TB6612.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//Defining motor parameters
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
const byte address[6] = "1Node";

RF24 radio(CE, CSN);
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.setRetries(15, 15);
  radio.setPayloadSize(32);
  radio.setDataRate(RF24_2MBPS);
  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);
  radio.startListening();
}

void loop() {
  while (radio.available()) {
    radio.read(&eta, sizeof(eta));
    Vr = ((eta * 7.00) + (2.00 * Vnot)) / 2.00; // In cm/s
    Vl = (2.00 * Vnot) - Vr;              // In cm/s
    Vr = (Vr + 1.0015) / 0.1059;                // Analog value
    Vl = ((Vl + 1.0015) / 0.1059);           // Analog value
    motor1.drive(Vr);
    motor2.drive(Vl);
    delay(dT);
  }
  motor1.drive(0);
  motor2.drive(0);
}
