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
 * Module : NRF24L01 — Diagnostic / Inspection
 * Summary: Prints the nRF24L01+ register values over Serial for hardware
 * verification. Expected healthy values:
 *   EN_AA=0x3f  EN_RXADDR=0x02  RF_CH=0x4c  RF_SETUP=0x03  CONFIG=0x0f
 */

#include <SPI.h>
#include <RF24.h>
#include <printf.h>

RF24 radio(6, A0);

byte addresses[][6] = {"1Node", "2Node"};


void setup() {
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]); 
  radio.startListening();
  
  Serial.begin(9600);
  printf_begin();

  radio.printDetails();
  
}

void loop() {
//  empty

}
