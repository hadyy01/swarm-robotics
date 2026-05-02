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
 * Module : Brownian Motion
 * Summary: Implements isotropic Brownian motion. Each motor receives an independent
 * random velocity drawn uniformly from [-V, V] at each time step dT.
 * Models a passive particle driven by thermal fluctuations.
 */

#include <SparkFun_TB6612.h>

#define PWMA 3
#define AIN2 4
#define AIN1 5
#define STBY 1
#define BIN1 7
#define BIN2 8
#define PWMB 9


const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

//INPUT PARAMETERS
float v = 6.00; //Velocity
float epsilon = 200; //Delay

void setup()
{
  Serial.begin(9600);
}


void loop() {
  int V = (v + 1.0015) / 0.1059;
  int value1 = random(-V, V + 1);
  int value2 = random(-V, V + 1);
  motor1.drive(value1);
  motor2.drive(value2);
  delay(epsilon);
}
