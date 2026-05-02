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
 * Module : Active Brownian Particle (ABP) Motion
 * Summary: Implements Active Brownian Particle motion: the robot translates at a
 * constant speed V0 while its heading undergoes rotational diffusion
 * with noise strength etaMax (rad/s). This is the canonical model for
 * self-propelled particles in active-matter physics.
 */

// This is the library for the TB6612 Motor Driver that contains the class Motor and all the functions
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
const int offsetA = 1;
const int offsetB = 1;

// Initializing motors.
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetA, STBY);

// Global variables
double Vr, Vl, eta;

// Input parameters
double Vnot = 6.00;   // Constant velocity in cm/s
double etaMax = 5.00;   // Strenght of rotational noise in rad/s
int    dT = 250;       // Delay time in ms

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  eta = random(-100, 101) / (100.00 / etaMax);
  Vr = ((eta * 7.00) + (2.00 * Vnot)) / 2.00; // In cm/s
  Vl = (2.00 * Vnot) - Vr;              // In cm/s

  Vr = (Vr + 1.0015) / 0.1059;           // Analog value
  Vl = ((Vl + 1.0015) / 0.1059);           // Analog value

  motor1.drive(Vr);
  motor2.drive(Vl);
  delay(dT);
}
