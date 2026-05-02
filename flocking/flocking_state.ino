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
 * Module : Reynolds Flocking — Decentralised State Machine
 * Summary: Equivalent flocking implementation with explicit state transitions.
 * Robots share position and movement vectors over nRF24L01+,
 * compute separation and alignment-cohesion vectors cooperatively,
 * and drive toward the emergent flock centre.
 */

#include "Vector2D.h"
#include <SparkFun_TB6612.h>
#include <RF24Network.h>
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

//MOTOR PINS
#define PWMA 3
#define AIN2 4
#define AIN1 5
#define STBY 1
#define BIN1 7
#define BIN2 8
#define PWMB 9

/*
THESE CONSTANTS ARE USED TO ALLOW YOU TO MAKE YOUR MOTOR CONFIGURABLE AND ALSO 
LINE UP WITH FUNCTION NAMES LIKE FORWARD. VALUE CAN BE -1 OR 1
*/
const int offsetA = 1;
const int offsetB = 1;

//INITIALIZING MOTORS
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// Define the nRF24L01 pins
#define CE_PIN 6
#define CSN_PIN A0

// Define the addresses for the nRF24L01 modules
byte addresses[][6] = {"robot3", "robot1", "robot2"};

// Define the TCRT5000 pins
#define TCRT_PIN_1 A5
#define TCRT_PIN_2 A2
#define TCRT_PIN_3 A1

// Define the robot's position and movement vectors
Vector2D position(0, 0);
Vector2D movement(0, 0);

// Define the separation and alignment-cohesion vectors
Vector2D separation_vector(0, 0);
Vector2D alignment_cohesion_vector(0, 0);

// Define the final flocking vector
Vector2D flocking_vector(0, 0);

// Define the radio object
RF24 radio(CE_PIN, CSN_PIN);

// Define the function to calculate the distance between two robots
float calculate_distance(Vector2D a, Vector2D b) {
  return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

// Define the function to calculate the separation vector between the robot and a nearby robot
Vector2D calculate_separation_vector(Vector2D other_position) {
  float distance = calculate_distance(position, other_position);
  if (distance > 0 && distance < 100) {
    Vector2D separation_direction = position.operator-(other_position).normalized();
    float separation_magnitude = map(distance, 0, 100, 50, 0);
    Vector2D separation_vector = separation_direction.operator*(separation_magnitude);
    return separation_vector;
  } else {
    return Vector2D(0, 0);
  }
}

// Define the function to calculate the alignment-cohesion vector between the robot and a nearby robot
Vector2D calculate_alignment_cohesion_vector(Vector2D other_position, Vector2D other_movement) {
  float distance = calculate_distance(position, other_position);
  if (distance > 0 && distance < 100) {
    Vector2D alignment_direction = other_movement.normalized();
    Vector2D cohesion_direction = other_position.operator-(position).normalized();
    float alignment_magnitude = map(distance, 0, 100, 0, 50);
    float cohesion_magnitude = map(distance, 0, 100, 0, 50);
    Vector2D alignment_vector = alignment_direction.operator*(alignment_magnitude);
    Vector2D cohesion_vector = cohesion_direction.operator*(cohesion_magnitude);
    return alignment_vector.operator+(cohesion_vector);
  } else {
    return Vector2D(0, 0);
  }
}

// Define the function to calculate the final flocking vector from the separation and alignment-cohesion vectors
Vector2D calculate_flocking_vector(Vector2D separation_vector, Vector2D alignment_cohesion_vector) {
  separation_vector = separation_vector.normalized().operator*(0.5);
  alignment_cohesion_vector = alignment_cohesion_vector.normalized().operator*(0.5);
  return separation_vector.operator+(alignment_cohesion_vector);
}

// Define the function to handle incoming data from other robots
void handle_data() {
if (radio.available()) {
Vector2D other_position(0, 0);
Vector2D other_movement(0, 0);
radio.read(&other_position, sizeof(Vector2D));
radio.read(&other_movement, sizeof(Vector2D));
separation_vector = separation_vector.operator+(calculate_separation_vector(other_position));
alignment_cohesion_vector = alignment_cohesion_vector.operator+(calculate_alignment_cohesion_vector(other_position, other_movement));
}
}

// Define the setup function
void setup() {
Serial.begin(9600);
// Set the motor speeds to 0
motor1.drive(0);
motor2.drive(0);

// Start the radio
radio.begin();
radio.setChannel(115);
radio.setPALevel(RF24_PA_MIN);
radio.setDataRate(RF24_250KBPS);
radio.openReadingPipe(1, addresses[0]);
radio.openWritingPipe(addresses[1]);
radio.openWritingPipe(addresses[2]);
}

// Define the loop function
void loop() {
  motor1.drive(50);
  motor2.drive(-50);
  delay(10000);
// Read the sensor values
int sensor_value_1 = analogRead(TCRT_PIN_1);
int sensor_value_2 = analogRead(TCRT_PIN_2);
int sensor_value_3 = analogRead(TCRT_PIN_3);

Serial.println(sensor_value_1);

// Update the position and movement vectors based on the sensor values
position.x = sensor_value_1;
position.y = (sensor_value_2 + sensor_value_3) / 2;
movement.x = sensor_value_1 - sensor_value_3;
movement.y = sensor_value_2 - sensor_value_3;

// Calculate the final flocking vector
separation_vector = separation_vector.operator*(0.5);
alignment_cohesion_vector = alignment_cohesion_vector.operator*(0.5);
flocking_vector = calculate_flocking_vector(separation_vector, alignment_cohesion_vector);

// .normalized the flocking vector
if (flocking_vector.magnitude() > 0.5) {
flocking_vector = flocking_vector.normalized().operator*(0.5);
}

// Transmit the position and movement vectors to the other robots
radio.write(&position, sizeof(Vector2D));
radio.write(&movement, sizeof(Vector2D));

// Handle incoming data from other robots
handle_data();

// Set the motor speeds based on the final flocking vector
motor1.drive(map(flocking_vector.x, -0.5, 0.5, 0, 255));
motor2.drive(map(flocking_vector.y, -0.5, 0.5, 0, 255));

// Move the robot
if (flocking_vector.x > 0) {
motor1.drive(50);
} else {
motor1.drive(-50);
}
if (flocking_vector.y > 0) {
motor2.drive(50);
} else {
motor2.drive(-50);
}

// Clear the separation and alignment-cohesion vectors
separation_vector = Vector2D(0, 0);
alignment_cohesion_vector = Vector2D(0, 0);
}