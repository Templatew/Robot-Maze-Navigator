// Pin definitions for sensors and motors
#define LIGHT_SENSOR_PIN 12
#define PROX_SENSOR_L_PIN 9
#define PROX_SENSOR_R_PIN A2
#define PROX_SENSOR_FL_PIN A3
#define PROX_SENSOR_FR_PIN A1
#define PROX_SENSOR_RL_PIN 6
#define PROX_SENSOR_RR_PIN A4
#define PROX_SENSOR_DL_PIN A5
#define PROX_SENSOR_DR_PIN A0

#define MOTOR_RF_PIN 2
#define MOTOR_RB_PIN 4
#define MOTOR_R_SPEED 3
#define MOTOR_LF_PIN 7
#define MOTOR_LB_PIN 8
#define MOTOR_L_SPEED 5

// Sensor indices
#define sensors_FL 0
#define sensors_FR 1
#define sensors_L 2
#define sensors_R 3
#define sensors_RL 4
#define sensors_RR 5
#define sensors_DL 6
#define sensors_DR 7
#define sensors_light 8

#define MAX_SPEED 255

#define BLACK_VALUE 40
#define RED_VALUE 500
#define MAX_DISTANCE 1023
#define DISTANCE_SENSOR_FORWARD 800
#define DISTANCE_SENSOR_SIDE_MAX 900
#define DISTANCE_SENSOR_SIDE_MIN 50
#define EPSILON_DR_RR 100
#define COMPENSATION_DR_RR 160

// Array to store sensor values
int val_sensors[9];

// Function to initialize hardware setup
void hardware_setup() {
  // Initialize DC motors
  new DCMotor_Hbridge(MOTOR_RF_PIN, MOTOR_RB_PIN, MOTOR_R_SPEED, "ePuck_rightJoint", 2.5, 3 * 3.14159, 1);
  new DCMotor_Hbridge(MOTOR_LF_PIN, MOTOR_LB_PIN, MOTOR_L_SPEED, "ePuck_leftJoint", 2.5, 3 * 3.14159, 1);

  // Initialize light sensor
  new VisionSensor(LIGHT_SENSOR_PIN, "ePuck_lightSensor", 0.1);

  // Initialize proximity sensors
  new ProximitySensor(PROX_SENSOR_FL_PIN, "ePuck_proxSensor3", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_FR_PIN, "ePuck_proxSensor4", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_L_PIN, "ePuck_proxSensor1", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_R_PIN, "ePuck_proxSensor6", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_RL_PIN, "ePuck_proxSensor8", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_RR_PIN, "ePuck_proxSensor7", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_DL_PIN, "ePuck_proxSensor2", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_DR_PIN, "ePuck_proxSensor5", 0.1, 1);
}

// Function to move forward with a given speed
void forward(int speed) {
  // Set motor directions
  digitalWrite(MOTOR_RF_PIN, HIGH);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, HIGH);
  digitalWrite(MOTOR_LB_PIN, LOW);

  // Set motor speeds
  analogWrite(MOTOR_R_SPEED, speed);
  analogWrite(MOTOR_L_SPEED, speed);
}

// Function to move backward with a given speed
void backward(int speed) {
  // Set motor directions
  digitalWrite(MOTOR_RF_PIN, LOW);
  digitalWrite(MOTOR_RB_PIN, HIGH);
  digitalWrite(MOTOR_LF_PIN, LOW);
  digitalWrite(MOTOR_LB_PIN, HIGH);

  // Set motor speeds
  analogWrite(MOTOR_R_SPEED, speed);
  analogWrite(MOTOR_L_SPEED, speed);
}

// Function to turn with given speeds for left and right motors
void turn(int speed_left, int speed_right) {
  // Set right motor direction based on speed
  if (speed_right < 0) {
    digitalWrite(MOTOR_RF_PIN, LOW);
    digitalWrite(MOTOR_RB_PIN, HIGH);
  } else {
    digitalWrite(MOTOR_RF_PIN, HIGH);
    digitalWrite(MOTOR_RB_PIN, LOW);
  }

  // Set left motor direction based on speed
  if (speed_left < 0) {
    digitalWrite(MOTOR_LF_PIN, LOW);
    digitalWrite(MOTOR_LB_PIN, HIGH);
  } else {
    digitalWrite(MOTOR_LF_PIN, HIGH);
    digitalWrite(MOTOR_LB_PIN, LOW);
  }

  // Set motor speeds
  analogWrite(MOTOR_R_SPEED, abs(speed_right));
  analogWrite(MOTOR_L_SPEED, abs(speed_left));
}

// Function to stop the motors
void stop() {
  analogWrite(MOTOR_R_SPEED, 0);
  analogWrite(MOTOR_L_SPEED, 0);
}

// Function to get sensor values
void get_sensors_value() {
  val_sensors[sensors_FL] = analogRead(PROX_SENSOR_FL_PIN);
  val_sensors[sensors_FR] = analogRead(PROX_SENSOR_FR_PIN);
  val_sensors[sensors_L] = analogRead(PROX_SENSOR_L_PIN);
  val_sensors[sensors_R] = analogRead(PROX_SENSOR_R_PIN);
  val_sensors[sensors_RL] = analogRead(PROX_SENSOR_RL_PIN);
  val_sensors[sensors_RR] = analogRead(PROX_SENSOR_RR_PIN);
  val_sensors[sensors_DL] = analogRead(PROX_SENSOR_DL_PIN);
  val_sensors[sensors_DR] = analogRead(PROX_SENSOR_DR_PIN);
  val_sensors[sensors_light] = analogRead(LIGHT_SENSOR_PIN);
}


void setup() {
  Serial.begin(4800);
  pinMode(MOTOR_RF_PIN, OUTPUT);
  pinMode(MOTOR_RB_PIN, OUTPUT);
  pinMode(MOTOR_R_SPEED, OUTPUT);
  pinMode(MOTOR_LF_PIN, OUTPUT);
  pinMode(MOTOR_LB_PIN, OUTPUT);
  pinMode(MOTOR_L_SPEED, OUTPUT);
  pinMode(PROX_SENSOR_FL_PIN, INPUT);
  pinMode(PROX_SENSOR_FR_PIN, INPUT);
  pinMode(PROX_SENSOR_L_PIN, INPUT);
  pinMode(PROX_SENSOR_R_PIN, INPUT);
  pinMode(PROX_SENSOR_RL_PIN, INPUT);
  pinMode(PROX_SENSOR_RR_PIN, INPUT);
  pinMode(PROX_SENSOR_DL_PIN, INPUT);
  pinMode(PROX_SENSOR_DR_PIN, INPUT);
  pinMode(LIGHT_SENSOR_PIN, INPUT);
}


// bool wall_on_the_right() {
//   return val_sensors[sensors_R] < DISTANCE_SENSOR_SIDE;
// }

// bool wall_on_the_left() {
//   return val_sensors[sensors_L] < DISTANCE_SENSOR_SIDE;
// }

bool wall_in_front() {
  return val_sensors[sensors_FL] < DISTANCE_SENSOR_FORWARD || val_sensors[sensors_FR] < DISTANCE_SENSOR_FORWARD;
}

void follow_wall_right() {

    while(wall_in_front()) {
            turn(0, MAX_SPEED);
            get_sensors_value();
    }

    if (val_sensors[sensors_DR] > DISTANCE_SENSOR_SIDE_MAX) {
        turn(MAX_SPEED*.90, MAX_SPEED*0.40);
    } 
    else if (val_sensors[sensors_DR] < DISTANCE_SENSOR_SIDE_MIN) {
        turn(MAX_SPEED*0.40, MAX_SPEED*0.90);
    } 
    else if ( (val_sensors[sensors_DR] + COMPENSATION_DR_RR)  - val_sensors[sensors_RR] > EPSILON_DR_RR) {
        turn(MAX_SPEED, MAX_SPEED*0.80);
    } 
    else if (val_sensors[sensors_RR] - (val_sensors[sensors_DR] + COMPENSATION_DR_RR) > EPSILON_DR_RR ) {
        turn(MAX_SPEED*0.80, MAX_SPEED);
    } 
    else {
        forward(MAX_SPEED);
    }
}

// Algorithm selection
enum Algorithm {FOLLOW_WALL_RIGHT, RANDOM};
Algorithm algorithm = FOLLOW_WALL_RIGHT;

// Objectives
enum Objective {FIND_BLACK_CASE, FIND_RED_CASE};
Objective objective = FIND_BLACK_CASE;

void loop() {

    // Switch between algorithms
    switch(algorithm) {

        

        // If the chosen algorithm is FOLLOW_WALL_RIGHT
        case FOLLOW_WALL_RIGHT:

            // Get sensor values
            get_sensors_value();

            follow_wall_right();
        
        // If the chosen algorithm is RANDOM
        case RANDOM:

            break;

        default:
            break;
    }

}
