#include "SensorManager.h"


#define MOTOR_RF_PIN 2
#define MOTOR_RB_PIN 4
#define MOTOR_R_SPEED 3
#define MOTOR_LF_PIN 7
#define MOTOR_LB_PIN 8
#define MOTOR_L_SPEED 5


// Function to initialize hardware setup
void hardware_setup() {
  // Initialize DC motors
  new DCMotor_Hbridge(MOTOR_RF_PIN, MOTOR_RB_PIN, MOTOR_R_SPEED, "ePuck_rightJoint", 2.5, 3 * 3.14159, 1);
  new DCMotor_Hbridge(MOTOR_LF_PIN, MOTOR_LB_PIN, MOTOR_L_SPEED, "ePuck_leftJoint", 2.5, 3 * 3.14159, 1);
}


// Function to move with given speeds for left and right motors
void move(int speed_left, int speed_right) {

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

SensorManager sensorManager;


void setup() {
  Serial.begin(4800);
  pinMode(MOTOR_RF_PIN, OUTPUT);
  pinMode(MOTOR_RB_PIN, OUTPUT);
  pinMode(MOTOR_R_SPEED, OUTPUT);
  pinMode(MOTOR_LF_PIN, OUTPUT);
  pinMode(MOTOR_LB_PIN, OUTPUT);
  pinMode(MOTOR_L_SPEED, OUTPUT);
}

#define DISTANCE_TARGET 700          // Target distance from the wall
#define MAX_SPEED 255                // Maximum speed of the motors
#define MIN_SPEED 50                 // Minimum speed of the motors
#define MAX_DISTANCE 1023            // Maximum distance value of the proximity sensors
#define DISTANCE_SENSOR_FORWARD 800  // Distance from the wall to stop when following the wall
#define EPSILON_COLOR 1              // Epsilon value for color detection
#define BLACK_VALUE 28               // Value of the light sensor when on black
#define RED_VALUE 284                // Value of the light sensor when on red
#define WHITE_VALUE 1023             // Value of the light sensor when on white


// Function to check if a wall is in front of the robot
bool wall_in_front() {
  return sensorManager.sensorValues[sensorManager.sensors_FL] < DISTANCE_SENSOR_FORWARD || sensorManager.sensorValues[sensorManager.sensors_FR] < DISTANCE_SENSOR_FORWARD;
}

// Function to constrain a value between two limits
int customConstrain(int x, int a, int b) {
  if (x < a) {
    return a;
  } else if (x > b) {
    return b;
  } else {
    return x;
  }
}

// PID coefficients
float Kp = 0.35;  // Proportional coefficient
float Ki = 0.0;   // Integral coefficient
float Kd = 0.0;   // Derivative coefficient

// PID variables
float integral = 0.0;
float previous_error = 0.0;

// Function to follow the wall on the right
void follow_wall_right() {

  // PID variables
  float error, derivative, output;
  int speed_left, speed_right;

  // Get sensor values
  sensorManager.getSensorValues();

  while (wall_in_front()) {
    move(0, MAX_SPEED);
    sensorManager.getSensorValues();
  }

  // Error calculation
  error = sensorManager.sensorValues[sensorManager.sensors_DR] - DISTANCE_TARGET;

  // Integral calculation
  integral += error;

  // Derivative calculation
  derivative = error - previous_error;

  // PID output calculation
  output = Kp * error + Ki * integral + Kd * derivative;

  // Calculate the adjusted speeds
  speed_left = MAX_SPEED + output;
  speed_right = MAX_SPEED - output;

  // Constrain the speeds between 0 and MAX_SPEED
  speed_left = customConstrain(speed_left, MIN_SPEED, MAX_SPEED);
  speed_right = customConstrain(speed_right, MIN_SPEED, MAX_SPEED);

  // Set motor speeds
  move(speed_left, speed_right);

  // Store the previous error
  previous_error = error;
}


//  Function to follow the wall on the left
void follow_wall_left() {

  // PID variables
  float error, derivative, output;
  int speed_left, speed_right;

  // Get sensor values
  sensorManager.getSensorValues();

  while (wall_in_front()) {
    move(MAX_SPEED, 0);
    sensorManager.getSensorValues();
  }

  // Error calculation
  error = sensorManager.sensorValues[sensorManager.sensors_DL] - DISTANCE_TARGET;

  // Integral calculation
  integral += error;

  // Derivative calculation
  derivative = error - previous_error;

  // PID output calculation
  output = Kp * error + Ki * integral + Kd * derivative;

  // Calculate the adjusted speeds
  speed_left = MAX_SPEED - output;
  speed_right = MAX_SPEED + output;

  // Constrain the speeds between 0 and MAX_SPEED
  speed_left = customConstrain(speed_left, MIN_SPEED, MAX_SPEED);
  speed_right = customConstrain(speed_right, MIN_SPEED, MAX_SPEED);

  // Set motor speeds
  move(speed_left, speed_right);

  // Store the previous error
  previous_error = error;
}

// Function to move randomly
void random() {

  // Get sensor values
  sensorManager.getSensorValues();

  // If no wall is detected in front of the robot, move forward
  if (!wall_in_front()) {
    move(MAX_SPEED, MAX_SPEED);  // Move forward
  }

  // If a wall is detected in front of the robot
  else {

    // If the wall is ont the left, turn right
    if (sensorManager.sensorValues[sensorManager.sensors_FL] < sensorManager.sensorValues[sensorManager.sensors_FR]) {

      // Turn right until the wall is not in front of the robot anymore
      while (sensorManager.sensorValues[sensorManager.sensors_FL] < MAX_DISTANCE) {
        sensorManager.getSensorValues();
        move(MAX_SPEED, -MAX_SPEED);
      }

    }

    // If the wall is on the right, turn left
    else {

      // Turn left until the wall is not in front of the robot anymore
      while (sensorManager.sensorValues[sensorManager.sensors_FR] < MAX_DISTANCE) {
        sensorManager.getSensorValues();
        move(-MAX_SPEED, MAX_SPEED);
      }
    }
  }
}

// Algorithm selection
enum Algorithm { FOLLOW_WALL_RIGHT,
                 RANDOM,
                 FOLLOW_WALL_LEFT,
                 STOP };
Algorithm algorithm = FOLLOW_WALL_RIGHT;

// Objectives
enum Objective { FIND_BLACK_CASE,
                 FIND_RED_CASE,
                 CONFIRMATION_BLACK,
                 CONFIRMATION_RED,
                 DONE };
Objective objective = FIND_BLACK_CASE;

// Time variables for confirmation of color
unsigned long start_time_color_confirmation = 0;
//
#define TIME_COLOR_CONFIRMATION 0.99


bool found_black = false;        // Boolean to know if the robot has found a black case
bool left_first_case = false;    // Boolean to know if the robot has left the starting case
bool algorithm_changed = false;  // Boolean to know if the algorithm has changed

// Switch algorithm in Random Mode
unsigned long lastChangeTime = 0;
const unsigned long changeInterval = 3 * 1000;  

// Function to switch between algorithms in Random Mode (FOLLOW_WALL_RIGHT, RANDOM, FOLLOW_WALL_LEFT)
void switchMode() {

  if (algorithm == FOLLOW_WALL_RIGHT) {
    algorithm = RANDOM;
  } else {
    algorithm = FOLLOW_WALL_RIGHT;
  }
}

void loop() {

  // Get sensor values
  sensorManager.getSensorValues();

  // Switch between objectives
  switch (objective) {

    // If the objective is to find a black case
    case FIND_BLACK_CASE:

      // If the robot is on a white case for the first time
      if (!left_first_case && abs(sensorManager.sensorValues[sensorManager.sensors_light] - WHITE_VALUE) < EPSILON_COLOR) {
        left_first_case = true;  // Set the boolean to true
        Serial.println("left_first_case");
      }

      // If the robot is on a black case
      else if (abs(sensorManager.sensorValues[sensorManager.sensors_light] - BLACK_VALUE) < EPSILON_COLOR) {
        objective = CONFIRMATION_BLACK;            // Go to confirmation
        start_time_color_confirmation = millis();  // Start the timer
      }

      // If the robot is on a red case and has already left the starting case
      else if (left_first_case && (sensorManager.sensorValues[sensorManager.sensors_light] - RED_VALUE) < EPSILON_COLOR) {
        objective = CONFIRMATION_RED;              // Go to find a red case
        start_time_color_confirmation = millis();  // Start the timer
      }
      break;

    // If the objective is to find a red case
    case FIND_RED_CASE:

      // If the robot is on a red case
      if (abs(sensorManager.sensorValues[sensorManager.sensors_light] - RED_VALUE) < EPSILON_COLOR) {
        objective = CONFIRMATION_RED;              // Go to confirmation
        start_time_color_confirmation = millis();  // Start the timer
      }
      break;

    // If the objective is to confirm that the robot is really on a black case
    case CONFIRMATION_BLACK:

      // If the robot is still on a black case after the confirmation time
      if (millis() - start_time_color_confirmation > TIME_COLOR_CONFIRMATION && abs(sensorManager.sensorValues[sensorManager.sensors_light] - BLACK_VALUE) < EPSILON_COLOR) {
        objective = FIND_RED_CASE;  // Go to find a red case
        found_black = true;         // Set the boolean to true
        Serial.println("FIND_RED_CASE");
      }

      // If the robot is not on a black case anymore
      else {
        objective = FIND_BLACK_CASE;  // Go to find a black case
      }
      break;

    // If the objective is to confirm that the robot is really on a red case
    case CONFIRMATION_RED:

      // If the robot is still on a red case after the confirmation time
      if (millis() - start_time_color_confirmation > TIME_COLOR_CONFIRMATION && abs(sensorManager.sensorValues[sensorManager.sensors_light] - RED_VALUE) < EPSILON_COLOR) {

        // If the robot has already found a black case
        if (found_black) {
          algorithm = STOP;  // Stop the robot
          objective = DONE;  // Set the objective to DONE
          Serial.println("DONE");
        }

        // If the robot has not found a black case yet
        else if (!found_black && !algorithm_changed) {
          algorithm = RANDOM;           // Change the algorithm to RANDOM
          objective = FIND_BLACK_CASE;  // Go to find a black case
          algorithm_changed = true;     // Set the boolean to true
          Serial.println("Algorithm changed");
        }

        else if (!found_black) {
        }

      }

      // If the robot is not on a red case anymore
      else {

        // If the robot has already found a black case
        if (found_black) {
          objective = FIND_RED_CASE;  // Go to find a red case
        }

        // If the robot has not found a black case yet
        else {
          objective = FIND_BLACK_CASE;  // Go to find a black case
        }
      }
      break;

    // If the objective is completed
    case DONE:

      algorithm = STOP;  // Stop the robot
      break;

    default:
      break;
  }

  // // Switch algorithm in Random Mode
  // if (algorithm_changed == true && millis() - lastChangeTime > changeInterval) {
  //   switchMode();
  //   lastChangeTime = millis();
  // }

  // Switch between algorithms
  switch (algorithm) {

    // If the chosen algorithm is FOLLOW_WALL_RIGHT
    case FOLLOW_WALL_RIGHT:

      follow_wall_right();  // Follow the wall on the right
      break;

    // If the chosen algorithm is FOLLOW_WALL_LEFT
    case FOLLOW_WALL_LEFT:

      follow_wall_left();  // Follow the wall on the left
      break;

    // If the chosen algorithm is RANDOM
    case RANDOM:

      random();  // Move randomly
      break;

    // If the chosen algorithm is STOP
    case STOP:

      stop();  // Stop the robot
      break;

    default:
      break;
  }
}
