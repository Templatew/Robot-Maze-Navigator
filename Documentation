# Robot Maze Navigation Code Documentation

## Overview
This code controls a robot designed to navigate through a maze. It's part of a robotics simulation conducted in CoppeliaSim with HoroSim. The robot utilizes proximity sensors for wall-following and a light sensor for color detection. The navigation strategy combines following walls and random movement to explore the maze efficiently.

## Pin Definitions and Sensor Indices
- Pins for motors and sensors are defined for easy reference and modification.
- Sensor indices (`sensors_FL`, `sensors_FR`, etc.) provide a clear, consistent way to refer to each sensor's data.

## Hardware Setup
The `hardware_setup()` function initializes the robot's DC motors and sensors:
- `DCMotor_Hbridge` and `VisionSensor` objects are created for motor control and visual sensing.
- This initialization is crucial for the robot's interaction with its environment.

## Movement Functions
- `move(int speed_left, int speed_right)`: Controls the robot's motors to move forward, backward, or turn based on the provided speed values for each motor.
- `stop()`: Stops the robot by setting the motor speeds to zero.

## Sensor Reading
- `get_sensors_value()`: Reads and stores the values from all proximity and light sensors into the `val_sensors` array for further processing.

## Navigation Algorithms
1. **Follow Wall Right (`follow_wall_right()`)**:
   - Implements a PID-based wall-following algorithm on the right side.
   - Adjusts the robot's path based on the distance from the right wall to navigate along it.

2. **Follow Wall Left (`follow_wall_left()`)**:
   - Similar to `follow_wall_right()`, but follows the wall on the left side.

3. **Random Movement (`random()`)**:
   - Moves the robot randomly, especially useful in open areas or when other strategies fail.
   - Turns away from walls based on proximity sensor readings.

## Algorithm and Objective Management
- **Algorithm Selection (`enum Algorithm`)**:
   - Enumerates different navigation strategies: `FOLLOW_WALL_RIGHT`, `FOLLOW_WALL_LEFT`, `RANDOM`, and `STOP`.

- **Objective Tracking (`enum Objective`)**:
   - Manages the robot's goals like finding a black or red case and confirming the case's color.

- **Switching Algorithm Mode (`switchMode()`)**:
   - Alternates between `FOLLOW_WALL_RIGHT` and `RANDOM` modes. This switch happens based on certain conditions, like elapsed time.

- **Main Loop (`loop()`)**:
   - Central execution point where the robot's behavior is determined based on the current objective and algorithm.
   - Manages sensor readings and executes the corresponding navigation algorithm.

## Important Notes
- **Variable Tuning**: Many variable values, especially for sensors and PID control, have been determined through experimentation. Depending on the specific simulation setup and hardware, these values might need to be adjusted for optimal performance.
- **Modular Design**: The code is structured in a way that allows easy modification and testing of individual components, like sensor readings or movement strategies.
