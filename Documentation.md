# Robot Maze Navigation Code Documentation

## Overview
This code controls a robot designed to navigate through a maze. It's part of a robotics simulation conducted in CoppeliaSim with HoroSim. The robot utilizes proximity sensors for wall-following and a light sensor for color detection. The navigation strategy combines following walls and random movement to explore the maze efficiently.

## Global Behavior of the Robot

The robot's navigation strategy in the maze is governed by a series of logical steps and mode switches to ensure comprehensive exploration and the achievement of its objectives. The behavior can be summarized as follows:

1. **Initial Strategy - Follow Right Wall**:
   - At the start, the robot adheres to a wall-following algorithm where it keeps the right wall in proximity. This strategy is effective for initial exploration and helps in mapping the periphery of the maze.

2. **Switch to Random Mode**:
   - If the robot returns to the starting position (the red case) without having detected the black case, it implies that the right wall-following strategy has not led to the goal. Consequently, the robot switches to a "random" navigation algorithm. This change in strategy is aimed at exploring the central or unvisited parts of the maze that might not be reachable with the wall-following approach.

3. **Objective Completion - Finding the Black Case**:
   - The primary objective for the robot is to find a black case within the maze. Once the robot detects a black case, it changes its goal to finding its way back to the red case (the starting position).

4. **Returning to the Start**:
   - After finding the black case, the robot aims to navigate back to the starting point. The strategy employed for the return journey may vary, but it typically involves either continuing the random navigation or switching back to a wall-following approach, depending on the maze's structure and the robot's current position.

5. **Adaptive Strategy**:
   - Throughout its navigation, the robot continually adapts its strategy based on the environmental feedback obtained through its sensors. This adaptive approach allows the robot to effectively deal with the dynamic and unpredictable nature of the maze environment.

### Note
- The values of various variables, especially those related to sensors and the PID controller, have been determined through trial and error. Depending on the simulation setup and the specific characteristics of the PC used (in this case, an i9-11900k processor), these variables may need to be adjusted for optimal performance.

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
