![image](https://github.com/Templatew/Robot-Maze-Navigator/assets/96289463/236eb8ce-0977-469f-9c5c-61a7a00cb674)

## Project Overview
This repository contains the code and documentation for a maze-navigating robot, designed as part of a robotics course. The robot, simulated in CoppeliaSim with HoroSim, utilizes a combination of wall-following and random movement strategies to navigate through dynamically generated mazes.

## Features
- **Wall-Following Algorithm**: The robot can follow the right or left wall to navigate through the maze.
- **Random Movement Strategy**: Used for exploring open areas and for cases where the wall-following algorithm is insufficient.
- **Color Detection**: Utilizes a reflectivity sensor to detect colored tiles on the maze floor, specifically targeting a black tile as the goal.
- **Automatic Mode Switching**: The robot alternates between wall-following and random movement to effectively explore the maze.

## Hardware and Simulation Setup
- **CoppeliaSim**: The simulation environment.
- **HoroSim**: Bridges the communication between Arduino and CoppeliaSim.
- **Sensors**: Equipped with 8 proximity sensors and a reflectivity sensor for color detection.

## Getting Started
To run this project, you will need to set up CoppeliaSim with HoroSim and load the provided simulation environment. The Arduino code can be uploaded to a compatible Arduino board for physical implementation or used directly within the simulation.

### Prerequisites
- CoppeliaSim (latest version)
- Arduino IDE
- HoroSim plugin for CoppeliaSim

### Installation
1. Clone the repository to your local machine.
2. Open the simulation environment in CoppeliaSim.
3. Load the Arduino code into the Arduino IDE and upload it to your Arduino board, or integrate it with the HoroSim plugin in CoppeliaSim.

## Usage
Once the setup is complete, you can start the simulation in CoppeliaSim. The robot will automatically begin navigating the maze, employing its algorithms to find the black tile and return to the starting point.

## Code
The source code for the robot's navigation system can be found here: [Robot Navigation Code](https://github.com/Templatew/Robot-Maze-Navigator/tree/main/Code).

## Documentation
For detailed information about the implementation and functionalities of the robot's navigation system, please refer to the [Code Documentation](https://github.com/Templatew/Robot-Maze-Navigator/blob/main/Documentation.md). This documentation provides an in-depth look at the structure, algorithms, and components of the code, offering insights into how the robot navigates through the maze and processes sensor data.

## Example Mazes
Examples of the mazes that the robot can navigate can be found here: [Maze Examples](https://github.com/Templatew/Robot-Maze-Navigator/tree/main/Maze%20Exemples).

## Contributors

 - [CAZAUBON Lorenz](https://github.com/Templatew)
 - [PERDREAU Robin](https://github.com/BlackJackGeary83)

## Gallery
In this section, you can find videos showcasing the successful execution of the robot's maze navigation code. These videos are screen recordings taken on a computer equipped with an i9-11900k processor, which demonstrate how the code performs in real-time within the CoppeliaSim environment.

### Video Demonstrations

- **Video 1**: [Robot Navigating Maze - Small](https://www.youtube.com/watch?v=gO2x4uKiWLo)
  

https://github.com/Templatew/Robot-Maze-Navigator/assets/96289463/07e115f3-0769-4a23-995a-0d825588bf9a


- **Video 2**: [Robot Navigating Maze - Normal 1](https://www.youtube.com/watch?v=eE6V4Z8Rsyo)
- **Video 3**: [Robot Navigating Maze - Normal 2](https://www.youtube.com/watch?v=hEwCGLeCXpU)
- **Video 4**: [Robot Navigating Maze - Big](https://www.youtube.com/watch?v=pYgpAhSzy7s)

### Screenshots
![image](https://github.com/Templatew/Robot-Maze-Navigator/assets/96289463/7290d963-7b00-4dda-8960-42e184f3d003)
