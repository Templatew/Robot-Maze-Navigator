# Robot Maze Navigator

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

## Code
The source code for the robot's navigation system can be found here: [Robot Navigation Code](https://github.com/Templatew/Robot-Maze-Navigator/tree/main/Code).

## Example Mazes
Examples of the mazes that the robot can navigate can be found here: [Maze Examples](https://github.com/Templatew/Robot-Maze-Navigator/tree/main/Maze%20Exemples).

## Usage
Once the setup is complete, you can start the simulation in CoppeliaSim. The robot will automatically begin navigating the maze, employing its algorithms to find the black tile and return to the starting point.

## Gallery
[![Maze Small]](https://www.youtube.com/watch?v=gO2x4uKiWLo)
