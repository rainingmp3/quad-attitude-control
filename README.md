# Quadcopter attitude controller simulation

This project simulates a quadrotor UAV PD-based attitude controller with 3D visualization of flight path.

## Features

- ✅ 6DOF drone body dynamics 
- ✅ Attitude control using PD 
- ✅ Inertia tensor, Mass & COM adaptation to variable payload
- ✅ Visualization and animation via matplotlib 3D 

## Key Files

- `main.py`: Simulation setup and control loop
- `drone.py`: Central Node of the simulation
- `controller.py`: Control input calculation  
- `dynamics.py` & `dynamic_utils.py`: Flight dynamics calculation
- `animator.py`: Visualization of drone position and attitude
## Demo
TODO: Add GIFs 
## Diploma  annotation in English

In the thesis, a system for controlling the direction and altitude of an Unmanned
Aerial Vehicle (UAV) such as a quadcopter was created. To calculate the flight dynamics of the UAV, a mathematical model of the vehicle was built, taking into account the effects of gravity, air resistance on the drone's body and its propellers, motor thrust, and the representation of forces and moments in different coordinate systems.
The created quadrotor flight algorithm is a second-order system and is capable of
bringing the drone to a given altitude, stabilizing the angles of roll, pitch, and yaw if the
initial position of the drone is not an equilibrium point.
An algorithm for adapting to an additional load is implemented in this work:
depending on the load mass, the center of mass, total mass, and inertia tensor change, which
consequently changes the drone's control equation and flight path.
To reproduce the results of the work, a simulation environment was built in the
Python programming language. The project did not use any other programming libraries
except numpy for working with matrices and matplotlib for playing animations and
displaying graphs of flight indicators.

## Sources 
Main sources of inspiration were Coursera Aerial Robotics by [Vijar Kumar](https://www.coursera.org/learn/chatgpt-excel-formulas-visualizations) and [MIT Robotics VNAV course](https://vnav.mit.edu/).