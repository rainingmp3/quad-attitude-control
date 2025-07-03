"""
Main script to initilize and run the drone simulation.
- Sets up initial conditions and drone parameters
- Runs the simulation loop to compute control input 
  and update states via Drone and Controller classes
- Visualizes simulation via Animator class

The drone has "x" shape configuration.
Off-diagonal terms of matrix of inertia are assumed to be 0.

Author: Kotanov Ivane
That was my diploma project :)
"""

from src.drone.drone import Drone
from src.control.controller import Controller
from src.visualization.animator import Animator
from src.visualization.plotter import Plotter     
from cases import case     

import numpy as np

case_params = case(4)

# Controller gains
# [x, y, z, phi, theta, psi]
Kp = np.array([0,0,5.2, 0.8,0.8,0])
Kd = np.array([0,0,2.9, 0.4,0.4,0])
# Drone physical characteristics:


sim_time = 10

drone_params = {
     "mass": 1.25,
     "armLength": 0.2,
     "rotorHeight":0.03,
     "Ixx": 0.023, 
     "Iyy": 0.023,
     "Izz": 0.047,
     "attMass":case_params["attMass"],
     "cubeHeight":0.05,
     "maxVoltage":11.1,
     "Kv": 850
     }

# Initial positional states of the drone in inertial frame: 
# [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
init_states = [
               0, 0,  0,       # Position (x,y,z)  
               0, 0,  0,      # Velocity (vx,vy,vz) 
               case_params["initPhi"], case_params["initTheta"],  0,      # Attitude (phi,theta,psi)
               0, 0,  0       # Angular Velocity (p,q,r)    
               ]

# Initial control inputs:
# [thrust, moment_x, moment_y, moment_z]
init_inputs = [0, 0, 0, 0]

# Desired states vector:
# [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
desired_states = [
                  0,  0,  0,       # Position (x,y,z)  
                  0,  0,  0,       # Velocity (vx,vy,vz)
                  0,  0,  0,       # Acceleration (ax,ay,az)
                  0,               # Yaw 
                  0,               # Yaw dot
]



# Timestep and simulation time setting
time_step = 0.05
gravity = 9.807
# Create Drone and Controller objects
schmetterling = Drone(params=drone_params,
                 initInputs=init_inputs,
                 initStates=init_states,
                 time_step=time_step, 
                 gravity=gravity)
controller = Controller(schmetterling, Kp,Kd ,time_step=time_step)


# Run simulation loop
for _ in np.arange(0,sim_time, time_step):
     # Compute control input based on difference between desired state and current
     schmetterling.update()   
     controller.compute_control_input(desiredStates= desired_states)
     # print(f"r_b2w is {schmetterling.euler_angles[0]}\n")

# Visualize simulation results
animator = Animator(schmetterling)
plotter = Plotter(schmetterling, controller)

animator.play(repeat=False)
plotter.plot_stats(show_time=1000)

# Print

# print(schmetterling.states_log[6:9])
