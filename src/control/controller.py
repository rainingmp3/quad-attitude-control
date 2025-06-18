import numpy as np
# from simple_pid import PID
from src.drone.rotation import wrap_angles


class Controller():
    """
    
    PID Controller for Drone.
    Uses proportional, derivative, and integral terms to compute
    control input. 
    
    """
    def __init__(self, 
                 drone, 
                 kp,
                 kd,
                 time_step: float) -> None: 
        """
        Initialize the PID controller.

        Args:
            drone: Drone object. 
            kp: Proportional gain.
            ki: Integral gain.
            kd: Derivative gain.
            time_step: Time step.

        """
        
        self.drone = drone
        self.kp = kp
        self.kd = kd
        self.time_step = time_step

        # Initial total error and previous time step error:
        # self.prev_err = 0 
        # self.sum_err  = 0 
        
        # Store error and desired state over simulation:
        self.error_log = []
        self.desired_log = []
        self.input_log = []

    def compute_control_input(self,desiredStates):
        """
        Compute control input signal via PID.
        Args:
            states: Current state vector.
            desiredStates: Desired state vector.
        Returns:
            Control signal vector control_input.
        """
         # Define setpoints
        desired_state = {}

        height = 5
        # desired_state["pos"] = desiredStates[:3]
        desired_state["pos"] = np.array([0,0,height])
        # desired_state["vel"] = desiredStates[3:6]
        desired_state["vel"] = np.array([0,0,0])
        desired_state["acc"] = np.array([0,0,0])
        desired_state["yaw"] = 0
        desired_state["yaw_dot"]  = 0


        # Define errors
        self.ep = desired_state["pos"] - self.drone.pos
        self.ed = desired_state["vel"] - self.drone.vel

        # Compute control thrust
        self.drone.control_input[0] = self.drone.mass * (self.drone.g + desired_state["acc"][2] +
                                             self.kd[2]* self.ed[2] + self.kp[2]* self.ep[2])
        
        # Define desired acelerations and attitude 
        ddxd = desired_state['acc'][0] + self.kd[0] * self.ed[0] + self.kp[0] * self.ep[0]
        ddyd = desired_state['acc'][1] + self.kd[1] * self.ed[1] + self.kp[1] * self.ep[1]
        
        phid = 0 
        thed = 0
        # Compute control torques
        self.drone.control_input[1] = self.kp[3] * (phid-self.drone.euler_angles[0]) - self.kd[3] * self.drone.angular_vel[0]
        self.drone.control_input[2] = self.kp[4] * (thed-self.drone.euler_angles[1]) - self.kd[4] * self.drone.angular_vel[1]
        self.drone.control_input[3] = self.kp[5] * (desired_state["yaw"] - self.drone.euler_angles[2])\
                        + self.kd[5] * (desired_state["yaw_dot"] - self.drone.angular_vel[2])
        
        # Limit outputs
        max_force = self.drone.maxF
        self.drone.control_input[0] = np.clip(self.drone.control_input[0],   0, max_force)
        self.drone.control_input[1] = np.clip(self.drone.control_input[1], - max_force * self.drone.arm_length, max_force * self.drone.arm_length)
        self.drone.control_input[2] = np.clip(self.drone.control_input[2], - max_force * self.drone.arm_length, max_force * self.drone.arm_length)
        self.drone.control_input[3] = np.clip(self.drone.control_input[3], - max_force * self.drone.arm_length, max_force * self.drone.arm_length)
        
        # Update error and desired state logs:
        self.desired_log.append(np.hstack([desired_state["pos"],
                                            desired_state["vel"],
                                            ddxd,ddyd,desired_state["acc"][2],
                                            phid,thed,desired_state["yaw"]])) # 11 x 1
        self.input_log.append(self.drone.control_input)    # 4 x 1 
        self.error_log.append(np.hstack((self.ep, self.ed))) # 6 x 1 
        self.update_control_input()
        
    def update_control_input(self) -> None:
        """
        Update control input.

        """
        # UPDATING DEPENDED STATES
        self.drone.thrust = self.drone.control_input[0]
        self.drone.torques = self.drone.control_input[1:]
        print(self.drone.control_input[0])

