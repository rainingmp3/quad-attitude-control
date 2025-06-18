import numpy as np
from .rotation import RotationMatrices
from .dynamics import Dynamics
from src.drone.mass_handle import com_shift


class Drone:
    '''
    
    Drone class  handles state evolution over simulation and logging. 

    '''
    def __init__(self, 
                 params: dict[str, float],
                 initStates: list, 
                 initInputs:list, 
                 gravity: float = 9.81,
                 time_step: float  = 0.05):

        
        # Physics constants
        self.g = gravity                          
        
        # Geometric & ineretial parameters
        self.mass = params['mass']             
        self.attached_mass = params['attMass']
        self.cube_height = params['cubeHeight']    
        self.arm_length = params['armLength']           
        self.arm_height = params['rotorHeight']
        self.inertia_matrix    = np.array([[params['Ixx'],            0,             0],      
                                           [0            ,params['Iyy'],             0],
                                           [0            ,            0, params['Izz']]])
        
        delta_com, total_mass, new_inertia = com_shift(self.mass,self.attached_mass,self.cube_height/2, self.inertia_matrix)
        self.arm_length += delta_com
        self.mass = total_mass
        self.inertia_matrix = new_inertia

        # Rotor positions in body frame
        self.half = np.cos(45/180*np.pi)*self.arm_length
        self.motor_pos = np.array([[ self.half,  self.half, self.arm_height],
                                   [ self.half, -self.half, self.arm_height],
                                   [-self.half, -self.half, self.arm_height],
                                   [-self.half,  self.half, self.arm_height]])
        self.maxV = params["maxVoltage"]
        self.kv = params["Kv"]
        self.maxF = 0
        
        # Time
        self.time_step = time_step                                
        self.time_elapsed = 0                                     
        
        # Drone state
        self.state = np.array(initStates, dtype=float).copy()  # STATE VECTOR 
        self.derivative_state = np.zeros(12)
        self.pos = self.state[:3]                                # POSITION VECTOR [m]
        self.vel = self.state[3:6]                              # VELOCITY VECTOR [m/s]
        self.euler_angles = (np.radians(self.state[6:9]))                # EULER ANGLES [rad]
        self.angular_vel = (np.radians(self.state[9:12]))                  # ANGULAR VELOCITIES [rad/s]

        # Classes Initialization
        self.rotation = RotationMatrices()
        self.dynamics = Dynamics(self)
        
        # Rotation handling
        self.r_b2w = self.rotation(self.euler_angles)
        self.tr_2euler = self.rotation(self.euler_angles, transformation=True)
        
        # Control inputs
        self.control_input = np.array(initInputs, dtype=float).copy()       # CONTROL INPUT 
        self.thrust = self.control_input[0]                                 # THRUST [N]
        self.torques = self.control_input[1:]                                # TORQUE [N*m]
        
        # Logs
        self.time_log = []
        self.states_log = []
        self.rotation_matrix_log = []

        
    def update_states(self,derivative_state) -> None:
        """
        Update current states based on equation of motions.
        """
        self.state += derivative_state * self.time_step
        self.time_elapsed += self.time_step 
        
    def log_states(self) -> None:
        """
        Log current states for plotting and animation.
        """
        self.time_log.append(self.time_elapsed)
        self.states_log.append(np.copy(self.state))
        self.rotation_matrix_log.append(self.r_b2w)
      
        # UPDATING DEPENDED STATES
        self.pos = self.state[:3]                               
        self.vel = self.state[3:6]                             
        self.euler_angles =  self.state[6:9]            
        self.angular_vel = self.state[9:12]
        self.r_b2w = self.rotation(self.euler_angles)
        self.tr_2euler = self.rotation(self.euler_angles,transformation=True)


    def update(self) -> None:
        """
        Update and log states.
        """
        self.derivative_state = self.dynamics.eom()
        self.update_states(self.derivative_state)
        self.log_states()
        # print(self.derivative_state)