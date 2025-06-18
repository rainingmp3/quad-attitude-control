
import numpy as np
from . import dynamics_utils as du

class Dynamics:
    """

    Dynamics class computes equations of motion.
    Represents physical plant of the drone.
    
    """
    def __init__(self, drone):            
        self.drone = drone
        self.cf = 8.5 * 10**(-6)                                    # thrust coefficient
        self.cd_prop = 0.016 #iris like                             # drag coefficient
        self.cd_drone = np.array([[0.1,   0,  0],
                                  [  0, 0.1,  0],
                                  [  0,   0, 0.3]])
        self.prop_direction = np.array([1, -1, +1, -1])
        self.drone.maxF = du.max_thrust_force(self.cf, self.drone.maxV, self.drone.kv)

        """
        Args:
            drone: Drone object to control
        """

    def eom(self) -> np.ndarray:    
        """
        eom function computes equations of motion for drone
        Returns:
            derivative_state: States derivative derivative_state (12,1)
        """
        derivatives = np.zeros(12)
        dn = self.drone
        e3 = np.array([0,0,1])
        
        thrust_force_real, torque_real = du.compute_motor_forces(dn.thrust,dn.r_b2w,self.cf,self.cd_prop,self.prop_direction,dn.motor_pos,e3, dn.torques, dn.arm_length)
        
        # thrust_w = dn.thrust
        sum_force_w = du.compute_other_forces(dn.r_b2w,e3,dn.vel,thrust_force_real,dn.mass,dn.g,self.cd_drone)
        derivatives = du.apply_forces(sum_force_w,torque_real,dn.vel,dn.mass,dn.angular_vel,dn.inertia_matrix,dn.tr_2euler)
        return derivatives   