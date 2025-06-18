import numpy as np

def compute_motor_forces(thrust, r_b2w, cf, cd_prop, prop_dir, motor_pos, e3, torques, arm):
    """Compute motor forces in body frame"""

    thrust_total = thrust             
    a = arm / np.sqrt(2)         

    alpha = torques[0] / a          
    beta  = torques[1] / a         

    thrust0 =  thrust_total/4.0 +  alpha/4.0 -  beta/4.0
    thrust1 =  thrust_total/4.0 -  alpha/4.0 -  beta/4.0
    thrust2 =  thrust_total/4.0 -  alpha/4.0 +  beta/4.0
    thrust3 =  thrust_total/4.0 +  alpha/4.0 +  beta/4.0

    thrust_arr = np.array([thrust0, thrust1, thrust2, thrust3])
    thrust_arr = np.clip(thrust_arr, 0.0, None)   
    thrust_body = np.array([0.0, 0.0, thrust0 + thrust1 + thrust2 + thrust3])
    wsq_arr = thrust_arr/cf

    moment_from_thrust_0 = prop_dir[0] * cd_prop * wsq_arr[0]   + np.cross(motor_pos[0], np.array([0.0, 0.0, thrust0]))
    moment_from_thrust_1 = prop_dir[1] * cd_prop * wsq_arr[1]  + np.cross(motor_pos[1], np.array([0.0, 0.0, thrust1]))
    moment_from_thrust_2 = prop_dir[2] * cd_prop * wsq_arr[2]  + np.cross(motor_pos[2], np.array([0.0, 0.0, thrust2]))
    moment_from_thrust_3 = prop_dir[3] * cd_prop * wsq_arr[3] + np.cross(motor_pos[3], np.array([0.0, 0.0, thrust3]))

    moments_body = moment_from_thrust_0 \
                     + moment_from_thrust_1 \
                     + moment_from_thrust_2 \
                     + moment_from_thrust_3
    return thrust_body, moments_body
    
def compute_other_forces(r_b2w,e3,v_w,thrust,mass,g,cd_drone):
    """Gravity + drag + thrust in world frame"""
    # sum_thrust = np.sum(thrust)
    thrust_force = r_b2w @ thrust 
    v_b = r_b2w.T @ v_w
    drag_force =  - r_b2w @ (cd_drone @ v_b)
    grav_force = - mass * g * e3 
    # print(thrust_force)
    total_force = drag_force + grav_force + thrust_force
    return total_force

def apply_forces(sum_force_w,moments,v_w,mass,angular_vel,inertia_matrix,tr_2euler):
    """Update derivative state metrice derivative_state"""
    derivative_state =np.zeros(12)
    derivative_state[:3] = v_w
    derivative_state[3:6] = sum_force_w / mass
    derivative_state[6:9] = tr_2euler @ angular_vel # confusing 
    derivative_state[9:12] = np.linalg.inv(inertia_matrix) @ (moments + np.cross(-angular_vel, inertia_matrix @ angular_vel))
    return derivative_state
    
def max_thrust_force(cf,maxV,kv):
    maxF = 4 * cf * (maxV * kv * 2 * np.pi /60) **2
    return maxF




