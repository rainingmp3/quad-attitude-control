import numpy as np

def com_shift(drone_m, att_m, distance, drone_inertia):
    """Attaching a cube, rigidly"""
    total_mass = (drone_m + att_m)
    delta_com = att_m * (distance + 0.02) / total_mass
    # Cube inertia 
    # as distance is defined tobe one to COM of attacment, hence total cube height is double the distance
    distance_vector_att = np.array([0,0,distance + 0.02 - delta_com])
    
    parallel_axis_input_att = att_m * ((np.dot(distance_vector_att.T, distance_vector_att) * np.eye(3)) - np.dot(distance_vector_att, distance_vector_att.T))
    
    delta_com_vector = np.array([0,0,-delta_com])
    parallel_axis_input_drone= drone_m * ((np.dot(delta_com_vector.T, delta_com_vector) * np.eye(3)) - np.dot(delta_com_vector, delta_com_vector.T))
    
    
    new_inertia = drone_inertia + parallel_axis_input_drone + 1/6 * att_m * (distance*2)**2 *np.eye(3)  + parallel_axis_input_att
    return delta_com, total_mass, new_inertia


# print(com_shift(1.25, 1, 0.05,np.array([[0.23,0,0],
#                                       [0,0.23,0],
#                                       [0,0,0.047]])))