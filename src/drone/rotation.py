import numpy as np

def wrap_angles(angles):
    """Periodisizes angles
    Args:
        angles (list/double)[rad]: angle to wrap around [-pi,pi] period.

    Returns:
        list/double: limiteted angle to its period.  
    """
    if isinstance(angles, (list,np.ndarray)):
        angles =  np.asarray(angles)
        return (angles + np.pi)%(2 * np.pi) - np.pi 
    else:
        try:
            return (angles + np.pi)%(2 * np.pi) - np.pi
        except:
            raise TypeError("Wrong data format")  

class RotationMatrices:
    """
    
    RotationMatrices class handles the transformation of vectors between body and inertial frames.
    Uses ZYX Euler angle convention.
    Transformation matrix handles angular velocity conversation between frames.

    """

    def __init__(self):
        """
        Initialize Rotation Matrices
        """
        
    def __call__(self,eule,transformation = False)-> np.ndarray:
        

        """
        
        Compute rotation matrix using ZYX convention.

        """
        phi, theta, psi = eule
        cos_phi = np.cos(phi)
        cos_theta = np.cos(theta)
        cos_psi = np.cos(psi)
        sin_phi = np.sin(phi)
        sin_theta = np.sin(theta)
        sin_psi =  np.sin(psi)
        sin_phi = np.sin(phi)
        tan_theta = np.tan(theta) 
        if transformation:   
            return np.array([
                [1, tan_theta * sin_phi, tan_theta * cos_phi],
                [0,             cos_phi,           - sin_phi],
                [0, sin_phi / cos_theta, cos_phi / cos_theta]])
            
        else:
            return np.array([
                [cos_psi * cos_theta, cos_psi * sin_theta * sin_phi - sin_psi * cos_phi, cos_psi * sin_theta * cos_phi + sin_psi * sin_phi],
                [sin_psi * cos_theta, sin_psi * sin_theta * sin_phi + cos_psi * cos_phi, sin_psi * sin_theta * cos_phi - cos_psi * sin_phi],
                [         -sin_theta,                              cos_theta * sin_phi ,                               cos_theta * cos_phi]])
