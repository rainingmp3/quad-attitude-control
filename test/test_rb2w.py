import numpy as np
from  ..src.drone.rotation import RotationMatrices

def test_rb2w():
    euler_angles = np.radians([1.4,1.11,0.56])       

    rotation = RotationMatrices()
    r_b2w = rotation(euler_angles)
    print(r_b2w.T @ r_b2w)

    assert np.allclose(r_b2w.T @  r_b2w, np.eye(3))
