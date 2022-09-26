import numpy as np
import scipy as sp
from scipy.spatial.transform import Rotation

class Autograder(object):

    def __init__(self) -> None:
        pass

    def testQ1a(self, hat, unhat):
        test_vector = np.array([1.,-2.,3.])
        if np.isclose(test_vector, unhat(hat(test_vector))).all():
            print('Passed Debug test')
        else:
            print('did not pass debug test')
    
    def testQ1b(self, rot_err):
        R1 = np.eye(3)
        R2 = Rotation.from_rotvec(np.pi/2 * np.array([0, 0, 1])).as_matrix()
        err = np.pi/2.0 * np.array([0., 0., -1])
        if np.isclose(rot_err(R1, R2), err).all():
            print('Passed Debug test')
        else:
            print('did not pass debug test')
autograder = Autograder()