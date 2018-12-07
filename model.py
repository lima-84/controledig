import numpy as np
import matplotlib.pyplot as plt
from control.matlab import *


"""

 7.276e-12 s^2 + 3.725e-08 s + 1e+10
-------------------------------------
s^3 + 1.4e+04 s^2 + 3.3e+07 s + 1e+10

s = c2d(sys)

0.001197 z^2 + 0.003446 z + 0.0005958
-------------------------------------
  z^3 - 2.07 z^2 + 1.322 z - 0.2466

dt = 0.0001

pole(s)
array([0.96510374, 0.77616971, 0.32919788])
zero(s)
array([-2.69383517, -0.18471442])

"""

A = 1e3*np.array([[-2, 1, 0], [1, -2, 1], [0, 10, -10]])
B = 1e3*np.array([[1], [0], [0]])
C = np.array([0, 0, 1])

sys = ss( A, B, C, 0)

s = c2d(tf(sys), 100e-6, method='zoh')
