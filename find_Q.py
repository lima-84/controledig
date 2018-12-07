import numpy as np
import matplotlib.pyplot as plt
from control.matlab import *
from numpy.linalg import inv
import argparse

from VDFT import *

t_exp, y_exp, u_exp = load_sample("samples/sample_2.txt")

denId, numId = MQ_sys(nPoles=3, nZeros=2,
                      y=y_exp, u=u_exp,
                      samples = t_exp.shape[0])
sysId = tf(numId, denId, 100e-6)

yid, _, _ = lsim(sysId, u_exp, t_exp)
plt.title('Sistema Gerado: Comparação')
plt.plot(t_exp, yid, label="identificado")
plt.plot(t_exp, y_exp, label="experimento")
plt.legend()
plt.show()

