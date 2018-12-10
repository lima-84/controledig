import numpy as np
import matplotlib.pyplot as plt
from control.matlab import *
from numpy.linalg import inv
import argparse

from VDFT import *

t_exp, y_exp, u_exp = load_sample("samples/sample_02.txt")

denId, numId = MQ_sys(nPoles=3, nZeros=2,
                      y=y_exp, u=u_exp,
                      samples = t_exp.shape[0])
sysId = tf(numId, denId, 1)

n, d = zpk2tf([1],[0.65,0.65],0.25)
Qd = tf(n, d, 1)
print(Qd)

t = np.arange(30)
yq, t = step(Qd, t)
t = np.array(t)
print(yq<0.2*max(yq))
# ys, _ = step(sysId, t_exp)

plt.title("step(Q)")
plt.plot(t, yq, label="q")
# plt.plot(t_exp, ys, label="s")
plt.show()

# yid, _, _ = lsim(sysId, u_exp, t_exp)
# plt.title('Sistema Gerado: Comparação')
# plt.plot(t_exp, yid, label="identificado")
# plt.plot(t_exp, y_exp, label="experimento")
# plt.legend()
# plt.show()

