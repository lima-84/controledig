# phiT = lambda t: phi(t).reshape(-1)

# sampled_n = np.arange(nPoles, samples-1)

# Y = y[sampled_n]
# PHI = np.array( [phiT(n) for n in sampled_n])


# Modelo da planta:
#  z
# -----:
# (z-0.5)(z-0.3)
# Controlador PID
import numpy as np
import matplotlib.pyplot as plt
from control.matlab import *
from numpy.linalg import inv
import argparse

from show_sample import load_sample


def parseArgs():
    # Parse arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs='+',
                    help="path to sample to use")
    ap.add_argument("-t", "--ts", required=False, default=100, type=float,
                    help="sample time in us")
    args = vars(ap.parse_args())
    return args


def MQ_sys(nPoles, nZeros, y, u, samples):

    def phi(t): return np.vstack((-1*y[t-nPoles:t].reshape(nPoles, 1),
                                  u[t-nPoles:t+nZeros-nPoles+1].reshape(1+nZeros, 1)))

    def phiT(t): return phi(t).reshape(-1)

    sampled_n = np.arange(nPoles, samples-1)

    Y = y[sampled_n]
    PHI = np.array([phiT(n) for n in sampled_n])

    theta = solve_MQ(PHI, Y)

    theta = theta.reshape(-1)
    denId, numId = np.append(1, theta[0:nPoles][::-1]), theta[nPoles:][::-1]

    return denId, numId


def solve_MQ(PHI, Y):
    return inv(PHI.T@PHI)@PHI.T@Y


def test_MQ():
    args = parseArgs()
    t_exp, y_exp, u_exp = load_sample(args['paths'][0])

    denId, numId = MQ_sys(nPoles=3, nZeros=2,
                          y=y_exp, u=u_exp,
                          samples=t_exp.shape[0])
    sysId = tf(numId, denId, 100e-6)

    yid, _, _ = lsim(sysId, u_exp, t_exp)

    print(sysId)
    print(pole(sysId))
    print(zero(sysId))
    plt.title('Sistema Gerado: Comparação')
    plt.plot(t_exp, yid, label="identificado")
    plt.plot(t_exp, y_exp, label="experimento")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    # ---------- VDFT ----------
    test_MQ()
    quit()

    args = parseArgs()
    t, y, u = load_sample(args['paths'][0])

    Iz = tf([1, 0], [1, -1], 100e-6)
    Dz = tf([1, -1], [1, 0], 100e-6)

    P = -y
    I, _, _ = lsim(Iz, -y, t)
    D, _, _ = lsim(Dz, -y, t)

    sampled_n = np.arange(0, t.shape[0])

    def phiT(t): return np.array([P[t], I[t], D[t]])
    PHI = np.array([phiT(n) for n in sampled_n])

    Y = y[sampled_n]
    theta = solve_MQ(PHI, Y)
