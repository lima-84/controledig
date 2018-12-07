import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import inv
import pdb
from control.matlab import *

def rand_stable_param(size):
    return rand_in_range(-1, 1, size=size)

def rand_in_range(a, b, size=1):
    return np.random.rand(size)*(b-a) + a

def main():
    # Generate a random transfer function with this number of poles and zeros
    nPoles = 3
    nZeros = 2

    # Generate the random vectors for zpk
    poles = rand_stable_param(size=nPoles)
    zeros = rand_stable_param(size=nZeros)
    gain = rand_in_range(-1,1)
    num, den = zpk2tf(zeros, poles, gain)

    Ts = 1
    sys = tf(num, den, Ts)

    samples = 20
    T = np.arange(start=0, stop=Ts*samples, step=Ts)
    U = np.ones(samples)

    y, t, X = lsim(sys, U, T)
    plt.stem(t, y)
    plt.title('Sistema Gerado: Resposta ao Salto')
    plt.show()

    U = np.random.rand(samples)
    y, t, X = lsim(sys, U, T)
    plt.stem(t, y)
    plt.title('Sistema Gerado: Experimento')
    plt.show()

    # Minquadr
    '''
        G = b0zm + b1zm-1 + ... + bm / zn + a1zn-1 + ... + an
        y[t+n] = b0u[t+m] + b1u[t+m-1] + ... bmy[t] - a1y[t+n-1] + a2[t+n-2] + ... + any[t]
        y[t] = b0u[t+m-n] + b1u[t+m-1-n] + ... bmy[t-n] - a1y[t-1] + a2[t-2] + ... + any[t-n]
    '''
    phi = lambda t: np.vstack((-1*y[t-nPoles:t].reshape(nPoles,1),
                               U[t-nPoles:t+nZeros-nPoles+1].reshape(1+nZeros,1)))
    phiT = lambda t: phi(t).reshape(-1)

    sampled_n = np.arange(nPoles, samples-1)

    Y = y[sampled_n]
    PHI = np.array( [phiT(n) for n in sampled_n])

    theta = inv(PHI.T@PHI)@PHI.T@Y

    theta=theta.reshape(-1)
    denId, numId = np.append(1,theta[0:nPoles][::-1]), theta[nPoles:][::-1]
    sysId = tf(numId, denId, 1)

    U = np.ones(samples)

    y, t, X = lsim(sys, U, T)
    plt.stem(t, y, label="orig")
    y, t, X = lsim(sysId, U, T)
    plt.stem(t, y, label="id")
    plt.title('Sistema Ident')
    plt.legend()
    plt.show()

if __name__=="__main__":
    main()
