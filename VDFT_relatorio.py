"""
Implementação do método VRFT em experimentos realizados para uma
planta de terceira ordem RC.
"""
import numpy as np
import matplotlib.pyplot as plt
from control.matlab import *
from numpy.linalg import inv
import argparse

from show_sample import load_sample


def parseArgs():
    """
    realiza o parse de argumentos da linha de comando
    (qual amostra utilizar, alteração do tempo de amostragem se necessário)
    """
    # Parse arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs='+',
                    help="path to sample to use")
    ap.add_argument("-t", "--ts", required=False, default=1000, type=float,
                    help="sample time in us")
    args = vars(ap.parse_args())
    return args


def solve_MQ(PHI, Y):
    """ resolve o mínimos quadrados. @ = produto de matriz"""
    return inv(PHI.T@PHI)@PHI.T@Y


def d_recurrency(y, a, K):
    """
        eq. de recorrencia para a Q da forma:
            K * (z-1)
               -------
                (z-a)²
    """
    d = []

    # Presume-se que os valores anteriores a 0 eram nulos
    d.append(y[1]/K -2*a*y[0]/K)

    for t in range(1,len(y)-1):
        d.append(y[t+1]/K -2*a*y[t]/K + a*a*y[t-1]/K + d[t-1])

    # Repete a última amostra para fins de implementação (ela será ignorada)
    d.append(d[-1])
    return np.array(d)

if __name__ == "__main__":

    args = parseArgs()

    # Carrega os dados da amostra desejada
    t, y, u = load_sample(args['paths'][0])

    # Funções de transferencia para P, I e D
    # na biblioteca python-control, o argumento ts é necessário
    Pz = tf([1],[1],1e-6*args['ts'])
    Iz = tf([1, 0], [1, -1], 1e-6*args['ts'])
    Dz = tf([1, -1], [1, 0], 1e-6*args['ts'])

    # Obtém os sinais P(t), I(T), D(T)
    P, _, _ = lsim(Pz, -y, t)
    I, _, _ = lsim(Iz, -y, t)
    D, _, _ = lsim(Dz, -y, t)

    # Reformula-os em vetores (1,N)
    P = P.reshape(-1)
    I = I.reshape(-1)
    D = D.reshape(-1)

    # Define os Ns a serem amostrados (eliminando primeiro e último)
    sampled_n = np.arange(1, t.shape[0]-1)

    # phiT(T) é o vetor [P(t), I(t), D(t)]
    def phiT(t): return np.array([P[t], I[t], D[t]])
    # PHI é o vetor em que cada linha i é phiT(i)
    PHI = np.array([phiT(n) for n in sampled_n])

    # obtem a perturbação virtual por recorrência
    d = d_recurrency(y, a=0.65, K=0.25)

    # E a saída do controlador virtul
    uc = u - d.reshape(-1)

    # Gera um gráfico para confirmação visual
    plt.plot(t, u, label="u")
    plt.plot(t, d, label="d")
    plt.plot(t, uc, label="uv")
    plt.plot(t,y, label="y")
    plt.legend()
    plt.show()

    # Cada linha de Y é uc[i]
    Y = uc[sampled_n]

    # Obtém os ganhos por mínimos quadrados com Y, PHI
    kp, ki, kd = solve_MQ(PHI, Y)

    # Define a função de transferência do controlador
    z = tf([1, 0],[1], 1e-6*args['ts'])
    C = (z*z*kp + z*(ki+kd-kp) - kd)/(z*(z-1))
    print(C)
    print("Ganhos obtidos:")
    print("KP", kp)
    print("KI", ki)
    print("KD", kd)
