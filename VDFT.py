"""
Implementa VDFT para a planta RC 3a ordem


"""
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
    ap.add_argument("-t", "--ts", required=False, default=1000, type=float,
                    help="sample time in us")
    args = vars(ap.parse_args())
    return args


def MQ_sys(nPoles, nZeros, y, u, samples):
    """
        Encontra um sistema por mínimos quadrados
    """

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
    """ resolve o mínimos quadrados. @ = produto de matriz"""
    return inv(PHI.T@PHI)@PHI.T@Y


def test_MQ():
    """ Para testar se meu MQ tá funcionado"""
    args = parseArgs()
    t_exp, y_exp, u_exp = load_sample(args['paths'][0])

    denId, numId = MQ_sys(nPoles=3, nZeros=2,
                          y=y_exp, u=u_exp,
                          samples=t_exp.shape[0])
    sysId = tf(numId, denId, 1e-6*args['ts'])

    yid, _, _ = lsim(sysId, u_exp, t_exp)

    print(sysId)
    print("DC gain:", dcgain(sysId))
    plt.title('Sistema Gerado: Comparação')
    plt.plot(t_exp[:-1], yid, label="identificado")
    plt.plot(t_exp, y_exp, label="experimento")
    plt.legend()
    plt.show()

    # Fiz um controle aqui pq tava no desespero
    print("poles:", pole(sysId))
    print("zeros:",zero(sysId))
    czeros =  np.sort(pole(sysId))[-2:]
    print("control zeros:", czeros)
    cpoles = np.array([1,0])
    num, den = zpk2tf(czeros,cpoles,1)
    C = tf(num, den, 1e-6*args['ts'])
    print(C)

    T = (sysId)/(1+sysId*C)
    y_control, _ = step(T, t_exp)
    print("poles of T:",pole(T))
    print("zeros of T:",zero(T))
    print(T)
    plt.title('sistema em MF')
    plt.plot(t_exp[:-1], y_control, label="mf")
    plt.legend()
    plt.show()



def d_recurrency(y, a, K):
    """
        eq. de recorrencia para a Q da forma:
            K * (z-1)
               -------
                (z-a)²
    """
    d = []

    d.append(y[1]/K -2*a*y[0]/K)

    for t in range(1,len(y)-1):
        d.append(y[t+1]/K -2*a*y[t]/K + a*a*y[t-1]/K + d[t-1])

    # Bota a última amostra duas vezes pra fechar o tamanho
    d.append(d[-1])
    return np.array(d)

if __name__ == "__main__":
    # ---------- VDFT ----------

    # test_MQ()
    # quit()

    args = parseArgs()

    # Processa as amostras (fiz um resample pq tava dando merda)
    t, y, u = load_sample(args['paths'][0])

    # Funções de transferencia do P, do I e do D
    Pz = tf([1],[1],1e-6*args['ts'])
    Iz = tf([1, 0], [1, -1], 1e-6*args['ts'])
    Dz = tf([1, -1], [1, 0], 1e-6*args['ts'])


    print(Iz)
    print(Dz)
    P, _, _ = lsim(Pz, -y, t)
    I, _, _ = lsim(Iz, -y, t)
    D, _, _ = lsim(Dz, -y, t)

    # Pra virarem vetores
    P = P.reshape(-1)
    I = I.reshape(-1)
    D = D.reshape(-1)

    # Define os Ns a serem amostrados (come o primeiro e o último)
    sampled_n = np.arange(1, t.shape[0]-1)

    # phiT(T) é o vetor [P, I, D]
    def phiT(t): return np.array([P[t], I[t], D[t]])
    PHI = np.array([phiT(n) for n in sampled_n])

    # Tava tentando simular a Q pelo lsim...
    # n, d = zpk2tf([1, -2, -0.9],[0.8, 0.8, -0.6],0.5)
    # Qd = tf(n, d, 1e-6*args['ts'])
    # d, _, _ = lsim(1/Qd, y, t)

    # D pela recorrencia 
    # d = d_recurrency(y, a=0.85, K=0.3)
    d = d_recurrency(y, a=0.8, K=0.26)

    # Uc = u controlador
    uc = u - d.reshape(-1)

    # Plota pra ver se tá show
    plt.plot(t, u, label="u")
    plt.plot(t, d, label="d")
    plt.plot(t, uc, label="uv")
    plt.plot(t,y, label="y")

    plt.legend()
    plt.show()


    # Y é o uc[t] pra cada t amostrado
    Y = uc[sampled_n]
    kp, ki, kd = solve_MQ(PHI, Y)

    z = tf([1, 0],[1], 1e-6*args['ts'])
    C = (z*z*kp + z*(ki+kd-kp) - kd)/(z*(z-1))

    print(np.linalg.cond(PHI.T@PHI))
    print(zero(z*z*kp + z*(ki+kd-kp) - kd))
    print (C)


    # Encontra o modelo do sistema e simula em MF
    # pra ver se deu boa
    print("PID:",kp,ki,kd)
    denId, numId = MQ_sys(nPoles=3, nZeros=2,
                          y=y, u=u,
                          samples=t.shape[0])
    G = tf(numId, denId, 1e-6*args['ts'])

    T = G/(1+C*G)

    # yprev, _, _ = step(T, d, t)
    yprev, _ = step(T, t)

    plt.plot(t[:-1], yprev, label="y previsto")
    # plt.plot(t,y, label="y")

    plt.legend()
    plt.show()
