import numpy as np
import matplotlib.pyplot as plt
from control.matlab import *

import argparse


def parseArgs():
    # Parse arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs = '+',
                    help="path to the sample to plot")
    ap.add_argument("-b", type=int, default = 0, required=False,
                    help="first sample to show")
    ap.add_argument("-e", type=int, default = -1, required=False,
                    help="last sample to show")
    ap.add_argument("--output",
                    help="path to output figure")
    ap.add_argument("--title",
                    help="title of the figure")
    ap.add_argument("-t", "--ts", required=False, default=1000, type=float,
                    help="sample time in us")
    args = vars(ap.parse_args())
    return args

def load_sample(path, ts = 1000):
    """
    returns t, y and u from sample in path
    ts is given in us
    """
    with open(path) as f:
        samples, inputs = f.read().split("inputs\n")
        y = np.array([float(line)*5.0/1023.0 for line in samples.splitlines()])
        u = np.array([float(line)*5.0/255.0 for line in inputs.splitlines()])
        u = np.repeat(u, y.shape[0]/u.shape[0])
        t = np.arange(y.shape[0])*ts*1e-6
        return t, y, u


if __name__ == "__main__":
    args = parseArgs()

    b = args['b']
    e = args['e']

    for path in args['paths']:
        fig, ax = plt.subplots()
        t, y, u = load_sample(path, ts=args['ts'])

        ax.plot(t[b:e],y[b:e], label='y[n]')
        ax.plot(t[b:e],u[b:e], label='v[n]')
        plt.xlabel("n")
        ax.legend()
        fig.show()
    if args['title']:
        plt.title(args['title'])
    if args['output']:
        plt.savefig(args['output'])
    else:
        plt.show()
