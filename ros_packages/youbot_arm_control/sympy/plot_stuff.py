import matplotlib.pyplot as plt

def plot(qs,dqs,ddqs,taus,ts):

    plt.subplot("411")
    plt.ylabel('$q$')
    plt.plot(ts, qs, linewidth=0.5)
    plt.legend(["q1", "q2", "q3", "q4", "q5"], loc='upper left', shadow=True, fontsize='x-small',)
    plt.grid(True)

    plt.subplot("412")
    plt.ylabel('$\dot q$')
    plt.plot(ts, dqs, linewidth=0.5)
    plt.legend(loc='upper left', shadow=True, fontsize='x-small')
    plt.grid(True)

    plt.subplot("413")
    plt.ylabel('$\ddot q$')
    plt.plot(ts, ddqs, linewidth=0.5)
    plt.legend(loc='upper left', shadow=True, fontsize='x-small')
    plt.grid(True)

    plt.subplot("414")
    plt.ylabel('$\\tau$')
    plt.plot(ts, taus, linewidth=0.5)
    plt.legend(loc='upper left', shadow=True, fontsize='x-small')
    plt.grid(True)

    plt.show()


def plotTaus(taus1, taus2, ts1, ts2):

    plt.subplot("311")
    plt.ylabel('$tau1$')
    plt.plot(ts1, taus1, linewidth=0.5)
    plt.legend(["q1", "q2", "q3", "q4", "q5"], loc='upper left', shadow=True, fontsize='x-small',)
    plt.grid(True)

    plt.subplot("312")
    plt.ylabel('$calc_tau1$')
    plt.plot(ts2, taus2, linewidth=0.5)
    # plt.legend(["q1", "q2", "q3", "q4", "q5"], loc='upper left', shadow=True, fontsize='x-small',)
    plt.grid(True)

    plt.show()