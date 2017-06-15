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
