import matplotlib.pyplot as plt


def plot(qs,dqs,ddqs,taus,ts):
    f, (row1, row2, row3, row4) = plt.subplots(4, 5, sharex='col', sharey='row')
    for i in range(5):
        row1[i].plot(ts, [row[i] for row in qs], linewidth=0.5, color='b')
        row2[i].plot(ts, [row[i] for row in dqs], linewidth=0.5, color='r')
        row3[i].plot(ts, [row[i] for row in ddqs], linewidth=0.5, color='g')
        row4[i].plot(ts, [row[i] for row in taus], linewidth=0.5, color='g')
        plt.grid(True)
    plt.subplots_adjust(top=0.95, bottom=0.05, left=0.05, right=0.95, hspace=0.10,
                        wspace=0.20)
    plt.show()


def plotTaus(taus1, taus2, ts1, ts2):
    f, (row1, row2) = plt.subplots(2, 5, sharex='col', sharey='row')
    for i in range(5):
        row1[i].plot(ts1, [row[i] for row in taus1], linewidth=0.5, color='b')
        row2[i].plot(ts2, [row[i] for row in taus2], linewidth=0.5, color='r')
        plt.grid(True)
    plt.subplots_adjust(top=0.95, bottom=0.05, left=0.05, right=0.95, hspace=0.10,
                        wspace=0.20)
    plt.show()


def plotTaus2(taus1, taus2, ts1, ts2):

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