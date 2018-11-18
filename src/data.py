from matplotlib import collections as mc, pyplot as plt
import pylab as pl
import numpy as np

if __name__ == '__main__':

    time_GRRT = []
    num_samples_GRRT = []
    cost_GRRT = []
    first = True

    with open('GRRT.txt', 'rU') as f:
        for line in f:
            if first:
                first = not first
                continue
            cur_line = line.split()
            time_GRRT.append(float(cur_line[1]))
            num_samples_GRRT.append(int(cur_line[0]))
            cost_GRRT.append(float(cur_line[2]))

    mu_GRRT, std_GRRT = str(round(np.mean(cost_GRRT),3)), str(round(np.std(cost_GRRT),3))
    mu_GRRT1, std_GRRT1 = str(round(np.mean(time_GRRT),3)), str(round(np.std(time_GRRT),3))
    mu_GRRT2, std_GRRT2 = str(round(np.mean(num_samples_GRRT),3)), str(round(np.std(num_samples_GRRT),3))

    time_RRT = []
    num_samples_RRT = []
    cost_RRT = []
    first = True

    with open('RRT.txt', 'rU') as f:
        for line in f:
            if first:
                first = not first
                continue
            cur_line = line.split()
            time_RRT.append(float(cur_line[1]))
            num_samples_RRT.append(int(cur_line[0]))
            cost_RRT.append(float(cur_line[2]))


    mu_RRT, std_RRT = str(round(np.mean(cost_RRT),3)), str(round(np.std(cost_RRT),3))
    mu_RRT1, std_RRT1 = str(round(np.mean(time_RRT),3)), str(round(np.std(time_RRT),3))
    mu_RRT2, std_RRT2 =  str(round(np.mean(num_samples_RRT),3)), str(round(np.std(num_samples_RRT),3))

    lbl_G =  'GRRT \ncost: ${\mu}$=' + mu_GRRT + ' ${\sigma}$='+std_GRRT + '\ntime: ${\mu}$=' + mu_GRRT1 + ' ${\sigma}$='+std_GRRT1 +'\nsamples: ${\mu}$=' + mu_GRRT2 + ' ${\sigma}$='+std_GRRT2
    lbl = 'RRT \ncost: ${\mu}$=' + mu_RRT + ' ${\sigma}$='+std_RRT +'\ntime: ${\mu}$=' + mu_RRT1 + ' ${\sigma}$='+std_RRT1 + '\nsamples: ${\mu}$=' + mu_RRT2 + ' ${\sigma}$='+std_RRT2

    fig = plt.figure()
    plt.scatter(time_GRRT, cost_GRRT, color='green', label=lbl_G)
    plt.scatter(time_RRT, cost_RRT, color='blue', label=lbl)
    plt.title('RRT vs. GRRT')
    ax = plt.subplot(111)
    ax.legend(loc='lower right')
    plt.xlabel('Time in Seconds')
    plt.ylabel('Path Cost')
    fig.tight_layout()

    fig1 = plt.figure()
    plt.scatter(num_samples_GRRT, cost_GRRT, color='green', label=lbl_G)
    plt.scatter(num_samples_RRT, cost_RRT, color='blue', label=lbl)
    plt.title('RRT vs. GRRT')
    ax = plt.subplot(111)
    ax.legend(loc='lower right')
    plt.xlabel('Number of Samples for First Solution')
    plt.ylabel('Path Cost')
    fig1.tight_layout()

    plt.show()
