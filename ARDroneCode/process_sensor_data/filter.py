import csv, os
from collections import defaultdict
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt


def main(args):
    data = defaultdict(list)
    reader = csv.DictReader(open(args[1], 'r'))
    for row in reader:
        for k, v in row.items():
            data[k].append(float(v))

    if args[2] not in data: return

    cutoff_freq = 0.15
    samp_rate = 50.0

    x = np.array(data[args[2]])
    nsamps = len(x)
    time = nsamps / samp_rate
    t = np.linspace(0, time, nsamps)
    fft_freqs = np.fft.fftfreq(nsamps, 1.0 / samp_rate)

    norm_pass = 2 * np.pi * cutoff_freq / samp_rate
    norm_stop = 1.5 * norm_pass
    print norm_pass, norm_stop
    N, Wn = signal.buttord(wp=norm_pass, ws=norm_stop, gpass=2, gstop=30, analog=0)
    b, a = signal.butter(N, Wn, btype='low', analog=0, output='ba')
    y = signal.lfilter(b, a, x) * 1e3
    sp_x = np.fft.fft(x)
    magnitudes_x = np.abs(sp_x)
    sp_y = np.fft.fft(y)
    magnitudes_y = np.abs(sp_y)

    plt.plot(t, x)
    plt.savefig('x.png')
    plt.clf()
    plt.plot(t, y)
    plt.savefig('y.png')
    plt.clf()
    plt.loglog(fft_freqs[:nsamps/2], magnitudes_x[:nsamps/2])
    plt.savefig('x_freq.png')
    plt.clf()
    plt.loglog(fft_freqs[:nsamps/2], magnitudes_y[:nsamps/2])
    plt.savefig('y_freq.png')


if __name__ == '__main__':
    import sys
    main(sys.argv)

