import numpy as np
import threading
import serial
import time
import matplotlib.pyplot as plt
from pylsl import StreamInlet, resolve_stream
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
from scipy import signal


class StreamMenta(threading.Thread):
    def __init__(self, stream_name, channels=64, srate=500, time=5):
        threading.Thread.__init__(self)
        self.stream_name = stream_name
        self.ch = channels
        self.srate = srate
        self.timepoints = int(srate * time)
        self.eeg_final = np.zeros((channels, self.timepoints))
        self.serstatus = 0
        scaled = 100 * np.arange(channels).reshape(-1, 1)
        self.scaled = np.repeat(scaled, time * srate, 1)
        self.stream = None
        self.collector_timeout = 25

    def get_stream(self):
        st = resolve_stream('type', 'ExG')
        for s in st:
            print(s.name())
        self.stream = resolve_stream('name', self.stream_name)
        print('EEG streams', self.stream, len(self.stream))
        self.inlet = StreamInlet(self.stream[0])
        self.serstatus = 1

    def run(self):
        self.get_stream()
        print("streaming began")
        time.sleep(0.01)
        while self.serstatus == 1:
            try:
                sample, timestamp = self.inlet.pull_chunk(timeout=self.collector_timeout, max_samples=30)
                sample = np.array(sample).transpose()
                self.shift_eeg(sample)
            except:
                self.serstatus = 0
                print("closing")
                self.inlet.close_stream()

    def shift_eeg(self, eeg_block):
        n_steps = eeg_block.shape[1]
        self.eeg_final[:, :-n_steps] = self.eeg_final[:, n_steps:]
        self.eeg_final[:, -n_steps:] = eeg_block

    def stop_streaming(self):
        self.serstatus = 0
        print("closing")
        self.inlet.close_stream()


class PlotEEG(threading.Thread):
    def __init__(self, dataobj):
        threading.Thread.__init__(self)
        self.stop_plotting = False
        self.dataobj = dataobj

    def run(self):
        while not self.stop_plotting:
            plt.plot(0.1 * self.dataobj.eeg[0, :])
            plt.ylim([-100, 100])
            plt.yticks([])
            plt.show(block=False)
            plt.pause(0.1)
            plt.clf()


def get_power_spectrum(eeg, fs=500):
    psd = np.zeros(int(fs / 2) + 1)
    for i in range(eeg.shape[0]):
        freqs, psd1 = signal.periodogram(eeg[i, :], fs, nfft=fs)
        psd += psd1
    all_psd = psd / i
    return freqs[:20], all_psd[:20]


def bandpass_filter_signal(eeg, low=1, high=30, fs=500, order=1):
    nyq = 0.5 * fs
    low_c = low / nyq
    high_c = high / nyq
    b, a = signal.butter(order, [low_c, high_c], btype='band')
    y = signal.filtfilt(b, a, eeg)
    return y


def get_fft(eeg, fs=500):
    powers = []
    for ch in range(eeg.shape[0]):
        data = eeg[ch, :]
        ps = 20 * np.log10(np.abs(np.fft.fft(data)) ** 2)
        freqs = np.fft.fftfreq(data.size, 1 / fs)
        powers.append(ps)
    return freqs, np.array(powers)


def fft_overlap(eeg, fs=500, window=500, overlap=250):
    all_powers = np.zeros((eeg.shape[0], 30))
    n_samples = eeg.shape[1]
    nframes = int(np.floor(n_samples / overlap) - 1) if overlap > 0 else int(np.floor(n_samples / window))
    for i in range(nframes):
        data = eeg[:, i * overlap: overlap * (i + 1) + overlap]
        freqs, powers = get_fft(data, fs=500)
        all_powers = all_powers + powers[:, 1:31]
    all_powers = all_powers / nframes
    return freqs[1:31], np.mean(all_powers, 0)


def get_welch(eeg, fs=500, nperseg=500, overlap=250):
    freq, psd = signal.welch(eeg, fs=fs, nperseg=nperseg, noverlap=overlap)
    return freq[1:31], np.mean(psd, 0)


def get_cca(eeg, fs=500, freqs=[8, 12, 15]):
    pass


if __name__ == '__main__':
    app = QtGui.QApplication([])
    win = pg.GraphicsWindow(size=(1800, 800))
    p1 = win.addPlot()
    curve_a1 = p1.plot()
    curve_a2 = p1.plot()
    empty = np.array([])
    mindo = StreamMenta(stream_name="Explore_849B_ExG", srate=500, channels=8)
    mindo.start()
    time.sleep(0.2)

    def update():
        global p1, curve_a1, curve_a2
        eeg = mindo.eeg_final
        ch2 = eeg[5, :] * 0.1
        ch4 = eeg[0, :] * 0.1 + 2
        curve_a1.setData(ch2)
        curve_a2.setData(ch4)

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(500)
    app.exec_()
    mindo.join()
    print("done")