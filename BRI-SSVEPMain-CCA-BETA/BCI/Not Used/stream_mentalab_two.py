## working code -- stable
import numpy as np
# import bluetooth
import threading
import serial
import time
import matplotlib.pyplot as plt
from pylsl import StreamInlet, resolve_stream, StreamOutlet
from copy import deepcopy
# import matplotlib
import atexit
# # matplotlib.use("GTK3Agg")
# from matplotlib.animation import FuncAnimation
from pyqtgraph.Qt import QtGui, QtCore
# # import numpy as np
import pyqtgraph as pg
# from multiprocessing import Process, Manager, Queue
from scipy import signal
# from asr_process import ASR
import pickle


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
        # atexit.register(self.stop_streaming)
        # self.float_math_pow =

    def get_stream(self):
        st = resolve_stream('type', 'ExG')
        # print("list:", list(st))
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
            # for i in range(1000):
            try:
                sample, timestamp = self.inlet.pull_chunk(timeout=self.collector_timeout, max_samples=30)
                sample = np.array(sample).transpose()  # Channel X timepoints
                # print(sample.shape)
                self.shift_eeg(sample)
            except:
                # self.stream
                self.serstatus = 0
                print("closing")
                # self.inlet.close_stream()
                self.inlet.close_stream()
            # time.sleep(1/self.srate)

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
        # plt.figure(1)
        self.stop_plotting = False
        self.dataobj = dataobj
        # self.ax = ax

    def run(self):
        while not self.stop_plotting:
            plt.plot(0.1 * self.dataobj.eeg[0, :])
            plt.ylim([-100, 100])
            plt.yticks([])
            plt.show(block=False)
            plt.pause(0.1)
            # time.sleep(0.01)
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
        # idx = np.argsort(freqs)
        powers.append(ps)
    # all_powers = np.mean(np.array(powers), 0)
    return freqs, np.array(powers)
    # return freqs[:20], all_powers


def fft_overlap(eeg, fs=500, window=500, overlap=250):
    all_powers = np.zeros((eeg.shape[0], 30))
    n_samples = eeg.shape[1]
    if overlap > 0:
        nframes = int(np.floor(n_samples / overlap) - 1)
    else:
        nframes = int(np.floor(n_samples / window))

    for i in range(nframes):
        data = eeg[:, i * overlap: overlap * (i + 1) + overlap]
        # print(i*overlap, overlap*(i+1)+overlap)
        freqs, powers = get_fft(data, fs=500)

        all_powers = all_powers + powers[:, 1:31]
    all_powers = all_powers / nframes
    # print(nframes)
    return freqs[1:31], np.mean(all_powers, 0)


def get_welch(eeg, fs=500, nperseg=500, overlap=250):
    freq, psd = signal.welch(eeg, fs=fs, nperseg=nperseg, noverlap=overlap)
    return freq[1:31], np.mean(psd, 0)


def get_cca(eeg, fs=500, freqs=[8, 12, 15]):
    pass


if __name__ == '__main__':
    # data = np.load("calib_data.npy")[:, :2500]
    # print(data.shape)
    # asr_state = pickle.load(open("br8_state.p", "rb"))
    # print(asr_state.keys())
    # asr_pro = ASR(srate=250)
    # data2, asr_state2 = asr_pro.asr_process(data, asr_state)
    app = QtGui.QApplication([])
    win = pg.GraphicsWindow(size=(1800, 800))

    p1 = win.addPlot()
    # p2 = win.addPlot()
    # p1.setRange(yRange=[-2000, -1000])
    # p2.setRange(yRange=[-200, 1000])
    curve_a1 = p1.plot()
    curve_a2 = p1.plot()
    empty = np.array([])

    # curves = [pg.PlotCurveItem(x=empty, y=empty, autoDownsample=True) for _ in range(8)]
    # for curve in curves:
    #     p1.addItem(curve)
    # curve_a3 = p1.plot()
    # curve_a4 = p1.plot()

    # curve_b1 = p2.plot()
    # curve_b2 = p2.plot()
    # curve_b3 = p2.plot()
    # curve_b4 = p2.plot()

    mindo = StreamMenta(stream_name="Explore_849B_ExG", srate=500, channels=8)
    # mindo = StreamLiveAmp(stream_name="BASIL_Stream", srate=500, channels=8)
    mindo.start()
    time.sleep(0.2)


    def update():
        # y = np.random.rand(100)
        # global p1, curve1
        global p1, asr_state, curve_a1, curve_a2, curve_a3, curve_a4
        global curve_b1, curve_b2, curve_b3, curve_b4, filt_eeg, asr_eeg
        # filt_eeg = bandpass_filter_signal(mindo.eeg_final, fs=50)
        # filt_eeg = mindo.eeg_final
        # print(filt_eeg[0,-10:])
        # asr_eeg, asr_state = asr_pro.asr_process(deepcopy(filt_eeg), asr_state)
        # asr_eeg, asr_state = asr_pro.asr_process(mindo.eeg_final, asr_state)
        # print(asr_eeg[0,-10:])
        # asr_eeg = asr_eeg
        # asr_eeg = asr_eeg
        # eeg = mindo.eeg_final + 0.5*mindo.scaled
        eeg = mindo.eeg_final
        # eeg = asr_eeg[[1,2]
        # start = time.time()
        # freq,psd = fft_overlap(eeg,fs=500)
        # freq,psd = get_welch(eeg)
        # end = time.time()
        # print("time:", end-start)
        # if np.inf in np.abs(psd):
        #     freq = np.arange(20)
        #     psd = np.zeros(20)
        # print(psd[:10])

        # curve_a1.setData(freq[:20],psd[:20])
        # ch1 = mindo.eeg_final[1, :]+200

        ch2 = eeg[5, :] * 0.1
        # ch2 = filt_eeg[6, :] + 1 * 200
        # ch3 = filt_eeg[1, :] + 2 * 200
        ch4 = eeg[0, :] * 0.1 + 2
        # ch2 = mindo.eeg_final[6, :] + 150
        curve_a1.setData(ch2)
        curve_a2.setData(ch4)
        # curve.setData(eeg)
        # for i in range(8):
        #     curves[i].setData(eeg[i, :])
        # curve_a3.setData(ch3)
        # curve_a4.setData(ch4)

        # ch1 = asr_eeg[7, :]
        # ch2 = asr_eeg[6, :] + 1 * 200
        # ch3 = asr_eeg[1, :] + 2 * 200
        # ch4 = asr_eeg[0, :] + 3 * 200
        # ch2 = mindo.eeg_final[6, :] + 150
        # curve_b1.setData(ch1)
        # curve_b2.setData(ch2)
        # curve_b3.setData(ch3)
        # curve_b4.setData(ch4)


    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(500)
    app.exec_()
    # print(mindo.serstatus)
    # while mindo.serstatus == 1:
    #
    #     k = 0
    #     for c in range(8):
    #         if (c < 2) or (c > 5):
    #             plt.plot(mindo.eeg_final[c, :] + (4 - k) * 100 * np.ones(mindo.eeg_final.shape[1]))
    #             k = k + 1
    #
    #     # plt.plot(mindo.eeg[1, :] + 200*np.ones(1500))
    #     # plt.plot(mindo.eeg[6, :] + 300 * np.ones(1500))
    #     plt.ylim([-100, 600])
    #     plt.yticks([])
    #     # plt.xticks([])
    #     plt.show(block=False)
    #     plt.pause(0.1)
    #     plt.clf()
    # # plotter.start()
    mindo.join()
    # print(filt_eeg[0, :100])
    # print(asr_eeg[0, :100])
    # np.save("sample_asr.npy",mindo.eeg_final)
    # time.sleep(0.1)
    # mindo.stop_streaming()
    # # plotter.stop_plotting = True
    print("done")
