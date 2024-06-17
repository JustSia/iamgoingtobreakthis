## code -- old mindo device
import numpy as np
# import bluetooth
import threading
import serial
import time
import matplotlib.pyplot as plt
# from copy import deepcopy
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
# import pickle


class StreamMindo(threading.Thread):

    def __init__(self, name, port="/dev/rfcomm9", channels=8, srate=500, notch_freq=50, resolution=24, gain=1, order=10,
                 time=5):
        threading.Thread.__init__(self)
        self.port = port
        self.name = name
        self.ch = channels
        self.srate = srate
        self.order = order
        self.timepoints = int(srate * time)
        self.eeg_final = np.zeros((channels, self.timepoints))
        self.ser = serial.Serial(port, baudrate=9600, timeout=10, write_timeout=2)
        self.resolution = resolution
        self.gain = gain
        self.notch = notch_freq
        self.commands()
        # self.onepoint = int(3 * channels + 4)
        self.fb_ids = (np.arange(channels) * 3).astype(np.int32)
        self.serstatus = 0
        scaled = 100 * np.arange(channels).reshape(-1, 1)
        self.scaled = np.repeat(scaled, 100, 1)
        self.intmin = 0
        self.intmax = 15
        self.intGetcounter = 0
        atexit.register(self.stop_streaming)
        # self.float_math_pow =

    def commands(self):
        if self.resolution == 24:
            bri_r = 0
        else:
            bri_r = 1
        if self.srate == 1000:
            bri_x = 10
        elif self.srate == 500:
            bri_x = 9
        elif self.srate == 250:
            bri_x = 8
        else:
            bri_x = 9
        cmd1 = bri_r * 16 + bri_x

        if self.gain == 6:
            bri_g = 0
        elif self.gain == 8:
            bri_g = 5
        elif self.gain == 12:
            bri_g = 6
        else:
            bri_g = self.gain

        bri_y = np.log2(self.ch)
        cmd2 = bri_g * 16 + bri_y

        self.start_cmd = np.array([254, 7, cmd1, cmd2, 255]).astype(np.uint8)
        self.stop_cmd = np.array([254, 0, 0, cmd2, 255]).astype(np.uint8)
        if self.notch == 50:
            self.notch_cmd = np.array([254, 5, 1, 0, 255]).astype(np.uint8)
        elif self.notch == 60:
            self.notch_cmd = np.array([254, 5, 0, 0, 255]).astype(np.uint8)
        else:
            self.notch_cmd = np.array([254, 4, 0, 0, 255]).astype(np.uint8)

    def run(self):
        self.start_device()
        print("status:", self.serstatus)
        time.sleep(0.1)
        print(self.ser.isOpen())
        # for i in range(10000):  ## process for 10000 samples
        while self.serstatus == 1:
            try:
                self.get_eeg_data()
            except:
                self.stop_streaming()
                time.sleep(0.1)
                self.serstatus = 0
                # self.serstatus = 0
                # time.sleep(0.001)
                # self.stop_streaming()

    def shift_eeg(self, vector):
        self.eeg_final[:, :-1] = self.eeg_final[:, 1:]
        self.eeg_final[:, -1] = vector

    def byte_conversion(self):
        if self.resolution == 24:
            byte_conv = np.array([65536, 256, 1])
        else:
            byte_conv = np.array([65536, 256, 0])
        float_math_pow = 8388607
        facot = 2.4
        magnification = 1365
        factor = (10 ** 6) * facot / (magnification * float_math_pow)
        self.byte_conv_mat = byte_conv * factor

    def start_device(self):

        # self.ser.flushOutput()
        try:
            self.ser.write(self.start_cmd)
            time.sleep(0.1)
            self.ser.write(self.notch_cmd)
            # time.sleep(0.1)
            # self.ser.flushOutput()

            time.sleep(0.1)
            self.serstatus = 1
            # return "streaming .."
        except:
            self.ser.write(self.stop_cmd)
            self.serstatus = 0
            self.ser.close()
            print("error")
            # return "Unable to stream!"

    def checkHeader(self):
        header1 = self.ser.read(1)
        header2 = self.ser.read(1)
        intgetcounter = header1[0] - 240
        check = ((self.intmin > intgetcounter) or (self.intmax < intgetcounter)) and ((header2[0] % 16) != 8)
        if check:
            while check:
                header1 = header2
                header2 = self.ser.read(1)
                intgetcounter = header1[0] - 240
                check = ((self.intmin > intgetcounter) or (self.intmax < intgetcounter)) and ((header2[0] % 16) != 8)
        # print(intgetcounter)
        #
        return header1, header2

    def setLeadOnOFF(self):
        lead = self.ser.read(2)

    def get_eeg_data(self):
        header1, header2 = self.checkHeader()
        eegpoint = np.empty(self.ch)
        for i in range(self.ch):
            eeg_byte = self.ser.read(3)
            if eeg_byte[0] >= 128:
                eeg_value = (eeg_byte[0] - 256) * 65536 + eeg_byte[1] * 256 + eeg_byte[2]
            else:
                eeg_value = (eeg_byte[0]) * 65536 + eeg_byte[1] * 256 + eeg_byte[2]

            eeg_value = (eeg_value * 2.4) / 8388607
            eeg_value = eeg_value / 1365
            eeg_value = np.round(eeg_value * 1000000, 2)
            eegpoint[i] = eeg_value
            # print(eeg_value)
        self.setLeadOnOFF()
        self.shift_eeg(vector=eegpoint)

    def stop_streaming(self):
        self.ser.write(self.stop_cmd)
        self.ser.flushOutput()
        self.ser.flushInput()
        time.sleep(0.1)
        self.serstatus = 0
        self.ser.close()


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
    app = QtGui.QApplication([])
    win = pg.GraphicsWindow(size=(1800, 800))

    p1 = win.addPlot()
    curve_a1 = p1.plot()
    curve_a2 = p1.plot()
    curve_a3 = p1.plot()
    curve_a4 = p1.plot()

    # curve_b1 = p2.plot()
    # curve_b2 = p2.plot()
    # curve_b3 = p2.plot()
    # curve_b4 = p2.plot()

    mindo = StreamMindo("BRA-801", port="/dev/rfcomm4", srate=500)  ## change the rfcomm port
    mindo.start()
    time.sleep(0.2)


    def update():
        # y = np.random.rand(100)
        # global p1, curve1
        global p1, asr_state, curve_a1, curve_a2, curve_a3, curve_a4
        global curve_b1, curve_b2, curve_b3, curve_b4, filt_eeg, asr_eeg
        filt_eeg = bandpass_filter_signal(mindo.eeg, fs=500)
        # ch1 = filt_eeg[7, :]
        # ch2 = filt_eeg[6, :] + 1 * 200
        ch3 = filt_eeg[1, :] + 2 * 200
        ch4 = filt_eeg[0, :] + 3 * 200
        # ch2 = mindo.eeg_final[6, :] + 150
        # curve_a1.setData(ch1)
        # curve_a2.setData(ch2)
        curve_a3.setData(ch3)
        curve_a4.setData(ch4)


    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(500)
    app.exec_()
    print("done")
