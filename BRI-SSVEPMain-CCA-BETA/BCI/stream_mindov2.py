## working code -- stable
import numpy as np
# import bluetooth
import threading
import serial
import time
import matplotlib.pyplot as plt
from copy import deepcopy
# import matplotlib
import atexit
# # matplotlib.use("GTK3Agg")
# from matplotlib.animation import FuncAnimation
import pyqtgraph
from pyqtgraph.Qt import QtGui, QtCore
# # import numpy as np
import pyqtgraph as pg
# from multiprocessing import Process, Manager, Queue
from scipy import signal
# from asr_process import ASR
import pickle
import os

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
        self.fb_ids = (np.arange(channels) * 3).astype(np.int32)
        self.serstatus = 0
        scaled = 100 * np.arange(channels).reshape(-1, 1)
        self.scaled = np.repeat(scaled, 100, 1)
        self.total_bytes = 35
        self.original_buffer = np.array([])
        self.byte_conv_mat = self.byte_conversion()
        # self.ser.open()
        atexit.register(self.stop_streaming)
        # self.float_math_pow =

    def byte_conversion(self):
        if self.resolution == 24:
            byte_conv = np.array([65536, 256, 1])
        else:
            byte_conv = np.array([65536, 256, 0])
        float_math_pow = 8388607
        facot = 2.4
        magnification = 1365
        factor = (10 ** 6) * facot / (magnification * float_math_pow)
        byte_conv_mat = byte_conv * factor
        byte_conv_mat = np.tile(byte_conv_mat, self.ch)
        return byte_conv_mat.reshape(1, -1)

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
        print(self.start_cmd)
        self.stop_cmd = np.array([254, 0, 0, cmd2, 255]).astype(np.uint8)
        if self.notch == 50:
            self.notch_cmd = np.array([254, 5, 1, 0, 255]).astype(np.uint8)
        elif self.notch == 60:
            self.notch_cmd = np.array([254, 5, 0, 0, 255]).astype(np.uint8)
        else:
            self.notch_cmd = np.array([254, 4, 0, 0, 255]).astype(np.uint8)

    def get_eeg(self, sam_size=int(35 * 10)):
        packet_hex = self.ser.read(sam_size)
        packet_byte = np.frombuffer(packet_hex, dtype=np.uint8)
        self.get_eeg_points(packet_byte)

    def run(self):
        self.start_device()
        print("status:", self.serstatus)
        time.sleep(0.01)
        print(self.ser.isOpen())
        # for i in range(1000):
        while self.serstatus == 1:
            try:
                self.get_eeg()
            except:
                self.stop_streaming()
                time.sleep(0.1)
                self.serstatus = 0
                # self.serstatus = 0
                # self.stop_streaming()

    def get_eeg_points(self, packet):
        data = np.concatenate((self.original_buffer, packet))
        zero_idx = np.where(data == 0)[0]
        n_zeros = len(zero_idx)
        buffer_idx = 0
        eeg_points = []
        for i in range(n_zeros):
            if (zero_idx[i] - buffer_idx == 34):
                uncob_data = data[buffer_idx:zero_idx[i]]
                after_cobs = self.uncobs_processing(uncob_data.astype(np.uint8))
                eeg_point_packet = after_cobs[6:30]
                eeg_points.append(eeg_point_packet.reshape(1, -1))
            buffer_idx = zero_idx[i] + 1
            if i == n_zeros - 1:
                self.buffer = data[zero_idx[i] + 1:]
        eeg_block = self.convert_byte_to_eeg_vector(np.vstack(eeg_points))
        self.shift_eeg(eeg_block)

    def convert_byte_to_eeg_vector(self, eeg_byte):
        eegbyte = eeg_byte.astype(np.int64)
        eegbyte[:, self.fb_ids] = (eegbyte[:, self.fb_ids] >= 128) * (eegbyte[:, self.fb_ids] - 256) + (
                eegbyte[:, self.fb_ids] < 128) * (
                                      eegbyte[:, self.fb_ids])
        conversion = np.tile(self.byte_conv_mat, (eegbyte.shape[0], 1))
        multipy_byte = np.multiply(eegbyte, conversion)
        eegblock = np.add.reduceat(multipy_byte, np.arange(0, multipy_byte.shape[1], 3), 1)
        return eegblock.transpose()

    def shift_eeg(self, eeg_block):
        n_steps = eeg_block.shape[1]
        self.eeg_final[:, :-n_steps] = self.eeg_final[:, n_steps:]
        self.eeg_final[:, -n_steps:] = eeg_block

    def uncobs_processing(self, uncobsdata):
        output = np.empty(self.total_bytes - 1).astype(np.uint8)
        end_idx = self.total_bytes - 2
        cobs_idx = -1
        dst_idx = -1
        while cobs_idx < end_idx:
            code = uncobsdata[cobs_idx + 1]
            cobs_idx += 1
            for i in range(code - 1):
                if cobs_idx >= end_idx:
                    break
                output[dst_idx + 1] = uncobsdata[cobs_idx + 1]
                dst_idx += 1
                cobs_idx += 1
            if code < 255:
                output[dst_idx + 1] = 0
                dst_idx += 1
        return output[:33]

    def convert_byte_to_eeg(self, eeg_byte):
        eeg_point = np.empty(self.ch)
        floatMathPow = 8388607
        for i in range(self.ch):
            x = eeg_byte[i * 3:(i + 1) * 3]
            if x[0] >= 128:
                point = (x[0] - 256) * 65536 + x[1] * 256 + x[2]
            else:
                point = x[0] * 65536 + x[1] * 256 + x[2]

            point = (point * 2.4) / floatMathPow
            point = point / 1365
            point = np.round(point * 1000000, 2)
            eeg_point[i] = point
        return eeg_point.reshape(-1, 1)

    def start_device(self):

        # self.ser.flushOutput()
        try:
            self.ser.write(self.notch_cmd)
            # time.sleep(0.1)
            # self.ser.flushOutput()
            time.sleep(0.1)
            self.ser.write(self.start_cmd)
            # time.sleep(0.01)
            self.serstatus = 1
            # return "streaming .."
        except:
            self.ser.write(self.stop_cmd)
            self.serstatus = 0
            self.ser.close()
            print("error")
            # return "Unable to stream!"

    def stop_streaming(self):
        # self.ser.flushOutput()
        # self.ser.flushInput()
        self.ser.write(self.stop_cmd)
        self.ser.flushOutput()
        self.ser.flushInput()
        time.sleep(0.1)
        self.serstatus = 0
        self.ser.close()


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
    # p1.setRange(yRange=[-200, 1000])
    # p2.setRange(yRange=[-200, 1000])
    curve_a1 = p1.plot()
    # curve_a2 = p1.plot()
    # curve_a3 = p1.plot()
    # curve_a4 = p1.plot()

    # curve_b1 = p2.plot()
    # curve_b2 = p2.plot()
    # curve_b3 = p2.plot()
    # curve_b4 = p2.plot()

    mindo = StreamMindo("BRA-801", port="/dev/rfcomm0", srate=500)
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
        eeg = mindo.eeg_final
        # eeg = mindo.eeg_final[[6,7], :]
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
        # ch1 = mindo.eeg_final[0, :]

        ch2 = eeg[0, :]
        # ch2 = filt_eeg[6, :] + 1 * 200
        # ch3 = filt_eeg[1, :] + 2 * 200
        ch4 = eeg[1, :] + 2 * 200
        # ch2 = mindo.eeg_final[6, :] + 150
        curve_a1.setData(ch4)
        curve_a2.setData(ch2)
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
