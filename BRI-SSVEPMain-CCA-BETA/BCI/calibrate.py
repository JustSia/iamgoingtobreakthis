import numpy as np
import sys

sys.path.append('../')
# import scipy.signal
# from OnlineSsvepRsvp.SSVEPCLF import SSVEPCLF
from scipy import signal
from sklearn.cross_decomposition import CCA
from sklearn import svm
from sklearn.neighbors import KNeighborsClassifier
from sklearn.model_selection import cross_val_score


# [7, 7.5, 8, 8.5, 9, 11]
# [7, 8, 9, 11, 7.5,8.5]
class classifier(object):
    def __init__(self, srate=500, display="hololens", duration=4, name="Liang"):
        self.srate = srate
        if display == "mobile":
            self.freqs = [7, 8, 9, 11, 7.5, 8.5]
        else:
            self.freqs = [7, 7.5, 8, 8.5, 9, 11]
        self.duration = duration
        self.name = name
        # self.rsvpclf = KNeighborsClassifier(n_neighbors=10)
        self.refsignal = self.init_refsignal().transpose((1, 2, 0))
        # self.ssvepchs, self.rsvpchs, self.rsvpskips = self.calibrate_clfs()

    def load_data(self):
        data = np.load("../ssvep_data_{}_session_{}.npy".format(self.name, 1))
        labels = np.load("../ssvep_labels_{}_session_{}.npy".format(self.name, 1))
        cls1labels_id = np.where(labels == 1)[0]
        cls2labels_id = np.where(labels == 2)[0]
        cls3labels_id = np.where(labels == 3)[0]
        cls1test = data[cls1labels_id[0:], :, :].transpose(1, 2, 0)
        cls2test = data[cls2labels_id[0:], :, :].transpose(1, 2, 0)
        cls3test = data[cls3labels_id[0:], :, :].transpose(1, 2, 0)
        test_data = np.concatenate((cls1test, cls2test, cls3test), 2)
        test_labels = np.concatenate(
            (0 * np.ones(cls1test.shape[2]), 1 * np.ones(cls2test.shape[2]), 2 * np.ones(cls3test.shape[2])))
        return test_data, test_labels

    def bandpass_filter_signal(self, eeg, low=2, high=45, order=3):
        nyq = 0.5 * self.srate
        low_c = low / nyq
        high_c = high / nyq
        b, a = signal.butter(order, [low_c, high_c], btype='band')
        y = signal.filtfilt(b, a, eeg)
        return y

    def init_refsignal(self):
        freqRef = []
        for fr in range(0, len(self.freqs)):
            freqRef.append(self.getReferenceSignals(int(self.duration * self.srate), self.freqs[fr]))
        refsignal = np.stack(freqRef)
        return refsignal

    def getReferenceSignals(self, length, target_freq, harmonics=4):
        """
        :param length: no. of data point in a channel
        :param target_freq: stimuli frequency
        :param harmonics: number of harmonics in reference signal
        :return:
        """
        reference_signals = []
        t = np.arange(0, (length / self.srate), step=1.0 / self.srate)

        for i in range(harmonics):
            reference_signals.append(np.sin(np.pi * 2 * (i + 1) * target_freq * t))
            reference_signals.append(np.cos(np.pi * 2 * (i + 1) * target_freq * t))
        reference_signals = np.array(reference_signals)
        return reference_signals

    def findCorr(self, signal_data, n_components=1):
        # Perform Canonical correlation analysis (CCA)
        freq = self.refsignal
        cca = CCA(n_components)
        corr = np.zeros(n_components)
        result = np.zeros(freq.shape[2])
        for freqIdx in range(0, freq.shape[2]):
            cca.fit(signal_data.T, np.squeeze(freq[:, :, freqIdx]).T)
            r_a, r_b = cca.transform(signal_data.T, np.squeeze(freq[:, :, freqIdx]).T)
            indVal = 0
            for indVal in range(0, n_components):
                corr[indVal] = np.corrcoef(r_a[:, indVal], r_b[:, indVal])[0, 1]
                result[freqIdx] = np.max(corr)
        return result

    def get_ssvep_channel(self, chs=[[0], [1], [0, 1]]):
        accs = self.get_ssvep_acc_per_channel_comb(chs=chs)
        id = np.argmax(accs)
        print("ssvep max: ", max(accs))
        return chs[id]

    def get_ssvep_acc_per_channel_comb(self, chs):
        test_data, test_labels = self.load_data()
        acc = np.zeros(len(chs))
        for i in range(len(chs)):
            data = test_data[chs[i], :, :]
            acc[i] = self.get_ssvep_acc(data, test_labels)
        return acc

    def get_ssvep_acc(self, data, labels):
        trials = data.shape[2]
        channels = data.shape[0]
        timepoints = int(self.srate * self.duration)
        data = data[:, -timepoints:, :]
        outs = np.zeros(trials)
        for i in range(trials):
            signal_data = data[:, :, i].reshape(channels, -1)
            result = self.findCorr(signal_data=signal_data)
            outs[i] = np.argmax(result)
        acc = np.sum(outs == labels) * 100 / trials
        return acc

    def softmax(self, x):
        e_x = np.exp(x - np.max(x))
        return e_x / e_x.sum(axis=0)

    def get_cca_cmd(self, eeg, n_components=1):
        timepoints = int(self.srate * self.duration)
        rhos = self.findCorr(eeg[:, -timepoints:], n_components=n_components)
        probs = self.softmax(rhos)
        cmd = np.argmax(probs)
        return probs, cmd

    def calibrate_rsvp(self):
        chs = [0, 1]
        skips = [50, 75, 100, 105, 110, 115, 120, 125, 130]
        accs = np.zeros((len(chs), len(skips)))
        for i in range(len(chs)):
            for j in range(len(skips)):
                data = np.load("../rsvp_data_{}_session_{}.npy".format(self.name, 1))[:, chs[i], skips[j]:]
                labels = np.load("../rsvp_labels_{}_session_{}.npy".format(self.name, 1)) - 4
                scores = cross_val_score(self.rsvpclf, data, labels, cv=5)
                accs[i, j] = np.mean(scores)
        # print(accs)
        max_xy = np.where(accs == accs.max())
        self.rsvpclf = KNeighborsClassifier(n_neighbors=10)
        data = np.load("../rsvp_data_{}_session_{}.npy".format(self.name, 1))[:, chs[max_xy[0][0]],
               skips[max_xy[1][0]]:]
        labels = np.load("../rsvp_labels_{}_session_{}.npy".format(self.name, 1)) - 4
        self.rsvpclf.fit(data, labels)
        print("rsvp max:", np.max(accs))
        # print(max_xy)
        return chs[max_xy[0][0]], skips[max_xy[1][0]]

    def get_ssvep_command(self, eeg_signal):
        # print("ssvep_Shape:", eeg_signal.shape)
        eeg = self.bandpass_filter_signal(eeg=eeg_signal)
        probs, cmd = self.get_cca_cmd(eeg)
        return cmd

    def calibrate_clfs(self):
        print("calibrating...")
        ssvepchs = self.get_ssvep_channel()
        rsvpchs, rsvpskip = self.calibrate_rsvp()

        print("done calibration!")
        return ssvepchs, rsvpchs, rsvpskip


if __name__ == '__main__':
    calib = classifier(name='tsai1', duration=3)
    # a = np.load("../ssvep_data_sai_session_1.npy")
    # eeg = a[0,:,:]
    # prob, cmds = calib.get_ssvep_command(eeg)
    # print(prob)
    # print(calib.ssvepchs, calib.rsvpchs, calib.rsvpskips)
    # a, b, c = calib.calibrate_clfs()
    # print(a, b, c)
