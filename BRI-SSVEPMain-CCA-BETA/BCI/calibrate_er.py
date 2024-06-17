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
    def __init__(self, srate=500, freqs=[7, 9], duration=4, name="sai"):
        self.srate = srate
        self.freqs = freqs
        self.duration = duration
        self.name = name
        # self.rsvpclf = KNeighborsClassifier(n_neighbors=10)
        self.refsignal = self.init_refsignal().transpose((1, 2, 0))

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

    def softmax(self, x):
        e_x = np.exp(x - np.max(x))
        return e_x / e_x.sum(axis=0)

    def get_cca_cmd(self, eeg, n_components=1):
        timepoints = int(self.srate * self.duration)
        rhos = self.findCorr(eeg[:, -timepoints:], n_components=n_components)
        probs = self.softmax(rhos)
        cmd = np.argmax(probs)
        return probs, cmd

    def get_ssvep_command(self, eeg_signal):
        # print("ssvep_Shape:", eeg_signal.shape)
        eeg = self.bandpass_filter_signal(eeg=eeg_signal)
        probs, cmd = self.get_cca_cmd(eeg)
        return cmd


if __name__ == '__main__':
    calib = classifier(name='tsai1', duration=3)
