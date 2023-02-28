from scipy import signal
import numpy as np
from matplotlib import pyplot as plt

FILTER_ORDER = 10
BANDPASS_WINDOW = np.array((-2, 2))
T = np.linspace(0, 10, 10000)

def extract_frequency(
    input_signal: np.ndarray,
    frequency: float
) -> np.ndarray:
    sos = signal.butter(
        N=FILTER_ORDER,
        #Wn=frequency+BANDPASS_WINDOW,
        Wn=frequency,
        fs=1000,
        #btype='bandpass',
        btype='lowpass',
        output='sos'
    )

    return signal.sosfilt(sos, input_signal)

def sine(f):
    num_periods = 5
    return np.sin(2*np.pi*f*T)

def noisy(sig):
    return sig + np.random.uniform(-0.5, 0.5, size=sig.shape[0])

if __name__ == '__main__':
    input_signal = noisy(sine(10))
    plt.plot(input_signal[int(7*len(input_signal)/8):])
    plt.plot(extract_frequency(input_signal, 10)[int(7*len(input_signal)/8):])
    plt.show()

