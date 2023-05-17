import os
import serial
import serialio
import sys
import glob
import time
from matplotlib import pyplot as plt
import numpy as np
from typing import Any, Optional
import csv
from pathlib import Path
from dataclasses import dataclass

plt.style.use('ggplot')

# minimum frequency at which sensor can measure impedence
MIN_FREQ_HZ = 100
# maximum frequency at which sensor can measure impedence
MAX_FREQ_HZ = 10000
# minimum amplitude at which sensor can measure impedence
MIN_AMPLITUDE_HZ = 0.1
# maximum amplitude at which sensor can measure impedence
MAX_AMPLITUDE_HZ = 1.0


# these should match the values in sensor.c
# number of periods coming from serial i/o
NUM_PERIODS = 10
# number of samples in each period coming from serial i/o
NUM_SAMPLES_PER_PERIOD = 25

# these should match the values in sensor.c
# frequencies to use for performing impedence spectroscopy
FREQUENCIES_FOR_SPECTROSCOPY = [
    100,
    200,
    300,
    400,
    500,
    600,
    700,
    800,
    900,
    1000,
    2000,
    3000,
    4000,
    5000,
    6000,
    7000,
    8000,
    9000,
    10000
]

# amount of time to wait in between serial i/o reads;
# make this ~2x the amount of time waited between serial i/o writes
SERIAL_WAIT_S = 0.001

# feedback resistance in transimpedence amplifier in ohms
TIA_RF = 10e3

# max voltage of pico in volts
VDD = 3.3
# number of bits in an ADC sample
ADC_NUM_BITS = 8
# converts an adc sample to a voltage
ADC_CONVERT = VDD / ((1 << ADC_NUM_BITS) - 1)

@dataclass
class Sine:
    '''
    datastructure representing a sine wave:
        sine(t) = A*sin*(2*pi*f*t + p) + o

    Attributes
    ----------
    amplitude : float
        A [V]
    frequency : float
        f [Hz]
    phase : float
        p [Radians]
    offset : float
        o [V]
    '''
    amplitude : float
    frequency : float
    phase : float
    offset : float


def characterize(single_period: np.ndarray[NUM_SAMPLES_PER_PERIOD], frequency: float) -> Sine:
    '''
    given one period of a sine wave, finds its characteristic properties

    Parameters
    ----------
    single_period : (NUM_SAMPLES_PER_PERIOD,) shaped np.ndarray
        one period of the sine wave
    frequency : float
        the frequency of the sine wave

    Returns
    -------
    Sine
        characterized sine wave
    '''
    # timestep
    dt = 1/(frequency*len(single_period))

    i_min = np.argmin(single_period)
    v_min = np.min(single_period)
    i_max = np.argmax(single_period)
    v_max = np.max(single_period)

    offset = (v_max + v_min) / 2
    amplitude = v_max - offset
    phase = np.pi/2 - 2*np.pi*frequency*i_max*dt

    return Sine(amplitude,frequency,phase,offset)

def impedence(
    vin : np.ndarray[NUM_SAMPLES_PER_PERIOD],
    vout : np.ndarray[NUM_SAMPLES_PER_PERIOD],
    frequency : float,
    Rf : float
) -> complex:
    '''
    calculates impedence based on input and output waveforms according to the following formula:
        Z = -Rf * vin/vout

    Parameters
    ----------
    vin : (NUM_SAMPLES_PER_PERIOD,) shaped np.ndarray
        input signal to sensor (in V)
    vout : (NUM_SAMPLES_PER_PERIOD,) shaped np.ndarray
        output signal from sensor
    frequency : float
        the frequency of the signals (in V)
    Rf : float
        feedback resistor in the transimpedence amplifier (in Ohms)

    Returns
    -------
    complex
        the complex impedence Z = R + Xj (in Ohms)
    '''
    vin_sine = characterize(vin, frequency)
    vout_sine = characterize(vout, frequency)

    magnitude = Rf*vin_sine.amplitude/vout_sine.amplitude
    phase = vin_sine.phase - vout_sine.phase + np.pi

    R = magnitude*np.cos(phase)
    X = magnitude*np.sin(phase)

    return R+1j*X

def format_impedence(impedence: complex) -> str:
    '''
    formats impedence as
        R + jX
    where R and X are in scientific notation with 3 significant digits of precision

    Parameters
    ----------
    impedence : complex
        complex number

    Returns
    -------
    str
        formatted complex number
    '''
    R = f'{impedence.real:0.2E}'
    X_abs = f'{np.abs(impedence.imag):0.2E}'
    X_sign = f'{"+-"[int(impedence.imag < 0)]}'

    return f'{R} {X_sign} j{X_abs}'

def vin_vout_plot(
    vin: np.ndarray,
    vout: np.ndarray,
    freq: Any,
    impedence: Optional[complex] = None,
    show_plot: bool = False,
    plot_file: str = '',
):
    '''
    plots the input signal and output signal read in on the adc pins of the pico

    Parameters
    ----------
    vin : np.ndarray
        signal being fed into the impedence that is being sensed
    vout : np.ndarray
        output signal from the impedence that is being sensed
    freq : Any
        frequency of signals. must be convertible to an int.
    impedence : complex, optional
        the impedence measurement based on these voltages;
        default is none, in which case the impedence measurement will not
        be included in the plot
    show_plot : bool, optional
        if true, display plot to user;
        defaults to false
    plot_file : str, optional
        png file where plot should be saved;
        defaults to "" in which case no plot is saved
    '''
    if not show_plot and not plot_file:
        return

    # setup time axis
    # note that they are ideally being plotted over one period, hence the
    # stop time of 1/freq
    # multiply by 1000 since we're plotting in milliseconds
    t_vin = np.linspace(0, 1/int(freq), len(vin))*1000
    t_vout = np.linspace(0, 1/int(freq), len(vout))*1000

    # plot
    plt.plot(t_vin, vin)
    plt.plot(t_vout, vout)

    # set labels and title, etc.
    plt.xlabel('Time [ms]')
    plt.ylabel('Voltage [V]')
    plt.legend([r'$V_{in}$', r'$V_{out}$'])
    title = f'{freq} Hz frequency'
    if impedence: title += f'; Z = {format_impedence(impedence)}'
    plt.title(title)

    if plot_file:
        Path(plot_file).parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(plot_file)
    if show_plot: plt.show()
    plt.close()

def bode_plot(
    magnitudes: np.ndarray,
    phases: np.ndarray,
    show_plot: bool = False,
    plot_file: str = '',
):
    '''
    gives bode plot of a spectrum of impedences

    Parameters
    ----------
    magnitudes : np.ndarray
        impedence magnitudes at each frequency
    phases : np.ndarray
        impedence phases at each frequency
    show_plot : bool, optional
        if true, display plot to user;
        defaults to false
    plot_file : str, optional
        png file where plot should be saved;
        defaults to "" in which case no plot is saved
    '''
    if not show_plot and not plot_file:
        return

    fig, (ax_mag, ax_phase) = plt.subplots(nrows=2)
    fig.suptitle('Bode Plot')

    ax_mag.loglog(FREQUENCIES_FOR_SPECTROSCOPY, magnitudes)
    ax_mag.set_xlabel('Frequency [Hz]')
    ax_mag.set_ylabel('Magnitude [$\Omega$]')

    ax_phase.semilogx(FREQUENCIES_FOR_SPECTROSCOPY, phases)
    ax_phase.set_xlabel('Frequency [Hz]')
    ax_phase.set_ylabel('Phase [Radians]')
    ax_phase.set_ylim(-np.pi, np.pi)
    ax_phase.set_yticks([-np.pi, -np.pi/2, 0, np.pi/2, np.pi], ['$-\pi$', r'$-\frac{{\pi}}{{2}}$', '$0$', r'$\frac{{\pi}}{{2}}$', '$\pi$'])

    if plot_file:
        Path(plot_file).parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(plot_file)
    if show_plot: plt.show()
    plt.close()

def get_impedence_single_frequency(
    frequency: int,
    amplitude: float,
    ser: serial.Serial,
    show_plot: bool = False,
    plot_file: str = '',
) -> complex:
    '''
    measure impedence at specified frequency

    Parameters
    ----------
    frequency : int
        frequency for waveforms
    amplitude : float
        amplitude for waveforms
    ser : serial.Serial
        serial connection
    show_plot : bool, optional
        if True, presents a plot of waveforms to user; default is False
    plot_file : str, optional
        if not empty, saves plot of waveforms to specified path; default is ''

    Returns
    -------
    complex
        the complex impedence Z = R + Xj (in Ohms)
    '''
    # make sure serial buffers don't have residual samples
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # send over frequency and amplitude parameters
    ser.write(f'f{frequency},{amplitude},'.encode('utf-8'))

    # wait for vin and vout waveforms to be returned;
    # each waveform will have NUM_SAMPLES_PER_PERIOD entries
    while ser.in_waiting < 2*NUM_SAMPLES_PER_PERIOD : pass

    # since the adc takes 8-bit samples, and each ASCII character
    # is represented by an 8-bit digit, we can just read the
    # samples directly without having to decode them
    samples = ADC_CONVERT * np.array([
        sample
        for sample in ser.read_until(size=2*NUM_SAMPLES_PER_PERIOD)
    ])

    # assume that the first half of samples are vin and the second half are vout
    vin = samples[:NUM_SAMPLES_PER_PERIOD]
    vout = samples[NUM_SAMPLES_PER_PERIOD:]

    Z = impedence(vin, vout, frequency, Rf=TIA_RF)

    vin_vout_plot(vin, vout, frequency, Z, show_plot, plot_file)

    return Z

def get_impedence_spectrum(
    amplitude: float,
    ser: serial.Serial,
    show_plots: bool = False,
    bode_plot_file: str = '',
) -> tuple[np.ndarray, np.ndarray]:
    '''
    performs impedence spectroscopy for the frequencies specified in FREQUENCIES_FOR_SPECTROSCOPY

    Parameters
    ----------
    amplitude : float
        amplitude of each of the waveforms
    ser : serial.Serial
        serial connection
    show_plots : bool, optional
        if true, display plots to user;
        defaults to false
    bode_plot_file : str, optional
        png file where bode plot should be saved;
        defaults to "" in which case no plot is saved

    Returns
    -------
    (len(FREQUENCIES_FOR_SPECTROSCOPY),) shaped np.ndarray
        magnitudes for each impedence
    (len(FREQUENCIES_FOR_SPECTROSCOPY),) shaped np.ndarray
        phases for each impedence
    '''
    # make sure serial buffers don't have residual samples
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # send over frequency and amplitude parameters
    ser.write(f's{amplitude},'.encode('utf-8'))

    # wait for pico to finish sending all samples over
    while ser.in_waiting == 0: pass
    prev_in_waiting = 0
    while ser.in_waiting != prev_in_waiting:
        prev_in_waiting = ser.in_waiting
        time.sleep(SERIAL_WAIT_S)

    # read in samples
    samples = np.array([
        float(sample) 
        for sample in ser.read_until(
            size=ser.in_waiting
        ).decode('utf-8').split(',')
    ])

    # assume magnitudes are the first half of the samples and phases are the second half
    magnitudes = samples[:len(FREQUENCIES_FOR_SPECTROSCOPY)]
    phases = samples[len(FREQUENCIES_FOR_SPECTROSCOPY):]
    # normalize phases
    phases = np.arctan2(np.sin(phases), np.cos(phases))

    bode_plot(magnitudes, phases, show_plots, bode_plot_file)

    return magnitudes, phases


def get_temperature_and_pressure(
    ser: serial.Serial,
) -> tuple[float, float]:
    '''
    gets temperature and pressure measurements

    Parameters
    ----------
    ser: serial.Serial
        serial connection

    Returns
    -------
    float
        temperature (in C)
    float
        pressure (in mBars)
    '''
    # make sure nothing is on the channel
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    ser.write(f't'.encode('utf-8'))

    # wait for pico to finish sending all samples over
    while ser.in_waiting == 0: pass
    prev_in_waiting = 0
    while ser.in_waiting != prev_in_waiting:
        prev_in_waiting = ser.in_waiting
        time.sleep(0.001)

    temperature, pressure = ser.read_until(size=ser.in_waiting).decode('utf-8').split(',')

    return float(temperature), float(pressure)