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

# rate at which information is transferred over serial port
BAUDRATE = 115200

# minimum frequency at which sensor can measure impedence
MIN_FREQ_HZ = 1000
# maximum frequency at which sensor can measure impedence
MAX_FREQ_HZ = 9999

# these should match the values in conductivity.c
# number of periods coming from serial i/o
NUM_PERIODS = 10
# number of samples in each period coming from serial i/o
NUM_SAMPLES_PER_PERIOD = 25

# amount of time to wait in between serial i/o reads;
# make this ~2x the amount of time waited between serial i/o writes
SERIAL_WAIT_S = 0.002

# feedback resistance in transimpedence amplifier in ohms
TIA_RF = 10e3

# max voltage of pico in volts
VDD = 3.3

def get_args() -> tuple[int, str, bool, str]:
    '''
    parses command-line arguments

    Returns
    -------
    int
        frequency
        if not specified at command-line, this will be -1 (indicating error)
    str
        path to csv file where to save output;
        if not specified, will default to "" (won't save)
    bool
        whether or not to plot the resulting waves
        if not specified at command-line, this will default to False
    str
        path to png file where to save plot;
        if not specified, will default to "" (won't save)
    '''
    frequency = -1
    output_file = ''
    plot = False
    plot_file = ''

    i = 0
    while i < len(sys.argv):
        if sys.argv[i].isdigit():
            frequency = int(sys.argv[i])
        elif sys.argv[i] == '--output-file':
            output_file = sys.argv[i+1]
            i += 1
        elif sys.argv[i] == '--plot':
            plot = True
        elif sys.argv[i] == '--plot-file':
            plot_file = sys.argv[i+1]
            i += 1
        i += 1

    return frequency, output_file, plot, plot_file

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

def plot(
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
    plt.xlabel('ms')
    plt.ylabel('V')
    plt.legend(['input signal to sensor', 'signal output from sensor'])
    title = f'{freq} Hz frequency'
    if impedence: title += f'; Z = {format_impedence(impedence)}'
    plt.title(title)

    if plot_file:
        Path(plot_file).parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(plot_file)
    if show_plot: plt.show()
    plt.close()


def impedence_dot_product(
    sin_input: np.ndarray,
    sin_output: np.ndarray,
    Rf: float = 1,
    offset: float = 0
) -> complex:
    '''
    calculates the impedence given the input and output signals using a dot
    product approximation.

    with the circuit we have, the impedence is given by:
        Z = -Rf(|Vin|/|Vout|)exp(-j phase_shift)
    where Z is the complex impedence in phaser form, Rf is the resistor used
    in the transimpdence amplifier, |Vin| is the magnitude of the Vin sine wave,
    |Vout| is the magnitude of the |Vout| sine wave, and phase_shift is the
    difference in phase between Vin and Vout.

    the inner product between two sines is defined as:
        <sin1, sin2> = Integral from -Pi to Pi of (sine1 * sine2)
    this can be approximated as a reimann sum, which ends up just being the
    vector dot product of the sampled version of the two sines, enabling the
    techniques used in this function.

    Parameters
    ----------
    sin_input : np.ndarray
        signal being fed into the impedence that is being sensed
    sin_output : np.ndarray
        output signal from the impedence that is being sensed
    Rf : float
        value of the resistor used in the transimpedence amplifier, in ohms
    offset : float
        the dc offset from zero of the two signals, in volts

    Returns
    -------
    complex
        complex impedance in cartesian form
    '''
    # remove the dc offset
    sin_input_offset = sin_input - offset
    sin_output_offset = sin_output - offset

    # amplitude gain/loss
    mag_impedence = Rf*np.linalg.norm(sin_input_offset)/np.linalg.norm(sin_output_offset)
    # phase shift
    phase_impedence = np.arccos(
        np.dot(sin_input_offset, sin_output_offset) / (
            np.linalg.norm(sin_input_offset) * np.linalg.norm(sin_output_offset)
        )
    ) + np.pi

    # resistance
    R = mag_impedence * np.cos(phase_impedence)
    # reactance
    X = mag_impedence * np.sin(phase_impedence)
    return R + X*1j
        
def get_measurements(
    frequency: int,
    ser: serial.Serial,
    output_file: str = '',
    show_plot: bool = False,
    plot_file: str = '',
) -> tuple[complex, float, float]:
    '''
    prompts user for frequency input, then sends job to pico over serial i/o.
    pico then returns sampled singals over serial i/o, which are used to calcuate
    the impedence. the temperature and pressure are also measured.

    Parameters
    ----------
    frequency : int
        frequency of sine wave
    ser : serial.Serial
        serial i/o connection
    output_file : str, optional
        csv file where output should be saved;
        defaults to "", in which case output isn't saved
    show_plot : bool, optional
        if True, plot resulting vin and vout waveforms and display plot to user
    plot_file : str, optional
        png file where plot should be saved;
        defaults to "", in which case plot isn't saved

    Returns
    -------
    complex
        complex impedence
    float
        temperature
    float
        pressure
    '''
    # wait a second to make sure everything's good
    #time.sleep(1)

    # make sure nothing is on the channel
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # write the frequency input to serial port
    # ser.write(f'{frequency}\n'.encode('utf-8'))
    serialio.write_string(f'{frequency}', ser)

    # wait for readings to be taken
    while ser.in_waiting == 0: pass

    #time.sleep(3.5)
    readings = []
    while ser.in_waiting > 0:
        time.sleep(SERIAL_WAIT_S)
        #reading = ser.readline().decode('utf-8')
        reading = serialio.readline(ser)
        readings.append(float(reading))

    # assume samples for conductivity measurement are the first portion of readings
    #samples = np.array(readings)
    samples = np.array(readings[:-2])
    # assume temperature measurement is second to last measurement
    temperature = readings[-2]
    #temperature = 0
    # assume pressure measurement is last measurement
    pressure = readings[-1]
    #pressure = 0

    # assume vin are the even-indexed samples
    vin = samples[::2]
    # assume vout are the odd-indexed samples
    vout = samples[1::2]

    # take average signal over each period
    vin = vin.reshape((NUM_PERIODS, NUM_SAMPLES_PER_PERIOD)).mean(axis=0)
    vout = vout.reshape((NUM_PERIODS, NUM_SAMPLES_PER_PERIOD)).mean(axis=0)

    impedence = impedence_dot_product(vin, vout, Rf=TIA_RF, offset=VDD/2)

    if output_file:
        path = Path(output_file)
        if not path.is_file():
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(output_file, 'w') as f:
                csv_file = csv.writer(f)
                csv_file.writerow(('frequency', 'resistance', 'reactance', 'temperature', 'pressure'))

        with open(output_file, 'a') as f:
            csv_file = csv.writer(f)
            csv_file.writerow((frequency, impedence.real, impedence.imag, temperature, pressure))

    plot(vin, vout, frequency, impedence, show_plot, plot_file)

    return impedence, temperature, pressure

def main():
    frequency, output_file, show_plot, plot_file = get_args()

    if frequency > MAX_FREQ_HZ or frequency < MIN_FREQ_HZ:
        # if frequency invalid, exit
        print(f'Error: please specify integer frequency between {MIN_FREQ_HZ} to {MAX_FREQ_HZ} Hz')
        exit(1)

    # initialize serial i/o
    ser = serialio.init_serial()

    # get measurement
    impedence, temperature, pressure = get_measurements(frequency, ser, output_file, show_plot, plot_file)

    print(f'Impedence: {format_impedence(impedence)} Ohms')
    print(f'Temperature: {temperature} C')
    print(f'Pressure: {pressure} mBars')

if __name__ == '__main__':
    main()
