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

# rate at which information is transferred over serial port
BAUDRATE = 115200

# minimum frequency at which sensor can measure impedence
MIN_FREQ_HZ = 100
# maximum frequency at which sensor can measure impedence
MAX_FREQ_HZ = 10000

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
    amplitude = 0.6

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
        elif sys.argv[i] == '--amplitude':
            amplitude = float(sys.argv[i+1])
            i += 1
        i += 1

    return frequency, output_file, plot, plot_file, amplitude

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

@dataclass
class Sine:
    amplitude : float
    frequency : float
    phase : float
    offset : float

def characterize(period, frequency):
    dt = 1/(frequency*len(period))

    i_min = np.argmin(period)
    v_min = np.min(period)
    i_max = np.argmax(period)
    v_max = np.max(period)

    offset = (v_max + v_min) / 2
    amplitude = v_max - offset
    phase = np.pi/2 - 2*np.pi*frequency*i_max*dt

    return Sine(amplitude,frequency,phase,offset)

def impedence(vin, vout, frequency, Rf):
    vin_sine = characterize(vin, frequency)
    vout_sine = characterize(vout, frequency)

    magnitude = Rf*vin_sine.amplitude/vout_sine.amplitude
    phase = vin_sine.phase - vout_sine.phase + np.pi

    R = magnitude*np.cos(phase)
    X = magnitude*np.sin(phase)

    return R+1j*X

def get_impedence_single_frequency(
    frequency: int,
    amplitude: float,
    ser: serial.Serial,
    show_plot: bool = False,
):
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    ser.write(f'f{frequency},{amplitude},'.encode('utf-8'))

    while ser.in_waiting < 2*NUM_SAMPLES_PER_PERIOD : pass

    samples = [sample for sample in ser.read_until(size=2*NUM_SAMPLES_PER_PERIOD)]
    samples = 3.3/((1<<8)-1) * np.array(samples)

    vin = samples[:NUM_SAMPLES_PER_PERIOD]
    vout = samples[NUM_SAMPLES_PER_PERIOD:]

    Z = impedence(vin, vout, frequency, Rf=TIA_RF)

    plot(vin, vout, frequency, Z, show_plot)

    return Z

def get_impedence_spectrum(
    amplitude: float,
    ser: serial.Serial,
):
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    ser.write(f's{amplitude},'.encode('utf-8'))

    prev_in_waiting = 0
    while ser.in_waiting == 0: pass
    while ser.in_waiting != prev_in_waiting:
        prev_in_waiting = ser.in_waiting
        time.sleep(0.001)

    from_serial = ser.read_until(size=ser.in_waiting).decode('utf-8')
    samples0 = [sample for sample in from_serial.split(',')]
    samples = np.array([float(sample) for sample in samples0])
    freqs = [100, 300, 500, 700, 900, 1000, 3000, 5000, 7000, 9000]
    mags = samples[:10]
    phases = samples[10:]
    phases = np.array([phase if phase < np.pi else phase - 2*np.pi for phase in phases])

    plt.loglog(freqs, mags)
    plt.show()
    plt.plot(freqs, phases)
    plt.xscale('log')
    plt.show()

def get_temperature_and_pressure(
    ser: serial.Serial,
):
    # make sure nothing is on the channel
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    ser.write(f't'.encode('utf-8'))

    while ser.in_waiting < 11 : pass
    temperature, pressure = ser.read_until(size=11).decode('utf-8').split(',')

    return float(temperature), float(pressure)

def get_measurements(
    frequency: int,
    amplitude: float,
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

    time.sleep(1)
    samples = []        
    #while ser.in_waiting > 0:
    samples += [sample for sample in ser.read_until(size=2*NUM_SAMPLES_PER_PERIOD)]
    samples = 3.3/((1<<8)-1) * np.array(samples)

    # assume samples for conductivity measurement are the first portion of readings
    #samples = np.array(readings)
    #samples = np.array(readings[:-2])
    # assume temperature measurement is second to last measurement
    #temperature = readings[-2]
    temperature = 0
    # assume pressure measurement is last measurement
    #pressure = readings[-1]
    pressure = 0
    time.sleep(1)
    if True:
        temperature = float(serialio.readline(ser))
        pressure = float(serialio.readline(ser))

    # assume vin are the even-indexed samples
    #vin = samples[::2]
    vin = samples[:NUM_SAMPLES_PER_PERIOD]
    # assume vout are the odd-indexed samples
    #vout = samples[1::2]
    vout = samples[NUM_SAMPLES_PER_PERIOD:]

    # take average signal over each period
    #vin = vin.reshape((NUM_PERIODS, NUM_SAMPLES_PER_PERIOD)).mean(axis=0)
    #vout = vout.reshape((NUM_PERIODS, NUM_SAMPLES_PER_PERIOD)).mean(axis=0)

    impedence = impedence_dot_product(vin, vout, Rf=TIA_RF, offset=0)

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
    frequency, output_file, show_plot, plot_file, amplitude = get_args()

    if frequency > MAX_FREQ_HZ or frequency < MIN_FREQ_HZ:
        # if frequency invalid, exit
        print(f'Error: please specify integer frequency between {MIN_FREQ_HZ} to {MAX_FREQ_HZ} Hz')
        exit(1)

    # initialize serial i/o
    ser = serialio.init_serial()

    # get measurement
    #impedence, temperature, pressure = get_measurements(frequency, ser, output_file, show_plot, plot_file)
    impedence = get_impedence_single_frequency(frequency, amplitude, ser, show_plot)
    #temperature, pressure = get_temperature_and_pressure(ser)
    #get_impedence_spectrum(amplitude, ser)

    print(f'Impedence: {format_impedence(impedence)} Ohms')
    #print(f'Temperature: {temperature} C')
    #print(f'Pressure: {pressure} mBars')

if __name__ == '__main__':
    main()
