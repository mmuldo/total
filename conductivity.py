import serial
import sys
import glob
import time
from matplotlib import pyplot as plt
import numpy as np
from typing import Any

# rate at which information is transferred over serial port
BAUDRATE = 115200

# minimum frequency at which sensor can measure impedence
MIN_FREQ_HZ = 200
# maximum frequency at which sensor can measure impedence
MAX_FREQ_HZ = 1000

def get_args() -> tuple[int, bool]:
    '''
    parses command-line arguments

    Returns
    -------
    int
        frequency
        if not specified at command-line, this will be -1 (indicating error)
    bool
        whether or not to plot the resulting waves
        if not specified at command-line, this will default to False
    '''
    frequency = -1
    plot = False

    for arg in sys.argv:
        if arg.isdigit():
            frequency = int(arg)
        elif arg == '--plot':
            plot = True

    return frequency, plot

def get_serial_ports():
    """
    Lists serial port names

    Raises
    ------
    EnvironmentError:
        On unsupported or unknown platforms

    Returns
    -------
        A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

def init_serial() -> serial.Serial:
    '''
    initialize serial i/o
    '''
    try:
        serial_port = get_serial_ports()[0]
        ser = serial.Serial(port=serial_port, baudrate=BAUDRATE)
    except serial.SerialException:
        print(f'Serial port {serial_port} not detected. Check connection and try again.')
        exit(1)
    except IndexError:
        print('No serial port detected')
        exit(1)
    except EnvironmentError as e:
        print(e)
        exit(1)
    
    # dummy write character to initialize the board's serial i/o.
    ser.write('a'.encode('utf-8'))
    # wait a second to make sure everything's good
    time.sleep(1)

    return ser

def plot(
    vin: np.ndarray,
    vout: np.ndarray,
    freq: Any
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
    '''
    # setup time axis
    # note that they are ideally being plotted over one period, hence the
    # stop time of 1/freq
    t_vin = np.linspace(0, 1/int(freq), len(vin))
    t_vout = np.linspace(0, 1/int(freq), len(vout))

    # plot
    plt.plot(t_vin, vin)
    plt.plot(t_vout, vout)

    # set labels and title, etc.
    plt.xlabel('s')
    plt.ylabel('V')
    plt.legend(['input signal to sensor', 'signal output from sensor'])
    plt.title(f'{freq} Hz frequency')

    plt.show()

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
    print(phase_impedence)

    # resistance
    R = mag_impedence * np.cos(phase_impedence)
    # reactance
    X = mag_impedence * np.sin(phase_impedence)
    return R + X*1j

def impedence_brute_force(
    sin_input: np.ndarray,
    sin_output: np.ndarray,
    Rf: float = 1,
    offset: float = 0
):
    '''
    calculates the impedence given the input and output signals using a sort
    of "brute force" technique.

    the method is as follows: find the max value in both sampled signals, and
    assume these values approximate the magnitudes. then find the location
    that these max values occur and subtract them (along with some other operations)
    to get the approximate phase shift.

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
    # remove dc offset
    sin_input_offset = sin_input - offset
    sin_output_offset = sin_output - offset

    mag_input = sin_input_offset.max()
    mag_output = sin_output_offset.max()

    # amplitude gain/loss
    mag_impedence = -Rf * mag_input/mag_output

    # phase shift
    phase_impedence = 2*np.pi*(sin_output.argmax() - sin_input.argmax())/len(sin_input)

    # resistance
    R = mag_impedence * np.cos(phase_impedence)
    # reactance
    X = mag_impedence * np.sin(phase_impedence)
    return R + X*1j

def get_one_period(sin_samples: np.ndarray, tolerance: float = 0.0001) -> tuple[int, int]:
    '''
    given a sampled arbitrary length sin wave, return the
    starting index and ending index of one period.
    sin_samples must contain at least one period.

    Parameters
    ----------
    sin_samples : np.ndarray
        the sampled sin wave, multiple periods in general
    tolerance : float, optional
        max difference in samples for them to be considered equal
        default is 0.0001
    
    Returns
    -------
    int
        index where the period starts in the sampled wave
    int
        index where the period ends in the sampled wave
    '''
    first_index = 1
    second_index = first_index + 1
    initial_gradient_polarity = np.sign(sin_samples[first_index] - sin_samples[first_index - 1])
    num_samples = len(sin_samples)

    def cycle_complete(first_index: int, second_index: int) -> bool:
        equal = np.abs(sin_samples[first_index] - sin_samples[second_index]) <= tolerance
        same_gradient_polarity = np.sign(sin_samples[second_index] - sin_samples[second_index - 1]) == initial_gradient_polarity
        return equal and same_gradient_polarity

    while second_index < num_samples - 1 and not cycle_complete(first_index, second_index):
        second_index += 1

    return first_index, second_index
        
def read_conductivity(frequency: int, ser: serial.Serial, make_plot: bool = False) -> complex:
    '''
    prompts user for frequency input, then sends job to pico over serial i/o.
    pico then returns sampled singals over serial i/o, which are used to calcuate
    the impedence. the impedence is then printed.

    Parameters
    ----------
    frequency : int
        frequency of sine wave
    ser : serial.Serial
        serial i/o connection
    make_plot : bool, optional
        if True, plot resulting vin and vout waveforms

    Returns
    -------
    complex
        complex impedence
    '''
    # wait a second to make sure everything's good
    #time.sleep(1)

    # write the frequency input to serial port
    ser.write(f'{frequency}\n'.encode('utf-8'))

    # wait for readings to be taken
    time.sleep(3.5)
    readings = []
    while ser.in_waiting > 0:
        reading = ser.readline().decode('utf-8')
        readings.append(float(reading))

    num_readings = len(readings)
    # assume the vin readings are the first half of readings
    num_vin_readings = int(num_readings/2)
    vin = np.array(readings[:num_vin_readings])
    #print(vin.shape)
    # assume the vout readings are the second half of readings
    vout = np.array(readings[num_vin_readings:])

    #first_index, second_index = get_one_period(vin, tolerance=0.0001)
    #print(vin[first_index], vin[second_index])
    #vin = vin[first_index:second_index]
    #print(vin.shape)
    #vout = vout[first_index:second_index]

    # uncomment if you want to see plots
    # comment out if you don't want to see plots
    plot(vin, vout, frequency)

    # choose which method of impedence calculation to use
    impedence = impedence_dot_product(vin, vout, Rf=100e3, offset=1.65)
    #impedence = impedence_brute_force(vin, vout, Rf=100e3, offset=0)

    return impedence

def main():
    frequency, make_plot = get_args()

    if frequency > MAX_FREQ_HZ or frequency < MIN_FREQ_HZ:
        # if frequency invalid, exit
        print(f'Error: please specify integer frequency between {MIN_FREQ_HZ} to {MAX_FREQ_HZ} Hz')
        exit(1)

    # initialize serial i/o
    ser = init_serial()

    # get measurement
    impedence = read_conductivity(frequency, ser, make_plot)

    print(impedence)

if __name__ == '__main__':
    main()
