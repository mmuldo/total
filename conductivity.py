import serial
import sys
import glob
import time
from matplotlib import pyplot as plt
import numpy as np
from typing import Any

# serial port of raspberry pi pico
# change to whichever port pico comes up on
SERIAL_PORT = '/dev/ttyACM1'

# rate at which information is transferred over serial port
BAUDRATE = 115200

# minimum frequency at which sensor can measure impedence
MIN_FREQ_HZ = 200
# maximum frequency at which sensor can measure impedence
MAX_FREQ_HZ = 1000

def serial_ports():
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
        serial_port = serial_ports()[0]
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
    mag_impedence = -Rf*np.linalg.norm(sin_input_offset)/np.linalg.norm(sin_output_offset)
    # phase shift
    phase_impedence = np.arccos(
        np.dot(sin_input_offset, sin_output_offset) / (
            np.linalg.norm(sin_input_offset) * np.linalg.norm(sin_output_offset)
        )
    )

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

def get_one_period(sin_samples: np.ndarray) -> np.ndarray:
    '''
    given a sampled arbitrary length sin wave, return one period of the
    wave starting at the first sample

    Parameters
    ----------
    sin_samples : np.ndarray
        the sampled sin wave, multiple periods in general
    
    Returns
    -------
    np.ndarray
        one period of the sin wave starting at the first sample
    '''
    zero_crossings = 0

    i = 0
    prev_polarity = np.sign(sin_samples[0])
    for _ in len(sin_samples):
        polarity = np.sign(sin_samples[i])
        if polarity != prev_polarity:
            zero_crossings += 1
        prev_polarity = polarity
    #TODO

        


def read_conductivity(ser: serial.Serial):
    '''
    prompts user for frequency input, then sends job to pico over serial i/o.
    pico then returns sampled singals over serial i/o, which are used to calcuate
    the impedence. the impedence is then printed.

    Parameters
    ----------
    ser : serial.Serial
        serial i/o connection
    '''
    # wait a second to make sure everything's good
    time.sleep(1)

    # get frequency from user
    freq = input(f'Frequency [{MIN_FREQ_HZ} - {MAX_FREQ_HZ} Hz]: ')
    while not (freq.isdigit() and int(freq) >= MIN_FREQ_HZ and int(freq) <= MAX_FREQ_HZ):
        # keep prompting until user input is valid
        freq = input(f'Input invalid. Please input an integer frequency between 200 and 1000 Hz: ')

    # write the frequency input to serial port
    ser.write(f'{freq}\n'.encode('utf-8'))

    # wait for readings to be taken
    time.sleep(5)
    readings = []
    while ser.in_waiting > 0:
        reading = ser.readline().decode('utf-8')
        readings.append(float(reading))

    num_readings = len(readings)
    # assume the vin readings are the first half of readings
    num_vin_readings = int(num_readings/2)
    vin = np.array(readings[:num_vin_readings])
    # assume the vout readings are the second half of readings
    vout = np.array(readings[num_vin_readings:])

    # uncomment if you want to see plots
    # comment out if you don't want to see plots
    plot(vin, vout, freq)

    # choose which method of impedence calculation to use
    #impedence = impedence_dot_product(vin, vout, Rf=100e3, offset=0)
    impedence = impedence_brute_force(vin, vout, Rf=100e3, offset=0)

    print(f'Impedence: {impedence}')


def main():
    # initialize serial i/o
    ser = init_serial()

    print('Please input a frequency at which to run conductivity sensor.')
    # get measurement and print it out
    read_conductivity(ser)

    # newline for spacing
    print()

if __name__ == '__main__':
    main()
