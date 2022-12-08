import serial
import sys
import time
from matplotlib import pyplot as plt
import numpy as np

SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200

MIN_FREQ_HZ = 200
MAX_FREQ_HZ = 1000

def init_serial() -> serial.Serial:
    '''
    initialize serial i/o
    '''
    try:
        ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUDRATE)
    except serial.SerialException:
        print(f'Serial port {SERIAL_PORT} not detected. Check connection and try again.')
        exit(1)
    
    # dummy write character to initialize the board's serial i/o.
    ser.write('a'.encode('utf-8'))
    # wait a second to make sure everything's good
    time.sleep(1)

    return ser

def read_conductivity(ser: serial.Serial):
    # wait a second to make sure everything's good
    time.sleep(1)

    freq = input(f'Frequency [{MIN_FREQ_HZ} - {MAX_FREQ_HZ} Hz]: ')
    while not (freq.isdigit() and int(freq) >= MIN_FREQ_HZ and int(freq) <= MAX_FREQ_HZ):
        freq = input(f'Input invalid. Please input an integer frequency between 200 and 1000 Hz: ')

    # write the frequency input
    ser.write(f'{freq}\n'.encode('utf-8'))

    # wait for readings to be taken
    time.sleep(5)
    readings = []
    while ser.in_waiting > 0:
        reading = ser.readline().decode('utf-8')
        readings.append(float(reading))

    num_readings = len(readings)
    vin = np.array(readings[:int(num_readings/2)])
    vout = np.array(readings[int(num_readings/2):])
    plt.plot(vin)
    plt.show()

    # phase shift
    phi = np.arccos(np.dot(vin, vout) / (np.linalg.norm(vin) * np.linalg.norm(vout)))
    # amplitude gain/loss
    G = np.linalg.norm(vout)/np.linalg.norm(vin)

    # resistance (real)
    R = G * np.cos(phi)
    # reactance (imaginary)
    X = G * np.sin(phi)

    print(f'Impedence: {R} + j{X}')

def main():
    # initialize serial i/o
    ser = init_serial()

    # if 'y', runs loop below
    run_again = 'y'

    while run_again in ['y', 'Y']:
        print('Please input a frequency at which to run conductivity sensor.')
        # get measurement and print it out
        read_conductivity(ser)

        input('Would you like to run another measurement [y/N]: ')
        # pass input to serial i/o
        ser.write(run_again.encode('utf-8'))

        # newline for spacing
        print()

if __name__ == '__main__':
    main()
