import serialio
import time
import matplotlib.pyplot as plt
import numpy as np

NUM_PERIODS = 10
NUM_SAMPLES_PER_PERIOD = 25

def main():
    ser = serialio.init_serial()
    ser.write('a'.encode('utf-8'))
    while ser.in_waiting == 0: pass
    samples = []
    while ser.in_waiting > 0:
        time.sleep(0.002)
        sample = ser.readline().decode('utf-8')
        samples.append(float(sample))

    samples = np.array(samples)
    vin = samples[::2]
    vout = samples[1::2]
    plt.plot(vin)
    plt.plot(vout)
    plt.show()

    vin = vin.reshape((NUM_PERIODS, NUM_SAMPLES_PER_PERIOD))
    vout = vout.reshape((NUM_PERIODS, NUM_SAMPLES_PER_PERIOD))

    plt.plot(vin.mean(axis=0))
    plt.plot(vout.mean(axis=0))
    plt.show()

if __name__ == '__main__':
    main()
