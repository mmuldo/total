import numpy as np
from numpy import dot, pi, sin, arccos
from numpy.linalg import norm
from matplotlib import pyplot as plt

def noise(amp, length):
    return amp*2*(np.random.rand(length) - 0.5)
n = 100
n_periods = 5
t = np.linspace(0, n_periods + 0.23, n*n_periods)

vin = 3 * sin(2 * pi * t) + noise(0.2, n*n_periods)
vout = sin(2 * pi * t - pi / 3) + noise(0.2, n*n_periods)

vin_good = vin[:n_periods * n]
vout_good = vout[:n_periods * n]

vin_period = vin_good.reshape(n_periods, n)
vout_period = vout_good.reshape(n_periods, n)

vin_avg = vin_period.mean(axis=0)
vout_avg = vout_period.mean(axis=0)

plt.plot(vin_avg)
plt.plot(vout_avg)
plt.show()

phi = arccos(dot(vin, vout) / (norm(vin) * norm(vout)))
g = norm(vin)/norm(vout)
print(phi)
print(np.pi / 3)
print(g)
