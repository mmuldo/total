#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "sine_analysis.h"

sine characterize(double sine_period[], double frequency, int num_samples) {
    sine s;
    double amplitude;
    double phase;
    double offset;

    double dt = 1/(frequency * num_samples);
    int i_max = 0;
    double value_max = sine_period[0];
    int i_min = 0;
    double value_min = sine_period[0];

    for (int i = 0; i < num_samples; i++) {
        if (sine_period[i] > value_max) {
            i_max = i;
            value_max = sine_period[i];
        }

        if (sine_period[i] < value_min) {
            i_min = i;
            value_min = sine_period[i];
        }
    }

    offset = (value_max + value_min) / 2;
    amplitude = value_max - offset;
    phase = PI/2 - 2*PI*frequency*i_max*dt;

    s.amplitude = amplitude;
    s.frequency = frequency;
    s.phase = phase;
    s.offset = offset;

    return s;
}

complex impedence(sine vin, sine vout, double Rf) {
    complex Z;

    Z.magnitude = Rf*vin.amplitude/vout.amplitude;
    Z.phase = vin.phase - vout.phase + PI;

    return Z;
}