#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "sine.h"
#include "adc.h"

/// @brief creates an table whose entries are the pwm counter compare level corresponding to the appropriate sine function value
/// @param length the length of the table
/// @param amplitude desired amplitude of the sine wave
/// @return the sine table
uint32_t * generate_sine_table(int length, double amplitude) {
    uint32_t * sine_table = malloc(length*sizeof(uint32_t));

    for (int i = 0; i < length; i++) {
        // "duty cycle" is the same thing as the pwm counter compare level in this case
        uint32_t duty_cycle = (uint32_t) round(length*((2*amplitude/VDD)*sin(2 * PI * i / length) + 1));
        sine_table[i] = duty_cycle;
    }

    return sine_table;
}

/// @brief calculates the length of the sine table that produces a sine wave of the given frequency when the pwm clkdiv is set to 1.0
/// @param frequency the sine wave frequency
/// @return sine table length
int highest_frequency_to_table_length(float frequency) {
    // the following comes from the fact that
    //      f_sine = f_clk / (wrap * length * clkdiv)
    // where we assume wrap = 2*length and clkdiv = 1
    return (int) round(sqrt(CLK_KHZ * KHZ_TO_HZ / (2 * frequency)));
}

void average_period(uint8_t samples[], uint8_t input_period[], uint8_t output_period[], int num_samples_per_period, int num_periods) {
    // clear buffers
    for (int j = 0; j < num_samples_per_period; j++) {
        input_period[j] = 0;
        output_period[j] = 0;
    }
    
    int sum_input;
    int sum_output;
    // average over all periods
    for (int j = 0; j < num_samples_per_period; j++) {
        sum_input = 0;
        sum_output = 0;

        for (int i = 0; i < num_periods; i++) {
            sum_input = sum_input + samples[2*(num_samples_per_period*i + j)];
            sum_output = sum_output + samples[2*(num_samples_per_period*i + j) + 1];
        }

        input_period[j] = (uint8_t) round((float) sum_input / num_periods);
        output_period[j] = (uint8_t) round((float) sum_output / num_periods);
    }
}

void samples_to_voltages(uint8_t samples[], double voltages[], int num_samples) {
    for (int i = 0; i < num_samples; i++) {
        voltages[i] = ADC_CONVERT * samples[i];
    }
}

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