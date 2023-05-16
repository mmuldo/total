#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"   
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"  
#include "hardware/gpio.h"

#include "i2c.h"
#include "sine.h"
#include "adc.h"
#include "pwm.h"

// pin at which sine wave will be emitted
#define INPUT_SIGNAL_PIN 0

// frequency (in kHz) to set clock at
#define CLK_KHZ 250000
#define KHZ_TO_HZ 1000.0

// saturation voltage in volts
#define VDD 3.3

// for calculating sine
#define PI 3.1415926535

#define AMPLITUDE 0.75

// number of samples in one period of a sine wave
#define NUM_SAMPLES_PER_PERIOD 25

// number of periods to sample for
#define NUM_PERIODS 10

// total number of samples that will be collected
// NOTE: it appears that the maximum number of samples that can
// be fed to the serial i/o at one time is 594, so if you don't
// plan on reading the samples from serial i/o in chunks, then
// don't let this constant exceed 594
#define TOTAL_NUM_SAMPLES 2*NUM_PERIODS*(NUM_SAMPLES_PER_PERIOD)

// frequency at which dedicated adc clock is running at
#define ADC_CLOCK_FREQUENCY_HZ 48000000

#define SERIAL_IO_WAIT_TIME_US 5*60*1000000

#define NUM_FREQUENCIES_FOR_SPECTROSCOPY 10

#define RF 10000.0

double frequencies_for_spectroscopy[] = {100, 300, 500, 700, 900, 1000, 3000, 5000, 7000, 9000};
double magnitude_spectrum[10];
double phase_spectrum[10];

// global pointer to sine_table (needs to be global so that dma channels have access)
uint32_t * sine_table;
uint32_t ** sine_table_pointer;
// sine table length also global just for consistency
int sine_table_length;
// buffer in which to collect samples
uint8_t samples[TOTAL_NUM_SAMPLES];
// for printing to serial i/o
char samples_string[TOTAL_NUM_SAMPLES+1];

uint8_t input_period[NUM_SAMPLES_PER_PERIOD];
uint8_t output_period[NUM_SAMPLES_PER_PERIOD];
double vin[NUM_SAMPLES_PER_PERIOD];
double vout[NUM_SAMPLES_PER_PERIOD];
char periods_string[2*NUM_SAMPLES_PER_PERIOD+1];
double sine_period[1000];

double read_frequency_from_serial() {
    double frequency = 0;
    int16_t character;
    
    character = getchar_timeout_us(SERIAL_IO_WAIT_TIME_US);
    while(character != ',') {
        if (character != PICO_ERROR_TIMEOUT) {
            frequency = 10*frequency + character - '0';
        }
        character = getchar_timeout_us(SERIAL_IO_WAIT_TIME_US);
    }

    return frequency;
}

double read_amplitude_from_serial() {
    double amplitude = 0;
    double order = 10;
    int16_t character;

    character = getchar_timeout_us(SERIAL_IO_WAIT_TIME_US);
    while(character != ',') {
        if (character != PICO_ERROR_TIMEOUT && character != '.') {
            amplitude = 10*amplitude + character - '0';
            order /= 10;
        }
        character = getchar_timeout_us(SERIAL_IO_WAIT_TIME_US);
    }

    return amplitude * order;
}

void measure_vin_vout_initial(
    double frequency,
    double amplitude,
    int pwm_pin,
    int pwm_dma_channel,
    int reset_dma_channel,
    int adc_dma_channel
) {
    // get sine table given frequency
    sine_table_length = highest_frequency_to_table_length(frequency);
    sine_table = generate_sine_table(sine_table_length, amplitude);
    sine_table_pointer = &sine_table;

    // initialize adc and the adc dma channel
    init_adc(frequency);
    init_adc_dma(adc_dma_channel, samples);

    // initialize pwm pin
    gpio_set_function(pwm_pin, GPIO_FUNC_PWM);

    // initialize pwm and the pwm dma channels
    int pwm_pin_slice = pwm_gpio_to_slice_num(pwm_pin);
    init_pwm(pwm_pin_slice, sine_table_length);
    init_pwm_dma(pwm_dma_channel, reset_dma_channel, pwm_pin_slice, sine_table, sine_table_pointer, sine_table_length);

    // start the pwm dma channels
    dma_channel_start(pwm_dma_channel);
    // wait for steady state
    sleep_ms(1000);

    sample_signals(adc_dma_channel);

    average_period(samples, input_period, output_period, NUM_SAMPLES_PER_PERIOD, NUM_PERIODS);
}

void measure_vin_vout_subsequent(
    double frequency,
    double amplitude,
    int pwm_pin,
    int pwm_dma_channel,
    int reset_dma_channel,
    int adc_dma_channel
) {
    // reset adc dma channel
    dma_channel_set_write_addr(adc_dma_channel, samples, false);

    // sample frequency must be twice as fast because we're sampling 2 signals
    float sample_frequency = 2*NUM_SAMPLES_PER_PERIOD * frequency;
    adc_set_clkdiv(ADC_CLOCK_FREQUENCY_HZ / sample_frequency);

    int pwm_pin_slice = pwm_gpio_to_slice_num(pwm_pin);

    // pause current running pwm (and thus also pwm dma channels)
    pwm_set_enabled(pwm_pin_slice, false);

    // generate new sine table
    free(sine_table);
    sine_table_length = highest_frequency_to_table_length(frequency);
    sine_table = generate_sine_table(sine_table_length, amplitude);
    sine_table_pointer = &sine_table;

    // re-init pwm with new parameters
    init_pwm(pwm_pin_slice, sine_table_length);

    // reconfigure pwm dma stuff
    dma_channel_set_read_addr(pwm_dma_channel, sine_table, false);
    dma_channel_set_trans_count(pwm_dma_channel, sine_table_length, false);
    dma_channel_set_read_addr(reset_dma_channel, sine_table_pointer, false);
    // resume pwm (and thus pwm dma channels)
    pwm_set_enabled(pwm_pin_slice, true);
    // wait for steady state
    sleep_ms(1000);

    sample_signals(adc_dma_channel);

    average_period(samples, input_period, output_period, NUM_SAMPLES_PER_PERIOD, NUM_PERIODS);
}

void print_temperature_and_pressure() {
    double temperature, pressure;

    get_temperature_and_pressure(&temperature, &pressure);
    printf("%.3f,", temperature);
    printf("%.3f", pressure);
}

void print_vin_vout_single_frequency(
    double frequency,
    double amplitude,
    int pwm_pin,
    int pwm_dma_channel,
    int reset_dma_channel,
    int adc_dma_channel
) {
    measure_vin_vout_subsequent(frequency, amplitude, pwm_pin, pwm_dma_channel, reset_dma_channel, adc_dma_channel);

    // copy samples to buffer
    for (int i = 0; i < NUM_SAMPLES_PER_PERIOD; i++) {
        periods_string[i] = input_period[i];
        periods_string[NUM_SAMPLES_PER_PERIOD + i] = output_period[i];
    }

    // ensure no zero samples (this would prematurely end the string)
    for (int i = 0; i < 2*NUM_SAMPLES_PER_PERIOD; i++) {
        periods_string[i] = periods_string[i] == '\0' ? 1 : periods_string[i];
    }
    
    // terminate string
    periods_string[2*NUM_SAMPLES_PER_PERIOD] = '\0';

    printf(periods_string);
}

void print_impedence_spectrum(
    double frequencies[],
    int num_frequencies,
    double amplitude,
    int pwm_pin,
    int pwm_dma_channel,
    int reset_dma_channel,
    int adc_dma_channel
) {
    sine vin_parameters;
    sine vout_parameters;
    complex Z;

    for (int i = 0; i < num_frequencies; i++) {
        measure_vin_vout_subsequent(frequencies[i], amplitude, pwm_pin, pwm_dma_channel, reset_dma_channel, adc_dma_channel);

        samples_to_voltages(input_period, vin, NUM_SAMPLES_PER_PERIOD);
        samples_to_voltages(output_period, vout, NUM_SAMPLES_PER_PERIOD);

        vin_parameters = characterize(vin, frequencies[i], NUM_SAMPLES_PER_PERIOD);
        vout_parameters = characterize(vout, frequencies[i], NUM_SAMPLES_PER_PERIOD);

        Z = impedence(vin_parameters, vout_parameters, RF);
        magnitude_spectrum[i] = Z.magnitude;
        phase_spectrum[i] = Z.phase;

    }

    // print stuff
    for (int i = 0; i < num_frequencies; i++) {
        printf("%.3f,", magnitude_spectrum[i]);
    }
    for (int i = 0; i < num_frequencies-1; i++) {
        printf("%.3f,", phase_spectrum[i]);
    }
    // don't include comma in last one
    printf("%.3f", phase_spectrum[num_frequencies-1]);
}

int main(void) {
    int16_t command;
    double frequency;
    double amplitude;

    stdio_init_all();

    set_sys_clock_khz(CLK_KHZ, true);
    init_i2c();

    // get some dma channels we can use
    int pwm_dma_channel = dma_claim_unused_channel(true);
    int reset_dma_channel = dma_claim_unused_channel(true);
    int adc_dma_channel = dma_claim_unused_channel(true);

    // do a dummy run, just to get things initialized
    frequency = 1000.0;
    measure_vin_vout_initial(frequency, AMPLITUDE, INPUT_SIGNAL_PIN, pwm_dma_channel, reset_dma_channel, adc_dma_channel);

    while (true) {
        command = getchar_timeout_us(SERIAL_IO_WAIT_TIME_US);
        switch (command) {
            case 't':
                print_temperature_and_pressure();
                break;
            case 's':
                amplitude = read_amplitude_from_serial();
                print_impedence_spectrum(frequencies_for_spectroscopy, NUM_FREQUENCIES_FOR_SPECTROSCOPY, amplitude, INPUT_SIGNAL_PIN, pwm_dma_channel, reset_dma_channel, adc_dma_channel);
                break;
            case 'f':
                frequency = read_frequency_from_serial();
                amplitude = read_amplitude_from_serial();
                print_vin_vout_single_frequency(frequency, amplitude, INPUT_SIGNAL_PIN, pwm_dma_channel, reset_dma_channel, adc_dma_channel);
                break;
        }
    }
}