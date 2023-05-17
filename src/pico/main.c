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

// number of samples in one period of a sine wave
#define NUM_SAMPLES_PER_PERIOD 25

// number of periods to sample for
#define NUM_PERIODS 10

// total number of samples that will be collected
#define TOTAL_NUM_SAMPLES 2*NUM_PERIODS*(NUM_SAMPLES_PER_PERIOD)

// frequency at which dedicated adc clock is running at
#define ADC_CLOCK_FREQUENCY_HZ 48000000

// arbitrarily long time to wait to receive characters over serial i/o
#define SERIAL_IO_WAIT_TIME_US 5*60*1000000

// time to wait for waveforms to reach steady state
#define STEADY_STATE_WAIT_TIME_MS 500

// value of feedback resistor in transimpedence amplifier (in Ohms)
#define RF_OHMS 10000.0

// number of frequencies to test impedence on during spectroscopy
#define NUM_FREQUENCIES_FOR_SPECTROSCOPY 19

// frequencies to test impedence on during spectroscopy
double FREQUENCIES_FOR_SPECTROSCOPY[] = {
    100,
    200,
    300,
    400,
    500,
    600,
    700,
    800,
    900,
    1000,
    2000,
    3000,
    4000,
    5000,
    6000,
    7000,
    8000,
    9000,
    10000
};

// buffer for storing one period of a sine wave; values are pwm counter compare values
uint32_t * sine_table;
// pointer to sine_table (needs to be global so that dma channels have access)
uint32_t ** sine_table_pointer;
// sine table length also global just for consistency
int sine_table_length;

/// @brief reads an integer frequency from serial i/o
/// @return the frequency that was read in
double read_frequency_from_serial() {
    double frequency = 0;
    int16_t character;
    
    character = getchar_timeout_us(SERIAL_IO_WAIT_TIME_US);
    // assume frequency will be terminated with a comma
    while(character != ',') {
        if (character != PICO_ERROR_TIMEOUT) {
            frequency = 10*frequency + character - '0';
        }
        character = getchar_timeout_us(SERIAL_IO_WAIT_TIME_US);
    }

    return frequency;
}

/// @brief reads an amplitude (0 < amplitude < 10) from serial i/o
/// @return the amplitude that was read in
double read_amplitude_from_serial() {
    double amplitude = 0;
    double order = 10;
    int16_t character;

    character = getchar_timeout_us(SERIAL_IO_WAIT_TIME_US);
    // assume amplitude will be terminated with a comma
    while(character != ',') {
        if (character != PICO_ERROR_TIMEOUT && character != '.') {
            amplitude = 10*amplitude + character - '0';
            order /= 10;
        }
        character = getchar_timeout_us(SERIAL_IO_WAIT_TIME_US);
    }

    return amplitude * order;
}

/// @brief initializes peripherals and measures impedence by generating vin and vout signal and comparing them
/// @param frequency the frequency of vin and vout
/// @param amplitude the amplitude of vin
/// @param pwm_pin the pin to generate the pwm signal (for vin) on
/// @param pwm_dma_channel the dma channel for writing samples to the pwm counter compare register
/// @param reset_dma_channel the dma channel for resetting the pwm dma channel
/// @param adc_dma_channel the dma channel for moving adc samples to a buffer
/// @param input_period buffer for storing one period of vin (adc samples)
/// @param output_period buffer for storing one period of vout (adc samples)
void measure_vin_vout_initial(
    double frequency,
    double amplitude,
    int pwm_pin,
    int pwm_dma_channel,
    int reset_dma_channel,
    int adc_dma_channel,
    uint8_t input_period[],
    uint8_t output_period[]
) {
    // buffer in which to collect samples
    uint8_t samples[TOTAL_NUM_SAMPLES];

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
    sleep_ms(STEADY_STATE_WAIT_TIME_MS);

    sample_signals(adc_dma_channel);

    average_period(samples, input_period, output_period, NUM_SAMPLES_PER_PERIOD, NUM_PERIODS);
}

/// @brief measures impedence by generating vin and vout signal and comparing them
/// @param frequency the frequency of vin and vout
/// @param amplitude the amplitude of vin
/// @param pwm_pin the pin to generate the pwm signal (for vin) on
/// @param pwm_dma_channel the dma channel for writing samples to the pwm counter compare register
/// @param reset_dma_channel the dma channel for resetting the pwm dma channel
/// @param adc_dma_channel the dma channel for moving adc samples to a buffer
/// @param input_period buffer for storing one period of vin (adc samples)
/// @param output_period buffer for storing one period of vout (adc samples)
void measure_vin_vout_subsequent(
    double frequency,
    double amplitude,
    int pwm_pin,
    int pwm_dma_channel,
    int reset_dma_channel,
    int adc_dma_channel,
    uint8_t input_period[],
    uint8_t output_period[]
) {
    // buffer in which to collect samples
    uint8_t samples[TOTAL_NUM_SAMPLES];
    
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
    sleep_ms(STEADY_STATE_WAIT_TIME_MS);

    sample_signals(adc_dma_channel);

    average_period(samples, input_period, output_period, NUM_SAMPLES_PER_PERIOD, NUM_PERIODS);
}

/// @brief measures temperature and pressure and prints them to serial i/o
void print_temperature_and_pressure() {
    double temperature, pressure;

    get_temperature_and_pressure(&temperature, &pressure);
    // send to serial i/o as "TEMP,PRES"
    printf("%.3f,", temperature);
    printf("%.3f", pressure);
}

/// @brief collects one period of vin and vout and prints them to serial i/o
/// @param frequency the frequency of vin and vout
/// @param amplitude the amplitude of vin
/// @param pwm_pin the pin to generate the pwm signal (for vin) on
/// @param pwm_dma_channel the dma channel for writing samples to the pwm counter compare register
/// @param reset_dma_channel the dma channel for resetting the pwm dma channel
/// @param adc_dma_channel the dma channel for moving adc samples to a buffer
/// @param input_period buffer for storing one period of vin (adc samples)
/// @param output_period buffer for storing one period of vout (adc samples)
void print_vin_vout_single_frequency(
    double frequency,
    double amplitude,
    int pwm_pin,
    int pwm_dma_channel,
    int reset_dma_channel,
    int adc_dma_channel,
    uint8_t input_period[],
    uint8_t output_period[]
) {
    // buffer to store all samples for printing to serial io
    char periods_string[2*NUM_SAMPLES_PER_PERIOD+1];

    measure_vin_vout_subsequent(
        frequency,
        amplitude,
        pwm_pin,
        pwm_dma_channel,
        reset_dma_channel,
        adc_dma_channel,
        input_period,
        output_period
    );

    // copy samples to buffer;
    // since the adc takes 8-bit readings, and ASCII characters are encoded as
    // 8-bit numbers, we can just print the samples directly
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

/// @brief measures impedence over a range of frequencies
/// @param frequencies the frequencies to measure impedence at
/// @param num_frequencies the number of frequencies to measure impedence at
/// @param amplitude amplitude of vin
/// @param pwm_pin the pin to generate the pwm signal (for vin) on
/// @param pwm_dma_channel the dma channel for writing samples to the pwm counter compare register
/// @param reset_dma_channel the dma channel for resetting the pwm dma channel
/// @param adc_dma_channel the dma channel for moving adc samples to a buffer
/// @param input_period buffer for storing one period of vin (adc samples)
/// @param output_period buffer for storing one period of vout (adc samples)
void print_impedence_spectrum(
    double frequencies[],
    int num_frequencies,
    double amplitude,
    int pwm_pin,
    int pwm_dma_channel,
    int reset_dma_channel,
    int adc_dma_channel,
    uint8_t input_period[],
    uint8_t output_period[]
) {
    // buffer for storing one period of vin in V
    double vin[NUM_SAMPLES_PER_PERIOD];
    // buffer for storing one period of vout in V
    double vout[NUM_SAMPLES_PER_PERIOD];
    // characteristics of vin
    sine vin_parameters;
    // characteristics of vout
    sine vout_parameters;
    // impedence
    complex Z;
    // buffer for storing impedence magnitudes
    double magnitude_spectrum[NUM_FREQUENCIES_FOR_SPECTROSCOPY];
    // buffer for storing phase magnitudes
    double phase_spectrum[NUM_FREQUENCIES_FOR_SPECTROSCOPY];

    for (int i = 0; i < num_frequencies; i++) {
        measure_vin_vout_subsequent(
            frequencies[i],
            amplitude,
            pwm_pin,
            pwm_dma_channel,
            reset_dma_channel,
            adc_dma_channel,
            input_period,
            output_period
        );

        samples_to_voltages(input_period, vin, NUM_SAMPLES_PER_PERIOD);
        samples_to_voltages(output_period, vout, NUM_SAMPLES_PER_PERIOD);

        vin_parameters = characterize(vin, frequencies[i], NUM_SAMPLES_PER_PERIOD);
        vout_parameters = characterize(vout, frequencies[i], NUM_SAMPLES_PER_PERIOD);

        Z = impedence(vin_parameters, vout_parameters, RF_OHMS);
        magnitude_spectrum[i] = Z.magnitude;
        phase_spectrum[i] = Z.phase;

    }

    // print stuff;
    // samples are seperated by commas
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
    // for reading in command
    //  t: get temperature and pressure
    //  f: get impedence for a single frequency
    //  s: get impedence spectrum
    int16_t command;
    // frequency of vin and vout sine waves
    double frequency;
    // amplitude of vin sine wave
    double amplitude;
    // buffer reading in one period of vin
    uint8_t input_period[NUM_SAMPLES_PER_PERIOD];
    // buffer reading in one period of vout
    uint8_t output_period[NUM_SAMPLES_PER_PERIOD];

    stdio_init_all();

    set_sys_clock_khz(CLK_KHZ, true);
    init_i2c();

    // get some dma channels we can use
    int pwm_dma_channel = dma_claim_unused_channel(true);
    int reset_dma_channel = dma_claim_unused_channel(true);
    int adc_dma_channel = dma_claim_unused_channel(true);

    // do a dummy run, just to get things initialized
    frequency = 1000.0;
    amplitude = 0.7;
    measure_vin_vout_initial(
        frequency,
        amplitude,
        INPUT_SIGNAL_PIN,
        pwm_dma_channel,
        reset_dma_channel,
        adc_dma_channel,
        input_period,
        output_period
    );

    while (true) {
        command = getchar_timeout_us(SERIAL_IO_WAIT_TIME_US);
        switch (command) {
            case 't':
                print_temperature_and_pressure();
                break;
            case 's':
                amplitude = read_amplitude_from_serial();
                print_impedence_spectrum(
                    FREQUENCIES_FOR_SPECTROSCOPY,
                    NUM_FREQUENCIES_FOR_SPECTROSCOPY,
                    amplitude,
                    INPUT_SIGNAL_PIN,
                    pwm_dma_channel,
                    reset_dma_channel,
                    adc_dma_channel,
                    input_period,
                    output_period
                );
                break;
            case 'f':
                frequency = read_frequency_from_serial();
                amplitude = read_amplitude_from_serial();
                print_vin_vout_single_frequency(
                    frequency,
                    amplitude,
                    INPUT_SIGNAL_PIN,
                    pwm_dma_channel,
                    reset_dma_channel,
                    adc_dma_channel,
                    input_period,
                    output_period
                );
                break;
        }
    }
}