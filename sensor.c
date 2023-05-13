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
#include "linalg.h"
#include "sine_analysis.h"
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

// global pointer to sine_table (needs to be global so that dma channels have access)
uint32_t * sine_table;
uint32_t ** sine_table_pointer;
// sine table length also global just for consistency
int sine_table_length;
// buffer in which to collect samples
uint8_t samples[TOTAL_NUM_SAMPLES];
// for printing to serial i/o
char samples_string[TOTAL_NUM_SAMPLES+1];

uint32_t input_period[NUM_SAMPLES_PER_PERIOD];
uint32_t output_period[NUM_SAMPLES_PER_PERIOD];
char periods_string[2*NUM_SAMPLES_PER_PERIOD+1];
double sine_period[1000];

float read_frequency_from_serial() {
    float frequency = 0;
    int16_t character;
    
    character = getchar_timeout_us(SERIAL_IO_WAIT_TIME_US);
    while(character != '\n') {
        if (character != PICO_ERROR_TIMEOUT) {
            frequency = 10*frequency + character - '0';
        }
        character = getchar_timeout_us(SERIAL_IO_WAIT_TIME_US);
    }

    return frequency;
}

void average_period(uint8_t samples[], uint32_t input_period[], uint32_t output_period[], char periods_string[]) {
    // clear buffers
    for (int j = 0; j < NUM_SAMPLES_PER_PERIOD; j++) {
        input_period[j] = 0;
        output_period[j] = 0;
    }
    
    // sum over all periods
    for (int i = 0; i < NUM_PERIODS; i++) {
        for (int j = 0; j < NUM_SAMPLES_PER_PERIOD; j++) {
            input_period[j] = input_period[j] + samples[2*(NUM_SAMPLES_PER_PERIOD*i + j)];
            output_period[j] = output_period[j] + samples[2*(NUM_SAMPLES_PER_PERIOD*i + j) + 1];
        }
    }
    
    // average over all periods
    for (int j = 0; j < NUM_SAMPLES_PER_PERIOD; j++) {
        periods_string[j] = (int) round((float) input_period[j] / NUM_PERIODS);
        periods_string[NUM_SAMPLES_PER_PERIOD + j] = (int) round((float) output_period[j] / NUM_PERIODS);
    }
    
    // ensure none are '\0' (would end string)
    for (int k = 0; k < 2*NUM_SAMPLES_PER_PERIOD; k++) {
        if (periods_string[k] == 0) {
            periods_string[k] = 1;
        }
    }
    
    // end string
    periods_string[2*NUM_SAMPLES_PER_PERIOD] = '\0';
}

void print_samples(uint8_t samples[], char samples_string[]) {
    // copy samples over
    for (int i = 0; i < TOTAL_NUM_SAMPLES; i++) {
        if (samples[i] == 0) {
            samples_string[i] = 1;
        } else {
            samples_string[i] = samples[i];
        }
    }
    // append string terminator
    samples_string[TOTAL_NUM_SAMPLES] = '\0';

    printf(samples_string);
}

int main(void) {
    stdio_init_all();
    init_i2c();
    double temperature, pressure;

    set_sys_clock_khz(CLK_KHZ, true);

    // read in initial sine frequency
    //float sine_frequency = read_frequency_from_serial();
    float sine_frequency = 1000.0;

    // get sine table given frequency
    sine_table_length = highest_frequency_to_table_length(sine_frequency);
    sine_table = generate_sine_table(sine_table_length, AMPLITUDE);
    sine_table_pointer = &sine_table;

    // get some dma channels we can use
    int pwm_dma_channel = dma_claim_unused_channel(true);
    int reset_dma_channel = dma_claim_unused_channel(true);
    int adc_dma_channel = dma_claim_unused_channel(true);

    // initialize adc and the adc dma channel
    init_adc(sine_frequency);
    init_adc_dma(adc_dma_channel, samples);

    // initialize pwm pin
    gpio_set_function(INPUT_SIGNAL_PIN, GPIO_FUNC_PWM);

    // initialize pwm and the pwm dma channels
    int pwm_pin_slice = pwm_gpio_to_slice_num(INPUT_SIGNAL_PIN);
    init_pwm(pwm_pin_slice, sine_table_length);
    init_pwm_dma(pwm_dma_channel, reset_dma_channel, pwm_pin_slice, sine_table, sine_table_pointer, sine_table_length);

    // start the pwm dma channels
    dma_channel_start(pwm_dma_channel);
    // wait for steady state
    sleep_ms(1000);

    sample_signals(adc_dma_channel);

    // for (int i = 0; i < TOTAL_NUM_SAMPLES; i += 2) {
    //     printf("%d,", samples[i]);
    // }
    //print_samples(samples, samples_string);
    average_period(samples, input_period, output_period, periods_string);
    printf(periods_string);
    
    if (true) {
        get_temperature_and_pressure(&temperature, &pressure);
        printf("%.3f\n", temperature);
        sleep_ms(1);
        printf("%.3f\n", pressure);
    }

    while (true) {
        // reset adc dma channel
        dma_channel_set_write_addr(adc_dma_channel, samples, false);

        // read in new sine frequency
        //sine_frequency = read_frequency_from_serial();
        sine_frequency = sine_frequency + 100;
        // sample frequency must be twice as fast because we're sampling 2 signals
        float sample_frequency = 2*NUM_SAMPLES_PER_PERIOD * sine_frequency;
        adc_set_clkdiv(ADC_CLOCK_FREQUENCY_HZ / sample_frequency);

        // pause current running pwm (and thus also pwm dma channels)
        pwm_set_enabled(pwm_pin_slice, false);

        // generate new sine table
        free(sine_table);
        sine_table_length = highest_frequency_to_table_length(sine_frequency);
        sine_table = generate_sine_table(sine_table_length, AMPLITUDE);
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

        // for (int i = 0; i < TOTAL_NUM_SAMPLES; i += 2) {
        //     printf("%d,", samples[i]);
        // }
        //print_samples(samples, samples_string);
        average_period(samples, input_period, output_period, periods_string);
        printf(periods_string);
        
        if (true) {
            get_temperature_and_pressure(&temperature, &pressure);
            printf("%.3f\n", temperature);
            sleep_ms(1);
            printf("%.3f\n", pressure);
        }
    }
}
