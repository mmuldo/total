#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"   
#include "pico/multicore.h"
#include "hardware/watchdog.h"
#include "hardware/dma.h"
#include "hardware/adc.h"

// libraries in this project
#include "i2c.h"
#include "adc.h"
//#include "pwm.h"
//#include "sine.h"
#include "dmapwm.h"


// arbitrary length of time to pause while waiting for things to initialize
#define PAUSE_MS 10

// amount of time to wait for signals to reach steady-state
#define SIGNAL_STEADY_WAIT_TIME_MS 1000

// amount of time watchdog should wait before rebooting system
#define WATCHDOG_SYSTEM_REBOOT_WAIT_TIME_MS 1000


// unit conversions
#define S_TO_US 1000000

#define SERIAL_IO_WAIT_TIME_US 5*60*1000000
#define MAX_INT_LENGTH 6


// buffer in which to collect samples
uint8_t samples[TOTAL_NUM_SAMPLES];

/*
 * reads from serial i/o until '\n' is encounted, then returns
 * the entire string (besides the '\n') that was read
 *
 * Returns
 * -------
 *  const char*
 *      string that was read
 */
uint32_t read_int_from_serial() {
    int16_t character;
    char str[MAX_INT_LENGTH];
    int index = 0;

    character = getchar_timeout_us(SERIAL_IO_WAIT_TIME_US);
    while(character != '\n') {
        if (character != PICO_ERROR_TIMEOUT) {
            str[index++] = character;
        }
        character = getchar_timeout_us(SERIAL_IO_WAIT_TIME_US);
    }

    return atoi(str);
}

float read_frequency_from_serial() {
    float frequency = 0;
    int16_t character;
    
    character = getchar_timeout_us(SERIAL_IO_WAIT_TIME_US);
    while(character != 'a') {
        if (character != PICO_ERROR_TIMEOUT) {
            frequency = 10*frequency + character - '0';
        }
        character = getchar_timeout_us(SERIAL_IO_WAIT_TIME_US);
    }

    return frequency;
}

/*
 * code that core 1 will run
 *
 * generates a sine wave using pwm
 */
// void core1_entry() {
//     // get frequency from core 0
//     uint32_t sine_freq = multicore_fifo_pop_blocking();

//     generate_sine_wave(sine_freq);
// }


int main(void) {
    stdio_init_all();

    set_sys_clock_khz(CLK_KHZ, true);

    // initialize pwm pin
    gpio_set_function(INPUT_SIGNAL_PIN, GPIO_FUNC_PWM);

    // get some dma channels
    int pwm_dma_channel = dma_claim_unused_channel(true);
    int reset_dma_channel = dma_claim_unused_channel(true);
    int adc_dma_channel = dma_claim_unused_channel(true);

    //init_i2c();
    float sine_frequency = read_frequency_from_serial();
    //float sine_frequency = 2000;

    generate_sine_wave(INPUT_SIGNAL_PIN, sine_frequency, pwm_dma_channel, reset_dma_channel);
    // init_adc_and_dma(sine_frequency, samples, adc_dma_channel);
    // sample_signals(samples, adc_dma_channel);
    // dma_start_channel_mask(1u << adc_dma_channel);
    // adc_run(true);
    // dma_channel_wait_for_finish_blocking(adc_dma_channel);

    //sleep_ms(1);
    while(true) {
        tight_loop_contents();
    }

    while (true) {
        float sine_frequency = read_frequency_from_serial();
        change_sine_wave(INPUT_SIGNAL_PIN, sine_frequency, pwm_dma_channel, reset_dma_channel);
        // init_adc_and_dma(sine_frequency, samples, adc_dma_channel);
        // sample_signals(samples, adc_dma_channel);
    }

//     // set system clock
//     set_sys_clock_khz(CLK_KHZ, true); 

//     // for reading temperature and pressure
//     double temperature, pressure;

//     while(true) {
//         // get sine frequency from serial input
//         uint32_t sine_frequency = read_int_from_serial();

//         // read temperature and pressure
//         get_temperature_and_pressure(&temperature, &pressure);

//         // restart core 1 send frequency to core 1
//         multicore_reset_core1();
//         multicore_launch_core1(core1_entry);
//         multicore_fifo_push_blocking(sine_frequency);

//         // wait for signals to achieve steady state before taking readings
//         sleep_ms(SIGNAL_STEADY_WAIT_TIME_MS);

//         init_adc_and_dma(sine_frequency, samples);
//         sample_signals(samples);
//         printf("%.3f\n", temperature);
//         // data tends to corrupt if we send to serial i/o too fast,
//         // so this is just to make sure it all gets printed cleanly
//         sleep_ms(1);
//         printf("%.3f\n", pressure);
//         sleep_ms(1);
//     }
}
