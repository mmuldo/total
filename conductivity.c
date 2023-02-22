#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"   
#include "pico/multicore.h"
#include "hardware/irq.h"  
#include "hardware/pwm.h"  
#include "hardware/sync.h" 
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/watchdog.h"

// libraries in this project
#include "i2c.h"
#include "adc.h"

// pin at which sine wave will be emitted
#define INPUT_SIGNAL_PIN 0


// amount to bit shift sample position such that the sample rate is reduced
// by a factor. explicitly, decreases the sample rate by a factor
// of 2^SAMPLE_RATE_FACTOR_SHIFT. e.g. to decrease the sample rate by a factor of 8,
// set this to 3.
#define SAMPLE_RATE_FACTOR_SHIFT 2

// frequency (in kHz) to set clock at
#define CLK_KHZ 250000

// number of clocks before pwm wraps and sets the interrupt
#define WRAP 100


// arbitrary length of time to pause while waiting for things to initialize
#define PAUSE_MS 10

// amount of time to wait for signals to reach steady-state
#define SIGNAL_STEADY_WAIT_TIME_MS 1000

// amount of time watchdog should wait before rebooting system
#define WATCHDOG_SYSTEM_REBOOT_WAIT_TIME_MS 1000


// unit conversions
#define S_TO_US 1000000
#define KHZ_TO_HZ 1000.0

#define SERIAL_IO_WAIT_TIME_US 5*60*1000000
#define MAX_INT_LENGTH 6


// buffer in which to collect samples
uint8_t samples[TOTAL_NUM_SAMPLES];

/* 
 * sine wave sample table
 */
#include "sine.h"

// initialize sine table index
int sine_position = 0;

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

/*
 * PWM Interrupt Handler which outputs PWM level and advances the 
 * current sample. 
 * 
 * We repeat the same value for 2^SAMPLE_RATE_FACTOR_SHIFT cycles. This means 
 * the sample rate is adjusted by factor of 2^SAMPLE_RATE_FACTOR_SHIFT.
 */
void pwm_interrupt_handler() {
    // clear interrupt flag
    pwm_clear_irq(pwm_gpio_to_slice_num(INPUT_SIGNAL_PIN));    

    if (sine_position < (SINE_TABLE_LENGTH<<SAMPLE_RATE_FACTOR_SHIFT) - 1) { 
        // set pwm level 
        pwm_set_gpio_level(INPUT_SIGNAL_PIN, SINE_TABLE[sine_position>>SAMPLE_RATE_FACTOR_SHIFT]);  
        sine_position++;
    } else {
        // reset to start
        sine_position = 0;
    }
}

/*
 * code that core 1 will run
 *
 * generates a sine wave using pwm
 */
void core1_entry() {
    // get frequency from core 0
    uint32_t sine_freq = multicore_fifo_pop_blocking();

    // initialize pwm pin
    gpio_set_function(INPUT_SIGNAL_PIN, GPIO_FUNC_PWM);
    
    int pin_slice = pwm_gpio_to_slice_num(INPUT_SIGNAL_PIN);

    // Setup PWM interrupt to fire when PWM cycle is complete
    pwm_clear_irq(pin_slice);
    pwm_set_irq_enabled(pin_slice, true);
    // use interrupt handler function defined above
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_interrupt_handler); 
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // initialize pwm config: clock division and wrap
    pwm_config config = pwm_get_default_config();
    float clkdiv = CLK_KHZ * KHZ_TO_HZ / (WRAP * (SINE_TABLE_LENGTH<<SAMPLE_RATE_FACTOR_SHIFT) * sine_freq);
    pwm_config_set_clkdiv(&config, clkdiv); 
    pwm_config_set_wrap(&config, WRAP); 
    pwm_init(pin_slice, &config, true);

    // set initial pwm level
    pwm_set_gpio_level(INPUT_SIGNAL_PIN, 0);

    while(1) {
        __wfi(); // Wait for Interrupt
    }
}


int main(void) {
    stdio_init_all();
    init_i2c();

    // set system clock
    set_sys_clock_khz(CLK_KHZ, true); 

    // for reading temperature and pressure
    double temperature, pressure;

    while(true) {
        // get sine frequency from serial input
        uint32_t sine_frequency = read_int_from_serial();

        // read temperature and pressure
        get_temperature_and_pressure(&temperature, &pressure);

        // restart core 1 send frequency to core 1
        multicore_reset_core1();
        multicore_launch_core1(core1_entry);
        multicore_fifo_push_blocking(sine_frequency);

        // wait for signals to achieve steady state before taking readings
        sleep_ms(SIGNAL_STEADY_WAIT_TIME_MS);

        init_adc_and_dma(sine_frequency, samples);
        sample_signals(samples);
        printf("%.3f\n", temperature);
        // data tends to corrupt if we send to serial i/o too fast,
        // so this is just to make sure it all gets printed cleanly
        sleep_ms(1);
        printf("%.3f\n", pressure);
        sleep_ms(1);
    }
}
