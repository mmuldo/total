#include <stdio.h>
#include "pico/stdlib.h"   
#include "pico/multicore.h"
#include "hardware/irq.h"  
#include "hardware/pwm.h"  
#include "hardware/sync.h" 
#include "hardware/gpio.h"
#include "hardware/adc.h"
 
// pins to switch between for adc round robin sampling
#define ADC_PIN_MASK 0b0011

// starting pin for adc round robin sampling
#define ADC_FIRST_PIN 0

// second pin for adc round robin sampling
#define ADC_SECOND_PIN 1

// the first gpio pin where the adc pins are located
#define ADC_GPIO_PINS 26

// adc voltage at max reading
#define ADC_VREF 3.3

// max adc reading
#define ADC_RANGE (1 << 12)

// multiply to convert adc reading to voltage
#define ADC_CONVERT (ADC_VREF / (ADC_RANGE - 1))

// adc sampling rate in hz
#define ADC_SAMPLE_RATE_HZ 50000

// amount of time adc sampling loop takes in us
#define ADC_LOOP_TIME_US 6

// pin at which sine wave will be emitted
#define INPUT_SIGNAL_PIN 0


// amount to bit shift sample position such that the sample rate is reduced
// by a factor. explicitly, decreases the sample rate by a factor
// of 2^SAMPLE_RATE_FACTOR_SHIFT. e.g. to decrease the sample rate by a factor of 8,
// set this to 3.
#define SAMPLE_RATE_FACTOR_SHIFT 3

// frequency (in kHz) to set clock at
#define CLK_KHZ 250000

// number of clocks before pwm wraps and sets the interrupt
#define WRAP 250


// arbitrary length of time to pause while waiting for things to initialize
#define PAUSE_MS 10

// amount of time (in microseconds) to wait for user to open up serial i/o before program gives up.
// setting this to an arbitrarily long time should be sufficient.
#define SERIAL_IO_INIT_WAIT_TIME 100000000

// amount of time to wait between serial i/o directives
#define SERIAL_IO_WAIT_TIME_MS 1

// unit conversions
#define S_TO_US 1000000
#define KHZ_TO_HZ 1000.0

/* 
 * sine wave sample table
 */
#include "sine.h"

// initialize sine table index
int sine_position = 0;
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

/*
 * samples signals on adc pins and outputs them to serial i/o
 */
void sample_signals(uint32_t sine_freq) {
    // initialize adc pins
    adc_init();
    adc_gpio_init(ADC_GPIO_PINS + ADC_FIRST_PIN);
    adc_gpio_init(ADC_GPIO_PINS + ADC_SECOND_PIN);
    adc_set_round_robin(ADC_PIN_MASK);
    adc_select_input(ADC_FIRST_PIN);

    // number of samples in one period
    uint num_samples = ADC_SAMPLE_RATE_HZ / sine_freq;
    // amount to sleep in adc loop in order to sample at the correct frequency
    uint adc_loop_period_us = S_TO_US / ADC_SAMPLE_RATE_HZ - ADC_LOOP_TIME_US;

    // initialize sample buffers
    float vin[num_samples];
    float vout[num_samples];

    sleep_ms(100*PAUSE_MS);
    for (uint i = 0; i < num_samples; i++) {
        vin[i] = adc_read() * ADC_CONVERT;
        vout[i] = adc_read() * ADC_CONVERT;
        sleep_us(adc_loop_period_us);
    }
    sleep_ms(PAUSE_MS);
    for (uint i = 0; i < num_samples; i++) {
        printf("%.3f\n", vin[i]);
        sleep_ms(SERIAL_IO_WAIT_TIME_MS);
    }
    sleep_ms(PAUSE_MS);
    for (uint i = 0; i < num_samples; i++) {
        printf("%.3f\n", vout[i]);
        sleep_ms(SERIAL_IO_WAIT_TIME_MS);
    }
}

int main(void) {
    stdio_init_all();
    // waits for user to press a character before proceeding with the program.
    // this is required to get my laptop to recognize the serial i/o port.
    // might not be necessary on all laptops.
    getchar_timeout_us(SERIAL_IO_INIT_WAIT_TIME);

    // set system clock
    set_sys_clock_khz(CLK_KHZ, true); 

    // for querying user input on whether or not to run again once complete.
    // default to no
    char run_again = 'n';

    do {
        // launch core 1 code
        multicore_reset_core1();
        multicore_launch_core1(core1_entry);

        // get sine frequency from serial input
        uint32_t sine_freq;
        scanf("%d", &sine_freq);
        // send frequency to core 1
        multicore_fifo_push_blocking(sine_freq);

        sample_signals(sine_freq);

        // query user to run again
        scanf("%c", &run_again);
    } while (run_again == 'y' || run_again == 'Y');

    return 0;
}
