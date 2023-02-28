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

#include "sine.h"

// buffer holding sine table
int *sine_table;

// length of sine table
int l;

// initialize sine table index
int sine_position = 0;

int duty_cycle(int index, int length, double amplitude) {
    return (int) round(length*((amplitude/VDD)*sin(2 * PI * index / length) + 1));
}

int length(int frequency) {
    return (int) sqrt(CLK_KHZ * KHZ_TO_HZ / (2 * frequency << SAMPLE_RATE_FACTOR_SHIFT));
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

    if (sine_position < (l<<SAMPLE_RATE_FACTOR_SHIFT) - 1) { 
        // set pwm level 
        pwm_set_gpio_level(INPUT_SIGNAL_PIN, sine_table[sine_position>>SAMPLE_RATE_FACTOR_SHIFT]);  
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
void generate_sine_wave(uint32_t sine_frequency) {
    l = length(sine_frequency);
    sine_table = malloc(l*sizeof(int));

    // generate sine table
    for (int i = 0; i < l; i++) {
        sine_table[i] = duty_cycle(i, l, AMPLITUDE);
    }

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
    pwm_config_set_clkdiv(&config, 1.0); 
    pwm_config_set_wrap(&config, 2*l); 
    pwm_init(pin_slice, &config, true);

    // set initial pwm level
    pwm_set_gpio_level(INPUT_SIGNAL_PIN, 0);

    while(1) {
        __wfi(); // Wait for Interrupt
    }

    free(sine_table);
}
