#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"   
#include "pico/multicore.h"
#include "hardware/pwm.h"  
#include "hardware/gpio.h"
#include "hardware/dma.h"

#include "sinetables.h"

#define PWM_DMA_CHANNEL 1
#define RESET_DMA_CHANNEL 2

#define INPUT_SIGNAL_PIN 0
#define CLK_KHZ 250000
#define KHZ_TO_HZ 1000
#define PI 3.1415926535
#define VDD 3.3

uint32_t * sine_table;
int sine_table_length;
uint32_t * sine_table_pointer;

uint32_t * generate_sine_table(int length, double amplitude) {
    uint32_t * sine_table = malloc(length*sizeof(uint32_t));
    for (int i = 0; i < length; i++) {
        uint32_t duty_cycle = (uint32_t) round(length*((2*amplitude/VDD)*sin(2 * PI * i / length) + 1));
        sine_table[i] = duty_cycle;
    }

    return sine_table;
}

int highest_frequency_to_table_length(float frequency) {
    return (int) round(sqrt(CLK_KHZ * KHZ_TO_HZ / (2 * frequency)));
}

int main(void) {
    float sine_frequency = 888;

    sine_table_length = highest_frequency_to_table_length(sine_frequency);
    sine_table = generate_sine_table(sine_table_length, 1);
    sine_table_pointer = sine_table;

    set_sys_clock_khz(CLK_KHZ, true);

    // initialize pwm pin
    gpio_set_function(INPUT_SIGNAL_PIN, GPIO_FUNC_PWM);
    
    int pin_slice = pwm_gpio_to_slice_num(INPUT_SIGNAL_PIN);

    // initialize pwm config: clock division and wrap
    pwm_config config = pwm_get_default_config();
    int wrap = 2*sine_table_length;
    pwm_config_set_clkdiv(&config, 1.0); 
    pwm_config_set_wrap(&config, wrap); 
    pwm_init(pin_slice, &config, true);

    int pwm_dma_channel = dma_claim_unused_channel(true);
    int reset_dma_channel = dma_claim_unused_channel(true);
    dma_channel_config pwm_channel = dma_channel_get_default_config(pwm_dma_channel);
    dma_channel_config reset_channel = dma_channel_get_default_config(reset_dma_channel);

    channel_config_set_transfer_data_size(&pwm_channel, DMA_SIZE_32);
    channel_config_set_read_increment(&pwm_channel, true);
    channel_config_set_write_increment(&pwm_channel, false);
    channel_config_set_dreq(&pwm_channel, DREQ_PWM_WRAP0 + pin_slice);
    channel_config_set_chain_to(&pwm_channel, reset_dma_channel);

    channel_config_set_transfer_data_size(&reset_channel, DMA_SIZE_32);
    channel_config_set_read_increment(&reset_channel, false);
    channel_config_set_write_increment(&reset_channel, false);
    channel_config_set_chain_to(&reset_channel, pwm_dma_channel);

    dma_channel_configure(
        pwm_dma_channel,
        &pwm_channel,
        &pwm_hw->slice[pin_slice].cc,            // write address
        sine_table,      // read address
        sine_table_length,  // number of transfers to do
        false               // don't start immediately
    );
    dma_channel_configure(
        reset_dma_channel,
        &reset_channel,
        &dma_hw->ch[pwm_dma_channel].read_addr,            // write address
        &sine_table_pointer,      // read address
        1,  // number of transfers to do
        false               // don't start immediately
    );

    dma_start_channel_mask(1u << pwm_dma_channel);
}
