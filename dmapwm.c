#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"   
#include "pico/multicore.h"
#include "hardware/pwm.h"  
#include "hardware/gpio.h"
#include "hardware/dma.h"

#include "sinetable.h"

#define PWM_DMA_CHANNEL 1
#define RESET_DMA_CHANNEL 2

#define INPUT_SIGNAL_PIN 0
#define CLK_KHZ 250000
#define KHZ_TO_HZ 1000
#define WRAP 2*SINE_TABLE_LENGTH
#define PI 3.1415926535
#define VDD 3.3

uint32_t * sine_table_address_pointer = &SINE_TABLE[0];

uint32_t * generate_sine_table(int length, double amplitude) {
    uint32_t * sine_table = malloc(length*sizeof(uint32_t));
    for (int i = 0; i < length; i++) {
        uint32_t duty_cycle = (uint32_t) round(length*((2*amplitude/VDD)*sin(2 * PI * i / length) + 1));
        sine_table[i] = duty_cycle;
    }

    return sine_table;
}

int main(void) {
    float sine_frequency = 3745;
    int sine_table_length = (int) round(sqrt(CLK_KHZ * KHZ_TO_HZ / (2 * sine_frequency)));
    uint32_t * sine_table = generate_sine_table(sine_table_length, 1);

    set_sys_clock_khz(CLK_KHZ, true);

    // initialize pwm pin
    gpio_set_function(INPUT_SIGNAL_PIN, GPIO_FUNC_PWM);
    
    int pin_slice = pwm_gpio_to_slice_num(INPUT_SIGNAL_PIN);

    // initialize pwm config: clock division and wrap
    pwm_config config = pwm_get_default_config();
    float clkdiv = CLK_KHZ * KHZ_TO_HZ / (WRAP * SINE_TABLE_LENGTH * sine_frequency);
    pwm_config_set_clkdiv(&config, 1.0); 
    pwm_config_set_wrap(&config, 2*sine_table_length); 
    pwm_init(pin_slice, &config, true);

    dma_channel_config pwm_channel = dma_channel_get_default_config(PWM_DMA_CHANNEL);
    dma_channel_config reset_channel = dma_channel_get_default_config(RESET_DMA_CHANNEL);

    channel_config_set_transfer_data_size(&pwm_channel, DMA_SIZE_32);
    channel_config_set_read_increment(&pwm_channel, true);
    channel_config_set_write_increment(&pwm_channel, false);
    channel_config_set_dreq(&pwm_channel, DREQ_PWM_WRAP0 + pin_slice);
    channel_config_set_chain_to(&pwm_channel, RESET_DMA_CHANNEL);

    channel_config_set_transfer_data_size(&reset_channel, DMA_SIZE_32);
    channel_config_set_read_increment(&reset_channel, false);
    channel_config_set_write_increment(&reset_channel, false);
    channel_config_set_chain_to(&reset_channel, PWM_DMA_CHANNEL);

    dma_channel_configure(
        PWM_DMA_CHANNEL,
        &pwm_channel,
        &pwm_hw->slice[pin_slice].cc,            // write address
        sine_table,      // read address
        sine_table_length,  // number of transfers to do
        false               // don't start immediately
    );
    dma_channel_configure(
        RESET_DMA_CHANNEL,
        &reset_channel,
        &dma_hw->ch[PWM_DMA_CHANNEL].read_addr,            // write address
        &sine_table,      // read address
        1,  // number of transfers to do
        false               // don't start immediately
    );

    dma_start_channel_mask(1u << PWM_DMA_CHANNEL);
}
