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
#define CLK_KHZ 100000
#define KHZ_TO_HZ 1000
#define WRAP 2*SINE_TABLE_LENGTH

uint8_t * sine_table_address_pointer = &SINE_TABLE[0];

int main(void) {
    set_sys_clock_khz(CLK_KHZ, true); 

    int sine_frequency = 10000;

    // initialize pwm pin
    gpio_set_function(INPUT_SIGNAL_PIN, GPIO_FUNC_PWM);
    
    int pin_slice = pwm_gpio_to_slice_num(INPUT_SIGNAL_PIN);

    // initialize pwm config: clock division and wrap
    pwm_config config = pwm_get_default_config();
    float clkdiv = CLK_KHZ * KHZ_TO_HZ / (WRAP * SINE_TABLE_LENGTH * sine_frequency);
    pwm_config_set_clkdiv(&config, clkdiv); 
    pwm_config_set_wrap(&config, WRAP); 
    pwm_init(pin_slice, &config, true);

    // set initial pwm level
    pwm_set_gpio_level(INPUT_SIGNAL_PIN, 0);


    dma_channel_config pwm_channel = dma_channel_get_default_config(PWM_DMA_CHANNEL);
    dma_channel_config reset_channel = dma_channel_get_default_config(RESET_DMA_CHANNEL);

    channel_config_set_transfer_data_size(&pwm_channel, DMA_SIZE_8);
    channel_config_set_read_increment(&pwm_channel, true);
    channel_config_set_write_increment(&pwm_channel, false);
    channel_config_set_dreq(&pwm_channel, DREQ_PWM_WRAP0 + pin_slice);

    channel_config_set_transfer_data_size(&reset_channel, DMA_SIZE_32);
    channel_config_set_read_increment(&reset_channel, false);
    channel_config_set_write_increment(&reset_channel, false);
    channel_config_set_chain_to(&reset_channel, PWM_DMA_CHANNEL);

    dma_channel_configure(
        PWM_DMA_CHANNEL,
        &pwm_channel,
        &pwm_hw->slice[pin_slice].cc,            // write address
        SINE_TABLE,      // read address
        SINE_TABLE_LENGTH,  // number of transfers to do
        false               // don't start immediately
    );
    dma_channel_configure(
        RESET_DMA_CHANNEL,
        &reset_channel,
        &dma_hw->ch[PWM_DMA_CHANNEL].write_addr,            // write address
        &sine_table_address_pointer,      // read address
        1,  // number of transfers to do
        false               // don't start immediately
    );

    dma_channel_start(PWM_DMA_CHANNEL);
    dma_channel_start(RESET_DMA_CHANNEL);
}
