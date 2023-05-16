#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "pico/stdlib.h"   
#include "hardware/pwm.h"  
#include "hardware/gpio.h"
#include "hardware/dma.h"

#include "pwm.h"

void init_pwm(int pin_slice, int sine_table_length) {
    pwm_config config = pwm_get_default_config();
    // it is convenient to set wrap = 2*length
    int wrap = 2*sine_table_length;
    pwm_config_set_clkdiv(&config, 1.0); 
    pwm_config_set_wrap(&config, wrap); 
    pwm_init(pin_slice, &config, true);
}

void init_pwm_dma(int pwm_dma_channel, int reset_dma_channel, int pwm_pin_slice, uint32_t * sine_table, uint32_t ** sine_table_pointer, int sine_table_length) {
    dma_channel_config pwm_dma_channel_config = dma_channel_get_default_config(pwm_dma_channel);
    dma_channel_config reset_dma_channel_config = dma_channel_get_default_config(reset_dma_channel);

    channel_config_set_transfer_data_size(&pwm_dma_channel_config, DMA_SIZE_32);
    // reading entries from a sine_table
    channel_config_set_read_increment(&pwm_dma_channel_config, true);
    // always writes to the same place
    channel_config_set_write_increment(&pwm_dma_channel_config, false);
    // pace with the pwm signal
    channel_config_set_dreq(&pwm_dma_channel_config, DREQ_PWM_WRAP0 + pwm_pin_slice);
    // when done, start the reset dma channel
    channel_config_set_chain_to(&pwm_dma_channel_config, reset_dma_channel);

    channel_config_set_transfer_data_size(&reset_dma_channel_config, DMA_SIZE_32);
    // only performing one read
    channel_config_set_read_increment(&reset_dma_channel_config, false);
    // only performing one write
    channel_config_set_write_increment(&reset_dma_channel_config, false);
    // when done, start the pwm dma channel
    channel_config_set_chain_to(&reset_dma_channel_config, pwm_dma_channel);

    dma_channel_configure(
        pwm_dma_channel,
        &pwm_dma_channel_config,
        // write address: pwm counter compare register
        &pwm_hw->slice[pwm_pin_slice].cc,
        // read address: the provided sine_table
        sine_table,
        // number of transfers: the length of the sine_table
        sine_table_length,
        // don't start immediately
        false
    );
    dma_channel_configure(
        reset_dma_channel,
        &reset_dma_channel_config,
        // write address: the pwm dma channel's read address
        &dma_hw->ch[pwm_dma_channel].read_addr,
        // read address: the pointer to the pointer to the first entry in the given sine_table
        sine_table_pointer,
        // number of transfers: one (just resetting the pwm dma channel)
        1,
        // don't start immediately
        false
    );
}
