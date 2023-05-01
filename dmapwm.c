#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"   
#include "pico/multicore.h"
#include "hardware/pwm.h"  
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "dmapwm.h"

// global pointer to sine_table (needs to be global so that dma channels have access)
uint32_t * sine_table;
// sine table length also global just for consistency
int sine_table_length;


/// @brief creates an table whose entries are the pwm counter compare level corresponding to the appropriate sine function value
/// @param length the length of the table
/// @param amplitude desired amplitude of the sine wave
/// @return the sine table
uint32_t * generate_sine_table(int length, double amplitude) {
    uint32_t * sine_table = malloc(length*sizeof(uint32_t));

    for (int i = 0; i < length; i++) {
        // "duty cycle" is the same thing as the pwm counter compare level in this case
        uint32_t duty_cycle = (uint32_t) round(length*((2*amplitude/VDD)*sin(2 * PI * i / length) + 1));
        sine_table[i] = duty_cycle;
    }

    return sine_table;
}

/// @brief calculates the length of the sine table that produces a sine wave of the given frequency when the pwm clkdiv is set to 1.0
/// @param frequency the sine wave frequency
/// @return sine table length
int highest_frequency_to_table_length(float frequency) {
    // the following comes from the fact that
    //      f_sine = f_clk / (wrap * length * clkdiv)
    // where we assume wrap = 2*length and clkdiv = 1
    return (int) round(sqrt(CLK_KHZ * KHZ_TO_HZ / (2 * frequency)));
}

/// @brief configures the given gpio pin for pwm
/// @param pin the pin to emit a pwm wave
/// @param sine_table_length length of the sine table that will be fed to the pwm counter compare level (used for calculating the wrap)
void configure_gpio_pwm_pin(int pin, int sine_table_length) {
    int pwm_pin_slice = pwm_gpio_to_slice_num(pin);

    pwm_config config = pwm_get_default_config();
    // it is convenient to set wrap = 2*length
    int wrap = 2*sine_table_length;
    pwm_config_set_clkdiv(&config, 1.0); 
    pwm_config_set_wrap(&config, wrap); 
    pwm_init(pwm_pin_slice, &config, true);
}

/// @brief configures dma channels for setting the counter compare register of a pwm gpio pin from a table, then starts the channels
/// @param pwm_dma_channel dma channel responsible for reading the given table and writing to the pwm counter compare register
/// @param reset_dma_channel dma channel responsible for resetting the pwm dma channel back to the start of the table when it is done reading the table
/// @param gpio_pin the pin to emittin the pwm wave
/// @param table a table of pwm counter compare levels to read from
/// @param table_length the length of the table
void configure_and_start_pwm_dma_channels(int pwm_dma_channel, int reset_dma_channel, int gpio_pin, uint32_t * table, int table_length) {
    int pwm_pin_slice = pwm_gpio_to_slice_num(gpio_pin);

    dma_channel_config pwm_dma_channel_config = dma_channel_get_default_config(pwm_dma_channel);
    dma_channel_config reset_dma_channel_config = dma_channel_get_default_config(reset_dma_channel);

    channel_config_set_transfer_data_size(&pwm_dma_channel_config, DMA_SIZE_32);
    // reading entries from a table
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
        // read address: the provided table
        table,
        // number of transfers: the length of the table
        table_length,
        // don't start immediately
        false
    );
    dma_channel_configure(
        reset_dma_channel,
        &reset_dma_channel_config,
        // write address: the pwm dma channel's read address
        &dma_hw->ch[pwm_dma_channel].read_addr,
        // read address: the pointer to the pointer to the first entry in the given table
        &table,
        // number of transfers: one (just resetting the pwm dma channel)
        1,
        // don't start immediately
        false
    );

    // start the channels
    dma_start_channel_mask(1u << pwm_dma_channel);

    sleep_ms(1000);
}


/// @brief emit a sinusoidal wave via pwm on the specified gpio pin
/// @param gpio_pin the pin to emit the wave
/// @param frequency the frequency of the wave
void generate_sine_wave(int gpio_pin, float frequency, int pwm_dma_channel, int reset_dma_channel) {
    sine_table_length = highest_frequency_to_table_length(frequency);
    sine_table = generate_sine_table(sine_table_length, 1);

    configure_gpio_pwm_pin(gpio_pin, sine_table_length);

    //int pwm_dma_channel = dma_claim_unused_channel(true);
    //int reset_dma_channel = dma_claim_unused_channel(true);
    configure_and_start_pwm_dma_channels(pwm_dma_channel, reset_dma_channel, gpio_pin, sine_table, sine_table_length);
}

void change_sine_wave(int gpio_pin, float frequency, int pwm_dma_channel, int reset_dma_channel) {
    int pwm_pin_slice = pwm_gpio_to_slice_num(gpio_pin);
    // pause everything
    pwm_set_enabled(pwm_pin_slice, false);

    free(sine_table);
    sine_table_length = highest_frequency_to_table_length(frequency);
    sine_table = generate_sine_table(sine_table_length, 1);

    // reconfigure pwm stuff
    configure_gpio_pwm_pin(gpio_pin, sine_table_length);

    // reconfigure dma stuff
    dma_channel_set_read_addr(pwm_dma_channel, sine_table, false);
    dma_channel_set_trans_count(pwm_dma_channel, sine_table_length, false);
    dma_channel_set_read_addr(reset_dma_channel, &sine_table, false);

    // resume
    pwm_set_enabled(pwm_pin_slice, true);

    // wait for steady state
    sleep_ms(1000);
}

// float read_frequency_from_serial() {
//     float frequency = 0;
//     int16_t character;
    
//     character = getchar_timeout_us(5*60*1000000);
//     while(character != 'a') {
//         if (character != PICO_ERROR_TIMEOUT) {
//             frequency = 10*frequency + character - '0';
//         }
//         character = getchar_timeout_us(5*60*1000000);
//     }

//     return frequency;
// }

// int main(void) {
//     stdio_init_all();

//     float sine_frequency = read_frequency_from_serial();

//     generate_sine_wave(INPUT_SIGNAL_PIN, sine_frequency);
// }