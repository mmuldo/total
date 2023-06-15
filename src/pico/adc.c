#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"   
#include "hardware/adc.h"
#include "hardware/dma.h"

#include "adc.h"

/// @brief initializes adc to read vin and vout and write to fifo
/// @param sine_frequency frequency of vin and vout
void init_adc(float sine_frequency) {
    adc_gpio_init(ADC_GPIO_PINS + ADC_FIRST_PIN);
    adc_gpio_init(ADC_GPIO_PINS + ADC_SECOND_PIN);
    adc_init();
    adc_set_round_robin(ADC_PIN_MASK);
    // read vin first
    adc_select_input(ADC_FIRST_PIN);

    adc_fifo_setup(
        true,   // write adc readings to FIFO
        true,   // enable dma data request (DREQ)
        1,      // DREQ asserted when at least 1 sample is present in the FIFO
        false,  // disable inclusion of error bit
        true    // FIFO shifts are one byte in size
    );

    // sample frequency must be twice as fast because we're sampling 2 signals
    float sample_frequency = 2*NUM_SAMPLES_PER_PERIOD * sine_frequency;
    adc_set_clkdiv(ADC_CLOCK_FREQUENCY_HZ / sample_frequency);
}

/// @brief initializes a dma channel for collecting adc samples
/// @param adc_dma_channel dma channel for this purpose
/// @param samples buffer that dma will write adc samples to
void init_adc_dma(int adc_dma_channel, uint8_t samples[]) {
    dma_channel_config channel = dma_channel_get_default_config(adc_dma_channel);

    // adc samples are 8 bits
    channel_config_set_transfer_data_size(&channel, DMA_SIZE_8);
    // reading from same place
    channel_config_set_read_increment(&channel, false);
    // writing to a buffer
    channel_config_set_write_increment(&channel, true);
    // pace with adc
    channel_config_set_dreq(&channel, DREQ_ADC);

    dma_channel_configure(
        adc_dma_channel,
        &channel,
        // write address: samples buffer
        samples,
        // read address: adc fifo
        &adc_hw->fifo,
        // number of transfers: length of samples buffer
        TOTAL_NUM_SAMPLES,
        // don't start immediately
        false
    );
}

/// @brief perform a sampling of vin and vout
/// @param adc_dma_channel dma channel for moving data from adc to a buffer
void sample_signals(int adc_dma_channel) {
    dma_channel_start(adc_dma_channel);
    adc_run(true);

    dma_channel_wait_for_finish_blocking(adc_dma_channel);

    // reset adc
    adc_run(false);
    adc_fifo_drain();
    adc_select_input(ADC_FIRST_PIN);
}