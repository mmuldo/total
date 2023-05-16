#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"   
#include "hardware/adc.h"
#include "hardware/dma.h"

#include "adc.h"

void init_adc(float sine_frequency) {
    // ADC STUFF
    adc_gpio_init(ADC_GPIO_PINS + ADC_FIRST_PIN);
    adc_gpio_init(ADC_GPIO_PINS + ADC_SECOND_PIN);
    adc_init();
    adc_set_round_robin(ADC_PIN_MASK);
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

void init_adc_dma(int adc_dma_channel, uint8_t samples[]) {
    // DMA STUFF
    dma_channel_config channel = dma_channel_get_default_config(adc_dma_channel);

    channel_config_set_transfer_data_size(&channel, DMA_SIZE_8);
    channel_config_set_read_increment(&channel, false);
    channel_config_set_write_increment(&channel, true);
    channel_config_set_dreq(&channel, DREQ_ADC);

    dma_channel_configure(
        adc_dma_channel,
        &channel,
        samples,            // write address
        &adc_hw->fifo,      // read address
        TOTAL_NUM_SAMPLES,  // number of transfers to do
        false               // don't start immediately
    );
}

void sample_signals(int adc_dma_channel) {
    dma_channel_start(adc_dma_channel);
    adc_run(true);

    dma_channel_wait_for_finish_blocking(adc_dma_channel);

    adc_run(false);
    adc_fifo_drain();
    adc_select_input(ADC_FIRST_PIN);
}