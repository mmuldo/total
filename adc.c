#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"   
#include "hardware/adc.h"
#include "hardware/dma.h"

#include "adc.h"

/*
 * initializes the adc pins and the dma
 *
 * Parameters
 * ----------
 *  sine_frequency : uint32_t
 *      frequency of sine waves that are being sampled;
 *      determines the sampling frequency
 *  samples : uint8_t[]
 *      buffer in which samples will be stored
 */
void init_adc_and_dma(uint32_t sine_frequency, uint8_t samples[]) {
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


    // DMA STUFF
    dma_channel_config channel = dma_channel_get_default_config(DMA_CHANNEL);

    channel_config_set_transfer_data_size(&channel, DMA_SIZE_8);
    channel_config_set_read_increment(&channel, false);
    channel_config_set_write_increment(&channel, true);
    channel_config_set_dreq(&channel, DREQ_ADC);

    dma_channel_configure(
        DMA_CHANNEL,
        &channel,
        samples,            // write address
        &adc_hw->fifo,      // read address
        TOTAL_NUM_SAMPLES,  // number of transfers to do
        false               // don't start immediately
    );
}

/*
 * samples the sine waves using the adc
 *
 * Parameters
 * ----------
 *  samples : uint8_t[]
 *      buffer in which samples will be stored
 */
void sample_signals(uint8_t samples[]) {
    dma_channel_start(DMA_CHANNEL);
    adc_run(true);
    dma_channel_wait_for_finish_blocking(DMA_CHANNEL);

    // stop adc and drain fifo, just so everything is clean for next run
    adc_run(false);
    adc_fifo_drain();

    // send samples over serial i/o
    for (uint i = 0; i < TOTAL_NUM_SAMPLES; i++) {
        printf("%.3f\n", (float) ADC_CONVERT*samples[i]);
        // data tends to corrupt if we send to serial i/o too fast,
        // so this is just to make sure it all gets printed cleanly
        sleep_ms(1);
    }
}

