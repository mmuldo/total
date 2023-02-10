#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"   
#include "pico/multicore.h"
#include "hardware/irq.h"  
#include "hardware/pwm.h"  
#include "hardware/sync.h" 
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/watchdog.h"
#include "hardware/dma.h"

#define ADC_CHANNEL 0
#define ADC_PIN 26
#define ADC_PIN_MASK 0b0011
#define ADC_FIRST_PIN 1
#define ADC_SECOND_PIN 0
#define ADC_GPIO_PINS 26

// max adc reading
#define ADC_RANGE (1 << 8)
#define ADC_VREF 3.3

// multiply to convert adc reading to voltage
#define ADC_CONVERT (ADC_VREF / (ADC_RANGE - 1))

// number of samples in one period of a sine wave
#define NUM_SAMPLES_PER_PERIOD 25

// number of periods to sample for
#define NUM_PERIODS 10

// total number of samples that will be collected
// NOTE: it appears that the maximum number of samples that can
// be fed to the serial i/o at one time is 594, so if you don't
// plan on reading the samples from serial i/o in chunks, then
// don't let this constant exceed 594
#define TOTAL_NUM_SAMPLES 2*NUM_PERIODS*(NUM_SAMPLES_PER_PERIOD)

//#define SAMPLE_FREQUENCY_HZ 50000
#define ADC_CLOCK_FREQUENCY_HZ 48000000

#define SERIAL_IO_WAIT_TIME_US 60*1000000

// buffer in which to collect samples
uint8_t samples[TOTAL_NUM_SAMPLES];
// the dma channel we will use
int dma_channel = 0;
int sine_frequency = 770;


/*
 * initializes the adc pins and the dma
 *
 * Parameters
 * ----------
 *  sine_frequency : uint32_t
 *      frequency of sine waves that are being sampled;
 *      determines the sampling frequency
 */
void init_adc_and_dma(uint32_t sine_frequency) {
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
    dma_channel_config channel = dma_channel_get_default_config(dma_channel);

    channel_config_set_transfer_data_size(&channel, DMA_SIZE_8);
    channel_config_set_read_increment(&channel, false);
    channel_config_set_write_increment(&channel, true);
    channel_config_set_dreq(&channel, DREQ_ADC);

    dma_channel_configure(
        dma_channel,
        &channel,
        samples,            // write address
        &adc_hw->fifo,      // read address
        TOTAL_NUM_SAMPLES,  // number of transfers to do
        false               // don't start immediately
    );
}

/*
 * samples the sine waves using the adc
 */
void sample_signals() {
    dma_channel_start(dma_channel);
    adc_run(true);
    dma_channel_wait_for_finish_blocking(dma_channel);

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

int main() {
    stdio_init_all();
    getchar_timeout_us(SERIAL_IO_WAIT_TIME_US);

    init_adc_and_dma(sine_frequency);
    sample_signals();
}

