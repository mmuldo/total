// pins to switch between for adc round robin sampling
#define ADC_PIN_MASK 0b0011

// starting pin for adc round robin sampling
#define ADC_FIRST_PIN 1

// second pin for adc round robin sampling
#define ADC_SECOND_PIN 0

// the first gpio pin where the adc pins are located
#define ADC_GPIO_PINS 26

// number of samples in one period of a sine wave
#define NUM_SAMPLES_PER_PERIOD 25

// number of periods to sample for
#define NUM_PERIODS 10

// total number of samples that will be collected
#define TOTAL_NUM_SAMPLES 2*NUM_PERIODS*(NUM_SAMPLES_PER_PERIOD)

// frequency at which dedicated adc clock is running at
#define ADC_CLOCK_FREQUENCY_HZ 48000000

void init_adc(float sine_frequency);
void init_adc_dma(int adc_dma_channel, uint8_t samples[]);
void sample_signals(int adc_dma_channel);