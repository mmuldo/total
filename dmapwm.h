// pin at which sine wave will be emitted
#define INPUT_SIGNAL_PIN 0

// frequency (in kHz) to set clock at
#define CLK_KHZ 250000
#define KHZ_TO_HZ 1000

// saturation voltage in volts
#define VDD 3.3

// for calculating sine
#define PI 3.1415926535

uint32_t * generate_sine_table(int length, double amplitude);
int highest_frequency_to_table_length(float frequency);
void configure_gpio_pwm_pin(int pin, int sine_table_length);
void configure_and_start_pwm_dma_channels(int pwm_dma_channel, int reset_dma_channel, int gpio_pin, volatile uint32_t * table, int table_length);
void generate_sine_wave(int gpio_pin, float frequency, int pwm_dma_channel, int reset_dma_channel);
void change_sine_wave(int gpio_pin, float frequency, int pwm_dma_channel, int reset_dma_channel);