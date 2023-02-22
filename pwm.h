// pin at which sine wave will be emitted
#define INPUT_SIGNAL_PIN 0

// amount to bit shift sample position such that the sample rate is reduced
// by a factor. explicitly, decreases the sample rate by a factor
// of 2^SAMPLE_RATE_FACTOR_SHIFT. e.g. to decrease the sample rate by a factor of 8,
// set this to 3.
#define SAMPLE_RATE_FACTOR_SHIFT 2

// frequency (in kHz) to set clock at
#define CLK_KHZ 250000
#define KHZ_TO_HZ 1000.0

// number of clocks before pwm wraps and sets the interrupt
#define WRAP 100

uint32_t read_int_from_serial();
void pwm_interrupt_handler();
void generate_sine_wave(uint32_t sine_frequency);
