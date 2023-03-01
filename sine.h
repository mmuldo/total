// pin at which sine wave will be emitted
#define INPUT_SIGNAL_PIN 0

// amount to bit shift sample position such that the sample rate is reduced
// by a factor. explicitly, decreases the sample rate by a factor
// of 2^SAMPLE_RATE_FACTOR_SHIFT. e.g. to decrease the sample rate by a factor of 8,
// set this to 3.
#define SAMPLE_RATE_FACTOR_SHIFT 2

// saturation voltage in volts
#define VDD 3.3

// amplitude of input sine wave in volts
#define AMPLITUDE 0.1

// frequency (in kHz) to set clock at
#define CLK_KHZ 200000
#define KHZ_TO_HZ 1000.0

// for calculating sine
#define PI 3.14159265

void pwm_interrupt_handler();
void generate_sine_wave(uint32_t sine_frequency);
