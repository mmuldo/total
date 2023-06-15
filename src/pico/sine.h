// frequency (in kHz) to set clock at
#define CLK_KHZ 250000
#define KHZ_TO_HZ 1000.0

// saturation voltage in volts
#define VDD 3.3

// max adc reading
#define ADC_RANGE (1 << 8)

// multiply to convert adc reading to voltage
#define ADC_CONVERT (VDD / (ADC_RANGE - 1))

#define PI 3.1415926535

/// @brief sine(t) = amplitude[V]*sin(2*pi*frequency[Hz]*t[s] + phase[radians]) + offset[V]
typedef struct {
    double amplitude;
    double frequency;
    double phase;
    double offset;
} sine;

/// @brief Z = magnitude[Ohms]*exp(j*phase[radians])
typedef struct {
    double magnitude;
    double phase;
} complex;

uint32_t * generate_sine_table(int length, double amplitude);
int highest_frequency_to_table_length(float frequency);
void average_period(uint8_t samples[], uint8_t input_period[], uint8_t output_period[], int num_samples_per_period, int num_periods);
void samples_to_voltages(uint8_t samples[], double voltages[], int num_samples);
sine characterize(double sine_period[], double frequency, int num_samples);
complex impedence(sine vin, sine vout, double Rf);