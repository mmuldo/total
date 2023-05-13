#define PI 3.1415926535

typedef struct {
    double amplitude;
    double frequency;
    double phase;
    double offset;
} sine;

typedef struct {
    double magnitude;
    double phase;
} complex;

sine characterize(double sine_period[], double frequency, int num_samples);
complex impedence(sine vin, sine vout, double Rf);