#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"   
#include "pico/multicore.h"
#include "hardware/irq.h"  
#include "hardware/pwm.h"  
#include "hardware/sync.h" 
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/watchdog.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"

#define I2C_CHANNEL i2c0
#define SDA_GPIO_PIN 16
#define SCL_GPIO_PIN 17
#define I2C_BAUDRATE_HZ 400000

#define MS5803_14BA_ADDRESS             0x76
#define MS5803_14BA_RESET_CMD           0x1E
#define MS5803_14BA_PROM_READ_BASE_CMD  0xA0
#define MS5803_14BA_D1_4096             0x40
#define MS5803_14BA_D2_4096             0x58
#define MS5803_14BA_ADC_READ            0x00
 
// pins to switch between for adc round robin sampling
#define ADC_PIN_MASK 0b0011

// starting pin for adc round robin sampling
#define ADC_FIRST_PIN 1

// second pin for adc round robin sampling
#define ADC_SECOND_PIN 0

// the first gpio pin where the adc pins are located
#define ADC_GPIO_PINS 26

// adc voltage at max reading
#define ADC_VREF 3.3

// max adc reading
#define ADC_RANGE (1 << 8)

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

// frequency at which dedicated adc clock is running at
#define ADC_CLOCK_FREQUENCY_HZ 48000000

// pin at which sine wave will be emitted
#define INPUT_SIGNAL_PIN 0


// amount to bit shift sample position such that the sample rate is reduced
// by a factor. explicitly, decreases the sample rate by a factor
// of 2^SAMPLE_RATE_FACTOR_SHIFT. e.g. to decrease the sample rate by a factor of 8,
// set this to 3.
#define SAMPLE_RATE_FACTOR_SHIFT 2

// frequency (in kHz) to set clock at
#define CLK_KHZ 250000

// number of clocks before pwm wraps and sets the interrupt
#define WRAP 100


// arbitrary length of time to pause while waiting for things to initialize
#define PAUSE_MS 10

// amount of time to wait for signals to reach steady-state
#define SIGNAL_STEADY_WAIT_TIME_MS 1000

// amount of time watchdog should wait before rebooting system
#define WATCHDOG_SYSTEM_REBOOT_WAIT_TIME_MS 1000


// unit conversions
#define S_TO_US 1000000
#define KHZ_TO_HZ 1000.0

#define SERIAL_IO_WAIT_TIME_US 5*60*1000000
#define MAX_INT_LENGTH 6


// buffer in which to collect samples
uint8_t samples[TOTAL_NUM_SAMPLES];

// the dma channel we will use
int dma_channel = 0;

/* 
 * sine wave sample table
 */
#include "sine.h"

// initialize sine table index
int sine_position = 0;

/*
 * reads from serial i/o until '\n' is encounted, then returns
 * the entire string (besides the '\n') that was read
 *
 * Returns
 * -------
 *  const char*
 *      string that was read
 */
uint32_t read_int_from_serial() {
    int16_t character;
    char str[MAX_INT_LENGTH];
    int index = 0;

    character = getchar_timeout_us(SERIAL_IO_WAIT_TIME_US);
    while(character != '\n') {
        if (character != PICO_ERROR_TIMEOUT) {
            str[index++] = character;
        }
        character = getchar_timeout_us(SERIAL_IO_WAIT_TIME_US);
    }

    return atoi(str);
}

/*
 * PWM Interrupt Handler which outputs PWM level and advances the 
 * current sample. 
 * 
 * We repeat the same value for 2^SAMPLE_RATE_FACTOR_SHIFT cycles. This means 
 * the sample rate is adjusted by factor of 2^SAMPLE_RATE_FACTOR_SHIFT.
 */
void pwm_interrupt_handler() {
    // clear interrupt flag
    pwm_clear_irq(pwm_gpio_to_slice_num(INPUT_SIGNAL_PIN));    

    if (sine_position < (SINE_TABLE_LENGTH<<SAMPLE_RATE_FACTOR_SHIFT) - 1) { 
        // set pwm level 
        pwm_set_gpio_level(INPUT_SIGNAL_PIN, SINE_TABLE[sine_position>>SAMPLE_RATE_FACTOR_SHIFT]);  
        sine_position++;
    } else {
        // reset to start
        sine_position = 0;
    }
}

/*
 * code that core 1 will run
 *
 * generates a sine wave using pwm
 */
void core1_entry() {
    // get frequency from core 0
    uint32_t sine_freq = multicore_fifo_pop_blocking();

    // initialize pwm pin
    gpio_set_function(INPUT_SIGNAL_PIN, GPIO_FUNC_PWM);
    
    int pin_slice = pwm_gpio_to_slice_num(INPUT_SIGNAL_PIN);

    // Setup PWM interrupt to fire when PWM cycle is complete
    pwm_clear_irq(pin_slice);
    pwm_set_irq_enabled(pin_slice, true);
    // use interrupt handler function defined above
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_interrupt_handler); 
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // initialize pwm config: clock division and wrap
    pwm_config config = pwm_get_default_config();
    float clkdiv = CLK_KHZ * KHZ_TO_HZ / (WRAP * (SINE_TABLE_LENGTH<<SAMPLE_RATE_FACTOR_SHIFT) * sine_freq);
    pwm_config_set_clkdiv(&config, clkdiv); 
    pwm_config_set_wrap(&config, WRAP); 
    pwm_init(pin_slice, &config, true);

    // set initial pwm level
    pwm_set_gpio_level(INPUT_SIGNAL_PIN, 0);

    while(1) {
        __wfi(); // Wait for Interrupt
    }
}

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

void ms5803_14ba_reset() {
    uint8_t buffer[2];
    uint8_t command = MS5803_14BA_RESET_CMD;
    uint8_t read_command = MS5803_14BA_PROM_READ_BASE_CMD;
    int result = PICO_ERROR_GENERIC;

    i2c_write_blocking(I2C_CHANNEL, MS5803_14BA_ADDRESS, &command, 1, false);

    // wait some time to wait for everything to settle
    sleep_us(10000);

}

uint16_t ms5803_14ba_get_coefficient(uint8_t coefficient) {
    uint8_t buffer[2];
    uint8_t command = MS5803_14BA_PROM_READ_BASE_CMD + (coefficient<<1);
    i2c_write_blocking(I2C_CHANNEL, MS5803_14BA_ADDRESS, &command, 1, false);
    i2c_read_blocking(I2C_CHANNEL, MS5803_14BA_ADDRESS, buffer, 2, false);

    uint8_t high_byte = buffer[0];
    uint8_t low_byte = buffer[1];

    return (high_byte<<8)+low_byte;
}


uint32_t ms5803_14ba_get_d1(void) {
    uint8_t buffer[3];
    uint8_t command = MS5803_14BA_D1_4096;
    uint8_t command_adc = MS5803_14BA_ADC_READ;
    i2c_write_blocking(I2C_CHANNEL, MS5803_14BA_ADDRESS, &command, 1, false);
    sleep_ms(10);
    i2c_write_blocking(I2C_CHANNEL, MS5803_14BA_ADDRESS, &command_adc, 1, false);

    i2c_read_blocking(I2C_CHANNEL, MS5803_14BA_ADDRESS, buffer, 3, false);

    uint32_t value = (buffer[0] << 16) | (buffer[1] << 8) | (buffer[2]);

    return value;
}


uint32_t ms5803_14ba_get_d2(void) {
    uint8_t buffer[3];
    uint8_t command = MS5803_14BA_D2_4096;
    uint8_t command_adc = MS5803_14BA_ADC_READ;
    i2c_write_blocking(I2C_CHANNEL, MS5803_14BA_ADDRESS, &command, 1, false);
    sleep_ms(10);
    i2c_write_blocking(I2C_CHANNEL, MS5803_14BA_ADDRESS, &command_adc, 1, false);

    i2c_read_blocking(I2C_CHANNEL, MS5803_14BA_ADDRESS, buffer, 3, false);

    uint32_t value = (buffer[0] << 16) | (buffer[1] << 8) | (buffer[2]);

    return value;
}

unsigned char crc4(unsigned int n_prom[]) {
    int cnt; // simple counter
    unsigned int n_rem; // crc reminder
    unsigned int crc_read; // original value of the crc
    unsigned char n_bit;
    n_rem = 0x00;
    crc_read=n_prom[7]; //save read CRC
    n_prom[7]=(0xFF00 & (n_prom[7])); //CRC byte is replaced by 0
    for (cnt = 0; cnt < 16; cnt++) // operation is performed on bytes
    { // choose LSB or MSB
        if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
        else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);
        for (n_bit = 8; n_bit > 0; n_bit--)
        {
            if (n_rem & (0x8000))
            {
                n_rem = (n_rem << 1) ^ 0x3000;
            }
            else
            {
                n_rem = (n_rem << 1);
            }
        }
    }
    n_rem= (0x000F & (n_rem >> 12)); // // final 4-bit reminder is CRC code
    n_prom[7]=crc_read; // restore the crc_read to its original place
    return (n_rem ^ 0x00);
} 

void init_i2c() {
    i2c_init(I2C_CHANNEL, I2C_BAUDRATE_HZ);
    gpio_set_function(SDA_GPIO_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_GPIO_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_GPIO_PIN);
    gpio_pull_up(SCL_GPIO_PIN);

    // wait some time to let everything settle
    sleep_us(1000);
}


void get_temperature_and_pressure(double* temperature_pointer, double* pressure_pointer) {
    double dT, OFF,SENS;
    uint32_t D1, D2;
    unsigned int c[8];

    ms5803_14ba_reset();

    for (int i = 0; i < sizeof(c)/sizeof(*c); i++) {
        c[i] = ms5803_14ba_get_coefficient(i);
    }

    D1 = ms5803_14ba_get_d1();
    D2 = ms5803_14ba_get_d2();

    dT = D2-c[5]*pow(2,8);
    OFF = c[2]*pow(2,16)+dT*c[4]/pow(2,7);
    SENS = c[1]*pow(2,15)+dT*c[3]/pow(2,8);

    *temperature_pointer = (2000+(dT*c[6])/pow(2,23))/100;
    *pressure_pointer = (((D1*SENS)/pow(2,21)-OFF)/pow(2,15))/10; 

    sleep_us(5000);
}

int main(void) {
    stdio_init_all();
    init_i2c();

    // set system clock
    set_sys_clock_khz(CLK_KHZ, true); 

    // for reading temperature and pressure
    double temperature, pressure;

    while(true) {
        // get sine frequency from serial input
        uint32_t sine_frequency = read_int_from_serial();

        // read temperature and pressure
        get_temperature_and_pressure(&temperature, &pressure);

        // restart core 1 send frequency to core 1
        multicore_reset_core1();
        multicore_launch_core1(core1_entry);
        multicore_fifo_push_blocking(sine_frequency);

        // wait for signals to achieve steady state before taking readings
        sleep_ms(SIGNAL_STEADY_WAIT_TIME_MS);

        init_adc_and_dma(sine_frequency);
        sample_signals();
        printf("%.3f\n", temperature);
        // data tends to corrupt if we send to serial i/o too fast,
        // so this is just to make sure it all gets printed cleanly
        sleep_ms(1);
        printf("%.3f\n", pressure);
        sleep_ms(1);
    }
}
