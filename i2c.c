#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"

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

int main() {
    stdio_init_all();
    while (true) {
        getchar_timeout_us(1000000*500);

        init_i2c();

        double T, P;
        get_temperature_and_pressure(&T, &P);

        printf("T = %.2f\n", T);
        printf("P = %.2f\n", P); 
    }
}


