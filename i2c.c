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

// cs bit could be wrong
#define MS5803_14BA_ADDRESS             0x76
// this might need to be shifted righ 1 bit
#define MS5803_14BA_RESET_CMD           0x1E
#define MS5803_14BA_PROM_READ_BASE_CMD  0xA0

void ms5803_14ba_reset() {
    uint8_t buffer[2];
    uint8_t command = MS5803_14BA_RESET_CMD;
    uint8_t read_command = MS5803_14BA_PROM_READ_BASE_CMD;
    int result = PICO_ERROR_GENERIC;

    i2c_write_blocking(I2C_CHANNEL, MS5803_14BA_ADDRESS, &command, 1, false);
    printf("sending reset\n");
/*
    while (result != 1) {
        printf("attempting to read\n");
        result = i2c_write_blocking(I2C_CHANNEL, MS5803_14BA_ADDRESS, &read_command, 1, true);
        result = i2c_write_blocking(I2C_CHANNEL, MS5803_14BA_ADDRESS, &read_command, 1, false);
    }

    printf("final read result: %d\n", result);
*/
    printf("read success\n");
    
}

uint16_t ms5803_14ba_get_coefficient(uint8_t coefficient) {
    uint8_t buffer[2];
    uint8_t command = MS5803_14BA_PROM_READ_BASE_CMD + (coefficient<<1);
    i2c_write_blocking(I2C_CHANNEL, MS5803_14BA_ADDRESS, &command, 1, false);
    i2c_read_blocking(I2C_CHANNEL, MS5803_14BA_ADDRESS, buffer, 2, false);

    uint8_t high_byte = buffer[0];
    uint8_t low_byte = buffer[1];

    printf("%d\n", high_byte);
    printf("%d\n", low_byte);

    return (high_byte<<8)+low_byte;
}

int main() {

        
    stdio_init_all();
    while (true) {
        
    getchar_timeout_us(1000000*500);

    i2c_init(I2C_CHANNEL, I2C_BAUDRATE_HZ);
    gpio_set_function(SDA_GPIO_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_GPIO_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_GPIO_PIN);
    gpio_pull_up(SCL_GPIO_PIN);

    sleep_us(1000);
    ms5803_14ba_reset();
        sleep_us(10000);

    uint16_t coef = ms5803_14ba_get_coefficient(5);

    printf("coef: %d\n", coef);
}
}

