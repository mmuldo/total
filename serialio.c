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
 
#define SERIAL_IO_WAIT_TIME_US 5*60*1000000
#define MAX_INT_LENGTH 6

/*
 * reads from serial i/o until '\n' is encounted, then returns
 * the entire string (besides the '\n') that was read
 *
 * Returns
 * -------
 *  const char*
 *      string that was read
 */
int read_int_from_serial() {
    char character;
    char str[MAX_INT_LENGTH];
    int index = 0;

    character = getchar_timeout_us(SERIAL_IO_WAIT_TIME_US);
    while(character != '\n') {
        if (character == PICO_ERROR_TIMEOUT) {
            continue;
        }

        str[index++] = character;
        character = getchar_timeout_us(SERIAL_IO_WAIT_TIME_US);
    }

    return atoi(str);
}

int main(void) {
    stdio_init_all();

    int frequency = read_int_from_serial();
    printf("%d\n\n", frequency);
}
